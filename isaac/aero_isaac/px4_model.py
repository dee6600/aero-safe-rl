"""Every number that describes the vehicle and its flight controller, read
from the files the PX4 side itself flies with -- never retyped (milestones.md
M8b, "Design"):

* PX4's pinned x500 model, Tools/simulation/gz/models/x500_base/model.sdf:
  bodies, masses, inertias, collision boxes, rotor positions.
* This repo's simulation/models/x500_aero/model.sdf: the motor-model
  parameters of each rotor, and which rotor is which motor (M6 re-routes the
  motors through the rotor-fault plugin, so this file, not PX4's x500, is
  what Gazebo actually runs).
* PX4's airframe 4001_gz_x500 and parameter defaults from the pinned source:
  control-allocation geometry, motor output range, controller gains.

The PX4 tree is the sibling checkout pinned at v1.17.0 (CLAUDE.md §1);
override its location with PX4_AUTOPILOT_DIR.
"""
from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
PX4_DIR = Path(os.environ.get("PX4_AUTOPILOT_DIR", Path.home() / "projects" / "PX4-Autopilot"))
X500_BASE_SDF = PX4_DIR / "Tools/simulation/gz/models/x500_base/model.sdf"
X500_AERO_SDF = REPO / "simulation/models/x500_aero/model.sdf"
AIRFRAME = PX4_DIR / "ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500"
MC_DEFAULTS = PX4_DIR / "ROMFS/px4fmu_common/init.d/rc.mc_defaults"
CONTROL_ALLOCATOR_YAML = PX4_DIR / "src/modules/control_allocator/module.yaml"


def _parse_sdf(path: Path):
    """The <model> element. Comments are stripped first: Gazebo's parser
    accepts "--" inside a comment (x500_aero's header has some), Python's
    strict XML parser does not."""
    text = re.sub(r"<!--.*?-->", "", Path(path).read_text(), flags=re.S)
    return ET.fromstring(text).find("model")


def _floats(text: str) -> list[float]:
    return [float(x) for x in text.split()]


@dataclass(frozen=True)
class Collision:
    pose: tuple[float, ...]   # x y z roll pitch yaw, in the link frame
    size: tuple[float, ...]   # box x y z


@dataclass(frozen=True)
class Visual:
    pose: tuple[float, ...]
    mesh: str | None          # absolute path, or None for a primitive (decals are skipped)
    scale: tuple[float, ...] = (1.0, 1.0, 1.0)


@dataclass(frozen=True)
class Link:
    name: str
    pose: tuple[float, ...]   # in the model frame (forward-left-up)
    mass: float
    inertia: tuple[float, ...]  # ixx iyy izz ixy ixz iyz
    collisions: tuple[Collision, ...]
    visuals: tuple[Visual, ...]


@dataclass(frozen=True)
class Rotor:
    motor: int                # PX4 motor / Gazebo motorNumber, 0..3
    link: str
    position_flu: tuple[float, float, float]
    turning: float            # +1 counter-clockwise seen from above, -1 clockwise
    motor_constant: float
    moment_constant: float
    tau_up: float
    tau_down: float
    max_rot_velocity: float
    drag_coefficient: float
    rolling_moment_coefficient: float


@dataclass(frozen=True)
class X500:
    links: tuple[Link, ...]
    rotors: tuple[Rotor, ...]           # ordered by motor index
    esc_min: tuple[float, ...]          # rotor speed at command 0 (SIM_GZ_EC_MINn)
    esc_max: tuple[float, ...]          # rotor speed at command 1 (SIM_GZ_EC_MAXn)
    spawn_height: float = field(default=0.0)

    @property
    def mass(self) -> float:
        return sum(l.mass for l in self.links)


def _pose(el) -> tuple[float, ...]:
    p = el.find("pose")
    return tuple(_floats(p.text)) if p is not None and p.text else (0.0,) * 6


def _mesh_path(uri: str) -> str:
    if uri.startswith("model://"):
        return str(PX4_DIR / "Tools/simulation/gz/models" / uri[len("model://"):])
    return uri


@lru_cache(maxsize=1)
def load_x500() -> X500:
    base = _parse_sdf(X500_BASE_SDF)
    links = []
    for ln in base.findall("link"):
        inert = ln.find("inertial")
        i = inert.find("inertia")
        inertia = tuple(float(i.find(k).text) if i.find(k) is not None else 0.0  # SDF omits zeros
                        for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"))
        cols = tuple(Collision(_pose(c), tuple(_floats(c.find("geometry/box/size").text)))
                     for c in ln.findall("collision") if c.find("geometry/box") is not None)
        vis = []
        for v in ln.findall("visual"):
            mesh = v.find("geometry/mesh/uri")
            if mesh is None:
                continue  # flat texture decals: no shape, no physics
            scale = v.find("geometry/mesh/scale")
            vis.append(Visual(_pose(v), _mesh_path(mesh.text.strip()),
                              tuple(_floats(scale.text)) if scale is not None else (1.0, 1.0, 1.0)))
        links.append(Link(ln.get("name"), _pose(ln), float(inert.find("mass").text), inertia,
                          cols, tuple(vis)))
    by_name = {l.name: l for l in links}

    aero = _parse_sdf(X500_AERO_SDF)
    rotors = []
    for pl in aero.findall("plugin"):
        if "multicopter-motor-model" not in pl.get("filename", ""):
            continue
        g = lambda k: pl.find(k).text.strip()  # noqa: E731
        link = g("linkName")
        rotors.append(Rotor(
            motor=int(g("motorNumber")), link=link, position_flu=tuple(by_name[link].pose[:3]),
            turning=1.0 if g("turningDirection") == "ccw" else -1.0,
            motor_constant=float(g("motorConstant")), moment_constant=float(g("momentConstant")),
            tau_up=float(g("timeConstantUp")), tau_down=float(g("timeConstantDown")),
            max_rot_velocity=float(g("maxRotVelocity")),
            drag_coefficient=float(g("rotorDragCoefficient")),
            rolling_moment_coefficient=float(g("rollingMomentCoefficient"))))
    rotors.sort(key=lambda r: r.motor)
    esc_min = tuple(px4_param(f"SIM_GZ_EC_MIN{r.motor + 1}") for r in rotors)
    esc_max = tuple(px4_param(f"SIM_GZ_EC_MAX{r.motor + 1}") for r in rotors)
    return X500(tuple(links), tuple(rotors), esc_min, esc_max)


# ----------------------------------------------------------------- params

@lru_cache(maxsize=1)
def _set_defaults() -> dict[str, float]:
    """`param set-default NAME VALUE` lines: rc.mc_defaults, then the airframe
    (the airframe sources rc.mc_defaults first, so its values win)."""
    out: dict[str, float] = {}
    for path in (MC_DEFAULTS, AIRFRAME):
        for m in re.finditer(r"^\s*param set-default\s+(\w+)\s+(-?[\d.eE+-]+)", path.read_text(), re.M):
            out[m.group(1)] = float(m.group(2))
    return out


@lru_cache(maxsize=None)
def _source_default(name: str) -> float:
    pat = re.compile(r"PARAM_DEFINE_(?:FLOAT|INT32)\(\s*" + re.escape(name) + r"\s*,\s*(-?[\d.eE+-]+)f?\s*\)")
    for path in (PX4_DIR / "src").rglob("*.c"):
        text = path.read_text(errors="ignore")
        if name in text:
            m = pat.search(text)
            if m:
                return float(m.group(1))
    templated = re.sub(r"\d+", "${i}", name, count=1)
    text = CONTROL_ALLOCATOR_YAML.read_text()
    m = re.search(r"^\s*" + re.escape(templated) + r":\n(?:.*\n)*?\s*default:\s*(-?[\d.eE+-]+)", text, re.M)
    if m:
        return float(m.group(1))
    raise KeyError(f"no default found for PX4 parameter {name}")


def px4_param(name: str) -> float:
    """The value PX4 runs with on this vehicle: the airframe's set-default if
    any, else the source default."""
    defaults = _set_defaults()
    return defaults[name] if name in defaults else _source_default(name)


@dataclass(frozen=True)
class AllocationRotor:
    px: float
    py: float
    km: float
    ct: float


def allocation_geometry() -> tuple[AllocationRotor, ...]:
    """PX4's own belief about the rotors (CA_ROTORn_*, forward-right-down) --
    what its allocation uses. Not the same numbers as the model's rotor
    positions, exactly as on the PX4 side."""
    n = int(px4_param("CA_ROTOR_COUNT"))
    return tuple(AllocationRotor(px4_param(f"CA_ROTOR{i}_PX"), px4_param(f"CA_ROTOR{i}_PY"),
                                 px4_param(f"CA_ROTOR{i}_KM"), px4_param(f"CA_ROTOR{i}_CT"))
                 for i in range(n))


def mass_properties(x500: X500):
    """(centre of mass (3,), inertia about it (3, 3)) of all bodies together,
    in the model frame (forward-left-up). Link inertias are diagonal about
    each link's own origin, as in PX4's file."""
    import numpy as np
    m = np.array([l.mass for l in x500.links])
    p = np.array([l.pose[:3] for l in x500.links])
    com = (m[:, None] * p).sum(0) / m.sum()
    total = np.zeros((3, 3))
    for mass, pos, link in zip(m, p, x500.links):
        ixx, iyy, izz, ixy, ixz, iyz = link.inertia
        own = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        d = pos - com
        total += own + mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return com, total
