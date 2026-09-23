"""PX4's x500, imported into Isaac Lab (milestones.md M8b, "Design" -- the user
asked for the same drone model as Gazebo rather than a stand-in shape).

px4_model.load_x500() reads PX4's pinned model file; this module writes the
same bodies, masses, inertias, collision boxes and meshes as a robot
description in the format Isaac's importer reads, and Isaac Lab converts that
to its own asset on first use. Both files go to a cache outside the repo and
are rebuilt from PX4's file.

The five bodies are merged into one rigid body with exactly their combined
mass, centre of mass and inertia (see urdf_text). Rotor forces and torques are computed per rotor (rotor.py) and
applied to that body at the centre of mass, which is physically the same for
a rigid airframe. Gazebo applies each rotor's drag torque to the frame, not
to the propeller joint, and a merged body does the same. The only thing lost
is the propellers' visual spin.
"""
from __future__ import annotations

import hashlib
import re
import shutil
from pathlib import Path

from aero_isaac.px4_model import X500, X500_BASE_SDF, load_x500, mass_properties

CACHE = Path.home() / ".cache" / "aero_isaac" / "x500"


def rest_height(x500: X500 | None = None) -> float:
    """Height of the frame origin when the drone stands on its skids: the
    lowest point of any collision box, from PX4's own geometry (0.227 m for the
    x500; PX4 spawns it at 0.24 m and it settles). Altitude is measured from
    this point, as PX4's local frame starts where the vehicle rests."""
    import numpy as np
    x500 = x500 or load_x500()
    lowest = 0.0
    for link in x500.links:
        for c in link.collisions:
            x, y, z, r, p, yw = c.pose
            cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(yw), np.sin(yw)
            R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                          [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                          [-sp, cp * sr, cp * cr]])
            half = np.array(c.size) / 2.0
            corners = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)]) * half
            z_all = (corners @ R.T)[:, 2] + z + link.pose[2]
            lowest = min(lowest, float(z_all.min()))
    return -lowest


def _origin(pose) -> str:
    x, y, z, r, p, yw = pose
    return f'<origin xyz="{x} {y} {z}" rpy="{r} {p} {yw}"/>'


def urdf_text(x500: X500) -> str:
    """One body: the frame plus the four propellers, merged here rather than by
    the importer (its merging of bodies that carry mass is deprecated and fails
    in Isaac Sim 5.1). Mass, centre of mass and inertia are the exact
    combination of PX4's five bodies (px4_model.mass_properties); every
    collision box and mesh keeps its place, moved into the frame's
    coordinates (the propeller bodies are not rotated relative to the frame,
    so that is a translation)."""
    com, inertia = mass_properties(x500)
    parts = ['<?xml version="1.0"?>', '<robot name="x500">', f'<link name="{x500.links[0].name}">',
             f'<inertial><origin xyz="{com[0]} {com[1]} {com[2]}" rpy="0 0 0"/><mass value="{x500.mass}"/>'
             f'<inertia ixx="{inertia[0, 0]}" iyy="{inertia[1, 1]}" izz="{inertia[2, 2]}" '
             f'ixy="{inertia[0, 1]}" ixz="{inertia[0, 2]}" iyz="{inertia[1, 2]}"/></inertial>']
    for link in x500.links:
        if any(abs(a) > 1e-12 for a in link.pose[3:]):
            raise ValueError(f"{link.name} is rotated relative to the frame; merging assumes it is not")
        shift = link.pose[:3]
        for v in link.visuals:
            pose = (*[a + b for a, b in zip(v.pose[:3], shift)], *v.pose[3:])
            parts.append(f'<visual>{_origin(pose)}<geometry><mesh filename="{_importable_mesh(v.mesh)}" '
                         f'scale="{" ".join(map(str, v.scale))}"/></geometry></visual>')
        for c in link.collisions:
            pose = (*[a + b for a, b in zip(c.pose[:3], shift)], *c.pose[3:])
            parts.append(f'<collision>{_origin(pose)}<geometry><box size="{" ".join(map(str, c.size))}"/>'
                         f'</geometry></collision>')
    parts += ['</link>', '</robot>']
    return "\n".join(parts)


def _importable_mesh(src: str) -> str:
    """A copy of a PX4 mesh under a name Isaac's importer accepts. The
    importer names each visual after its file, and scene paths cannot contain
    '-' or start with a digit (PX4's are NXP-HGD-CF.dae, 1345_prop_ccw.stl, ...).
    PX4's texture folder is copied alongside at the same relative place, so
    meshes that reference textures still find them."""
    src_path = Path(src)
    mesh_dir = CACHE / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    textures = src_path.parent.parent / "materials"
    if textures.is_dir() and not (CACHE / "materials").exists():
        shutil.copytree(textures, CACHE / "materials")
        for png in (textures / "textures").glob("*.png"):   # .dae files look beside themselves
            shutil.copyfile(png, mesh_dir / png.name)
    dst = mesh_dir / ("mesh_" + re.sub(r"[^A-Za-z0-9_]", "_", src_path.stem) + src_path.suffix)
    if not dst.exists():
        shutil.copyfile(src_path, dst)
    return str(dst)


def write_urdf() -> Path:
    """The robot description, named by a digest of PX4's model file and this
    writer, so a change to either produces a fresh conversion."""
    text = urdf_text(load_x500())
    tag = hashlib.sha256((X500_BASE_SDF.read_text() + text).encode()).hexdigest()[:12]
    path = CACHE / f"x500_{tag}.urdf"
    if not path.exists():
        CACHE.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    return path


def rigid_object_cfg(prim_path: str = "/World/envs/env_.*/Robot"):
    """Isaac Lab config for the imported x500: one rigid body, so a
    RigidObject rather than a jointed articulation. Needs Isaac Sim running
    (imports isaaclab). Spawns standing on its skids (rest_height)."""
    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObjectCfg

    urdf = write_urdf()
    return RigidObjectCfg(
        prim_path=prim_path,
        spawn=sim_utils.UrdfFileCfg(
            asset_path=str(urdf), usd_dir=str(CACHE / "usd"), usd_file_name=urdf.stem + ".usd",
            fix_base=False, merge_fixed_joints=False, joint_drive=None, make_instanceable=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False, max_depenetration_velocity=10.0, enable_gyroscopic_forces=True),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, rest_height())),
    )
