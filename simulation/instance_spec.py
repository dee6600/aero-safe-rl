"""Identity of one PX4 SITL worker — the single source of truth.

A *worker* in this project is one fully isolated simulation stack: its own
Gazebo server, its own uXRCE-DDS agent, its own PX4 instance. Eight coupled
resources have to agree for that stack to work, and getting any one of them
wrong produces a worker that starts cleanly and then silently does nothing —
no error, no log line, no ack.

Everything that needs to know about a worker derives it from here, once.
Nothing else in this repository may recompute ``8888 + instance`` or assemble a
``/fmu/...`` topic string by hand. See ``CLAUDE.md`` §2 and
``docs/parallelism.md``.

Two facts about PX4 v1.17.0 that this module exists to absorb:

* ``MAV_SYS_ID`` is ``instance + 1`` (``ROMFS/px4fmu_common/init.d-posix/rcS``).
  ``Commander.cpp`` drops any ``VehicleCommand`` whose ``target_system`` matches
  neither ``0`` nor that value, so a hardcoded ``1`` is silently ignored
  everywhere except instance 0.
* PX4 applies the DDS namespace ``-n px4_<N>`` only when ``N != 0``, so instance
  0 publishes bare ``/fmu/out/...`` while every other instance is namespaced.
  We override that asymmetry (``PX4_UXRCE_DDS_NS``) so all instances are
  namespaced uniformly and there is exactly one code path.

This module is deliberately dependency-free — standard library only, no ROS, no
numpy — so that tests covering it run in milliseconds without a simulator.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Mapping

SPEC_VERSION = "1"

#: uXRCE-DDS agent UDP port for instance 0; later instances offset from here.
XRCE_BASE_PORT = 8888

#: ROS 2 domain ids above this can collide with the Linux ephemeral port range
#: (``/proc/sys/net/ipv4/ip_local_port_range``, typically 32768-60999) under
#: default DDS settings. The theoretical maximum is 232; 101 is the safe one.
MAX_ROS_DOMAIN_ID = 101

DEFAULT_MODEL = "x500"
DEFAULT_WORLD = "default"

#: Direction segment of a PX4 DDS topic name. ``out`` is PX4 publishing to us.
_TOPIC_DIRECTIONS = ("in", "out")


class InstanceSpecError(ValueError):
    """Raised when a requested worker identity is not representable."""


@dataclass(frozen=True)
class InstanceSpec:
    """Every per-worker resource, derived from the instance number.

    Construct with :meth:`for_instance` rather than calling this directly, so
    the derived fields are always computed the same way.
    """

    spec_version: str
    instance: int
    world_index: int
    mav_sys_id: int
    xrce_port: int
    ros_domain_id: int
    topic_ns: str
    gz_partition: str
    world: str
    model: str
    model_name: str
    spawn_pose: tuple[float, float, float, float, float, float]
    speed_factor: float
    headless: bool = True

    # ---------------------------------------------------------------- build

    @classmethod
    def for_instance(
        cls,
        instance: int,
        *,
        world_index: int | None = None,
        world: str = DEFAULT_WORLD,
        model: str = DEFAULT_MODEL,
        speed_factor: float = 1.0,
        spawn_pose: tuple[float, float, float, float, float, float] | None = None,
        headless: bool = True,
    ) -> "InstanceSpec":
        """Derive a worker's full identity from its instance number.

        ``world_index`` defaults to ``instance``, i.e. one drone per world
        (decision D7). M4's throughput benchmark is the only thing that will
        pass a different value, to group several drones into one world; keeping
        the Gazebo partition keyed off ``world_index`` rather than ``instance``
        means that change is a different argument here, not a redesign.

        ``spawn_pose`` defaults to the world origin. Because each worker owns
        its own world, every worker can spawn at the same place — which removes
        a per-instance special case, since a mission defined in local
        coordinates is then identical for every worker.
        """
        if not isinstance(instance, int) or isinstance(instance, bool):
            raise InstanceSpecError(f"instance must be an int, got {instance!r}")
        if instance < 0:
            raise InstanceSpecError(f"instance must be >= 0, got {instance}")
        if instance > MAX_ROS_DOMAIN_ID:
            raise InstanceSpecError(
                f"instance {instance} would need ROS_DOMAIN_ID {instance}, above the "
                f"safe maximum of {MAX_ROS_DOMAIN_ID} (higher ids can collide with the "
                f"Linux ephemeral port range)"
            )
        if speed_factor <= 0:
            raise InstanceSpecError(f"speed_factor must be > 0, got {speed_factor}")

        world_index = instance if world_index is None else world_index
        if world_index < 0:
            raise InstanceSpecError(f"world_index must be >= 0, got {world_index}")

        pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) if spawn_pose is None else spawn_pose
        if len(pose) != 6:
            raise InstanceSpecError(
                f"spawn_pose must be 6 values (x,y,z,roll,pitch,yaw), got {len(pose)}"
            )

        return cls(
            spec_version=SPEC_VERSION,
            instance=instance,
            world_index=world_index,
            # PX4 rcS: param set MAV_SYS_ID $((px4_instance+1))
            mav_sys_id=instance + 1,
            xrce_port=XRCE_BASE_PORT + instance,
            ros_domain_id=instance,
            # Uniform for every instance including 0 -- see module docstring.
            topic_ns=f"px4_{instance}",
            gz_partition=f"aero_{world_index}",
            world=world,
            model=model,
            # PX4 rcS names the spawned model "<model>_<instance>".
            model_name=f"{model}_{instance}",
            spawn_pose=tuple(float(v) for v in pose),  # type: ignore[arg-type]
            speed_factor=float(speed_factor),
            headless=headless,
        )

    # ---------------------------------------------------------------- topics

    def topic(self, name: str, direction: str = "out") -> str:
        """Full ROS topic name for a PX4 message.

        The only sanctioned way to name a PX4 topic in this project. Note that
        PX4 appends ``_v<N>`` to messages whose ``MESSAGE_VERSION`` is above 0
        (``vehicle_status_v1``, ``battery_status_v1``); that suffix is part of
        ``name`` and is not added here.
        """
        if direction not in _TOPIC_DIRECTIONS:
            raise InstanceSpecError(
                f"direction must be one of {_TOPIC_DIRECTIONS}, got {direction!r}"
            )
        return f"/{self.topic_ns}/fmu/{direction}/{name}"

    # ------------------------------------------------------------ processes

    @property
    def process_labels(self) -> tuple[str, str, str]:
        """Stable names for this worker's three processes, for logs and PID files."""
        return (
            f"gz_world_{self.world_index}",
            f"xrce_agent_{self.instance}",
            f"px4_instance_{self.instance}",
        )

    # ---------------------------------------------------------------- env

    def px4_env(self) -> dict[str, str]:
        """Environment variables the ``px4`` binary needs for this worker.

        ``GZ_PARTITION`` must also be set on the Gazebo server process; PX4
        shells out to ``gz service``/``gz topic`` and ``gz_bridge`` opens its
        own transport node, so both ends need it or the instance silently joins
        a neighbour's world.
        """
        return {
            "PX4_SIM_MODEL": f"gz_{self.model}",
            "PX4_GZ_WORLD": self.world,
            "PX4_GZ_STANDALONE": "1",
            "PX4_SIM_SPEED_FACTOR": _fmt_number(self.speed_factor),
            "PX4_GZ_MODEL_POSE": ",".join(_fmt_number(v) for v in self.spawn_pose),
            "PX4_UXRCE_DDS_PORT": str(self.xrce_port),
            "PX4_UXRCE_DDS_NS": self.topic_ns,
            "ROS_DOMAIN_ID": str(self.ros_domain_id),
            "GZ_PARTITION": self.gz_partition,
            "GZ_IP": "127.0.0.1",
        }

    # --------------------------------------------------------------- (de)ser

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["spawn_pose"] = list(self.spawn_pose)
        return d

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "InstanceSpec":
        got = data.get("spec_version")
        if got != SPEC_VERSION:
            raise InstanceSpecError(
                f"spec_version mismatch: file has {got!r}, this code writes "
                f"{SPEC_VERSION!r}. Restart the workers rather than mixing versions."
            )
        payload = dict(data)
        payload["spawn_pose"] = tuple(payload["spawn_pose"])
        return cls(**payload)

    @classmethod
    def from_file(cls, path: str | Path) -> "InstanceSpec":
        """Read a worker's identity from its ``instance_<N>.json`` handshake file."""
        data = json.loads(Path(path).read_text())
        return cls.from_dict(data["spec"] if "spec" in data else data)


@dataclass
class InstanceRuntime:
    """What the launcher learned at start time — PIDs, logs, timestamps.

    Kept separate from :class:`InstanceSpec` because identity is a pure function
    of the instance number and must stay testable without a simulator, whereas
    this half only exists once processes are running.
    """

    pid_gz: int | None = None
    pid_agent: int | None = None
    pid_px4: int | None = None
    log_gz: str | None = None
    log_agent: str | None = None
    log_px4: str | None = None
    started_utc: str | None = None
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def write_instance_file(
    path: str | Path, spec: InstanceSpec, runtime: InstanceRuntime | None = None
) -> Path:
    """Write the handshake file both the shell and Python layers read."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "spec": spec.to_dict(),
        "runtime": (runtime or InstanceRuntime()).to_dict(),
    }
    p.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return p


def _fmt_number(value: float) -> str:
    """Render a float without a trailing ``.0``, so env vars stay readable."""
    return str(int(value)) if float(value).is_integer() else repr(float(value))


def _main() -> int:
    ap = argparse.ArgumentParser(
        description="Emit one worker's identity as JSON, for shell consumption."
    )
    ap.add_argument("-i", "--instance", type=int, required=True)
    ap.add_argument("--world-index", type=int, default=None)
    ap.add_argument("-w", "--world", default=DEFAULT_WORLD)
    ap.add_argument("-m", "--model", default=DEFAULT_MODEL)
    ap.add_argument("-s", "--speed", type=float, default=1.0)
    ap.add_argument("-p", "--pose", default=None, help="x,y,z,roll,pitch,yaw")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument(
        "--env",
        action="store_true",
        help="emit shell 'export K=V' lines for the px4 process instead of JSON",
    )
    ap.add_argument(
        "--shell",
        action="store_true",
        help="emit SPEC_* shell assignments for a launcher to eval",
    )
    args = ap.parse_args()

    pose = None
    if args.pose:
        parts = [float(v) for v in args.pose.split(",")]
        pose = tuple(parts)  # length validated in for_instance

    try:
        spec = InstanceSpec.for_instance(
            args.instance,
            world_index=args.world_index,
            world=args.world,
            model=args.model,
            speed_factor=args.speed,
            spawn_pose=pose,
            headless=not args.gui,
        )
    except InstanceSpecError as exc:
        print(f"ERROR: {exc}", flush=True)
        return 1

    if args.env:
        for key, value in spec.px4_env().items():
            print(f"export {key}={_shell_quote(value)}")
    elif args.shell:
        for key, value in spec.to_dict().items():
            if key == "spawn_pose":
                value = ",".join(_fmt_number(v) for v in spec.spawn_pose)
            elif isinstance(value, bool):
                value = "1" if value else "0"
            elif isinstance(value, float):
                value = _fmt_number(value)
            print(f"SPEC_{key.upper()}={_shell_quote(str(value))}")
    else:
        print(json.dumps(spec.to_dict(), indent=2, sort_keys=True))
    return 0


def _shell_quote(value: str) -> str:
    return "'" + value.replace("'", "'\\''") + "'"


if __name__ == "__main__":
    raise SystemExit(_main())
