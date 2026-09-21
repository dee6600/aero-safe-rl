"""M3 task 5: the three-tier reset ladder between episodes.

Each tier answers "how do we get back to a known-clean starting state" at a
different cost, verified live against the real simulator while writing this
module (not assumed -- see the two findings below, which changed the design):

* **soft** -- disarm (idempotent), teleport the model back to the spawn pose
  via Gazebo's `/world/<w>/set_pose` service, then wait for PX4's own
  position estimate to reconverge to that pose. Fastest, but only safe from
  an already-disarmed, at-rest vehicle: measured live that force-disarming
  a hovering vehicle makes it free-fall and bounce, after which neither
  set_pose nor `WorldControl.reset.model_only` reliably restores a clean
  position/velocity (gz's own reported pose and PX4's fused odometry
  disagreed after that reset in a live test -- not well enough understood to
  rely on). soft_reset() therefore requires the vehicle to already be
  disarmed and raises ResetUnsafe otherwise, pushing the caller to escalate.
  Every M3 episode reaches this state via mission_executor's land_and_wait(),
  so the precondition holds on the path this milestone actually exercises;
  M6's fault injection is expected to hit the unsafe path routinely, which is
  exactly why medium and hard exist.
* **medium** -- the same teleport, plus `VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN`.
  Measured live: PX4's OS process does NOT exit (same PID throughout), so the
  existing rclpy Node/PX4Interface/GzSimClock objects stay valid across it --
  no reconnection needed, unlike hard. BUT also measured live: this only
  recovers cleanly from a PX4 instance that has never yet armed in that
  session -- pre_flight_checks_pass returned true ~0.5s after the command
  there, and a fresh arm+offboard+land cycle worked immediately after. Sent
  to a PX4 that HAS already armed and flown (the only state this tier would
  ever actually be invoked from), pre_flight_checks_pass was observed to
  never recover, for as long as this was measured (up to 75s wall); PX4's own
  own log stops emitting any further module output at all right after the
  command in that case (no "logger"/"commander" lines, though telemetry
  keeps publishing), suggesting the internal reinit this command triggers is
  incomplete for a PX4 that has already flown, under this exact stack (PX4
  v1.17.0 SITL + Gazebo Harmonic via gz-bridge). Not further root-caused --
  out of scope for M3, and hard_reset is unaffected by it (a real process
  restart, not this in-place one) -- but medium_reset() is therefore NOT the
  reliable middle tier its cost alone would suggest; prefer hard until this
  is investigated further. Kept implemented (the milestone asks for all
  three tiers to exist and be measured) and raises ResetTimeout honestly
  rather than hanging or pretending to have succeeded.
* **hard** -- scripts/sim_stop.sh + scripts/sim_start.sh: a full worker
  restart. Always correct, always available regardless of what state the
  vehicle is in, slowest. Unlike soft/medium, the caller's rclpy Node,
  PX4Interface and GzSimClock are for the OLD process and must be rebuilt
  after this returns -- hard_reset() cannot hide that the way the other two
  hide their own internals, so it deliberately has a different signature.

PX4 clears its rate/attitude controller integrators on every disarm
transition on its own; no explicit "clear integrators" step exists here
because there is nothing to add to that.
"""
from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from px4_msgs.msg import VehicleCommand, VehicleStatus

from aero_bridge.px4_clock import PX4Clock
from aero_bridge.px4_interface import PX4Interface
from experiments.episode_schema import ResetTier
from simulation import sim_clock  # import side effect: gz Python bindings on sys.path
from simulation.instance_spec import InstanceSpec

import gz.transport13 as _gz_transport
from gz.msgs10.boolean_pb2 import Boolean as _GzBoolean
from gz.msgs10.pose_pb2 import Pose as _GzPose

SOFT_RESET_WALL_TIMEOUT_S = 15.0
MEDIUM_RESET_WALL_TIMEOUT_S = 25.0
HARD_RESET_WALL_TIMEOUT_S = 150.0

# How close position/velocity must get to "reset" before soft/medium return.
# Generous relative to the accuracy the mission itself needs (acceptance
# radii of order 1m) -- this just needs to prove nothing is still moving or
# badly out of place, not hit sub-centimetre precision.
POSITION_TOLERANCE_M = 0.3
VELOCITY_TOLERANCE_M_S = 0.2

REPO_DIR = Path(__file__).resolve().parents[4]


class ResetError(RuntimeError):
    """Base class for reset failures."""


class ResetUnsafe(ResetError):
    """Requested tier's precondition does not hold (e.g. soft_reset() called
    on a still-armed vehicle) -- the caller should escalate to a stronger
    tier rather than retry the same one."""


class ResetTimeout(ResetError):
    """The tier completed its actions but the vehicle never settled to
    within tolerance before the wall-clock deadline."""


@dataclass
class ResetResult:
    tier: str
    wall_duration_s: float


def _is_armed(status) -> bool:
    return status is not None and status.arming_state == VehicleStatus.ARMING_STATE_ARMED


def _ned_to_gz_enu(x: float, y: float, z: float) -> tuple[float, float, float]:
    """PX4's local NED frame (x=north, y=east, z=down) vs Gazebo's world
    frame (x=east, y=north, z=up) -- measured live via set_pose + odometry
    round-trip while writing this module: commanding gz (x=3, y=2) settled
    at PX4 odometry (x=2, y=3), i.e. X/Y swapped, and Z negated."""
    return (y, x, -z)


def _teleport_to_spawn(spec: InstanceSpec) -> None:
    node = _gz_transport.Node()
    gx, gy, gz_ = _ned_to_gz_enu(*spec.spawn_pose[:3])
    req = _GzPose()
    req.name = spec.model_name
    req.position.x = gx
    req.position.y = gy
    req.position.z = gz_
    req.orientation.w = 1.0  # level, spawn yaw -- M3's one fixed spawn pose
    ok, resp = node.request(
        f"/world/{spec.world}/set_pose", req, _GzPose, _GzBoolean, 3000)
    if not ok or not resp.data:
        raise ResetError(f"gz set_pose to spawn failed for {spec.model_name}")


def _already_at_spawn(px4: PX4Interface, spec: InstanceSpec) -> bool:
    """Whether the vehicle is already close enough to spawn that a teleport
    would be a no-op. Checked so resets skip `gz set_pose` when it is not
    needed -- found live while measuring M3's 20-episode baseline that
    REPEATED set_pose calls (however small the actual jump) accumulate
    towards PX4's magnetometer consistency check tripping permanently
    ("Preflight Fail: Compass 0 fault", never observed to clear again short
    of a hard reset). square_circuit.yaml's mission always returns to its
    own start point before landing, so the vehicle is normally already at
    spawn by the time reset runs; not teleporting in that (normal) case
    avoids the fault entirely rather than requiring the run to recover from
    it after the fact."""
    odom = px4.latest['vehicle_odometry']
    if odom is None:
        return False
    target = spec.spawn_pose[:3]
    pos_err = sum((a - b) ** 2 for a, b in zip(odom.position, target)) ** 0.5
    return pos_err <= POSITION_TOLERANCE_M


def _force_disarm_if_armed(px4: PX4Interface, clock: PX4Clock) -> None:
    """Idempotent: a no-op wait-and-confirm if the vehicle is already
    disarmed (the normal post-land_and_wait() case), a real force-disarm
    (PX4's documented force-disarm magic value, param2=21196) otherwise."""
    status = px4.latest['vehicle_status']
    if not _is_armed(status):
        return
    px4.publish_vehicle_command(
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0, param2=21196.0)


def _wait_until_settled(px4: PX4Interface, clock: PX4Clock, spec: InstanceSpec,
                         wall_timeout_s: float, require_disarmed: bool) -> None:
    target = spec.spawn_pose[:3]
    wall_deadline = time.monotonic() + wall_timeout_s
    pos_err = speed = None
    while True:
        clock.pump(0.02)
        odom = px4.latest['vehicle_odometry']
        status = px4.latest['vehicle_status']
        if odom is not None and status is not None:
            pos_err = sum((a - b) ** 2 for a, b in zip(odom.position, target)) ** 0.5
            speed = sum(v * v for v in odom.velocity) ** 0.5
            disarmed_ok = (not require_disarmed) or not _is_armed(status)
            if pos_err <= POSITION_TOLERANCE_M and speed <= VELOCITY_TOLERANCE_M_S and disarmed_ok:
                return
        if time.monotonic() > wall_deadline:
            raise ResetTimeout(
                f"reset did not settle within {wall_timeout_s}s "
                f"(pos_err={pos_err}, speed={speed})")


def soft_reset(node, px4: PX4Interface, clock: PX4Clock, spec: InstanceSpec, *,
               timeout_s: float = SOFT_RESET_WALL_TIMEOUT_S) -> ResetResult:
    """Fastest tier. Raises ResetUnsafe if the vehicle is currently armed --
    see the module docstring for why this precondition is enforced rather
    than worked around."""
    t0 = time.monotonic()
    status = px4.latest['vehicle_status']
    if _is_armed(status):
        raise ResetUnsafe("soft_reset requires an already-disarmed vehicle; escalate to medium/hard")

    if not _already_at_spawn(px4, spec):
        _teleport_to_spawn(spec)
    _wait_until_settled(px4, clock, spec, timeout_s, require_disarmed=True)
    return ResetResult(tier=ResetTier.SOFT.value, wall_duration_s=time.monotonic() - t0)


def medium_reset(node, px4: PX4Interface, clock: PX4Clock, spec: InstanceSpec, *,
                  timeout_s: float = MEDIUM_RESET_WALL_TIMEOUT_S) -> ResetResult:
    """Same Gazebo server and PX4 OS process, but PX4's own estimator and
    mode state machine get a real restart -- when it works. Measured live
    (module docstring) to reliably recover only from a PX4 that has not yet
    armed this session; from a post-flight PX4 (this tier's actual intended
    use) pre_flight_checks_pass was never observed to recover, so this
    raises ResetTimeout in that case rather than hang or claim success.
    Prefer hard_reset() until that gap is investigated further."""
    t0 = time.monotonic()
    _force_disarm_if_armed(px4, clock)
    wall_deadline = time.monotonic() + timeout_s
    while _is_armed(px4.latest['vehicle_status']):
        clock.pump(0.02)
        if time.monotonic() > wall_deadline:
            raise ResetTimeout("medium_reset: force-disarm never confirmed")

    if not _already_at_spawn(px4, spec):
        _teleport_to_spawn(spec)
    px4.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1=1.0)

    remaining = timeout_s - (time.monotonic() - t0)
    _wait_until_settled(px4, clock, spec, max(remaining, 1.0), require_disarmed=False)

    # PREFLIGHT_REBOOT_SHUTDOWN clears arming_state too, but confirm it
    # explicitly rather than trusting _wait_until_settled's velocity/position
    # check alone to have implied "disarmed" here.
    remaining = timeout_s - (time.monotonic() - t0)
    wall_deadline = time.monotonic() + max(remaining, 1.0)
    while True:
        status = px4.latest['vehicle_status']
        if status is not None and status.pre_flight_checks_pass and not _is_armed(status):
            break
        if time.monotonic() > wall_deadline:
            raise ResetTimeout("medium_reset: PX4 never became ready (preflight pass, disarmed) after reboot")
        clock.pump(0.02)

    return ResetResult(tier=ResetTier.MEDIUM.value, wall_duration_s=time.monotonic() - t0)


def hard_reset(spec: InstanceSpec, *, run_dir: Optional[str] = None,
               repo_dir: Path = REPO_DIR, timeout_s: float = HARD_RESET_WALL_TIMEOUT_S) -> ResetResult:
    """Full worker restart via scripts/sim_stop.sh + scripts/sim_start.sh.
    Always correct, regardless of the vehicle's prior state -- the only tier
    safe to call with no telemetry at all (e.g. after a dead DDS link).

    Unlike soft_reset/medium_reset, this does not take (or reuse) a node,
    PX4Interface or PX4Clock: those belong to the OLD PX4 process, which no
    longer exists once this returns. The caller must construct fresh ones
    against the same `spec` (instance number, world, etc. are unchanged --
    only the processes and their PIDs are new).
    """
    t0 = time.monotonic()
    extra = ["--run-dir", run_dir] if run_dir else []

    stop = subprocess.run(
        [str(repo_dir / "scripts" / "sim_stop.sh"), "-i", str(spec.instance), *extra],
        capture_output=True, text=True, timeout=timeout_s / 2)
    if stop.returncode != 0:
        raise ResetError(f"hard_reset: sim_stop.sh failed:\n{stop.stdout}\n{stop.stderr}")

    remaining = max(timeout_s - (time.monotonic() - t0), 10.0)
    start = subprocess.run(
        [str(repo_dir / "scripts" / "sim_start.sh"),
         "-i", str(spec.instance), "-w", spec.world, "-m", spec.model,
         "-s", str(spec.speed_factor), *extra],
        capture_output=True, text=True, timeout=remaining)
    if start.returncode != 0:
        raise ResetError(f"hard_reset: sim_start.sh failed:\n{start.stdout}\n{start.stderr}")

    return ResetResult(tier=ResetTier.HARD.value, wall_duration_s=time.monotonic() - t0)
