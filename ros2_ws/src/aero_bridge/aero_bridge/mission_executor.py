"""M3's one mission executor: takeoff -> fly a sequence of waypoints in
offboard mode, holding briefly at each -> return to the start -> hover ->
land. Built entirely on aero_bridge.arming_sequence (the project's one
flight-transition implementation) and PX4Clock (the project's one sim-time
source, CLAUDE.md D10); this file adds no new low-level flight primitive of
its own, only the waypoint sequencing and per-step measurement on top of
them.

Missions are plain dicts loaded from configs/missions/*.yaml -- see
load_mission()/validate_mission_config(). Waypoints, acceptance radius, hold
times, geofence and the sim-time timeout all live in that file, not in code
(milestones.md M3 task 3), so a new mission is a new YAML file, never a new
Python module.

Produces one per-step record per control tick (10Hz -- independent of the
20Hz offboard setpoint stream arming_sequence needs) and a single per-episode
MissionResult once the mission ends, in whatever way it ends. Every episode
gets a result: MissionResult is built in a `finally` so a HoldTimeout or an
unexpected exception still yields the steps collected so far and a
termination_reason, rather than losing the partial flight's data.
"""
from __future__ import annotations

import math
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping, Optional

import yaml
from px4_msgs.msg import VehicleStatus

from aero_bridge.arming_sequence import (
    ArmTimeout,
    FlightSequenceError,
    HoldTimeout,
    LandTimeout,
    OffboardLost,
    OffboardRejected,
    PreflightFailed,
    arm_and_engage_offboard,
    hold_position_until,
    land_and_wait,
)
from aero_bridge.px4_clock import PX4Clock
from aero_bridge.px4_interface import PX4Interface
from experiments.episode_schema import TerminationReason

CONTROL_RATE_HZ = 10.0  # per-step record rate; independent of the setpoint stream rate
CONTROL_PERIOD_S = 1.0 / CONTROL_RATE_HZ

# A single stuck waypoint/hover's wall-clock hang watchdog (arming_sequence's
# hold_position_until timeout_s). Deliberately NOT the mission's own
# timeout_s: that field is a sim-time budget for the whole mission, checked
# separately below by check_mission_deadline() on every poll, and reusing it
# here would make a single stuck step wait the entire mission budget before
# failing.
WAYPOINT_WALL_WATCHDOG_S = 60.0


class MissionConfigError(ValueError):
    """A mission YAML is missing a field or is internally inconsistent
    (e.g. a waypoint outside its own geofence)."""


class MissionTimeout(FlightSequenceError):
    """The mission's own sim-time budget (timeout_s) elapsed before the
    mission finished. Distinct from HoldTimeout/ArmTimeout/LandTimeout,
    which are wall-clock hang watchdogs for a single stuck step, not a
    mission-duration budget."""


class SimFault(FlightSequenceError):
    """Position or velocity reported a non-finite value (docs/parallelism.md
    §8's last catalogue row). Distinct from every other FlightSequenceError
    here: those are outcomes a real vehicle can also produce (a timeout, a
    rejected mode switch); this one means the simulation state itself broke
    (Gazebo/PX4 diverged into a physically meaningless state), so the
    episode is invalidated rather than treated as an ordinary unsuccessful
    flight."""


def _is_finite_state(px: float, py: float, pz: float,
                      vx: float, vy: float, vz: float) -> bool:
    """True iff every position/velocity component is a real, finite number.
    A pure function so it's testable without a simulator or even a Node."""
    return all(math.isfinite(v) for v in (px, py, pz, vx, vy, vz))


def _quaternion_to_euler(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Hamiltonian quaternion q=(w, x, y, z), FRD body frame relative to NED
    earth frame -- confirmed from VehicleAttitude.msg's own comment
    ("The quaternion uses the Hamilton convention, and the order is
    q(w, x, y, z)"), not guessed (M0's rule: verify actual field meanings,
    don't assume them). Returns (roll, pitch, yaw) in radians, the standard
    aerospace ZYX Euler sequence.

    A pure function of four floats, tested directly
    (tests/test_mission_executor.py) rather than only indirectly through a
    live flight -- this is exactly the kind of conversion that is easy to
    get silently backwards (M5's own watch-out-for note)."""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


_REQUIRED_MISSION_FIELDS = (
    "mission_id", "schema_version", "altitude_m", "acceptance_radius_m",
    "hold_time_s", "final_hover_s", "timeout_s", "waypoints", "geofence",
)
_REQUIRED_GEOFENCE_FIELDS = ("x_min_m", "x_max_m", "y_min_m", "y_max_m", "z_min_m", "z_max_m")

_REASON_FOR_ERROR: dict[type, TerminationReason] = {
    PreflightFailed: TerminationReason.PREFLIGHT_FAILED,
    ArmTimeout: TerminationReason.ARM_TIMEOUT,
    OffboardRejected: TerminationReason.OFFBOARD_REJECTED,
    OffboardLost: TerminationReason.OFFBOARD_LOST,
    HoldTimeout: TerminationReason.HOLD_TIMEOUT,
    LandTimeout: TerminationReason.LAND_TIMEOUT,
    MissionTimeout: TerminationReason.EPISODE_TIMEOUT,
    SimFault: TerminationReason.SIM_FAULT,
}


def load_mission(path: str | Path) -> dict[str, Any]:
    """Loads and validates a mission YAML. The only sanctioned way to get a
    mission dict -- nothing else in this project parses a mission file."""
    mission = yaml.safe_load(Path(path).read_text())
    validate_mission_config(mission)
    return mission


def validate_mission_config(mission: Mapping[str, Any]) -> None:
    """Raises MissionConfigError if `mission` is missing a required field or
    is internally inconsistent. A waypoint outside the mission's own
    geofence is a config bug, and is caught here rather than mid-flight."""
    missing = [f for f in _REQUIRED_MISSION_FIELDS if f not in mission]
    if missing:
        raise MissionConfigError(f"mission config missing required fields: {missing}")

    geofence = mission["geofence"]
    missing_fence = [f for f in _REQUIRED_GEOFENCE_FIELDS if f not in geofence]
    if missing_fence:
        raise MissionConfigError(f"geofence missing required fields: {missing_fence}")

    if geofence["x_min_m"] >= geofence["x_max_m"]:
        raise MissionConfigError("geofence x_min_m must be < x_max_m")
    if geofence["y_min_m"] >= geofence["y_max_m"]:
        raise MissionConfigError("geofence y_min_m must be < y_max_m")
    if geofence["z_min_m"] >= geofence["z_max_m"]:
        raise MissionConfigError("geofence z_min_m must be < z_max_m (NED: min is the ceiling)")

    takeoff_z = -abs(mission["altitude_m"])
    if not (geofence["z_min_m"] <= takeoff_z <= geofence["z_max_m"]):
        raise MissionConfigError(
            f"mission altitude {mission['altitude_m']}m (z={takeoff_z}) falls outside "
            f"geofence z range [{geofence['z_min_m']}, {geofence['z_max_m']}]"
        )

    for i, (wx, wy) in enumerate(mission["waypoints"]):
        if not (geofence["x_min_m"] <= wx <= geofence["x_max_m"]):
            raise MissionConfigError(f"waypoint {i} x={wx} falls outside geofence x range")
        if not (geofence["y_min_m"] <= wy <= geofence["y_max_m"]):
            raise MissionConfigError(f"waypoint {i} y={wy} falls outside geofence y range")


@dataclass
class MissionResult:
    termination_reason: str
    n_steps: int
    waypoints_reached: int
    position_rmse_m: float
    final_position_error_m: float
    t_sim_start_s: Optional[float]
    t_sim_end_s: Optional[float]
    steps: list[dict] = field(default_factory=list)


def fly_mission(node, px4: PX4Interface, clock: PX4Clock, mission: dict[str, Any],
                 on_step: Optional[Callable[[dict], None]] = None) -> MissionResult:
    """Flies `mission` (as returned by load_mission()) to completion.

    `on_step` is called once per control tick with a step-record dict as
    soon as it is built -- the caller (EpisodeLogger, or a test) decides
    what to do with it. Kept as a callback rather than this function owning
    an EpisodeLogger so fly_mission stays a pure flight-and-measure
    primitive, reusable from a script, a test, or (M9) a Gym env step loop.

    Always returns a MissionResult, never raises: every FlightSequenceError
    (including this module's own MissionTimeout) is caught and mapped to a
    termination_reason via _REASON_FOR_ERROR, and an unexpected exception is
    reported as ABORTED_ERROR. The caller decides what to do next (M3's
    run_episodes.py resets and moves on to the next episode either way) --
    an episode that ended badly is a real, recordable measurement, not a
    reason to crash the run.
    """
    takeoff_z = -abs(mission["altitude_m"])
    accept_r = mission["acceptance_radius_m"]
    hold_s = mission["hold_time_s"]
    final_hover_s = mission["final_hover_s"]
    timeout_s = mission["timeout_s"]
    waypoints = mission["waypoints"]

    steps: list[dict] = []
    errors: list[float] = []
    waypoints_reached = 0
    state = {"step_index": 0, "t_sim_start_s": None}

    def check_mission_deadline() -> None:
        if state["t_sim_start_s"] is None:
            return
        now_us = clock.now_us()
        if now_us is None:
            return
        elapsed = now_us / 1e6 - state["t_sim_start_s"]
        if elapsed > timeout_s:
            raise MissionTimeout(f"mission exceeded its {timeout_s}s sim-time budget")

    def record_step(target: tuple[float, float, float]) -> None:
        odom = px4.latest['vehicle_odometry']
        status = px4.latest['vehicle_status']
        attitude = px4.latest['vehicle_attitude']
        sensors = px4.latest['sensor_combined']
        motors = px4.latest['actuator_motors']
        if odom is None or status is None or attitude is None or sensors is None:
            return
        battery = px4.latest['battery_status']
        px, py, pz = odom.position[0], odom.position[1], odom.position[2]
        vx, vy, vz = odom.velocity[0], odom.velocity[1], odom.velocity[2]
        if not _is_finite_state(px, py, pz, vx, vy, vz):
            raise SimFault(f"non-finite vehicle state: pos=({px},{py},{pz}) vel=({vx},{vy},{vz})")
        err = math.dist((px, py, pz), target)
        errors.append(err)
        roll, pitch, yaw = _quaternion_to_euler(tuple(attitude.q))
        # actuator_motors.control has up to 12 entries; this project's
        # airframe (x500) uses the first 4 (schema v3 -- M5 task 2). NaN
        # ("not supported by this output" per ActuatorMotors.msg) can appear
        # briefly around arm/disarm transitions; passed through as-is rather
        # than papered over, since a NaN motor output IS the true value.
        motor_outputs = (
            tuple(motors.control[0:4]) if motors is not None else (float('nan'),) * 4)
        row = dict(
            step_index=state["step_index"],
            t_sim_s=(clock.now_us() or 0) / 1e6,
            t_wall_utc=time.time(),
            armed=status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            nav_state=int(status.nav_state),
            pos_x=px, pos_y=py, pos_z=pz,
            vel_x=vx, vel_y=vy, vel_z=vz,
            target_x=target[0], target_y=target[1], target_z=target[2],
            position_error_m=err,
            battery_remaining=(battery.remaining if battery is not None else float('nan')),
            roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw,
            rate_p_rad_s=sensors.gyro_rad[0], rate_q_rad_s=sensors.gyro_rad[1],
            rate_r_rad_s=sensors.gyro_rad[2],
            accel_x_m_s2=sensors.accelerometer_m_s2[0],
            accel_y_m_s2=sensors.accelerometer_m_s2[1],
            accel_z_m_s2=sensors.accelerometer_m_s2[2],
            motor_0_output=motor_outputs[0], motor_1_output=motor_outputs[1],
            motor_2_output=motor_outputs[2], motor_3_output=motor_outputs[3],
        )
        state["step_index"] += 1
        steps.append(row)
        if on_step is not None:
            on_step(row)

    def make_hold_condition(target: tuple[float, float, float], hold_duration_s: float):
        """A reached()-and-held() condition: True once the vehicle has been
        within acceptance_radius_m of `target` for hold_duration_s of SIM
        time. Also records a step and checks the mission deadline on every
        poll, so both happen at the same ~control-loop cadence as the
        vehicle is polled -- no separate timer thread."""
        held_since = {"t": None}
        next_record = {"t": 0.0}

        def condition() -> bool:
            check_mission_deadline()
            now_us = clock.now_us()
            now_s = (now_us or 0) / 1e6
            if now_s >= next_record["t"]:
                record_step(target)
                next_record["t"] = now_s + CONTROL_PERIOD_S

            odom = px4.latest['vehicle_odometry']
            if odom is None:
                held_since["t"] = None
                return False
            within = math.dist(
                (odom.position[0], odom.position[1], odom.position[2]), target) <= accept_r
            if not within:
                held_since["t"] = None
                return False
            if now_us is None:
                # A transient clock read failure (clock.now_us() returning
                # None) -- rare, and more likely under heavy multi-worker
                # CPU contention. Don't let it corrupt the hold timer: don't
                # start OR evaluate it on a tick with no real timestamp.
                # Found live (M4 task 6's throughput sweep, worker_count=3):
                # this used to set held_since["t"] = None while `within` was
                # True, and a LATER tick with a real timestamp then computed
                # <int> - None and crashed the whole mission with an
                # unhandled TypeError.
                return False
            if held_since["t"] is None:
                held_since["t"] = now_us
            return (now_us - held_since["t"]) / 1e6 >= hold_duration_s

        return condition

    try:
        arm_and_engage_offboard(node, px4, clock, takeoff_z=takeoff_z)
        state["t_sim_start_s"] = (clock.now_us() or 0) / 1e6

        for wx, wy in waypoints:
            target = (wx, wy, takeoff_z)
            hold_position_until(
                node, px4, clock, x=target[0], y=target[1], z=target[2],
                is_reached=make_hold_condition(target, hold_s),
                timeout_s=WAYPOINT_WALL_WATCHDOG_S,
                description=f"reach waypoint ({wx}, {wy})")
            waypoints_reached += 1

        final = (waypoints[-1][0], waypoints[-1][1], takeoff_z)
        hold_position_until(
            node, px4, clock, x=final[0], y=final[1], z=final[2],
            is_reached=make_hold_condition(final, final_hover_s),
            timeout_s=WAYPOINT_WALL_WATCHDOG_S, description="final hover")

        land_and_wait(node, px4, clock)
        reason = TerminationReason.COMPLETED
    except FlightSequenceError as exc:
        reason = _REASON_FOR_ERROR.get(type(exc), TerminationReason.ABORTED_ERROR)
    except Exception as exc:
        # Deliberately still caught rather than propagated (this function's
        # contract is to always return a MissionResult so a caller flying
        # many episodes in a loop does not lose the whole run to one bad
        # episode) -- but logged with a traceback rather than silently
        # relabelled, so an actual bug here reads as an actual bug and not
        # as unexplained simulator flakiness three episodes later. A logger
        # bug that raised SchemaValidationError from on_step() did exactly
        # that before this was added.
        node.get_logger().error(
            f"mission aborted by unexpected exception: {exc!r}\n{traceback.format_exc()}")
        reason = TerminationReason.ABORTED_ERROR

    t_sim_end_s = (clock.now_us() or 0) / 1e6 if clock.now_us() is not None else None
    rmse = math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else float('nan')
    final_err = errors[-1] if errors else float('nan')

    return MissionResult(
        termination_reason=reason.value,
        n_steps=len(steps),
        waypoints_reached=waypoints_reached,
        position_rmse_m=rmse,
        final_position_error_m=final_err,
        t_sim_start_s=state["t_sim_start_s"],
        t_sim_end_s=t_sim_end_s,
        steps=steps,
    )
