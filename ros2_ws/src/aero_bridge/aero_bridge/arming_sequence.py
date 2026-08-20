"""The project's only implementations of the recurring flight transitions:
stream setpoints -> engage offboard -> arm; hold position until some
condition is true; land and wait for disarm. Every node that flies -- this
milestone's test flight, M3's mission executor, M9's Gym environment -- calls
these. Nothing re-implements them.

All three share one shape: publish a heartbeat + setpoint at a fixed
wall-clock rate (this is a real-world publish rate, not mission timing, so
wall-clock pacing is correct here), pump the ROS executor via PX4Clock, check
a condition, and raise a typed, diagnosable exception if a wall-clock deadline
passes first. The deadline is a hang watchdog (CLAUDE.md §4's explicit
carve-out for wall time), never mission duration -- a hover duration, for
example, must be expressed as a sim-time condition passed in by the caller
(see test_flight.py's use of clock.now_us()), not as this module's timeout_s.

Offboard commands (arm, engage_offboard_mode, land) travel over BEST_EFFORT
QoS with no delivery retry at the DDS layer, so a single message can be lost
in transit. Every command here is reissued on a fixed cadence until its
effect is confirmed, rather than sent once. This matters more than it looks:
confirmed during M2 (docs/parallelism.md §2.6) that PX4 can report
offboard_control_signal_lost -- and act on it -- even when this process's
own publish loop never misses a single scheduled setpoint, meaningfully more
often with a second worker also running. The loss happens somewhere in the
transport (DDS / MicroXRCEAgent / uxrce_dds_client), not in the publish
timing here, and re-issuing the offboard-engage command (hold_position_until
does this too, not just the initial arm) is what recovers it.
"""

import time

from px4_msgs.msg import VehicleStatus

from aero_bridge.px4_clock import PX4Clock
from aero_bridge.px4_interface import PX4Interface

STREAM_RATE_HZ = 20.0  # PX4 drops out of offboard if setpoints stop arriving at this rate
STREAM_PERIOD_S = 1.0 / STREAM_RATE_HZ
WARMUP_S = 0.75         # setpoints streamed before the first arm+offboard attempt
RESEND_INTERVAL_S = 1.0  # cadence for reissuing a BEST_EFFORT command until confirmed

ARM_TIMEOUT_S = 30.0
LAND_TIMEOUT_S = 30.0

# FailsafeFlags fields worth surfacing when a deadline is hit -- named per
# FailsafeFlags.msg's own convention (false == no failure), so "true" here
# always means "this is part of why we're stuck".
_DIAGNOSTIC_FLAGS = (
    'local_position_invalid', 'local_altitude_invalid', 'global_position_invalid',
    'home_position_invalid', 'offboard_control_signal_lost',
    'manual_control_signal_lost', 'gcs_connection_lost', 'battery_unhealthy',
    'fd_critical_failure', 'fd_esc_arming_failure', 'fd_imbalanced_prop',
    'fd_motor_failure', 'navigator_failure',
)


class FlightSequenceError(RuntimeError):
    """Base class for every typed failure this module raises."""


class PreflightFailed(FlightSequenceError):
    """PX4 never reported pre_flight_checks_pass=True within the deadline."""


class ArmTimeout(FlightSequenceError):
    """Preflight passed but PX4 never confirmed ARMED within the deadline."""


class OffboardRejected(FlightSequenceError):
    """Armed, but PX4 never entered NAVIGATION_STATE_OFFBOARD within the deadline."""


class HoldTimeout(FlightSequenceError):
    """hold_position_until's condition never became true within the deadline."""


class LandTimeout(FlightSequenceError):
    """PX4 never confirmed DISARMED within the deadline after landing was commanded."""


def _failing_flags(px4: PX4Interface) -> list:
    flags = px4.latest.get('failsafe_flags')
    if flags is None:
        return ['<no FailsafeFlags received yet>']
    failing = [name for name in _DIAGNOSTIC_FLAGS if getattr(flags, name, False)]
    return failing or ['<none reported>']


def _is_armed(status) -> bool:
    return status is not None and status.arming_state == VehicleStatus.ARMING_STATE_ARMED


def _is_offboard(status) -> bool:
    return status is not None and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD


def arm_and_engage_offboard(node, px4: PX4Interface, clock: PX4Clock, *,
                             takeoff_z: float, timeout_s: float = ARM_TIMEOUT_S) -> None:
    """Stream position setpoints, engage offboard mode, arm, and block until
    PX4 confirms both ARMED and NAVIGATION_STATE_OFFBOARD. Raises
    PreflightFailed, ArmTimeout, or OffboardRejected -- whichever the
    telemetry at deadline time actually indicates -- rather than a single
    generic timeout, so a caller (or a human reading the log) knows which
    stage actually got stuck.
    """
    wall_deadline = time.monotonic() + timeout_s
    next_publish = time.monotonic()
    next_command = next_publish + WARMUP_S

    while True:
        now = time.monotonic()

        if now >= next_publish:
            px4.publish_offboard_heartbeat(position=True)
            px4.publish_position_setpoint(0.0, 0.0, takeoff_z)
            next_publish = now + STREAM_PERIOD_S

        status = px4.latest['vehicle_status']
        if _is_armed(status) and _is_offboard(status):
            return

        if now >= next_command:
            px4.engage_offboard_mode()
            px4.arm()
            next_command = now + RESEND_INTERVAL_S

        if now > wall_deadline:
            _raise_arm_failure(px4, status, timeout_s)

        clock.pump()


def _raise_arm_failure(px4, status, timeout_s: float) -> None:
    failing = _failing_flags(px4)
    if status is None or not status.pre_flight_checks_pass:
        raise PreflightFailed(
            f"pre_flight_checks_pass never became true within {timeout_s}s; "
            f"failing flags: {failing}")
    if not _is_armed(status):
        raise ArmTimeout(
            f"never confirmed ARMED within {timeout_s}s (preflight passed); "
            f"failing flags: {failing}")
    raise OffboardRejected(
        f"armed but never reached NAVIGATION_STATE_OFFBOARD within {timeout_s}s; "
        f"failing flags: {failing}")


def hold_position_until(node, px4: PX4Interface, clock: PX4Clock, *,
                         x: float, y: float, z: float,
                         is_reached, timeout_s: float, description: str) -> None:
    """Keep streaming a fixed position setpoint (required to stay in offboard
    mode) while polling `is_reached()`, returning as soon as it is True.

    `is_reached` decides what "done" means and may reference PX4Clock's sim
    time for a duration-based wait (see test_flight.py's hover), or vehicle
    telemetry for a state-based one (see test_flight.py's takeoff altitude).
    Either way this function's own timeout_s remains a wall-clock hang
    watchdog, never the mission-relevant duration itself.

    Re-engages offboard mode if PX4 ever leaves it mid-wait. Found during M2
    (confirmed via .ulg log analysis, and via a publish loop instrumented to
    log any gap over 150ms): PX4 can report offboard_control_signal_lost --
    and act on it, switching nav_state away from OFFBOARD -- even though this
    process never actually missed a publish deadline. The loss happens
    somewhere in the BEST_EFFORT transport (DDS / MicroXRCEAgent /
    uxrce_dds_client), not in this loop, so no amount of publish-timing
    tuning here prevents it; it is meaningfully more frequent with a second
    worker also running (~65% of concurrent two-worker flights hit it at
    least once during development, vs ~10-20% solo). Streaming setpoints
    alone does not bring PX4 back into OFFBOARD once it has left -- explicit
    re-engagement is required, the same reasoning arm_and_engage_offboard
    already applies to arm/offboard-engage. See docs/parallelism.md §2.6.
    """
    wall_deadline = time.monotonic() + timeout_s
    next_publish = time.monotonic()
    next_reengage = 0.0  # re-engage immediately the first time it's needed

    while True:
        now = time.monotonic()

        if now >= next_publish:
            px4.publish_offboard_heartbeat(position=True)
            px4.publish_position_setpoint(x, y, z)
            next_publish = now + STREAM_PERIOD_S

        status = px4.latest['vehicle_status']
        if not _is_offboard(status) and now >= next_reengage:
            px4.engage_offboard_mode()
            next_reengage = now + RESEND_INTERVAL_S

        if is_reached():
            return

        if now > wall_deadline:
            raise HoldTimeout(
                f"timed out waiting to {description} within {timeout_s}s; "
                f"failing flags: {_failing_flags(px4)}")

        clock.pump()


def land_and_wait(node, px4: PX4Interface, clock: PX4Clock, *,
                   timeout_s: float = LAND_TIMEOUT_S) -> None:
    """Command a landing and block until PX4 confirms DISARMED. Once
    VEHICLE_CMD_NAV_LAND is accepted, PX4's own AUTO_LAND mode takes over and
    no longer needs offboard setpoints, so -- unlike arm_and_engage_offboard
    and hold_position_until -- this does not stream anything, matching the
    original hand-verified behaviour from M1's sim_watch.sh checks.
    """
    wall_deadline = time.monotonic() + timeout_s
    next_command = time.monotonic()

    while True:
        now = time.monotonic()

        status = px4.latest['vehicle_status']
        if not _is_armed(status):
            return

        if now >= next_command:
            px4.land()
            next_command = now + RESEND_INTERVAL_S

        if now > wall_deadline:
            raise LandTimeout(
                f"never confirmed DISARMED within {timeout_s}s after commanding "
                f"land; failing flags: {_failing_flags(px4)}")

        clock.pump()
