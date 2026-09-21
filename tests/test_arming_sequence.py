"""Unit tests for arming_sequence.py (M2). No rclpy, no simulator: a fake
PX4Interface-like object and a real PX4Clock (already independently tested in
test_px4_clock.py) wired to a fake pump_fn that advances the fake vehicle's
state, so these exercise the real state-machine logic deterministically and
in milliseconds.
"""
import pytest

from px4_msgs.msg import VehicleStatus

from aero_bridge.arming_sequence import (
    ArmTimeout, HoldTimeout, LandTimeout, OffboardLost, OffboardRejected, PreflightFailed,
    _failing_flags, _is_armed, _is_offboard,
    arm_and_engage_offboard, hold_position_until, land_and_wait,
)
from aero_bridge.px4_clock import PX4Clock


def make_status(armed=False, offboard=False, preflight_ok=True) -> VehicleStatus:
    msg = VehicleStatus()
    msg.arming_state = (VehicleStatus.ARMING_STATE_ARMED if armed
                        else VehicleStatus.ARMING_STATE_DISARMED)
    msg.nav_state = (VehicleStatus.NAVIGATION_STATE_OFFBOARD if offboard
                     else VehicleStatus.NAVIGATION_STATE_ALTCTL)
    msg.pre_flight_checks_pass = preflight_ok
    return msg


class FakePX4:
    """Stands in for PX4Interface. `latest` is the same shape (a dict with
    'vehicle_status' / 'failsafe_flags' keys); commands are just counted so
    tests can assert on resend behaviour."""

    def __init__(self, status=None, flags=None):
        self.latest = {'vehicle_status': status, 'failsafe_flags': flags}
        self.heartbeats = 0
        self.setpoints = []
        self.arm_calls = 0
        self.engage_calls = 0
        self.land_calls = 0

    def publish_offboard_heartbeat(self, position=True):
        self.heartbeats += 1

    def publish_position_setpoint(self, x, y, z):
        self.setpoints.append((x, y, z))

    def arm(self):
        self.arm_calls += 1

    def engage_offboard_mode(self):
        self.engage_calls += 1

    def land(self):
        self.land_calls += 1


def make_clock(pump_fn):
    return PX4Clock(now_us_fn=lambda: 0, pump_fn=pump_fn, poll_interval_s=0.001)


# ------------------------------------------------------------------ helpers


def test_is_armed_and_is_offboard():
    assert _is_armed(make_status(armed=True)) is True
    assert _is_armed(make_status(armed=False)) is False
    assert _is_armed(None) is False
    assert _is_offboard(make_status(offboard=True)) is True
    assert _is_offboard(make_status(offboard=False)) is False
    assert _is_offboard(None) is False


def test_failing_flags_with_no_flags_received():
    px4 = FakePX4()
    assert _failing_flags(px4) == ['<no FailsafeFlags received yet>']


def test_failing_flags_reports_only_true_ones():
    from px4_msgs.msg import FailsafeFlags
    flags = FailsafeFlags()
    flags.offboard_control_signal_lost = True
    flags.local_position_invalid = False
    px4 = FakePX4(flags=flags)
    assert _failing_flags(px4) == ['offboard_control_signal_lost']


def test_failing_flags_none_true_is_explicit():
    from px4_msgs.msg import FailsafeFlags
    px4 = FakePX4(flags=FailsafeFlags())  # all fields default False
    assert _failing_flags(px4) == ['<none reported>']


# ------------------------------------------------------------- arm_and_engage


def test_arm_and_engage_offboard_succeeds_once_armed_and_offboard():
    px4 = FakePX4(status=make_status(armed=False, offboard=False))
    calls = [0]

    def pump(_t):
        calls[0] += 1
        if calls[0] >= 3:
            px4.latest['vehicle_status'] = make_status(armed=True, offboard=True)

    clock = make_clock(pump)
    arm_and_engage_offboard(None, px4, clock, takeoff_z=-5.0, timeout_s=1.0)
    assert px4.heartbeats > 0
    assert px4.setpoints[-1] == (0.0, 0.0, -5.0)


def test_arm_and_engage_offboard_issues_arm_and_engage_commands():
    """Commands are issued after WARMUP_S (0.75s) of wall-clock warmup, even
    if the vehicle never actually confirms armed+offboard -- verified via the
    resulting ArmTimeout rather than timing a success, since WARMUP_S is real
    wall-clock time this test must actually let elapse either way.
    """
    px4 = FakePX4(status=make_status())  # never becomes armed/offboard
    clock = make_clock(lambda t: None)
    with pytest.raises(ArmTimeout):
        arm_and_engage_offboard(None, px4, clock, takeoff_z=-5.0, timeout_s=1.0)
    assert px4.arm_calls >= 1
    assert px4.engage_calls >= 1


def test_arm_and_engage_offboard_raises_preflight_failed():
    px4 = FakePX4(status=make_status(armed=False, offboard=False, preflight_ok=False))
    clock = make_clock(lambda t: None)
    with pytest.raises(PreflightFailed):
        arm_and_engage_offboard(None, px4, clock, takeoff_z=-5.0, timeout_s=0.02)


def test_arm_and_engage_offboard_raises_arm_timeout_when_preflight_ok_but_never_armed():
    px4 = FakePX4(status=make_status(armed=False, offboard=False, preflight_ok=True))
    clock = make_clock(lambda t: None)
    with pytest.raises(ArmTimeout):
        arm_and_engage_offboard(None, px4, clock, takeoff_z=-5.0, timeout_s=0.02)


def test_arm_and_engage_offboard_raises_offboard_rejected_when_armed_but_not_offboard():
    px4 = FakePX4(status=make_status(armed=True, offboard=False, preflight_ok=True))
    clock = make_clock(lambda t: None)
    with pytest.raises(OffboardRejected):
        arm_and_engage_offboard(None, px4, clock, takeoff_z=-5.0, timeout_s=0.02)


def test_arm_and_engage_offboard_error_message_includes_failing_flags():
    from px4_msgs.msg import FailsafeFlags
    flags = FailsafeFlags()
    flags.local_position_invalid = True
    px4 = FakePX4(status=make_status(preflight_ok=False), flags=flags)
    clock = make_clock(lambda t: None)
    with pytest.raises(PreflightFailed, match="local_position_invalid"):
        arm_and_engage_offboard(None, px4, clock, takeoff_z=-5.0, timeout_s=0.02)


# ------------------------------------------------------------ hold_position


def test_hold_position_until_returns_when_condition_true():
    px4 = FakePX4(status=make_status(armed=True, offboard=True))
    clock = make_clock(lambda t: None)
    hold_position_until(None, px4, clock, x=0, y=0, z=-5.0,
                        is_reached=lambda: True, timeout_s=1.0, description="test")
    assert px4.setpoints[-1] == (0, 0, -5.0)


def test_hold_position_until_raises_hold_timeout():
    px4 = FakePX4(status=make_status(armed=True, offboard=True))
    clock = make_clock(lambda t: None)
    with pytest.raises(HoldTimeout, match="reach the sky"):
        hold_position_until(None, px4, clock, x=0, y=0, z=-5.0,
                            is_reached=lambda: False, timeout_s=0.02,
                            description="reach the sky")


def test_hold_position_until_reengages_offboard_when_lost():
    """The M2 fix: if nav_state leaves OFFBOARD mid-wait, hold_position_until
    must re-issue the offboard-engage command, not just keep streaming and
    hope. This is the test that would catch a regression removing the fix
    documented in docs/parallelism.md.

    Offboard never recovers in this fake (unlike a real re-engage, nothing
    here flips it back to True), so the deadline is hit while still outside
    OFFBOARD -- exactly the case M4 (schema v2) gave its own exception,
    OffboardLost, distinct from a stuck-but-still-offboard HoldTimeout. See
    test_hold_position_until_raises_hold_timeout for that other case.
    """
    px4 = FakePX4(status=make_status(armed=True, offboard=True))
    calls = [0]

    def pump(_t):
        calls[0] += 1
        if calls[0] == 5:
            # PX4 drops out of offboard mid-wait.
            px4.latest['vehicle_status'] = make_status(armed=True, offboard=False)

    clock = make_clock(pump)
    with pytest.raises(OffboardLost):
        hold_position_until(None, px4, clock, x=0, y=0, z=-5.0,
                            is_reached=lambda: False, timeout_s=0.05,
                            description="test")
    assert px4.engage_calls >= 1, "must re-issue engage_offboard_mode after losing offboard"


def test_hold_position_until_raises_hold_timeout_when_offboard_recovers_in_time():
    """The other half of the OffboardLost/HoldTimeout split: offboard drops
    mid-wait but comes back (as a real re-engage normally would) before the
    deadline -- the eventual timeout is then a genuine stuck hold, not an
    offboard-loss outcome, so it must still raise plain HoldTimeout."""
    px4 = FakePX4(status=make_status(armed=True, offboard=True))
    calls = [0]

    def pump(_t):
        calls[0] += 1
        if calls[0] == 5:
            px4.latest['vehicle_status'] = make_status(armed=True, offboard=False)
        elif calls[0] == 10:
            px4.latest['vehicle_status'] = make_status(armed=True, offboard=True)

    clock = make_clock(pump)
    with pytest.raises(HoldTimeout):
        hold_position_until(None, px4, clock, x=0, y=0, z=-5.0,
                            is_reached=lambda: False, timeout_s=0.05,
                            description="test")


def test_hold_position_until_does_not_reengage_while_still_offboard():
    px4 = FakePX4(status=make_status(armed=True, offboard=True))
    clock = make_clock(lambda t: None)
    hold_position_until(None, px4, clock, x=0, y=0, z=-5.0,
                        is_reached=lambda: True, timeout_s=1.0, description="test")
    assert px4.engage_calls == 0


# ----------------------------------------------------------------- land


def test_land_and_wait_returns_immediately_if_already_disarmed():
    px4 = FakePX4(status=make_status(armed=False))
    clock = make_clock(lambda t: None)
    land_and_wait(None, px4, clock, timeout_s=1.0)
    assert px4.land_calls == 0


def test_land_and_wait_commands_land_and_waits_for_disarm():
    px4 = FakePX4(status=make_status(armed=True))
    calls = [0]

    def pump(_t):
        calls[0] += 1
        if calls[0] >= 3:
            px4.latest['vehicle_status'] = make_status(armed=False)

    clock = make_clock(pump)
    land_and_wait(None, px4, clock, timeout_s=1.0)
    assert px4.land_calls >= 1


def test_land_and_wait_raises_land_timeout():
    px4 = FakePX4(status=make_status(armed=True))
    clock = make_clock(lambda t: None)
    with pytest.raises(LandTimeout):
        land_and_wait(None, px4, clock, timeout_s=0.02)
