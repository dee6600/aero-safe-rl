"""Unit tests for TestFlight's state machine. No PX4 or Gazebo needed --
telemetry is faked by writing directly into px4.latest, and terminal-state
behavior is verified without waiting on real timeouts, by moving
state_entered_at/hover_start into the past instead of sleeping.
"""
import unittest

import rclpy
from px4_msgs.msg import VehicleOdometry, VehicleStatus

from aero_bridge.test_flight import TestFlight


def make_status(armed: bool, nav_state=None) -> VehicleStatus:
    msg = VehicleStatus()
    msg.arming_state = (VehicleStatus.ARMING_STATE_ARMED if armed
                         else VehicleStatus.ARMING_STATE_DISARMED)
    if nav_state is not None:
        msg.nav_state = nav_state
    return msg


def make_odometry(altitude_m: float) -> VehicleOdometry:
    msg = VehicleOdometry()
    msg.position = [0.0, 0.0, -altitude_m]  # NED: z is negative-up
    return msg


class TestFlightStateMachine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = TestFlight(hover_alt=5.0, hover_seconds=5.0)

    def tearDown(self):
        self.node.destroy_node()

    # --- regression test for the hang bug found during M2 development ---
    # rclpy.shutdown() alone did not reliably break rclpy.spin()'s loop when
    # called from inside a timer callback; every run left its process alive
    # indefinitely. The fix was to raise SystemExit instead. This test
    # exists so a future refactor can't silently reintroduce the old
    # pattern without a test failing.

    def test_succeed_raises_system_exit_zero(self):
        with self.assertRaises(SystemExit) as ctx:
            self.node._succeed()
        self.assertEqual(ctx.exception.code, 0)

    def test_fail_raises_system_exit_one(self):
        with self.assertRaises(SystemExit) as ctx:
            self.node._fail()
        self.assertEqual(ctx.exception.code, 1)

    # --- state machine progression ---

    def test_starts_in_stream_setpoints(self):
        self.assertEqual(self.node.state, 'STREAM_SETPOINTS')

    def test_streams_setpoints_before_arming(self):
        # Fewer than ARM_AFTER_SETPOINTS ticks: must not attempt to arm yet.
        for _ in range(5):
            self.node._tick()
        self.assertEqual(self.node.state, 'STREAM_SETPOINTS')

    def test_transitions_to_wait_armed_offboard_after_streaming(self):
        for _ in range(20):
            self.node._tick()
        self.assertEqual(self.node.state, 'WAIT_ARMED_OFFBOARD')

    def test_transitions_to_takeoff_once_armed_and_offboard(self):
        for _ in range(20):
            self.node._tick()
        self.assertEqual(self.node.state, 'WAIT_ARMED_OFFBOARD')

        self.node.px4.latest['vehicle_status'] = make_status(
            armed=True, nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.node._tick()
        self.assertEqual(self.node.state, 'TAKEOFF')

    def test_wait_armed_offboard_times_out_and_fails(self):
        for _ in range(20):
            self.node._tick()
        self.assertEqual(self.node.state, 'WAIT_ARMED_OFFBOARD')

        # Never arms -- simulate the timeout having already elapsed instead
        # of actually waiting 30s in a unit test.
        self.node.state_entered_at -= 999
        with self.assertRaises(SystemExit) as ctx:
            self.node._tick()
        self.assertEqual(ctx.exception.code, 1)

    def test_transitions_to_hover_once_altitude_reached(self):
        for _ in range(20):
            self.node._tick()
        self.node.px4.latest['vehicle_status'] = make_status(
            armed=True, nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.node._tick()
        self.assertEqual(self.node.state, 'TAKEOFF')

        self.node.px4.latest['vehicle_odometry'] = make_odometry(altitude_m=4.5)  # >= 85% of 5m
        self.node._tick()
        self.assertEqual(self.node.state, 'HOVER')

    def test_hover_then_land_then_disarm_succeeds(self):
        for _ in range(20):
            self.node._tick()
        self.node.px4.latest['vehicle_status'] = make_status(
            armed=True, nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.node._tick()
        self.node.px4.latest['vehicle_odometry'] = make_odometry(altitude_m=5.0)
        self.node._tick()
        self.assertEqual(self.node.state, 'HOVER')

        # Skip the real 5s hover wait.
        self.node.hover_start -= 999
        self.node._tick()
        self.assertEqual(self.node.state, 'LAND')

        # Skip the 1s post-land-command delay.
        self.node.state_entered_at -= 999
        self.node._tick()
        self.assertEqual(self.node.state, 'WAIT_LANDED')

        # Simulate PX4 confirming disarm.
        self.node.px4.latest['vehicle_status'] = make_status(armed=False)
        with self.assertRaises(SystemExit) as ctx:
            self.node._tick()
        self.assertEqual(ctx.exception.code, 0)

    def test_altitude_reads_ned_sign_correctly(self):
        # z=-5.0 (NED, up) must read as +5.0m altitude, not -5.0m. Getting
        # this backwards is called out repeatedly in this project's own
        # docs as the classic bug with this stack.
        self.node.px4.latest['vehicle_odometry'] = make_odometry(altitude_m=5.0)
        self.assertAlmostEqual(self.node._current_altitude(), 5.0, places=5)


if __name__ == '__main__':
    unittest.main()
