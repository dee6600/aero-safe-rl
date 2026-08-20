"""Unit tests for PX4Interface. No PX4 or Gazebo needed -- these only need
rclpy itself, verifying our own pub/sub wiring and message construction
logic, using a second node in the same process to observe what
PX4Interface actually publishes.
"""
import time
import unittest

import rclpy
from rclpy.node import Node

from aero_bridge.px4_interface import PX4_QOS, TELEMETRY_TOPICS, PX4Interface
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand


def spin_until(node_list, predicate, timeout_s=3.0):
    """Spin the given nodes until predicate() is True or timeout elapses."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for n in node_list:
            rclpy.spin_once(n, timeout_sec=0.05)
        if predicate():
            return True
    return False


class TestPX4Interface(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_px4_interface_node')
        self.px4 = PX4Interface(self.node)
        self.observer = Node('test_observer_node')

    def tearDown(self):
        self.node.destroy_node()
        self.observer.destroy_node()

    def test_all_nine_telemetry_topics_present(self):
        expected = {
            'vehicle_odometry', 'vehicle_attitude', 'sensor_combined',
            'actuator_motors', 'actuator_outputs', 'vehicle_status',
            'battery_status', 'failsafe_flags', 'estimator_status_flags',
        }
        self.assertEqual(set(TELEMETRY_TOPICS.keys()), expected)

    def test_versioned_topic_names_are_correct(self):
        # Regression test: found by checking actual `ros2 topic list` output
        # against assumptions -- VehicleStatus and BatteryStatus are
        # MESSAGE_VERSION=1 in this px4_msgs checkout, so PX4 publishes them
        # as .../vehicle_status_v1 and .../battery_status_v1, not the
        # unsuffixed names. If a px4_msgs update changes this, this test
        # should be the thing that catches it, not a live sim run.
        topic, _ = TELEMETRY_TOPICS['vehicle_status']
        self.assertTrue(topic.endswith('_v1'), topic)
        topic, _ = TELEMETRY_TOPICS['battery_status']
        self.assertTrue(topic.endswith('_v1'), topic)
        # And the ones without a declared MESSAGE_VERSION should NOT be
        # versioned.
        for key in ('sensor_combined', 'actuator_outputs', 'failsafe_flags',
                    'estimator_status_flags', 'vehicle_odometry',
                    'actuator_motors'):
            topic, _ = TELEMETRY_TOPICS[key]
            self.assertFalse(topic.endswith('_v1'), topic)

    def test_construction_creates_a_subscription_per_topic(self):
        self.assertEqual(len(self.px4._subs), len(TELEMETRY_TOPICS))

    def test_latest_starts_as_none_for_every_topic(self):
        self.assertEqual(set(self.px4.latest.keys()), set(TELEMETRY_TOPICS.keys()))
        for value in self.px4.latest.values():
            self.assertIsNone(value)

    def test_arm_publishes_correct_vehicle_command(self):
        received = []
        self.observer.create_subscription(
            VehicleCommand, '/fmu/in/vehicle_command',
            lambda msg: received.append(msg), PX4_QOS)

        self.px4.arm()
        ok = spin_until([self.node, self.observer], lambda: len(received) > 0)

        self.assertTrue(ok, "never received the arm VehicleCommand")
        msg = received[0]
        self.assertEqual(msg.command, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual(msg.param1, 1.0)
        self.assertTrue(msg.from_external)

    def test_disarm_sends_param1_zero(self):
        received = []
        self.observer.create_subscription(
            VehicleCommand, '/fmu/in/vehicle_command',
            lambda msg: received.append(msg), PX4_QOS)

        self.px4.disarm()
        ok = spin_until([self.node, self.observer], lambda: len(received) > 0)

        self.assertTrue(ok, "never received the disarm VehicleCommand")
        self.assertEqual(received[0].param1, 0.0)

    def test_position_setpoint_preserves_ned_sign(self):
        # This is the exact bug class M2's docs warn about repeatedly: an
        # accidental sign flip on altitude. Assert the z we pass through is
        # exactly the z that gets published -- no silent negation anywhere
        # in between.
        received = []
        self.observer.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
            lambda msg: received.append(msg), PX4_QOS)

        self.px4.publish_position_setpoint(1.0, 2.0, -5.0)
        ok = spin_until([self.node, self.observer], lambda: len(received) > 0)

        self.assertTrue(ok, "never received the trajectory setpoint")
        msg = received[0]
        self.assertAlmostEqual(msg.position[0], 1.0, places=5)
        self.assertAlmostEqual(msg.position[1], 2.0, places=5)
        self.assertAlmostEqual(msg.position[2], -5.0, places=5)


if __name__ == '__main__':
    unittest.main()
