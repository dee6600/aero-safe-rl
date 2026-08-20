"""M2 flight validation node: arm -> offboard -> takeoff to 5m -> hover ->
land -> disarm, entirely over ROS 2 (no MAVLink, no manual steps).

NED reminder (see px4_interface.py docstring): z is negative-up. Takeoff
altitude of 5m is z = -5.0.

Usage: ros2 run aero_bridge test_flight [--ros-args -p hover_alt:=5.0 -p hover_seconds:=5.0]
"""
import sys

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus

from aero_bridge.px4_interface import PX4Interface

SETPOINT_RATE_HZ = 20.0  # must be >=20Hz or PX4 drops out of offboard
ARM_AFTER_SETPOINTS = 15  # ~0.75s of setpoints streamed before engaging offboard+arm
RETRY_EVERY_N_TICKS = 20  # resend arm+offboard roughly once per second while waiting
ALT_REACHED_FRACTION = 0.85
STATE_TIMEOUT_S = 30.0


class TestFlight(Node):
    def __init__(self, hover_alt: float, hover_seconds: float):
        super().__init__('test_flight')
        self.px4 = PX4Interface(self)
        self.hover_alt = hover_alt
        self.hover_seconds = hover_seconds
        self.takeoff_z = -abs(hover_alt)

        self.state = 'STREAM_SETPOINTS'
        self.state_entered_at = self._now()
        self.setpoint_count = 0
        self.hover_start = None

        self.timer = self.create_timer(1.0 / SETPOINT_RATE_HZ, self._tick)
        self.get_logger().info(
            f"test_flight starting: hover_alt={hover_alt}m hover_seconds={hover_seconds}s")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _elapsed_in_state(self) -> float:
        return self._now() - self.state_entered_at

    def _goto(self, state: str) -> None:
        self.get_logger().info(f"state: {self.state} -> {state}")
        self.state = state
        self.state_entered_at = self._now()

    def _current_altitude(self):
        """Returns altitude in metres (positive up), or None if no odometry yet."""
        odom = self.px4.latest['vehicle_odometry']
        if odom is None:
            return None
        return -odom.position[2]

    def _nav_state(self):
        status = self.px4.latest['vehicle_status']
        return status.nav_state if status is not None else None

    def _armed(self) -> bool:
        status = self.px4.latest['vehicle_status']
        return status is not None and status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _tick(self) -> None:
        # Offboard control mode must be streamed continuously, in every state
        # from before arming through to landing.
        self.px4.publish_offboard_heartbeat(position=True)

        if self.state == 'STREAM_SETPOINTS':
            self.px4.publish_position_setpoint(0.0, 0.0, self.takeoff_z)
            self.setpoint_count += 1
            if self.setpoint_count >= ARM_AFTER_SETPOINTS:
                self.px4.engage_offboard_mode()
                self.px4.arm()
                self._goto('WAIT_ARMED_OFFBOARD')

        elif self.state == 'WAIT_ARMED_OFFBOARD':
            self.px4.publish_position_setpoint(0.0, 0.0, self.takeoff_z)
            if self._armed() and self._nav_state() == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._goto('TAKEOFF')
            elif self._elapsed_in_state() > STATE_TIMEOUT_S:
                self.get_logger().error("timed out waiting for arm + offboard mode")
                self._fail()
            elif int(self._elapsed_in_state() * SETPOINT_RATE_HZ) % RETRY_EVERY_N_TICKS == 0:
                # arm()/engage_offboard_mode() are sent over BEST_EFFORT QoS
                # with no retry -- a single lost message (DDS discovery not
                # yet settled, a CPU-contention hiccup from e.g. the Gazebo
                # GUI competing for cycles) leaves the vehicle stuck here
                # forever. Confirmed by testing: this state's one-shot
                # version worked reliably headless but intermittently hung
                # with the GUI on. Resend periodically instead of once.
                self.px4.engage_offboard_mode()
                self.px4.arm()

        elif self.state == 'TAKEOFF':
            self.px4.publish_position_setpoint(0.0, 0.0, self.takeoff_z)
            alt = self._current_altitude()
            if alt is not None and alt >= self.hover_alt * ALT_REACHED_FRACTION:
                self.hover_start = self._now()
                self._goto('HOVER')
            elif self._elapsed_in_state() > STATE_TIMEOUT_S:
                self.get_logger().error(f"timed out reaching takeoff altitude (last alt={alt})")
                self._fail()

        elif self.state == 'HOVER':
            self.px4.publish_position_setpoint(0.0, 0.0, self.takeoff_z)
            if self._now() - self.hover_start >= self.hover_seconds:
                self._goto('LAND')

        elif self.state == 'LAND':
            self.px4.land()
            if self._elapsed_in_state() > 1.0:
                self._goto('WAIT_LANDED')

        elif self.state == 'WAIT_LANDED':
            if not self._armed():
                self.get_logger().info("landed and disarmed")
                self._succeed()
            elif self._elapsed_in_state() > STATE_TIMEOUT_S:
                self.get_logger().error("timed out waiting for disarm after land")
                self._fail()

    def _succeed(self) -> None:
        # rclpy.shutdown() alone does NOT reliably break rclpy.spin()'s loop
        # when called from inside a timer callback -- confirmed by testing:
        # every run left its process alive indefinitely, still spinning,
        # until killed externally. SystemExit propagates through rclpy's
        # executor (it isn't caught by a plain `except Exception`) and
        # actually terminates the process -- this is the same pattern the
        # vendored px4_ros_com reference example uses (its timer_callback
        # calls exit(0) directly), which is why it was chosen here too.
        raise SystemExit(0)

    def _fail(self) -> None:
        raise SystemExit(1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TestFlight(hover_alt=5.0, hover_seconds=5.0)
    code = 1
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = e.code if isinstance(e.code, int) else 0
    except KeyboardInterrupt:
        code = 1
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(code)


if __name__ == '__main__':
    main()
