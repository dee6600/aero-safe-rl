"""PX4 <-> ROS 2 telemetry and command interface, shared by every node in
this project that talks to PX4.

PX4 uses NED coordinates (north-east-DOWN). Down is positive, so altitude
is a NEGATIVE number in vehicle_local_position.z / vehicle_odometry.
"5 metres up" is z = -5.0. Getting this backwards is the classic bug with
this stack -- if a vehicle "climbs" straight into the ground, check the sign
first.
"""

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    ActuatorMotors,
    ActuatorOutputs,
    BatteryStatus,
    EstimatorStatusFlags,
    FailsafeFlags,
    OffboardControlMode,
    SensorCombined,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)

# Matches PX4's uxrce_dds_client QoS -- see px4_ros_com's offboard_control.py,
# the reference example vendored in ros2_ws/src/px4_ros_com/.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# The 9 telemetry topics M2 requires we can read, with sensible values.
#
# NOTE: PX4 appends "_v<MESSAGE_VERSION>" to the DDS topic name for any
# message whose .msg file declares MESSAGE_VERSION > 0 (a scheme for
# non-breaking evolution of message definitions). VehicleStatus and
# BatteryStatus are both MESSAGE_VERSION=1 in this px4_msgs checkout, so
# their real topics are .../vehicle_status_v1 and .../battery_status_v1, NOT
# the unsuffixed names you'd guess from the uORB topic name or
# dds_topics.yaml. Found by checking `ros2 topic list` against actual
# output rather than assuming -- exactly the kind of thing M0 warned about
# ("verify actual values, not just that a topic exists").
TELEMETRY_TOPICS = {
    'vehicle_odometry': ('/fmu/out/vehicle_odometry', VehicleOdometry),
    'vehicle_attitude': ('/fmu/out/vehicle_attitude', VehicleAttitude),
    'sensor_combined': ('/fmu/out/sensor_combined', SensorCombined),
    'actuator_motors': ('/fmu/out/actuator_motors', ActuatorMotors),
    'actuator_outputs': ('/fmu/out/actuator_outputs', ActuatorOutputs),
    'vehicle_status': ('/fmu/out/vehicle_status_v1', VehicleStatus),
    'battery_status': ('/fmu/out/battery_status_v1', BatteryStatus),
    'failsafe_flags': ('/fmu/out/failsafe_flags', FailsafeFlags),
    'estimator_status_flags': ('/fmu/out/estimator_status_flags', EstimatorStatusFlags),
}


class PX4Interface:
    """Wires up every PX4 telemetry subscription and command publisher this
    project needs, on a given rclpy Node. Not a Node itself -- compose it
    into whichever node needs PX4 access, so every node uses the identical
    topic names, types, and QoS (no drift between nodes)."""

    def __init__(self, node: Node):
        self.node = node
        self.latest = {name: None for name in TELEMETRY_TOPICS}
        self._subs = []

        for name, (topic, msg_type) in TELEMETRY_TOPICS.items():
            sub = node.create_subscription(
                msg_type, topic,
                self._make_callback(name), PX4_QOS)
            self._subs.append(sub)

        self.offboard_control_mode_pub = node.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.trajectory_setpoint_pub = node.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.vehicle_command_pub = node.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

    def _make_callback(self, name):
        def _cb(msg):
            self.latest[name] = msg
        return _cb

    def _now_us(self) -> int:
        return int(self.node.get_clock().now().nanoseconds / 1000)

    def publish_offboard_heartbeat(self, position=True, velocity=False,
                                    acceleration=False, attitude=False,
                                    body_rate=False) -> None:
        """Must be sent continuously at >=20 Hz before AND during offboard
        mode, or PX4 drops out of it."""
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = acceleration
        msg.attitude = attitude
        msg.body_rate = body_rate
        msg.timestamp = self._now_us()
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float,
                                   yaw: float = 0.0) -> None:
        """z is NED -- negative is up. z=-5.0 means 5m above the origin."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self._now_us()
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command: int, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.vehicle_command_pub.publish(msg)

    def arm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def engage_offboard_mode(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
