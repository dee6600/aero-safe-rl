"""PX4 <-> ROS 2 telemetry and command interface, shared by every node in
this project that talks to PX4.

PX4 uses NED coordinates (north-east-DOWN). Down is positive, so altitude
is a NEGATIVE number in vehicle_local_position.z / vehicle_odometry.
"5 metres up" is z = -5.0. Getting this backwards is the classic bug with
this stack -- if a vehicle "climbs" straight into the ground, check the sign
first.

Every topic name and every VehicleCommand's target_system is derived from an
InstanceSpec (simulation/instance_spec.py), never hardcoded. Two silent bugs
shipped here before this was true: target_system fixed at 1 (dropped by
Commander.cpp:746 on every instance except 0, with no error), and topic names
hardcoded to /fmu/out/... (instance N>0 actually publishes on
/px4_N/fmu/out/..., so the subscription succeeded and received nothing,
forever). See docs/parallelism.md §2.1-2.2.
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

from simulation.instance_spec import InstanceSpec

# Matches PX4's uxrce_dds_client QoS -- see px4_ros_com's offboard_control.py,
# the reference example vendored in ros2_ws/src/px4_ros_com/. A mismatch here
# (e.g. RELIABLE instead of BEST_EFFORT) gives a subscription that silently
# never fires -- the same symptom as a namespace mistake, from a different
# cause, so it is worth remembering this exists as a second place to check.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# The 9 telemetry topics M2 requires we can read, with sensible values.
# Values are the topic SUFFIX passed to InstanceSpec.topic() -- never a full
# path. Building a full "/fmu/out/..." string anywhere outside this file (or
# InstanceSpec itself) is the bug this project has already hit once.
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
    'vehicle_odometry': ('vehicle_odometry', VehicleOdometry),
    'vehicle_attitude': ('vehicle_attitude', VehicleAttitude),
    'sensor_combined': ('sensor_combined', SensorCombined),
    'actuator_motors': ('actuator_motors', ActuatorMotors),
    'actuator_outputs': ('actuator_outputs', ActuatorOutputs),
    'vehicle_status': ('vehicle_status_v1', VehicleStatus),
    'battery_status': ('battery_status_v1', BatteryStatus),
    'failsafe_flags': ('failsafe_flags', FailsafeFlags),
    'estimator_status_flags': ('estimator_status_flags', EstimatorStatusFlags),
}

# Same treatment for the 3 command topics we publish to.
COMMAND_TOPICS = {
    'offboard_control_mode': ('offboard_control_mode', OffboardControlMode),
    'trajectory_setpoint': ('trajectory_setpoint', TrajectorySetpoint),
    'vehicle_command': ('vehicle_command', VehicleCommand),
}


class PX4Interface:
    """Wires up every PX4 telemetry subscription and command publisher this
    project needs, on a given rclpy Node, for one specific worker. Not a Node
    itself -- compose it into whichever node needs PX4 access, so every node
    uses the identical topic names, types, QoS and target_system for a given
    instance (no drift between nodes, and no instance-0 special case)."""

    def __init__(self, node: Node, spec: InstanceSpec):
        self.node = node
        self.spec = spec
        self.latest = {name: None for name in TELEMETRY_TOPICS}
        # Freshest message timestamp seen across ANY telemetry topic, in
        # microseconds. Despite the name, this is NOT simulated time: PX4's
        # uxrce_dds_client resynchronizes every published timestamp to the
        # agent's WALL clock before it reaches ROS 2 (confirmed empirically
        # during M2 -- see simulation/sim_clock.py's module docstring for the
        # measurement). Useful only as a link-freshness indicator ("has
        # anything arrived recently?"). PX4Clock's sim-time source is
        # GzSimClock (simulation/sim_clock.py), never this field.
        self.last_timestamp_us = None
        self._subs = []

        for name, (suffix, msg_type) in TELEMETRY_TOPICS.items():
            sub = node.create_subscription(
                msg_type, spec.topic(suffix, 'out'),
                self._make_callback(name), PX4_QOS)
            self._subs.append(sub)

        self.offboard_control_mode_pub = node.create_publisher(
            OffboardControlMode, spec.topic('offboard_control_mode', 'in'), PX4_QOS)
        self.trajectory_setpoint_pub = node.create_publisher(
            TrajectorySetpoint, spec.topic('trajectory_setpoint', 'in'), PX4_QOS)
        self.vehicle_command_pub = node.create_publisher(
            VehicleCommand, spec.topic('vehicle_command', 'in'), PX4_QOS)

    def _make_callback(self, name):
        def _cb(msg):
            self.latest[name] = msg
            ts = getattr(msg, 'timestamp', None)
            if ts and (self.last_timestamp_us is None or ts > self.last_timestamp_us):
                self.last_timestamp_us = ts
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

    def build_vehicle_command(self, command: int, **params) -> VehicleCommand:
        """Constructs (but does not publish) a VehicleCommand. Split out from
        publish_vehicle_command so field construction -- in particular
        target_system -- can be unit tested without a live DDS round trip."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        # MAV_SYS_ID = instance + 1 (PX4 rcS). Commander.cpp:746 drops any
        # command whose target_system matches neither 0 nor this value, with
        # no error -- a hardcoded 1 here is the single bug that made every
        # instance above 0 silently ignore arm/offboard/land.
        msg.target_system = self.spec.mav_sys_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        return msg

    def publish_vehicle_command(self, command: int, **params) -> None:
        self.vehicle_command_pub.publish(self.build_vehicle_command(command, **params))

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
