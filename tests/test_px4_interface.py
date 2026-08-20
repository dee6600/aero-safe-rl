"""Unit tests for PX4Interface (M2). No simulator, no DDS round trip.

Topic names are asserted via rclpy's own `.topic_name` on the Publisher/
Subscription objects PX4Interface creates -- resolved at construction time,
with no spinning or network activity needed. Command field construction is
asserted directly on the message `build_vehicle_command` returns, again with
no publish/subscribe round trip. Both approaches are deliberately immune to
DDS discovery timing, which is a real source of flakiness the more obvious
publish-and-observe style would otherwise introduce.

These two tests are the ones that would have caught M2's two known bugs
(docs/parallelism.md §2.1-2.2) had they existed before the bugs shipped:
  - test_topic_names_follow_namespace
  - test_target_system_matches_instance
"""
import pytest

rclpy = pytest.importorskip('rclpy', reason="needs ROS 2 sourced (scripts/activate.sh)")

from rclpy.node import Node  # noqa: E402
from px4_msgs.msg import VehicleCommand  # noqa: E402

from simulation.instance_spec import InstanceSpec  # noqa: E402
from aero_bridge.px4_interface import (  # noqa: E402
    COMMAND_TOPICS, TELEMETRY_TOPICS, PX4Interface,
)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_interface(instance: int):
    spec = InstanceSpec.for_instance(instance)
    node = Node(f'test_px4_interface_{instance}')
    px4 = PX4Interface(node, spec)
    return node, px4, spec


@pytest.fixture
def px4_instance0():
    node, px4, spec = make_interface(0)
    yield px4, spec
    node.destroy_node()


@pytest.fixture
def px4_instance3():
    node, px4, spec = make_interface(3)
    yield px4, spec
    node.destroy_node()


# --------------------------------------------------------------- topic names


@pytest.mark.parametrize('instance', [0, 3])
def test_topic_names_follow_namespace(instance):
    """Every one of the 9 subscriptions and 3 publishers resolves to the
    namespaced name -- including instance 0, which PX4 itself does NOT
    namespace by default (docs/parallelism.md §2.2). A hardcoded
    '/fmu/out/...' string would pass this for instance 0 and silently fail
    it for instance 3, which is exactly the bug this test exists to catch.
    """
    node, px4, spec = make_interface(instance)
    try:
        expected_ns = f'/px4_{instance}/fmu/'
        sub_topics = {sub.topic_name for sub in px4._subs}
        assert len(sub_topics) == len(TELEMETRY_TOPICS)
        for topic in sub_topics:
            assert topic.startswith(expected_ns + 'out/'), topic

        pub_topics = {
            px4.offboard_control_mode_pub.topic_name,
            px4.trajectory_setpoint_pub.topic_name,
            px4.vehicle_command_pub.topic_name,
        }
        assert len(pub_topics) == len(COMMAND_TOPICS)
        for topic in pub_topics:
            assert topic.startswith(expected_ns + 'in/'), topic
    finally:
        node.destroy_node()


def test_versioned_topic_names_are_correct(px4_instance0):
    # Regression: VehicleStatus and BatteryStatus are MESSAGE_VERSION=1 in
    # this px4_msgs checkout, so PX4 publishes them as .../vehicle_status_v1
    # and .../battery_status_v1, not the unsuffixed name the uORB topic name
    # would suggest. Found by checking `ros2 topic list` against assumption.
    px4, spec = px4_instance0
    subs_by_topic = {sub.topic_name: sub for sub in px4._subs}
    assert spec.topic('vehicle_status_v1') in subs_by_topic
    assert spec.topic('battery_status_v1') in subs_by_topic
    # And topics with no declared MESSAGE_VERSION must NOT be suffixed.
    for unversioned in ('sensor_combined', 'actuator_outputs', 'failsafe_flags',
                        'estimator_status_flags', 'vehicle_odometry', 'actuator_motors'):
        assert spec.topic(unversioned) in subs_by_topic


def test_all_nine_telemetry_topics_present(px4_instance0):
    expected = {
        'vehicle_odometry', 'vehicle_attitude', 'sensor_combined',
        'actuator_motors', 'actuator_outputs', 'vehicle_status',
        'battery_status', 'failsafe_flags', 'estimator_status_flags',
    }
    assert set(TELEMETRY_TOPICS.keys()) == expected


def test_no_topic_string_is_hardcoded_outside_the_spec():
    """Every subscription/publisher topic must equal exactly what
    InstanceSpec.topic() would build -- proving PX4Interface never
    constructs a topic string itself.
    """
    node, px4, spec = make_interface(2)
    try:
        for name, (suffix, _msg_type) in TELEMETRY_TOPICS.items():
            sub = next(s for s in px4._subs
                       if s.topic_name == spec.topic(suffix, 'out'))
            assert sub.topic_name == spec.topic(suffix, 'out')
        assert px4.offboard_control_mode_pub.topic_name == spec.topic('offboard_control_mode', 'in')
        assert px4.trajectory_setpoint_pub.topic_name == spec.topic('trajectory_setpoint', 'in')
        assert px4.vehicle_command_pub.topic_name == spec.topic('vehicle_command', 'in')
    finally:
        node.destroy_node()


# ---------------------------------------------------------- command building


@pytest.mark.parametrize('instance,expected_sys_id', [(0, 1), (3, 4), (7, 8)])
def test_target_system_matches_instance(instance, expected_sys_id):
    """MAV_SYS_ID = instance + 1 (PX4 rcS). Commander.cpp:746 silently drops
    any command whose target_system matches neither 0 nor this value -- a
    hardcoded 1 here made arm/offboard/land no-ops on every instance above 0,
    with no error anywhere. This is the test that catches it.
    """
    node, px4, _spec = make_interface(instance)
    try:
        msg = px4.build_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
        assert msg.target_system == expected_sys_id
    finally:
        node.destroy_node()


def test_command_message_fields(px4_instance0):
    px4, _spec = px4_instance0

    arm = px4.build_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    assert arm.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
    assert arm.param1 == 1.0
    assert arm.from_external is True

    disarm = px4.build_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
    assert disarm.param1 == 0.0

    offboard = px4.build_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    assert offboard.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE
    assert offboard.param1 == 1.0 and offboard.param2 == 6.0

    land = px4.build_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    assert land.command == VehicleCommand.VEHICLE_CMD_NAV_LAND


def test_arm_disarm_engage_land_publish_expected_commands(px4_instance0):
    """The convenience methods (arm/disarm/engage_offboard_mode/land) must
    publish exactly what build_vehicle_command would build -- no drift
    between the two.
    """
    px4, _spec = px4_instance0
    captured = []
    px4.vehicle_command_pub.publish = captured.append  # no DDS involved

    px4.arm()
    px4.disarm()
    px4.engage_offboard_mode()
    px4.land()

    assert [m.command for m in captured] == [
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        VehicleCommand.VEHICLE_CMD_NAV_LAND,
    ]
    assert captured[0].param1 == 1.0   # arm
    assert captured[1].param1 == 0.0   # disarm
    assert (captured[2].param1, captured[2].param2) == (1.0, 6.0)  # offboard


def test_position_setpoint_preserves_ned_sign(px4_instance0):
    """The exact bug class M2's docs warn about repeatedly: an accidental
    sign flip on altitude. Assert the z passed through is the z published --
    no silent negation anywhere in between.
    """
    px4, _spec = px4_instance0
    captured = []
    px4.trajectory_setpoint_pub.publish = captured.append

    px4.publish_position_setpoint(1.0, 2.0, -5.0)

    assert len(captured) == 1
    assert captured[0].position[0] == pytest.approx(1.0)
    assert captured[0].position[1] == pytest.approx(2.0)
    assert captured[0].position[2] == pytest.approx(-5.0)


# --------------------------------------------------------------- last_timestamp_us


def test_last_timestamp_us_starts_none(px4_instance0):
    px4, _spec = px4_instance0
    assert px4.last_timestamp_us is None


def test_last_timestamp_us_tracks_the_freshest_message(px4_instance0):
    """PX4Clock reads this field; it must reflect the newest timestamp seen
    across ANY telemetry topic, not just one specific one.
    """
    px4, _spec = px4_instance0
    from px4_msgs.msg import VehicleOdometry, VehicleAttitude

    odom = VehicleOdometry()
    odom.timestamp = 1_000_000
    px4._make_callback('vehicle_odometry')(odom)
    assert px4.last_timestamp_us == 1_000_000

    older = VehicleAttitude()
    older.timestamp = 500_000
    px4._make_callback('vehicle_attitude')(older)
    assert px4.last_timestamp_us == 1_000_000  # unchanged -- older message

    newer = VehicleAttitude()
    newer.timestamp = 2_000_000
    px4._make_callback('vehicle_attitude')(newer)
    assert px4.last_timestamp_us == 2_000_000
