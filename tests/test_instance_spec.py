"""Tests for worker identity (M1b).

These pin the behaviour that PX4 v1.17.0 actually has, not what it looks like
it has from the outside. Several of them exist specifically to fail loudly if a
future PX4 bump changes a convention we depend on -- a failing unit test is a
far cheaper way to discover that than a training run whose workers publish
nothing.

No simulator, no ROS: this whole file runs in milliseconds.
"""

import json

import pytest

from simulation.instance_spec import (
    MAX_ROS_DOMAIN_ID,
    SPEC_VERSION,
    InstanceRuntime,
    InstanceSpec,
    InstanceSpecError,
    write_instance_file,
)

# Enough instances to catch collisions without making the test slow.
INSTANCES = list(range(8))


def test_identity_is_pure():
    """Same instance number must always give the same identity.

    Anything non-deterministic here -- a free-port search, a timestamp, a random
    domain -- shows up as two workers racing for the same resource under M4.
    """
    a = InstanceSpec.for_instance(3, speed_factor=4.0)
    b = InstanceSpec.for_instance(3, speed_factor=4.0)
    assert a == b


@pytest.mark.parametrize(
    "field",
    [
        "xrce_port",
        "ros_domain_id",
        "topic_ns",
        "gz_partition",
        "mav_sys_id",
        "model_name",
    ],
)
def test_no_field_collides_across_instances(field):
    """Every per-worker resource must be unique across workers.

    One shared value between two workers is enough to cross-wire them, and the
    symptom is silence rather than an error.
    """
    values = [getattr(InstanceSpec.for_instance(i), field) for i in INSTANCES]
    assert len(set(values)) == len(values), f"{field} collides: {values}"


@pytest.mark.parametrize("instance", INSTANCES)
def test_mav_sys_id_is_instance_plus_one(instance):
    """Pins PX4's rcS behaviour: `param set MAV_SYS_ID $((px4_instance+1))`.

    Commander drops any VehicleCommand whose target_system matches neither 0 nor
    this value, so getting it wrong means arm/offboard/land are ignored with no
    error anywhere. If a PX4 upgrade changes the convention, this test is where
    we want to find out.
    """
    assert InstanceSpec.for_instance(instance).mav_sys_id == instance + 1


@pytest.mark.parametrize("instance", INSTANCES)
def test_namespace_is_uniform(instance):
    """Instance 0 must be namespaced like every other instance.

    PX4 itself applies `-n px4_<N>` only when N != 0, so out of the box instance
    0 publishes bare /fmu/out/... and instance 1 publishes /px4_1/fmu/out/....
    That asymmetry is exactly what lets code pass every single-instance test and
    then fail silently the moment it is scaled up. We override it so there is
    one code path.
    """
    spec = InstanceSpec.for_instance(instance)
    assert spec.topic_ns == f"px4_{instance}"
    assert spec.px4_env()["PX4_UXRCE_DDS_NS"] == f"px4_{instance}"


def test_namespace_override_is_passed_to_px4():
    """The uniform namespace is only real if PX4 is actually told about it."""
    env = InstanceSpec.for_instance(0).px4_env()
    assert env["PX4_UXRCE_DDS_NS"] == "px4_0"
    assert env["ROS_DOMAIN_ID"] == "0"


def test_spec_roundtrips_json(tmp_path):
    """A spec written to disk and read back must be identical.

    The handshake file is how the shell launcher and the Python layer agree on
    identity; a lossy round trip reintroduces the two-sources-of-truth problem
    the file exists to remove.
    """
    spec = InstanceSpec.for_instance(
        2, speed_factor=8.0, spawn_pose=(1.0, 2.0, 0.0, 0.0, 0.0, 1.57)
    )
    path = write_instance_file(
        tmp_path / "instance_2.json",
        spec,
        InstanceRuntime(pid_px4=123, pid_gz=456, pid_agent=789),
    )
    assert InstanceSpec.from_file(path) == spec

    payload = json.loads(path.read_text())
    assert payload["runtime"]["pid_px4"] == 123
    assert payload["spec"]["spec_version"] == SPEC_VERSION


def test_stale_spec_version_is_rejected(tmp_path):
    """Mixing spec versions across running workers must fail loudly, not silently."""
    spec = InstanceSpec.for_instance(0)
    path = write_instance_file(tmp_path / "instance_0.json", spec)
    payload = json.loads(path.read_text())
    payload["spec"]["spec_version"] = "999"
    path.write_text(json.dumps(payload))

    with pytest.raises(InstanceSpecError, match="spec_version"):
        InstanceSpec.from_file(path)


# --------------------------------------------------------------------- topics


@pytest.mark.parametrize("instance", [0, 1, 3])
def test_topic_names_follow_namespace(instance):
    """Topic names are built from the namespace, never hardcoded.

    This is the check that would have caught the /fmu/out/... bug in
    px4_interface.py: a subscription to a topic nobody publishes is not an
    error in ROS 2, so it fails as silence.
    """
    spec = InstanceSpec.for_instance(instance)
    assert spec.topic("vehicle_odometry") == f"/px4_{instance}/fmu/out/vehicle_odometry"
    assert spec.topic("trajectory_setpoint", "in") == (
        f"/px4_{instance}/fmu/in/trajectory_setpoint"
    )


def test_versioned_message_suffix_is_preserved():
    """PX4 appends _v<N> to bumped messages; the helper must not mangle it."""
    spec = InstanceSpec.for_instance(0)
    assert spec.topic("vehicle_status_v1") == "/px4_0/fmu/out/vehicle_status_v1"


def test_bad_topic_direction_rejected():
    with pytest.raises(InstanceSpecError):
        InstanceSpec.for_instance(0).topic("vehicle_odometry", "sideways")


# ------------------------------------------------------------------ topology


def test_one_drone_per_world():
    """D7: every instance gets its own Gazebo partition -- no shared worlds."""
    assert InstanceSpec.for_instance(0).gz_partition == "aero_0"
    assert InstanceSpec.for_instance(1).gz_partition == "aero_1"
    assert InstanceSpec.for_instance(0).gz_partition != InstanceSpec.for_instance(1).gz_partition


def test_default_spawn_pose_is_origin():
    """Each worker owns its world, so they can all spawn at the same place.

    That deliberately removes a per-instance special case: a mission defined in
    local coordinates is then identical for every worker.
    """
    assert InstanceSpec.for_instance(5).spawn_pose == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------- validation


@pytest.mark.parametrize("bad", [-1, MAX_ROS_DOMAIN_ID + 1, True, 1.5])
def test_invalid_instance_rejected(bad):
    with pytest.raises(InstanceSpecError):
        InstanceSpec.for_instance(bad)


def test_invalid_speed_factor_rejected():
    with pytest.raises(InstanceSpecError):
        InstanceSpec.for_instance(0, speed_factor=0)


def test_short_spawn_pose_rejected():
    with pytest.raises(InstanceSpecError):
        InstanceSpec.for_instance(0, spawn_pose=(1.0, 2.0, 3.0))


def test_spec_is_immutable():
    """Identity must not drift after construction."""
    spec = InstanceSpec.for_instance(0)
    with pytest.raises(Exception):
        spec.instance = 4  # type: ignore[misc]


# ----------------------------------------------------------------- env block


def test_px4_env_is_complete():
    """Every variable the launcher must export, in one place.

    GZ_PARTITION appears here *and* must be set on the Gazebo server process --
    missing it on either end makes the instance silently join a neighbour's
    world (docs/parallelism.md §2.3).
    """
    env = InstanceSpec.for_instance(1, speed_factor=4.0).px4_env()
    assert env["PX4_SIM_MODEL"] == "gz_x500"
    assert env["PX4_GZ_STANDALONE"] == "1"
    assert env["PX4_UXRCE_DDS_PORT"] == "8889"
    assert env["PX4_SIM_SPEED_FACTOR"] == "4"
    assert env["GZ_PARTITION"] == "aero_1"
    assert env["PX4_GZ_MODEL_POSE"] == "0,0,0,0,0,0"
