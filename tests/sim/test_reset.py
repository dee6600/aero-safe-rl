"""M3 task 5's sim-marked gate: the reset ladder against a real worker.

Flies to altitude, lands (the normal end-of-episode state every M3 mission
reaches), then exercises soft, medium and hard reset in turn on the SAME
worker, checking each leaves position/velocity/arming clean -- the
"test_reset_returns_clean_state" the milestone asks for, run once per tier
since each tier's own code path is worth confirming, not just the fastest
one.
"""
import time

import pytest
import rclpy
from rclpy.node import Node

from aero_bridge.arming_sequence import arm_and_engage_offboard, land_and_wait
from aero_bridge.px4_clock import PX4Clock
from aero_bridge.px4_interface import PX4Interface
from aero_bridge.reset import ResetTimeout, ResetUnsafe, hard_reset, medium_reset, soft_reset
from px4_msgs.msg import VehicleStatus
from simulation.instance_spec import InstanceSpec
from simulation.sim_clock import GzSimClock

POSITION_TOLERANCE_M = 0.5
VELOCITY_TOLERANCE_M_S = 0.3


class _Rig:
    """Builds/rebuilds the ROS-side objects for one instance -- needed because
    hard_reset() invalidates them (module docstring), while soft/medium do not."""

    def __init__(self, instance: int):
        self.spec = InstanceSpec.for_instance(instance)
        self.node = Node(f'test_reset_{instance}')
        self.px4 = PX4Interface(self.node, self.spec)
        self.gz_clock = GzSimClock(world=self.spec.world, gz_partition=self.spec.gz_partition)
        self.clock = PX4Clock(now_us_fn=self.gz_clock.now_us,
                               pump_fn=lambda t: rclpy.spin_once(self.node, timeout_sec=t))

    def close(self):
        self.gz_clock.close()
        self.node.destroy_node()

    def fly_and_land(self):
        arm_and_engage_offboard(self.node, self.px4, self.clock, takeoff_z=-3.0)

        def reached():
            odom = self.px4.latest['vehicle_odometry']
            return odom is not None and -odom.position[2] >= 2.5
        _hold_briefly(self.node, self.px4, self.clock, reached)
        land_and_wait(self.node, self.px4, self.clock)


def _hold_briefly(node, px4, clock, is_reached, timeout_s=30.0):
    from aero_bridge.arming_sequence import hold_position_until
    hold_position_until(node, px4, clock, x=0.0, y=0.0, z=-3.0,
                         is_reached=is_reached, timeout_s=timeout_s, description="reach altitude")


def _assert_clean_state(rig: _Rig):
    odom = rig.px4.latest['vehicle_odometry']
    status = rig.px4.latest['vehicle_status']
    assert odom is not None and status is not None
    target = rig.spec.spawn_pose[:3]
    pos_err = sum((a - b) ** 2 for a, b in zip(odom.position, target)) ** 0.5
    speed = sum(v * v for v in odom.velocity) ** 0.5
    assert pos_err <= POSITION_TOLERANCE_M, f"position error {pos_err}m after reset"
    assert speed <= VELOCITY_TOLERANCE_M_S, f"speed {speed}m/s after reset"
    assert status.arming_state == VehicleStatus.ARMING_STATE_DISARMED


@pytest.fixture
def rig(sim_worker):
    rclpy.init()
    r = _Rig(sim_worker)
    yield r
    r.close()
    rclpy.shutdown()


def test_soft_reset_returns_clean_state(rig):
    rig.fly_and_land()
    result = soft_reset(rig.node, rig.px4, rig.clock, rig.spec)
    assert result.tier == "soft"
    _assert_clean_state(rig)


def test_soft_reset_refuses_while_armed(rig):
    arm_and_engage_offboard(rig.node, rig.px4, rig.clock, takeoff_z=-3.0)
    with pytest.raises(ResetUnsafe):
        soft_reset(rig.node, rig.px4, rig.clock, rig.spec)
    land_and_wait(rig.node, rig.px4, rig.clock)  # leave the worker clean for teardown


def test_medium_reset_returns_clean_state_from_fresh_worker(rig):
    """medium_reset from a PX4 that has never armed this session -- the one
    case reset.py's module docstring documents as reliably recovering
    pre_flight_checks_pass (measured ~0.5s). Does NOT fly first: a live
    measurement while writing reset.py found that after a real arm+flight,
    pre_flight_checks_pass does not recover this way at all (tested to 75s
    wall) -- a genuine PX4-SITL limitation, not this project's bug, and not
    exercised by this test (see test_medium_reset_after_flight_is_unreliable
    below, which documents rather than hides that gap)."""
    result = medium_reset(rig.node, rig.px4, rig.clock, rig.spec)
    assert result.tier == "medium"
    _assert_clean_state(rig)

    # PX4's own process survived (module docstring) -- the same objects
    # should still be able to fly a fresh mission afterward.
    arm_and_engage_offboard(rig.node, rig.px4, rig.clock, takeoff_z=-3.0)
    land_and_wait(rig.node, rig.px4, rig.clock)


def test_medium_reset_after_flight_is_unreliable(rig):
    """Documents, rather than hides, the gap described in reset.py's module
    docstring: medium_reset() after a real flight raises ResetTimeout in
    this stack (PX4 v1.17.0 SITL + Gazebo Harmonic) rather than recovering.
    If a future PX4/Gazebo version fixes this, this test starts failing --
    that is the point: it should be revisited and medium_reset() promoted
    back to a real fallback, not left silently broken."""
    rig.fly_and_land()
    with pytest.raises(ResetTimeout):
        medium_reset(rig.node, rig.px4, rig.clock, rig.spec, timeout_s=20.0)
    # Leave the worker itself clean via the tier that IS proven reliable from
    # any state; the `rig` fixture's own teardown then destroys the (now
    # stale, since hard_reset just killed and replaced the old processes)
    # node/clock objects, which is safe -- they are local client-side
    # handles that do not require the remote process to still be alive.
    hard_reset(rig.spec)


def test_hard_reset_returns_clean_state(sim_worker):
    """Does not use the `rig` fixture: hard_reset kills and restarts the
    worker's processes, so the ROS-side objects (and the rclpy context
    itself) must be fully torn down and rebuilt around it, not just once at
    fixture scope -- exactly the asymmetry the reset.py module docstring
    describes for this tier."""
    rclpy.init()
    before = _Rig(sim_worker)
    try:
        before.fly_and_land()
        spec = before.spec
    finally:
        before.close()
        rclpy.shutdown()

    result = hard_reset(spec)
    assert result.tier == "hard"

    rclpy.init()
    fresh = _Rig(spec.instance)
    try:
        deadline = time.monotonic() + 10.0
        while fresh.px4.latest['vehicle_status'] is None and time.monotonic() < deadline:
            fresh.clock.pump(0.1)
        _assert_clean_state(fresh)
    finally:
        fresh.close()
        rclpy.shutdown()
