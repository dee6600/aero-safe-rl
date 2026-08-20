#!/usr/bin/env python3
"""M2 flight check: arm -> engage offboard -> takeoff -> hover -> land ->
disarm, entirely over ROS 2, no MAVLink. The flight-transition logic lives in
arming_sequence.py; this file is orchestration only.

Works identically on any instance number -- pass --instance N to fly a
worker started with `scripts/sim_start.sh -i N`. That is the whole point of
M2 (see milestones.md): code that only works on instance 0 passes every test
written before this and fails silently once a second worker exists.

Usage: ros2 run aero_bridge test_flight --instance 1 --hover-alt 5.0
"""
import argparse
import os
import sys

ALT_REACHED_FRACTION = 0.85
DEFAULT_HOVER_ALT_M = 5.0
DEFAULT_HOVER_SECONDS = 5.0
TAKEOFF_TIMEOUT_S = 30.0


def fly(instance: int, hover_alt: float, hover_seconds: float) -> int:
    # Deferred: rclpy must be import(ed)/init()ed only after ROS_DOMAIN_ID is
    # set in main(), or this process joins the wrong DDS domain and never
    # sees the worker at all.
    import rclpy
    from rclpy.node import Node

    from simulation.instance_spec import InstanceSpec
    from simulation.sim_clock import GzSimClock
    from aero_bridge.px4_interface import PX4Interface
    from aero_bridge.px4_clock import PX4Clock
    from aero_bridge.arming_sequence import (
        FlightSequenceError, arm_and_engage_offboard, hold_position_until, land_and_wait,
    )

    spec = InstanceSpec.for_instance(instance)
    node = Node('test_flight')
    px4 = PX4Interface(node, spec)
    # Sim time comes from Gazebo directly, NOT from px4_msgs timestamps --
    # those are silently resynced to wall clock by uxrce_dds_client and are
    # useless for this. See simulation/sim_clock.py's module docstring for
    # the measurement that found this.
    gz_clock = GzSimClock(world=spec.world, gz_partition=spec.gz_partition)
    clock = PX4Clock(now_us_fn=gz_clock.now_us,
                      pump_fn=lambda t: rclpy.spin_once(node, timeout_sec=t))
    takeoff_z = -abs(hover_alt)
    log = node.get_logger()

    try:
        arm_and_engage_offboard(node, px4, clock, takeoff_z=takeoff_z)
        log.info("armed, offboard engaged")

        def altitude_reached() -> bool:
            odom = px4.latest['vehicle_odometry']
            return odom is not None and -odom.position[2] >= hover_alt * ALT_REACHED_FRACTION

        hold_position_until(node, px4, clock, x=0.0, y=0.0, z=takeoff_z,
                             is_reached=altitude_reached, timeout_s=TAKEOFF_TIMEOUT_S,
                             description=f"reach {hover_alt}m")
        log.info(f"reached takeoff altitude ({hover_alt}m)")

        hover_start_us = clock.now_us()

        def hovered_long_enough() -> bool:
            now_us = clock.now_us()
            return now_us is not None and now_us - hover_start_us >= hover_seconds * 1e6

        hold_position_until(node, px4, clock, x=0.0, y=0.0, z=takeoff_z,
                             is_reached=hovered_long_enough,
                             timeout_s=hover_seconds + 15.0,
                             description=f"hover {hover_seconds}s")
        log.info("hover complete")

        land_and_wait(node, px4, clock)
        log.info("landed and disarmed")
        return 0
    except FlightSequenceError as exc:
        log.error(str(exc))
        return 1
    finally:
        gz_clock.close()
        node.destroy_node()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--instance', type=int, default=0)
    ap.add_argument('--hover-alt', type=float, default=DEFAULT_HOVER_ALT_M)
    ap.add_argument('--hover-seconds', type=float, default=DEFAULT_HOVER_SECONDS)
    # ros2 run passes --ros-args ... through; argparse would otherwise choke
    # on the unrecognized flag.
    args, _ = ap.parse_known_args(argv)

    # ROS_DOMAIN_ID must match the worker's before rclpy (and the DDS layer
    # underneath it) initialises -- it cannot be changed afterwards. Every
    # instance's domain is its instance number (D9), so this is the one env
    # var a client MUST set itself rather than relying on the caller to have
    # exported it. Topic naming/target_system are handled separately by
    # InstanceSpec + PX4Interface and don't need an env var.
    from simulation.instance_spec import InstanceSpec
    os.environ['ROS_DOMAIN_ID'] = str(InstanceSpec.for_instance(args.instance).ros_domain_id)

    import rclpy
    rclpy.init()
    try:
        code = fly(args.instance, args.hover_alt, args.hover_seconds)
    finally:
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
