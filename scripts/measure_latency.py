#!/usr/bin/env python3
"""Measure PX4 -> ROS 2 telemetry latency, on any instance.

PX4 SITL's hrt_absolute_time() is CLOCK_MONOTONIC-based internally
(platforms/posix/src/px4/common/drv_hrt.cpp) -- but the `timestamp` field
you actually receive in a px4_msgs message over the ROS 2 bridge is NOT that
raw value. uxrce_dds_client resynchronizes it to the agent's wall clock
before transmission (see "synchronized with time offset ...us" in the PX4
log at startup). Confirmed independently and in more depth during M2 (see
simulation/sim_clock.py's module docstring): px4_msgs timestamps track wall
clock almost exactly regardless of PX4_SIM_SPEED_FACTOR, which is exactly
why they cannot be used as a simulated-time source (GzSimClock exists for
that) -- but it is also exactly what makes CLOCK_REALTIME the right basis
for a LATENCY measurement like this one, since both sides of the comparison
are already in the same (wall-clock) domain.

Usage: python measure_latency.py --instance 1 [--topic sensor_combined] [--seconds 15]
"""
import argparse
import os
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

WARMUP_SAMPLES = 10


def _run(instance: int, topic_key: str, duration_s: float) -> None:
    import rclpy
    from rclpy.node import Node

    from simulation.instance_spec import InstanceSpec
    from aero_bridge.px4_interface import PX4_QOS, TELEMETRY_TOPICS

    spec = InstanceSpec.for_instance(instance)
    suffix, msg_type = TELEMETRY_TOPICS[topic_key]
    topic = spec.topic(suffix)

    samples_us = []
    n_seen = 0
    start_time = time.monotonic()

    node = Node('measure_latency')

    def _cb(msg) -> None:
        nonlocal n_seen
        now_us = time.time() * 1e6
        latency_us = now_us - msg.timestamp
        n_seen += 1
        if n_seen > WARMUP_SAMPLES:
            samples_us.append(latency_us)
        if time.monotonic() - start_time > duration_s:
            # rclpy.shutdown() alone does not reliably break rclpy.spin()'s
            # loop from inside a callback -- see the same fix (and
            # explanation) in aero_bridge/test_flight.py. SystemExit
            # propagates through the executor and actually ends spin().
            raise SystemExit(0)

    node.create_subscription(msg_type, topic, _cb, PX4_QOS)
    node.get_logger().info(f"Measuring latency on {topic} (instance {instance}) for {duration_s}s...")

    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()

    if len(samples_us) < 5:
        print(f"Too few samples ({len(samples_us)}) -- is instance {instance}'s "
              f"sim + MicroXRCEAgent running, and ROS_DOMAIN_ID={spec.ros_domain_id}?")
        raise SystemExit(1)

    samples_sorted = sorted(samples_us)
    p95 = samples_sorted[int(len(samples_sorted) * 0.95)]

    print(f"\nInstance: {instance}   Topic: {topic}")
    print(f"Samples: {len(samples_us)} (after discarding {WARMUP_SAMPLES} warmup)")
    print(f"Mean latency:   {statistics.mean(samples_us) / 1000:.3f} ms")
    print(f"Median latency: {statistics.median(samples_us) / 1000:.3f} ms")
    print(f"P95 latency:    {p95 / 1000:.3f} ms")
    print(f"Min / Max:      {min(samples_us) / 1000:.3f} / {max(samples_us) / 1000:.3f} ms")
    print(f"Stdev:          {statistics.pstdev(samples_us) / 1000:.3f} ms")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--instance', type=int, default=0)
    ap.add_argument('--topic', default='sensor_combined')
    ap.add_argument('--seconds', type=float, default=15.0)
    args = ap.parse_args()

    # Must be set before rclpy.init() -- the DDS layer reads it at that
    # point and it cannot be changed afterward. See test_flight.py for the
    # same requirement and reasoning (D9: every instance's domain is its
    # instance number).
    from simulation.instance_spec import InstanceSpec
    os.environ['ROS_DOMAIN_ID'] = str(InstanceSpec.for_instance(args.instance).ros_domain_id)

    import rclpy
    rclpy.init()
    try:
        _run(args.instance, args.topic, args.seconds)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
