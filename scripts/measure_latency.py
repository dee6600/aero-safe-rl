#!/usr/bin/env python3
"""Measure PX4 -> ROS 2 telemetry latency.

PX4 SITL's hrt_absolute_time() is CLOCK_MONOTONIC-based internally
(platforms/posix/src/px4/common/drv_hrt.cpp) -- but the `timestamp` field
you actually receive in a px4_msgs message over the ROS 2 bridge is NOT that
raw value. uxrce_dds_client resynchronizes it to the agent's wall clock
before transmission (see "synchronized with time offset ...us" in the PX4
log at startup -- that offset is ~current epoch seconds). Confirmed
empirically: comparing msg.timestamp against CLOCK_MONOTONIC first produced
a ~56-YEAR-off latency reading, which is exactly the Unix epoch gap you'd
expect from that mismatch. CLOCK_REALTIME (wall clock) is the correct basis.
rclpy's default node clock isn't used here either, for the same reason
python's os on Linux (`time.time()`) is more direct.

Usage: python measure_latency.py [--topic sensor_combined] [--seconds 15]
"""
import argparse
import statistics
import time

import rclpy
from rclpy.node import Node

from aero_bridge.px4_interface import PX4_QOS, TELEMETRY_TOPICS

WARMUP_SAMPLES = 10


class LatencyMeasurer(Node):
    def __init__(self, topic_key: str, duration_s: float):
        super().__init__('measure_latency')
        topic, msg_type = TELEMETRY_TOPICS[topic_key]
        self.duration_s = duration_s
        self.start_time = time.monotonic()
        self.samples_us = []
        self.n_seen = 0
        self.sub = self.create_subscription(msg_type, topic, self._cb, PX4_QOS)
        self.get_logger().info(f"Measuring latency on {topic} for {duration_s}s...")

    def _cb(self, msg) -> None:
        now_us = time.time() * 1e6
        latency_us = now_us - msg.timestamp
        self.n_seen += 1
        if self.n_seen > WARMUP_SAMPLES:
            self.samples_us.append(latency_us)
        if time.monotonic() - self.start_time > self.duration_s:
            # rclpy.shutdown() alone does not reliably break rclpy.spin()'s
            # loop from inside a callback -- see the same fix (and
            # explanation) in aero_bridge/test_flight.py. SystemExit
            # propagates through the executor and actually ends spin().
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='sensor_combined', choices=list(TELEMETRY_TOPICS))
    ap.add_argument('--seconds', type=float, default=15.0)
    args = ap.parse_args()

    rclpy.init()
    node = LatencyMeasurer(args.topic, args.seconds)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass

    samples = node.samples_us
    node.destroy_node()
    rclpy.shutdown()

    if len(samples) < 5:
        print(f"Too few samples ({len(samples)}) -- is the sim + MicroXRCEAgent running?")
        raise SystemExit(1)

    samples_sorted = sorted(samples)
    p95 = samples_sorted[int(len(samples_sorted) * 0.95)]

    print(f"\nTopic: {args.topic}")
    print(f"Samples: {len(samples)} (after discarding {WARMUP_SAMPLES} warmup)")
    print(f"Mean latency:   {statistics.mean(samples) / 1000:.3f} ms")
    print(f"Median latency: {statistics.median(samples) / 1000:.3f} ms")
    print(f"P95 latency:    {p95 / 1000:.3f} ms")
    print(f"Min / Max:      {min(samples) / 1000:.3f} / {max(samples) / 1000:.3f} ms")
    print(f"Stdev:          {statistics.pstdev(samples) / 1000:.3f} ms")


if __name__ == '__main__':
    main()
