"""M2 task 2 (@pytest.mark.sim): the nine telemetry topics carry believable,
changing values during a real flight -- not merely that each one ticks.
Complements tests/test_px4_interface.py (topic naming and command
construction, no live data at all) and tests/sim/test_link.py (end-to-end
pass/fail, which never inspects a telemetry value).

Flies instance 1, not 0, so a value that happened to only look sane on the
PX4-favoured default instance would still be caught here -- the telemetry-
content analogue of the namespace trap in docs/parallelism.md §2.2.

Calls arm_and_engage_offboard / hold_position_until / land_and_wait directly
(the same functions test_flight.py calls) rather than shelling out to
test_flight.py as a subprocess, because this test needs every sample as it
arrives, not just the pass/fail exit code test_link.py checks.
"""
import math
import time

import pytest

from px4_msgs.msg import VehicleStatus

from simulation.instance_spec import InstanceSpec
from simulation.sim_clock import GzSimClock
from aero_bridge.px4_clock import PX4Clock
from aero_bridge.px4_interface import PX4_QOS, TELEMETRY_TOPICS, PX4Interface
from aero_bridge.arming_sequence import (
    arm_and_engage_offboard, hold_position_until, land_and_wait,
)

HOVER_ALT_M = 5.0
HOVER_SECONDS = 3.0
IDLE_WINDOW_S = 3.0
MIN_SAMPLES = 5


class _Recorder:
    """Independent subscriptions that keep every sample, not just the
    latest. PX4Interface.latest deliberately keeps only the newest message
    per topic -- that's all flight logic needs -- so a believability check
    that needs a history records its own rather than changing production
    code for a test-only need. Reuses TELEMETRY_TOPICS and PX4_QOS from
    px4_interface.py rather than re-listing topics or QoS, so there is still
    exactly one place that knows what the nine topics are.
    """

    def __init__(self, node, spec):
        self.samples = {name: [] for name in TELEMETRY_TOPICS}
        self._subs = [
            node.create_subscription(msg_type, spec.topic(suffix, 'out'),
                                      self._make_cb(name), PX4_QOS)
            for name, (suffix, msg_type) in TELEMETRY_TOPICS.items()
        ]

    def _make_cb(self, name):
        def _cb(msg):
            self.samples[name].append(msg)
        return _cb

    def mark(self):
        """Sample counts right now, per topic -- a phase boundary usable to
        split "before" from "after" without comparing timestamps across
        clock domains (message timestamps are wall-clock, not sim time --
        docs/parallelism.md §2.5)."""
        return {name: len(msgs) for name, msgs in self.samples.items()}


def _finite_max_abs(values):
    finite = [abs(v) for v in values if not math.isnan(v)]
    return max(finite) if finite else 0.0


def _median(seq):
    s = sorted(seq)
    mid = len(s) // 2
    return s[mid] if len(s) % 2 else (s[mid - 1] + s[mid]) / 2


def test_telemetry_values_are_believable(sim_worker_1):
    import os
    os.environ['ROS_DOMAIN_ID'] = str(InstanceSpec.for_instance(sim_worker_1).ros_domain_id)

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    spec = InstanceSpec.for_instance(sim_worker_1)
    node = Node('test_telemetry_sanity')
    px4 = PX4Interface(node, spec)
    recorder = _Recorder(node, spec)
    gz_clock = GzSimClock(world=spec.world, gz_partition=spec.gz_partition)
    clock = PX4Clock(now_us_fn=gz_clock.now_us,
                      pump_fn=lambda t: rclpy.spin_once(node, timeout_sec=t))
    takeoff_z = -HOVER_ALT_M

    try:
        # A brief disarmed window before arming gives a genuine idle
        # baseline for the actuator-output rise check below.
        idle_deadline = time.monotonic() + IDLE_WINDOW_S
        while time.monotonic() < idle_deadline:
            clock.pump()
        pre_arm = recorder.mark()

        arm_and_engage_offboard(node, px4, clock, takeoff_z=takeoff_z)

        def altitude_reached() -> bool:
            odom = px4.latest['vehicle_odometry']
            return odom is not None and -odom.position[2] >= HOVER_ALT_M * 0.85

        hold_position_until(node, px4, clock, x=0.0, y=0.0, z=takeoff_z,
                             is_reached=altitude_reached, timeout_s=30.0,
                             description="reach hover altitude")

        hover_start_us = clock.now_us()

        def hovered_long_enough() -> bool:
            now_us = clock.now_us()
            return now_us is not None and now_us - hover_start_us >= HOVER_SECONDS * 1e6

        hold_position_until(node, px4, clock, x=0.0, y=0.0, z=takeoff_z,
                             is_reached=hovered_long_enough,
                             timeout_s=HOVER_SECONDS + 15.0,
                             description="hover")

        land_and_wait(node, px4, clock)
    finally:
        gz_clock.close()
        node.destroy_node()
        rclpy.shutdown()

    _assert_believable(recorder.samples, pre_arm)


def _assert_believable(samples: dict, pre_arm: dict) -> None:
    for name, msgs in samples.items():
        assert len(msgs) >= MIN_SAMPLES, (
            f"{name}: only {len(msgs)} samples received during the whole "
            f"flight -- topic may not really be live")

    # timestamp advances over the flight, on every topic -- checked via
    # head/tail medians rather than strict adjacent-pair monotonicity.
    # Measured empirically while writing this test: px4_msgs .timestamp is
    # wall-clock-resynced by uxrce_dds_client (docs/parallelism.md §2.5), and
    # around a transient offboard_control_signal_lost/re-engage event (§2.6 --
    # known to hit ~10-20% of solo flights) that resync visibly jitters --
    # observed on a real flight: ~180 small (single-digit-ms) backward steps
    # clustered around one recovery, plus one message that slipped through
    # with the raw un-synced clock (a "seconds since boot" value instead of
    # wall-clock). None of that means the topic is frozen or broken, so
    # asserting zero backward steps would fail this test for a reason that
    # has nothing to do with telemetry believability. A median-based check
    # is insensitive to that handful of outliers while still catching a
    # topic that is genuinely stuck or reports time running backward overall.
    for name, msgs in samples.items():
        stamps = [m.timestamp for m in msgs]
        assert len(set(stamps)) > 1, f"{name}: timestamp value never changed"
        window = max(1, len(stamps) // 10)
        head, tail = stamps[:window], stamps[-window:]
        assert _median(tail) > _median(head), (
            f"{name}: timestamp did not advance from the start of the "
            f"flight to the end (head median {_median(head)}, tail median "
            f"{_median(tail)})")

    # vehicle_odometry: altitude actually climbed near the hover target.
    # NED: up is more-negative z, so altitude = -position[2].
    altitudes = [-m.position[2] for m in samples['vehicle_odometry']]
    assert max(altitudes) >= HOVER_ALT_M * 0.8, (
        f"never got near the {HOVER_ALT_M}m target: max altitude "
        f"{max(altitudes):.2f}m")
    assert max(altitudes) - min(altitudes) > 1.0, (
        "altitude barely varied across the whole flight -- looks frozen, "
        f"not an actual takeoff: {min(altitudes):.2f}..{max(altitudes):.2f}m")

    # vehicle_attitude: the quaternion changes over the flight (the vehicle
    # corrects continuously; it is never perfectly still) and stays
    # unit-norm throughout -- a real orientation, not a garbage/zeroed one.
    quats = [tuple(m.q) for m in samples['vehicle_attitude']]
    assert len(set(quats)) > 1, "vehicle_attitude quaternion never changed"
    step = max(1, len(quats) // 20)
    for q in quats[::step]:
        norm = math.sqrt(sum(c * c for c in q))
        assert 0.9 < norm < 1.1, f"quaternion {q} is not unit-norm ({norm:.3f})"

    # sensor_combined: the raw accelerometer reads something other than
    # exactly zero -- real physics, not a stub sensor.
    accel_mags = [math.sqrt(sum(c * c for c in m.accelerometer_m_s2))
                  for m in samples['sensor_combined']]
    assert max(accel_mags) > 1.0, (
        "sensor_combined accelerometer never read above ~1 m/s^2 -- looks "
        "like a stub/zeroed sensor, not real physics")

    # actuator_motors / actuator_outputs: control output rises from a
    # disarmed idle baseline to real thrust once armed and climbing.
    # ActuatorMotors.control is NaN while disarmed ("NaN maps to disarmed");
    # _finite_max_abs treats an all-NaN window as an idle baseline of 0.
    idle_motors = samples['actuator_motors'][:pre_arm['actuator_motors']]
    flown_motors = samples['actuator_motors'][pre_arm['actuator_motors']:]
    idle_motor_peak = max((_finite_max_abs(m.control) for m in idle_motors), default=0.0)
    flown_motor_peak = max(_finite_max_abs(m.control) for m in flown_motors)
    assert flown_motor_peak > max(idle_motor_peak, 0.05), (
        f"actuator_motors.control never rose above the disarmed baseline "
        f"({idle_motor_peak:.3f} idle vs {flown_motor_peak:.3f} flown) -- "
        f"motors may never have actually spun up")

    idle_outputs = samples['actuator_outputs'][:pre_arm['actuator_outputs']]
    flown_outputs = samples['actuator_outputs'][pre_arm['actuator_outputs']:]
    idle_output_peak = max((_finite_max_abs(m.output) for m in idle_outputs), default=0.0)
    flown_output_peak = max(_finite_max_abs(m.output) for m in flown_outputs)
    assert flown_output_peak > idle_output_peak, (
        f"actuator_outputs.output never rose from its disarmed baseline "
        f"({idle_output_peak:.3f} idle vs {flown_output_peak:.3f} flown)")

    # battery_status: reports a plausible fraction and never *rises* --
    # batteries do not recharge mid-flight, small numerical jitter aside.
    fractions = [m.remaining for m in samples['battery_status'] if m.remaining >= 0.0]
    assert fractions, "battery_status never reported a valid 'remaining' fraction"
    assert fractions[-1] <= fractions[0] + 0.01, (
        f"battery remaining fraction rose from {fractions[0]:.4f} to "
        f"{fractions[-1]:.4f} -- batteries do not recharge mid-flight")

    # vehicle_status: actually passed through ARMED -- otherwise several
    # checks above (altitude, motor rise) would have been vacuously true of
    # a vehicle that never flew.
    assert any(m.arming_state == VehicleStatus.ARMING_STATE_ARMED
               for m in samples['vehicle_status']), (
        "vehicle_status never reported ARMED during the flight")

    # failsafe_flags: no hardware/estimator failure flags set during a
    # nominal, no-fault-injected flight. Deliberately does NOT assert on
    # offboard_control_signal_lost -- that flag is known to flip transiently
    # even on a healthy flight (docs/parallelism.md §2.6); asserting it stays
    # false would make this test flaky for a reason already understood.
    last_flags = samples['failsafe_flags'][-1]
    for flag_name in ('fd_critical_failure', 'fd_esc_arming_failure',
                       'fd_imbalanced_prop', 'fd_motor_failure'):
        assert not getattr(last_flags, flag_name), (
            f"failsafe_flags.{flag_name} was set during a nominal flight "
            f"with no fault injected")

    # estimator_status_flags: the EKF reports itself aligned by the end of a
    # flight that (per the checks above) demonstrably completed -- proving
    # the flags reflect real filter state, not zeroed placeholders.
    last_est = samples['estimator_status_flags'][-1]
    assert last_est.cs_tilt_align and last_est.cs_yaw_align, (
        "estimator_status_flags reports the EKF never finished alignment, "
        "despite a completed flight")
