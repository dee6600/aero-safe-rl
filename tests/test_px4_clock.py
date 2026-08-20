"""Unit tests for PX4Clock (M2). No rclpy, no simulator -- now_us_fn and
pump_fn are fakes, so these run in milliseconds and exercise the sim-time
semantics directly rather than via a real, slow, speed-factor-dependent wait.
"""
import time

import pytest

from aero_bridge.px4_clock import ClockTimeout, PX4Clock


def test_now_us_reflects_the_injected_source():
    clock = PX4Clock(now_us_fn=lambda: 42_000_000)
    assert clock.now_us() == 42_000_000
    assert clock.now_s() == pytest.approx(42.0)


def test_now_us_is_none_before_any_message():
    clock = PX4Clock(now_us_fn=lambda: None)
    assert clock.now_us() is None
    assert clock.now_s() is None


def test_clock_uses_message_time():
    """sleep_sim(1.0) must return once the (fake) message stream reports 1.0s
    of SIM time has passed -- regardless of how much wall time that took.
    The fake pump here advances sim time by a fixed 50ms per call, simulating
    a fast stream (well above the 20Hz PX4 publishes at); the test asserts
    real wall time elapsed is tiny, proving sleep_sim did not fall back to
    waiting a real 1.0 wall-second.
    """
    sim_us = [0]

    def now_us_fn():
        return sim_us[0]

    def pump_fn(_timeout_s):
        sim_us[0] += 50_000  # 50ms of sim time "arrives" per pump call

    clock = PX4Clock(now_us_fn=now_us_fn, pump_fn=pump_fn, poll_interval_s=0.001)

    wall_start = time.monotonic()
    clock.sleep_sim(1.0)
    wall_elapsed = time.monotonic() - wall_start

    assert sim_us[0] >= 1_000_000
    assert wall_elapsed < 0.5, (
        f"sleep_sim took {wall_elapsed:.3f}s of real time for a fake stream "
        f"that reports sim time passing in ~20 pump calls -- looks like it "
        f"fell back to wall-clock waiting"
    )


def test_sleep_sim_measures_from_the_current_moment_not_zero():
    """sleep_sim(1.0) called when sim time is already at 10s must wait until
    11s, not until 1s (which would be in the past).
    """
    sim_us = [10_000_000]

    def pump_fn(_timeout_s):
        sim_us[0] += 100_000

    clock = PX4Clock(now_us_fn=lambda: sim_us[0], pump_fn=pump_fn, poll_interval_s=0.001)
    clock.sleep_sim(1.0)
    assert sim_us[0] >= 11_000_000


def test_clock_deadline_raises():
    """No messages arriving must raise ClockTimeout, not hang forever."""
    clock = PX4Clock(now_us_fn=lambda: None, pump_fn=lambda t: None,
                     wall_margin_s=0.05, poll_interval_s=0.01)
    with pytest.raises(ClockTimeout):
        clock.sleep_sim(0.05)


def test_deadline_raises_if_sim_time_stalls_partway():
    """Sim time arrives initially, then stops advancing -- must still raise
    rather than waiting indefinitely for the remainder.
    """
    sim_us = [0]
    calls = [0]

    def pump_fn(_timeout_s):
        calls[0] += 1
        if calls[0] <= 3:
            sim_us[0] += 100_000  # advances a bit, then stalls forever

    clock = PX4Clock(now_us_fn=lambda: sim_us[0], pump_fn=pump_fn,
                     wall_margin_s=0.05, poll_interval_s=0.01)
    with pytest.raises(ClockTimeout):
        clock.sleep_sim(1.0)  # needs 1,000,000 us; only ever reaches 300,000


def test_negative_duration_rejected():
    clock = PX4Clock(now_us_fn=lambda: 0)
    with pytest.raises(ValueError):
        clock.sleep_sim(-1.0)


def test_zero_duration_returns_immediately_once_a_timestamp_exists():
    clock = PX4Clock(now_us_fn=lambda: 5_000_000, pump_fn=lambda t: None)
    clock.sleep_sim(0.0)  # must not raise or hang


def test_pump_delegates_to_the_injected_function():
    calls = []
    clock = PX4Clock(now_us_fn=lambda: 0, pump_fn=lambda t: calls.append(t))
    clock.pump(0.3)
    assert calls == [0.3]


def test_pump_default_timeout_matches_poll_interval():
    calls = []
    clock = PX4Clock(now_us_fn=lambda: 0, pump_fn=lambda t: calls.append(t),
                     poll_interval_s=0.02)
    clock.pump()
    assert calls == [0.02]
