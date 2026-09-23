"""M8 task 2: experiments.metrics.classify_outcome on hand-built flights."""
import math

import numpy as np
import pytest

from experiments.metrics import Outcome, classify_outcome, first_ground_contact
from rl.policies.base_policy import load_outcome_spec

SPEC = load_outcome_spec()


def _flight(alt, vz=None, tilt_deg=None):
    """Step columns for an altitude profile (metres up). vz is NED (+down)."""
    alt = np.asarray(alt, dtype=float)
    n = len(alt)
    vz = np.zeros(n) if vz is None else np.asarray(vz, dtype=float)
    tilt = np.zeros(n) if tilt_deg is None else np.radians(np.asarray(tilt_deg, dtype=float))
    return dict(pos_z=-alt, vel_z=vz, roll_rad=tilt, pitch_rad=np.zeros(n))


TAKEOFF = [0.0, 0.1, 1.5, 3.0, 5.0, 5.0, 5.0]


def _landing(speed):
    """Hover, then descend at `speed` m/s to the ground and stay there."""
    alt = TAKEOFF + [4.0, 2.0, 0.5, 0.1, 0.0, 0.0]
    vz = [0.0] * len(TAKEOFF) + [speed] * 4 + [0.0, 0.0]
    return _flight(alt, vz)


def test_spec_values_are_the_confirmed_ones():
    assert SPEC.crash_touchdown_speed_m_s == 2.0
    assert SPEC.crash_tilt_deg == 60.0


def test_completed_gentle_landing_is_success():
    r = classify_outcome("completed", _landing(0.7))
    assert r.outcome == Outcome.MISSION_SUCCESS
    assert r.touchdown_speed_m_s == pytest.approx(0.7)


def test_completed_but_hard_touchdown_is_a_crash():
    assert classify_outcome("completed", _landing(2.5)).outcome == Outcome.CRASH


def test_limit_is_exclusive():
    assert classify_outcome("recovery_landed", _landing(2.0)).outcome == Outcome.SAFE_LANDING
    assert classify_outcome("recovery_landed", _landing(2.01)).outcome == Outcome.CRASH


def test_commanded_or_uncommanded_gentle_touchdown_is_safe_landing():
    assert classify_outcome("recovery_landed", _landing(1.0)).outcome == Outcome.SAFE_LANDING
    assert classify_outcome("ground_contact", _landing(1.5)).outcome == Outcome.SAFE_LANDING


def test_fall_is_a_crash_whatever_the_termination_reason():
    for reason in ("ground_contact", "hold_timeout", "recovery_landed"):
        assert classify_outcome(reason, _landing(4.7)).outcome == Outcome.CRASH


def test_touchdown_speed_uses_the_tick_before_contact():
    """At 10 Hz the contact sample can already read ~0; the tick before it
    carries the impact speed."""
    alt = TAKEOFF + [2.0, 0.6, 0.0]
    vz = [0.0] * len(TAKEOFF) + [3.0, 3.4, 0.0]
    r = classify_outcome("ground_contact", _flight(alt, vz))
    assert r.touchdown_speed_m_s == pytest.approx(3.4)
    assert r.outcome == Outcome.CRASH


def test_tilt_over_limit_is_a_crash_even_airborne():
    tilt = [0.0] * len(TAKEOFF)
    tilt[-1] = 61.0
    r = classify_outcome("episode_timeout", _flight(TAKEOFF, tilt_deg=tilt))
    assert r.outcome == Outcome.CRASH and r.max_tilt_deg == pytest.approx(61.0)


def test_aggressive_but_healthy_tilt_is_not_a_crash():
    tilt = [0.0] * len(TAKEOFF)
    tilt[-2] = 45.0
    assert classify_outcome("completed", _flight(TAKEOFF, tilt_deg=tilt)).outcome == Outcome.MISSION_SUCCESS


def test_tilt_before_takeoff_is_ignored():
    tilt = [80.0] + [0.0] * (len(TAKEOFF) - 1)
    assert classify_outcome("completed", _flight(TAKEOFF, tilt_deg=tilt)).outcome == Outcome.MISSION_SUCCESS


def test_airborne_without_contact_and_unfinished_is_incomplete():
    r = classify_outcome("offboard_lost", _flight(TAKEOFF))
    assert r.outcome == Outcome.INCOMPLETE and math.isnan(r.touchdown_speed_m_s)


def test_never_airborne():
    r = classify_outcome("preflight_failed", _flight([0.0, 0.0, 0.1]))
    assert r.outcome == Outcome.INCOMPLETE and math.isnan(r.max_tilt_deg)


def test_sensitivity_override():
    f = _landing(1.8)
    assert classify_outcome("recovery_landed", f).outcome == Outcome.SAFE_LANDING
    assert classify_outcome("recovery_landed", f, crash_touchdown_speed_m_s=1.5).outcome == Outcome.CRASH


def test_first_ground_contact_ignores_the_takeoff_roll():
    airborne, contact = first_ground_contact(np.array([0.0, 0.2, 1.2, 0.2]), SPEC)
    assert (airborne, contact) == (2, 3)
    assert first_ground_contact(np.array([0.0, 0.2, 0.9]), SPEC) == (None, None)
    assert first_ground_contact(np.array([0.0, 1.2, 5.0]), SPEC) == (1, None)
