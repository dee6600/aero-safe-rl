"""M8b task 6: the fit of the simulated detector (experiments/fit_detector_sim.py)
on hand-made traces."""
import numpy as np
import pytest

from experiments.fit_detector_sim import bucket, fault_stats, ramp_duration


def _trace(target=0.3, profile="ramp", ramp_s=4.0, onset=5.0, detect_at=6.0, end=30.0):
    t = np.round(np.arange(0.0, end, 0.1), 3)
    since = t - onset
    frac = np.clip(since / ramp_s, 0, 1) if profile == "ramp" else np.ones_like(t)
    sev = np.where(since > 0, target * frac, 0.0)
    p = np.where(t >= detect_at, 0.99, 0.001)
    return dict(t=t, sev=sev, rotor=1, profile=profile, target=target, ramp_s=ramp_s if profile == "ramp" else 0.0,
                p=p, rotor_pred=np.ones_like(t), sev_est=sev, unc=np.zeros_like(t), descent_start=None, source="test")


def test_ramp_length_is_the_commanded_one_even_when_the_trace_ends_first():
    # the recovery controller landed 1 s into a 4 s ramp: still a 4 s ramp, not a step
    tr = _trace(ramp_s=4.0, onset=5.0, detect_at=5.6, end=6.0)
    assert ramp_duration(tr) == 4.0
    assert ramp_duration(_trace(profile="step")) is None


def test_ramp_detection_level_is_the_severity_reached_at_detection():
    tr = _trace(target=0.3, ramp_s=4.0, onset=5.0, detect_at=6.1)
    f = fault_stats([tr])
    first_active = 5.1
    assert f["ramp_level"][bucket(0.3)] == [pytest.approx(0.3 * (6.1 - first_active) / 4.0)]
    assert f["delays"][(bucket(0.3), "ramp")] == [pytest.approx(6.1 - first_active)]


def test_a_ramp_detected_after_it_ends_counts_at_the_full_severity():
    tr = _trace(target=0.3, ramp_s=2.0, onset=5.0, detect_at=9.0)
    assert fault_stats([tr])["ramp_level"][bucket(0.3)] == [pytest.approx(0.3)]
