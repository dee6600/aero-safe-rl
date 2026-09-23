"""M7 task 2: experiments/metrics.py's detector metrics, on hand-built
sequences whose right answers can be worked out by hand."""
from __future__ import annotations

import numpy as np
import pytest

from ai.detector.dataset import EpisodeCategory, EpisodeData
from experiments.metrics import (
    DetectorTrace, auroc, detection_delay_s, evaluate_detector, expected_calibration_error,
    false_alarm_count, healthy_seconds, rising_edges, sustained, threshold_at_healthy_quantile,
)

DT = 0.1


def _episode(n=100, onset=None, severity=0.5, rotor=1, profile="step", key="0/ep"):
    elapsed = np.arange(n) * DT
    active = np.zeros(n, bool)
    if onset is not None:
        active[onset:] = True
    return EpisodeData(
        key=key,
        category=EpisodeCategory.HEALTHY if onset is None else EpisodeCategory.FAULT_APPLIED,
        severity_commanded=0.0 if onset is None else severity,
        profile="none" if onset is None else profile,
        elapsed_s=elapsed, features=np.zeros((n, 19), np.float32), fault_active=active,
        rotor_class=np.where(active, rotor + 1, 0),
        severity=np.where(active, severity, 0.0).astype(np.float32))


def test_sustained_needs_consecutive_ticks():
    mask = np.array([1, 1, 0, 1, 1, 1, 1, 0], bool)
    assert sustained(mask, 3).tolist() == [0, 0, 0, 0, 0, 1, 1, 0]


def test_sustained_is_causal():
    rng = np.random.default_rng(0)
    mask = rng.random(50) > 0.3
    changed = mask.copy()
    changed[30:] = ~changed[30:]
    np.testing.assert_array_equal(sustained(mask, 4)[:30], sustained(changed, 4)[:30])


def test_rising_edges_include_tick_zero():
    assert rising_edges(np.array([1, 1, 0, 1, 0, 0, 1], bool)).tolist() == [0, 3, 6]


def test_detection_delay():
    active = np.zeros(40, bool)
    active[10:] = True
    elapsed = np.arange(40) * DT
    alarm = np.zeros(40, bool)
    alarm[13:] = True
    assert detection_delay_s(alarm, active, elapsed) == pytest.approx(0.3)
    # an alarm already on at onset counts as immediate detection
    alarm[5:] = True
    assert detection_delay_s(alarm, active, elapsed) == pytest.approx(0.0)
    # never alarmed after onset: missed
    assert detection_delay_s(np.zeros(40, bool), active, elapsed) is None
    # no fault: no delay
    assert detection_delay_s(alarm, np.zeros(40, bool), elapsed) is None


def test_false_alarms_count_only_edges_on_healthy_ticks():
    active = np.zeros(30, bool)
    active[20:] = True
    alarm = np.zeros(30, bool)
    alarm[3:5] = True     # false alarm
    alarm[8:9] = True     # false alarm
    alarm[22:] = True     # true detection
    assert false_alarm_count(alarm, active) == 2
    assert healthy_seconds(active, np.arange(30) * DT) == pytest.approx(2.0)


def test_auroc():
    labels = np.array([0, 0, 1, 1], bool)
    assert auroc(np.array([0.1, 0.2, 0.8, 0.9]), labels) == 1.0
    assert auroc(np.array([0.9, 0.8, 0.2, 0.1]), labels) == 0.0
    assert auroc(np.array([0.1, 0.2]), np.array([1, 1], bool)) is None


def test_ece_zero_when_calibrated_and_known_when_not():
    # 10 ticks at p=0.3 with exactly 3 positives: perfectly calibrated
    p = np.full(10, 0.3)
    y = np.array([1, 1, 1, 0, 0, 0, 0, 0, 0, 0])
    assert expected_calibration_error(p, y) == pytest.approx(0.0)
    # always 0.9 confident, right half the time: |0.5 - 0.9| = 0.4
    assert expected_calibration_error(np.full(10, 0.9), np.arange(10) % 2) == pytest.approx(0.4)


def test_threshold_uses_healthy_ticks_only():
    ep = _episode(n=100, onset=50)
    score = np.concatenate([np.linspace(0, 1, 50), np.full(50, 100.0)])
    t = threshold_at_healthy_quantile([ep], [DetectorTrace(score, False)], q=1.0)
    assert t == pytest.approx(1.0)


def test_evaluate_detector_end_to_end():
    healthy = _episode(n=100, key="0/h")
    low = _episode(n=100, onset=40, severity=0.3, rotor=2, key="0/low")
    high = _episode(n=100, onset=60, severity=0.7, rotor=0, profile="ramp", key="1/high")

    # healthy: one 6-tick blip -> one false alarm
    h_score = np.zeros(100)
    h_score[20:26] = 1.0
    # low: score rises 2 ticks after onset -> alarm after 2 + 5 - 1 ticks -> 0.6 s
    l_score = np.zeros(100)
    l_score[42:] = 1.0
    # high: never detected
    x_score = np.zeros(100)

    rotor_low = np.full(100, 2)       # always right
    rotor_high = np.full(100, 3)      # always wrong
    traces = [
        DetectorTrace(h_score, True, rotor=np.zeros(100, int), severity=np.zeros(100)),
        DetectorTrace(l_score, True, rotor=rotor_low, severity=np.full(100, 0.4)),
        DetectorTrace(x_score, True, rotor=rotor_high, severity=np.full(100, 0.5)),
    ]
    out = evaluate_detector([healthy, low, high], traces, threshold=0.5, hold_ticks=5)

    assert out["false_alarms_per_healthy_hour"] == pytest.approx(1 / ((100 + 40 + 60) * DT / 3600))
    assert out["healthy_episodes_with_any_alarm"] == 1.0
    g = out["groups"]
    assert set(g) == {"faulty", "sev_0.2_0.4", "sev_0.6_0.8", "profile_step", "profile_ramp"}
    assert g["faulty"]["detection_rate"] == 0.5
    assert g["sev_0.2_0.4"]["delay_s_p50"] == pytest.approx(0.6)
    assert g["sev_0.6_0.8"]["detection_rate"] == 0.0
    assert g["sev_0.2_0.4"]["rotor_accuracy"] == 1.0
    assert g["sev_0.6_0.8"]["rotor_accuracy"] == 0.0
    assert g["sev_0.2_0.4"]["severity_mae"] == pytest.approx(0.1)
    assert g["sev_0.6_0.8"]["severity_bias"] == pytest.approx(-0.2)
    assert out["delays_s"]["sev_0.6_0.8"] == [None]
    assert "ece" in out


def test_evaluate_detector_omits_what_a_detector_cannot_produce():
    ep = _episode(n=50, onset=10)
    out = evaluate_detector([ep, _episode(n=50, key="h")],
                            [DetectorTrace(np.ones(50), False), DetectorTrace(np.zeros(50), False)],
                            threshold=0.5)
    row = out["groups"]["faulty"]
    assert "rotor_accuracy" not in row and "severity_mae" not in row
    assert "ece" not in out and "uncertainty_error_auroc" not in out
