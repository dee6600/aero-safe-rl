"""M7 task 2: ai/detector/baselines.py -- causal window statistics and the
three baseline detectors, on synthetic episodes with a planted fault."""
from __future__ import annotations

import numpy as np
import pytest

from ai.detector.baselines import (
    MOTOR_IDX, RESIDUAL_IDX, MotorImbalanceDetector, RandomForestDetector,
    ResidualThresholdDetector, causal_rolling_mean, causal_rolling_std, motor_imbalance,
)
from ai.detector.dataset import EpisodeCategory, EpisodeData
from ai.features.feature_extractor import window_capacity

W = window_capacity()


def _naive_mean(x, w):
    return np.array([x[max(0, i + 1 - w):i + 1].mean(axis=0) for i in range(len(x))])


def test_rolling_mean_and_std_match_naive_loop():
    x = np.random.default_rng(0).normal(size=(40, 3))
    np.testing.assert_allclose(causal_rolling_mean(x, 7), _naive_mean(x, 7))
    naive_std = np.array([x[max(0, i - 6):i + 1].std(axis=0) for i in range(40)])
    np.testing.assert_allclose(causal_rolling_std(x, 7), naive_std, atol=1e-9)


def test_rolling_mean_is_causal():
    x = np.random.default_rng(1).normal(size=(40, 2))
    y = x.copy()
    y[25:] += 100
    np.testing.assert_allclose(causal_rolling_mean(x, 5)[:25], causal_rolling_mean(y, 5)[:25])


def _planted(n=200, onset=80, rotor=2, severity=0.5, seed=0) -> EpisodeData:
    """Hover-like features with rotor `rotor` commanded `severity * 0.3`
    above the others from `onset` on -- the signature the probe measured."""
    rng = np.random.default_rng(seed)
    f = rng.normal(scale=0.01, size=(n, 19)).astype(np.float32)
    f[:, MOTOR_IDX] += 0.6
    active = np.zeros(n, bool)
    if onset is not None:
        active[onset:] = True
        f[onset:, MOTOR_IDX[rotor]] += 0.3 * severity
    return EpisodeData(
        key=f"0/ep_{seed}", profile="step" if onset is not None else "none",
        category=EpisodeCategory.FAULT_APPLIED if onset is not None else EpisodeCategory.HEALTHY,
        severity_commanded=severity if onset is not None else 0.0,
        elapsed_s=np.arange(n) * 0.1, features=f, fault_active=active,
        rotor_class=np.where(active, rotor + 1, 0),
        severity=np.where(active, severity, 0.0).astype(np.float32))


def test_motor_imbalance_finds_the_planted_rotor():
    ep = _planted(rotor=2)
    imb = motor_imbalance(ep.features)
    settled = slice(80 + W, None)
    assert (imb[settled].argmax(axis=1) == 2).all()
    assert imb[:80].max() < imb[settled, 2].min()


def test_motor_imbalance_detector_fits_severity_map():
    eps = [_planted(severity=s, seed=i) for i, s in enumerate((0.2, 0.4, 0.6, 0.8))]
    det = MotorImbalanceDetector().fit(eps)
    tr = det.predict(_planted(severity=0.5, seed=9).features)
    assert not tr.is_probability
    assert np.median(tr.severity[80 + W:]) == pytest.approx(0.5, abs=0.05)
    assert (tr.rotor[80 + W:] == 2).all()


def test_residual_detector_is_window_mean_of_residual():
    f = np.random.default_rng(2).normal(size=(50, 19)).astype(np.float32)
    tr = ResidualThresholdDetector().fit([]).predict(f)
    np.testing.assert_allclose(tr.score, _naive_mean(f[:, RESIDUAL_IDX].astype(float), W), rtol=1e-5)
    assert tr.rotor is None and tr.severity is None


def test_random_forest_learns_presence_and_rotor():
    train = ([_planted(rotor=r, severity=s, seed=10 * r + i)
              for r in range(4) for i, s in enumerate((0.3, 0.6))]
             + [_planted(onset=None, seed=100 + i) for i in range(4)])
    det = RandomForestDetector(seed=0, n_estimators=30, stride=2).fit(train)
    tr = det.predict(_planted(rotor=1, severity=0.5, seed=999).features)
    assert tr.is_probability
    assert tr.score[:70].mean() < 0.2
    assert tr.score[80 + W:].mean() > 0.8
    assert (tr.rotor[80 + W:] == 1).mean() > 0.9


def test_random_forest_is_causal():
    train = [_planted(rotor=r, seed=r) for r in range(4)] + [_planted(onset=None, seed=50)]
    det = RandomForestDetector(seed=0, n_estimators=10).fit(train)
    f = _planted(seed=7).features
    g = f.copy()
    g[120:] += 5.0
    np.testing.assert_allclose(det.predict(f).score[:120], det.predict(g).score[:120])
