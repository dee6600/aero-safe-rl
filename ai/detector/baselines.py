"""M7 task 2: the baselines the learned detector is measured against.

  * MotorImbalanceDetector -- threshold on the largest rotor's command above
    the four-rotor mean, averaged over the feature window. The statistic
    milestones.md M7's dataset probe found near-perfect on settled faults;
    it is the bar, not a strawman, and is never tuned down (anti-pattern 13).
  * ResidualThresholdDetector -- threshold on configs/features.yaml's
    thrust_accel_residual, averaged over the window: planning.md's
    "threshold-on-residual" baseline. No rotor, no severity.
  * RandomForestDetector -- the classical baseline, on each tick's features
    plus their causal window mean and standard deviation.

All take the raw feature matrix of ai.detector.dataset.EpisodeData (the one
feature extractor's output) and return an experiments.metrics.DetectorTrace.
Every window statistic here is causal: tick t sees ticks <= t only.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np
from sklearn.ensemble import RandomForestClassifier, RandomForestRegressor

from ai.detector.dataset import EpisodeData
from ai.features.feature_extractor import all_feature_names, window_capacity
from experiments.metrics import DetectorTrace

_NAMES = all_feature_names()
MOTOR_IDX = [_NAMES.index(f"motor_{i}_output") for i in range(4)]
RESIDUAL_IDX = _NAMES.index("thrust_accel_residual")


def causal_rolling_mean(x: np.ndarray, window: int) -> np.ndarray:
    """Mean over the last `window` ticks (fewer at the start), per column.
    NaN is treated as 0."""
    x = np.nan_to_num(np.asarray(x, dtype=np.float64))
    c = np.cumsum(np.concatenate([np.zeros((1,) + x.shape[1:]), x]), axis=0)
    idx = np.arange(1, len(x) + 1)
    lo = np.maximum(idx - window, 0)
    n = (idx - lo).reshape((-1,) + (1,) * (x.ndim - 1))
    return (c[idx] - c[lo]) / n


def causal_rolling_std(x: np.ndarray, window: int) -> np.ndarray:
    mean = causal_rolling_mean(x, window)
    sq = causal_rolling_mean(np.nan_to_num(np.asarray(x, dtype=np.float64)) ** 2, window)
    return np.sqrt(np.maximum(sq - mean ** 2, 0.0))


def motor_imbalance(features: np.ndarray, window: int | None = None) -> np.ndarray:
    """(T, 4): each rotor's command minus the four-rotor mean, window-averaged."""
    m = features[:, MOTOR_IDX].astype(np.float64)
    rel = m - m.mean(axis=1, keepdims=True)
    return causal_rolling_mean(rel, window or window_capacity())


class MotorImbalanceDetector:
    """Score = largest window-averaged rotor imbalance; rotor = its argmax;
    severity = a least-squares linear map of the score, fitted on train."""
    name = "motor_imbalance_threshold"

    def __init__(self):
        self.coef = (0.0, 0.0)

    def fit(self, episodes: Sequence[EpisodeData]) -> "MotorImbalanceDetector":
        s = np.concatenate([motor_imbalance(e.features).max(axis=1)[e.fault_active]
                            for e in episodes])
        y = np.concatenate([e.severity[e.fault_active] for e in episodes])
        a, b = np.polyfit(s, y, 1)
        self.coef = (float(a), float(b))
        return self

    def predict(self, features: np.ndarray) -> DetectorTrace:
        imb = motor_imbalance(features)
        score = imb.max(axis=1)
        a, b = self.coef
        return DetectorTrace(score=score, is_probability=False, rotor=imb.argmax(axis=1),
                             severity=np.clip(a * score + b, 0.0, 1.0))


class ResidualThresholdDetector:
    name = "residual_threshold"

    def fit(self, episodes: Sequence[EpisodeData]) -> "ResidualThresholdDetector":
        return self

    def predict(self, features: np.ndarray) -> DetectorTrace:
        score = causal_rolling_mean(features[:, [RESIDUAL_IDX]], window_capacity())[:, 0]
        return DetectorTrace(score=score, is_probability=False)


def window_summary(features: np.ndarray) -> np.ndarray:
    """(T, 3F): the tick's features, their causal window mean, window std."""
    w = window_capacity()
    return np.hstack([np.nan_to_num(features.astype(np.float64)),
                      causal_rolling_mean(features, w), causal_rolling_std(features, w)])


class RandomForestDetector:
    """5-class forest {healthy, rotor 0..3} for presence and rotor, and a
    regression forest on fault-active ticks for severity. Trained on every
    `stride`-th tick of the training episodes (neighbouring ticks are nearly
    duplicates, and the full set is ~250k rows)."""
    name = "random_forest"

    def __init__(self, seed: int = 0, n_estimators: int = 200, stride: int = 3):
        self.stride = stride
        self.clf = RandomForestClassifier(n_estimators=n_estimators, min_samples_leaf=5,
                                          n_jobs=-1, random_state=seed)
        self.reg = RandomForestRegressor(n_estimators=n_estimators, min_samples_leaf=5,
                                         n_jobs=-1, random_state=seed)

    def fit(self, episodes: Sequence[EpisodeData]) -> "RandomForestDetector":
        X = np.vstack([window_summary(e.features)[::self.stride] for e in episodes])
        y = np.concatenate([e.rotor_class[::self.stride] for e in episodes])
        sev = np.concatenate([e.severity[::self.stride] for e in episodes])
        self.clf.fit(X, y)
        self.reg.fit(X[y > 0], sev[y > 0])
        return self

    def predict(self, features: np.ndarray) -> DetectorTrace:
        X = window_summary(features)
        proba = np.zeros((len(X), 5))
        proba[:, self.clf.classes_] = self.clf.predict_proba(X)
        return DetectorTrace(score=1.0 - proba[:, 0], is_probability=True,
                             rotor=proba[:, 1:].argmax(axis=1),
                             severity=self.reg.predict(X))
