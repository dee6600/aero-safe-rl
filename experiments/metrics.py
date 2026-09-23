"""The one place any metric in this project is computed (milestones.md M10).
Created in M7 with the fault-detector metrics; M10 adds the recovery metrics
here rather than in a second module.

Detector metrics take whole episodes (ai.detector.dataset.EpisodeData) and
one DetectorTrace per episode -- the per-tick output of any detector, learned
or baseline -- so the model and every baseline are scored by exactly the
same code. Pure NumPy + scikit-learn, no I/O.

Protocol (milestones.md M7 task 2):
  * An alarm is a score at or above the threshold sustained for
    ALARM_HOLD_TICKS consecutive ticks (0.5 s at 10 Hz).
  * Every detector's threshold is set by the same rule on the *validation*
    split -- the HEALTHY_QUANTILE quantile of its score over healthy ticks --
    so all detectors run at the same per-tick healthy false-positive rate
    and are compared on what that buys them.
  * Detection delay: from the first fault-active tick to the first alarm at
    or after it. No alarm after onset = missed.
  * A false alarm is an alarm rising edge on a healthy tick (healthy
    episodes, and faulty episodes before onset), counted per healthy
    flight-hour.
  * Rotor-ID accuracy and severity error are over every fault-active tick,
    detected or not, so a detector cannot improve them by alarming less.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np
from sklearn.metrics import roc_auc_score

from ai.detector.dataset import EpisodeCategory, EpisodeData, stratum

ALARM_HOLD_TICKS = 5
HEALTHY_QUANTILE = 0.995
ECE_BINS = 10


@dataclass(frozen=True, eq=False)
class DetectorTrace:
    """One episode's per-tick detector output, causal by the producer's
    construction. `score` is higher-means-faultier; it is a probability
    only when `is_probability`. Fields a detector cannot produce are None
    (the residual-threshold baseline identifies no rotor, for example)."""
    score: np.ndarray                       # (T,)
    is_probability: bool
    rotor: Optional[np.ndarray] = None      # (T,) predicted rotor index 0..3
    severity: Optional[np.ndarray] = None   # (T,) predicted severity
    uncertainty: Optional[np.ndarray] = None  # (T,) higher = less sure


def sustained(mask: np.ndarray, hold_ticks: int = ALARM_HOLD_TICKS) -> np.ndarray:
    """alarm[t] is True when mask has been True for the last `hold_ticks`
    ticks, t included. Causal: alarm[t] depends on mask[:t+1] only."""
    mask = np.asarray(mask, dtype=bool)
    run = np.zeros(len(mask), dtype=np.int64)
    count = 0
    for i, m in enumerate(mask):
        count = count + 1 if m else 0
        run[i] = count
    return run >= hold_ticks


def rising_edges(alarm: np.ndarray) -> np.ndarray:
    """Indices where the alarm turns on, including tick 0 if it starts on."""
    alarm = np.asarray(alarm, dtype=bool)
    prev = np.concatenate([[False], alarm[:-1]])
    return np.flatnonzero(alarm & ~prev)


def detection_delay_s(alarm: np.ndarray, fault_active: np.ndarray,
                      elapsed_s: np.ndarray) -> Optional[float]:
    """Seconds from the first fault-active tick to the first alarm at or
    after it; None if the fault never became active or was never detected."""
    active = np.flatnonzero(fault_active)
    if len(active) == 0:
        return None
    onset = active[0]
    hits = np.flatnonzero(alarm[onset:])
    if len(hits) == 0:
        return None
    return float(elapsed_s[onset + hits[0]] - elapsed_s[onset])


def false_alarm_count(alarm: np.ndarray, fault_active: np.ndarray) -> int:
    edges = rising_edges(alarm)
    return int(np.sum(~np.asarray(fault_active, dtype=bool)[edges]))


def healthy_seconds(fault_active: np.ndarray, elapsed_s: np.ndarray) -> float:
    if len(elapsed_s) < 2:
        return 0.0
    dt = float(np.median(np.diff(elapsed_s)))
    return float(np.sum(~np.asarray(fault_active, dtype=bool)) * dt)


def auroc(scores: np.ndarray, labels: np.ndarray) -> Optional[float]:
    labels = np.asarray(labels, dtype=bool)
    if labels.all() or not labels.any():
        return None
    return float(roc_auc_score(labels, scores))


def expected_calibration_error(prob: np.ndarray, labels: np.ndarray,
                               n_bins: int = ECE_BINS) -> float:
    """Tick-weighted mean |accuracy - confidence| over equal-width bins of
    predicted probability of the positive class."""
    prob = np.asarray(prob, dtype=float)
    labels = np.asarray(labels, dtype=float)
    bins = np.minimum((prob * n_bins).astype(int), n_bins - 1)
    ece = 0.0
    for b in range(n_bins):
        sel = bins == b
        if sel.any():
            ece += sel.mean() * abs(labels[sel].mean() - prob[sel].mean())
    return float(ece)


def threshold_at_healthy_quantile(episodes: Sequence[EpisodeData],
                                  traces: Sequence[DetectorTrace],
                                  q: float = HEALTHY_QUANTILE) -> float:
    """The alarm threshold for a detector: quantile `q` of its score over
    every healthy tick of `episodes` (the validation split, by protocol)."""
    healthy = np.concatenate([t.score[~e.fault_active] for e, t in zip(episodes, traces)])
    healthy = healthy[np.isfinite(healthy)]
    return float(np.quantile(healthy, q))


def _group_keys(e: EpisodeData) -> list[str]:
    if e.category != EpisodeCategory.FAULT_APPLIED:
        return ["healthy"]
    return ["faulty", stratum(e), f"profile_{e.profile}"]


def evaluate_detector(episodes: Sequence[EpisodeData], traces: Sequence[DetectorTrace],
                      threshold: float, hold_ticks: int = ALARM_HOLD_TICKS) -> dict:
    """Every M7 detector metric, overall and per group ('faulty', each
    severity stratum, each profile). Also returns the raw per-episode
    delays, which M8b's detector-output simulator is fitted to."""
    if len(episodes) != len(traces):
        raise ValueError("one trace per episode")
    alarms = [sustained(np.nan_to_num(t.score, nan=-np.inf) >= threshold, hold_ticks)
              for t in traces]

    all_healthy_scores = np.concatenate([t.score[~e.fault_active]
                                         for e, t in zip(episodes, traces)])
    n_false = sum(false_alarm_count(a, e.fault_active) for e, a in zip(episodes, alarms))
    healthy_h = sum(healthy_seconds(e.fault_active, e.elapsed_s) for e in episodes) / 3600
    healthy_eps = [(e, a) for e, a in zip(episodes, alarms)
                   if e.category != EpisodeCategory.FAULT_APPLIED]

    labels = np.concatenate([e.fault_active for e in episodes])
    scores = np.concatenate([t.score for t in traces])
    out: dict = {
        "threshold": threshold,
        "hold_ticks": hold_ticks,
        "n_episodes": len(episodes),
        "false_alarms_per_healthy_hour": n_false / healthy_h if healthy_h > 0 else None,
        "healthy_episodes_with_any_alarm": (
            float(np.mean([a.any() for _, a in healthy_eps])) if healthy_eps else None),
        "auroc_tick": auroc(scores, labels),
        "groups": {},
        "delays_s": {},
    }
    if traces[0].is_probability:
        out["ece"] = expected_calibration_error(scores, labels)
    if traces[0].uncertainty is not None:
        # Is the uncertainty output informative? AUROC of uncertainty for
        # predicting a wrong fault/no-fault call at p = 0.5.
        unc = np.concatenate([t.uncertainty for t in traces])
        out["uncertainty_error_auroc"] = auroc(unc, (scores >= 0.5) != labels)

    groups: dict[str, list[int]] = {}
    for i, e in enumerate(episodes):
        for g in _group_keys(e):
            groups.setdefault(g, []).append(i)

    for g, idx in sorted(groups.items()):
        if g == "healthy":
            continue
        delays = [detection_delay_s(alarms[i], episodes[i].fault_active, episodes[i].elapsed_s)
                  for i in idx]
        hit = [d for d in delays if d is not None]
        act_scores = np.concatenate([traces[i].score[episodes[i].fault_active] for i in idx])
        row = {
            "n_episodes": len(idx),
            "detection_rate": len(hit) / len(idx),
            "delay_s_p50": float(np.median(hit)) if hit else None,
            "delay_s_p90": float(np.quantile(hit, 0.9)) if hit else None,
            "delay_s_mean": float(np.mean(hit)) if hit else None,
            "auroc_tick": auroc(np.concatenate([act_scores, all_healthy_scores]),
                                np.concatenate([np.ones(len(act_scores), bool),
                                                np.zeros(len(all_healthy_scores), bool)])),
        }
        if traces[0].rotor is not None:
            pred = np.concatenate([traces[i].rotor[episodes[i].fault_active] for i in idx])
            true = np.concatenate([episodes[i].rotor_class[episodes[i].fault_active] - 1
                                   for i in idx])
            row["rotor_accuracy"] = float(np.mean(pred == true))
        if traces[0].severity is not None:
            err = np.concatenate([traces[i].severity[episodes[i].fault_active]
                                  - episodes[i].severity[episodes[i].fault_active] for i in idx])
            row["severity_mae"] = float(np.mean(np.abs(err)))
            row["severity_bias"] = float(np.mean(err))
            row["severity_err_std"] = float(np.std(err))
        out["groups"][g] = row
        out["delays_s"][g] = delays
    return out
