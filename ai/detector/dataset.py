"""M7 task 1: the fault detector's labelled dataset, built from an M6
fault-dataset run directory (results/<run_id>/worker_*/episode_*_{summary,
steps}.parquet).

One record per episode, never a pool of timesteps. Features come from
ai.features.feature_extractor.extract_series (the one feature
implementation, CLAUDE.md §1.4); per-tick labels come from
experiments.fault_schedule.commanded_severity (the same function
EpisodeRunner commands the plugin with), so a tick's label is by
construction what was sent to the plugin at that tick.

Splitting is by episode and only by episode (milestones.md M7 task 1): the
split is a set of episode keys, and EpisodeSplit.select() is the only way to
turn it into data. Nothing in this module accepts timestep indices, so
overlapping windows from one flight cannot land on both sides of a split.

Pure except for load_episodes(), which reads parquet. No ROS import.
"""
from __future__ import annotations

import enum
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import pandas as pd

from ai.features.feature_extractor import RAW_FRAME_FIELDS, all_feature_names, extract_series
from experiments.fault_schedule import commanded_severity

# An echo arriving later than this after the commanded onset tick means the
# plugin's actual onset time is unknown, so the episode's labels cannot be
# trusted. Measured on m6_dataset_v1: median echo lag 0.10 s, p95 0.20 s.
MAX_ECHO_LAG_S = 1.0

# Severity strata for the split. Same buckets as
# experiments/analysis/fault_dataset_report.py's report.
SEVERITY_BUCKETS = ((0.2, 0.4), (0.4, 0.6), (0.6, 0.8), (0.8, 1.0))

SPLIT_FRACTIONS = (0.70, 0.15, 0.15)

# rotor_class label: 0 = healthy, k + 1 = rotor k faulty.
HEALTHY_CLASS = 0


class EpisodeCategory(str, enum.Enum):
    FAULT_APPLIED = "fault_applied"              # onset reached and echoed by the plugin
    FAULT_NEVER_ONSET = "fault_never_onset"      # commanded onset came after the flight ended
    HEALTHY = "healthy"
    EXCLUDED_INVALID = "excluded_invalid"        # valid=False
    EXCLUDED_NO_STEPS = "excluded_no_steps"      # never got airborne, nothing logged
    EXCLUDED_ONSET_UNVERIFIED = "excluded_onset_unverified"  # no echo, late echo, or echo without onset

    @property
    def included(self) -> bool:
        return not self.value.startswith("excluded")


def _onset_tick(summary: Mapping, elapsed_s: np.ndarray) -> int | None:
    """Index of the first tick whose mission-elapsed time reached the
    requested onset -- the tick at which EpisodeRunner commanded the fault.
    None when the flight ended first."""
    reached = np.flatnonzero(elapsed_s >= summary["fault_onset_time_s_requested"])
    return int(reached[0]) if len(reached) else None


def classify_episode(summary: Mapping, elapsed_s: np.ndarray) -> EpisodeCategory:
    """Which inclusion category an episode falls in. `elapsed_s` is each
    logged tick's t_sim_s minus the first tick's -- EpisodeRunner's own
    clock reference for fault onset."""
    if not summary["valid"]:
        return EpisodeCategory.EXCLUDED_INVALID
    if len(elapsed_s) == 0:
        return EpisodeCategory.EXCLUDED_NO_STEPS
    if not summary["fault_applied"]:
        return EpisodeCategory.HEALTHY

    observed = summary["fault_onset_time_s_observed"]
    echoed = observed is not None and np.isfinite(observed)
    tick = _onset_tick(summary, elapsed_s)
    if tick is None:
        return (EpisodeCategory.EXCLUDED_ONSET_UNVERIFIED if echoed
                else EpisodeCategory.FAULT_NEVER_ONSET)
    if not echoed or observed - elapsed_s[tick] > MAX_ECHO_LAG_S:
        return EpisodeCategory.EXCLUDED_ONSET_UNVERIFIED
    return EpisodeCategory.FAULT_APPLIED


def tick_labels(summary: Mapping, elapsed_s: np.ndarray
                ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per-tick (fault_active, rotor_class, severity) for an included
    episode. Severity is the instantaneous commanded severity (ramp-aware);
    a tick is fault_active when that severity is > 0, so a ramp's onset
    tick itself (commanded 0.0) is still healthy."""
    n = len(elapsed_s)
    severity = np.zeros(n, dtype=np.float32)
    tick = _onset_tick(summary, elapsed_s) if summary["fault_applied"] else None
    if tick is not None:
        t_since = elapsed_s - elapsed_s[tick]
        severity = np.array([
            commanded_severity(summary["fault_severity_commanded"], summary["fault_profile"],
                               summary["fault_ramp_duration_s"], t) for t in t_since
        ], dtype=np.float32)
    active = severity > 0
    rotor_class = np.where(active, int(summary["fault_rotor_index"]) + 1,
                           HEALTHY_CLASS).astype(np.int64)
    return active, rotor_class, severity


@dataclass(frozen=True, eq=False)
class EpisodeData:
    key: str                      # "<worker_id>/<episode_id>"
    category: EpisodeCategory
    severity_commanded: float     # episode's target severity; 0.0 for healthy
    profile: str                  # "step" / "ramp"; "none" unless the fault was applied
    elapsed_s: np.ndarray         # (T,)
    features: np.ndarray          # (T, F) raw, unnormalised, all_feature_names() order
    fault_active: np.ndarray      # (T,) bool
    rotor_class: np.ndarray       # (T,) int, 0 = healthy, k + 1 = rotor k
    severity: np.ndarray          # (T,) float32, instantaneous commanded severity


def build_episode(summary: Mapping, steps: pd.DataFrame) -> EpisodeData | EpisodeCategory:
    """One episode's labelled record, or its exclusion category."""
    elapsed = (steps["t_sim_s"].to_numpy(float) - float(steps["t_sim_s"].iloc[0])
               if len(steps) else np.zeros(0))
    category = classify_episode(summary, elapsed)
    if not category.included:
        return category
    if not np.all(np.isfinite(elapsed)):
        raise ValueError(f"episode {summary['episode_id']}: non-finite t_sim_s in steps")

    frames = steps[list(RAW_FRAME_FIELDS)].to_dict("records")
    names = all_feature_names()
    features = np.array([[f[n] for n in names] for f in extract_series(frames)],
                        dtype=np.float32)
    active, rotor_class, severity = tick_labels(summary, elapsed)
    return EpisodeData(
        key=f"{summary['worker_id']}/{summary['episode_id']}",
        category=category,
        severity_commanded=(float(summary["fault_severity_commanded"])
                            if category == EpisodeCategory.FAULT_APPLIED else 0.0),
        profile=(str(summary["fault_profile"])
                 if category == EpisodeCategory.FAULT_APPLIED else "none"),
        elapsed_s=elapsed, features=features,
        fault_active=active, rotor_class=rotor_class, severity=severity,
    )


def load_episodes(run_dir: str | Path) -> tuple[list[EpisodeData], dict[str, int]]:
    """Every included episode under `run_dir`, sorted by key, plus a count
    of episodes per category (excluded ones included) for the report."""
    run_dir = Path(run_dir)
    summary_paths = sorted(run_dir.glob("worker_*/episode_*_summary.parquet"))
    if not summary_paths:
        raise FileNotFoundError(f"no episode summaries under {run_dir}")

    episodes: list[EpisodeData] = []
    counts: dict[str, int] = {c.value: 0 for c in EpisodeCategory}
    for sp in summary_paths:
        summary = pd.read_parquet(sp).iloc[0].to_dict()
        steps_path = sp.with_name(sp.name.replace("_summary.parquet", "_steps.parquet"))
        steps = pd.read_parquet(steps_path) if steps_path.exists() else pd.DataFrame()
        result = build_episode(summary, steps)
        if isinstance(result, EpisodeCategory):
            counts[result.value] += 1
        else:
            counts[result.category.value] += 1
            episodes.append(result)
    episodes.sort(key=lambda e: e.key)
    return episodes, counts


def stratum(episode: EpisodeData) -> str:
    """Split stratum: 'healthy' for every episode without an applied fault
    (including FAULT_NEVER_ONSET), else its severity bucket."""
    if episode.category != EpisodeCategory.FAULT_APPLIED:
        return "healthy"
    s = episode.severity_commanded
    for lo, hi in SEVERITY_BUCKETS:
        if lo <= s < hi or (hi == SEVERITY_BUCKETS[-1][1] and s == hi):  # top bucket is closed
            return f"sev_{lo:.1f}_{hi:.1f}"
    raise ValueError(f"episode {episode.key}: severity {s} outside every bucket")


@dataclass(frozen=True)
class EpisodeSplit:
    train: tuple[str, ...]
    val: tuple[str, ...]
    test: tuple[str, ...]
    seed: int

    def select(self, episodes: Sequence[EpisodeData], part: str) -> list[EpisodeData]:
        """The episodes of one part ('train', 'val' or 'test'). The only way
        to get data out of a split -- always whole episodes."""
        keys = set(getattr(self, part))
        return [e for e in episodes if e.key in keys]

    def digest(self) -> str:
        """Stable id for this exact split, recorded in every checkpoint and
        report trained or evaluated on it."""
        payload = json.dumps({"train": sorted(self.train), "val": sorted(self.val),
                              "test": sorted(self.test), "seed": self.seed},
                             sort_keys=True).encode()
        return hashlib.sha256(payload).hexdigest()[:16]


def split_by_episode(episodes: Sequence[EpisodeData], seed: int,
                     fractions: tuple[float, float, float] = SPLIT_FRACTIONS) -> EpisodeSplit:
    """Stratified train/val/test split of episode keys. Deterministic for a
    given seed and episode set, independent of input order."""
    if not np.isclose(sum(fractions), 1.0):
        raise ValueError(f"split fractions must sum to 1, got {fractions}")
    rng = np.random.default_rng(seed)
    by_stratum: dict[str, list[str]] = {}
    for e in episodes:
        by_stratum.setdefault(stratum(e), []).append(e.key)

    parts: tuple[list[str], list[str], list[str]] = ([], [], [])
    for name in sorted(by_stratum):
        keys = sorted(by_stratum[name])
        keys = [keys[i] for i in rng.permutation(len(keys))]
        n_train = round(len(keys) * fractions[0])
        n_val = round(len(keys) * fractions[1])
        parts[0].extend(keys[:n_train])
        parts[1].extend(keys[n_train:n_train + n_val])
        parts[2].extend(keys[n_train + n_val:])
    return EpisodeSplit(train=tuple(sorted(parts[0])), val=tuple(sorted(parts[1])),
                        test=tuple(sorted(parts[2])), seed=seed)
