#!/usr/bin/env python3
"""M5 task 5: compute per-feature normalisation statistics from a healthy-
flight run and freeze them to configs/rl/normalization_v1.yaml.

Run once, against a real dataset flown under episode schema v3 (M3's
existing 20-healthy-run fixtures predate v3 and lack the raw attitude/
rate/acceleration/motor-output fields FeatureExtractor needs). The output
file is a frozen artifact from here on -- CLAUDE.md anti-pattern 11 ("never
recompute normalisation statistics after training has started"). Re-running
this script and overwriting configs/rl/normalization_v1.yaml in place after
any downstream milestone has consumed it would silently invalidate every
model trained against the old statistics; that must be a deliberate
feature_version/norm_version bump, not an accidental re-run.

Only steps from valid=True episodes are used -- a worker-restarted or
sim_fault episode's telemetry is not a measurement of healthy flight
statistics.

Usage:
    python experiments/analysis/compute_normalization_stats.py results/<run_id>/ \
        --out configs/rl/normalization_v1.yaml
"""
from __future__ import annotations

import argparse
import datetime
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

REPO = Path(__file__).resolve().parent.parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from ai.features.feature_extractor import all_feature_names, extract_series, feature_version

NORM_VERSION = "1"


def _valid_episode_ids(run_dir: Path) -> set[str]:
    ids: set[str] = set()
    for summary_path in sorted(run_dir.glob("worker_*/episode_*_summary.parquet")):
        df = pd.read_parquet(summary_path)
        for _, row in df.iterrows():
            if bool(row["valid"]):
                ids.add((str(summary_path.parent.name), str(row["episode_id"])))
    return ids


def collect_feature_vectors(run_dir: Path) -> dict[str, np.ndarray]:
    """Runs FeatureExtractor causally over every valid episode's steps and
    returns, per feature name, a 1-D array of every value seen across every
    step of every valid episode in the run."""
    valid = _valid_episode_ids(run_dir)
    if not valid:
        raise SystemExit(f"no valid=True episodes found under {run_dir}")

    per_feature: dict[str, list[float]] = {name: [] for name in all_feature_names()}
    n_episodes = 0
    for steps_path in sorted(run_dir.glob("worker_*/episode_*_steps.parquet")):
        key = (steps_path.parent.name, steps_path.stem[len("episode_"):-len("_steps")])
        if key not in valid:
            continue
        df = pd.read_parquet(steps_path)
        if df.empty:
            # A valid=True episode can still have zero steps (e.g.
            # preflight_failed -- the flight never got telemetry to record).
            # Nothing to extract; not an error.
            print(f"  skipping {steps_path.relative_to(run_dir)}: no steps recorded")
            continue
        n_episodes += 1
        df = df.sort_values("t_sim_s")
        frames = df.to_dict(orient="records")
        series = extract_series(frames)
        for row in series:
            for name, value in row.items():
                per_feature[name].append(value)

    if n_episodes == 0:
        raise SystemExit(f"no valid episode step files found under {run_dir}")

    print(f"computed from {n_episodes} valid episode(s), "
          f"{len(next(iter(per_feature.values())))} total steps")
    return {name: np.array(values, dtype=np.float64) for name, values in per_feature.items()}


def compute_stats(per_feature: dict[str, np.ndarray]) -> dict[str, dict[str, float]]:
    stats = {}
    for name, values in per_feature.items():
        finite = values[np.isfinite(values)]
        if len(finite) == 0:
            raise SystemExit(f"feature {name!r} has no finite values in this run -- "
                              f"cannot compute normalisation statistics")
        n_nonfinite = len(values) - len(finite)
        if n_nonfinite:
            print(f"  warning: {name} had {n_nonfinite} non-finite value(s), excluded from stats")
        stats[name] = {
            "mean": float(finite.mean()),
            "std": float(finite.std()),
            "n": int(len(finite)),
        }
    return stats


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", help="results/<run_id>/ directory, flown under schema v3")
    ap.add_argument("--out", default=str(REPO / "configs" / "rl" / "normalization_v1.yaml"))
    args = ap.parse_args(argv)

    run_dir = Path(args.run_dir)
    per_feature = collect_feature_vectors(run_dir)
    stats = compute_stats(per_feature)

    out = {
        "norm_version": NORM_VERSION,
        "feature_version": feature_version(),
        "computed_from": {
            "run_dir": str(run_dir),
            "computed_at_utc": datetime.datetime.now(tz=datetime.timezone.utc).isoformat(),
        },
        "stats": stats,
    }

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.safe_dump(out, sort_keys=False))
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
