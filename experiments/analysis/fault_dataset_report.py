#!/usr/bin/env python3
"""Prints the numbers M6 task 10's docs/fault_dataset.md needs -- read back
from a fault-dataset run's episode summaries (experiments/
generate_fault_dataset.py). One run directory in, one table out, same
pattern as experiments/analysis/noise_floor.py (M3): every number here is a
plain read of what the run already wrote, nothing computed elsewhere.

This is the actual research-premise check M6 exists for: whether PX4's own
FailureDetector stays silent at the severities this project targets, and
whether the plugin's confirmed-applied rate holds up at real dataset scale
-- broken down by severity bucket, not just an aggregate that could hide a
severity range where either assumption quietly stops holding.

Usage: python experiments/analysis/fault_dataset_report.py results/<run_id>/
"""
from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

# Severity is continuous (planning.md §6: s in [0.2, 0.9]); bucketed here
# purely for a readable report -- the underlying data (per-episode
# fault_severity_commanded) is never itself binned.
_SEVERITY_BUCKETS = [(0.2, 0.4), (0.4, 0.6), (0.6, 0.8), (0.8, 1.0)]


def load_summaries(run_dir: Path) -> pd.DataFrame:
    paths = sorted(run_dir.glob("worker_*/episode_*_summary.parquet"))
    if not paths:
        raise SystemExit(f"no episode summaries found under {run_dir}")
    return pd.concat([pd.read_parquet(p) for p in paths], ignore_index=True)


def _fmt_pct(numerator: int, denominator: int) -> str:
    if denominator == 0:
        return "n/a (0 episodes)"
    return f"{numerator}/{denominator} ({100 * numerator / denominator:.1f}%)"


def print_report(df: pd.DataFrame) -> None:
    n = len(df)
    print(f"episodes: {n}")

    print("\nby termination_reason:")
    print(df["termination_reason"].value_counts().to_string())

    print(f"\nvalid: {_fmt_pct(int(df['valid'].sum()), n)}")

    faulty = df[df["fault_applied"]]
    healthy = df[~df["fault_applied"]]
    print(f"\nfaulty episodes: {len(faulty)}   healthy (negative) episodes: {len(healthy)}")

    if len(faulty) == 0:
        print("\nno faulty episodes in this run -- nothing further to report")
        return

    print(f"\noverall fault confirmation rate: "
          f"{_fmt_pct(int(faulty['fault_confirmed_applied'].sum()), len(faulty))}")
    print(f"overall PX4 FailureDetector-silent rate (all episodes): "
          f"{_fmt_pct(int(df['px4_failure_detector_silent'].sum()), n)}")
    print(f"PX4 FailureDetector-silent rate (faulty episodes only): "
          f"{_fmt_pct(int(faulty['px4_failure_detector_silent'].sum()), len(faulty))}")

    print("\nby severity bucket (faulty episodes only):")
    header = f"  {'range':12s} {'n':>5s}  {'confirmed_applied':>20s}  {'detector_silent':>18s}"
    print(header)
    for lo, hi in _SEVERITY_BUCKETS:
        bucket = faulty[(faulty["fault_severity_commanded"] >= lo)
                         & (faulty["fault_severity_commanded"] < hi + 1e-9)]
        if len(bucket) == 0:
            print(f"  [{lo:.1f}, {hi:.1f}]   {0:>5d}  {'n/a':>20s}  {'n/a':>18s}")
            continue
        confirmed = _fmt_pct(int(bucket["fault_confirmed_applied"].sum()), len(bucket))
        silent = _fmt_pct(int(bucket["px4_failure_detector_silent"].sum()), len(bucket))
        print(f"  [{lo:.1f}, {hi:.1f}]   {len(bucket):>5d}  {confirmed:>20s}  {silent:>18s}")

    print("\nby profile (faulty episodes only):")
    print(faulty.groupby("fault_profile")["fault_confirmed_applied"].agg(["count", "mean"])
          .to_string())

    print("\nby rotor index (faulty episodes only):")
    print(faulty.groupby("fault_rotor_index")["fault_confirmed_applied"].agg(["count", "mean"])
          .to_string())

    # Episodes where a fault was requested but never confirmed applied --
    # the "completed before onset" edge case found live during task 9's
    # small run (a mission that finishes before a late-sampled onset time
    # is reached), or a genuine confirmation failure. Both matter, worth
    # surfacing explicitly rather than only as a rate.
    unconfirmed = faulty[~faulty["fault_confirmed_applied"]]
    if len(unconfirmed) > 0:
        print(f"\n{len(unconfirmed)} faulty episode(s) never confirmed applied -- "
              f"termination_reason breakdown:")
        print(unconfirmed["termination_reason"].value_counts().to_string())


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", help="results/<run_id>/ directory")
    args = ap.parse_args(argv)

    df = load_summaries(Path(args.run_dir))
    print_report(df)


if __name__ == "__main__":
    main()
