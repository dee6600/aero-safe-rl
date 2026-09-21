#!/usr/bin/env python3
"""Prints the numbers M3 tasks 6-8 need: noise floor, run-to-run divergence,
success rate, and reset cost by tier -- read back from one run's episode
summaries. One run directory in, one table out; every number here is a plain
read of what run_episodes.py already wrote, nothing computed elsewhere.

Usage: python experiments/analysis/noise_floor.py results/<run_id>/
"""
from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd


def load_summaries(run_dir: Path) -> pd.DataFrame:
    paths = sorted(run_dir.glob("worker_*/episode_*_summary.parquet"))
    if not paths:
        raise SystemExit(f"no episode summaries found under {run_dir}")
    return pd.concat([pd.read_parquet(p) for p in paths], ignore_index=True)


def _fmt(x: float) -> str:
    return f"{x:.4f}" if pd.notna(x) else "nan"


def print_report(df: pd.DataFrame) -> None:
    n = len(df)
    completed = df[df["termination_reason"] == "completed"]
    print(f"episodes: {n}   completed: {len(completed)}/{n} "
          f"({100 * len(completed) / n:.1f}%)")

    if (df["termination_reason"] != "completed").any():
        print("\nnon-completed episodes:")
        bad = df[df["termination_reason"] != "completed"]
        print(bad[["episode_id", "termination_reason", "reset_tier"]].to_string(index=False))

    if len(completed) == 0:
        return

    print("\nnoise floor (completed episodes only):")
    for col in ("position_rmse_m", "final_position_error_m"):
        print(f"  {col:24s} mean={_fmt(completed[col].mean())}  "
              f"std={_fmt(completed[col].std())}  "
              f"min={_fmt(completed[col].min())}  max={_fmt(completed[col].max())}")

    print("\nrun-to-run divergence (completed episodes; D11):")
    print(f"  {'position_rmse_m':24s} std={_fmt(completed['position_rmse_m'].std())}")
    for axis in ("final_pos_x", "final_pos_y", "final_pos_z"):
        if axis in completed.columns:
            print(f"  {axis:24s} std={_fmt(completed[axis].std())}  "
                  f"range=[{_fmt(completed[axis].min())}, {_fmt(completed[axis].max())}]")
    print(f"  {'t_wall_duration_s':24s} mean={_fmt(completed['t_wall_duration_s'].mean())}  "
          f"std={_fmt(completed['t_wall_duration_s'].std())}")
    print(f"  {'t_sim_duration_s':24s} mean={_fmt(completed['t_sim_duration_s'].mean())}  "
          f"std={_fmt(completed['t_sim_duration_s'].std())}")

    if "reset_wall_duration_s" in df.columns:
        print("\nreset cost by tier:")
        by_tier = df.groupby("reset_tier")["reset_wall_duration_s"].agg(["count", "mean", "std"])
        print(by_tier.to_string())


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", help="results/<run_id>/ directory")
    args = ap.parse_args(argv)

    df = load_summaries(Path(args.run_dir))
    print_report(df)


if __name__ == "__main__":
    main()
