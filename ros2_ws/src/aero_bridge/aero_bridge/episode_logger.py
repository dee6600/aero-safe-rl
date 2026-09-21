"""The project's one episode-record writer (M3 task 4, CLAUDE.md §3.5 /
anti-pattern 15: one writer per file, N workers never append to one file).

One EpisodeLogger is constructed per (run_id, worker_id) and only ever
writes under results/<run_id>/worker_<worker_id>/, so two workers can never
collide on a path even if their episode ids coincide. Each episode gets its
own pair of Parquet files -- steps and summary -- rather than one growing
per-worker file, so writing never needs a read-modify-write and a killed
process never leaves a half-written row behind an already-flushed one.

Every record is validated against configs/schema/episode_record.yaml
(experiments/episode_schema.py) before being written: an invalid record
raises rather than landing in results/ silently wrong.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import pandas as pd

from experiments.episode_schema import validate_episode, validate_step


class EpisodeLogger:
    """Buffers one episode's step rows in memory (a few hundred to a few
    thousand small dicts -- negligible) and flushes both files together in
    write_episode(), once the episode's termination_reason is known."""

    def __init__(self, run_id: str, worker_id: int, results_dir: str | Path = "results"):
        self.run_id = run_id
        self.worker_id = worker_id
        self.dir = Path(results_dir) / run_id / f"worker_{worker_id}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self._steps: list[dict] = []

    def log_step(self, row: Mapping[str, Any]) -> None:
        validate_step(row)
        self._steps.append(dict(row))

    def write_episode(self, summary: Mapping[str, Any]) -> tuple[Path, Path]:
        """Validates and writes the buffered steps plus the episode summary,
        then clears the buffer so this logger is ready for the next episode.
        Returns (steps_path, summary_path)."""
        validate_episode(summary)
        episode_id = summary["episode_id"]

        steps_path = self.dir / f"episode_{episode_id}_steps.parquet"
        summary_path = self.dir / f"episode_{episode_id}_summary.parquet"

        pd.DataFrame(self._steps).to_parquet(steps_path, index=False)
        pd.DataFrame([dict(summary)]).to_parquet(summary_path, index=False)

        self._steps = []
        return steps_path, summary_path
