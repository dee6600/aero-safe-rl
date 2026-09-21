"""M4 task 5: the one per-run manifest every SimFarm run writes -- run id,
this repo's git SHA, config digests (mission + everything already captured in
env_versions, which carries PX4's own SHA -- see scripts/env_report.sh),
seeds, worker->instance map, episode-outcome counts by termination_reason,
restart counts, and start/end time (CLAUDE.md §7: "every artifact written to
results/ records the versions it was produced under").

Written to results/<run_id>/manifest.json. `.write()` is called after every
update, not just once at the end, so a run killed mid-flight still leaves a
readable partial manifest -- that incremental-write property is the whole
point (a manifest that only exists once a run finishes successfully is no use
for exactly the runs where you need it most).

Pure bookkeeping over data SimFarm already has -- no rclpy, no simulator, safe
to construct and update from SimFarm's own (parent) process. SimFarm owns
calling record_episode()/record_restart()/write()/finalize() at the right
points; see experiments/sim_farm.py.
"""
from __future__ import annotations

import datetime
import json
import subprocess
from pathlib import Path
from typing import Any, Mapping, Optional

REPO = Path(__file__).resolve().parent.parent


def _iso_now() -> str:
    return datetime.datetime.now(tz=datetime.timezone.utc).isoformat()


def _git_sha() -> str:
    """This repo's current commit -- "unknown" rather than raising if git
    isn't available or the tree is somehow not a repo, since a manifest
    field being "unknown" is recoverable and a manifest failing to write at
    all is not."""
    try:
        proc = subprocess.run(["git", "-C", str(REPO), "rev-parse", "HEAD"],
                               capture_output=True, text=True, timeout=10)
        sha = proc.stdout.strip()
        return sha if proc.returncode == 0 and sha else "unknown"
    except (subprocess.SubprocessError, OSError):
        return "unknown"


class RunManifest:
    """One instance per SimFarm run. Every field that's known at construction
    time is captured immediately; episode outcomes and restarts accumulate as
    the run progresses via record_episode()/record_restart()."""

    def __init__(self, *, run_id: str, mission_id: str, mission_config_digest: str,
                 env_versions_json: str, seed_base: int, worker_to_instance: Mapping[int, int],
                 restart_budget_per_worker: int, restart_rate_abort_threshold: float):
        self.run_id = run_id
        self.repo_git_sha = _git_sha()
        self.mission_id = mission_id
        self.mission_config_digest = mission_config_digest
        self.env_versions = json.loads(env_versions_json)
        self.seed_base = seed_base
        self.worker_to_instance = dict(worker_to_instance)
        self.restart_budget_per_worker = restart_budget_per_worker
        self.restart_rate_abort_threshold = restart_rate_abort_threshold
        self.start_time_utc = _iso_now()
        self.end_time_utc: Optional[str] = None
        self.total_episodes = 0
        self.episode_counts_by_outcome: dict[str, int] = {}
        self.restart_counts: dict[int, int] = {instance: 0 for instance in self.worker_to_instance.values()}

    def record_episode(self, summary: Mapping[str, Any]) -> None:
        reason = summary["termination_reason"]
        self.episode_counts_by_outcome[reason] = self.episode_counts_by_outcome.get(reason, 0) + 1
        self.total_episodes += 1

    def record_restart(self, instance: int) -> None:
        self.restart_counts[instance] = self.restart_counts.get(instance, 0) + 1

    def to_dict(self) -> dict[str, Any]:
        return dict(
            run_id=self.run_id, repo_git_sha=self.repo_git_sha,
            mission_id=self.mission_id, mission_config_digest=self.mission_config_digest,
            env_versions=self.env_versions, seed_base=self.seed_base,
            worker_to_instance=self.worker_to_instance,
            restart_budget_per_worker=self.restart_budget_per_worker,
            restart_rate_abort_threshold=self.restart_rate_abort_threshold,
            start_time_utc=self.start_time_utc, end_time_utc=self.end_time_utc,
            total_episodes=self.total_episodes,
            episode_counts_by_outcome=self.episode_counts_by_outcome,
            restart_counts=self.restart_counts,
        )

    def write(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2, sort_keys=True))

    def finalize(self, path: str | Path) -> None:
        """Call once, when the run is over (successfully or not). Sets the
        end time and writes one last time."""
        self.end_time_utc = _iso_now()
        self.write(path)
