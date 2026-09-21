"""Starting and stopping one worker's OS processes -- the one place in this
repository that shells out to scripts/sim_start.sh and scripts/sim_stop.sh.

Extracted from aero_bridge/reset.py's hard_reset() (M3), which needed this
exact stop+start sequence for a mid-run reset, and experiments/worker_supervisor.py
(M4), which needs it for a worker's initial start and for restart-after-failure.
Two call sites needing the identical subprocess invocation is exactly the
"second implementation" CLAUDE.md §1.4 forbids -- this module exists so there
is one.

Deliberately dependency-free (standard library only, no rclpy): starting/
stopping the OS-level px4/gz/agent processes never needs a DDS connection,
and keeping it that way is what lets WorkerSupervisor call this from the
PARENT process without violating CLAUDE.md §3.3 (rclpy only inside a spawned
worker process).
"""
from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Optional

from simulation.instance_spec import InstanceSpec

REPO_DIR = Path(__file__).resolve().parent.parent

DEFAULT_START_TIMEOUT_S = 120.0
DEFAULT_STOP_TIMEOUT_S = 60.0


class WorkerProcessError(RuntimeError):
    """sim_start.sh or sim_stop.sh exited non-zero, or timed out."""


def start_worker(spec: InstanceSpec, *, run_dir: Optional[str] = None,
                  repo_dir: Path = REPO_DIR, timeout_s: float = DEFAULT_START_TIMEOUT_S) -> None:
    """Runs scripts/sim_start.sh for `spec`. Raises WorkerProcessError on any
    non-zero exit or timeout; never silently leaves a half-started worker
    unreported.

    Passes --gui when spec.headless is False. InstanceSpec has carried a
    headless field since M1b, but nothing read it here until this was found
    while building a visualization walkthrough (vis_sim.md) -- WorkerSupervisor/
    SimFarm-launched workers were silently always headless, with no way to
    watch one in a GUI window, regardless of what the spec said."""
    extra = ["--run-dir", run_dir] if run_dir else []
    if not spec.headless:
        extra.append("--gui")
    try:
        result = subprocess.run(
            [str(repo_dir / "scripts" / "sim_start.sh"),
             "-i", str(spec.instance), "-w", spec.world, "-m", spec.model,
             "-s", str(spec.speed_factor), *extra],
            capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired as exc:
        raise WorkerProcessError(
            f"start_worker: sim_start.sh -i {spec.instance} timed out after {timeout_s}s"
        ) from exc
    if result.returncode != 0:
        raise WorkerProcessError(
            f"start_worker: sim_start.sh -i {spec.instance} failed:\n"
            f"{result.stdout}\n{result.stderr}")


def stop_worker(instance: int, *, run_dir: Optional[str] = None,
                 repo_dir: Path = REPO_DIR, timeout_s: float = DEFAULT_STOP_TIMEOUT_S) -> None:
    """Runs scripts/sim_stop.sh -i `instance`. Raises WorkerProcessError on
    any non-zero exit or timeout. Never uses the --all/--sweep sweep -- that
    would stop every OTHER worker too (CLAUDE.md anti-pattern 7)."""
    extra = ["--run-dir", run_dir] if run_dir else []
    try:
        result = subprocess.run(
            [str(repo_dir / "scripts" / "sim_stop.sh"), "-i", str(instance), *extra],
            capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired as exc:
        raise WorkerProcessError(
            f"stop_worker: sim_stop.sh -i {instance} timed out after {timeout_s}s"
        ) from exc
    if result.returncode != 0:
        raise WorkerProcessError(
            f"stop_worker: sim_stop.sh -i {instance} failed:\n"
            f"{result.stdout}\n{result.stderr}")
