"""M4 task 2: WorkerSupervisor owns one worker's processes -- start, health
check, stop, restart.

Lives entirely in SimFarm's (the parent's) process. It never imports rclpy
and never touches a DDS object, by design: CLAUDE.md §3.3 forbids
rclpy.init() anywhere but inside a spawned worker process, and this class's
job -- starting/stopping the OS-level px4/gz/agent processes via
simulation.worker_process, and managing the worker's child Python process
handle -- never needs one. See experiments/sim_farm.py's module docstring for
the full split between what runs here (the parent) and what runs inside each
worker's own child process.

Health checks implement docs/parallelism.md §8's catalogue, to the extent
that is possible from OUTSIDE the worker's process:
  - PID alive: instance_<N>.json's runtime.pid_gz/pid_agent/pid_px4, checked
    with a zero-signal kill (no permission to send anything, just probes
    existence). This is the one check performable with no cooperation from
    the child at all.
  - Child process alive: multiprocessing.Process.is_alive() / .exitcode.
  - Telemetry stalled / DDS link down: the child owns the only rclpy
    connection that could observe this, so it cannot be checked directly
    from here. Instead the child writes a small heartbeat file
    (heartbeat_path) on every control tick (mission_executor's existing
    on_step callback -- no new hook needed in fly_mission); a heartbeat
    older than `heartbeat_stall_timeout_s` is treated the same as a dead PID.

ensure_healthy() reports whether it restarted, so SimFarm can invalidate the
in-flight episode -- the milestone's own required contract.
"""
from __future__ import annotations

import json
import multiprocessing
import os
import signal
import time
from pathlib import Path
from typing import Optional

from simulation.instance_spec import InstanceSpec
from simulation.worker_process import WorkerProcessError, start_worker, stop_worker

DEFAULT_RUN_DIR = os.environ.get("AERO_RUN_DIR", "/tmp/aero-safe-rl-sim")

# How long to wait for a child process to exit on its own after terminate()
# before escalating to SIGKILL -- generous enough for rclpy.shutdown() to run
# in the child's own finally block, short enough not to hang a restart.
CHILD_JOIN_TIMEOUT_S = 10.0


class WorkerSupervisorError(RuntimeError):
    """start()/stop() failed, or ensure_healthy() exhausted its restart budget."""


class RestartBudgetExhausted(WorkerSupervisorError):
    """A worker needed more restarts than restart_budget_per_worker allows --
    failing loudly here (CLAUDE.md §5) rather than continuing to restart a
    worker that is never going to stay up, which would silently bias the
    dataset (docs/parallelism.md §8)."""


def _pid_alive(pid: Optional[int]) -> bool:
    """Zero-signal kill: probes existence without sending anything. Raises
    ProcessLookupError for a genuinely dead pid; a PermissionError means the
    process exists but is owned by someone else, which still counts as
    "alive" here (this project never runs a worker as a different user, but
    treating PermissionError as "dead" would be the wrong answer if it ever
    happened)."""
    if pid is None:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _read_runtime_pids(instance: int, run_dir: str) -> dict[str, Optional[int]]:
    path = Path(run_dir) / f"instance_{instance}.json"
    if not path.exists():
        return {"pid_gz": None, "pid_agent": None, "pid_px4": None}
    data = json.loads(path.read_text())
    runtime = data.get("runtime", {})
    return {
        "pid_gz": runtime.get("pid_gz"),
        "pid_agent": runtime.get("pid_agent"),
        "pid_px4": runtime.get("pid_px4"),
    }


def _heartbeat_path(instance: int, run_dir: str) -> Path:
    return Path(run_dir) / f"worker_{instance}_heartbeat.json"


def write_heartbeat(instance: int, run_dir: str, *, last_odometry_wall_s: float) -> None:
    """Called from INSIDE a worker's child process (experiments/sim_farm.py's
    _worker_main), once per control tick, via the same on_step callback
    EpisodeLogger already uses. One writer per file (CLAUDE.md §3.5) -- only
    this worker's own child process ever writes its heartbeat file."""
    path = _heartbeat_path(instance, run_dir)
    path.write_text(json.dumps({"t_wall": time.time(), "last_odometry_wall_s": last_odometry_wall_s}))


def _heartbeat_age_s(instance: int, run_dir: str) -> Optional[float]:
    path = _heartbeat_path(instance, run_dir)
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text())
    except (json.JSONDecodeError, OSError):
        return None
    return time.time() - data.get("t_wall", 0.0)


class WorkerSupervisor:
    """One instance per worker. `process` is the multiprocessing.Process
    handle for that worker's child (set by SimFarm after spawning it) --
    WorkerSupervisor does not create the child itself, since the picklable
    entry point and its arguments are SimFarm's concern, but it owns
    deciding whether that child (and the OS processes under it) are healthy
    and orchestrating restarts of either or both."""

    def __init__(self, spec: InstanceSpec, *, run_dir: str = DEFAULT_RUN_DIR,
                 heartbeat_stall_timeout_s: float = 45.0, restart_budget: int = 5):
        self.spec = spec
        self.run_dir = run_dir
        self.heartbeat_stall_timeout_s = heartbeat_stall_timeout_s
        self.restart_budget = restart_budget
        self.process: Optional[multiprocessing.Process] = None
        self.restart_count = 0

    # ------------------------------------------------------------- lifecycle

    def start(self) -> None:
        """Starts this worker's OS processes (px4/gz/agent) via sim_start.sh.
        Does NOT spawn the child Python process -- SimFarm does that
        separately and assigns the handle to self.process, since only
        SimFarm knows the picklable worker_main function and its arguments.

        Clears any pre-existing heartbeat file for this instance first.
        Found live: run_dir (like sim_start.sh's own instance_<N>.json) is a
        SHARED, PERSISTENT location across runs, not per-run -- a heartbeat
        file left over from a previous run (or a previous attempt at this
        same instance) is old enough to look "stale" to is_healthy() before
        the freshly started worker has written its own first heartbeat,
        which triggers a spurious restart of a worker that was never
        actually unhealthy. Confirmed live: this produced a real duplicate-
        episode bug (the same worker flying and recording twice) before this
        fix, in the M4 sim-marked gate test."""
        _heartbeat_path(self.spec.instance, self.run_dir).unlink(missing_ok=True)
        try:
            start_worker(self.spec, run_dir=self.run_dir)
        except WorkerProcessError as exc:
            raise WorkerSupervisorError(f"worker {self.spec.instance}: start failed: {exc}") from exc

    def stop(self) -> None:
        """Stops this worker's child process (if any) and its OS processes.
        Never uses sim_stop.sh --all/--sweep -- only this worker's own
        recorded PIDs (CLAUDE.md anti-pattern 7)."""
        self._stop_child_process()
        try:
            stop_worker(self.spec.instance, run_dir=self.run_dir)
        except WorkerProcessError as exc:
            raise WorkerSupervisorError(f"worker {self.spec.instance}: stop failed: {exc}") from exc

    def _stop_child_process(self) -> None:
        if self.process is None or not self.process.is_alive():
            return
        self.process.terminate()
        self.process.join(timeout=CHILD_JOIN_TIMEOUT_S)
        if self.process.is_alive():
            os.kill(self.process.pid, signal.SIGKILL)
            self.process.join(timeout=CHILD_JOIN_TIMEOUT_S)

    # --------------------------------------------------------------- health

    def is_healthy(self) -> bool:
        """True iff every layer this supervisor can observe looks alive:
        the three OS processes' recorded PIDs, the child Python process, and
        a recent-enough heartbeat. Any one failing means the worker is not
        healthy -- ensure_healthy() decides what to do about it."""
        pids = _read_runtime_pids(self.spec.instance, self.run_dir)
        if not all(_pid_alive(pid) for pid in pids.values()):
            return False
        if self.process is not None and not self.process.is_alive():
            return False
        age = _heartbeat_age_s(self.spec.instance, self.run_dir)
        # No heartbeat file yet is not itself unhealthy -- a worker that has
        # started but not yet flown its first control tick (still arming,
        # still resetting) has nothing to have written. Only a heartbeat
        # that EXISTS and is stale counts as a stall.
        if age is not None and age > self.heartbeat_stall_timeout_s:
            return False
        return True

    def ensure_healthy(self) -> bool:
        """Checks health and restarts if needed. Returns True iff a restart
        happened (the caller must invalidate the in-flight episode in that
        case). Raises RestartBudgetExhausted if this worker has already used
        up its restart budget -- a worker that cannot stay up is a run-level
        problem, not something to keep silently retrying forever."""
        if self.is_healthy():
            return False

        if self.restart_count >= self.restart_budget:
            raise RestartBudgetExhausted(
                f"worker {self.spec.instance}: {self.restart_count} restarts already "
                f"used (budget {self.restart_budget}); this worker will not stay up")

        self.restart_count += 1
        self._stop_child_process()
        try:
            stop_worker(self.spec.instance, run_dir=self.run_dir)
        except WorkerProcessError:
            # Already dead is fine -- stop_worker's job here is "make sure
            # nothing is left running", and a worker we're restarting BECAUSE
            # it died is expected to fail a clean stop sometimes.
            pass
        self.start()
        return True
