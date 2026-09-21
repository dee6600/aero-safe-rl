"""M4 task 3: SimFarm owns N workers -- deterministic worker->instance
assignment, starting them all, flying each its share of episodes
concurrently, restarting failures via WorkerSupervisor, and guaranteeing
every worker is stopped on exit (including on exception or Ctrl-C).

The parent/child split this module implements, and why (CLAUDE.md §3.3 is
absolute: rclpy.init() only inside a spawned worker process, never in the
parent):

    SimFarm (THIS process)                  each worker's OWN child process
    ------------------------                -----------------------------
    WorkerSupervisor.start()/stop()          rclpy.init()
      (shells to sim_start.sh/sim_stop.sh,   EpisodeRunner (owns the Node,
       no rclpy -- simulation/worker_process) PX4Interface, GzSimClock)
    ensure_healthy() (PID + heartbeat-file   flies its assigned episodes,
      staleness checks, no rclpy)             pushes each summary onto a
    multiprocessing.Process handle            shared multiprocessing.Queue,
      (start/is_alive/terminate/join)         writes a heartbeat file on
                                               every control tick

_worker_main is the picklable entry point spawned for each worker (spawn
start method, set explicitly -- never the platform default, CLAUDE.md §3.3).
Nothing above this module re-implements "fly a worker's episodes"; it is a
caller of EpisodeRunner and next_reset_tier(), exactly like run_episodes.py.

Explicitly out of scope for this session (M4 tasks 1-3; see milestones.md):
task 4's full structured-failure handling -- this module restarts a dead
worker and counts the restart, but does not yet synthesize a placeholder
`valid=false, termination_reason=worker_restarted` record for the exact
episode that was in flight when the worker died (that episode's result is
simply never produced, which is a real gap task 4 closes, not silently
ignored -- restart_counts already gives visibility that it happened). Also
out of scope: the run manifest (task 5), the throughput sweep (task 6), the
soak test (task 7).
"""
from __future__ import annotations

import json
import multiprocessing
import queue as queue_module
import time
from pathlib import Path
from typing import Optional

from experiments.episode_runner import capture_env_versions
from experiments.worker_supervisor import (
    DEFAULT_RUN_DIR,
    RestartBudgetExhausted,
    WorkerSupervisor,
    write_heartbeat,
)
from simulation.instance_spec import InstanceSpec

REPO = Path(__file__).resolve().parent.parent

# Generous per-episode ceiling used to derive SimFarm's overall wall-clock
# watchdog when the caller doesn't supply one explicitly: hard reset measured
# at ~19.8s (M3) + a full mission (~42-53s measured, M3) + worker startup
# margin. This is a hang watchdog for the WHOLE run (CLAUDE.md §5 -- no
# unbounded loop), not a mission-duration budget.
DEFAULT_PER_EPISODE_WALL_BUDGET_S = 180.0


class SimFarmError(RuntimeError):
    """A worker failed to start/stop, exhausted its restart budget, or the
    run exceeded its overall wall-clock watchdog."""


def worker_instance(k: int, instance_base: int) -> int:
    """Worker k's PX4 instance id -- a pure function, never a free-port or
    free-domain search (CLAUDE.md anti-pattern 6). Deterministic across
    processes and runs: worker k always gets instance_base + k."""
    return instance_base + k


def _worker_main(spec: InstanceSpec, mission_id: str, mission: dict, mission_digest: str,
                  env_versions_json: str, n_episodes: int, start_index: int, seed_base: int,
                  reset_tier: str, run_id: str, results_dir: str, sim_run_dir: str,
                  result_queue) -> None:
    """Picklable entry point for one worker's child process (spawn start
    method). Flies `n_episodes` episodes starting at global episode index
    `start_index` (NOT always 0 -- a worker respawned after a restart must
    continue numbering from where the dead attempt left off, or its fresh
    ep_0000/ep_0001/... would silently overwrite the previous attempt's
    already-written parquet files under the same worker_<k>/ directory).

    Sets ROS_DOMAIN_ID before importing rclpy (must happen before the DDS
    layer initialises -- D9) and calls rclpy.init() here, inside the child,
    never in SimFarm's own process.
    """
    import os
    os.environ['ROS_DOMAIN_ID'] = str(spec.ros_domain_id)

    import rclpy

    from aero_bridge.reset import ResetError
    from experiments.episode_runner import EpisodeRunner, next_reset_tier
    from experiments.episode_schema import FEATURE_VERSION_UNSET

    rclpy.init()
    runner = None
    try:
        runner = EpisodeRunner(spec, run_id=run_id, mission_id=mission_id, mission=mission,
                                mission_digest=mission_digest, env_versions_json=env_versions_json,
                                results_dir=results_dir, feature_version=FEATURE_VERSION_UNSET)

        def heartbeat_on_step(row):
            write_heartbeat(spec.instance, sim_run_dir, last_odometry_wall_s=row["t_wall_utc"])

        last_termination_reason = None
        for i in range(n_episodes):
            episode_id = f"ep_{start_index + i:04d}"
            seed = seed_base + start_index + i
            # Nothing to reset from on this child's own first episode --
            # matches run_episodes.py's identical convention. This is still
            # correct after a restart: the OS-level px4/gz processes were
            # JUST freshly started by WorkerSupervisor.ensure_healthy(), so
            # there is genuinely nothing to reset from here either.
            effective_tier = "none" if i == 0 else next_reset_tier(reset_tier, last_termination_reason)
            try:
                summary = runner.run_episode(episode_id=episode_id, reset_tier=effective_tier,
                                              seed=seed, on_step=heartbeat_on_step)
            except ResetError:
                summary = runner.run_episode(episode_id=episode_id, reset_tier="hard",
                                              seed=seed, on_step=heartbeat_on_step)
            result_queue.put(summary)
            last_termination_reason = summary["termination_reason"]
    finally:
        if runner is not None:
            runner.close()
        rclpy.shutdown()


class SimFarm:
    """Context manager owning N workers. Usage:

        with SimFarm(worker_count=2, mission_id="square_circuit",
                     n_episodes_per_worker=50) as farm:
            results = farm.run()

    __enter__ starts every worker's OS processes (staggered -- simultaneous
    starts contend for CPU during EKF convergence and can time out readiness
    checks that pass fine one at a time). __exit__ always stops every worker,
    including when the with-block raised -- best-effort per worker, so one
    worker's stop failing does not prevent the others from being stopped.
    """

    def __init__(self, worker_count: int, mission_id: str, n_episodes_per_worker: int, *,
                 instance_base: int = 0, seed_base: int = 0, reset_tier: str = "soft",
                 speed_factor: float = 1.0, headless: bool = True, run_id: Optional[str] = None,
                 results_dir: str = "results", sim_run_dir: str = DEFAULT_RUN_DIR,
                 heartbeat_stall_timeout_s: float = 45.0, restart_budget_per_worker: int = 5,
                 stagger_s: float = 3.0, health_poll_interval_s: float = 2.0,
                 max_wall_s: Optional[float] = None):
        import datetime
        import uuid

        self.worker_count = worker_count
        self.mission_id = mission_id
        self.n_episodes_per_worker = n_episodes_per_worker
        self.seed_base = seed_base
        self.reset_tier = reset_tier
        self.results_dir = results_dir
        self.sim_run_dir = sim_run_dir
        self.stagger_s = stagger_s
        self.health_poll_interval_s = health_poll_interval_s
        self.max_wall_s = (max_wall_s if max_wall_s is not None
                            else n_episodes_per_worker * DEFAULT_PER_EPISODE_WALL_BUDGET_S)
        self.run_id = run_id or (
            f"run_{datetime.datetime.now(tz=datetime.timezone.utc):%Y%m%dT%H%M%SZ}_{uuid.uuid4().hex[:6]}")

        self.specs = [InstanceSpec.for_instance(worker_instance(k, instance_base),
                                                 speed_factor=speed_factor, headless=headless)
                      for k in range(worker_count)]
        self.supervisors = [
            WorkerSupervisor(spec, run_dir=sim_run_dir,
                              heartbeat_stall_timeout_s=heartbeat_stall_timeout_s,
                              restart_budget=restart_budget_per_worker)
            for spec in self.specs
        ]
        self.results: list[dict] = []
        self.restart_counts: dict[int, int] = {spec.instance: 0 for spec in self.specs}
        self._completed_per_worker: dict[int, int] = {spec.instance: 0 for spec in self.specs}
        self._ctx = multiprocessing.get_context("spawn")
        self._result_queue = self._ctx.Queue()

    # ------------------------------------------------------------- context

    def __enter__(self) -> "SimFarm":
        for i, sup in enumerate(self.supervisors):
            if i > 0:
                time.sleep(self.stagger_s)
            sup.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        errors = []
        for sup in self.supervisors:
            try:
                sup.stop()
            except Exception as e:  # noqa: BLE001 -- every worker must get a stop attempt
                errors.append((sup.spec.instance, e))
        for instance, e in errors:
            print(f"WARNING: failed to stop worker {instance}: {e}")
        if errors and exc_type is None:
            # Only surface a stop failure as the run's own exception if
            # nothing else was already propagating -- a real exception from
            # inside the with-block always takes priority.
            raise SimFarmError(f"failed to stop workers: {[i for i, _ in errors]}")
        return False  # never swallow an exception from inside the with-block

    # ------------------------------------------------------------------ run

    def run(self) -> list[dict]:
        """Flies every worker's share of episodes concurrently, restarting
        failures via WorkerSupervisor, and returns every summary collected
        (in arrival order, not per-worker order). Must be called inside the
        `with SimFarm(...) as farm:` block, after __enter__ has started the
        workers' OS processes."""
        from aero_bridge.mission_executor import load_mission
        from experiments.episode_schema import digest

        mission_path = REPO / "configs" / "missions" / f"{self.mission_id}.yaml"
        mission = load_mission(mission_path)
        mission_digest = digest(mission)
        env_versions_json = json.dumps(capture_env_versions(), sort_keys=True)

        for sup in self.supervisors:
            self._spawn_worker(sup, start_index=0, n_episodes=self.n_episodes_per_worker,
                                mission=mission, mission_digest=mission_digest,
                                env_versions_json=env_versions_json)

        deadline = time.monotonic() + self.max_wall_s
        while True:
            self._drain_queue()
            if all(not sup.process.is_alive() for sup in self.supervisors):
                self._drain_queue()  # final drain after every child has exited
                break
            if time.monotonic() > deadline:
                raise SimFarmError(
                    f"SimFarm.run() exceeded its {self.max_wall_s}s wall-clock watchdog "
                    f"with {sum(self._completed_per_worker.values())} episodes completed")

            for sup in self.supervisors:
                if sup.process.is_alive():
                    continue
                completed = self._completed_per_worker[sup.spec.instance]
                if completed >= self.n_episodes_per_worker:
                    continue  # finished its quota cleanly; nothing to restart
                # Process exited before finishing its quota -- unhealthy by
                # definition (is_healthy() would also catch a hung-but-alive
                # process via the heartbeat-staleness check below).
                self._restart_and_resume(sup, mission, mission_digest, env_versions_json)

            for sup in self.supervisors:
                if not sup.process.is_alive():
                    continue
                try:
                    if sup.ensure_healthy():
                        self._on_restart(sup, mission, mission_digest, env_versions_json)
                except RestartBudgetExhausted as exc:
                    raise SimFarmError(str(exc)) from exc

            time.sleep(self.health_poll_interval_s)

        return self.results

    # --------------------------------------------------------------- helpers

    def _spawn_worker(self, sup: WorkerSupervisor, *, start_index: int, n_episodes: int,
                       mission: dict, mission_digest: str, env_versions_json: str) -> None:
        proc = self._ctx.Process(
            target=_worker_main,
            args=(sup.spec, self.mission_id, mission, mission_digest, env_versions_json,
                  n_episodes, start_index, self.seed_base, self.reset_tier, self.run_id,
                  self.results_dir, self.sim_run_dir, self._result_queue),
            daemon=True,
        )
        proc.start()
        sup.process = proc

    def _on_restart(self, sup: WorkerSupervisor, mission, mission_digest, env_versions_json) -> None:
        """ensure_healthy() already restarted the OS-level px4/gz/agent
        processes and stopped the old child; this respawns a NEW child to
        fly whatever episodes remain, continuing the episode index/seed
        sequence rather than restarting it (see _worker_main's docstring)."""
        self.restart_counts[sup.spec.instance] += 1
        remaining = self.n_episodes_per_worker - self._completed_per_worker[sup.spec.instance]
        if remaining > 0:
            self._spawn_worker(sup, start_index=self._completed_per_worker[sup.spec.instance],
                                n_episodes=remaining, mission=mission, mission_digest=mission_digest,
                                env_versions_json=env_versions_json)

    def _restart_and_resume(self, sup: WorkerSupervisor, mission, mission_digest,
                             env_versions_json) -> None:
        try:
            sup.ensure_healthy()  # process is already dead, so this always restarts
        except RestartBudgetExhausted as exc:
            raise SimFarmError(str(exc)) from exc
        self._on_restart(sup, mission, mission_digest, env_versions_json)

    def _drain_queue(self) -> None:
        while True:
            try:
                summary = self._result_queue.get_nowait()
            except queue_module.Empty:
                return
            self.results.append(summary)
            self._completed_per_worker[summary["worker_id"]] += 1
