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

**Any script that constructs a SimFarm must guard that code with
`if __name__ == "__main__":`.** This is not a style preference here -- the
spawn start method re-imports the launching script in every child process to
bootstrap it, so a SimFarm(...) call sitting at a script's top level runs
AGAIN, recursively, inside every worker process it creates (which then tries
to spawn its own children, ad infinitum in principle; in practice the nested
attempts fail fast because sim_start.sh refuses an already-running instance
-- but not before wasting real time and CPU competing with the real run).
Found live while building vis_sim.md: an unguarded example script produced
exactly this -- workers started correctly, but the recursive re-imports
piled enough CPU contention onto the real run that scripts/env_report.sh's
own 30s subprocess timeout in __init__ (see below) tripped, which looked
like a crash. run_episodes.py already does this correctly (see its own
`if __name__ == "__main__":` block) -- follow that pattern, always.

M4 task 4 (structured failure handling) closed here: when a restart happens
mid-episode, `_on_restart` now synthesizes a `valid=false,
termination_reason=worker_restarted` placeholder record for the exact
episode that was in flight (`_synthesize_lost_episode_record`), via a
standalone `EpisodeLogger` -- no rclpy needed for that, so it's safe to do
from this (parent) process. It also enforces a run-level restart-rate
abort (`restart_rate_abort_threshold`): a run whose total restarts across
all workers exceed that fraction of total planned episodes fails loudly
rather than quietly producing a dataset biased toward whichever fault
severities crash workers more often (docs/parallelism.md §8).

M4 task 5 also closed here: every run writes `results/<run_id>/manifest.json`
(`experiments/run_manifest.py`'s `RunManifest`) incrementally -- after every
episode and every restart, not only at the end -- so a killed run still
leaves a readable manifest. `run()` also takes optional `on_result`/
`on_restart` progress callbacks, fired from the same two points
(`_drain_queue`/`_on_restart`) the manifest updates from; task 6's throughput
sweep and task 7's soak test both use these to print live per-episode
progress rather than going silent for the length of a whole run.

Still out of scope: the throughput sweep (task 6), the soak test (task 7).
"""
from __future__ import annotations

import json
import multiprocessing
import queue as queue_module
import time
from pathlib import Path
from typing import Optional

from experiments.episode_runner import _iso, capture_env_versions
from experiments.run_manifest import RunManifest
from experiments.worker_supervisor import (
    DEFAULT_RUN_DIR,
    RestartBudgetExhausted,
    WorkerSupervisor,
    WorkerSupervisorError,
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
                 heartbeat_stall_timeout_s: float = 45.0, restart_budget_per_worker: int = 20,
                 restart_rate_abort_threshold: float = 0.5,
                 stagger_s: float = 3.0, health_poll_interval_s: float = 2.0,
                 max_wall_s: Optional[float] = None):
        import datetime
        import uuid

        from aero_bridge.mission_executor import load_mission
        from experiments.episode_schema import digest

        # Loaded and captured HERE, before any worker starts (even before
        # __enter__), not in run(). Found live: capture_env_versions() shells
        # out to scripts/env_report.sh with a 30s subprocess timeout, and
        # env_report.sh itself runs several subprocess calls of its own (gz,
        # git, python); when run() used to call it AFTER __enter__ had
        # already started GUI workers, two concurrent GUI-rendered Gazebo
        # instances contended for CPU heavily enough (load average >10 on a
        # 12-thread machine, measured) that env_report.sh missed its 30s
        # budget, raised, and unwound the whole `with` block -- stopping both
        # freshly started workers and exiting, which looked exactly like a
        # crash. A bad mission_id now also fails here, before any worker is
        # started, rather than inside run() after workers are already up.
        mission_path = REPO / "configs" / "missions" / f"{mission_id}.yaml"
        self._mission = load_mission(mission_path)
        self._mission_digest = digest(self._mission)
        self._env_versions_json = json.dumps(capture_env_versions(), sort_keys=True)

        self.worker_count = worker_count
        self.mission_id = mission_id
        self.n_episodes_per_worker = n_episodes_per_worker
        self.seed_base = seed_base
        self.reset_tier = reset_tier
        self.results_dir = results_dir
        self.sim_run_dir = sim_run_dir
        self.stagger_s = stagger_s
        self.health_poll_interval_s = health_poll_interval_s
        # M4 task 4 (docs/parallelism.md §8): total restarts across every
        # worker, divided by total planned episodes, past this fraction
        # aborts the whole run loudly rather than quietly finishing with a
        # dataset biased toward whichever fault severities crash workers
        # more often. Checked in _on_restart(), which every restart path
        # funnels through.
        self.restart_rate_abort_threshold = restart_rate_abort_threshold
        self.restart_budget_per_worker = restart_budget_per_worker
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
        # M4 tasks 6/7 found live: multiprocessing.Queue.put() can return
        # before the item is actually flushed through the underlying pipe --
        # if the child process then dies right after (e.g. its own
        # rclpy.shutdown() raises under heavy multi-worker resource
        # contention), the parent can conclude the worker died with that
        # episode still missing, synthesize a worker_restarted placeholder
        # for it, and THEN have the real, late-arriving result turn up in a
        # later _drain_queue() call -- two conflicting records for the same
        # (worker_id, episode_id). Tracked here so _record_result can keep
        # exactly one record per episode regardless of arrival order.
        self._recorded_episode_ids: set[tuple[int, str]] = set()
        self._ctx = multiprocessing.get_context("spawn")
        self._result_queue = self._ctx.Queue()

        # M4 task 5: one manifest per run, written incrementally (see
        # run_manifest.py). worker_to_instance uses the SAME k->instance
        # mapping worker_instance() derives self.specs from, so it's read
        # back here rather than recomputed.
        worker_to_instance = {k: spec.instance for k, spec in enumerate(self.specs)}
        self.manifest = RunManifest(
            run_id=self.run_id, mission_id=self.mission_id,
            mission_config_digest=self._mission_digest, env_versions_json=self._env_versions_json,
            seed_base=self.seed_base, worker_to_instance=worker_to_instance,
            restart_budget_per_worker=self.restart_budget_per_worker,
            restart_rate_abort_threshold=self.restart_rate_abort_threshold)
        self._manifest_path = Path(self.results_dir) / self.run_id / "manifest.json"
        self.manifest.write(self._manifest_path)

        # M4 tasks 6/7's progress callbacks (optional; set for real by run()).
        # Initialized here, not only in run(), because _on_restart() and
        # _synthesize_lost_episode_record() are also called directly outside
        # a run() invocation in this module's own unit tests.
        self._progress_on_result = None
        self._progress_on_restart = None

    # ------------------------------------------------------------- context

    def __enter__(self) -> "SimFarm":
        for i, sup in enumerate(self.supervisors):
            if i > 0:
                time.sleep(self.stagger_s)
            self._start_with_retries(sup)
        return self

    def _start_with_retries(self, sup: WorkerSupervisor, *, max_attempts: int = 3) -> None:
        """The initial start (unlike a mid-run restart, which already has
        ensure_healthy()'s restart_budget) had no retry budget at all: one
        transient cold-start hiccup failed the entire run before a single
        episode flew. Found live (M4 task 7's soak test): simultaneous
        4-worker startup occasionally times out waiting for the first
        worker's telemetry (~90s deadline) under contention -- observed on
        two different workers on two different attempts, so it's a real,
        reproducible startup-contention issue, not a single flaky instance.
        A small, separate retry budget here (not reusing
        restart_budget_per_worker, which is about a worker that died
        mid-run, not one that never came up) gives a transient hiccup a
        chance to clear rather than failing the whole run on the first one.
        """
        last_exc: Optional[WorkerSupervisorError] = None
        for attempt in range(1, max_attempts + 1):
            try:
                sup.start()
                return
            except WorkerSupervisorError as exc:
                last_exc = exc
                print(f"WARNING: worker {sup.spec.instance}: initial start attempt "
                      f"{attempt}/{max_attempts} failed: {exc}", flush=True)
        raise SimFarmError(
            f"worker {sup.spec.instance}: failed to start after {max_attempts} attempts"
        ) from last_exc

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

    def run(self, *, on_result=None, on_restart=None) -> list[dict]:
        """Flies every worker's share of episodes concurrently, restarting
        failures via WorkerSupervisor, and returns every summary collected
        (in arrival order, not per-worker order). Must be called inside the
        `with SimFarm(...) as farm:` block, after __enter__ has started the
        workers' OS processes.

        `on_result(record)`, if given, is called once per episode record
        (real or a synthesized worker_restarted placeholder) as soon as it
        lands; `on_restart(instance)` once per restart. Both exist so a long
        run (M4 tasks 6/7's throughput sweep and soak test) can print live
        per-episode progress instead of going silent for the run's whole
        duration -- EpisodeRunner/SimFarm stay ignorant of what a caller does
        with the callback, same pattern as EpisodeRunner's own on_step."""
        self._progress_on_result = on_result
        self._progress_on_restart = on_restart

        for sup in self.supervisors:
            self._spawn_worker(sup, start_index=0, n_episodes=self.n_episodes_per_worker,
                                mission=self._mission, mission_digest=self._mission_digest,
                                env_versions_json=self._env_versions_json)

        deadline = time.monotonic() + self.max_wall_s
        while True:
            self._drain_queue()
            if all(not sup.process.is_alive() for sup in self.supervisors):
                self._drain_queue()  # final drain after every child has exited
                self.manifest.finalize(self._manifest_path)
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
                self._restart_and_resume(sup, self._mission, self._mission_digest,
                                          self._env_versions_json)

            for sup in self.supervisors:
                if not sup.process.is_alive():
                    continue
                try:
                    if sup.ensure_healthy():
                        self._on_restart(sup, self._mission, self._mission_digest,
                                          self._env_versions_json)
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

    def _synthesize_lost_episode_record(self, sup: WorkerSupervisor) -> None:
        """M4 task 4.2: the episode a dead child never got to report is not
        silently dropped. Writes a valid=false, termination_reason=
        worker_restarted placeholder for it via a standalone EpisodeLogger
        (needs no rclpy, so this is safe to call from SimFarm's own parent
        process) and appends it to self.results, then advances
        _completed_per_worker so the NEXT spawned child's start_index skips
        past this episode's id rather than overwriting the placeholder's
        already-written parquet files.

        A no-op when the worker had already cleanly finished its whole
        quota before dying -- nothing was actually in flight to lose."""
        instance = sup.spec.instance
        completed = self._completed_per_worker[instance]
        if completed >= self.n_episodes_per_worker:
            return

        from aero_bridge.episode_logger import EpisodeLogger
        from experiments.episode_schema import FEATURE_VERSION_UNSET, SCHEMA_VERSION, TerminationReason, digest

        episode_id = f"ep_{completed:04d}"
        now_iso = _iso(time.time())
        record = dict(
            schema_version=SCHEMA_VERSION, run_id=self.run_id, episode_id=episode_id,
            worker_id=instance, instance=instance, seed=self.seed_base + completed,
            instance_spec_digest=digest(sup.spec.to_dict()), mission_id=self.mission_id,
            mission_config_digest=self._mission_digest, feature_version=FEATURE_VERSION_UNSET,
            env_versions=self._env_versions_json,
            # The reset tier the dead child would actually have used for this
            # episode isn't observable from this (parent) process -- record
            # the run's configured tier rather than guessing at an escalation
            # decision that only ever happens inside the child.
            reset_tier=self.reset_tier,
            termination_reason=TerminationReason.WORKER_RESTARTED.value, valid=False,
            t_sim_start_s=0.0, t_sim_end_s=0.0, t_sim_duration_s=0.0,
            t_wall_start_utc=now_iso, t_wall_end_utc=now_iso, t_wall_duration_s=0.0,
            n_steps=0, waypoints_reached=0,
            position_rmse_m=float('nan'), final_position_error_m=float('nan'),
        )
        EpisodeLogger(self.run_id, worker_id=instance, results_dir=self.results_dir).write_episode(record)
        # Always newly-recorded in practice (the completed-quota guard above
        # already prevents re-synthesizing the same index twice), but go
        # through the same dedup-aware path as every other record rather
        # than assuming that.
        if self._record_result(record):
            self._completed_per_worker[instance] += 1

    def _on_restart(self, sup: WorkerSupervisor, mission, mission_digest, env_versions_json) -> None:
        """ensure_healthy() already restarted the OS-level px4/gz/agent
        processes and stopped the old child. Before respawning a new child,
        records the episode that was in flight (if any) as lost, and checks
        whether this run's total restart rate has crossed the abort
        threshold -- a run already over the threshold doesn't get one more
        doomed worker started. The new child continues the episode
        index/seed sequence rather than restarting it (see _worker_main's
        docstring)."""
        self._synthesize_lost_episode_record(sup)
        self._record_restart(sup.spec.instance)

        total_restarts = sum(self.restart_counts.values())
        total_planned = self.worker_count * self.n_episodes_per_worker
        if total_planned > 0 and total_restarts / total_planned > self.restart_rate_abort_threshold:
            raise SimFarmError(
                f"restart rate {total_restarts}/{total_planned} exceeded "
                f"restart_rate_abort_threshold={self.restart_rate_abort_threshold} -- "
                f"aborting rather than risk a dataset biased toward whichever fault "
                f"severities crash workers more often (docs/parallelism.md §8)")

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

    def _record_result(self, record: dict) -> bool:
        """Shared by _drain_queue (real episodes) and
        _synthesize_lost_episode_record (placeholders): appends to
        self.results, updates the manifest, and fires the progress callback
        run() was given -- the single point every episode record passes
        through, real or synthesized.

        Returns True iff this record was newly recorded. Returns False, and
        drops it, if this exact (worker_id, episode_id) was already
        recorded -- the dedup that closes the queue race documented on
        self._recorded_episode_ids in __init__: whichever of a synthesized
        placeholder / a late-arriving real result gets here FIRST wins."""
        key = (record["worker_id"], record["episode_id"])
        if key in self._recorded_episode_ids:
            print(f"WARNING: dropping duplicate episode record for worker "
                  f"{record['worker_id']} {record['episode_id']} "
                  f"(termination_reason={record['termination_reason']!r}) -- "
                  f"already recorded; see sim_farm.py's queue-race note.", flush=True)
            return False
        self._recorded_episode_ids.add(key)
        self.results.append(record)
        self.manifest.record_episode(record)
        self.manifest.write(self._manifest_path)
        if self._progress_on_result is not None:
            self._progress_on_result(record)
        return True

    def _record_restart(self, instance: int) -> None:
        self.restart_counts[instance] += 1
        self.manifest.record_restart(instance)
        self.manifest.write(self._manifest_path)
        if self._progress_on_restart is not None:
            self._progress_on_restart(instance)

    def _drain_queue(self) -> None:
        while True:
            try:
                summary = self._result_queue.get_nowait()
            except queue_module.Empty:
                return
            if self._record_result(summary):
                self._completed_per_worker[summary["worker_id"]] += 1
