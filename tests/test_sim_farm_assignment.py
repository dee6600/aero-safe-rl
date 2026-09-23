"""Unit tests for experiments/sim_farm.py's assignment logic and context-
manager contract (M4 task 3). No rclpy, no simulator: worker->instance
assignment is tested directly against InstanceSpec; the context-manager
tests use fake WorkerSupervisor-shaped objects, never real ones, since real
supervisors call out to sim_start.sh/sim_stop.sh.
"""
import math
import queue

import pytest

from experiments.episode_schema import validate_episode
from experiments.sim_farm import SimFarm, SimFarmError, worker_instance
from experiments.worker_supervisor import WorkerSupervisorError
from simulation.instance_spec import InstanceSpec


# --------------------------------------------------------- worker_instance


def test_worker_to_instance_is_deterministic():
    """Worker k always gets instance_base + k, across repeated calls -- a
    pure function, never a free-port/free-domain search."""
    for _ in range(3):
        assert [worker_instance(k, 0) for k in range(4)] == [0, 1, 2, 3]
        assert [worker_instance(k, 10) for k in range(4)] == [10, 11, 12, 13]


@pytest.mark.parametrize("instance_base", [0, 1, 5, 20])
def test_no_resource_collision_for_n_workers(instance_base):
    """For N up to 8, no two workers' specs share a port, domain, partition,
    namespace or model name."""
    n = 8
    specs = [InstanceSpec.for_instance(worker_instance(k, instance_base)) for k in range(n)]
    for field in ("xrce_port", "ros_domain_id", "gz_partition", "topic_ns", "model_name"):
        values = [getattr(s, field) for s in specs]
        assert len(set(values)) == n, f"{field} collided across {n} workers: {values}"


# --------------------------------------------------- context manager (fakes)


class FakeSupervisor:
    """Stands in for WorkerSupervisor -- start()/stop() are just call
    counters, so __enter__/__exit__ can be tested without sim_start.sh/
    sim_stop.sh or a real process ever existing."""

    def __init__(self, instance, *, fail_stop=False, fail_start_times=0):
        # A real InstanceSpec (not a bare stand-in object) -- task 4's
        # restart-handling tests need a working .to_dict() for the
        # synthesized episode record's instance_spec_digest, and a real
        # spec satisfies every existing use of .instance just as well.
        self.spec = InstanceSpec.for_instance(instance)
        self.start_calls = 0
        self.stop_calls = 0
        self.fail_stop = fail_stop
        # How many of the NEXT start() calls raise before one finally
        # succeeds -- simulates a transient cold-start hiccup for
        # _start_with_retries' tests.
        self.fail_start_times = fail_start_times
        self.process = None

    def start(self):
        self.start_calls += 1
        if self.fail_start_times > 0:
            self.fail_start_times -= 1
            raise WorkerSupervisorError(f"simulated start failure for worker {self.spec.instance}")

    def stop(self):
        self.stop_calls += 1
        if self.fail_stop:
            raise RuntimeError(f"simulated stop failure for worker {self.spec.instance}")


def _farm_with_fakes(monkeypatch, n=2, **fake_kwargs):
    """A SimFarm built the normal way (so worker_count/mission_id/etc. are
    real), then with its supervisors swapped for fakes before the
    with-block runs -- __enter__/__exit__ only ever call .start()/.stop()
    on self.supervisors, so this is enough to test the contract in full.

    capture_env_versions() is stubbed out: SimFarm.__init__ calls it (a real
    subprocess running scripts/env_report.sh, ~1-2s) so that it happens
    before any worker starts rather than contending with one for CPU (found
    live: under two concurrent GUI workers this call missed its own 30s
    timeout and looked like a crash -- see experiments/sim_farm.py's
    __init__ docstring). These tests don't exercise that subprocess at all,
    so stubbing it is what keeps this file's own promise of running in
    milliseconds, not ~2s x however many SimFarm() constructions it does."""
    monkeypatch.setattr("experiments.sim_farm.capture_env_versions", lambda: {"stub": True})
    farm = SimFarm(worker_count=n, mission_id="square_circuit", n_episodes_per_worker=1,
                    stagger_s=0.0)
    farm.supervisors = [FakeSupervisor(k, **fake_kwargs) for k in range(n)]
    return farm


# ------------------------------------------------- M6 task 9: fault_specs

def test_fault_specs_by_worker_wrong_worker_count_raises(monkeypatch):
    from experiments.fault_schedule import FaultSpec
    monkeypatch.setattr("experiments.sim_farm.capture_env_versions", lambda: {"stub": True})
    with pytest.raises(ValueError, match="fault_specs_by_worker"):
        SimFarm(worker_count=2, mission_id="square_circuit", n_episodes_per_worker=3,
                fault_specs_by_worker=[[FaultSpec.healthy(i) for i in range(3)]])  # only 1 entry, need 2


def test_fault_specs_by_worker_wrong_episode_count_raises(monkeypatch):
    from experiments.fault_schedule import FaultSpec
    monkeypatch.setattr("experiments.sim_farm.capture_env_versions", lambda: {"stub": True})
    with pytest.raises(ValueError, match="fault_specs_by_worker\\[1\\]"):
        SimFarm(worker_count=2, mission_id="square_circuit", n_episodes_per_worker=3,
                fault_specs_by_worker=[
                    [FaultSpec.healthy(i) for i in range(3)],
                    [FaultSpec.healthy(i) for i in range(2)],  # wrong length for this worker
                ])


def test_spawn_worker_passes_this_workers_own_fault_specs_slice(monkeypatch):
    from experiments.fault_schedule import FaultProfile, FaultSpec, FaultType

    monkeypatch.setattr("experiments.sim_farm.capture_env_versions", lambda: {"stub": True})
    specs_by_worker = [
        [FaultSpec.healthy(i) for i in range(2)],
        [FaultSpec(episode_index=i, fault_applied=True,
                    fault_type=FaultType.ROTOR_THRUST_DEGRADATION,
                    rotor_index=1, severity=0.5, onset_time_s=1.0, profile=FaultProfile.STEP,
                    ramp_duration_s=0.0) for i in range(2)],
    ]
    farm = SimFarm(worker_count=2, mission_id="square_circuit", n_episodes_per_worker=2,
                    enable_rotor_fault=True, fault_specs_by_worker=specs_by_worker,
                    fault_config_digest="digest123")
    farm.supervisors = [FakeSupervisor(k) for k in range(2)]

    captured_args = []

    class _FakeProcess:
        def __init__(self, target, args, daemon):
            captured_args.append(args)

        def start(self):
            pass

    monkeypatch.setattr(farm._ctx, "Process", _FakeProcess)

    farm._spawn_worker(farm.supervisors[0], start_index=0, n_episodes=2,
                        mission=farm._mission, mission_digest=farm._mission_digest,
                        env_versions_json=farm._env_versions_json)
    farm._spawn_worker(farm.supervisors[1], start_index=0, n_episodes=2,
                        mission=farm._mission, mission_digest=farm._mission_digest,
                        env_versions_json=farm._env_versions_json)

    # args tuple order: (..., result_queue, enable_rotor_fault, fault_specs,
    # fault_config_digest, recovery)
    assert captured_args[0][-4] is True  # enable_rotor_fault
    assert captured_args[0][-3] == specs_by_worker[0]
    assert captured_args[0][-2] == "digest123"
    assert captured_args[1][-3] == specs_by_worker[1]
    assert captured_args[1][-3] is not captured_args[0][-3]
    # M8: every worker gets the run's recovery config (default: no recovery).
    assert captured_args[0][-1] == captured_args[1][-1] == farm.recovery
    assert farm.recovery.policy == "nominal"


def test_resume_seeds_completed_per_worker_from_existing_files(monkeypatch, tmp_path):
    """M6: a run killed mid-flight leaves worker_<k>/episode_ep_NNNN_summary.parquet
    files on disk -- resume=True must pick up each worker's own next unused
    index from the highest one present, not restart at 0."""
    import pandas as pd

    monkeypatch.setattr("experiments.sim_farm.capture_env_versions", lambda: {"stub": True})
    run_id = "resume_test"
    for worker_id, indices in ((0, [0, 1, 2, 3]), (1, [0, 1])):
        worker_dir = tmp_path / run_id / f"worker_{worker_id}"
        worker_dir.mkdir(parents=True)
        for i in indices:
            pd.DataFrame([{"x": i}]).to_parquet(worker_dir / f"episode_ep_{i:04d}_summary.parquet")

    farm = SimFarm(worker_count=2, mission_id="square_circuit", n_episodes_per_worker=10,
                    run_id=run_id, resume=True, results_dir=str(tmp_path))
    assert farm._completed_per_worker == {0: 4, 1: 2}


def test_resume_with_no_existing_files_starts_at_zero(monkeypatch, tmp_path):
    monkeypatch.setattr("experiments.sim_farm.capture_env_versions", lambda: {"stub": True})
    farm = SimFarm(worker_count=2, mission_id="square_circuit", n_episodes_per_worker=10,
                    run_id="brand_new_run", resume=True, results_dir=str(tmp_path))
    assert farm._completed_per_worker == {0: 0, 1: 0}


def test_resume_without_run_id_raises(monkeypatch):
    monkeypatch.setattr("experiments.sim_farm.capture_env_versions", lambda: {"stub": True})
    with pytest.raises(ValueError, match="resume=True requires"):
        SimFarm(worker_count=2, mission_id="square_circuit", n_episodes_per_worker=10,
                resume=True)


def test_run_skips_spawning_a_worker_already_at_full_quota(monkeypatch, tmp_path):
    """The edge case a resumed run can hit: one worker already finished its
    whole quota before the interruption. run() must not crash on
    sup.process being None for that worker."""
    import pandas as pd

    monkeypatch.setattr("experiments.sim_farm.capture_env_versions", lambda: {"stub": True})
    run_id = "resume_full_quota_test"
    worker_dir = tmp_path / run_id / "worker_0"
    worker_dir.mkdir(parents=True)
    for i in range(3):
        pd.DataFrame([{"x": i}]).to_parquet(worker_dir / f"episode_ep_{i:04d}_summary.parquet")

    farm = SimFarm(worker_count=1, mission_id="square_circuit", n_episodes_per_worker=3,
                    run_id=run_id, resume=True, results_dir=str(tmp_path))
    assert farm._completed_per_worker == {0: 3}
    farm.supervisors = [FakeSupervisor(0)]

    with farm:
        results = farm.run()
    assert results == []  # nothing left to fly; must not crash on sup.process is None


def test_enter_starts_every_worker(monkeypatch):
    farm = _farm_with_fakes(monkeypatch, n=3)
    with farm:
        pass
    assert all(sup.start_calls == 1 for sup in farm.supervisors)


def test_exit_stops_every_worker(monkeypatch):
    farm = _farm_with_fakes(monkeypatch, n=3)
    with farm:
        pass
    assert all(sup.stop_calls == 1 for sup in farm.supervisors)


def test_enter_retries_a_transient_start_failure(monkeypatch):
    """Found live (M4 task 7's soak test): a simultaneous 4-worker startup
    occasionally times out waiting for one worker's telemetry under
    contention -- observed on two different workers on two different
    attempts, so it's real, reproducible startup-contention, not one flaky
    instance. __enter__ used to have NO retry budget at all for this,
    failing the whole run on the very first hiccup before a single episode
    flew."""
    farm = _farm_with_fakes(monkeypatch, n=2)
    farm.supervisors = [FakeSupervisor(0, fail_start_times=2), FakeSupervisor(1)]

    with farm:
        pass

    assert farm.supervisors[0].start_calls == 3, "2 failures then a 3rd, successful attempt"
    assert farm.supervisors[1].start_calls == 1


def test_enter_gives_up_after_max_attempts(monkeypatch):
    farm = _farm_with_fakes(monkeypatch, n=1)
    farm.supervisors = [FakeSupervisor(0, fail_start_times=99)]  # never succeeds

    with pytest.raises(SimFarmError, match="failed to start after"):
        with farm:
            pass

    assert farm.supervisors[0].start_calls == 3, "the default max_attempts, not unbounded retries"


def test_farm_stops_all_on_exception(monkeypatch):
    """The milestone's own required test: an exception inside the
    context manager still stops every worker."""
    farm = _farm_with_fakes(monkeypatch, n=3)
    with pytest.raises(ValueError, match="boom"):
        with farm:
            raise ValueError("boom")
    assert all(sup.stop_calls == 1 for sup in farm.supervisors), \
        "every worker must be stopped even though the with-block raised"


def test_exit_attempts_every_stop_even_if_one_fails(monkeypatch):
    """One worker's stop() failing must not prevent the OTHERS from being
    stopped -- CLAUDE.md's "leave nothing running" requirement doesn't get
    to make an exception for the first failure it hits."""
    farm = _farm_with_fakes(monkeypatch, n=3)
    farm.supervisors[1].fail_stop = True
    with pytest.raises(SimFarmError):
        with farm:
            pass
    assert all(sup.stop_calls == 1 for sup in farm.supervisors), \
        "worker 0 and 2 must still be stopped despite worker 1's stop() raising"


def test_original_exception_takes_priority_over_a_stop_failure(monkeypatch):
    """If the with-block itself raised AND a stop() also fails, the
    with-block's own exception must be what propagates -- a cleanup failure
    must never mask the real error that caused it."""
    farm = _farm_with_fakes(monkeypatch, n=2)
    farm.supervisors[0].fail_stop = True
    with pytest.raises(ValueError, match="the real error"):
        with farm:
            raise ValueError("the real error")
    assert all(sup.stop_calls == 1 for sup in farm.supervisors)


# --------------------------------------------- task 4: structured failure handling


def _make_farm(monkeypatch, tmp_path, *, n=2, n_episodes_per_worker=3,
                restart_rate_abort_threshold=0.5):
    """Like _farm_with_fakes, but also routes EpisodeLogger writes under
    tmp_path (task 4's tests write real placeholder records) and lets the
    caller control episode count / abort threshold, which the restart-rate
    tests need."""
    monkeypatch.setattr("experiments.sim_farm.capture_env_versions", lambda: {"stub": True})
    farm = SimFarm(worker_count=n, mission_id="square_circuit",
                    n_episodes_per_worker=n_episodes_per_worker, stagger_s=0.0,
                    results_dir=str(tmp_path),
                    restart_rate_abort_threshold=restart_rate_abort_threshold)
    farm.supervisors = [FakeSupervisor(k) for k in range(n)]
    return farm


def test_synthesize_lost_episode_record_writes_invalid_record(monkeypatch, tmp_path):
    farm = _make_farm(monkeypatch, tmp_path, n=1, n_episodes_per_worker=3)
    sup = farm.supervisors[0]
    farm._completed_per_worker[sup.spec.instance] = 1  # episode 0 already done; 1 was in flight

    farm._synthesize_lost_episode_record(sup)

    assert len(farm.results) == 1
    record = farm.results[0]
    assert record["valid"] is False
    assert record["termination_reason"] == "worker_restarted"
    assert record["episode_id"] == "ep_0001"
    assert record["seed"] == farm.seed_base + 1
    validate_episode(record)  # raises SchemaValidationError if malformed
    assert math.isnan(record["position_rmse_m"])
    assert farm._completed_per_worker[sup.spec.instance] == 2

    summary_path = (tmp_path / farm.run_id / f"worker_{sup.spec.instance}"
                     / "episode_ep_0001_summary.parquet")
    assert summary_path.exists()


def test_synthesize_skipped_when_worker_already_finished_quota(monkeypatch, tmp_path):
    farm = _make_farm(monkeypatch, tmp_path, n=1, n_episodes_per_worker=2)
    sup = farm.supervisors[0]
    farm._completed_per_worker[sup.spec.instance] = 2  # quota already met, nothing was in flight

    farm._synthesize_lost_episode_record(sup)

    assert farm.results == []
    assert farm._completed_per_worker[sup.spec.instance] == 2


def test_on_restart_records_lost_episode_before_respawning(monkeypatch, tmp_path):
    farm = _make_farm(monkeypatch, tmp_path, n=1, n_episodes_per_worker=3)
    sup = farm.supervisors[0]
    farm._completed_per_worker[sup.spec.instance] = 1

    spawn_calls = []
    monkeypatch.setattr(
        farm, "_spawn_worker",
        lambda sup, *, start_index, n_episodes, **kw: spawn_calls.append(
            (sup.spec.instance, start_index, n_episodes)))

    farm._on_restart(sup, farm._mission, farm._mission_digest, farm._env_versions_json)

    assert len(farm.results) == 1
    assert farm.results[0]["termination_reason"] == "worker_restarted"
    # start_index=2 skips past the just-recorded lost episode 1; 1 episode remains of 3.
    assert spawn_calls == [(sup.spec.instance, 2, 1)]
    assert farm.restart_counts[sup.spec.instance] == 1


def test_duplicate_episode_record_is_dropped_not_double_recorded(monkeypatch, tmp_path):
    """The queue-race regression test found live in M4 tasks 6/7's real
    throughput sweep: multiprocessing.Queue.put() can return before an item
    is actually flushed through the pipe. If the child then dies right
    after (e.g. its own rclpy.shutdown() raising under heavy multi-worker
    resource contention), the parent can conclude the episode was lost,
    synthesize a worker_restarted placeholder for it, and THEN see the same
    episode's real result turn up on a LATER _drain_queue() call -- two
    conflicting records for one (worker_id, episode_id). Exactly one must
    survive, whichever was recorded first."""
    farm = _make_farm(monkeypatch, tmp_path, n=1, n_episodes_per_worker=3)
    sup = farm.supervisors[0]
    farm._completed_per_worker[sup.spec.instance] = 1  # episode 0 done; 1 was "in flight"

    farm._synthesize_lost_episode_record(sup)  # placeholder for ep_0001, arrives "first"
    assert len(farm.results) == 1
    assert farm._completed_per_worker[sup.spec.instance] == 2

    late_real_result = dict(farm.results[0], termination_reason="aborted_error", valid=True)
    recorded = farm._record_result(late_real_result)

    assert recorded is False, "a duplicate (worker_id, episode_id) must be reported as not newly recorded"
    assert len(farm.results) == 1, "the late duplicate must be dropped, not appended"
    assert farm.results[0]["termination_reason"] == "worker_restarted", \
        "whichever record was recorded FIRST is what's kept"


def test_drain_queue_does_not_double_count_a_duplicate_result(monkeypatch, tmp_path):
    """Same race as above, exercised through _drain_queue() itself (with a
    fake queue standing in for the real multiprocessing.Queue -- using the
    real one here would reintroduce the exact put()/flush timing
    non-determinism this test exists to pin down): a duplicate pulled off
    the queue must not increment _completed_per_worker a second time."""
    farm = _make_farm(monkeypatch, tmp_path, n=1, n_episodes_per_worker=3)
    sup = farm.supervisors[0]
    farm._completed_per_worker[sup.spec.instance] = 1
    farm._synthesize_lost_episode_record(sup)  # placeholder for ep_0001
    assert farm._completed_per_worker[sup.spec.instance] == 2

    class _FakeQueue:
        def __init__(self, items):
            self._items = list(items)

        def get_nowait(self):
            if not self._items:
                raise queue.Empty
            return self._items.pop(0)

    late_real_result = dict(farm.results[0], termination_reason="aborted_error", valid=True)
    farm._result_queue = _FakeQueue([late_real_result])

    farm._drain_queue()

    assert len(farm.results) == 1
    assert farm._completed_per_worker[sup.spec.instance] == 2, \
        "a dropped duplicate must not double-increment the completed count"


def test_restart_rate_abort_not_triggered_below_threshold(monkeypatch, tmp_path):
    farm = _make_farm(monkeypatch, tmp_path, n=2, n_episodes_per_worker=5,
                       restart_rate_abort_threshold=0.5)
    monkeypatch.setattr(farm, "_spawn_worker", lambda *a, **kw: None)
    sup = farm.supervisors[0]

    for _ in range(4):  # 4 restarts / (2 workers * 5 episodes) = 0.4, below 0.5
        farm._on_restart(sup, farm._mission, farm._mission_digest, farm._env_versions_json)

    assert farm.restart_counts[sup.spec.instance] == 4


def test_restart_rate_abort_raises_past_threshold(monkeypatch, tmp_path):
    farm = _make_farm(monkeypatch, tmp_path, n=2, n_episodes_per_worker=10,
                       restart_rate_abort_threshold=0.5)
    monkeypatch.setattr(farm, "_spawn_worker", lambda *a, **kw: None)
    sup = farm.supervisors[0]

    for _ in range(10):  # 10 restarts / (2 * 10) = 0.50 -- at, not over, the threshold
        farm._on_restart(sup, farm._mission, farm._mission_digest, farm._env_versions_json)
    assert farm.restart_counts[sup.spec.instance] == 10

    with pytest.raises(SimFarmError, match="restart rate"):
        farm._on_restart(sup, farm._mission, farm._mission_digest, farm._env_versions_json)
