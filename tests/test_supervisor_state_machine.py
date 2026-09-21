"""Unit tests for experiments/worker_supervisor.py (M4 task 2). No rclpy, no
simulator: simulation.worker_process.start_worker/stop_worker are
monkeypatched to fake subprocess calls, and instance_<N>.json /
worker_<N>_heartbeat.json are written directly to a tmp_path run_dir to
simulate PID/heartbeat state without a real process ever existing.
"""
import json
import signal
import time

import pytest

from experiments.worker_supervisor import (
    RestartBudgetExhausted,
    WorkerSupervisor,
    _pid_alive,
    write_heartbeat,
)
from simulation.instance_spec import InstanceSpec
from simulation.worker_process import WorkerProcessError


def write_instance_json(run_dir, instance, *, pid_gz, pid_agent, pid_px4):
    spec = InstanceSpec.for_instance(instance)
    path = run_dir / f"instance_{instance}.json"
    path.write_text(json.dumps({
        "spec": spec.to_dict(),
        "runtime": {"pid_gz": pid_gz, "pid_agent": pid_agent, "pid_px4": pid_px4},
    }))
    return spec


class FakeProcess:
    """Stands in for multiprocessing.Process -- lets tests control
    is_alive()/terminate()/join() without a real child process."""

    def __init__(self, alive=True):
        self._alive = alive
        self.pid = 424242
        self.terminate_calls = 0
        self.join_calls = 0

    def is_alive(self):
        return self._alive

    def terminate(self):
        self.terminate_calls += 1
        self._alive = False

    def join(self, timeout=None):
        self.join_calls += 1


@pytest.fixture
def alive_pid():
    """A real, currently-alive PID that isn't this test process itself --
    using os.getpid() would also work, but a dedicated fixture makes the
    intent ('this PID is alive') explicit at each call site."""
    import os
    return os.getpid()


# ------------------------------------------------------------- start / stop


def test_start_calls_worker_process_start(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr("experiments.worker_supervisor.start_worker",
                         lambda spec, run_dir: calls.append((spec.instance, run_dir)))
    spec = InstanceSpec.for_instance(3)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))
    sup.start()
    assert calls == [(3, str(tmp_path))]


def test_stop_calls_worker_process_stop_and_child_terminate(tmp_path, monkeypatch):
    stop_calls = []
    monkeypatch.setattr("experiments.worker_supervisor.stop_worker",
                         lambda instance, run_dir: stop_calls.append(instance))
    spec = InstanceSpec.for_instance(1)
    write_instance_json(tmp_path, 1, pid_gz=1, pid_agent=1, pid_px4=1)  # stop() only calls
    # stop_worker when there's a tracked instance file -- see
    # test_stop_is_a_noop_when_no_instance_file_exists for the no-file case.
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))
    sup.process = FakeProcess(alive=True)
    sup.stop()
    assert stop_calls == [1]
    assert sup.process.terminate_calls == 1


def test_stop_is_a_noop_on_child_process_when_none_or_dead(tmp_path, monkeypatch):
    monkeypatch.setattr("experiments.worker_supervisor.stop_worker", lambda instance, run_dir: None)
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))
    sup.process = None
    sup.stop()  # must not raise with no child process at all

    sup.process = FakeProcess(alive=False)
    sup.stop()
    assert sup.process.terminate_calls == 0, "must not terminate an already-dead process"


def test_stop_is_a_noop_when_no_instance_file_exists(tmp_path, monkeypatch):
    """Found live (M4 tasks 6/7): a worker whose most recent restart ATTEMPT
    itself failed (e.g. start_worker's own failure-cleanup already ran) has
    no instance_<N>.json and nothing left running -- stop() must treat that
    as already-stopped, not raise. This used to make SimFarm.__exit__ crash
    an otherwise-finished run purely because one worker had already, and
    correctly, ended up with nothing to stop."""
    stop_worker_calls = []
    monkeypatch.setattr("experiments.worker_supervisor.stop_worker",
                         lambda instance, run_dir: stop_worker_calls.append(instance))
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))  # no instance_0.json ever written

    sup.stop()  # must not raise

    assert stop_worker_calls == [], "stop_worker (sim_stop.sh) must not even be invoked"


# --------------------------------------------------------------------- pids


def test_pid_alive_true_for_running_process(alive_pid):
    assert _pid_alive(alive_pid) is True


def test_pid_alive_false_for_dead_pid():
    assert _pid_alive(999999999) is False


def test_pid_alive_false_for_none():
    assert _pid_alive(None) is False


# ------------------------------------------------------------- is_healthy


def test_is_healthy_true_when_all_pids_alive_and_no_heartbeat_yet(tmp_path, alive_pid):
    write_instance_json(tmp_path, 0, pid_gz=alive_pid, pid_agent=alive_pid, pid_px4=alive_pid)
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))
    # No heartbeat file written yet -- must not be treated as unhealthy
    # (a worker that has started but not yet flown its first control tick
    # has nothing to have written).
    assert sup.is_healthy() is True


def test_is_healthy_false_when_any_pid_is_dead(tmp_path, alive_pid):
    write_instance_json(tmp_path, 0, pid_gz=alive_pid, pid_agent=999999999, pid_px4=alive_pid)
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))
    assert sup.is_healthy() is False


def test_is_healthy_false_when_instance_file_missing(tmp_path):
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))
    assert sup.is_healthy() is False


def test_is_healthy_false_when_child_process_dead(tmp_path, alive_pid):
    write_instance_json(tmp_path, 0, pid_gz=alive_pid, pid_agent=alive_pid, pid_px4=alive_pid)
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))
    sup.process = FakeProcess(alive=False)
    assert sup.is_healthy() is False


def test_is_healthy_false_when_heartbeat_stale(tmp_path, alive_pid):
    write_instance_json(tmp_path, 0, pid_gz=alive_pid, pid_agent=alive_pid, pid_px4=alive_pid)
    heartbeat = tmp_path / "worker_0_heartbeat.json"
    heartbeat.write_text(json.dumps({"t_wall": time.time() - 1000.0, "last_odometry_wall_s": 0.0}))
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path), heartbeat_stall_timeout_s=45.0)
    assert sup.is_healthy() is False


def test_is_healthy_true_when_heartbeat_fresh(tmp_path, alive_pid):
    write_instance_json(tmp_path, 0, pid_gz=alive_pid, pid_agent=alive_pid, pid_px4=alive_pid)
    write_heartbeat(0, str(tmp_path), last_odometry_wall_s=0.0)
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path), heartbeat_stall_timeout_s=45.0)
    assert sup.is_healthy() is True


# ---------------------------------------------------------- ensure_healthy


def test_ensure_healthy_does_nothing_and_returns_false_when_healthy(tmp_path, alive_pid, monkeypatch):
    write_instance_json(tmp_path, 0, pid_gz=alive_pid, pid_agent=alive_pid, pid_px4=alive_pid)
    start_calls = []
    monkeypatch.setattr("experiments.worker_supervisor.start_worker",
                         lambda spec, run_dir: start_calls.append(spec.instance))
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))
    assert sup.ensure_healthy() is False
    assert start_calls == [], "must not restart a healthy worker"
    assert sup.restart_count == 0


def test_supervisor_restart_marks_episode_invalid(tmp_path, monkeypatch):
    """The milestone's own required test: a simulated process death (dead
    pid_px4) is detected, ensure_healthy() restarts and reports True --
    which is the exact signal SimFarm uses to mark the in-flight episode
    invalid with termination_reason=worker_restarted (experiments/sim_farm.py)."""
    write_instance_json(tmp_path, 0, pid_gz=12345, pid_agent=12345, pid_px4=999999999)
    start_calls, stop_calls = [], []
    monkeypatch.setattr("experiments.worker_supervisor.start_worker",
                         lambda spec, run_dir: start_calls.append(spec.instance))
    monkeypatch.setattr("experiments.worker_supervisor.stop_worker",
                         lambda instance, run_dir: stop_calls.append(instance))
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))

    restarted = sup.ensure_healthy()

    assert restarted is True
    assert start_calls == [0]
    assert stop_calls == [0]
    assert sup.restart_count == 1


def test_is_healthy_true_when_child_alive_even_if_instance_file_missing(tmp_path):
    """The regression test for the real bug found live in M4 tasks 6/7's
    first real hard reset: a worker's own child process legitimately has NO
    instance_<N>.json for ~20s while it runs its own internal hard_reset()
    (EpisodeRunner.run_episode() calls it directly, in the child). The old
    is_healthy() treated that absence as "dead" and raced the child's own
    in-flight reset with a second, redundant restart -- PX4 refused the
    second start ("server already running"), which crashed the run and left
    orphaned, untracked processes behind. No instance_0.json is written at
    all here, simulating exactly that window."""
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))
    sup.process = FakeProcess(alive=True)
    write_heartbeat(0, str(tmp_path), last_odometry_wall_s=0.0)  # fresh

    assert sup.is_healthy() is True


def test_is_healthy_false_when_child_alive_but_heartbeat_stale_despite_missing_pids(tmp_path):
    """The other half of the same fix: a live child is trusted over raw PID
    presence, but NOT unconditionally -- a genuinely stuck child (heartbeat
    stopped advancing) must still be caught, via the heartbeat, not via
    PIDs. No instance_0.json here either, so this also confirms the PID
    check isn't what's making this fail."""
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path), heartbeat_stall_timeout_s=45.0)
    sup.process = FakeProcess(alive=True)
    heartbeat = tmp_path / "worker_0_heartbeat.json"
    heartbeat.write_text(json.dumps({"t_wall": time.time() - 1000.0, "last_odometry_wall_s": 0.0}))

    assert sup.is_healthy() is False


def test_ensure_healthy_survives_a_failed_restart_attempt(tmp_path, monkeypatch):
    """start() failing during a restart attempt (e.g. a genuinely stuck old
    process, disk issue, whatever) must not crash the whole run with an
    unhandled exception -- it's handled the same as every other restart
    failure mode: counted against the budget, reported, and left for the
    next health-poll cycle to retry. Found live: this used to propagate a
    raw WorkerSupervisorError out of ensure_healthy() and take the entire
    SimFarm run down."""
    write_instance_json(tmp_path, 0, pid_gz=12345, pid_agent=12345, pid_px4=999999999)
    monkeypatch.setattr("experiments.worker_supervisor.stop_worker", lambda instance, run_dir: None)

    def failing_start(spec, run_dir):
        raise WorkerProcessError("simulated: px4 exited during startup")

    monkeypatch.setattr("experiments.worker_supervisor.start_worker", failing_start)
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path), restart_budget=3)

    restarted = sup.ensure_healthy()  # must not raise

    assert restarted is True
    assert sup.restart_count == 1


def test_ensure_healthy_force_kills_a_pid_left_alive_after_stop(tmp_path, monkeypatch, alive_pid):
    """Found live (M4 tasks 6/7, worker_count>=3 under heavy CPU load):
    stop_worker() can return -- successfully or with a swallowed failure --
    while one of the three OS processes it was supposed to kill is still
    actually alive. Left alone, that permanently leaks an untracked process
    that blocks every FUTURE start attempt for this same instance (a real
    gz sim + MicroXRCEAgent pair was found still running, and still holding
    the instance's UDP port, minutes after the "restart" that was supposed
    to replace them). ensure_healthy() must verify and force-kill anything
    still alive rather than trusting stop_worker()."""
    write_instance_json(tmp_path, 0, pid_gz=999999999, pid_agent=alive_pid, pid_px4=999999999)
    monkeypatch.setattr("experiments.worker_supervisor.stop_worker", lambda instance, run_dir: None)
    monkeypatch.setattr("experiments.worker_supervisor.start_worker", lambda spec, run_dir: None)
    killed = []

    def fake_kill(pid, sig):
        # os.kill is also how _pid_alive() PROBES liveness (sig=0) -- this
        # fake must keep that working correctly (999999999 "dead", alive_pid
        # "alive") in addition to recording the real SIGKILL this test cares
        # about, or _pid_alive() itself silently breaks for every caller.
        if pid == 999999999:
            raise ProcessLookupError()
        if sig == 0:
            return
        killed.append((pid, sig))

    monkeypatch.setattr("experiments.worker_supervisor.os.kill", fake_kill)
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path))

    sup.ensure_healthy()

    assert killed == [(alive_pid, signal.SIGKILL)], \
        "only the still-alive pid must be force-killed, not the already-dead ones"


def test_restart_budget_exhausted_raises(tmp_path, monkeypatch):
    """Exceeding the restart budget fails loudly (CLAUDE.md §5) instead of
    looping forever on a worker that will not stay up."""
    write_instance_json(tmp_path, 0, pid_gz=12345, pid_agent=12345, pid_px4=999999999)
    monkeypatch.setattr("experiments.worker_supervisor.start_worker", lambda spec, run_dir: None)
    monkeypatch.setattr("experiments.worker_supervisor.stop_worker", lambda instance, run_dir: None)
    spec = InstanceSpec.for_instance(0)
    sup = WorkerSupervisor(spec, run_dir=str(tmp_path), restart_budget=2)

    # The pid stays dead in instance_0.json for every check below (this test
    # never rewrites it), so every ensure_healthy() call sees the same
    # unhealthy state and (until the budget is hit) restarts again.
    assert sup.ensure_healthy() is True
    assert sup.ensure_healthy() is True
    with pytest.raises(RestartBudgetExhausted):
        sup.ensure_healthy()
    assert sup.restart_count == 2, "budget-exhausted attempt must not itself count as a restart"
