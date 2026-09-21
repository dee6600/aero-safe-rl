"""Unit tests for experiments/sim_farm.py's assignment logic and context-
manager contract (M4 task 3). No rclpy, no simulator: worker->instance
assignment is tested directly against InstanceSpec; the context-manager
tests use fake WorkerSupervisor-shaped objects, never real ones, since real
supervisors call out to sim_start.sh/sim_stop.sh.
"""
import pytest

from experiments.sim_farm import SimFarm, SimFarmError, worker_instance
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

    def __init__(self, instance, *, fail_stop=False):
        self.spec = type("Spec", (), {"instance": instance})()
        self.start_calls = 0
        self.stop_calls = 0
        self.fail_stop = fail_stop
        self.process = None

    def start(self):
        self.start_calls += 1

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
