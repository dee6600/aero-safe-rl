"""Unit tests for simulation/worker_process.py (M4). No rclpy, no simulator:
subprocess.run is monkeypatched so these check the exact command built for
sim_start.sh/sim_stop.sh, never actually invoking them.
"""
import subprocess

import pytest

from simulation.instance_spec import InstanceSpec
from simulation.worker_process import WorkerProcessError, start_worker, stop_worker


class _FakeCompletedProcess:
    def __init__(self, returncode=0):
        self.returncode = returncode
        self.stdout = ""
        self.stderr = ""


def test_start_worker_headless_by_default_omits_gui(monkeypatch):
    captured = {}

    def fake_run(cmd, **kwargs):
        captured["cmd"] = cmd
        return _FakeCompletedProcess(0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    spec = InstanceSpec.for_instance(0)  # headless=True by default
    start_worker(spec)
    assert "--gui" not in captured["cmd"]


def test_start_worker_passes_gui_when_spec_not_headless(monkeypatch):
    """Found live while building vis_sim.md: InstanceSpec.headless existed
    since M1b but start_worker() never read it, so a SimFarm/WorkerSupervisor
    -launched worker could never be watched in a GUI window regardless of
    what the spec said. This is the test that would have caught it."""
    captured = {}

    def fake_run(cmd, **kwargs):
        captured["cmd"] = cmd
        return _FakeCompletedProcess(0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    spec = InstanceSpec.for_instance(0, headless=False)
    start_worker(spec)
    assert "--gui" in captured["cmd"]


def test_start_worker_raises_on_nonzero_exit(monkeypatch):
    monkeypatch.setattr(subprocess, "run", lambda cmd, **kwargs: _FakeCompletedProcess(1))
    spec = InstanceSpec.for_instance(0)
    with pytest.raises(WorkerProcessError):
        start_worker(spec)


def test_start_worker_raises_on_timeout(monkeypatch):
    def fake_run(cmd, **kwargs):
        raise subprocess.TimeoutExpired(cmd=cmd, timeout=kwargs.get("timeout", 0))

    monkeypatch.setattr(subprocess, "run", fake_run)
    spec = InstanceSpec.for_instance(0)
    with pytest.raises(WorkerProcessError):
        start_worker(spec)


def test_stop_worker_never_uses_all_or_sweep(monkeypatch):
    """CLAUDE.md anti-pattern 7: the broad pkill/--all sweep must never be
    used to stop a single worker -- only its own recorded PIDs (-i N)."""
    captured = {}

    def fake_run(cmd, **kwargs):
        captured["cmd"] = cmd
        return _FakeCompletedProcess(0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    stop_worker(3)
    assert "-i" in captured["cmd"] and "3" in captured["cmd"]
    assert "--all" not in captured["cmd"]
    assert "--sweep" not in captured["cmd"]


def test_stop_worker_raises_on_nonzero_exit(monkeypatch):
    monkeypatch.setattr(subprocess, "run", lambda cmd, **kwargs: _FakeCompletedProcess(1))
    with pytest.raises(WorkerProcessError):
        stop_worker(0)
