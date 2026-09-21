"""Unit tests for experiments/run_manifest.py (M4 task 5). No rclpy, no
simulator: RunManifest is pure bookkeeping over data a caller already has.
"""
import json

from experiments.run_manifest import RunManifest

ENV_VERSIONS_JSON = json.dumps({"px4": {"commit": "deadbeef"}, "gazebo_version": "8.15.0"})


def _make_manifest(**overrides):
    kwargs = dict(
        run_id="run_test_0001", mission_id="square_circuit",
        mission_config_digest="abc123", env_versions_json=ENV_VERSIONS_JSON,
        seed_base=0, worker_to_instance={0: 0, 1: 1},
        restart_budget_per_worker=5, restart_rate_abort_threshold=0.5,
    )
    kwargs.update(overrides)
    return RunManifest(**kwargs)


def test_manifest_captures_static_fields_at_construction():
    m = _make_manifest()
    d = m.to_dict()
    assert d["run_id"] == "run_test_0001"
    assert d["mission_id"] == "square_circuit"
    assert d["mission_config_digest"] == "abc123"
    assert d["env_versions"] == json.loads(ENV_VERSIONS_JSON)
    assert d["worker_to_instance"] == {0: 0, 1: 1}
    assert d["restart_budget_per_worker"] == 5
    assert d["restart_rate_abort_threshold"] == 0.5
    assert d["start_time_utc"]
    assert d["end_time_utc"] is None
    # repo_git_sha is environment-dependent (a real SHA or "unknown" if git
    # isn't available) -- just assert it's present and non-empty.
    assert d["repo_git_sha"]


def test_record_episode_updates_outcome_counts():
    m = _make_manifest()
    m.record_episode({"termination_reason": "completed"})
    m.record_episode({"termination_reason": "completed"})
    m.record_episode({"termination_reason": "worker_restarted"})

    d = m.to_dict()
    assert d["total_episodes"] == 3
    assert d["episode_counts_by_outcome"] == {"completed": 2, "worker_restarted": 1}


def test_record_restart_updates_restart_counts():
    m = _make_manifest()
    m.record_restart(0)
    m.record_restart(0)
    m.record_restart(1)

    assert m.to_dict()["restart_counts"] == {0: 2, 1: 1}


def test_write_produces_valid_json_at_the_given_path(tmp_path):
    m = _make_manifest()
    path = tmp_path / "run_test_0001" / "manifest.json"
    m.write(path)

    assert path.exists()
    on_disk = json.loads(path.read_text())
    assert on_disk["run_id"] == "run_test_0001"


def test_manifest_readable_after_kill(tmp_path):
    """The milestone's own required test: a manifest written mid-run (as if
    the process had been killed right after) still parses and reflects
    exactly the updates that happened before the kill -- not a corrupt or
    empty file."""
    m = _make_manifest()
    path = tmp_path / "manifest.json"
    m.write(path)  # the initial write, at construction time

    m.record_episode({"termination_reason": "completed"})
    m.write(path)  # simulates SimFarm's own write-after-every-update pattern

    m.record_restart(0)
    m.write(path)  # ...and then the process is imagined to die right here

    on_disk = json.loads(path.read_text())  # must still parse
    assert on_disk["total_episodes"] == 1
    assert on_disk["episode_counts_by_outcome"] == {"completed": 1}
    assert on_disk["restart_counts"] == {"0": 1, "1": 0}  # JSON keys are strings
    assert on_disk["end_time_utc"] is None  # finalize() was never called


def test_finalize_sets_end_time_and_writes(tmp_path):
    m = _make_manifest()
    path = tmp_path / "manifest.json"
    assert m.end_time_utc is None

    m.finalize(path)

    assert m.end_time_utc is not None
    on_disk = json.loads(path.read_text())
    assert on_disk["end_time_utc"] == m.end_time_utc
