"""M4 tasks 1-3's actual gate: two SimFarm workers fly a full M3 mission at
the same time. This is the test that would catch cross-wiring -- one
worker's data ending up under the other's identity -- which single-instance
testing cannot (docs/parallelism.md §2.2: instance 0 is PX4's special case,
so a solo test proves almost nothing about a second worker).
"""
import pandas as pd

from experiments.episode_schema import validate_episode
from experiments.sim_farm import SimFarm


def test_two_workers_fly_concurrently(clean_sim_slate, tmp_path):
    with SimFarm(worker_count=2, mission_id="square_circuit", n_episodes_per_worker=1,
                 speed_factor=4.0, results_dir=str(tmp_path), stagger_s=3.0) as farm:
        results = farm.run()

    assert len(results) == 2, f"expected exactly 2 episode summaries, got {len(results)}"

    by_worker = {r["worker_id"]: r for r in results}
    assert set(by_worker) == {0, 1}, "both workers must be represented, by their own identity"

    for worker_id, summary in by_worker.items():
        validate_episode(summary)  # raises if it doesn't satisfy the (v2) schema
        assert summary["instance"] == worker_id
        assert summary["worker_id"] == worker_id
        assert summary["valid"] is True

        # Each worker's data lives ONLY under its own worker_<k>/ directory --
        # this is the check that would catch one worker's records landing in
        # the other's directory (a cross-wiring bug, not a schema bug, so
        # validate_episode() above wouldn't catch it).
        worker_dir = tmp_path / farm.run_id / f"worker_{worker_id}"
        summary_files = list(worker_dir.glob("episode_*_summary.parquet"))
        assert len(summary_files) == 1
        on_disk = pd.read_parquet(summary_files[0]).iloc[0]
        assert on_disk["worker_id"] == worker_id
        assert on_disk["instance"] == worker_id
        # NOTE: both workers legitimately have a file named "episode_ep_0000_..."
        # -- episode_id numbering is per-worker, not global, so filename
        # collision across worker_<k>/ directories is expected and is not
        # itself a cross-wiring signal. The check that actually catches
        # cross-wiring is on_disk["worker_id"]/["instance"] above: those come
        # from EACH FILE'S OWN CONTENT, not from which directory it lives in,
        # so a record written under the wrong worker's identity would fail
        # exactly here.

    # Not asserting termination_reason == "completed" for both: the M2-era
    # concurrent-worker offboard_control_signal_lost gap (docs/parallelism.md
    # §2.6) is real, confirmed, and not this milestone's job to eliminate --
    # M4's own job is exactly to handle it as a recorded outcome rather than
    # a crash, which the validate_episode() call above already proves for
    # whatever reason either episode actually ended with.
    for worker_id, summary in by_worker.items():
        print(f"worker {worker_id}: termination_reason={summary['termination_reason']!r} "
              f"reset_tier={summary['reset_tier']!r}")
