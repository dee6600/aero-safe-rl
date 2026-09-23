"""M6 task 10: a scaled-down regression version of the real dataset
generation run (results/m6_dataset_v1/, 750 episodes, documented in
docs/fault_dataset.md) -- not the full run itself (far too slow for routine
execution, CLAUDE.md §6's own tiering), but the exact same code path
(experiments/generate_fault_dataset.py's generate(), same fault config) at a
small enough scale to run explicitly and catch a regression, the same
relationship tests/slow/test_soak.py has to M4's real 400-episode soak run.

@pytest.mark.slow -- not run by the default suite. Run explicitly:

    pytest -s tests/slow/test_fault_dataset_run.py

Worker count and episode count are read from env vars
(AERO_FAULT_DATASET_TEST_WORKER_COUNT / AERO_FAULT_DATASET_TEST_N_EPISODES),
defaulting to 2 workers x 10 episodes (20 total) -- large enough to see a
mix of healthy/faulty episodes and both onset profiles at the shipped
config's healthy_fraction (0.2) and profile split, small enough to run in a
few minutes.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))


def _sim_stop_all() -> None:
    subprocess.run([str(REPO / "scripts" / "sim_stop.sh"), "--all"],
                    capture_output=True, text=True, timeout=120)


def _orphan_process_count() -> int:
    proc = subprocess.run(
        ["pgrep", "-cf", "^gz sim |px4_sitl_default/bin/px4|MicroXRCEAgent"],
        capture_output=True, text=True, timeout=10)
    try:
        return int(proc.stdout.strip())
    except ValueError:
        return 0


@pytest.fixture
def clean_sim_slate():
    _sim_stop_all()
    yield
    _sim_stop_all()


@pytest.mark.slow
@pytest.mark.timeout(3600)  # 1h outer bound -- generous above the expected few-minute run
def test_fault_dataset_generation_regression(clean_sim_slate, tmp_path):
    from experiments.episode_schema import validate_episode
    from experiments.generate_fault_dataset import generate

    worker_count = int(os.environ.get("AERO_FAULT_DATASET_TEST_WORKER_COUNT", "2"))
    n_episodes = int(os.environ.get("AERO_FAULT_DATASET_TEST_N_EPISODES", "10")) * worker_count
    print(f"\nFault dataset regression run: {worker_count} workers, "
          f"{n_episodes} episodes total", flush=True)

    results = generate(
        fault_config_path=str(REPO / "configs" / "faults" / "rotor_thrust_degradation_v1.yaml"),
        worker_count=worker_count, speed_factor=1.0, model="x500_aero",
        n_episodes=n_episodes, seed=12345,
        run_id="fault_dataset_regression_test", results_dir=str(tmp_path))

    assert len(results) == n_episodes
    for record in results:
        validate_episode(record)  # raises if it doesn't satisfy schema v4

    valid = [r for r in results if r["valid"]]
    faulty = [r for r in valid if r["fault_applied"]]
    healthy = [r for r in valid if not r["fault_applied"]]
    print(f"\nvalid={len(valid)}/{len(results)} faulty={len(faulty)} healthy={len(healthy)}",
          flush=True)

    # Same real code path as the 750-episode run -- not tuned to make this
    # pass, just the same "did the pipeline actually do its job" bar.
    assert len(valid) / len(results) > 0.5, "too many invalid episodes for a healthy run"
    assert faulty, "no faulty episodes generated at all -- the schedule or the fault " \
                    "injection path is broken, not just unlucky at this small a sample"

    assert _orphan_process_count() == 0, "orphan px4/gz sim/MicroXRCEAgent process after run"
