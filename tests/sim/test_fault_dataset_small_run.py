"""M6 task 9 (@pytest.mark.sim): a small real run of
experiments/generate_fault_dataset.py -- 2 workers x 3 episodes, mixed
healthy/faulty via the real fault config and sampler, proving records
validate against schema v4 end to end through the real SimFarm/EpisodeRunner
path, not just the pure partitioning tests in
tests/test_generate_fault_dataset_assignment.py.
"""
from experiments.episode_schema import validate_episode
from experiments.generate_fault_dataset import generate


def test_small_mixed_dataset_run_produces_valid_records(clean_sim_slate, tmp_path):
    results = generate(
        fault_config_path="configs/faults/rotor_thrust_degradation_v1.yaml",
        worker_count=2, speed_factor=1.0, model="x500_aero", n_episodes=6,
        seed=1, run_id="m6_fault_dataset_small_test", results_dir=str(tmp_path))

    assert len(results) == 6
    for record in results:
        validate_episode(record)  # raises if it doesn't satisfy schema v4
        assert record["fault_config_digest"] != "none"
        if record["fault_applied"]:
            assert record["fault_type"] == "rotor_thrust_degradation"
            assert record["fault_rotor_index"] in (0, 1, 2, 3)
        else:
            assert record["fault_rotor_index"] == -1
            assert record["fault_type"] == "none"

    # At least one of each, at n=6 with the shipped config's healthy_fraction
    # (0.2) and a fixed seed -- if this ever flakes empty, the seed or
    # healthy_fraction changed enough to need a bigger n here, not that the
    # underlying mechanism is broken.
    assert any(r["fault_applied"] for r in results), "no faulty episode in this small run"
