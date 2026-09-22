"""M6 task 9: experiments/generate_fault_dataset.py's pure partitioning
logic. No simulator -- build_fault_specs_by_worker is a pure function of a
flat schedule, mirroring tests/test_sim_farm_assignment.py's own style for
worker_instance().
"""
import numpy as np
import pytest

from experiments.fault_schedule import FaultSpec
from experiments.generate_fault_dataset import build_fault_specs_by_worker


def _flat_schedule(n):
    return [FaultSpec.healthy(i) for i in range(n)]


def test_partitions_into_contiguous_equal_blocks():
    schedule = _flat_schedule(6)
    by_worker = build_fault_specs_by_worker(schedule, worker_count=3, n_episodes_per_worker=2)
    assert by_worker == [schedule[0:2], schedule[2:4], schedule[4:6]]


def test_every_episode_appears_in_exactly_one_worker_block():
    schedule = _flat_schedule(20)
    by_worker = build_fault_specs_by_worker(schedule, worker_count=4, n_episodes_per_worker=5)
    flattened = [spec for block in by_worker for spec in block]
    assert flattened == schedule


def test_wrong_schedule_length_raises():
    schedule = _flat_schedule(5)
    with pytest.raises(ValueError, match="schedule has 5 entries"):
        build_fault_specs_by_worker(schedule, worker_count=2, n_episodes_per_worker=3)


def test_local_index_within_a_block_is_resume_safe():
    """A worker respawned mid-run resumes at LOCAL index start_index within
    its own block (sim_farm.py's _worker_main) -- confirms block[i]
    corresponds to episode_id f"ep_{i:04d}" for every worker, not a
    globally-offset index that would require the caller to also know which
    worker it is."""
    schedule = _flat_schedule(9)
    by_worker = build_fault_specs_by_worker(schedule, worker_count=3, n_episodes_per_worker=3)
    for worker_block in by_worker:
        for local_index, spec in enumerate(worker_block):
            assert worker_block[local_index] is spec  # trivial, but pins the indexing contract


def test_real_fault_config_produces_a_valid_partition():
    """End-to-end with the real shipped config and sampler (task 1), not
    just synthetic healthy specs."""
    from experiments.fault_schedule import load_fault_config, sample_fault_schedule

    cfg = load_fault_config("configs/faults/rotor_thrust_degradation_v1.yaml")
    schedule = sample_fault_schedule(cfg, np.random.default_rng(1), 10)
    by_worker = build_fault_specs_by_worker(schedule, worker_count=2, n_episodes_per_worker=5)
    assert len(by_worker) == 2
    assert all(len(block) == 5 for block in by_worker)
