"""M6 task 1: the fault schedule contract (experiments/fault_schedule.py,
configs/faults/rotor_thrust_degradation_v1.yaml).

No simulator needed -- load_fault_config/sample_fault_schedule are pure
functions of a dict/rng, exactly like M3's episode-schema tests
(tests/test_episode_schema.py) and M5's feature-extractor tests
(tests/test_feature_extractor.py). Needs ROS sourced (load_fault_config
pulls in aero_bridge.mission_executor to check the onset/ramp-fits-mission
constraint), but not a live simulator.
"""
import copy

import numpy as np
import pytest

from experiments.fault_schedule import (
    FaultConfigError,
    FaultProfile,
    FaultSpec,
    FaultType,
    load_fault_config,
    sample_fault_schedule,
    validate_fault_config,
)

FAULT_CONFIG_PATH = "configs/faults/rotor_thrust_degradation_v1.yaml"


def _base_config(**overrides):
    cfg = {
        "fault_schema_version": "1",
        "fault_type": "rotor_thrust_degradation",
        "mission_id": "square_circuit",
        "dataset": {"n_episodes": 20, "seed": 1, "healthy_fraction": 0.2},
        "rotor_indices": [0, 1, 2, 3],
        "severity_range_s": [0.2, 0.9],
        "onset_time_range_s": [5.0, 40.0],
        "profiles": ["step", "ramp"],
        "ramp_duration_range_s": [1.0, 5.0],
    }
    cfg.update(overrides)
    return cfg


def test_shipped_config_validates():
    cfg = load_fault_config(FAULT_CONFIG_PATH)
    assert cfg["fault_schema_version"] == "1"


def test_missing_top_field_raises():
    cfg = _base_config()
    del cfg["severity_range_s"]
    with pytest.raises(FaultConfigError, match="severity_range_s"):
        validate_fault_config(cfg)


def test_missing_dataset_field_raises():
    cfg = _base_config()
    del cfg["dataset"]["seed"]
    with pytest.raises(FaultConfigError, match="seed"):
        validate_fault_config(cfg)


def test_unsupported_fault_type_raises():
    cfg = _base_config(fault_type="gps_dropout")
    with pytest.raises(FaultConfigError, match="fault_type"):
        validate_fault_config(cfg)


def test_rotor_index_out_of_range_raises():
    cfg = _base_config(rotor_indices=[0, 4])
    with pytest.raises(FaultConfigError, match="rotor_indices"):
        validate_fault_config(cfg)


def test_intermittent_profile_is_rejected():
    cfg = _base_config(profiles=["step", "intermittent"])
    with pytest.raises(FaultConfigError, match="profiles"):
        validate_fault_config(cfg)


def test_severity_range_out_of_bounds_raises():
    cfg = _base_config(severity_range_s=[0.0, 1.5])
    with pytest.raises(FaultConfigError, match="severity_range_s"):
        validate_fault_config(cfg)


def test_severity_range_inverted_raises():
    cfg = _base_config(severity_range_s=[0.9, 0.2])
    with pytest.raises(FaultConfigError, match="severity_range_s"):
        validate_fault_config(cfg)


def test_healthy_fraction_out_of_bounds_raises():
    cfg = _base_config()
    cfg["dataset"]["healthy_fraction"] = 1.0
    with pytest.raises(FaultConfigError, match="healthy_fraction"):
        validate_fault_config(cfg)


def test_onset_plus_ramp_leaves_mission_time_remaining():
    # square_circuit.yaml: timeout_s=120, hold_time_s=2, final_hover_s=3 ->
    # worst-case tail needed = 5. An onset range whose max, plus the max
    # ramp duration, blows past timeout_s - 5 must be rejected.
    cfg = _base_config(onset_time_range_s=[5.0, 118.0], ramp_duration_range_s=[1.0, 5.0])
    with pytest.raises(FaultConfigError, match="onset_time_range_s"):
        validate_fault_config(cfg)


def test_onset_plus_ramp_fits_is_accepted():
    cfg = _base_config(onset_time_range_s=[5.0, 40.0], ramp_duration_range_s=[1.0, 5.0])
    validate_fault_config(cfg)  # must not raise


def test_onset_check_ignores_ramp_when_step_only():
    # If 'ramp' isn't in the configured profiles at all, its (irrelevant)
    # duration range must not count against the onset budget.
    cfg = _base_config(
        onset_time_range_s=[5.0, 114.0], profiles=["step"],
        ramp_duration_range_s=[1.0, 5.0])
    validate_fault_config(cfg)  # must not raise: no ramp will ever be sampled


# --- sample_fault_schedule ---------------------------------------------

def test_sampler_is_deterministic_given_seed():
    cfg = _base_config()
    a = sample_fault_schedule(cfg, np.random.default_rng(42), 50)
    b = sample_fault_schedule(cfg, np.random.default_rng(42), 50)
    assert a == b


def test_two_different_seeds_give_different_schedules():
    cfg = _base_config()
    a = sample_fault_schedule(cfg, np.random.default_rng(1), 50)
    b = sample_fault_schedule(cfg, np.random.default_rng(2), 50)
    assert a != b


def test_healthy_fraction_is_respected_within_tolerance():
    cfg = _base_config()
    cfg["dataset"]["healthy_fraction"] = 0.3
    schedule = sample_fault_schedule(cfg, np.random.default_rng(7), 2000)
    healthy = sum(1 for s in schedule if not s.fault_applied)
    assert abs(healthy / len(schedule) - 0.3) < 0.03


def test_every_rotor_index_in_configured_range():
    cfg = _base_config(rotor_indices=[1, 3])
    schedule = sample_fault_schedule(cfg, np.random.default_rng(3), 200)
    faulty = [s for s in schedule if s.fault_applied]
    assert faulty, "sample too small/unlucky to exercise this test"
    assert all(s.rotor_index in (1, 3) for s in faulty)


def test_severity_within_configured_range():
    cfg = _base_config(severity_range_s=[0.3, 0.6])
    schedule = sample_fault_schedule(cfg, np.random.default_rng(4), 200)
    faulty = [s for s in schedule if s.fault_applied]
    assert faulty
    assert all(0.3 <= s.severity <= 0.6 for s in faulty)


def test_ramp_duration_only_set_for_ramp_profile():
    cfg = _base_config()
    schedule = sample_fault_schedule(cfg, np.random.default_rng(5), 200)
    for s in schedule:
        if not s.fault_applied or s.profile == FaultProfile.STEP:
            assert s.ramp_duration_s == 0.0
        elif s.profile == FaultProfile.RAMP:
            assert s.ramp_duration_s > 0.0


def test_healthy_spec_uses_fixed_sentinels():
    spec = FaultSpec.healthy(3)
    assert spec.fault_applied is False
    assert spec.fault_type == FaultType.NONE
    assert spec.rotor_index == -1
    assert spec.severity == 0.0
    assert spec.profile == FaultProfile.NONE
    assert spec.ramp_duration_s == 0.0
    assert spec.onset_time_s is None


def test_two_healthy_specs_at_same_index_are_equal():
    # Regression test for the NaN-sentinel trap: a frozen dataclass with a
    # NaN field is never equal to itself under == (nan != nan), which would
    # silently break test_sampler_is_deterministic_given_seed for any
    # schedule containing a healthy episode. onset_time_s uses None instead,
    # specifically so this holds.
    assert FaultSpec.healthy(5) == FaultSpec.healthy(5)


def test_to_episode_fields_maps_healthy_sentinel_none_to_nan():
    import math
    fields = FaultSpec.healthy(0).to_episode_fields()
    assert fields["fault_applied"] is False
    assert fields["fault_type"] == "none"
    assert fields["fault_rotor_index"] == -1
    assert math.isnan(fields["fault_onset_time_s_requested"])


def test_to_episode_fields_maps_faulty_spec():
    spec = FaultSpec(
        episode_index=0, fault_applied=True,
        fault_type=FaultType.ROTOR_THRUST_DEGRADATION, rotor_index=2,
        severity=0.5, onset_time_s=12.0, profile=FaultProfile.RAMP,
        ramp_duration_s=3.0)
    fields = spec.to_episode_fields()
    assert fields == dict(
        fault_applied=True, fault_type="rotor_thrust_degradation",
        fault_rotor_index=2, fault_severity_commanded=0.5,
        fault_onset_time_s_requested=12.0, fault_profile="ramp",
        fault_ramp_duration_s=3.0)


def test_sample_fault_schedule_does_not_mutate_config():
    cfg = _base_config()
    before = copy.deepcopy(cfg)
    sample_fault_schedule(cfg, np.random.default_rng(9), 50)
    assert cfg == before
