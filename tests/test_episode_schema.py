"""M3 task 1: the episode record schema and its validator
(experiments/episode_schema.py, configs/schema/episode_record.yaml).

No simulator needed -- validate_episode/validate_step are pure functions of
a dict, tested here against hand-built records exactly like CLAUDE.md §6
asks ("most logic must be testable without a simulator")."""
import pytest

from experiments.episode_schema import (
    FEATURE_VERSION_UNSET,
    ResetTier,
    SCHEMA_VERSION,
    SchemaValidationError,
    TerminationReason,
    digest,
    load_schema,
    validate_episode,
    validate_step,
)

VALID_EPISODE = dict(
    schema_version=SCHEMA_VERSION,
    run_id="run_0001",
    episode_id="ep_0000",
    worker_id=0,
    instance=0,
    seed=42,
    instance_spec_digest="abc123",
    mission_id="square_circuit",
    mission_config_digest="def456",
    feature_version=FEATURE_VERSION_UNSET,
    env_versions="{}",
    reset_tier=ResetTier.NONE.value,
    termination_reason=TerminationReason.COMPLETED.value,
    valid=True,
    t_sim_start_s=0.0,
    t_sim_end_s=61.2,
    t_sim_duration_s=61.2,
    t_wall_start_utc="2026-08-21T00:00:00Z",
    t_wall_end_utc="2026-08-21T00:00:15Z",
    t_wall_duration_s=15.0,
    n_steps=600,
    waypoints_reached=5,
    position_rmse_m=0.31,
    final_position_error_m=0.05,
)

VALID_STEP = dict(
    schema_version=SCHEMA_VERSION,
    run_id="run_0001",
    episode_id="ep_0000",
    worker_id=0,
    step_index=0,
    t_sim_s=0.1,
    t_wall_utc=1234567890.0,
    armed=True,
    nav_state=14,
    pos_x=0.0, pos_y=0.0, pos_z=-5.0,
    vel_x=0.0, vel_y=0.0, vel_z=0.0,
    target_x=0.0, target_y=0.0, target_z=-5.0,
    position_error_m=0.0,
    battery_remaining=1.0,
)


def test_valid_record_passes():
    validate_episode(VALID_EPISODE)
    validate_step(VALID_STEP)


def test_missing_field_fails():
    bad = dict(VALID_EPISODE)
    del bad["seed"]
    with pytest.raises(SchemaValidationError, match="seed"):
        validate_episode(bad)

    bad_step = dict(VALID_STEP)
    del bad_step["pos_x"]
    with pytest.raises(SchemaValidationError, match="pos_x"):
        validate_step(bad_step)


def test_unknown_termination_reason_fails():
    bad = dict(VALID_EPISODE, termination_reason="vibes_felt_off")
    with pytest.raises(SchemaValidationError, match="termination_reason"):
        validate_episode(bad)


def test_unknown_reset_tier_fails():
    bad = dict(VALID_EPISODE, reset_tier="turbo")
    with pytest.raises(SchemaValidationError, match="reset_tier"):
        validate_episode(bad)


def test_schema_version_recorded():
    bad = dict(VALID_EPISODE, schema_version="99")
    with pytest.raises(SchemaValidationError, match="schema_version"):
        validate_episode(bad)

    bad_step = dict(VALID_STEP, schema_version="99")
    with pytest.raises(SchemaValidationError, match="schema_version"):
        validate_step(bad_step)


def test_termination_reason_enum_closed():
    """The enum in code and the list in the schema file must be identical
    sets -- this is the test that catches drift between the two (the
    milestone's explicit requirement)."""
    schema = load_schema()
    assert set(schema["termination_reasons"]) == {r.value for r in TerminationReason}
    assert set(schema["reset_tiers"]) == {t.value for t in ResetTier}


def test_digest_is_stable_and_order_independent():
    a = digest({"x": 1, "y": 2})
    b = digest({"y": 2, "x": 1})
    assert a == b
    assert a != digest({"x": 1, "y": 3})


def test_digest_is_a_short_hex_string():
    d = digest({"anything": [1, 2, 3]})
    assert len(d) == 16
    int(d, 16)  # raises if not hex
