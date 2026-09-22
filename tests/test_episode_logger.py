"""M3 task 4: the episode logger (aero_bridge/episode_logger.py).

No simulator needed -- log_step/write_episode take plain dicts. Uses a
tmp_path results dir so nothing here touches the real results/ tree.
"""
import pandas as pd
import pytest

from aero_bridge.episode_logger import EpisodeLogger
from experiments.episode_schema import (
    FEATURE_VERSION_UNSET, FaultProfile, FaultType, ResetTier, SCHEMA_VERSION,
    SchemaValidationError, TerminationReason,
)

STEP_TEMPLATE = dict(
    schema_version=SCHEMA_VERSION, run_id="run_0001", episode_id="ep_0000", worker_id=0,
    t_wall_utc=0.0, armed=True, nav_state=14,
    pos_x=0.0, pos_y=0.0, pos_z=-5.0, vel_x=0.0, vel_y=0.0, vel_z=0.0,
    target_x=0.0, target_y=0.0, target_z=-5.0, position_error_m=0.0, battery_remaining=1.0,
    roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0,
    rate_p_rad_s=0.0, rate_q_rad_s=0.0, rate_r_rad_s=0.0,
    accel_x_m_s2=0.0, accel_y_m_s2=0.0, accel_z_m_s2=-9.81,
    motor_0_output=0.5, motor_1_output=0.5, motor_2_output=0.5, motor_3_output=0.5,
    px4_failure_detector_status=0,
)

EPISODE_TEMPLATE = dict(
    schema_version=SCHEMA_VERSION, run_id="run_0001", episode_id="ep_0000", worker_id=0,
    instance=0, seed=1, instance_spec_digest="abc", mission_id="square_circuit",
    mission_config_digest="def", feature_version=FEATURE_VERSION_UNSET, env_versions="{}",
    reset_tier=ResetTier.NONE.value, termination_reason=TerminationReason.COMPLETED.value,
    valid=True, t_sim_start_s=0.0, t_sim_end_s=10.0, t_sim_duration_s=10.0,
    t_wall_start_utc="x", t_wall_end_utc="y", t_wall_duration_s=1.0,
    n_steps=3, waypoints_reached=5, position_rmse_m=0.1, final_position_error_m=0.05,
    fault_config_digest="none", fault_applied=False, fault_type=FaultType.NONE.value,
    fault_rotor_index=-1, fault_severity_commanded=0.0,
    fault_onset_time_s_requested=float('nan'), fault_onset_time_s_observed=float('nan'),
    fault_profile=FaultProfile.NONE.value, fault_ramp_duration_s=0.0,
    fault_confirmed_applied=False, fault_confirmed_severity_final=0.0,
    px4_failure_detector_silent=True,
)


def _step(i, **overrides):
    return dict(STEP_TEMPLATE, step_index=i, t_sim_s=i * 0.1, **overrides)


def test_logger_row_count(tmp_path):
    logger = EpisodeLogger("run_0001", worker_id=0, results_dir=tmp_path)
    for i in range(3):
        logger.log_step(_step(i))
    steps_path, summary_path = logger.write_episode(EPISODE_TEMPLATE)

    steps_df = pd.read_parquet(steps_path)
    summary_df = pd.read_parquet(summary_path)
    assert len(steps_df) == 3
    assert len(summary_df) == 1


def test_logger_validates_steps_before_buffering():
    logger = EpisodeLogger("run_0001", worker_id=0, results_dir="/tmp/should-not-be-created")
    bad_step = dict(_step(0))
    del bad_step["pos_x"]
    with pytest.raises(SchemaValidationError):
        logger.log_step(bad_step)


def test_logger_validates_episode_before_writing(tmp_path):
    logger = EpisodeLogger("run_0001", worker_id=0, results_dir=tmp_path)
    logger.log_step(_step(0))
    bad_summary = dict(EPISODE_TEMPLATE)
    del bad_summary["seed"]
    with pytest.raises(SchemaValidationError):
        logger.write_episode(bad_summary)


def test_logger_resets_buffer_between_episodes(tmp_path):
    logger = EpisodeLogger("run_0001", worker_id=0, results_dir=tmp_path)
    logger.log_step(_step(0))
    logger.write_episode(dict(EPISODE_TEMPLATE, episode_id="ep_0000", n_steps=1))

    logger.log_step(_step(0))
    logger.log_step(_step(1))
    steps_path, _ = logger.write_episode(dict(EPISODE_TEMPLATE, episode_id="ep_0001", n_steps=2))

    assert len(pd.read_parquet(steps_path)) == 2


def test_logger_one_writer_per_file(tmp_path):
    """Two logger instances with different worker ids never target the same
    path, even for the same run and episode id."""
    logger_a = EpisodeLogger("run_0001", worker_id=0, results_dir=tmp_path)
    logger_b = EpisodeLogger("run_0001", worker_id=1, results_dir=tmp_path)

    logger_a.log_step(_step(0))
    logger_b.log_step(_step(0, worker_id=1))

    path_a, _ = logger_a.write_episode(dict(EPISODE_TEMPLATE, worker_id=0))
    path_b, _ = logger_b.write_episode(dict(EPISODE_TEMPLATE, worker_id=1))

    assert path_a != path_b
    assert path_a.parent != path_b.parent
