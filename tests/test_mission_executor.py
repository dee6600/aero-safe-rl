"""Unit tests for mission_executor.py's sim_fault detection (M4 task 4) and
_quaternion_to_euler (M5 task 2). No rclpy, no simulator: all three are pure
functions/plain data, so they're testable with no fakes at all.
"""
import math

from aero_bridge.mission_executor import (
    _REASON_FOR_ERROR,
    SimFault,
    _is_finite_state,
    _quaternion_to_euler,
)
from experiments.episode_schema import TerminationReason

FINITE_STATE = (1.0, 2.0, -3.0, 0.1, -0.2, 0.0)


def test_is_finite_state_true_for_ordinary_values():
    assert _is_finite_state(*FINITE_STATE) is True


def test_is_finite_state_false_for_nan_in_any_position():
    for i in range(6):
        state = list(FINITE_STATE)
        state[i] = float('nan')
        assert _is_finite_state(*state) is False, f"NaN in position {i} must be caught"


def test_is_finite_state_false_for_inf_in_any_position():
    for i in range(6):
        state = list(FINITE_STATE)
        state[i] = float('inf')
        assert _is_finite_state(*state) is False, f"inf in position {i} must be caught"


def test_sim_fault_maps_to_sim_fault_termination_reason():
    assert _REASON_FOR_ERROR[SimFault] == TerminationReason.SIM_FAULT


def test_quaternion_to_euler_identity_is_zero():
    roll, pitch, yaw = _quaternion_to_euler((1.0, 0.0, 0.0, 0.0))
    assert math.isclose(roll, 0.0, abs_tol=1e-9)
    assert math.isclose(pitch, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)


def test_quaternion_to_euler_90_degree_roll():
    # q = (cos(45deg), sin(45deg), 0, 0) -- a pure +90deg rotation about
    # body X (roll).
    half = math.radians(90.0) / 2.0
    q = (math.cos(half), math.sin(half), 0.0, 0.0)
    roll, pitch, yaw = _quaternion_to_euler(q)
    assert math.isclose(roll, math.radians(90.0), abs_tol=1e-6)
    assert math.isclose(pitch, 0.0, abs_tol=1e-6)
    assert math.isclose(yaw, 0.0, abs_tol=1e-6)


def test_quaternion_to_euler_90_degree_yaw():
    half = math.radians(90.0) / 2.0
    q = (math.cos(half), 0.0, 0.0, math.sin(half))
    roll, pitch, yaw = _quaternion_to_euler(q)
    assert math.isclose(roll, 0.0, abs_tol=1e-6)
    assert math.isclose(pitch, 0.0, abs_tol=1e-6)
    assert math.isclose(yaw, math.radians(90.0), abs_tol=1e-6)


def test_quaternion_to_euler_handles_gimbal_lock_pitch_without_crashing():
    # +90deg pitch: 2*(w*y - z*x) rounds to exactly 1.0 or slightly above --
    # this is the case the clip in _quaternion_to_euler exists for.
    half = math.radians(90.0) / 2.0
    q = (math.cos(half), 0.0, math.sin(half), 0.0)
    roll, pitch, yaw = _quaternion_to_euler(q)
    assert math.isclose(pitch, math.radians(90.0), abs_tol=1e-6)
    assert math.isfinite(roll) and math.isfinite(yaw)
