"""Unit tests for mission_executor.py's sim_fault detection (M4 task 4). No
rclpy, no simulator: _is_finite_state is a pure function and _REASON_FOR_ERROR
is a plain dict, so both are testable with no fakes at all.
"""
import math

from aero_bridge.mission_executor import _REASON_FOR_ERROR, SimFault, _is_finite_state
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
