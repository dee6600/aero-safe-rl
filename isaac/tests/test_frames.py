"""M8b task 2: frame conversions, on rotations with known answers."""
import math

import pytest
import torch

from aero_isaac.frames import euler_ned, flip, quat_ned_frd, specific_force_frd


def _q(axis, angle):
    s = math.sin(angle / 2)
    v = [0.0, 0.0, 0.0]
    v["xyz".index(axis)] = s
    return torch.tensor([[math.cos(angle / 2), *v]], dtype=torch.float64)


def _euler(q_isaac):
    return euler_ned(quat_ned_frd(q_isaac))[0].tolist()


def test_level_is_zero():
    assert _euler(_q("x", 0.0)) == pytest.approx([0.0, 0.0, 0.0])


def test_turning_left_in_isaac_is_negative_yaw_in_ned():
    # +90 deg about Isaac's up axis turns the nose from north to west.
    assert _euler(_q("z", math.pi / 2)) == pytest.approx([0.0, 0.0, -math.pi / 2])


def test_right_wing_down_is_positive_roll():
    # +roll about Isaac's forward axis lifts the left wing = right wing down.
    assert _euler(_q("x", 0.3)) == pytest.approx([0.3, 0.0, 0.0])


def test_nose_down_is_negative_pitch():
    # +rotation about Isaac's left axis pitches the nose down.
    assert _euler(_q("y", 0.2)) == pytest.approx([0.0, -0.2, 0.0])


def test_vectors_flip():
    assert flip(torch.tensor([[1.0, 2.0, 3.0]])).tolist() == [[1.0, -2.0, -3.0]]


def test_accelerometer_at_rest_reads_minus_g_down():
    f = specific_force_frd(_q("x", 0.0), torch.zeros(1, 3, dtype=torch.float64))
    assert f[0].tolist() == pytest.approx([0.0, 0.0, -9.80665])


def test_accelerometer_rolled_90_reads_gravity_sideways():
    # Right wing down 90 deg: gravity reaction points along body -y (left).
    f = specific_force_frd(_q("x", math.pi / 2), torch.zeros(1, 3, dtype=torch.float64))
    assert f[0].tolist() == pytest.approx([0.0, -9.80665, 0.0], abs=1e-9)
