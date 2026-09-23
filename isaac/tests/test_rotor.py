"""M8b task 3: the Isaac-side rotor and rotor-fault model (CLAUDE.md §1.6).
The fault must mean the same thing as the Gazebo plugin for the same
severity -- checked against the fixture M6 recorded from the plugin."""
import json
import math
from pathlib import Path

import pytest
import torch

from aero_isaac.px4_model import load_x500
from aero_isaac.rotor import RotorModel, hover_command

X500 = load_x500()
FIXTURE = Path(__file__).resolve().parent.parent.parent / "tests" / "fixtures" / "rotor_fault_thrust_curve.json"
F64 = torch.float64


def _steady(model, command, scale, seconds=1.0, dt=0.004):
    for _ in range(int(seconds / dt)):
        thrust, torque = model.step(command, scale, dt)
    return thrust, torque


@pytest.mark.parametrize("point", json.loads(FIXTURE.read_text())["curve"], ids=lambda p: f"s={p['severity']}")
@pytest.mark.parametrize("command", [0.5, 0.736, 1.0])
def test_fault_matches_the_gazebo_plugin_fixture(point, command):
    """Thrust and drag torque of the faulted rotor are (1 - s) of healthy, at
    any command including full -- the plugin's measured speed ratio squared."""
    s = point["severity"]
    m = RotorModel(X500, 1, dtype=F64)
    cmd = torch.full((1, 4), command, dtype=F64)
    healthy_t, healthy_q = _steady(m, cmd, torch.ones(1, 4, dtype=F64))
    m.reset(torch.arange(1))
    scale = RotorModel.fault_speed_scale(torch.tensor([s], dtype=F64), torch.tensor([2]))
    assert scale[0, 2].item() == pytest.approx(point["measured_velocity_ratio"], abs=1e-9)
    t, q = _steady(m, cmd, scale)
    assert (t[0, 2] / healthy_t[0, 2]).item() == pytest.approx(point["derived_thrust_ratio"], abs=1e-6)
    assert (q[0, 2] / healthy_q[0, 2]).item() == pytest.approx(1 - s, abs=1e-6)
    assert torch.allclose(t[0, [0, 1, 3]], healthy_t[0, [0, 1, 3]])      # other rotors untouched


def test_no_fault_is_a_unit_scale():
    scale = RotorModel.fault_speed_scale(torch.tensor([0.7, 0.0]), torch.tensor([-1, 1]))
    assert torch.equal(scale, torch.ones(2, 4))


def test_command_maps_to_px4s_output_range():
    m = RotorModel(X500, 1, dtype=F64)
    ref = m.reference_speed(torch.tensor([[0.0, 0.5, 1.0, 1.2]], dtype=F64), torch.ones(1, 4, dtype=F64))
    assert ref[0].tolist() == pytest.approx([150.0, 575.0, 1000.0, 1000.0])


def test_first_order_lag_uses_gazebos_time_constants():
    m = RotorModel(X500, 1, dtype=F64)
    dt = 1e-4
    target = m.reference_speed(torch.ones(1, 4, dtype=F64), torch.ones(1, 4, dtype=F64))[0, 0].item()
    for _ in range(int(round(X500.rotors[0].tau_up / dt))):
        m.step(torch.ones(1, 4, dtype=F64), torch.ones(1, 4, dtype=F64), dt)
    assert m.speed[0, 0].item() / target == pytest.approx(1 - math.exp(-1), abs=1e-3)


def test_spin_direction_sets_the_drag_torque_sign():
    m = RotorModel(X500, 1, dtype=F64)
    _, q = _steady(m, torch.full((1, 4), 0.7, dtype=F64), torch.ones(1, 4, dtype=F64))
    for i, r in enumerate(X500.rotors):
        assert math.copysign(1, q[0, i].item()) == -r.turning   # counter-clockwise rotor twists the frame clockwise


def test_model_hover_command_is_close_to_what_px4_measured():
    """Weight / 4 per rotor through the motor law gives the hover command.
    The M6 flights measured a median 0.736 over whole missions."""
    assert hover_command(X500) == pytest.approx(0.729, abs=0.002)
    assert abs(hover_command(X500) - 0.736) < 0.05
