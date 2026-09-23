"""M8b task 3: the PyTorch port of PX4's controller. Allocation hand cases,
then closed-loop flight on the test-only rigid body (rigid_body.py):
hover, a waypoint leg, and the weak-rotor cliff near s* ~= 0.41. The
closed-loop ones run on the processor for ~40 s and are marked slow."""
import math

import pytest
import torch

from aero_isaac.controller import PX4Controller, allocate, allocation_matrices
from aero_isaac.px4_model import load_x500
from aero_isaac.rotor import RotorModel, hover_command
from rigid_body import RigidBodySim, fly

X500 = load_x500()
F64 = torch.float64
DT, POS_EVERY = 0.004, 5


def _control(roll=0.0, pitch=0.0, yaw=0.0, thrust=0.6):
    return torch.tensor([[roll, pitch, yaw, 0.0, 0.0, -thrust]], dtype=F64)


def _achieved(u):
    B, _, scale = allocation_matrices(dtype=F64)
    return (u @ B.T) * scale


def test_pure_thrust_is_shared_equally():
    _, mix, _ = allocation_matrices(dtype=F64)
    u = allocate(_control(thrust=0.6), mix)
    assert torch.allclose(u, torch.full_like(u, 0.6), atol=1e-6)


def test_unsaturated_demand_is_met_exactly():
    _, mix, _ = allocation_matrices(dtype=F64)
    c = _control(roll=0.1, pitch=-0.05, yaw=0.05, thrust=0.5)
    assert torch.allclose(_achieved(allocate(c, mix)), c, atol=1e-6)


def test_saturation_gives_up_thrust_before_roll_and_yaw_before_both():
    """Airmode off: at full collective with a large roll demand, PX4 cuts
    collective thrust to keep roll; a yaw demand on top is what gets cut."""
    _, mix, _ = allocation_matrices(dtype=F64)
    c = _control(roll=0.4, yaw=0.4, thrust=0.95)
    u = allocate(c, mix).clamp(0, 1)
    got = _achieved(u)[0]
    assert got[0].item() == pytest.approx(0.4, abs=0.02)        # roll kept
    assert -got[5].item() < 0.95 - 0.05                         # collective reduced
    assert got[2].item() < 0.4 - 0.05                           # yaw reduced


def _sim(n=1):
    sim = RigidBodySim(X500, n, DT)
    ctrl = PX4Controller(n, DT, POS_EVERY, dtype=F64)
    return sim, ctrl


@pytest.mark.slow
def test_takes_off_and_hovers_at_the_model_hover_command():
    sim, ctrl = _sim()
    sp = torch.tensor([[0.0, 0.0, -5.0]], dtype=F64)
    cmds = fly(sim, ctrl, 12.0, sp)
    assert torch.allclose(sim.pos, sp, atol=0.1)
    assert cmds[-250:].mean().item() == pytest.approx(hover_command(X500), abs=0.01)


@pytest.mark.slow
def test_flies_a_15_m_leg_like_px4():
    """PX4's offboard position jump: capped at MPC_XY_VEL_MAX but in
    practice limited by the 45 deg tilt. The M6 flights peaked at ~9 m/s."""
    sim, ctrl = _sim()
    fly(sim, ctrl, 8.0, torch.tensor([[0.0, 0.0, -5.0]], dtype=F64))
    rec = []
    fly(sim, ctrl, 8.0, torch.tensor([[15.0, 0.0, -5.0]], dtype=F64), record=rec)
    speeds = torch.stack([v[0, :2].norm() for _, v, _ in rec])
    tilts = torch.stack([t[0] for _, _, t in rec])
    assert (sim.pos[0] - torch.tensor([15.0, 0.0, -5.0], dtype=F64)).norm() < 0.75
    print(f"peak speed {speeds.max():.2f} m/s, peak tilt {tilts.max():.1f} deg")
    assert 5.0 < speeds.max().item() < 12.5
    assert tilts.max().item() < 50.0


@pytest.mark.slow
def test_the_weak_rotor_cliff():
    """Hover with one weak rotor holds altitude below s* ~= 0.41 and cannot
    above it (milestones.md M8: hover needs 59% of total thrust). Three
    drones at once: s = 0.30, 0.35 hold; s = 0.55 comes down."""
    severity = torch.tensor([0.30, 0.35, 0.55], dtype=F64)
    sim, ctrl = _sim(3)
    sp = torch.tensor([[0.0, 0.0, -5.0]], dtype=F64).repeat(3, 1)
    fly(sim, ctrl, 10.0, sp)
    scale = RotorModel.fault_speed_scale(severity, torch.zeros(3, dtype=torch.long))
    rec = []
    fly(sim, ctrl, 6.0, sp, speed_scale=scale, record=rec)
    alt_min = torch.stack([-p[:, 2] for p, _, _ in rec]).min(dim=0).values
    print("min altitude by severity:", dict(zip(severity.tolist(), alt_min.tolist())))
    assert alt_min[0] > 3.5 and alt_min[1] > 3.5
    assert alt_min[2] < 1.0
