"""Test-only: a minimal rigid-body integrator for the x500, in PX4's frames
(north-east-down world, forward-right-down body), with a flat ground at z = 0.
It lets the controller and rotor model be checked in closed loop without
starting Isaac Sim. The real environment uses Isaac Lab's physics engine;
this is not a second simulator for results, only a test harness."""
import math

import torch

from aero_isaac.controller import qmul
from aero_isaac.px4_model import mass_properties
from aero_isaac.rotor import RotorModel

G = 9.80665
FLIP = torch.tensor((1.0, -1.0, -1.0), dtype=torch.float64)


def rotate(q, v):
    w, u = q[:, :1], q[:, 1:]
    t = 2.0 * torch.cross(u, v, dim=1)
    return v + w * t + torch.cross(u, t, dim=1)


class RigidBodySim:

    def __init__(self, x500, n, dt, dtype=torch.float64):
        com, inertia = mass_properties(x500)
        self.n, self.dt = n, dt
        self.mass = x500.mass
        self.I = torch.tensor(inertia, dtype=dtype) * torch.outer(FLIP, FLIP)   # into forward-right-down
        self.I_inv = torch.linalg.inv(self.I)
        pos_flu = torch.tensor([r.position_flu for r in x500.rotors], dtype=dtype) - torch.tensor(com, dtype=dtype)
        self.r_frd = pos_flu * FLIP                                             # (4, 3)
        self.rotors = RotorModel(x500, n, dtype=dtype)
        self.pos = torch.zeros(n, 3, dtype=dtype)
        self.vel = torch.zeros(n, 3, dtype=dtype)
        self.q = torch.tensor([[1.0, 0.0, 0.0, 0.0]], dtype=dtype).repeat(n, 1)
        self.w = torch.zeros(n, 3, dtype=dtype)

    def step(self, command, speed_scale):
        thrust, drag_tq_up = self.rotors.step(command, speed_scale, self.dt)
        f_rotor = torch.zeros(self.n, 4, 3, dtype=thrust.dtype)
        f_rotor[..., 2] = -thrust                                             # up is -z in the body
        force_b = f_rotor.sum(1)
        torque_b = torch.cross(self.r_frd.expand(self.n, -1, -1), f_rotor, dim=2).sum(1)
        torque_b[:, 2] += -drag_tq_up.sum(1)                                  # about body z (down)
        acc = rotate(self.q, force_b) / self.mass + torch.tensor((0.0, 0.0, G), dtype=thrust.dtype)
        ground = (self.pos[:, 2] >= 0.0) & (acc[:, 2] > 0.0)
        acc[:, 2] = torch.where(ground, torch.zeros_like(acc[:, 2]), acc[:, 2])
        self.vel = self.vel + acc * self.dt
        self.vel[:, 2] = torch.where(ground & (self.vel[:, 2] > 0), torch.zeros_like(self.vel[:, 2]), self.vel[:, 2])
        self.pos = self.pos + self.vel * self.dt
        self.pos[:, 2] = self.pos[:, 2].clamp(max=0.0)
        Iw = self.w @ self.I.T
        wdot = (torque_b - torch.cross(self.w, Iw, dim=1)) @ self.I_inv.T
        self.w = self.w + wdot * self.dt
        dq = qmul(self.q, torch.cat([torch.zeros_like(self.w[:, :1]), self.w], dim=1)) * 0.5
        self.q = self.q + dq * self.dt
        self.q = self.q / self.q.norm(dim=1, keepdim=True)
        return thrust


def tilt_deg(q):
    zb = rotate(q, torch.tensor([[0.0, 0.0, 1.0]], dtype=q.dtype).expand(len(q), -1))
    return torch.rad2deg(torch.acos(zb[:, 2].clamp(-1, 1)))


def fly(sim, ctrl, seconds, pos_sp, speed_scale=None, record=None):
    n = sim.n
    speed_scale = torch.ones(n, 4, dtype=sim.pos.dtype) if speed_scale is None else speed_scale
    yaw = torch.zeros(n, dtype=sim.pos.dtype)
    vz = torch.full((n,), float("nan"), dtype=sim.pos.dtype)
    steps = int(round(seconds / sim.dt))
    cmds = []
    for _ in range(steps):
        u = ctrl.step(pos_sp, vz, yaw, sim.pos, sim.vel, sim.q, sim.w)
        sim.step(u, speed_scale)
        cmds.append(u)
        if record is not None:
            record.append((sim.pos.clone(), sim.vel.clone(), tilt_deg(sim.q)))
    return torch.stack(cmds)
