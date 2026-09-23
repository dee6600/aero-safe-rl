"""The Isaac side's rotors: Gazebo's MulticopterMotorModel (gz-sim 8), and the
rotor-fault plugin in front of it, vectorised over N drones. The Gazebo-side
fault is simulation/gz_plugins/src/RotorDegradationSystem.cc; CLAUDE.md §1.6
requires the two to mean the same physical thing for the same severity, and
isaac/tests/test_rotor.py checks this one against the recorded fixture
tests/fixtures/rotor_fault_thrust_curve.json.

Per rotor, as in Gazebo (source read 2026-09-24):
  reference speed = min(esc_min + (esc_max - esc_min) * command, max_rot_velocity)
      -- PX4's SIM_GZ_EC_MIN/MAX output mapping (150..1000 rad/s on the x500)
  fault: the faulted rotor's reference speed is scaled by sqrt(1 - s)
      -- the plugin scales the commanded speed, so thrust and drag torque both
         become (1 - s) of normal, even at full command
  speed follows the reference with a first-order lag (tau_up / tau_down)
  thrust        = motor_constant * speed^2, along the rotor axis
  drag torque   = -turning * moment_constant * thrust, about the rotor axis
  air drag      = -speed * rotor_drag_coefficient * (rotor velocity perpendicular to its axis)
  rolling moment= -speed * rolling_moment_coefficient * (same velocity)
Gazebo 8's motor model does not scale thrust with airflow along the rotor
axis, so neither does this.
"""
from __future__ import annotations

import math

import torch

from aero_isaac.px4_model import X500


class RotorModel:

    def __init__(self, x500: X500, num_envs: int, device=None, dtype=torch.float32):
        def t(values):
            return torch.tensor(values, dtype=dtype, device=device)
        r = x500.rotors
        self.esc_min, self.esc_max = t(x500.esc_min), t(x500.esc_max)
        self.max_rot = t([x.max_rot_velocity for x in r])
        self.k_thrust = t([x.motor_constant for x in r])
        self.k_moment = t([x.moment_constant for x in r])
        self.turning = t([x.turning for x in r])
        self.tau_up = t([x.tau_up for x in r])
        self.tau_down = t([x.tau_down for x in r])
        self.k_drag = t([x.drag_coefficient for x in r])
        self.k_roll = t([x.rolling_moment_coefficient for x in r])
        self.speed = torch.zeros(num_envs, len(r), dtype=dtype, device=device)

    def reset(self, env_ids: torch.Tensor) -> None:
        self.speed[env_ids] = 0.0

    @staticmethod
    def fault_speed_scale(severity: torch.Tensor, rotor: torch.Tensor, n_rotors: int = 4) -> torch.Tensor:
        """(N,) severity in [0, 1] and faulted rotor index (-1 = none) ->
        (N, n_rotors) commanded-speed scale, sqrt(1 - s) on the faulted rotor."""
        scale = torch.ones(len(severity), n_rotors, dtype=severity.dtype, device=severity.device)
        hit = rotor >= 0
        idx = rotor.clamp(min=0)
        faulted = torch.sqrt((1.0 - severity).clamp(min=0.0))
        scale[hit, idx[hit]] = faulted[hit]
        return scale

    def reference_speed(self, command: torch.Tensor, speed_scale: torch.Tensor) -> torch.Tensor:
        ref = (self.esc_min + (self.esc_max - self.esc_min) * command.clamp(0.0, 1.0)) * speed_scale
        return torch.minimum(ref, self.max_rot)

    def step(self, command: torch.Tensor, speed_scale: torch.Tensor, dt: float
             ) -> tuple[torch.Tensor, torch.Tensor]:
        """Advance the motors by dt. Returns (thrust, drag_torque), both
        (N, 4): thrust along each rotor's axis (newtons), drag torque about it
        (newton-metres, sign per the rotor's spin)."""
        ref = self.reference_speed(command, speed_scale)
        tau = torch.where(ref > self.speed, self.tau_up, self.tau_down)
        alpha = torch.exp(-dt / tau)
        self.speed = alpha * self.speed + (1.0 - alpha) * ref
        thrust = self.k_thrust * self.speed ** 2
        return thrust, -self.turning * self.k_moment * thrust

    def air_drag(self, v_perp: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """(N, 4, 3) rotor velocity perpendicular to its axis -> (force,
        rolling moment), each (N, 4, 3), in the same frame as v_perp."""
        s = self.speed.unsqueeze(-1)
        return -s * self.k_drag[:, None] * v_perp, -s * self.k_roll[:, None] * v_perp


def hover_command(x500: X500, gravity: float = 9.80665) -> float:
    """The motor command that holds a healthy x500 in hover, from the model
    alone: each rotor carries a quarter of the weight."""
    r = x500.rotors[0]
    speed = math.sqrt(x500.mass * gravity / len(x500.rotors) / r.motor_constant)
    return (speed - x500.esc_min[0]) / (x500.esc_max[0] - x500.esc_min[0])
