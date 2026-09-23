"""Frame conventions between Isaac and the PX4 side -- the one place they are
converted.

Isaac world is z-up. This project fixes it as x = north, y = west, z = up, so
PX4's north-east-down (NED) vectors are a sign flip away: NED = T·world,
T = diag(1, -1, -1). Isaac bodies are forward-left-up; PX4 bodies are
forward-right-down, and T converts those too. T is a half-turn about x, so an
attitude quaternion converts by conjugation: (w, x, y, z) -> (w, x, -y, -z).

Everything the policy sees is expressed the PX4 way (configs/features.yaml
defines each feature in NED / forward-right-down), so it means the same on
both sides.
"""
from __future__ import annotations

import torch

_T = (1.0, -1.0, -1.0)


def flip(v: torch.Tensor) -> torch.Tensor:
    """(N, 3) world <-> NED, or body forward-left-up <-> forward-right-down."""
    return v * torch.tensor(_T, dtype=v.dtype, device=v.device)


def quat_ned_frd(q_world_body: torch.Tensor) -> torch.Tensor:
    """Isaac root quaternion (w, x, y, z), body-to-world -> PX4 attitude
    quaternion (w, x, y, z), forward-right-down body to NED."""
    return q_world_body * torch.tensor((1.0, 1.0, -1.0, -1.0), dtype=q_world_body.dtype,
                                       device=q_world_body.device)


def euler_ned(q: torch.Tensor) -> torch.Tensor:
    """(N, 4) Hamilton quaternion (w, x, y, z) -> (N, 3) roll, pitch, yaw in
    radians, aerospace Z-Y-X order -- the same formula as the PX4 side's
    aero_bridge/mission_executor.py:_quaternion_to_euler."""
    w, x, y, z = q.unbind(dim=1)
    roll = torch.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = torch.asin((2.0 * (w * y - z * x)).clamp(-1.0, 1.0))
    yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return torch.stack([roll, pitch, yaw], dim=1)


def rotate_to_body(q_world_body: torch.Tensor, v_world: torch.Tensor) -> torch.Tensor:
    """Express world vectors in the body frame: R(q)^T v."""
    w = q_world_body[:, :1]
    u = -q_world_body[:, 1:]  # inverse rotation
    t = 2.0 * torch.cross(u, v_world, dim=1)
    return v_world + w * t + torch.cross(u, t, dim=1)


def specific_force_frd(q_world_body: torch.Tensor, accel_world: torch.Tensor,
                       gravity: float = 9.80665) -> torch.Tensor:
    """What an accelerometer reads, forward-right-down body frame: body
    acceleration minus gravity. Level hover reads (0, 0, -g), as PX4's
    sensor_combined does."""
    g = torch.tensor((0.0, 0.0, -gravity), dtype=accel_world.dtype, device=accel_world.device)
    return flip(rotate_to_body(q_world_body, accel_world - g))
