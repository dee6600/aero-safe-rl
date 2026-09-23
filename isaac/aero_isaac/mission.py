"""The Isaac side's mission tracker: rl/mission_tracker.py's MissionTracker,
vectorised over N drones in PyTorch. Same legs, same moving setpoint, same
hold rule, same order of operations -- isaac/tests/test_mission.py replays the
PX4 side's recorded trajectories (tests/fixtures/isaac_contract_v1.json) and
requires identical outputs.

All positions are in the PX4 side's north-east-down frame (frames.py
converts from the Isaac world). Times are simulated seconds.
"""
from __future__ import annotations

import torch

from aero_isaac.contracts import ActionSpec


class MissionTracker:

    def __init__(self, mission: dict, spec: ActionSpec, num_envs: int, device=None,
                 dtype=torch.float32):
        wps = torch.tensor(mission["waypoints"], dtype=dtype, device=device)
        if len(wps) == 0:
            raise ValueError("mission has no waypoints")
        self.n_waypoints = len(wps)
        # Legs: every waypoint with hold_time_s, then the last one again with final_hover_s.
        self.legs = torch.cat([wps, wps[-1:]], dim=0)                                  # (L, 2)
        self.holds = torch.tensor([float(mission["hold_time_s"])] * len(wps)
                                  + [float(mission["final_hover_s"])], dtype=dtype, device=device)
        self.n_legs = len(self.legs)
        self.altitude_m = abs(float(mission["altitude_m"]))
        self.accept_r = float(mission["acceptance_radius_m"])
        self.v_max = spec.v_max_xy_m_s
        self.v_z = spec.v_z_m_s
        n = num_envs
        self.carrot = torch.zeros(n, 2, dtype=dtype, device=device)
        self.offset = torch.zeros(n, dtype=dtype, device=device)
        self.leg = torch.zeros(n, dtype=torch.long, device=device)
        nan = torch.full((n,), float("nan"), dtype=dtype, device=device)
        self.held_since, self.t_last, self.t_start = nan.clone(), nan.clone(), nan.clone()

    def reset(self, env_ids: torch.Tensor, start_xy: torch.Tensor) -> None:
        self.carrot[env_ids] = start_xy.to(self.carrot.dtype)
        self.offset[env_ids] = 0.0
        self.leg[env_ids] = 0
        for buf in (self.held_since, self.t_last, self.t_start):
            buf[env_ids] = float("nan")

    # ------------------------------------------------------------ views

    @property
    def done(self) -> torch.Tensor:
        return self.leg >= self.n_legs

    @property
    def waypoints_reached(self) -> torch.Tensor:
        return self.leg.clamp(max=self.n_waypoints)

    def _z(self) -> torch.Tensor:
        return -(self.altitude_m + self.offset)

    def _waypoint(self) -> torch.Tensor:
        return self.legs[self.leg.clamp(max=self.n_legs - 1)]

    @property
    def setpoint(self) -> torch.Tensor:
        return torch.cat([self.carrot, self._z()[:, None]], dim=1)

    @property
    def target(self) -> torch.Tensor:
        return torch.cat([self._waypoint(), self._z()[:, None]], dim=1)

    def progress(self, t: torch.Tensor, position: torch.Tensor) -> dict[str, torch.Tensor]:
        """MissionProgress fields, (N,) each."""
        w = self._waypoint()
        idx = torch.where(self.done, torch.full_like(self.leg, self.n_waypoints),
                          self.leg.clamp(max=self.n_waypoints - 1))
        return dict(
            waypoint_index=idx.to(position.dtype),
            n_waypoints=torch.full_like(t, float(self.n_waypoints)),
            distance_to_waypoint_m=torch.linalg.norm(w - position[:, :2], dim=1),
            altitude_m=-position[:, 2],
            elapsed_s=torch.where(torch.isnan(self.t_start), torch.zeros_like(t), t - self.t_start))

    # ------------------------------------------------------------ update

    def update(self, t: torch.Tensor, position: torch.Tensor, action: torch.Tensor) -> None:
        """Advance every drone by its sim time since its previous call.
        `action` is decoded (N, 3): speed_scale, altitude_offset_m, land."""
        self.t_start = torch.where(torch.isnan(self.t_start), t, self.t_start)
        dt = torch.where(torch.isnan(self.t_last), torch.zeros_like(t), (t - self.t_last).clamp(min=0.0))
        self.t_last = t.clone()
        active = ~self.done

        dz = self.v_z * dt
        step = torch.minimum(torch.maximum(action[:, 1] - self.offset, -dz), dz)
        self.offset = torch.where(active, self.offset + step, self.offset)

        w = self._waypoint()
        d = w - self.carrot
        dist = torch.linalg.norm(d, dim=1)
        move = action[:, 0] * self.v_max * dt
        snap = dist <= move
        moved = self.carrot + d / dist.clamp(min=1e-12)[:, None] * move[:, None]
        carrot = torch.where(snap[:, None], w, moved)
        self.carrot = torch.where(active[:, None], carrot, self.carrot)

        arrived = (self.carrot == w).all(dim=1)
        within = torch.linalg.norm(position - self.target, dim=1) <= self.accept_r
        holding = active & arrived & within
        self.held_since = torch.where(holding & torch.isnan(self.held_since), t, self.held_since)
        hold_s = self.holds[self.leg.clamp(max=self.n_legs - 1)]
        advance = holding & (t - self.held_since >= hold_s)
        self.leg = self.leg + advance.long()
        clear = active & (advance | ~holding)
        self.held_since = torch.where(clear, torch.full_like(t, float("nan")), self.held_since)
