"""M8 task 1: what a recovery action means on the PX4 side -- the one
implementation of it (CLAUDE.md §1.4). M8b's Isaac env must reproduce these
semantics; configs/rl/action_v1.yaml is the contract both sides read.

A mission is a list of legs: fly to each waypoint and hold there
hold_time_s, then hold at the last one for final_hover_s more -- the same
sequence mission_executor.fly_mission always flew. What changes is the
setpoint: instead of jumping to the next waypoint, a "carrot" moves towards
it at speed_scale x v_max_xy_m_s, and the altitude offset moves towards the
commanded one at v_z_m_s. At the nominal action the carrot crosses a 15 m leg
in 1.25 s, so the flight is effectively the old position jump
(milestones.md M8, "Design").

Pure: advanced only by the sim time passed to update(), no clock of its own,
no ROS import, so every rule here is unit-tested without a simulator.
"""
from __future__ import annotations

import math
from typing import Any, Mapping, Optional

from rl.policies.base_policy import Action, ActionSpec, MissionProgress

Vec3 = tuple[float, float, float]


class MissionTracker:
    """One per episode. `start_xy` is the vehicle's position when the mission
    starts; the carrot begins there."""

    def __init__(self, mission: Mapping[str, Any], spec: ActionSpec,
                 start_xy: tuple[float, float] = (0.0, 0.0)):
        waypoints = [(float(x), float(y)) for x, y in mission["waypoints"]]
        if not waypoints:
            raise ValueError("mission has no waypoints")
        self._legs = [(x, y, float(mission["hold_time_s"])) for x, y in waypoints]
        self._legs.append((*waypoints[-1], float(mission["final_hover_s"])))
        self._n_waypoints = len(waypoints)
        self._altitude_m = abs(float(mission["altitude_m"]))
        self._accept_r = float(mission["acceptance_radius_m"])
        self._spec = spec

        self._carrot = (float(start_xy[0]), float(start_xy[1]))
        self._offset_m = 0.0
        self._leg = 0
        self._held_since: Optional[float] = None
        self._t_last: Optional[float] = None
        self._t_start: Optional[float] = None

    @property
    def done(self) -> bool:
        return self._leg >= len(self._legs)

    @property
    def waypoints_reached(self) -> int:
        return min(self._leg, self._n_waypoints)

    @property
    def altitude_offset_m(self) -> float:
        return self._offset_m

    def _z(self) -> float:
        return -(self._altitude_m + self._offset_m)  # NED: up is negative

    def _waypoint(self) -> tuple[float, float]:
        x, y, _ = self._legs[min(self._leg, len(self._legs) - 1)]
        return x, y

    @property
    def setpoint(self) -> Vec3:
        """Position setpoint to stream to PX4 now."""
        return (self._carrot[0], self._carrot[1], self._z())

    @property
    def target(self) -> Vec3:
        """The waypoint being flown to, at the current altitude -- what
        position_error_m is measured against, as before M8."""
        x, y = self._waypoint()
        return (x, y, self._z())

    def update(self, t_sim_s: float, position: Vec3, action: Action) -> None:
        """Advance by the sim time since the previous call, under `action`."""
        if self._t_start is None:
            self._t_start = t_sim_s
        dt = 0.0 if self._t_last is None else max(0.0, t_sim_s - self._t_last)
        self._t_last = t_sim_s
        if self.done:
            return

        dz_max = self._spec.v_z_m_s * dt
        self._offset_m += min(max(action.altitude_offset_m - self._offset_m, -dz_max), dz_max)

        wx, wy, hold_s = self._legs[self._leg]
        dx, dy = wx - self._carrot[0], wy - self._carrot[1]
        dist = math.hypot(dx, dy)
        move = action.speed_scale * self._spec.v_max_xy_m_s * dt
        if dist <= move:
            self._carrot = (wx, wy)
        else:
            self._carrot = (self._carrot[0] + dx / dist * move, self._carrot[1] + dy / dist * move)

        arrived = self._carrot == (wx, wy)
        if arrived and math.dist(position, self.target) <= self._accept_r:
            if self._held_since is None:
                self._held_since = t_sim_s
            if t_sim_s - self._held_since >= hold_s:
                self._leg += 1
                self._held_since = None
        else:
            self._held_since = None

    def progress(self, t_sim_s: float, position: Vec3) -> MissionProgress:
        x, y = self._waypoint()
        return MissionProgress(
            waypoint_index=self._n_waypoints if self.done else min(self._leg, self._n_waypoints - 1),
            n_waypoints=self._n_waypoints,
            distance_to_waypoint_m=math.hypot(x - position[0], y - position[1]),
            altitude_m=-position[2],
            elapsed_s=0.0 if self._t_start is None else t_sim_s - self._t_start,
        )
