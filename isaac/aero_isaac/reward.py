"""The training reward (configs/rl/reward_v2.yaml), per decision step, per
drone, with every part kept separate so each can be logged on its own and
reward gaming shows up as one part moving by itself (planning.md §7.2).

What it may read is fixed by its signature: the share of the mission flown,
whether the mission was completed on this step, whether the episode ended on
this step, the outcome_v1 verdict and the touchdown speed. All of these
describe what physically happened to the
flight. There is deliberately no fault argument (CLAUDE.md §1.7), and
isaac/tests/test_reward.py checks that this file never mentions one.

Parts:
  mission_success / safe_landing / crash / incomplete
      the terminal value for the verdict, on the step the episode ends --
      except that under reward_v2 a mission's success is paid on the step
      the mission is completed (last waypoint reached and held), before
      PX4's final landing, and taken back at the end if the verdict turns
      out otherwise (reward_v2.yaml says why);
  touchdown_speed
      touchdown vertical speed x its weight, on the same step, if the drone
      touched the ground;
  progress
      weight x (discount x P' - P), with P' = 0 on the step the episode ends.
      Its discounted sum over any episode from the ground is zero, so it
      speeds up learning without changing which policy is best: the four
      terminal values alone set the preferences.
"""
from __future__ import annotations

import torch

from aero_isaac.contracts import RewardSpec
from aero_isaac.outcome import CRASH, INCOMPLETE, MISSION_SUCCESS, OUTCOME_NAMES, SAFE_LANDING

PARTS = OUTCOME_NAMES + ("touchdown_speed", "progress")


class Reward:

    def __init__(self, spec: RewardSpec, discount: float, num_envs: int, device=None, dtype=torch.float32):
        if not 0.0 < discount <= 1.0:
            raise ValueError(f"discount {discount} outside (0, 1]")
        self.spec = spec
        self.discount = float(discount)
        self._p_prev = torch.zeros(num_envs, dtype=dtype, device=device)
        self._success_paid = torch.zeros(num_envs, dtype=torch.bool, device=device)

    def reset(self, env_ids: torch.Tensor) -> None:
        """A new episode starts on the ground, where the share flown is 0."""
        self._p_prev[env_ids] = 0.0
        self._success_paid[env_ids] = False

    def step(self, progress: torch.Tensor, ended: torch.Tensor, outcome: torch.Tensor,
             touchdown_speed: torch.Tensor, mission_complete: torch.Tensor | None = None) -> dict[str, torch.Tensor]:
        """One decision step. `progress` (N,) share of the mission flown now;
        `ended` (N,) bool, the episode terminated on this step (not cut off
        by the length cap); `outcome` (N,) outcome code, read only where
        ended; `touchdown_speed` (N,) m/s, NaN if never on the ground;
        `mission_complete` (N,) bool, every leg flown by now (needed when
        success is paid at mission completion). Returns {part: (N,)}."""
        zero = torch.zeros_like(progress)
        parts: dict[str, torch.Tensor] = {}
        for code in (MISSION_SUCCESS, SAFE_LANDING, CRASH, INCOMPLETE):
            parts[OUTCOME_NAMES[code]] = torch.where(ended & (outcome == code),
                                                     torch.full_like(progress, self.spec.terminal[code]), zero)
        if self.spec.success_paid_at == "mission_complete":
            if mission_complete is None:
                raise ValueError("this reward pays success at mission completion: pass mission_complete")
            success = self.spec.terminal[MISSION_SUCCESS]
            pay_now = mission_complete & ~self._success_paid
            take_back = ended & (self._success_paid | pay_now) & (outcome != MISSION_SUCCESS)
            parts["mission_success"] = torch.where(pay_now, torch.full_like(progress, success), zero) - \
                torch.where(take_back, torch.full_like(progress, success), zero)
            self._success_paid |= pay_now
        touched = ended & torch.isfinite(touchdown_speed)
        parts["touchdown_speed"] = torch.where(
            touched, self.spec.touchdown_speed_per_m_s * torch.nan_to_num(touchdown_speed), zero)
        p_next = torch.where(ended, zero, progress)
        parts["progress"] = self.spec.progress_weight * (self.discount * p_next - self._p_prev)
        self._p_prev = p_next
        return parts


def total(parts: dict[str, torch.Tensor]) -> torch.Tensor:
    return torch.stack([parts[k] for k in PARTS]).sum(0)
