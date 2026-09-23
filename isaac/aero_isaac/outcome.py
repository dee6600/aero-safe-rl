"""The Isaac side's outcome rule (configs/rl/outcome_v1.yaml), streaming:
the environment updates it once per step-record tick, so it knows at every
moment whether the drone has touched the ground and, at the end, how the
episode is judged. Stepping it over a whole recorded series gives exactly
experiments/metrics.py:classify_outcome's answer -- isaac/tests/test_outcome.py
checks that against the recorded fixture.

Termination codes (the Isaac side's equivalent of the PX4 side's
termination_reason, restricted to the ones an Isaac episode can end with):
COMPLETED, RECOVERY_LANDED, GROUND_CONTACT, TIMEOUT.
Outcome codes: MISSION_SUCCESS, SAFE_LANDING, CRASH, INCOMPLETE.
"""
from __future__ import annotations

import math

import torch

from aero_isaac.contracts import OutcomeSpec

COMPLETED, RECOVERY_LANDED, GROUND_CONTACT, TIMEOUT = 0, 1, 2, 3
MISSION_SUCCESS, SAFE_LANDING, CRASH, INCOMPLETE = 0, 1, 2, 3
OUTCOME_NAMES = ("mission_success", "safe_landing", "crash", "incomplete")


class OutcomeTracker:

    def __init__(self, spec: OutcomeSpec, num_envs: int, device=None, dtype=torch.float32):
        self.spec = spec
        n = num_envs
        self.airborne = torch.zeros(n, dtype=torch.bool, device=device)
        self.contact = torch.zeros(n, dtype=torch.bool, device=device)
        self.touchdown_speed = torch.full((n,), float("nan"), dtype=dtype, device=device)
        self.max_tilt_deg = torch.full((n,), float("nan"), dtype=dtype, device=device)
        self._prev_vz = torch.full((n,), float("nan"), dtype=dtype, device=device)

    def reset(self, env_ids: torch.Tensor) -> None:
        self.airborne[env_ids] = False
        self.contact[env_ids] = False
        for buf in (self.touchdown_speed, self.max_tilt_deg, self._prev_vz):
            buf[env_ids] = float("nan")

    def update(self, alt_m: torch.Tensor, vz_down: torch.Tensor, tilt_deg: torch.Tensor) -> torch.Tensor:
        """One tick. Returns (N,) bool: first ground contact happens on this
        tick. Tilt counts from the first airborne tick on, including ticks
        after contact (a flip on impact is a crash)."""
        self.airborne |= alt_m >= self.spec.airborne_alt_m
        new_contact = self.airborne & ~self.contact & (alt_m < self.spec.ground_contact_alt_m)
        speed = torch.fmax(self._prev_vz, vz_down)  # the tick before contact, and the contact tick
        self.touchdown_speed = torch.where(new_contact, speed, self.touchdown_speed)
        self.contact |= new_contact
        tilt = torch.where(self.airborne, tilt_deg, torch.full_like(tilt_deg, float("nan")))
        self.max_tilt_deg = torch.fmax(self.max_tilt_deg, tilt)
        self._prev_vz = vz_down.clone()
        return new_contact

    def outcome(self, termination: torch.Tensor) -> torch.Tensor:
        crash = (self.max_tilt_deg > self.spec.crash_tilt_deg) | \
                (self.touchdown_speed > self.spec.crash_touchdown_speed_m_s)
        out = torch.full_like(termination, INCOMPLETE)
        out = torch.where(self.contact, torch.full_like(out, SAFE_LANDING), out)
        out = torch.where(termination == COMPLETED, torch.full_like(out, MISSION_SUCCESS), out)
        return torch.where(crash, torch.full_like(out, CRASH), out)


def tilt_deg(roll: torch.Tensor, pitch: torch.Tensor) -> torch.Tensor:
    """The PX4 side's tilt measure: degrees of hypot(roll, pitch)."""
    return torch.hypot(roll, pitch) * (180.0 / math.pi)
