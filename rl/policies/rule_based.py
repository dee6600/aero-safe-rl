"""M8 task 4: the rule-based recovery baseline -- what the RL policy must beat.

A state machine over the detector's output, acting only through action_v1
(the same PolicyInput -> Action interface the RL policy gets):

    NORMAL --p >= suspect_p--> SUSPECTED --held confirm_hold_s--> confirmed:
        severity estimate <  land_severity -> RECOVERING (continue, degraded)
        severity estimate >= land_severity -> ABORTED    (land now)
    SUSPECTED --p < clear_p held clear_hold_s--> NORMAL
    RECOVERING --severity estimate rises >= land_severity--> ABORTED

planning.md's CONFIRMED is the transition out of SUSPECTED; LANDED is the
flight's outcome, judged afterwards (experiments.metrics.classify_outcome).
The fault model persists to the end of the episode (planning.md §6), so a
confirmation latches: RECOVERING and ABORTED never return to NORMAL.

Hysteresis is on both sides: suspicion needs p >= suspect_p, clearing needs
the lower p < clear_p, and each must hold for its own sim-time duration.
A detector that flickers above and below suspect_p therefore never reaches
a confirmation (each dip below suspect_p restarts the confirmation hold) and
never thrashes back to NORMAL (the dips do not reach clear_p for long).

Every number lives in configs/rl/fsm_v1.yaml; how each was chosen is written
there and in docs/recovery_baseline.md.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

from rl.policies.base_policy import Action, ActionSpec, BasePolicy, PolicyInput, load_action_spec


class FsmState(str, enum.Enum):
    NORMAL = "NORMAL"
    SUSPECTED = "SUSPECTED"
    RECOVERING = "RECOVERING"
    ABORTED = "ABORTED"


@dataclass(frozen=True)
class FsmConfig:
    suspect_p: float
    clear_p: float
    confirm_hold_s: float
    clear_hold_s: float
    suspected_speed_scale: float
    suspected_altitude_offset_m: float
    degraded_speed_scale: float
    degraded_altitude_offset_m: float
    land_severity: float

    def __post_init__(self):
        if not 0.0 <= self.clear_p < self.suspect_p <= 1.0:
            raise ValueError("need 0 <= clear_p < suspect_p <= 1 (hysteresis)")
        if self.confirm_hold_s <= 0 or self.clear_hold_s <= 0:
            raise ValueError("hold times must be positive")


def load_fsm_config(path: str | Path) -> FsmConfig:
    raw = yaml.safe_load(Path(path).read_text())
    d, r = raw["detection"], raw["responses"]
    return FsmConfig(
        suspect_p=float(d["suspect_p"]), clear_p=float(d["clear_p"]),
        confirm_hold_s=float(d["confirm_hold_s"]), clear_hold_s=float(d["clear_hold_s"]),
        suspected_speed_scale=float(r["suspected"]["speed_scale"]),
        suspected_altitude_offset_m=float(r["suspected"]["altitude_offset_m"]),
        degraded_speed_scale=float(r["degraded"]["speed_scale"]),
        degraded_altitude_offset_m=float(r["degraded"]["altitude_offset_m"]),
        land_severity=float(r["land_severity"]),
    )


class RuleBasedPolicy(BasePolicy):

    def __init__(self, config: FsmConfig, spec: Optional[ActionSpec] = None):
        self.config = config
        self.spec = spec or load_action_spec()
        self.reset()

    @classmethod
    def from_yaml(cls, path: str | Path, spec: Optional[ActionSpec] = None) -> "RuleBasedPolicy":
        return cls(load_fsm_config(path), spec)

    def reset(self) -> None:
        self.state = FsmState.NORMAL
        self._high_since: Optional[float] = None  # p >= suspect_p continuously since
        self._low_since: Optional[float] = None   # p <  clear_p continuously since
        self.confirmed_at_s: Optional[float] = None

    @property
    def state_name(self) -> str:
        return self.state.value

    def act(self, obs: PolicyInput) -> Action:
        c, t = self.config, obs.t_sim_s
        p, sev = obs.detector.p_fault, obs.detector.severity

        self._high_since = (self._high_since if self._high_since is not None else t) if p >= c.suspect_p else None
        self._low_since = (self._low_since if self._low_since is not None else t) if p < c.clear_p else None

        if self.state == FsmState.NORMAL and self._high_since is not None:
            self.state = FsmState.SUSPECTED
        if self.state == FsmState.SUSPECTED:
            if self._high_since is not None and t - self._high_since >= c.confirm_hold_s:
                self.confirmed_at_s = t
                self.state = FsmState.ABORTED if sev >= c.land_severity else FsmState.RECOVERING
            elif self._low_since is not None and t - self._low_since >= c.clear_hold_s:
                self.state = FsmState.NORMAL
        if self.state == FsmState.RECOVERING and p >= c.suspect_p and sev >= c.land_severity:
            self.state = FsmState.ABORTED

        return self._action()

    def _action(self) -> Action:
        c = self.config
        if self.state == FsmState.NORMAL:
            return self.spec.nominal()
        if self.state == FsmState.SUSPECTED:
            return Action(c.suspected_speed_scale, c.suspected_altitude_offset_m, False)
        if self.state == FsmState.RECOVERING:
            return Action(c.degraded_speed_scale, c.degraded_altitude_offset_m, False)
        return Action(0.0, c.degraded_altitude_offset_m, True)
