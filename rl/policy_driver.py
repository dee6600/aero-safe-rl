"""M8 task 3: runs a recovery policy inside a flight, one telemetry row at a
time. mission_executor.fly_mission calls step() on every 10 Hz step record;
this module runs the detector on every row, asks the policy for an action at
action_v1's decision rate (5 Hz of *simulated* time), and writes the action,
the policy's state and the detector's output onto the row before it is
logged (episode schema v5).

Every flight goes through a PolicyDriver -- NominalPolicy and no detector
when the caller asks for nothing -- so there is one flight code path for
every condition in M10.

RecoveryConfig is the small, picklable description SimFarm hands to each
worker; the worker builds the driver itself (CLAUDE.md §3.3), so the
detector's torch state is created in the process that uses it.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import MutableMapping, Optional

from ai.features.feature_extractor import FeatureExtractor, TelemetryWindow
from rl.policies.base_policy import (
    Action, ActionSpec, BasePolicy, MissionProgress, NominalPolicy, PolicyInput,
    load_action_spec, observation_feature_names)

REPO = Path(__file__).resolve().parent.parent

# Logged on every row when no detector ran -- distinguishable from a real
# "no fault" output (which has finite values).
NO_DETECTOR_FIELDS = dict(det_p_fault=float("nan"), det_rotor=-1, det_severity=float("nan"),
                          det_uncertainty=float("nan"), det_alarm=False)

# Half a 10 Hz tick: a decision due at t fires on the first row at or after
# t minus this, so row-timing jitter cannot skip a decision.
_DECISION_SLACK_S = 0.05


def _no_detection():
    from ai.detector.runtime import DetectorOutput
    return DetectorOutput(p_fault=0.0, rotor=0, severity=0.0, uncertainty=0.0, alarm=False)


@dataclass(frozen=True)
class RecoveryConfig:
    """Which policy and detector a run flies with. Paths are repo-relative or
    absolute. The default is no recovery and no detector."""
    policy: str = "nominal"                    # "nominal" | "rule_based"
    policy_config: Optional[str] = None        # e.g. configs/rl/fsm_v1.yaml
    detector_checkpoint: Optional[str] = None  # e.g. results/m7_detector_v1/detector.pt

    def _path(self, p: Optional[str]) -> Optional[Path]:
        if p is None:
            return None
        path = Path(p)
        return path if path.is_absolute() else REPO / path

    def provenance(self) -> dict:
        """The episode-record fields naming what flew the episode."""
        from ai.detector.model import file_digest
        cfg, ckpt = self._path(self.policy_config), self._path(self.detector_checkpoint)
        return dict(
            policy_name=self.policy,
            policy_config_digest=file_digest(cfg) if cfg else "none",
            action_spec_digest=load_action_spec().digest,
            detector_checkpoint_digest=file_digest(ckpt) if ckpt else "none",
        )

    def build(self) -> "PolicyDriver":
        spec = load_action_spec()
        if self.policy == "nominal":
            policy: BasePolicy = NominalPolicy(spec)
        elif self.policy == "rule_based":
            from rl.policies.rule_based import RuleBasedPolicy
            if self.policy_config is None:
                raise ValueError("rule_based policy needs policy_config")
            policy = RuleBasedPolicy.from_yaml(self._path(self.policy_config), spec)
        else:
            raise ValueError(f"unknown policy {self.policy!r}")
        detector = None
        if self.detector_checkpoint is not None:
            from ai.detector.runtime import DetectorRuntime
            detector = DetectorRuntime(self._path(self.detector_checkpoint))
        return PolicyDriver(policy, spec, detector, provenance=self.provenance())


class PolicyDriver:
    """One per worker; reset() at the start of every episode."""

    def __init__(self, policy: BasePolicy, spec: Optional[ActionSpec] = None, detector=None,
                 provenance: Optional[dict] = None):
        self.policy = policy
        self.spec = spec or load_action_spec()
        self.detector = detector
        self.provenance = provenance or RecoveryConfig().provenance()
        self._extractor = FeatureExtractor()
        self._obs_names = observation_feature_names()
        self.reset()

    def reset(self) -> None:
        self.policy.reset()
        if self.detector is not None:
            self.detector.reset()
        self._window = TelemetryWindow()
        self._action = self.spec.nominal()
        self._next_decision_t: Optional[float] = None
        self.n_decisions = 0

    @property
    def action(self) -> Action:
        return self._action

    def step(self, row: MutableMapping, progress: MissionProgress, *, decide: bool = True) -> Action:
        """Consumes one step record, annotates it, returns the action now in
        force. With decide=False (the landing phase) the detector still runs
        and is logged, but the policy is not consulted. A land decision
        latches: once committed, the policy is never asked again."""
        self._window.append(row)
        if self.detector is not None:
            det = self.detector.step(row)
            row.update(det_p_fault=det.p_fault, det_rotor=det.rotor, det_severity=det.severity,
                       det_uncertainty=det.uncertainty, det_alarm=det.alarm)
        else:
            det = _no_detection()
            row.update(NO_DETECTOR_FIELDS)

        t = float(row["t_sim_s"])
        due = self._next_decision_t is None or t >= self._next_decision_t - _DECISION_SLACK_S
        if decide and due and not self._action.land:
            feats = self._extractor.extract(self._window)
            obs = PolicyInput(t_sim_s=t, features={n: feats[n] for n in self._obs_names},
                              detector=det, mission=progress)
            self._action = self.spec.clip(self.policy.act(obs))
            self._next_decision_t = t + self.spec.decision_period_s
            self.n_decisions += 1

        a = self._action
        row.update(policy_state=self.policy.state_name, action_speed_scale=a.speed_scale,
                   action_altitude_offset_m=a.altitude_offset_m, action_land=a.land)
        return a

