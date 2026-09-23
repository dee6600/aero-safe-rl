"""M8 task 1: the one interface every recovery policy implements.

The rule-based recovery controller (M8) and the reinforcement-learning
policy (M9) both receive a PolicyInput and
return an Action, and nothing else. That is what keeps M10's comparison fair:
the learned policy cannot quietly see more, or do more, than the
rule-based controller
(milestones.md M8). The Action's meaning lives in configs/rl/action_v1.yaml,
loaded here and nowhere else; rl/mission_tracker.py turns it into setpoints.

PolicyInput carries the detector's *estimate*, never the true fault state
(CLAUDE.md §1.7). C5/C6 in M10 substitute a different DetectorOutput-shaped
value; they never add a field.
"""
from __future__ import annotations

import abc
import hashlib
import json
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import TYPE_CHECKING, Any, Mapping, Optional, Sequence

import numpy as np
import yaml

if TYPE_CHECKING:
    from ai.detector.runtime import DetectorOutput

ACTION_SPEC_PATH = Path(__file__).resolve().parent.parent.parent / "configs" / "rl" / "action_v1.yaml"
OUTCOME_SPEC_PATH = ACTION_SPEC_PATH.parent / "outcome_v1.yaml"
OBSERVATION_SPEC_PATH = ACTION_SPEC_PATH.parent / "observation_v1.yaml"
OBSERVATION_V2_PATH = ACTION_SPEC_PATH.parent / "observation_v2.yaml"
REPO = ACTION_SPEC_PATH.parent.parent.parent


class ActionSpecError(ValueError):
    """configs/rl/action_v1.yaml is missing a field or disagrees with Action."""


@dataclass(frozen=True)
class Action:
    """One decoded, in-range action. Field order is the spec's vector order."""
    speed_scale: float
    altitude_offset_m: float
    land: bool

    def to_vector(self) -> list[float]:
        return [self.speed_scale, self.altitude_offset_m, 1.0 if self.land else 0.0]


@dataclass(frozen=True)
class ActionSpec:
    action_version: str
    decision_rate_hz: float
    names: tuple[str, ...]
    low: tuple[float, ...]
    high: tuple[float, ...]
    nominal_vector: tuple[float, ...]
    land_threshold: float
    v_max_xy_m_s: float
    v_z_m_s: float
    digest: str

    @property
    def decision_period_s(self) -> float:
        return 1.0 / self.decision_rate_hz

    def decode(self, vector: Sequence[float]) -> Action:
        """Clips a raw vector (e.g. an RL policy's output) into range and
        thresholds `land`. The only way a vector becomes an Action."""
        if len(vector) != len(self.names):
            raise ActionSpecError(f"action vector has {len(vector)} dims, spec has {len(self.names)}")
        v = [min(max(float(x), lo), hi) for x, lo, hi in zip(vector, self.low, self.high)]
        return Action(speed_scale=v[0], altitude_offset_m=v[1], land=v[2] >= self.land_threshold)

    def clip(self, action: Action) -> Action:
        """Brings a hand-built Action (the rule-based controller's) into range the same way."""
        return self.decode(action.to_vector())

    def nominal(self) -> Action:
        """No recovery: today's mission flight (milestones.md M8, "Design")."""
        return self.decode(self.nominal_vector)


def load_action_spec(path: str | Path = ACTION_SPEC_PATH) -> ActionSpec:
    raw = yaml.safe_load(Path(path).read_text())
    try:
        entries = raw["actions"]
        names = tuple(e["name"] for e in entries)
        spec = ActionSpec(
            action_version=str(raw["action_version"]),
            decision_rate_hz=float(raw["decision_rate_hz"]),
            names=names,
            low=tuple(float(e["low"]) for e in entries),
            high=tuple(float(e["high"]) for e in entries),
            nominal_vector=tuple(float(e["nominal"]) for e in entries),
            land_threshold=float(raw["land_threshold"]),
            v_max_xy_m_s=float(raw["carrot"]["v_max_xy_m_s"]),
            v_z_m_s=float(raw["carrot"]["v_z_m_s"]),
            digest=hashlib.sha256(json.dumps(raw, sort_keys=True).encode()).hexdigest()[:16],
        )
    except (KeyError, TypeError) as exc:
        raise ActionSpecError(f"{path}: missing or malformed field: {exc}") from exc

    expected = tuple(f.name for f in fields(Action))
    if names != expected:
        raise ActionSpecError(f"{path}: action names {names} != Action fields {expected}")
    for name, lo, hi, nom in zip(names, spec.low, spec.high, spec.nominal_vector):
        if not lo <= nom <= hi:
            raise ActionSpecError(f"{path}: {name} nominal {nom} outside [{lo}, {hi}]")
    return spec


@dataclass(frozen=True)
class OutcomeSpec:
    """configs/rl/outcome_v1.yaml -- how a flight is judged."""
    outcome_version: str
    ground_contact_alt_m: float
    airborne_alt_m: float
    crash_touchdown_speed_m_s: float
    crash_tilt_deg: float
    sensitivity_touchdown_speeds_m_s: tuple[float, ...]
    landed_settle_s: float


def load_outcome_spec(path: str | Path = OUTCOME_SPEC_PATH) -> OutcomeSpec:
    raw = yaml.safe_load(Path(path).read_text())
    try:
        return OutcomeSpec(
            outcome_version=str(raw["outcome_version"]),
            ground_contact_alt_m=float(raw["ground_contact_alt_m"]),
            airborne_alt_m=float(raw["airborne_alt_m"]),
            crash_touchdown_speed_m_s=float(raw["crash_touchdown_speed_m_s"]),
            crash_tilt_deg=float(raw["crash_tilt_deg"]),
            sensitivity_touchdown_speeds_m_s=tuple(
                float(v) for v in raw["sensitivity_touchdown_speeds_m_s"]),
            landed_settle_s=float(raw["landed_settle_s"]),
        )
    except (KeyError, TypeError) as exc:
        raise ActionSpecError(f"{path}: missing or malformed field: {exc}") from exc


def observation_feature_names(path: str | Path = OBSERVATION_SPEC_PATH) -> tuple[str, ...]:
    """The features a policy may see, in configs/rl/observation_v1.yaml's order."""
    return tuple(yaml.safe_load(Path(path).read_text())["features"])


@dataclass(frozen=True)
class MissionProgress:
    waypoint_index: int            # waypoint currently being flown to (== n_waypoints once done)
    n_waypoints: int
    distance_to_waypoint_m: float  # horizontal, vehicle to that waypoint
    altitude_m: float              # above the spawn point
    elapsed_s: float               # sim time since the mission started


@dataclass(frozen=True)
class PolicyInput:
    """Everything a policy may see at one decision. `features` holds exactly
    configs/rl/observation_v1.yaml's features, keyed by name, raw (an RL
    policy applies the frozen normalisation itself)."""
    t_sim_s: float
    features: Mapping[str, float]
    detector: "DetectorOutput"
    mission: MissionProgress
    previous_action: Optional[Action] = None  # the action in force before this decision


@dataclass(frozen=True)
class ObservationSpec:
    """configs/rl/observation_v2.yaml: the flat observation layout."""
    obs_version: str
    entries: tuple[dict, ...]
    norm: Mapping[str, tuple[float, float]]
    digest: str

    @property
    def names(self) -> tuple[str, ...]:
        """One readable name per vector position."""
        out = []
        for e in self.entries:
            t = e.get("transform")
            suffix = f"_{t['one_hot']}" if isinstance(t, dict) and "one_hot" in t else ""
            out.append(e["source"] + suffix)
        return tuple(out)


def load_observation_spec(path: str | Path = OBSERVATION_V2_PATH) -> ObservationSpec:
    from ai.detector.model import load_normalization
    raw = yaml.safe_load(Path(path).read_text())
    norm = load_normalization(REPO / raw["normalization"])
    entries = tuple(raw["entries"])
    feature_names = set(observation_feature_names())
    for e in entries:
        block, field = e["source"].split(".", 1)
        if block not in ("feature", "detector", "mission", "previous_action"):
            raise ActionSpecError(f"{path}: unknown block in {e['source']!r}")
        if block == "feature" and field not in feature_names:
            raise ActionSpecError(f"{path}: {field!r} is not an observation_v1 feature")
    return ObservationSpec(
        obs_version=str(raw["obs_version"]), entries=entries, norm=norm,
        digest=hashlib.sha256(json.dumps(raw, sort_keys=True).encode()).hexdigest()[:16])


def _block_values(obs: PolicyInput) -> dict[str, Any]:
    if obs.previous_action is None:
        raise ValueError("PolicyInput.previous_action is required to build the observation vector")
    d = obs.detector
    return {
        "feature": dict(obs.features),
        "detector": dict(p_fault=d.p_fault, rotor=d.rotor, severity=d.severity,
                         uncertainty=d.uncertainty, alarm=d.alarm),
        "mission": asdict(obs.mission),
        "previous_action": asdict(obs.previous_action),
    }


def flatten_observation(obs: PolicyInput, spec: ObservationSpec) -> np.ndarray:
    """The policy's input vector, in observation_v2.yaml's order. The one
    PX4-side implementation; the Isaac side's is held to it by
    tests/fixtures/isaac_contract_v1.json."""
    blocks = _block_values(obs)
    out = np.empty(len(spec.entries), dtype=np.float32)
    for i, e in enumerate(spec.entries):
        block, field = e["source"].split(".", 1)
        x = float(blocks[block][field])
        t = e.get("transform")
        if t == "normalize":
            mean, std = spec.norm[field]
            x = (x - mean) / std
        elif isinstance(t, dict) and "scale" in t:
            x = x / float(t["scale"])
        elif isinstance(t, dict) and "per" in t:
            x = x / float(blocks[block][t["per"]])
        elif isinstance(t, dict) and "one_hot" in t:
            x = 1.0 if int(x) == int(t["one_hot"]) else 0.0
        elif t is not None:
            raise ActionSpecError(f"unknown transform {t!r}")
        out[i] = x
    return out


class BasePolicy(abc.ABC):
    """reset() at the start of every episode; act() once per decision."""

    def reset(self) -> None:
        pass

    @abc.abstractmethod
    def act(self, obs: PolicyInput) -> Action:
        ...

    @property
    def state_name(self) -> str:
        """Logged on every step. The rule-based controller reports its state; others ''."""
        return ""


class NominalPolicy(BasePolicy):
    """No recovery -- always the nominal action. C1 and C2 fly this, through
    the same code path C3/C4 use, so the setpoint generator is not a confound."""

    def __init__(self, spec: ActionSpec | None = None):
        self._action = (spec or load_action_spec()).nominal()

    def act(self, obs: PolicyInput) -> Action:
        return self._action


class ConstantPolicy(BasePolicy):
    """One fixed action for the whole flight -- the scripted policy the M8b
    agreement check flies in both simulators (e.g. "slow and low from the
    start")."""

    def __init__(self, vector: Sequence[float], spec: ActionSpec | None = None):
        self._action = (spec or load_action_spec()).decode(vector)

    def act(self, obs: PolicyInput) -> Action:
        return self._action
