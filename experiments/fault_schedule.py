"""M6 task 1: the fault schedule contract -- one seeded, pure sampler that
turns configs/faults/*.yaml into a list of per-episode fault specs.

FaultSpec is the ground-truth label experiments/episode_runner.py (task 6)
uses to drive onset timing during a flight and experiments/episode_schema.py
(schema v4) writes into every episode record. It is a LABEL, never a policy
input (CLAUDE.md anti-pattern 12) -- nothing here computes or exposes
anything shaped like a feature vector; see
tests/test_fault_fields_not_in_observation.py for the test that guards this.

sample_fault_schedule() is a pure function of (config, seeded Generator,
n_episodes): no simulator, no I/O, no global RNG state (CLAUDE.md
anti-pattern 16 -- always a Generator passed explicitly, never
np.random.seed()). Two calls with the same seed produce the same schedule,
which is what lets a worker respawned mid-dataset-generation (M6 task 9)
resume from its own episode index rather than needing to remember what it
already sampled.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np
import yaml

REPO = Path(__file__).resolve().parent.parent
FAULT_SCHEMA_VERSION = "1"

_SUPPORTED_FAULT_TYPES = ("rotor_thrust_degradation",)
_SUPPORTED_PROFILES = ("step", "ramp")


class FaultConfigError(ValueError):
    """A fault schedule YAML is missing a field, internally inconsistent, or
    cannot fit inside its own mission's sim-time budget."""


class FaultProfile(str, enum.Enum):
    NONE = "none"
    STEP = "step"
    RAMP = "ramp"


class FaultType(str, enum.Enum):
    NONE = "none"
    ROTOR_THRUST_DEGRADATION = "rotor_thrust_degradation"


@dataclass(frozen=True)
class FaultSpec:
    """One episode's ground-truth fault label. `fault_applied=False` is the
    healthy case -- every other field is then a fixed sentinel
    (rotor_index=-1, severity=0.0, onset_time_s=None, profile=NONE,
    ramp_duration_s=0.0), never left as "whatever the last draw happened to
    produce", so a healthy record is unambiguous to read back later.

    `onset_time_s` is `Optional[float]` (`None` when not applicable) rather
    than NaN: a frozen dataclass with a NaN field is not equal to itself
    under `==` (`nan != nan`), which silently breaks determinism/equality
    tests and any future deduplication logic. `None` has no such trap.
    Schema v4 (task 2) maps `None` -> NaN only at the point a FaultSpec is
    written into a parquet/pandas column, where NaN is this project's
    established missing-float convention (see mission_executor.py's
    `battery_remaining`)."""
    episode_index: int
    fault_applied: bool
    fault_type: FaultType
    rotor_index: int
    severity: float
    onset_time_s: Optional[float]
    profile: FaultProfile
    ramp_duration_s: float

    @staticmethod
    def healthy(episode_index: int) -> "FaultSpec":
        return FaultSpec(
            episode_index=episode_index, fault_applied=False,
            fault_type=FaultType.NONE, rotor_index=-1, severity=0.0,
            onset_time_s=None, profile=FaultProfile.NONE,
            ramp_duration_s=0.0,
        )

    def to_episode_fields(self) -> dict[str, Any]:
        """Maps this spec onto configs/schema/episode_record.yaml's
        fault_*_requested episode fields -- the REQUESTED side only. The
        OBSERVED/CONFIRMED fields, and fault_config_digest, are runtime/
        caller concerns (they depend on what actually happened during the
        flight, which this pure dataclass has no way to know) filled in
        separately by whoever drives the episode --
        experiments/episode_runner.py, task 6. `None` -> NaN is the one
        place that conversion happens, at this schema-write boundary (see
        this class's own docstring for why `onset_time_s` is `None`, not
        NaN, everywhere else)."""
        return dict(
            fault_applied=self.fault_applied,
            fault_type=self.fault_type.value,
            fault_rotor_index=self.rotor_index,
            fault_severity_commanded=self.severity,
            fault_onset_time_s_requested=(
                self.onset_time_s if self.onset_time_s is not None else float('nan')),
            fault_profile=self.profile.value,
            fault_ramp_duration_s=self.ramp_duration_s,
        )


_REQUIRED_TOP_FIELDS = (
    "fault_schema_version", "fault_type", "mission_id", "dataset",
    "rotor_indices", "severity_range_s", "onset_time_range_s", "profiles",
    "ramp_duration_range_s",
)
_REQUIRED_DATASET_FIELDS = ("n_episodes", "seed", "healthy_fraction")


def load_fault_config(path: str | Path) -> dict[str, Any]:
    """Loads and validates a fault schedule YAML against its own mission's
    sim-time budget. The only sanctioned way to get a fault config dict --
    nothing else in this project parses one."""
    cfg = yaml.safe_load(Path(path).read_text())
    validate_fault_config(cfg)
    return cfg


def validate_fault_config(cfg: dict[str, Any]) -> None:
    """Raises FaultConfigError if `cfg` is missing a field, internally
    inconsistent, or if its worst-case onset+ramp could leave no time for
    the fault to matter within its own mission's sim-time budget."""
    missing = [f for f in _REQUIRED_TOP_FIELDS if f not in cfg]
    if missing:
        raise FaultConfigError(f"fault config missing required fields: {missing}")
    missing_ds = [f for f in _REQUIRED_DATASET_FIELDS if f not in cfg["dataset"]]
    if missing_ds:
        raise FaultConfigError(f"fault config 'dataset' missing required fields: {missing_ds}")

    if cfg["fault_type"] not in _SUPPORTED_FAULT_TYPES:
        raise FaultConfigError(
            f"fault_type {cfg['fault_type']!r} not supported; v1 implements "
            f"exactly {_SUPPORTED_FAULT_TYPES} (planning.md §6)")

    rotors = cfg["rotor_indices"]
    if not rotors or any(r not in (0, 1, 2, 3) for r in rotors):
        raise FaultConfigError(f"rotor_indices must be a non-empty subset of 0..3, got {rotors}")

    profiles = cfg["profiles"]
    if not profiles or any(p not in _SUPPORTED_PROFILES for p in profiles):
        raise FaultConfigError(
            f"profiles must be a non-empty subset of {_SUPPORTED_PROFILES}, got {profiles} "
            f"('intermittent' is deferred past v1 -- planning.md §6's backlog table)")

    lo, hi = cfg["severity_range_s"]
    if not (0.0 <= lo < hi <= 1.0):
        raise FaultConfigError(f"severity_range_s must satisfy 0 <= lo < hi <= 1, got [{lo}, {hi}]")

    onset_lo, onset_hi = cfg["onset_time_range_s"]
    if not (0.0 <= onset_lo < onset_hi):
        raise FaultConfigError(
            f"onset_time_range_s must satisfy 0 <= lo < hi, got [{onset_lo}, {onset_hi}]")

    ramp_lo, ramp_hi = cfg["ramp_duration_range_s"]
    if not (0.0 < ramp_lo <= ramp_hi):
        raise FaultConfigError(
            f"ramp_duration_range_s must satisfy 0 < lo <= hi, got [{ramp_lo}, {ramp_hi}]")

    hf = cfg["dataset"]["healthy_fraction"]
    if not (0.0 <= hf < 1.0):
        raise FaultConfigError(f"dataset.healthy_fraction must satisfy 0 <= f < 1, got {hf}")

    # The worst case this config can sample must still leave at least one
    # hold + the final hover of mission time remaining -- a fault "onsetting"
    # after the mission has effectively ended would never be observable.
    from aero_bridge.mission_executor import load_mission  # deferred: pulls in px4_msgs
    mission_path = REPO / "configs" / "missions" / f"{cfg['mission_id']}.yaml"
    mission = load_mission(mission_path)
    max_ramp = ramp_hi if "ramp" in profiles else 0.0
    worst_case_tail_needed = mission["hold_time_s"] + mission["final_hover_s"]
    if onset_hi + max_ramp > mission["timeout_s"] - worst_case_tail_needed:
        raise FaultConfigError(
            f"onset_time_range_s max ({onset_hi}) + ramp_duration_range_s max "
            f"({max_ramp}) leaves less than {worst_case_tail_needed}s of mission "
            f"'{cfg['mission_id']}' timeout_s ({mission['timeout_s']}) remaining "
            f"for the fault to matter -- narrow the ranges or lengthen the mission")


def sample_fault_schedule(cfg: dict[str, Any], rng: np.random.Generator,
                           n_episodes: int) -> list[FaultSpec]:
    """Pure function: the same (cfg, rng starting state, n_episodes) always
    produces the same schedule. `cfg` must already be validated
    (load_fault_config does this) -- this function trusts it rather than
    re-validating on every call, since a dataset-generation run calls it
    once for potentially thousands of episodes."""
    rotors = cfg["rotor_indices"]
    sev_lo, sev_hi = cfg["severity_range_s"]
    onset_lo, onset_hi = cfg["onset_time_range_s"]
    profiles = cfg["profiles"]
    ramp_lo, ramp_hi = cfg["ramp_duration_range_s"]
    healthy_fraction = cfg["dataset"]["healthy_fraction"]

    schedule: list[FaultSpec] = []
    for i in range(n_episodes):
        if rng.random() < healthy_fraction:
            schedule.append(FaultSpec.healthy(i))
            continue

        rotor_index = int(rng.choice(rotors))
        severity = float(rng.uniform(sev_lo, sev_hi))
        onset_time_s = float(rng.uniform(onset_lo, onset_hi))
        profile = FaultProfile(str(rng.choice(profiles)))
        ramp_duration_s = (
            float(rng.uniform(ramp_lo, ramp_hi)) if profile == FaultProfile.RAMP else 0.0)

        schedule.append(FaultSpec(
            episode_index=i, fault_applied=True,
            fault_type=FaultType(cfg["fault_type"]), rotor_index=rotor_index,
            severity=severity, onset_time_s=onset_time_s, profile=profile,
            ramp_duration_s=ramp_duration_s,
        ))
    return schedule
