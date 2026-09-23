"""Reads the contract files the PX4 side also reads (CLAUDE.md §0.1): the
action, outcome and observation specs, the frozen normalisation statistics
and the mission. Plain dataclasses of Python numbers -- the PyTorch modules
turn them into tensors.

Digests are computed the same way the PX4 side computes them, so a test can
confirm both sides read the same files
(tests/fixtures/isaac_contract_v1.json's `contract_files`).
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import torch
import yaml

REPO = Path(__file__).resolve().parent.parent.parent
CONFIGS = REPO / "configs"


def file_digest_of_yaml(path: Path) -> str:
    """sha256 of the parsed file as sorted JSON, first 16 hex characters --
    the PX4 side's `digest()` / spec `digest` convention."""
    raw = yaml.safe_load(Path(path).read_text())
    return hashlib.sha256(json.dumps(raw, sort_keys=True, default=str).encode()).hexdigest()[:16]


@dataclass(frozen=True)
class ActionSpec:
    names: tuple[str, ...]
    low: tuple[float, ...]
    high: tuple[float, ...]
    nominal: tuple[float, ...]
    land_threshold: float
    decision_period_s: float
    v_max_xy_m_s: float
    v_z_m_s: float

    def decode(self, raw: torch.Tensor) -> torch.Tensor:
        """(N, 3) raw policy output -> (N, 3) in-range action with land as
        0.0 / 1.0 (configs/rl/action_v1.yaml)."""
        low = torch.tensor(self.low, dtype=raw.dtype, device=raw.device)
        high = torch.tensor(self.high, dtype=raw.dtype, device=raw.device)
        a = torch.maximum(torch.minimum(raw, high), low)
        land = (a[:, 2] >= self.land_threshold).to(raw.dtype)
        return torch.cat([a[:, :2], land[:, None]], dim=1)

    def nominal_tensor(self, n: int, device=None, dtype=torch.float32) -> torch.Tensor:
        return torch.tensor(self.nominal, dtype=dtype, device=device).expand(n, -1).clone()


def load_action_spec(path: Path = CONFIGS / "rl" / "action_v1.yaml") -> ActionSpec:
    raw = yaml.safe_load(Path(path).read_text())
    entries = raw["actions"]
    names = tuple(e["name"] for e in entries)
    if names != ("speed_scale", "altitude_offset_m", "land"):
        raise ValueError(f"{path}: unexpected action layout {names}")
    return ActionSpec(
        names=names, low=tuple(float(e["low"]) for e in entries),
        high=tuple(float(e["high"]) for e in entries),
        nominal=tuple(float(e["nominal"]) for e in entries),
        land_threshold=float(raw["land_threshold"]),
        decision_period_s=1.0 / float(raw["decision_rate_hz"]),
        v_max_xy_m_s=float(raw["carrot"]["v_max_xy_m_s"]), v_z_m_s=float(raw["carrot"]["v_z_m_s"]))


@dataclass(frozen=True)
class OutcomeSpec:
    ground_contact_alt_m: float
    airborne_alt_m: float
    crash_touchdown_speed_m_s: float
    crash_tilt_deg: float
    landed_settle_s: float


def load_outcome_spec(path: Path = CONFIGS / "rl" / "outcome_v1.yaml") -> OutcomeSpec:
    raw = yaml.safe_load(Path(path).read_text())
    return OutcomeSpec(*(float(raw[k]) for k in (
        "ground_contact_alt_m", "airborne_alt_m", "crash_touchdown_speed_m_s", "crash_tilt_deg",
        "landed_settle_s")))


@dataclass(frozen=True)
class ObservationSpec:
    entries: tuple[dict, ...]
    norm: dict[str, tuple[float, float]]


def load_observation_spec(path: Path = CONFIGS / "rl" / "observation_v2.yaml") -> ObservationSpec:
    raw = yaml.safe_load(Path(path).read_text())
    norm_raw = yaml.safe_load((REPO / raw["normalization"]).read_text())
    if str(norm_raw["feature_version"]) != str(raw["feature_version"]):
        raise ValueError("normalisation file is for a different feature_version")
    norm = {k: (float(v["mean"]), float(v["std"])) for k, v in norm_raw["stats"].items()}
    return ObservationSpec(entries=tuple(raw["entries"]), norm=norm)


def load_mission(name: str = "square_circuit") -> dict:
    return yaml.safe_load((CONFIGS / "missions" / f"{name}.yaml").read_text())
