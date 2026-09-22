"""Single source of truth for the episode record format (M3, CLAUDE.md §7).

Every writer -- aero_bridge/episode_logger.py, and everything from M4 on --
validates against this module before writing. This is the one place that
parses configs/schema/episode_record.yaml; a second parser or a second copy
of the field list anywhere else is exactly the "second implementation"
CLAUDE.md §1.4 forbids.

TerminationReason and ResetTier are Python enums so the rest of the codebase
gets typed values instead of bare strings; test_termination_reason_enum_closed
pins them equal to the schema file's termination_reasons/reset_tiers lists so
the two representations cannot drift apart silently.

FaultType and FaultProfile (v4, M6 task 2) are NOT redefined here -- they are
imported from experiments.fault_schedule, which already owns them (a fault
schedule has to define what a fault type/profile IS before anything can
sample one). Re-declaring a second copy here would be exactly the drift
CLAUDE.md §1.4 forbids; test_fault_type_enum_closed/test_fault_profile_enum_
closed pin the imported enums equal to this schema file's fault_types/
fault_profiles lists, the same pattern as termination_reason/reset_tier.
"""
from __future__ import annotations

import enum
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import yaml

from experiments.fault_schedule import FaultProfile, FaultType

SCHEMA_PATH = Path(__file__).resolve().parent.parent / "configs" / "schema" / "episode_record.yaml"
SCHEMA_VERSION = "4"

# configs/features.yaml now defines a real feature_version ("1", as of M5),
# but no writer in this repo computes and tags actual feature vectors yet --
# that is M6's dataset builder or M7's, whichever consumes
# ai/features/feature_extractor.py first (milestones.md M5: "What M5 does
# not do"). Every current writer (EpisodeRunner, run_episodes.py, SimFarm)
# still uses this placeholder.
FEATURE_VERSION_UNSET = "unversioned"


class TerminationReason(str, enum.Enum):
    """Closed enum -- extend deliberately, never write a free-text reason
    (CLAUDE.md anti-pattern 16).

    WORKER_RESTARTED and OFFBOARD_LOST were added in schema v2 (M4 tasks
    1-3); SIM_FAULT was added in M4 task 4 -- see
    configs/schema/episode_record.yaml's module comment."""
    COMPLETED = "completed"
    PREFLIGHT_FAILED = "preflight_failed"
    ARM_TIMEOUT = "arm_timeout"
    OFFBOARD_REJECTED = "offboard_rejected"
    OFFBOARD_LOST = "offboard_lost"
    HOLD_TIMEOUT = "hold_timeout"
    LAND_TIMEOUT = "land_timeout"
    EPISODE_TIMEOUT = "episode_timeout"
    WORKER_RESTARTED = "worker_restarted"
    SIM_FAULT = "sim_fault"
    ABORTED_ERROR = "aborted_error"


class ResetTier(str, enum.Enum):
    NONE = "none"
    SOFT = "soft"
    MEDIUM = "medium"
    HARD = "hard"


class SchemaValidationError(ValueError):
    """Raised by validate_episode/validate_step when a record does not
    satisfy configs/schema/episode_record.yaml."""


_schema_cache: dict | None = None


def load_schema() -> dict:
    """Parses configs/schema/episode_record.yaml once per process and caches
    it -- every episode calls validate_step() at ~10Hz, and re-reading +
    re-parsing the file on every call would make logging the bottleneck."""
    global _schema_cache
    if _schema_cache is None:
        _schema_cache = yaml.safe_load(SCHEMA_PATH.read_text())
    return _schema_cache


def digest(obj: Any) -> str:
    """Short, stable hex digest of any JSON-serialisable object -- used for
    instance_spec_digest and mission_config_digest so an episode record can
    reference exactly which identity/config produced it without embedding
    the whole (often large, always redundant-across-rows) object on every
    row. sort_keys makes it independent of dict insertion order."""
    encoded = json.dumps(obj, sort_keys=True, default=str).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()[:16]


def _check_required(record: Mapping[str, Any], required: list[str], kind: str) -> None:
    missing = [f for f in required if f not in record]
    if missing:
        raise SchemaValidationError(f"{kind} record missing required fields: {missing}")

    version = record.get("schema_version")
    if version != SCHEMA_VERSION:
        raise SchemaValidationError(
            f"{kind} record schema_version {version!r} != expected {SCHEMA_VERSION!r}"
        )


def validate_episode(record: Mapping[str, Any]) -> None:
    """Raises SchemaValidationError if `record` does not satisfy the episode
    schema. Called by EpisodeLogger.write_episode() before every write --
    CLAUDE.md §7: "a test asserts that written records validate against the
    schema", and a writer must fail loudly rather than write an invalid one."""
    schema = load_schema()
    _check_required(record, schema["episode_fields"]["required"], "episode")

    reason = record.get("termination_reason")
    valid_reasons = set(schema["termination_reasons"])
    if reason not in valid_reasons:
        raise SchemaValidationError(
            f"termination_reason {reason!r} is not one of {sorted(valid_reasons)}"
        )

    tier = record.get("reset_tier")
    valid_tiers = set(schema["reset_tiers"])
    if tier not in valid_tiers:
        raise SchemaValidationError(f"reset_tier {tier!r} is not one of {sorted(valid_tiers)}")

    fault_type = record.get("fault_type")
    valid_fault_types = set(schema["fault_types"])
    if fault_type not in valid_fault_types:
        raise SchemaValidationError(f"fault_type {fault_type!r} is not one of {sorted(valid_fault_types)}")

    fault_profile = record.get("fault_profile")
    valid_fault_profiles = set(schema["fault_profiles"])
    if fault_profile not in valid_fault_profiles:
        raise SchemaValidationError(
            f"fault_profile {fault_profile!r} is not one of {sorted(valid_fault_profiles)}")


def validate_step(row: Mapping[str, Any]) -> None:
    """Raises SchemaValidationError if `row` does not satisfy the per-step
    schema. Called by EpisodeLogger.log_step() before every write."""
    schema = load_schema()
    _check_required(row, schema["step_fields"]["required"], "step")
