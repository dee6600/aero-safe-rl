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
"""
from __future__ import annotations

import enum
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import yaml

SCHEMA_PATH = Path(__file__).resolve().parent.parent / "configs" / "schema" / "episode_record.yaml"
SCHEMA_VERSION = "1"

# M5 has not been built yet, but the schema requires feature_version on every
# record so M5 does not need to migrate an already-written field into
# existence. Every M3/M4 writer uses this placeholder until configs/features.yaml
# defines a real one.
FEATURE_VERSION_UNSET = "unversioned"


class TerminationReason(str, enum.Enum):
    """Closed enum -- extend deliberately, never write a free-text reason
    (CLAUDE.md anti-pattern 16)."""
    COMPLETED = "completed"
    PREFLIGHT_FAILED = "preflight_failed"
    ARM_TIMEOUT = "arm_timeout"
    OFFBOARD_REJECTED = "offboard_rejected"
    HOLD_TIMEOUT = "hold_timeout"
    LAND_TIMEOUT = "land_timeout"
    EPISODE_TIMEOUT = "episode_timeout"
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


def validate_step(row: Mapping[str, Any]) -> None:
    """Raises SchemaValidationError if `row` does not satisfy the per-step
    schema. Called by EpisodeLogger.log_step() before every write."""
    schema = load_schema()
    _check_required(row, schema["step_fields"]["required"], "step")
