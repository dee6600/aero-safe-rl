"""The project's one feature-extraction implementation (CLAUDE.md §1.4),
imported by the detector (M7), the RL policy observation (M9, PX4 side) and
-- for the shared subset only -- by the Isaac side (M8b).

Pure Python + NumPy only: no ROS, no rclpy, no I/O beyond reading
configs/features.yaml, no global mutable state. This is what lets the
isaacsim conda env (Python 3.11) import this module unmodified without
dragging in anything from the aero-safe-rl side (CLAUDE.md §0.1), and what
lets M7 iterate on it offline against recorded fixtures in seconds instead
of against a live simulator.

Causality is structural, not a discipline someone has to remember:
TelemetryWindow is append-only and rejects an out-of-order frame, so a
vector computed from a window can never have seen telemetry from after the
window's own latest frame. See tests/test_feature_extractor.py::
test_extractor_is_causal for the regression test that would catch
lookahead leakage.
"""
from __future__ import annotations

import math
from collections import deque
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import yaml

FEATURES_PATH = Path(__file__).resolve().parent.parent.parent / "configs" / "features.yaml"

STANDARD_GRAVITY_M_S2 = 9.80665

# Raw per-step fields a TelemetryWindow frame must carry -- these are exactly
# configs/schema/episode_record.yaml's schema-v3 step_fields additions
# (mission_executor.py's record_step() is the one writer of them) plus the
# t_sim_s ordering key and the already-existing position_error_m/battery_remaining
# fields from schema v2.
RAW_FRAME_FIELDS = (
    "t_sim_s",
    "roll_rad", "pitch_rad", "yaw_rad",
    "rate_p_rad_s", "rate_q_rad_s", "rate_r_rad_s",
    "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2",
    "vel_x", "vel_y", "vel_z",
    "position_error_m",
    "battery_remaining",
    "motor_0_output", "motor_1_output", "motor_2_output", "motor_3_output",
)

# Features that pass straight through from the window's latest frame --
# every configs/features.yaml entry except the one genuinely derived
# feature, thrust_accel_residual, computed in _thrust_accel_residual below.
_PASSTHROUGH_FEATURES = (
    "roll_rad", "pitch_rad", "yaw_rad",
    "rate_p_rad_s", "rate_q_rad_s", "rate_r_rad_s",
    "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2",
    "vel_x", "vel_y", "vel_z",
    "position_error_m",
    "battery_remaining",
    "motor_0_output", "motor_1_output", "motor_2_output", "motor_3_output",
)

_MOTOR_FIELDS = ("motor_0_output", "motor_1_output", "motor_2_output", "motor_3_output")
_ACCEL_FIELDS = ("accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2")


class FeatureConfigError(ValueError):
    """configs/features.yaml is missing a field or internally inconsistent."""


_features_config_cache: dict | None = None


def load_features_config() -> dict:
    """Parses configs/features.yaml once per process and caches it -- the
    same pattern as experiments/episode_schema.py's load_schema(), for the
    same reason (this gets called at up to window.rate_hz)."""
    global _features_config_cache
    if _features_config_cache is None:
        _features_config_cache = yaml.safe_load(FEATURES_PATH.read_text())
    return _features_config_cache


def _feature_entries() -> list[dict]:
    cfg = load_features_config()
    entries = cfg.get("features")
    if not entries:
        raise FeatureConfigError("configs/features.yaml has no 'features' list")
    return entries


def all_feature_names() -> tuple[str, ...]:
    """Every v1 feature name, in the fixed order FeatureExtractor.extract()
    returns them in."""
    return tuple(f["name"] for f in _feature_entries())


def shared_feature_names() -> tuple[str, ...]:
    return tuple(f["name"] for f in _feature_entries() if f["side"] == "shared")


def px4_only_feature_names() -> tuple[str, ...]:
    return tuple(f["name"] for f in _feature_entries() if f["side"] == "px4_only")


def feature_version() -> str:
    return str(load_features_config()["feature_version"])


def window_capacity() -> int:
    """Number of frames a TelemetryWindow holds, derived from
    configs/features.yaml's window.length_s / window.rate_hz, rounded to the
    nearest whole frame (minimum 1)."""
    w = load_features_config()["window"]
    return max(1, round(w["length_s"] * w["rate_hz"]))


class TelemetryWindow:
    """A causal, fixed-capacity buffer of raw telemetry frames.

    append() is the *only* way to add data, and it rejects a frame whose
    t_sim_s is earlier than the last one already in the buffer. That single
    invariant is what makes FeatureExtractor.extract(window) causal by
    construction: nothing can ever place a frame from the future ahead of
    where it belongs, and nothing can hand the extractor a frame later than
    the window's own "now".
    """

    def __init__(self, capacity: int | None = None):
        cap = capacity if capacity is not None else window_capacity()
        if cap < 1:
            raise ValueError("capacity must be >= 1")
        self.capacity = cap
        self._frames: deque[dict] = deque(maxlen=cap)

    def append(self, frame: Mapping[str, float]) -> None:
        missing = [f for f in RAW_FRAME_FIELDS if f not in frame]
        if missing:
            raise ValueError(f"telemetry frame missing required fields: {missing}")
        if self._frames and frame["t_sim_s"] < self._frames[-1]["t_sim_s"]:
            raise ValueError(
                f"frames must be appended in non-decreasing t_sim_s order "
                f"(got {frame['t_sim_s']} after {self._frames[-1]['t_sim_s']})"
            )
        self._frames.append(dict(frame))

    def __len__(self) -> int:
        return len(self._frames)

    @property
    def frames(self) -> list[dict]:
        """A snapshot list, oldest first. A copy -- mutating the result does
        not affect the window."""
        return list(self._frames)


def _thrust_accel_residual(frames: Sequence[Mapping[str, float]]) -> float:
    """See configs/features.yaml's thrust_accel_residual entry for the full
    rationale. Self-relative (this instant vs. the window's own baseline)
    rather than an absolute hover-thrust physics constant, deliberately: the
    accelerometer's sign convention was not independently re-verified when
    this was written, and a magnitude-based, self-relative formula is
    correct regardless of that sign.

    Zero on a single-frame window (no baseline yet to deviate from) rather
    than raising or producing NaN -- a fresh episode's first tick must
    still produce a valid, finite feature vector.
    """
    if len(frames) < 2:
        return 0.0

    thrust_fracs = np.array(
        [np.mean([f[m] for m in _MOTOR_FIELDS]) for f in frames], dtype=float)
    accel_mags = np.array(
        [math.sqrt(sum(f[a] ** 2 for a in _ACCEL_FIELDS)) for f in frames], dtype=float)

    thrust_delta = thrust_fracs[-1] - thrust_fracs.mean()
    accel_delta = (accel_mags[-1] - accel_mags.mean()) / STANDARD_GRAVITY_M_S2
    return float(thrust_delta - accel_delta)


class FeatureExtractor:
    """Constructed once (reads configs/features.yaml), then called on
    successive causal windows. Stateless across calls -- extract() is a
    pure function of the window it is given, which is what makes
    test_extract_is_deterministic and test_extractor_is_causal meaningful.
    """

    def __init__(self):
        self.feature_names = all_feature_names()

    def extract(self, window: TelemetryWindow) -> dict[str, float]:
        if len(window) == 0:
            raise ValueError("cannot extract features from an empty window")
        frames = window.frames
        latest = frames[-1]

        values: dict[str, float] = {}
        for name in _PASSTHROUGH_FEATURES:
            values[name] = float(latest[name])
        values["thrust_accel_residual"] = _thrust_accel_residual(frames)

        # Fixed order, always -- every consumer (detector, policy obs
        # builder, tests) indexes this dict by configs/features.yaml's
        # declared order via self.feature_names, never dict iteration order.
        return {name: values[name] for name in self.feature_names}

    def extract_vector(self, window: TelemetryWindow) -> np.ndarray:
        """extract(), as a fixed-order NumPy vector -- what a detector model
        or an RL observation builder actually consumes."""
        d = self.extract(window)
        return np.array([d[name] for name in self.feature_names], dtype=np.float32)


def extract_series(frames: Sequence[Mapping[str, float]],
                    extractor: FeatureExtractor | None = None,
                    capacity: int | None = None) -> list[dict[str, float]]:
    """Offline/batch helper: runs FeatureExtractor causally over a full,
    already-recorded episode's frames (oldest first), producing one feature
    dict per input frame. Used by M6's dataset builder and by
    experiments/analysis/compute_normalization_stats.py so neither has to
    re-implement causal windowing.

    Causal by construction: the vector produced for frames[i] is computed
    from a window built out of frames[:i+1] only, via the same
    TelemetryWindow.append() used for online/live extraction -- so this is
    provably the same computation an online caller would have gotten at
    that point in the episode, not a second implementation of it.
    """
    ex = extractor if extractor is not None else FeatureExtractor()
    window = TelemetryWindow(capacity=capacity)
    out = []
    for frame in frames:
        window.append(frame)
        out.append(ex.extract(window))
    return out
