"""Unit tests for ai/features/feature_extractor.py (M5). No ROS, no
simulator -- FeatureExtractor is a pure function of a TelemetryWindow, and
these tests are exactly why it was built that way (planning.md Phase 5:
"lets Phase 7 iterate on it offline in seconds").
"""
import numpy as np
import pytest
import yaml

from ai.features.feature_extractor import (
    RAW_FRAME_FIELDS,
    FeatureExtractor,
    TelemetryWindow,
    all_feature_names,
    extract_series,
    feature_version,
    load_features_config,
    px4_only_feature_names,
    shared_feature_names,
    window_capacity,
)

OBSERVATION_V1_PATH = "configs/rl/observation_v1.yaml"
NORMALIZATION_V1_PATH = "configs/rl/normalization_v1.yaml"


def make_frame(t_sim_s: float, **overrides) -> dict:
    frame = {name: 0.0 for name in RAW_FRAME_FIELDS}
    frame["t_sim_s"] = t_sim_s
    frame["motor_0_output"] = 0.5
    frame["motor_1_output"] = 0.5
    frame["motor_2_output"] = 0.5
    frame["motor_3_output"] = 0.5
    frame["accel_z_m_s2"] = -9.80665
    frame["battery_remaining"] = 0.9
    frame.update(overrides)
    return frame


# --- TelemetryWindow -------------------------------------------------------

def test_window_rejects_out_of_order_append():
    w = TelemetryWindow(capacity=5)
    w.append(make_frame(1.0))
    with pytest.raises(ValueError):
        w.append(make_frame(0.5))


def test_window_accepts_equal_timestamps():
    # Non-decreasing, not strictly increasing -- two frames can legitimately
    # share a t_sim_s if a clock read is coarse; only going backwards is a bug.
    w = TelemetryWindow(capacity=5)
    w.append(make_frame(1.0))
    w.append(make_frame(1.0))
    assert len(w) == 2


def test_window_respects_capacity():
    w = TelemetryWindow(capacity=3)
    for i in range(10):
        w.append(make_frame(float(i)))
    assert len(w) == 3
    assert [f["t_sim_s"] for f in w.frames] == [7.0, 8.0, 9.0]


def test_window_append_rejects_incomplete_frame():
    w = TelemetryWindow(capacity=3)
    with pytest.raises(ValueError):
        w.append({"t_sim_s": 0.0})


def test_window_capacity_from_config_is_positive():
    assert window_capacity() >= 1


# --- FeatureExtractor: shape and order -------------------------------------

def test_extract_output_matches_feature_order():
    w = TelemetryWindow(capacity=5)
    w.append(make_frame(0.0))
    ex = FeatureExtractor()
    result = ex.extract(w)
    assert tuple(result.keys()) == all_feature_names()


def test_extract_raises_on_empty_window():
    ex = FeatureExtractor()
    with pytest.raises(ValueError):
        ex.extract(TelemetryWindow(capacity=5))


def test_extract_vector_is_float32_and_matches_dict_order():
    w = TelemetryWindow(capacity=5)
    w.append(make_frame(0.0, roll_rad=0.1, pitch_rad=-0.2))
    ex = FeatureExtractor()
    d = ex.extract(w)
    vec = ex.extract_vector(w)
    assert vec.dtype == np.float32
    assert vec.shape == (len(all_feature_names()),)
    np.testing.assert_allclose(vec, [d[n] for n in all_feature_names()], atol=1e-6)


# --- Determinism and causality ---------------------------------------------

def test_extract_is_deterministic():
    w = TelemetryWindow(capacity=5)
    for i in range(5):
        w.append(make_frame(float(i), roll_rad=0.01 * i, accel_x_m_s2=0.1 * i))
    ex = FeatureExtractor()
    a = ex.extract(w)
    b = ex.extract(w)
    assert a == b


def test_extractor_is_causal():
    """The regression test for lookahead leakage: the feature vector
    computed at frame k must not change depending on what gets appended
    after it -- whether we look at that point "online" (stop appending) or
    "in the middle of a longer run" (keep appending), the vector for frame k
    is identical."""
    frames = [
        make_frame(float(i), roll_rad=0.05 * i, motor_0_output=0.4 + 0.02 * i,
                   accel_x_m_s2=0.3 * (-1) ** i)
        for i in range(20)
    ]
    ex = FeatureExtractor()

    # Vector for frame 9 computed from a window that stops at frame 9.
    w_stopped = TelemetryWindow(capacity=15)
    for f in frames[:10]:
        w_stopped.append(f)
    vec_stopped = ex.extract(w_stopped)

    # Vector for frame 9, captured mid-way through a run that keeps going
    # to frame 19.
    w_running = TelemetryWindow(capacity=15)
    captured = None
    for i, f in enumerate(frames):
        w_running.append(f)
        if i == 9:
            captured = ex.extract(w_running)

    assert captured == vec_stopped


def test_extract_series_matches_incremental_online_extraction():
    frames = [make_frame(float(i), roll_rad=0.02 * i) for i in range(12)]
    ex = FeatureExtractor()

    batch_result = extract_series(frames, extractor=ex, capacity=5)

    online = []
    w = TelemetryWindow(capacity=5)
    for f in frames:
        w.append(f)
        online.append(ex.extract(w))

    assert batch_result == online


def test_extract_series_output_for_a_frame_is_unaffected_by_later_frames():
    frames = [make_frame(float(i), roll_rad=0.02 * i) for i in range(10)]
    ex = FeatureExtractor()

    short = extract_series(frames[:5], extractor=ex, capacity=5)
    long_ = extract_series(frames, extractor=ex, capacity=5)

    assert short == long_[:5]


# --- thrust_accel_residual --------------------------------------------------

def test_thrust_accel_residual_is_zero_on_a_single_frame_window():
    w = TelemetryWindow(capacity=5)
    w.append(make_frame(0.0))
    ex = FeatureExtractor()
    result = ex.extract(w)
    assert result["thrust_accel_residual"] == 0.0


def test_thrust_accel_residual_is_finite_and_reacts_to_a_thrust_spike():
    w = TelemetryWindow(capacity=15)
    for i in range(10):
        w.append(make_frame(float(i)))
    # A sudden jump in commanded thrust with no matching acceleration change
    # -- the degraded-rotor signature the residual is meant to catch.
    w.append(make_frame(10.0, motor_0_output=0.9, motor_1_output=0.9,
                         motor_2_output=0.9, motor_3_output=0.9))
    ex = FeatureExtractor()
    result = ex.extract(w)
    assert np.isfinite(result["thrust_accel_residual"])
    assert result["thrust_accel_residual"] > 0.0


# --- configs/features.yaml consistency -------------------------------------

def test_every_feature_has_a_valid_side():
    cfg = load_features_config()
    for f in cfg["features"]:
        assert f["side"] in ("shared", "px4_only"), f["name"]


def test_shared_and_px4_only_partition_all_features():
    assert set(shared_feature_names()) | set(px4_only_feature_names()) == set(all_feature_names())
    assert set(shared_feature_names()).isdisjoint(px4_only_feature_names())


def test_observation_v1_features_are_all_marked_shared():
    """The test the D12 constraint exists for (milestones.md M5): the
    policy observation may only ever contain features computable in both
    simulators."""
    obs = yaml.safe_load(open(OBSERVATION_V1_PATH))
    shared = set(shared_feature_names())
    missing = [name for name in obs["features"] if name not in shared]
    assert not missing, f"observation_v1.yaml references non-shared feature(s): {missing}"


def test_observation_v1_feature_version_matches_features_yaml():
    obs = yaml.safe_load(open(OBSERVATION_V1_PATH))
    assert obs["feature_version"] == feature_version()


@pytest.mark.skipif(
    not __import__("pathlib").Path(NORMALIZATION_V1_PATH).exists(),
    reason="normalization_v1.yaml is generated by M5 task 5's script, once, "
           "against a real healthy-flight dataset -- not present until that "
           "has been run.",
)
def test_normalization_v1_feature_version_matches_features_yaml():
    norm = yaml.safe_load(open(NORMALIZATION_V1_PATH))
    assert norm["feature_version"] == feature_version()
