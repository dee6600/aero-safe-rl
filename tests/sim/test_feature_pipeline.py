"""M5 task 6 (@pytest.mark.sim): the actual planning.md Phase 5 validation --
"Feature vector logged during a healthy mission with no NaNs/gaps" -- run
against a real flight, not a synthetic fixture. This is also what confirms
the quaternion/gyro/accelerometer field names and the Euler conversion
mission_executor.py's record_step() uses are actually right, not merely
plausible (M0's rule).

Flies one real mission via SimFarm (the same mechanism M4's
test_two_workers.py uses), then runs FeatureExtractor causally over the
resulting schema-v3 step log.
"""
import math

import pandas as pd

from ai.features.feature_extractor import extract_series, shared_feature_names
from experiments.sim_farm import SimFarm


def test_healthy_mission_features_have_no_nans_or_gaps(clean_sim_slate, tmp_path):
    with SimFarm(worker_count=1, mission_id="square_circuit", n_episodes_per_worker=1,
                 speed_factor=4.0, results_dir=str(tmp_path)) as farm:
        results = farm.run()

    assert len(results) == 1
    summary = results[0]
    assert summary["valid"] is True, f"episode was not valid: {summary}"

    steps_path = (tmp_path / farm.run_id / f"worker_{summary['worker_id']}"
                  / f"episode_{summary['episode_id']}_steps.parquet")
    df = pd.read_parquet(steps_path).sort_values("t_sim_s")
    assert len(df) > 10, "mission was too short to be a meaningful check"

    # No dropped steps: t_sim_s must never go backwards (TelemetryWindow's
    # own append() contract -- a tick landing on the same sim-time instant
    # as the previous one is a legitimate duplicate, not a gap; going
    # backwards would be). At speed_factor=4x, mission_executor's ~10Hz
    # control period (sim-time-gated, not wall-clock-gated) lands roughly
    # every ~0.4s of sim time between actually-recorded ticks on this
    # hardware -- measured live, one occasional ~2s outlier tick occurs
    # (CPU/DDS scheduling jitter right after arm+offboard engage, the same
    # "startup is the most fragile moment" effect M4 documents), which is
    # why the outlier bound below is generous and the check is on the
    # median (the actual recording rate), not the max -- the same choice
    # M2's telemetry sanity test made for the same reason (head/tail
    # medians, not strict per-sample checks).
    deltas = df["t_sim_s"].diff().dropna()
    assert (deltas >= 0).all(), "t_sim_s must never go backwards, step to step"
    assert deltas.median() < 1.0, f"median step spacing ({deltas.median():.3f}s) too large"
    assert deltas.max() < 5.0, (
        f"a gap this large ({deltas.max():.3f}s) means recording actually stalled, "
        f"not ordinary scheduling jitter")

    frames = df.to_dict(orient="records")
    series = extract_series(frames)
    assert len(series) == len(frames)

    for name in shared_feature_names():
        values = [row[name] for row in series]
        assert all(math.isfinite(v) for v in values), f"{name} had a NaN/Inf value"

    # Attitude sanity: this mission flies level (5m hover + waypoints at
    # constant altitude), so roll/pitch should stay small throughout, not
    # pegged at some nonsense constant -- a coarse check that
    # _quaternion_to_euler is producing physically sane numbers, not just
    # finite ones.
    roll = [row["roll_rad"] for row in series]
    pitch = [row["pitch_rad"] for row in series]
    assert max(abs(v) for v in roll) < math.radians(45), "roll implausibly large for level flight"
    assert max(abs(v) for v in pitch) < math.radians(45), "pitch implausibly large for level flight"
