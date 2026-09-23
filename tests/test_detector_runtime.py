"""M7 task 5: ai/detector/runtime.py -- streaming inference equals batch
inference over the same recorded ticks, reset() really resets, the alarm is
the metrics module's alarm, and a tick fits the 20 ms budget. Uses an
untrained checkpoint: these are properties of the plumbing, not the weights."""
from __future__ import annotations

import time

import numpy as np
import pytest

from ai.detector import model as dm
from ai.detector.runtime import DetectorRuntime
from ai.features.feature_extractor import RAW_FRAME_FIELDS, all_feature_names, extract_series
from experiments.metrics import sustained


def _frames(n=60, seed=0) -> list[dict]:
    rng = np.random.default_rng(seed)
    frames = []
    for i in range(n):
        f = {k: float(rng.normal()) for k in RAW_FRAME_FIELDS}
        f["t_sim_s"] = 50.0 + i * 0.1
        for m in range(4):
            f[f"motor_{m}_output"] = 0.6 + 0.05 * float(rng.normal()) + (0.1 if m == 1 and i > 30 else 0)
        frames.append(f)
    frames[10]["battery_remaining"] = float("nan")
    return frames


@pytest.fixture(scope="module")
def ckpt(tmp_path_factory):
    ens = dm.build_ensemble(3, dm.load_normalization(), seeds=[1, 2, 3]).eval()
    ens.temperature = 1.3
    path = tmp_path_factory.mktemp("ckpt") / "detector.pt"
    dm.save_checkpoint(path, ens, seeds=[1, 2, 3], split_digest="test", threshold=0.5)
    return path


def _batch_trace(path, frames):
    ens, _ = dm.load_checkpoint(path)
    names = all_feature_names()
    feats = np.array([[f[n] for n in names] for f in extract_series(frames)], np.float32)
    return dm.predict_episode(ens, feats)


def test_streaming_equals_batch(ckpt):
    frames = _frames()
    rt = DetectorRuntime(ckpt)
    outs = [rt.step(f) for f in frames]
    batch = _batch_trace(ckpt, frames)
    np.testing.assert_allclose([o.p_fault for o in outs], batch.score, atol=1e-5)
    np.testing.assert_allclose([o.severity for o in outs], batch.severity, atol=1e-5)
    np.testing.assert_allclose([o.uncertainty for o in outs], batch.uncertainty, atol=1e-5)
    assert [o.rotor for o in outs] == batch.rotor.tolist()


def test_alarm_matches_metrics_definition(ckpt):
    frames = _frames(seed=3)
    batch = _batch_trace(ckpt, frames)
    # A threshold inside the observed range, so the alarm actually toggles.
    thr = float(np.quantile(batch.score, 0.4))
    rt = DetectorRuntime(ckpt, threshold=thr)
    alarms = [rt.step(f).alarm for f in frames]
    assert alarms == sustained(batch.score >= thr).tolist()
    assert any(alarms) and not all(alarms)


def test_reset_starts_a_fresh_episode(ckpt):
    frames = _frames()
    rt = DetectorRuntime(ckpt)
    first = [rt.step(f).p_fault for f in frames[:40]]
    rt.reset()
    # Earlier t_sim_s than the last frame seen: only legal after reset.
    again = [rt.step(f).p_fault for f in frames[:40]]
    np.testing.assert_allclose(first, again, atol=1e-6)


def test_threshold_defaults_to_checkpoint(ckpt):
    assert DetectorRuntime(ckpt).threshold == 0.5


def test_step_fits_the_20ms_budget(ckpt):
    rt = DetectorRuntime(ckpt)
    frames = _frames(n=80)
    for f in frames[:20]:
        rt.step(f)  # warm-up
    times = []
    for f in frames[20:]:
        t0 = time.perf_counter()
        rt.step(f)
        times.append(time.perf_counter() - t0)
    assert np.median(times) < 0.020
