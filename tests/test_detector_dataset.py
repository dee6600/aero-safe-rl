"""M7 task 1: ai/detector/dataset.py -- per-tick labels, episode inclusion
rules, and the by-episode split. Hand-built episodes with known answers; no
simulator, no real dataset."""
from __future__ import annotations

import inspect
import math

import numpy as np
import pandas as pd
import pytest

from ai.detector import dataset as ds
from ai.detector.dataset import (
    EpisodeCategory, EpisodeData, build_episode, classify_episode, load_episodes,
    split_by_episode, tick_labels,
)
from ai.features.feature_extractor import RAW_FRAME_FIELDS, all_feature_names, extract_series

RATE_HZ = 10.0


def _summary(**over) -> dict:
    s = dict(
        worker_id=0, episode_id="ep_0000", valid=True, fault_applied=False,
        fault_rotor_index=-1, fault_severity_commanded=0.0,
        fault_onset_time_s_requested=math.nan, fault_onset_time_s_observed=math.nan,
        fault_profile="none", fault_ramp_duration_s=0.0,
    )
    s.update(over)
    return s


def _faulty(**over) -> dict:
    base = dict(fault_applied=True, fault_rotor_index=2, fault_severity_commanded=0.5,
                fault_onset_time_s_requested=2.0, fault_onset_time_s_observed=2.1,
                fault_profile="step")
    base.update(over)
    return _summary(**base)


def _elapsed(n: int) -> np.ndarray:
    return np.arange(n) / RATE_HZ


def _steps(n: int, t0: float = 100.0, seed: int = 0) -> pd.DataFrame:
    rng = np.random.default_rng(seed)
    data = {f: rng.normal(size=n) for f in RAW_FRAME_FIELDS if f != "t_sim_s"}
    data["t_sim_s"] = t0 + _elapsed(n)
    return pd.DataFrame(data)


# --- inclusion rules -------------------------------------------------------

@pytest.mark.parametrize("summary, n, expected", [
    (_summary(valid=False), 50, EpisodeCategory.EXCLUDED_INVALID),
    (_summary(), 0, EpisodeCategory.EXCLUDED_NO_STEPS),
    (_summary(), 50, EpisodeCategory.HEALTHY),
    (_faulty(), 50, EpisodeCategory.FAULT_APPLIED),
    # onset requested at 10 s, flight only 5 s long, plugin never echoed
    (_faulty(fault_onset_time_s_requested=10.0, fault_onset_time_s_observed=math.nan),
     50, EpisodeCategory.FAULT_NEVER_ONSET),
    # onset reached but the plugin never echoed it
    (_faulty(fault_onset_time_s_observed=math.nan), 50,
     EpisodeCategory.EXCLUDED_ONSET_UNVERIFIED),
    # echo 1.5 s after the commanded tick: actual onset time unknown
    (_faulty(fault_onset_time_s_observed=3.5), 50, EpisodeCategory.EXCLUDED_ONSET_UNVERIFIED),
    # echo without the onset ever being reached: inconsistent record
    (_faulty(fault_onset_time_s_requested=10.0, fault_onset_time_s_observed=9.0),
     50, EpisodeCategory.EXCLUDED_ONSET_UNVERIFIED),
])
def test_classify_episode(summary, n, expected):
    assert classify_episode(summary, _elapsed(n)) == expected


def test_excluded_categories_are_not_included():
    assert {c for c in EpisodeCategory if not c.included} == {
        EpisodeCategory.EXCLUDED_INVALID, EpisodeCategory.EXCLUDED_NO_STEPS,
        EpisodeCategory.EXCLUDED_ONSET_UNVERIFIED}


# --- per-tick labels -------------------------------------------------------

def test_step_labels_start_exactly_at_the_commanded_tick():
    active, rotor, sev = tick_labels(_faulty(), _elapsed(50))
    onset = 20  # first tick with elapsed >= 2.0 s at 10 Hz
    assert not active[:onset].any()
    assert active[onset:].all()
    assert (rotor[:onset] == 0).all() and (rotor[onset:] == 3).all()  # rotor 2 -> class 3
    np.testing.assert_allclose(sev[onset:], 0.5)
    assert (sev[:onset] == 0).all()


def test_onset_between_ticks_rounds_up_to_the_next_tick():
    # The runner commands on the first tick whose elapsed time has *reached*
    # onset, never on the tick before it.
    active, _, _ = tick_labels(_faulty(fault_onset_time_s_requested=2.05), _elapsed(50))
    assert np.flatnonzero(active)[0] == 21


def test_ramp_labels_rise_linearly_then_hold():
    summary = _faulty(fault_profile="ramp", fault_ramp_duration_s=1.0,
                      fault_severity_commanded=0.8)
    active, rotor, sev = tick_labels(summary, _elapsed(50))
    onset = 20
    # The ramp's onset tick itself is commanded 0.0, so it is still healthy.
    assert sev[onset] == 0 and not active[onset]
    np.testing.assert_allclose(sev[onset:onset + 11], 0.8 * np.arange(11) / 10, atol=1e-6)
    np.testing.assert_allclose(sev[onset + 10:], 0.8)
    assert active[onset + 1:].all()
    assert (rotor[active] == 3).all() and (rotor[~active] == 0).all()


def test_never_onset_and_healthy_episodes_are_all_healthy_ticks():
    for summary in (_summary(),
                    _faulty(fault_onset_time_s_requested=10.0,
                            fault_onset_time_s_observed=math.nan)):
        active, rotor, sev = tick_labels(summary, _elapsed(50))
        assert not active.any() and (rotor == 0).all() and (sev == 0).all()


# --- episode records -------------------------------------------------------

def test_build_episode_uses_the_one_feature_extractor():
    steps = _steps(40)
    ep = build_episode(_faulty(), steps)
    assert isinstance(ep, EpisodeData)
    expected = extract_series(steps[list(RAW_FRAME_FIELDS)].to_dict("records"))
    names = all_feature_names()
    np.testing.assert_allclose(ep.features, [[f[n] for n in names] for f in expected], rtol=1e-6)
    assert ep.features.shape == (40, len(names))
    assert ep.key == "0/ep_0000"
    # elapsed is measured from the first logged tick, not from absolute sim time
    assert ep.elapsed_s[0] == 0.0


def test_build_episode_returns_category_for_excluded():
    assert build_episode(_summary(valid=False), _steps(10)) == EpisodeCategory.EXCLUDED_INVALID
    assert build_episode(_summary(), pd.DataFrame()) == EpisodeCategory.EXCLUDED_NO_STEPS


def test_never_onset_episode_carries_no_target_severity():
    ep = build_episode(_faulty(fault_onset_time_s_requested=10.0,
                               fault_onset_time_s_observed=math.nan), _steps(40))
    assert ep.category == EpisodeCategory.FAULT_NEVER_ONSET
    assert ep.severity_commanded == 0.0
    assert ep.profile == "none"


def _write_episode(run_dir, summary, steps):
    wdir = run_dir / f"worker_{summary['worker_id']}"
    wdir.mkdir(parents=True, exist_ok=True)
    pd.DataFrame([summary]).to_parquet(wdir / f"episode_{summary['episode_id']}_summary.parquet")
    steps.to_parquet(wdir / f"episode_{summary['episode_id']}_steps.parquet")


def test_load_episodes_reads_a_run_directory(tmp_path):
    _write_episode(tmp_path, _faulty(worker_id=1, episode_id="ep_0003"), _steps(40))
    _write_episode(tmp_path, _summary(worker_id=0, episode_id="ep_0001"), _steps(30))
    _write_episode(tmp_path, _summary(worker_id=0, episode_id="ep_0002", valid=False), _steps(30))
    _write_episode(tmp_path, _summary(worker_id=1, episode_id="ep_0004"), pd.DataFrame())

    episodes, counts = load_episodes(tmp_path)
    assert [e.key for e in episodes] == ["0/ep_0001", "1/ep_0003"]
    assert counts["healthy"] == 1 and counts["fault_applied"] == 1
    assert counts["excluded_invalid"] == 1 and counts["excluded_no_steps"] == 1


def test_load_episodes_rejects_an_empty_directory(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_episodes(tmp_path)


# --- the split -------------------------------------------------------------

def _fake_episode(i: int, severity: float | None) -> EpisodeData:
    """A minimal EpisodeData; severity None = healthy."""
    cat = EpisodeCategory.HEALTHY if severity is None else EpisodeCategory.FAULT_APPLIED
    z = np.zeros(5)
    return EpisodeData(key=f"{i % 2}/ep_{i:04d}", category=cat,
                       severity_commanded=severity or 0.0,
                       profile="none" if severity is None else "step", elapsed_s=z,
                       features=np.zeros((5, 3), np.float32), fault_active=z.astype(bool),
                       rotor_class=z.astype(np.int64), severity=z.astype(np.float32))


def _population(n_per_stratum: int = 40) -> list[EpisodeData]:
    eps, i = [], 0
    for sev in (None, 0.3, 0.5, 0.7, 0.9):
        for _ in range(n_per_stratum):
            eps.append(_fake_episode(i, sev))
            i += 1
    return eps


def test_split_parts_are_disjoint_and_cover_every_episode():
    eps = _population()
    split = split_by_episode(eps, seed=1)
    train, val, test = set(split.train), set(split.val), set(split.test)
    assert not (train & val) and not (train & test) and not (val & test)
    assert train | val | test == {e.key for e in eps}


def test_split_is_deterministic_and_order_independent():
    eps = _population()
    a = split_by_episode(eps, seed=7)
    b = split_by_episode(list(reversed(eps)), seed=7)
    assert a == b and a.digest() == b.digest()
    assert split_by_episode(eps, seed=8).digest() != a.digest()


def test_split_is_stratified_by_severity_bucket():
    eps = _population(n_per_stratum=40)
    split = split_by_episode(eps, seed=3)
    for part, frac in (("train", 0.70), ("val", 0.15), ("test", 0.15)):
        chosen = split.select(eps, part)
        for name in {ds.stratum(e) for e in eps}:
            n = sum(ds.stratum(e) == name for e in chosen)
            assert n == round(40 * frac), (part, name, n)


def test_select_returns_whole_episodes_only():
    eps = _population()
    split = split_by_episode(eps, seed=0)
    for part in ("train", "val", "test"):
        for e in split.select(eps, part):
            assert any(e is orig for orig in eps)  # the same record, not a slice of one


def test_no_public_function_takes_timestep_indices():
    # The by-episode split is structural: nothing in this module can be
    # asked to split, select or materialise by timestep.
    forbidden = {"index", "indices", "idx", "tick", "ticks", "timestep", "timesteps", "rows"}
    for name, obj in inspect.getmembers(ds):
        if name.startswith("_") or getattr(obj, "__module__", None) != ds.__name__:
            continue
        funcs = ([obj] if inspect.isfunction(obj) else
                 [m for _, m in inspect.getmembers(obj, inspect.isfunction)] if inspect.isclass(obj)
                 else [])
        for f in funcs:
            assert not forbidden & set(inspect.signature(f).parameters), f"{name}.{f.__name__}"


def test_split_rejects_bad_fractions():
    with pytest.raises(ValueError):
        split_by_episode(_population(4), seed=0, fractions=(0.5, 0.2, 0.2))
