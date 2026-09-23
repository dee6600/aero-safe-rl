"""M8: the pure parts of experiments/run_recovery.py and experiments/tune_fsm.py."""
import types

import numpy as np
import pandas as pd
import pytest

from experiments.fault_schedule import FaultProfile
from experiments.run_recovery import ONSET_RANGE_S, recovery_schedule, summarize
from experiments.tune_fsm import pick_detection, replay_fsm
from rl.policies.rule_based import FsmConfig, RuleBasedPolicy

CFG = FsmConfig(suspect_p=0.1, clear_p=0.05, confirm_hold_s=1.0, clear_hold_s=1.0,
                suspected_speed_scale=0.5, suspected_altitude_offset_m=0.0,
                degraded_speed_scale=0.3, degraded_altitude_offset_m=-3.0, land_severity=0.4)


def test_schedule_cells_and_determinism():
    a = recovery_schedule([0.0, 0.4, 0.5], 4, np.random.default_rng(1))
    b = recovery_schedule([0.0, 0.4, 0.5], 4, np.random.default_rng(1))
    assert a == b and len(a) == 12
    assert [s.episode_index for s in a] == list(range(12))
    sev = sorted(s.severity for s in a)
    assert sev == [0.0] * 4 + [0.4] * 4 + [0.5] * 4
    assert sum(not s.fault_applied for s in a) == 4
    for cell in (0.4, 0.5):
        profiles = [s.profile for s in a if s.severity == cell]
        assert profiles.count(FaultProfile.STEP) == profiles.count(FaultProfile.RAMP) == 2
    for s in a:
        if s.fault_applied:
            assert ONSET_RANGE_S[0] <= s.onset_time_s <= ONSET_RANGE_S[1]
            assert (s.ramp_duration_s > 0) == (s.profile == FaultProfile.RAMP)


def test_schedule_is_shuffled_across_workers():
    """A contiguous half of the schedule (one worker's block) must not be a
    single severity -- both workers should see every cell."""
    s = recovery_schedule([0.0, 0.5], 10, np.random.default_rng(3))
    first_half = {x.severity for x in s[:10]}
    assert first_half == {0.0, 0.5}


def _trace(p, sev=0.0):
    n = len(p)
    return types.SimpleNamespace(score=np.asarray(p, float), rotor=np.zeros(n, int),
                                 severity=np.full(n, sev), uncertainty=np.zeros(n))


def test_replay_fsm_decides_at_5hz_and_reports_confirmation():
    elapsed = np.arange(40) * 0.1
    p = [0.0] * 10 + [0.9] * 30        # high from t=1.0
    suspicions, confirmed = replay_fsm(RuleBasedPolicy(CFG), elapsed, _trace(p, 0.3))
    assert suspicions == [pytest.approx(1.0)]
    assert confirmed == pytest.approx(2.0)


def test_replay_fsm_short_burst_suspects_but_never_confirms():
    elapsed = np.arange(60) * 0.1
    p = [0.0] * 10 + [0.9] * 8 + [0.0] * 42
    suspicions, confirmed = replay_fsm(RuleBasedPolicy(CFG), elapsed, _trace(p))
    assert len(suspicions) == 1 and confirmed is None


def test_pick_detection_rule():
    rows = [
        dict(suspect_p=0.1, confirm_hold_s=0.4, false_confirmations=2, band_delay_s_p50=0.5),
        dict(suspect_p=0.1, confirm_hold_s=1.0, false_confirmations=0, band_delay_s_p50=1.4),
        dict(suspect_p=0.1, confirm_hold_s=1.2, false_confirmations=0, band_delay_s_p50=1.4),
        dict(suspect_p=0.3, confirm_hold_s=0.6, false_confirmations=0, band_delay_s_p50=0.9),
        dict(suspect_p=0.3, confirm_hold_s=1.0, false_confirmations=0, band_delay_s_p50=1.6),
    ]
    runs = {0.1: 0.0, 0.3: 0.0}
    assert pick_detection(rows, runs)["confirm_hold_s"] == 0.6
    # A 0.72 s healthy run above 0.3 rules out holds shorter than 0.92 s.
    runs = {0.1: 0.0, 0.3: 0.72}
    assert pick_detection(rows, runs)["confirm_hold_s"] == 1.2  # tie at 0.1 -> longer hold
    with pytest.raises(SystemExit):
        pick_detection([rows[0]], runs)


def test_healthy_runs_are_measured_as_the_fsm_sees_them():
    from experiments.tune_fsm import healthy_runs_s
    t = np.arange(20) * 0.1
    p = np.array([0.0] * 4 + [0.9] * 7 + [0.0] * 4 + [0.9] + [0.0] * 4)
    # decisions at even ticks: 4,6,8,10 high -> 0.6 s; tick 15 is odd -> unseen
    assert healthy_runs_s(p, t, 0.5) == [pytest.approx(0.6)]


def test_summarize_groups_outcomes_by_severity():
    rows = pd.DataFrame([
        dict(severity=0.0, outcome="mission_success", policy_landed=False, t_sim_duration_s=44.0,
             touchdown_speed_m_s=0.7, crash_at_1_5=False),
        dict(severity=0.5, outcome="crash", policy_landed=True, t_sim_duration_s=20.0,
             touchdown_speed_m_s=3.0, crash_at_1_5=True),
        dict(severity=0.5, outcome="safe_landing", policy_landed=True, t_sim_duration_s=22.0,
             touchdown_speed_m_s=1.8, crash_at_1_5=True),
    ])
    s = summarize("x", rows)
    assert s["n_valid"] == 3
    c = s["by_severity"]["0.50"]
    assert (c["n"], c["crash"], c["safe_landing"], c["crash_at_1_5"]) == (2, 0.5, 0.5, 1.0)
    assert s["by_severity"]["0.00"]["mission_success"] == 1.0


def _summary(crash, success, n=4):
    from experiments.tune_fsm import SWEEP_SEVERITIES
    return {"by_severity": {f"{s:.2f}": dict(n=n, crash=crash, mission_success=success)
                            for s in SWEEP_SEVERITIES}}


def test_pick_response_rule():
    from experiments.tune_fsm import pick_response
    results = {"base": _summary(0.5, 0.2), "land_0.35": _summary(0.25, 0.1),
               "degraded_fast": _summary(0.25, 0.3)}
    assert pick_response(results) == "degraded_fast"          # tie on crash -> more success
    results["degraded_fast"] = _summary(0.25, 0.1)
    assert pick_response(results) == "degraded_fast"          # full tie -> candidate order
    results["base"] = _summary(0.25, 0.1)
    assert pick_response(results) == "base"                   # base is the simplest


def test_candidates_are_valid_fsm_configs(tmp_path):
    from experiments.tune_fsm import RESPONSE_CANDIDATES, write_candidates
    from rl.policies.rule_based import load_fsm_config
    paths = write_candidates(tmp_path)
    assert len(paths) == len(RESPONSE_CANDIDATES)
    cfgs = {p.stem: load_fsm_config(p) for p in paths}
    assert cfgs["fsm_land_0.35"].land_severity == 0.35
    assert cfgs["fsm_degraded_high"].degraded_altitude_offset_m == 0.0
    assert cfgs["fsm_suspect_descend"].suspected_altitude_offset_m == -3.0
    assert cfgs["fsm_base"] == load_fsm_config("configs/rl/fsm_v1.yaml")
