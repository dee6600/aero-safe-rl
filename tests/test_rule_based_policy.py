"""M8 task 4: the rule-based recovery controller (rl/policies/rule_based.py)."""
from dataclasses import replace
from pathlib import Path

import pytest

from ai.detector.runtime import DetectorOutput
from rl.policies.base_policy import Action, MissionProgress, PolicyInput, load_action_spec
from rl.policies.rule_based import FsmConfig, FsmState, RuleBasedPolicy, load_fsm_config
from rl.policy_driver import RecoveryConfig

REPO = Path(__file__).resolve().parent.parent
FSM_PATH = REPO / "configs" / "rl" / "fsm_v1.yaml"
SPEC = load_action_spec()
DT = 0.2  # 5 Hz decisions

CFG = FsmConfig(suspect_p=0.1, clear_p=0.05, confirm_hold_s=1.0, clear_hold_s=1.0,
                suspected_speed_scale=0.5, suspected_altitude_offset_m=-1.0,
                degraded_speed_scale=0.3, degraded_altitude_offset_m=-3.0, land_severity=0.4)


def _obs(t, p, sev=0.0):
    return PolicyInput(t_sim_s=t, features={},
                       detector=DetectorOutput(p_fault=p, rotor=1, severity=sev, uncertainty=0.0,
                                               alarm=False),
                       mission=MissionProgress(1, 5, 5.0, 5.0, t))


def _run(policy, ps, sevs=None, t0=0.0):
    """Feeds p_fault (and severity) sequences at 5 Hz; returns (states, actions)."""
    sevs = sevs if sevs is not None else [0.0] * len(ps)
    states, actions = [], []
    for i, (p, s) in enumerate(zip(ps, sevs)):
        actions.append(policy.act(_obs(t0 + i * DT, p, s)))
        states.append(policy.state)
    return states, actions


def test_healthy_stays_normal_with_nominal_action():
    policy = RuleBasedPolicy(CFG, SPEC)
    states, actions = _run(policy, [0.01] * 50)
    assert set(states) == {FsmState.NORMAL}
    assert set(actions) == {SPEC.nominal()}


def test_short_false_alarm_never_confirms():
    """Every M7 healthy false alarm lasted <= 0.9 s; with a 1.0 s hold none
    of them may confirm, and the vehicle returns to NORMAL."""
    policy = RuleBasedPolicy(CFG, SPEC)
    burst = [0.9] * 5  # 0.8 s from first to last tick high
    states, actions = _run(policy, [0.01] * 5 + burst + [0.01] * 10)
    assert FsmState.RECOVERING not in states and FsmState.ABORTED not in states
    assert FsmState.SUSPECTED in states
    assert states[-1] == FsmState.NORMAL
    assert not any(a.land for a in actions)


def test_flicker_does_not_thrash():
    """p alternating above suspect_p and between the two thresholds: stays
    SUSPECTED (never confirms, never clears)."""
    policy = RuleBasedPolicy(CFG, SPEC)
    states, _ = _run(policy, [0.9, 0.07] * 20)
    assert states[0] == FsmState.SUSPECTED
    assert set(states) == {FsmState.SUSPECTED}


def test_sustained_low_severity_fault_recovers_and_latches():
    policy = RuleBasedPolicy(CFG, SPEC)
    states, actions = _run(policy, [0.99] * 10 + [0.01] * 20, [0.3] * 30)
    # confirmed once p has been high for confirm_hold_s: tick 5 (t=1.0)
    assert states[4] == FsmState.SUSPECTED and states[5] == FsmState.RECOVERING
    assert policy.confirmed_at_s == pytest.approx(1.0)
    assert states[-1] == FsmState.RECOVERING  # latched, even after p drops
    assert actions[-1] == Action(0.3, -3.0, False)


def test_high_severity_fault_lands():
    policy = RuleBasedPolicy(CFG, SPEC)
    states, actions = _run(policy, [0.99] * 10, [0.6] * 10)
    assert states[5] == FsmState.ABORTED
    assert actions[5].land and actions[-1].land


def test_severity_threshold_is_inclusive():
    for sev, expected in ((0.3999, FsmState.RECOVERING), (0.4, FsmState.ABORTED)):
        policy = RuleBasedPolicy(CFG, SPEC)
        states, _ = _run(policy, [0.99] * 6, [sev] * 6)
        assert states[-1] == expected


def test_ramp_escalates_from_recovering_to_aborted():
    policy = RuleBasedPolicy(CFG, SPEC)
    sevs = [0.2] * 8 + [0.3, 0.35, 0.45, 0.5]
    states, actions = _run(policy, [0.99] * len(sevs), sevs)
    assert FsmState.RECOVERING in states
    assert states[-2] == FsmState.ABORTED and actions[-1].land


def test_aborted_is_irreversible():
    policy = RuleBasedPolicy(CFG, SPEC)
    states, actions = _run(policy, [0.99] * 6 + [0.0] * 10, [0.8] * 6 + [0.0] * 10)
    assert states[-1] == FsmState.ABORTED and actions[-1].land


def test_suspected_action_is_the_precaution():
    policy = RuleBasedPolicy(CFG, SPEC)
    _, actions = _run(policy, [0.01, 0.9])
    assert actions[1] == Action(0.5, -1.0, False)


def test_reset_returns_to_normal():
    policy = RuleBasedPolicy(CFG, SPEC)
    _run(policy, [0.99] * 10, [0.8] * 10)
    policy.reset()
    assert policy.state == FsmState.NORMAL and policy.confirmed_at_s is None
    assert policy.state_name == "NORMAL"


def test_hysteresis_invariant_enforced():
    with pytest.raises(ValueError):
        replace(CFG, clear_p=0.2)
    with pytest.raises(ValueError):
        replace(CFG, confirm_hold_s=0.0)


def test_shipped_config_loads_and_builds_through_recovery_config():
    cfg = load_fsm_config(FSM_PATH)
    assert cfg.clear_p < cfg.suspect_p
    driver = RecoveryConfig(policy="rule_based", policy_config="configs/rl/fsm_v1.yaml").build()
    assert isinstance(driver.policy, RuleBasedPolicy)
    assert driver.provenance["policy_config_digest"] != "none"
