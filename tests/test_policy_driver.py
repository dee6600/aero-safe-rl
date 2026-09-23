"""M8 task 3: rl/policy_driver.py -- running a policy on the 10 Hz step stream."""
import math
import pickle

import pytest

from ai.detector.runtime import DetectorOutput
from ai.features.feature_extractor import RAW_FRAME_FIELDS
from aero_bridge.mission_executor import _REASON_FOR_ERROR, GroundContact
from experiments.episode_schema import TerminationReason
from rl.policies.base_policy import (
    Action, BasePolicy, MissionProgress, NominalPolicy, load_action_spec, observation_feature_names)
from rl.policy_driver import NO_DETECTOR_FIELDS, PolicyDriver, RecoveryConfig

SPEC = load_action_spec()
PROGRESS = MissionProgress(1, 5, 10.0, 5.0, 3.0)


def _row(t):
    row = {f: 0.0 for f in RAW_FRAME_FIELDS}
    row["t_sim_s"] = t
    return row


class RecordingPolicy(BasePolicy):
    def __init__(self, actions=None):
        self.calls = []
        self._actions = list(actions or [])

    def act(self, obs):
        self.calls.append(obs)
        return self._actions.pop(0) if self._actions else SPEC.nominal()

    @property
    def state_name(self):
        return "RECORDING"


class FakeDetector:
    def __init__(self):
        self.resets = 0
        self.n = 0

    def reset(self):
        self.resets += 1

    def step(self, frame):
        self.n += 1
        return DetectorOutput(p_fault=0.9, rotor=2, severity=0.4, uncertainty=0.01, alarm=True)


def test_decides_at_5hz_of_sim_time_on_a_10hz_stream():
    policy = RecordingPolicy()
    driver = PolicyDriver(policy, SPEC)
    for i in range(20):
        driver.step(_row(i * 0.1), PROGRESS)
    assert len(policy.calls) == 10
    assert [round(o.t_sim_s, 1) for o in policy.calls] == [round(0.2 * k, 1) for k in range(10)]


def test_row_jitter_does_not_skip_decisions():
    policy = RecordingPolicy()
    driver = PolicyDriver(policy, SPEC)
    for t in [0.0, 0.1, 0.197, 0.3, 0.402, 0.49, 0.61, 0.7, 0.795]:
        driver.step(_row(t), PROGRESS)
    assert len(policy.calls) == 5


def test_landing_phase_never_consults_the_policy():
    policy = RecordingPolicy()
    driver = PolicyDriver(policy, SPEC)
    for i in range(10):
        driver.step(_row(i * 0.1), PROGRESS, decide=False)
    assert policy.calls == []


def test_land_latches():
    land = Action(0.0, -2.0, True)
    policy = RecordingPolicy([Action(0.5, 0.0, False), land, Action(1.0, 0.0, False)])
    driver = PolicyDriver(policy, SPEC)
    for i in range(20):
        driver.step(_row(i * 0.1), PROGRESS)
    assert len(policy.calls) == 2
    assert driver.action == land


def test_policy_output_is_clipped_to_the_spec():
    driver = PolicyDriver(RecordingPolicy([Action(3.0, -9.0, False)]), SPEC)
    assert driver.step(_row(0.0), PROGRESS) == Action(1.0, -3.5, False)


def test_rows_are_annotated_without_a_detector():
    driver = PolicyDriver(RecordingPolicy([Action(0.4, -1.0, False)]), SPEC)
    row = _row(0.0)
    driver.step(row, PROGRESS)
    for k, v in NO_DETECTOR_FIELDS.items():
        assert (math.isnan(row[k]) and math.isnan(v)) or row[k] == v
    assert row["policy_state"] == "RECORDING"
    assert (row["action_speed_scale"], row["action_altitude_offset_m"], row["action_land"]) == (0.4, -1.0, False)


def test_policy_sees_detector_output_and_only_observation_features():
    policy = RecordingPolicy()
    det = FakeDetector()
    driver = PolicyDriver(policy, SPEC, detector=det)
    row = _row(0.0)
    driver.step(row, PROGRESS)
    obs = policy.calls[0]
    assert obs.detector.p_fault == 0.9 and obs.detector.rotor == 2
    assert tuple(obs.features) == observation_feature_names()
    assert "motor_0_output" not in obs.features  # px4_only
    assert obs.mission == PROGRESS
    assert (row["det_p_fault"], row["det_rotor"], row["det_alarm"]) == (0.9, 2, True)


def test_detector_runs_on_every_row_including_landing():
    det = FakeDetector()
    driver = PolicyDriver(NominalPolicy(SPEC), SPEC, detector=det)
    for i in range(6):
        driver.step(_row(i * 0.1), PROGRESS, decide=i < 3)
    assert det.n == 6


def test_reset_clears_the_latch_and_the_detector():
    det = FakeDetector()
    driver = PolicyDriver(RecordingPolicy([Action(0.0, 0.0, True)]), SPEC, detector=det)
    driver.step(_row(0.0), PROGRESS)
    assert driver.action.land
    driver.reset()
    assert driver.action == SPEC.nominal() and det.resets == 2
    driver.step(_row(0.0), PROGRESS)  # time restarts at 0 without raising


def test_recovery_config_default_is_no_recovery():
    cfg = RecoveryConfig()
    prov = cfg.provenance()
    assert prov["policy_name"] == "nominal"
    assert prov["policy_config_digest"] == "none" and prov["detector_checkpoint_digest"] == "none"
    assert prov["action_spec_digest"] == SPEC.digest
    driver = cfg.build()
    assert isinstance(driver.policy, NominalPolicy) and driver.detector is None
    assert pickle.loads(pickle.dumps(cfg)) == cfg  # crosses the spawn boundary


def test_recovery_config_rejects_unknown_or_unconfigured_policy():
    with pytest.raises(ValueError):
        RecoveryConfig(policy="magic").build()
    with pytest.raises(ValueError):
        RecoveryConfig(policy="rule_based").build()


def test_ground_contact_maps_to_its_termination_reason():
    assert _REASON_FOR_ERROR[GroundContact] == TerminationReason.GROUND_CONTACT


def test_policy_sees_the_previous_action():
    policy = RecordingPolicy([Action(0.4, -1.0, False), Action(0.2, -2.0, False)])
    driver = PolicyDriver(policy, SPEC)
    for i in range(4):
        driver.step(_row(i * 0.1), PROGRESS)
    assert policy.calls[0].previous_action == SPEC.nominal()
    assert policy.calls[1].previous_action == Action(0.4, -1.0, False)


def test_constant_policy_through_recovery_config():
    cfg = RecoveryConfig(policy="constant", constant_action=(0.3, -3.0, 0.0))
    driver = cfg.build()
    assert driver.step(_row(0.0), PROGRESS) == Action(0.3, -3.0, False)
    assert cfg.provenance()["policy_config_digest"] != "none"
    assert pickle.loads(pickle.dumps(cfg)) == cfg
    with pytest.raises(ValueError):
        RecoveryConfig(policy="constant").build()
