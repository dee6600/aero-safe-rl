"""M8 task 1: configs/rl/action_v1.yaml and the policy interface around it."""
from pathlib import Path

import pytest
import yaml

from ai.detector.runtime import DetectorOutput
from aero_bridge.mission_executor import load_mission
from rl.policies.base_policy import (
    ACTION_SPEC_PATH, Action, ActionSpecError, MissionProgress, NominalPolicy, PolicyInput,
    load_action_spec)

REPO = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def spec():
    return load_action_spec()


def _obs(p_fault=0.0):
    return PolicyInput(
        t_sim_s=0.0, features={},
        detector=DetectorOutput(p_fault=p_fault, rotor=0, severity=0.0, uncertainty=0.0, alarm=False),
        mission=MissionProgress(0, 5, 0.0, 5.0, 0.0))


def test_spec_matches_action_dataclass(spec):
    assert spec.action_version == "1"
    assert spec.names == ("speed_scale", "altitude_offset_m", "land")
    assert spec.decision_period_s == pytest.approx(0.2)


def test_nominal_is_no_recovery(spec):
    assert spec.nominal() == Action(speed_scale=1.0, altitude_offset_m=0.0, land=False)


def test_decode_clips_into_range(spec):
    a = spec.decode([7.0, -99.0, -3.0])
    assert a == Action(speed_scale=1.0, altitude_offset_m=-3.5, land=False)
    a = spec.decode([-1.0, 4.0, 0.2])
    assert a == Action(speed_scale=0.0, altitude_offset_m=0.0, land=False)


def test_land_threshold_is_inclusive(spec):
    assert spec.decode([1.0, 0.0, 0.5]).land is True
    assert spec.decode([1.0, 0.0, 0.4999]).land is False


def test_clip_hand_built_action(spec):
    assert spec.clip(Action(1.5, -10.0, True)) == Action(1.0, -3.5, True)


def test_wrong_length_raises(spec):
    with pytest.raises(ActionSpecError):
        spec.decode([1.0, 0.0])


def test_to_vector_round_trips(spec):
    a = Action(0.3, -2.0, True)
    assert spec.decode(a.to_vector()) == a


def test_nominal_policy_ignores_input(spec):
    policy = NominalPolicy(spec)
    assert policy.act(_obs(0.0)) == spec.nominal()
    assert policy.act(_obs(1.0)) == spec.nominal()
    assert policy.state_name == ""


def _write_spec(tmp_path, mutate):
    raw = yaml.safe_load(ACTION_SPEC_PATH.read_text())
    mutate(raw)
    p = tmp_path / "action.yaml"
    p.write_text(yaml.safe_dump(raw))
    return p


def test_reordered_actions_rejected(tmp_path):
    p = _write_spec(tmp_path, lambda r: r["actions"].reverse())
    with pytest.raises(ActionSpecError, match="Action fields"):
        load_action_spec(p)


def test_nominal_out_of_range_rejected(tmp_path):
    def mutate(r):
        r["actions"][0]["nominal"] = 2.0
    with pytest.raises(ActionSpecError, match="nominal"):
        load_action_spec(_write_spec(tmp_path, mutate))


def test_missing_field_rejected(tmp_path):
    with pytest.raises(ActionSpecError):
        load_action_spec(_write_spec(tmp_path, lambda r: r.pop("carrot")))


def test_digest_changes_with_content(tmp_path, spec):
    def mutate(r):
        r["carrot"]["v_z_m_s"] = 1.0
    assert load_action_spec(_write_spec(tmp_path, mutate)).digest != spec.digest


@pytest.mark.parametrize("mission_path", sorted((REPO / "configs" / "missions").glob("*.yaml")))
def test_lowest_altitude_stays_above_geofence_floor(spec, mission_path):
    """The most negative altitude offset must never command the vehicle
    through its mission's geofence floor (NED z_max_m)."""
    mission = load_mission(mission_path)
    lowest_z = -(abs(mission["altitude_m"]) + spec.low[1])
    assert lowest_z < mission["geofence"]["z_max_m"]
