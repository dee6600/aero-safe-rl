"""M8b task 1: configs/rl/observation_v2.yaml and the PX4-side builder
(rl.policies.base_policy.flatten_observation), plus the shared fixture."""
import pytest

from ai.detector.runtime import DetectorOutput
from experiments.write_isaac_fixtures import main as fixture_main
from rl.policies.base_policy import (
    Action, MissionProgress, PolicyInput, flatten_observation, load_observation_spec,
    observation_feature_names)

SPEC = load_observation_spec()


def _obs(**over):
    features = {n: SPEC.norm[n][0] for n in observation_feature_names()}  # all at their mean
    base = dict(t_sim_s=0.0, features=features,
                detector=DetectorOutput(p_fault=0.8, rotor=2, severity=0.35, uncertainty=0.01,
                                        alarm=True),
                mission=MissionProgress(waypoint_index=2, n_waypoints=5, distance_to_waypoint_m=7.5,
                                        altitude_m=2.5, elapsed_s=30.0),
                previous_action=Action(0.3, -1.75, True))
    base.update(over)
    return PolicyInput(**base)


def test_layout():
    assert SPEC.obs_version == "2" and len(SPEC.names) == 27
    assert SPEC.names[:13] == tuple(f"feature.{n}" for n in observation_feature_names())
    assert SPEC.names[16:20] == tuple(f"detector.rotor_{k}" for k in range(4))
    assert SPEC.names[-3:] == ("previous_action.speed_scale", "previous_action.altitude_offset_m",
                               "previous_action.land")


def test_values_by_hand():
    v = dict(zip(SPEC.names, flatten_observation(_obs(), SPEC)))
    assert all(v[f"feature.{n}"] == pytest.approx(0.0) for n in observation_feature_names())
    assert (v["detector.p_fault"], v["detector.severity"]) == pytest.approx((0.8, 0.35))
    assert [v[f"detector.rotor_{k}"] for k in range(4)] == [0.0, 0.0, 1.0, 0.0]
    assert v["mission.waypoint_index"] == pytest.approx(2 / 5)
    assert v["mission.distance_to_waypoint_m"] == pytest.approx(0.5)
    assert v["mission.altitude_m"] == pytest.approx(0.5)
    assert v["mission.elapsed_s"] == pytest.approx(0.25)
    assert v["previous_action.altitude_offset_m"] == pytest.approx(-0.5)
    assert v["previous_action.land"] == 1.0


def test_normalisation_uses_the_frozen_stats():
    n = "vel_x"
    mean, std = SPEC.norm[n]
    obs = _obs(features={**_obs().features, n: mean + 2 * std})
    assert dict(zip(SPEC.names, flatten_observation(obs, SPEC)))[f"feature.{n}"] == pytest.approx(2.0)


def test_previous_action_is_required():
    with pytest.raises(ValueError, match="previous_action"):
        flatten_observation(_obs(previous_action=None), SPEC)


def test_isaac_fixture_is_up_to_date():
    """tests/fixtures/isaac_contract_v1.json must be what the current
    PX4-side code produces. If this fails, a contract changed: bump its
    version, regenerate with experiments/write_isaac_fixtures.py, and re-run
    the Isaac-side tests."""
    assert fixture_main(["--check"]) == 0
