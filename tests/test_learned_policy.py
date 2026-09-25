"""M9 task 4: the trained policy on the PX4 side (rl/policies/learned.py).

tests/fixtures/policy_export_v1.pt is written by the Isaac side's own export
code (isaac/aero_isaac/train.py fixture): a random-weight network of the
trained shape, plus its outputs on the 12 recorded observation vectors.
Reproducing those outputs here is what shows the two sides run the same
network on the same inputs. Loading must refuse a file trained under
different contract files (CLAUDE.md §0.1).
"""
import json
from pathlib import Path

import numpy as np
import pytest
import torch

from ai.detector.model import CheckpointMismatch, file_digest
from ai.detector.runtime import DetectorOutput
from ai.features.feature_extractor import RAW_FRAME_FIELDS
from rl.policies.base_policy import (
    Action, MissionProgress, PolicyInput, flatten_observation, load_action_spec, load_observation_spec)
from rl.policies.learned import CHECKED, LearnedPolicy, contract_fingerprints
from rl.policy_driver import PolicyDriver, RecoveryConfig

FIXTURES = Path(__file__).resolve().parent / "fixtures"
POLICY = FIXTURES / "policy_export_v1.pt"
SPEC = load_action_spec()
OBS_SPEC = load_observation_spec()


def _cases():
    return json.loads((FIXTURES / "isaac_contract_v1.json").read_text())["observation"]


def _input(case) -> PolicyInput:
    return PolicyInput(t_sim_s=10.0, features=case["features"], detector=DetectorOutput(**case["detector"]),
                       mission=MissionProgress(**case["mission"]), previous_action=Action(**case["previous_action"]))


def _variant(tmp_path, **changes) -> Path:
    ck = torch.load(POLICY, weights_only=True)
    for k, v in changes.items():
        if k == "fingerprint":
            ck["fingerprints"] = dict(ck["fingerprints"], **v)
        else:
            ck[k] = v
    out = tmp_path / "variant.pt"
    torch.save(ck, out)
    return out


def test_reproduces_the_isaac_sides_outputs():
    fx = torch.load(POLICY, weights_only=True)["fixture"]
    got = LearnedPolicy(POLICY).vector(fx["observations"].numpy())
    assert np.allclose(got, fx["outputs"].numpy(), atol=1e-5)


def test_act_builds_the_observation_then_decodes_the_output():
    """From the recorded PolicyInputs: the PX4 side's own observation vector,
    through the network, then action_v1's decode -- the whole decision."""
    policy = LearnedPolicy(POLICY)
    fx = torch.load(POLICY, weights_only=True)["fixture"]
    for i, case in enumerate(_cases()):
        obs = _input(case)
        assert np.allclose(flatten_observation(obs, OBS_SPEC), case["vector"], atol=1e-6)
        got, want = policy.act(obs), SPEC.decode(fx["outputs"][i].tolist())
        assert np.allclose(got.to_vector(), want.to_vector(), atol=1e-5) and got.land == want.land


def test_this_checkout_matches_the_fixtures_fingerprints():
    fx = torch.load(POLICY, weights_only=True)
    ours = contract_fingerprints(SPEC, OBS_SPEC)
    assert {k: fx["fingerprints"][k] for k in CHECKED} == ours


@pytest.mark.parametrize("which", CHECKED)
def test_refuses_a_policy_trained_under_other_contract_files(tmp_path, which):
    with pytest.raises(CheckpointMismatch, match=which):
        LearnedPolicy(_variant(tmp_path, fingerprint={which: "0000000000000000"}))


@pytest.mark.parametrize("change", [dict(format="something_else"), dict(obs_dim=26), dict(act_dim=5),
                                    dict(activation="swish")])
def test_refuses_a_file_of_the_wrong_shape_or_kind(tmp_path, change):
    with pytest.raises(CheckpointMismatch):
        LearnedPolicy(_variant(tmp_path, **change))


def test_actions_always_in_range():
    policy = LearnedPolicy(POLICY)
    rng = np.random.default_rng(3)
    for x in rng.normal(scale=4.0, size=(200, 27)).astype(np.float32):
        a = SPEC.decode(policy.vector(x).tolist())
        assert 0.0 <= a.speed_scale <= 1.0 and -3.5 <= a.altitude_offset_m <= 0.0 and isinstance(a.land, bool)


def _constant_output(tmp_path, out) -> Path:
    """A policy file whose network always outputs `out`."""
    ck = torch.load(POLICY, weights_only=True)
    ck["layers"][-1]["weight"] = torch.zeros_like(ck["layers"][-1]["weight"])
    ck["layers"][-1]["bias"] = torch.tensor(out, dtype=torch.float32)
    path = tmp_path / "constant.pt"
    torch.save(ck, path)
    return path


def _row(t):
    row = {f: 0.0 for f in RAW_FRAME_FIELDS}
    row["t_sim_s"] = t
    return row


def test_landing_latches_through_the_driver(tmp_path):
    policy = LearnedPolicy(_constant_output(tmp_path, [0.4, -1.0, 0.9]))
    driver = PolicyDriver(policy, SPEC)
    progress = MissionProgress(1, 5, 10.0, 5.0, 3.0)
    first = driver.step(_row(0.0), progress)
    assert np.allclose(first.to_vector(), [0.4, -1.0, 1.0], atol=1e-6)
    policy._layers[-1] = (policy._layers[-1][0], torch.tensor([1.0, 0.0, 0.0]))   # would fly on
    for i in range(1, 10):
        assert driver.step(_row(i * 0.1), progress) == first


def test_recovery_config_builds_it_and_records_the_file(tmp_path):
    rc = RecoveryConfig(policy="learned", policy_config=str(POLICY))
    driver = rc.build()
    assert isinstance(driver.policy, LearnedPolicy)
    prov = rc.provenance()
    assert prov["policy_name"] == "learned" and prov["policy_config_digest"] == file_digest(POLICY)
    with pytest.raises(ValueError):
        RecoveryConfig(policy="learned").build()
