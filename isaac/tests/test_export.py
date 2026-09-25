"""M9 task 3: exporting a trained policy for the PX4 side.

Pure PyTorch (no simulator): an rsl_rl actor of the trained shape is built
directly, and its export must give exactly the trainer's mean action after
the training settings' action mapping; the fingerprints the PX4 side checks must be
present and match the contract files; and the fixture the PX4 side replays
(tests/fixtures/policy_export_v1.pt) must be current.
"""
import pytest
import torch
from tensordict import TensorDict

from aero_isaac.contracts import load_action_spec, load_train_config
from aero_isaac.train import (CHECKED_FINGERPRINTS, FIXTURE_PATH, POLICY_FORMAT, export_policy, fingerprints,
                              run_exported, runner_cfg)
from fixture import contract

TRAIN = load_train_config()


def _actor(seed=0):
    from rsl_rl.models import MLPModel
    torch.manual_seed(seed)
    cfg = runner_cfg(TRAIN, "test")["actor"]
    cfg.pop("class_name")
    obs = TensorDict({"policy": torch.zeros(4, 27)}, batch_size=[4])
    return MLPModel(obs, {"actor": ["policy"]}, "actor", 3, **cfg)


def test_export_equals_the_trainers_mean_action_after_the_mapping():
    actor = _actor()
    with torch.no_grad():                       # trained-looking, not freshly initialised, weights
        for p in actor.mlp.parameters():
            p.add_(torch.randn_like(p) * 0.3)
    x = torch.randn(64, 27) * 2
    with torch.no_grad():
        mean = actor(TensorDict({"policy": x}, batch_size=[64]))            # deterministic output = the mean
    off = torch.tensor(TRAIN["action_map"]["offset"])
    sc = torch.tensor(TRAIN["action_map"]["scale"])
    exported = export_policy(actor.state_dict(), TRAIN, seed=3, update=299)
    assert torch.allclose(run_exported(exported, x), off + sc * mean, atol=1e-5)


def test_exported_file_loads_without_pickled_code(tmp_path):
    exported = export_policy(_actor().state_dict(), TRAIN, seed=1, update=0)
    torch.save(exported, tmp_path / "policy.pt")
    back = torch.load(tmp_path / "policy.pt", weights_only=True)
    assert back["format"] == POLICY_FORMAT and back["obs_dim"] == 27 and back["act_dim"] == 3
    assert len(back["layers"]) == len(TRAIN["network"]["hidden"]) + 1


def test_fingerprints_are_present_and_match_the_contract_files():
    fp = fingerprints(TRAIN)
    assert set(CHECKED_FINGERPRINTS) <= set(fp)
    assert all(isinstance(v, str) and len(v) == 16 for v in fp.values())
    recorded = contract()["contract_files"]                # written by the PX4 side's code
    assert fp["action_v1"] == recorded["configs/rl/action_v1.yaml"]
    assert fp["observation_v2"] == recorded["configs/rl/observation_v2.yaml"]


def test_wrong_shaped_actor_is_refused():
    state = dict(_actor().state_dict())
    state["mlp.4.weight"] = torch.zeros(2, 128)
    state["mlp.4.bias"] = torch.zeros(2)
    with pytest.raises(ValueError):
        export_policy(state, TRAIN, seed=1, update=0)


def test_untrained_policy_flies_the_nominal_mission_and_never_lands():
    """The action mapping is centred on the nominal flight: a zero network
    output is the nominal action, and land needs an output of 4."""
    spec = load_action_spec()
    off = torch.tensor(TRAIN["action_map"]["offset"])
    sc = torch.tensor(TRAIN["action_map"]["scale"])
    assert spec.decode(off[None]).tolist() == [list(spec.nominal)]
    assert float(spec.decode((off + sc * torch.tensor([0.0, 0.0, 3.99]))[None])[0, 2]) == 0.0
    assert float(spec.decode((off + sc * torch.tensor([0.0, 0.0, 4.0]))[None])[0, 2]) == 1.0


def test_px4_side_fixture_is_current():
    """tests/fixtures/policy_export_v1.pt regenerates identically from this
    code (python -m aero_isaac.train fixture); a stale one fails here."""
    fx = torch.load(FIXTURE_PATH, weights_only=True)
    assert fx["fingerprints"] == fingerprints(TRAIN)
    assert torch.allclose(run_exported(fx, fx["fixture"]["observations"]), fx["fixture"]["outputs"], atol=1e-6)
    want = torch.tensor([c["vector"] for c in contract()["observation"]])
    assert torch.equal(fx["fixture"]["observations"], want)
