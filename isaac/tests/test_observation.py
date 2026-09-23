"""M8b task 2: the Isaac-side observation equals the PX4 side's
flatten_observation on the recorded cases, and nothing outside the four
observable blocks can be passed in."""
import pytest
import torch

from aero_isaac.contracts import load_observation_spec
from aero_isaac.observation import ObservationBuilder
from fixture import contract


def _blocks(cases):
    def col(get):
        return torch.tensor([float(get(c)) for c in cases], dtype=torch.float64)
    names = cases[0]["features"].keys()
    return {
        "feature": {n: col(lambda c, n=n: c["features"][n]) for n in names},
        "detector": {k: col(lambda c, k=k: c["detector"][k]) for k in cases[0]["detector"]},
        "mission": {k: col(lambda c, k=k: c["mission"][k]) for k in cases[0]["mission"]},
        "previous_action": {k: col(lambda c, k=k: c["previous_action"][k])
                            for k in cases[0]["previous_action"]},
    }


def test_matches_px4_side():
    cases = contract()["observation"]
    got = ObservationBuilder(load_observation_spec()).assemble(_blocks(cases))
    want = torch.tensor([c["vector"] for c in cases], dtype=torch.float32)
    assert got.shape == (len(cases), len(contract()["observation_names"]))
    assert torch.allclose(got, want, atol=1e-5)


def test_ground_truth_block_is_rejected():
    blocks = _blocks(contract()["observation"])
    blocks["fault"] = {"severity": torch.zeros(len(contract()["observation"]))}
    with pytest.raises(KeyError, match="not an observation block"):
        ObservationBuilder(load_observation_spec()).assemble(blocks)
