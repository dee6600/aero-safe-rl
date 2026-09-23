"""M8b task 2: the Isaac side reads the same contract files, and decodes
actions the same way, as the PX4 side did when it recorded the fixture."""
import pytest
import torch

from aero_isaac.contracts import REPO, file_digest_of_yaml, load_action_spec
from fixture import contract


@pytest.mark.parametrize("path", sorted(contract()["contract_files"]))
def test_contract_file_unchanged_since_fixture(path):
    assert file_digest_of_yaml(REPO / path) == contract()["contract_files"][path], (
        f"{path} changed since the fixture was recorded: regenerate it on the PX4 side "
        "(experiments/write_isaac_fixtures.py) and bump the contract's version")


def test_action_decoding_matches():
    spec = load_action_spec()
    cases = contract()["action"]
    raw = torch.tensor([c["vector"] for c in cases], dtype=torch.float64)
    want = torch.tensor([c["decoded"] for c in cases], dtype=torch.float64)
    assert torch.allclose(spec.decode(raw), want, atol=1e-9)


def test_decision_period_is_the_px4_sides():
    assert load_action_spec().decision_period_s == pytest.approx(0.2)
