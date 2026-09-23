"""M8b task 2: the streaming outcome tracker, stepped over each recorded
series, gives the PX4 side's classify_outcome answer."""
import math

import pytest
import torch

from aero_isaac.contracts import load_outcome_spec
from aero_isaac.outcome import (COMPLETED, GROUND_CONTACT, OUTCOME_NAMES, RECOVERY_LANDED, TIMEOUT,
                                OutcomeTracker)
from fixture import contract

TERMINATION = {"completed": COMPLETED, "recovery_landed": RECOVERY_LANDED,
               "ground_contact": GROUND_CONTACT}


@pytest.mark.parametrize("case", contract()["outcome"], ids=lambda c: c["termination_reason"])
def test_outcome_matches_px4_side(case):
    tr = OutcomeTracker(load_outcome_spec(), 1, dtype=torch.float64)
    for alt, vz, roll in zip(case["alt"], case["vel_z"], case["roll_deg"]):
        tr.update(torch.tensor([alt], dtype=torch.float64), torch.tensor([vz], dtype=torch.float64),
                  torch.tensor([abs(roll)], dtype=torch.float64))
    term = torch.tensor([TERMINATION.get(case["termination_reason"], TIMEOUT)])
    assert OUTCOME_NAMES[int(tr.outcome(term)[0])] == case["outcome"]
    for got, want in ((tr.touchdown_speed[0], case["touchdown_speed_m_s"]),
                      (tr.max_tilt_deg[0], case["max_tilt_deg"])):
        if want is None:
            assert math.isnan(float(got))
        else:
            assert float(got) == pytest.approx(want, abs=1e-6)


def test_contact_fires_once():
    tr = OutcomeTracker(load_outcome_spec(), 1)
    fired = [bool(tr.update(torch.tensor([a]), torch.tensor([1.0]), torch.tensor([0.0]))[0])
             for a in (0.0, 2.0, 5.0, 0.2, 0.1, 0.0)]
    assert fired == [False, False, False, True, False, False]
