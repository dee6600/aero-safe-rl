"""M8b task 4 and 6: the Isaac environment, headless, 16 drones
(aero_isaac.probe smoke, in its own process -- Isaac Sim is started once per
process and never shuts down cleanly headless). About five minutes: the healthy
drones fly the whole 46 s mission at 250 physics steps per second."""
import json
import os
import signal
import subprocess
import sys
from pathlib import Path

import pytest

from aero_isaac.contracts import load_action_spec, load_observation_spec
from aero_isaac.outcome import COMPLETED, CRASH, GROUND_CONTACT, MISSION_SUCCESS

ISAAC_DIR = Path(__file__).resolve().parent.parent

pytestmark = pytest.mark.isaac


@pytest.fixture(scope="module")
def smoke(tmp_path_factory):
    out = tmp_path_factory.mktemp("smoke") / "smoke.json"
    proc = subprocess.Popen([sys.executable, "-m", "aero_isaac.probe", "smoke", "--out", str(out)],
                            cwd=ISAAC_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)
    try:
        log, _ = proc.communicate(timeout=600)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)       # the whole Isaac Sim process group, never a stray kit
        log, _ = proc.communicate()
    assert out.exists(), log[-4000:]
    return json.loads(out.read_text())


def test_observation_shape_and_values(smoke):
    assert smoke["obs_shape"] == [16, len(load_observation_spec().entries)]
    assert smoke["all_finite"]


def test_decision_period_matches_the_action_contract(smoke):
    assert smoke["decision_period_s"] == pytest.approx(load_action_spec().decision_period_s)
    assert smoke["physics_dt_s"] == pytest.approx(0.004)


def test_every_drone_finishes_and_is_reset_on_the_ground(smoke):
    assert len(smoke["first_episode"]) == 16
    assert all(smoke["back_on_ground"].values())


def test_terminations_follow_the_outcome_rule(smoke):
    for e in smoke["first_episode"].values():
        sev = round(e["severity"], 2)
        if sev in (0.0, 0.3):
            assert (e["termination"], e["outcome"]) == (COMPLETED, MISSION_SUCCESS), e
        if sev == 0.7:
            assert (e["termination"], e["outcome"]) == (GROUND_CONTACT, CRASH), e


def test_true_fault_does_not_reach_the_observation(smoke):
    leak = smoke["leak"]
    assert leak["max_change_true_fault"] == 0.0
    assert leak["max_change_detector"] > 0.1      # the check can see a change when there is one
