"""M8b task 4 and 6: the Isaac environment, headless, 16 drones
(aero_isaac.probe smoke, in its own process -- Isaac Sim is started once per
process and never shuts down cleanly headless). About five minutes: the healthy
drones fly the whole 46 s mission at 250 physics steps per second.

M9 task 2: the same environment under the training settings (train_v2.yaml)
(aero_isaac.probe smoke_train, 64 drones, random network-scale actions):
randomisation stays in range, rewards stay finite and add up to each
episode's recorded return, and the true fault still cannot reach the
observation with randomisation on. Another ~5 minutes."""
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


def _probe(mode: str, out: Path) -> dict:
    proc = subprocess.Popen([sys.executable, "-m", "aero_isaac.probe", mode, "--out", str(out)],
                            cwd=ISAAC_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)
    try:
        log, _ = proc.communicate(timeout=600)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)       # the whole Isaac Sim process group, never a stray kit
        log, _ = proc.communicate()
    assert out.exists(), log[-4000:]
    return json.loads(out.read_text())


@pytest.fixture(scope="module")
def smoke(tmp_path_factory):
    return _probe("smoke", tmp_path_factory.mktemp("smoke") / "smoke.json")


@pytest.fixture(scope="module")
def smoke_train(tmp_path_factory):
    return _probe("smoke_train", tmp_path_factory.mktemp("smoke_train") / "smoke_train.json")


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


# ---------------------------------------------------------------- M9 training settings

def _records(result) -> list[dict]:
    return [dict(zip(result["fields"], r)) for r in result["records"]]


def test_training_settings_stay_finite(smoke_train):
    assert smoke_train["finite"] == {"obs": True, "reward": True}
    assert len(smoke_train["records"]) >= 32, "too few finished episodes to check anything"


def test_randomisation_stays_in_range(smoke_train):
    r = smoke_train["randomization"]
    recs = _records(smoke_train)
    examples = set(range(len(smoke_train["example_flights"]["severities"])))
    free = [e for e in recs if int(e["env"]) not in examples]

    def within(key, rng):
        vals = [e[key] for e in free]
        assert min(vals) >= rng[0] - 1e-6 and max(vals) <= rng[1] + 1e-6, (key, min(vals), max(vals), rng)
        assert max(vals) - min(vals) > 0.2 * (rng[1] - rng[0]), f"{key} barely varies"

    within("mass_scale", r["mass_scale"])
    within("wind_n", r["wind_force_n"])
    within("noise_scale", r["sensor_noise_scale"])
    for k, rng in r["detector"].items():
        within(f"det_{k}", rng)
    faulted = [e for e in free if e["rotor"] >= 0]
    assert all(r["severity_range"][0] <= e["severity"] <= r["severity_range"][1] for e in faulted)
    assert 0.5 <= len(faulted) / len(free) <= 1.0          # 80% faulted, with room for a small sample
    if r.get("severity_focus"):                             # train_v3: half the faults drawn from the focus range
        lo, hi = r["severity_focus"]["range"]
        share = sum(lo <= e["severity"] <= hi for e in faulted) / len(faulted)
        assert 0.4 <= share <= 0.85, share                  # expected ~0.61 (0.5 + 0.5 x 0.2/0.9)


def test_example_drones_fly_their_fixed_faults(smoke_train):
    ex = smoke_train["example_flights"]
    for e in _records(smoke_train):
        i = int(e["env"])
        if i < len(ex["severities"]):
            assert e["severity"] == pytest.approx(ex["severities"][i])
            assert e["ramp_s"] == 0.0
            if ex["severities"][i] > 0:
                assert e["onset_s"] == pytest.approx(ex["onset_s"])
    assert smoke_train["example_traces_finished"] >= 1


def test_rewards_paid_add_up_to_the_recorded_returns(smoke_train):
    recs = _records(smoke_train)
    assert sum(e["return"] for e in recs) == pytest.approx(smoke_train["reward_paid_finished"], rel=1e-4, abs=1e-3)
    parts = [k for k in smoke_train["fields"] if k.startswith("r_")]
    for e in recs:
        assert sum(e[k] for k in parts) == pytest.approx(e["return"], abs=1e-3)


def test_terminal_reward_matches_the_verdict(smoke_train):
    names = {0: "r_mission_success", 1: "r_safe_landing", 2: "r_crash", 3: "r_incomplete"}
    for e in _records(smoke_train):
        paid = {k: e[k] for k in names.values()}
        want = names[int(e["outcome"])]
        assert all(v == 0.0 for k, v in paid.items() if k != want), e


def test_true_fault_does_not_reach_the_observation_under_randomisation(smoke_train):
    leak = smoke_train["leak"]
    assert leak["max_change_true_fault"] == 0.0
    assert leak["max_change_detector"] > 0.1
