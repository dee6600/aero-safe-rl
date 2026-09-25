"""M9 tasks 1-2: the training reward (configs/rl/reward_v2.yaml, the one
training settings name).

Pure PyTorch, no simulator. Checks that the terminal values rank outcomes
the way the user chose, that touchdown speed only ever makes things worse,
that progress shaping cannot change which policy is best (its discounted sum
over an episode is zero), and that the reward has no way to see the fault.
"""
import ast
from pathlib import Path

import pytest
import torch

from aero_isaac.contracts import load_action_spec, load_mission, load_reward_spec, load_train_config
from aero_isaac.mission import MissionTracker
from aero_isaac.outcome import CRASH, INCOMPLETE, MISSION_SUCCESS, SAFE_LANDING
from aero_isaac.reward import PARTS, Reward, total
from fixture import contract

F64 = torch.float64
TRAIN = load_train_config()
SPEC = TRAIN["reward_spec"]
GAMMA = float(TRAIN["algorithm"]["discount"])
T = lambda *v: torch.tensor(v)                                               # noqa: E731


def _end(outcome: int, touchdown: float = float("nan"), progress: float = 0.5) -> float:
    r = Reward(SPEC, GAMMA, 1, dtype=F64)
    r._p_prev[:] = progress
    parts = r.step(torch.tensor([progress], dtype=F64), torch.tensor([True]), torch.tensor([outcome]),
                   torch.tensor([touchdown], dtype=F64), mission_complete=T(outcome == MISSION_SUCCESS))
    return float(total(parts)[0])


def test_outcomes_rank_as_chosen():
    """success > safe landing > incomplete > crash, and the 70% break-even
    the user chose: 10 p - 10 (1 - p) = 4 at p = 0.7."""
    s, l, c, i = (SPEC.terminal[k] for k in (MISSION_SUCCESS, SAFE_LANDING, CRASH, INCOMPLETE))
    assert s > l > i > c
    assert (l - c) / (s - c) == pytest.approx(0.7)


def test_same_progress_same_touchdown_outcome_order_holds():
    v = {o: _end(o, touchdown=1.0) for o in (MISSION_SUCCESS, SAFE_LANDING, CRASH)}
    v[INCOMPLETE] = _end(INCOMPLETE)          # still in the air when the timer ran out
    assert v[MISSION_SUCCESS] > v[SAFE_LANDING] > v[CRASH]
    assert v[INCOMPLETE] > v[CRASH]


@pytest.mark.parametrize("outcome", [MISSION_SUCCESS, SAFE_LANDING, CRASH])
def test_faster_touchdown_is_strictly_worse(outcome):
    speeds = [0.3, 0.7, 1.9, 2.5, 6.0]
    vals = [_end(outcome, touchdown=v) for v in speeds]
    assert all(a > b for a, b in zip(vals, vals[1:]))


def test_no_touchdown_no_touchdown_term():
    r = Reward(SPEC, GAMMA, 1, dtype=F64)
    parts = r.step(torch.tensor([0.4], dtype=F64), torch.tensor([True]), torch.tensor([INCOMPLETE]),
                   torch.tensor([float("nan")], dtype=F64), mission_complete=T(False))
    assert float(parts["touchdown_speed"][0]) == 0.0 and torch.isfinite(total(parts)).all()


def test_nothing_terminal_paid_before_the_end():
    r = Reward(SPEC, GAMMA, 1, dtype=F64)
    parts = r.step(torch.tensor([0.3], dtype=F64), torch.tensor([False]), torch.tensor([CRASH]),
                   torch.tensor([5.0], dtype=F64), mission_complete=T(False))
    assert all(float(parts[k][0]) == 0.0 for k in PARTS if k != "progress")


def _nominal_progress() -> list[float]:
    """Share of the mission flown along the PX4 side's recorded nominal
    mission (tests/fixtures/isaac_contract_v1.json), one value per 0.1 s."""
    traj = contract()["tracker"][0]
    assert traj["name"] == "nominal_full_mission" and traj["done"][-1]
    tr = MissionTracker(load_mission(), load_action_spec(), 1, dtype=F64)
    tr.reset(torch.arange(1), torch.zeros(1, 2, dtype=F64))
    out = []
    for t, p, a in zip(traj["t"], traj["position"], traj["action"]):
        pos = torch.tensor([p], dtype=F64)
        tr.update(torch.tensor([t], dtype=F64), pos, torch.tensor([a], dtype=F64))
        out.append(float(tr.path_progress(pos)[0]))
    return out


def test_mission_progress_runs_from_zero_to_one_along_the_nominal_mission():
    p = _nominal_progress()
    assert p[0] == 0.0 and p[-1] == 1.0
    # nearly monotone: only the drone's small overshoots at corners go backwards
    assert sum(max(0.0, a - b) for a, b in zip(p, p[1:])) < 0.05


def _progress_parts(progress: list[float], gamma: float) -> list[float]:
    r = Reward(SPEC, gamma, 1, dtype=F64)
    out = []
    for i, p in enumerate(progress):
        last = i == len(progress) - 1
        parts = r.step(torch.tensor([p], dtype=F64), torch.tensor([last]),
                       torch.tensor([MISSION_SUCCESS]), torch.tensor([0.5], dtype=F64), mission_complete=T(p >= 1.0))
        out.append(float(parts["progress"][0]))
    return out


def test_progress_pays_its_weight_while_flying_then_hands_it_back():
    """Undiscounted: +2 over the flight, -P at the end, so progress cannot be
    farmed. Discounted with the training discount: exactly zero."""
    p = _nominal_progress()
    parts = _progress_parts(p, 1.0)
    assert sum(parts[:-1]) == pytest.approx(SPEC.progress_weight * p[-2], abs=1e-9)
    assert sum(parts[:-1]) == pytest.approx(SPEC.progress_weight, abs=1e-6)
    assert sum(parts) == pytest.approx(0.0, abs=1e-9)
    disc = _progress_parts(p, GAMMA)
    assert sum(GAMMA ** i * v for i, v in enumerate(disc)) == pytest.approx(0.0, abs=1e-9)


def test_flying_back_and_forth_earns_nothing_extra():
    """A wiggling path earns exactly what flying straight to the same point
    earns, discounted or not: the shaping depends only on where the path
    ends (it telescopes)."""
    wiggle = [0.0, 0.2, 0.1, 0.3, 0.1, 0.3, 0.1, 0.1]
    direct = [0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    assert sum(_progress_parts(wiggle, 1.0)[:-1]) == pytest.approx(sum(_progress_parts(direct, 1.0)[:-1]))
    assert sum(_progress_parts(wiggle, 1.0)[:-1]) == pytest.approx(SPEC.progress_weight * 0.1)
    disc = lambda xs: sum(GAMMA ** i * v for i, v in enumerate(_progress_parts(xs, GAMMA)[:-1]))  # noqa: E731
    assert disc(wiggle) == pytest.approx(disc(direct), abs=1e-12)


def test_reset_starts_the_episode_from_the_ground():
    r = Reward(SPEC, GAMMA, 2, dtype=F64)
    r._p_prev[:] = 0.8
    r.reset(torch.tensor([1]))
    assert r._p_prev.tolist() == [0.8, 0.0]


def test_reward_code_never_names_a_fault():
    """No argument, variable or attribute in reward.py refers to the fault
    or its severity: the reward judges the flight's result only."""
    src = Path(__file__).resolve().parent.parent / "aero_isaac" / "reward.py"
    names = set()
    for node in ast.walk(ast.parse(src.read_text())):
        if isinstance(node, ast.Name):
            names.add(node.id)
        elif isinstance(node, ast.Attribute):
            names.add(node.attr)
        elif isinstance(node, ast.arg):
            names.add(node.arg)
    bad = sorted(n for n in names if any(w in n.lower() for w in ("fault", "sever", "rotor")))
    assert not bad, f"reward.py refers to {bad}"


def test_training_discount_is_a_valid_shaping_discount():
    assert 0.9 < GAMMA < 1.0
    with pytest.raises(ValueError):
        Reward(SPEC, 1.5, 1)


# ---------------------------------------------------------------- reward_v2: success paid at mission completion

def _episode(complete_at: int | None, end_at: int, outcome: int, spec=SPEC) -> list[dict]:
    """Per-step parts of one episode: mission completed on step complete_at
    (None = never), episode ends on step end_at with `outcome`."""
    r = Reward(spec, GAMMA, 1, dtype=F64)
    out = []
    for i in range(end_at + 1):
        done = complete_at is not None and i >= complete_at
        parts = r.step(torch.tensor([1.0 if done else 0.5], dtype=F64), T(i == end_at), T(outcome),
                       torch.tensor([0.6 if i == end_at else float("nan")], dtype=F64), mission_complete=T(done))
        out.append({k: float(v[0]) for k, v in parts.items()})
    return out


def test_success_is_paid_when_the_mission_is_completed_not_after_the_landing():
    steps = _episode(complete_at=200, end_at=245, outcome=MISSION_SUCCESS)
    paid = [i for i, p in enumerate(steps) if p["mission_success"] != 0.0]
    assert paid == [200] and steps[200]["mission_success"] == SPEC.terminal[MISSION_SUCCESS]


def test_a_crash_on_the_final_landing_takes_the_success_back():
    steps = _episode(complete_at=200, end_at=230, outcome=CRASH)
    assert sum(p["mission_success"] for p in steps) == 0.0
    assert steps[-1]["crash"] == SPEC.terminal[CRASH]


def test_how_long_the_final_landing_takes_no_longer_changes_the_success_value():
    """The flaw reward_v1 had: a shorter final landing (flying low at the end)
    made the same success worth more under the discount."""
    def disc_success(end_at):
        return sum(GAMMA ** i * p["mission_success"] for i, p in enumerate(
            _episode(complete_at=200, end_at=end_at, outcome=MISSION_SUCCESS)))
    assert disc_success(220) == pytest.approx(disc_success(245))
    v1 = load_reward_spec(Path(__file__).resolve().parents[2] / "configs" / "rl" / "reward_v1.yaml")
    assert v1.success_paid_at == "episode_end"
    v1_value = lambda end_at: sum(GAMMA ** i * p["mission_success"]  # noqa: E731
                                  for i, p in enumerate(_episode(200, end_at, MISSION_SUCCESS, spec=v1)))
    assert v1_value(220) > v1_value(245)                      # v1's flaw, kept visible


def test_a_mission_never_completed_is_never_paid_success():
    for outcome in (SAFE_LANDING, CRASH, INCOMPLETE):
        assert all(p["mission_success"] == 0.0 for p in _episode(None, 100, outcome))


def test_success_paying_reward_needs_the_completion_signal():
    r = Reward(SPEC, GAMMA, 1)
    with pytest.raises(ValueError):
        r.step(torch.tensor([0.5]), T(False), T(0), torch.tensor([float("nan")]))
