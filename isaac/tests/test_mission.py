"""M8b task 2: the vectorised mission tracker reproduces the PX4 side's
MissionTracker on its recorded trajectories -- all trajectories at once, as
separate drones."""
import torch

from aero_isaac.contracts import load_action_spec, load_mission
from aero_isaac.mission import MissionTracker
from fixture import contract

F64 = torch.float64


def test_tracker_matches_px4_side_on_recorded_trajectories():
    trajs = contract()["tracker"]
    n = len(trajs)
    steps = max(len(t["t"]) for t in trajs)
    tr = MissionTracker(load_mission(), load_action_spec(), n, dtype=F64)
    tr.reset(torch.arange(n), torch.zeros(n, 2, dtype=F64))
    for i in range(steps):
        live = [k for k, t in enumerate(trajs) if i < len(t["t"])]
        if len(live) < n:
            break  # compare only while every trajectory still has data
        t = torch.tensor([trajs[k]["t"][i] for k in range(n)], dtype=F64)
        pos = torch.tensor([trajs[k]["position"][i] for k in range(n)], dtype=F64)
        act = torch.tensor([trajs[k]["action"][i] for k in range(n)], dtype=F64)
        prog = tr.progress(t, pos)
        want_prog = torch.tensor([trajs[k]["progress"][i] for k in range(n)], dtype=F64)
        got_prog = torch.stack([prog[f] for f in ("waypoint_index", "n_waypoints",
                                                  "distance_to_waypoint_m", "altitude_m", "elapsed_s")], 1)
        assert torch.allclose(got_prog, want_prog, atol=1e-6), f"progress differs at step {i}"
        tr.update(t, pos, act)
        for key, got in (("setpoint", tr.setpoint), ("target", tr.target),
                         ("altitude_offset_m", tr.offset)):
            want = torch.tensor([trajs[k][key][i] for k in range(n)], dtype=F64)
            assert torch.allclose(got, want, atol=1e-6), f"{key} differs at step {i}"
        assert tr.waypoints_reached.tolist() == [trajs[k]["waypoints_reached"][i] for k in range(n)], i
        assert tr.done.tolist() == [trajs[k]["done"][i] for k in range(n)], i


def test_full_nominal_mission_completes():
    traj = contract()["tracker"][0]
    assert traj["name"] == "nominal_full_mission" and traj["done"][-1]
    tr = MissionTracker(load_mission(), load_action_spec(), 1, dtype=F64)
    tr.reset(torch.arange(1), torch.zeros(1, 2, dtype=F64))
    for t, p, a in zip(traj["t"], traj["position"], traj["action"]):
        tr.update(torch.tensor([t], dtype=F64), torch.tensor([p], dtype=F64), torch.tensor([a], dtype=F64))
    assert bool(tr.done[0]) and int(tr.waypoints_reached[0]) == 5


def test_reset_restarts_one_drone_only():
    tr = MissionTracker(load_mission(), load_action_spec(), 2, dtype=F64)
    tr.reset(torch.arange(2), torch.zeros(2, 2, dtype=F64))
    tr.leg[:] = 3
    tr.reset(torch.tensor([1]), torch.tensor([[1.0, 2.0]], dtype=F64))
    assert tr.leg.tolist() == [3, 0] and tr.carrot[1].tolist() == [1.0, 2.0]


import pytest  # noqa: E402


@pytest.mark.parametrize("traj", contract()["tracker"], ids=lambda t: t["name"])
def test_each_trajectory_over_its_full_length(traj):
    tr = MissionTracker(load_mission(), load_action_spec(), 1, dtype=F64)
    tr.reset(torch.arange(1), torch.zeros(1, 2, dtype=F64))
    for i, (t, p, a) in enumerate(zip(traj["t"], traj["position"], traj["action"])):
        tr.update(torch.tensor([t], dtype=F64), torch.tensor([p], dtype=F64), torch.tensor([a], dtype=F64))
        assert torch.allclose(tr.setpoint[0], torch.tensor(traj["setpoint"][i], dtype=F64), atol=1e-6), i
        assert int(tr.waypoints_reached[0]) == traj["waypoints_reached"][i], i
        assert bool(tr.done[0]) == traj["done"][i], i
