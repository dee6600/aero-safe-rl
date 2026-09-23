"""M8 task 1: rl/mission_tracker.py -- the PX4-side meaning of an action."""
import math

import pytest

from rl.mission_tracker import MissionTracker
from rl.policies.base_policy import Action, load_action_spec

SPEC = load_action_spec()
NOMINAL = SPEC.nominal()
DT = 0.1

# A 3-waypoint mission along x, simple enough to hand-check.
MISSION = dict(waypoints=[[0.0, 0.0], [15.0, 0.0], [15.0, 15.0]], altitude_m=5.0,
               acceptance_radius_m=0.75, hold_time_s=2.0, final_hover_s=3.0)


def _ideal_flight(tracker, action=NOMINAL, t_end=200.0):
    """A vehicle that sits exactly on the setpoint. Returns sim time at done."""
    t = 0.0
    while not tracker.done and t < t_end:
        tracker.update(t, tracker.setpoint, action)
        t = round(t + DT, 10)
    return t


def test_takeoff_is_not_rate_limited():
    tr = MissionTracker(MISSION, SPEC)
    assert tr.setpoint == (0.0, 0.0, -5.0)


def test_carrot_speed_is_scale_times_vmax():
    tr = MissionTracker(MISSION, SPEC)
    tr._leg = 1  # flying to (15, 0)
    slow = Action(0.25, 0.0, False)
    tr.update(0.0, (0.0, 0.0, -5.0), slow)
    tr.update(1.0, (0.0, 0.0, -5.0), slow)
    assert tr.setpoint[0] == pytest.approx(0.25 * SPEC.v_max_xy_m_s * 1.0)


def test_nominal_carrot_crosses_a_leg_in_about_1_25_s():
    tr = MissionTracker(MISSION, SPEC)
    tr._leg = 1
    t = 0.0
    while tr.setpoint[0] < 15.0:
        tr.update(t, (0.0, 0.0, -5.0), NOMINAL)
        t += DT
    assert t == pytest.approx(15.0 / 12.0 + DT, abs=DT + 1e-9)


def test_speed_zero_holds_and_never_counts_the_waypoint():
    tr = MissionTracker(MISSION, SPEC)
    tr._leg = 1
    hold = Action(0.0, 0.0, False)
    for i in range(100):
        tr.update(i * DT, tr.setpoint, hold)
    assert tr.setpoint[:2] == (0.0, 0.0)
    assert tr.waypoints_reached == 1


def test_waypoint_needs_the_full_hold_and_leaving_resets_it():
    tr = MissionTracker(MISSION, SPEC)
    at_wp0 = (0.0, 0.0, -5.0)
    tr.update(0.0, at_wp0, NOMINAL)
    tr.update(1.9, at_wp0, NOMINAL)
    assert tr.waypoints_reached == 0
    tr.update(2.0, (2.0, 0.0, -5.0), NOMINAL)   # drifts out of the radius
    tr.update(3.9, at_wp0, NOMINAL)             # back in: hold restarts here
    tr.update(5.8, at_wp0, NOMINAL)
    assert tr.waypoints_reached == 0
    tr.update(5.9, at_wp0, NOMINAL)
    assert tr.waypoints_reached == 1


def test_hold_counts_in_3d():
    tr = MissionTracker(MISSION, SPEC)
    too_low = (0.0, 0.0, -4.0)  # 1 m below the target, horizontally on it
    for i in range(40):
        tr.update(i * DT, too_low, NOMINAL)
    assert tr.waypoints_reached == 0


def test_full_mission_includes_final_hover():
    tr = MissionTracker(MISSION, SPEC)
    t_done = _ideal_flight(tr)
    assert tr.done and tr.waypoints_reached == 3
    # 3 holds of 2 s + final hover 3 s + two 15 m legs at 12 m/s, each
    # quantised to whole ticks.
    travel = 2 * math.ceil(15.0 / (12.0 * DT)) * DT
    assert t_done == pytest.approx(3 * 2.0 + 3.0 + travel, abs=4 * DT)


def test_slower_mission_takes_longer():
    fast = _ideal_flight(MissionTracker(MISSION, SPEC))
    slow = _ideal_flight(MissionTracker(MISSION, SPEC), Action(0.25, 0.0, False))
    assert slow > fast + 2 * 15.0 / 12.0


def test_altitude_offset_is_rate_limited():
    tr = MissionTracker(MISSION, SPEC)
    lower = Action(1.0, -3.0, False)
    tr.update(0.0, (0.0, 0.0, -5.0), lower)
    tr.update(1.0, (0.0, 0.0, -5.0), lower)
    assert tr.altitude_offset_m == pytest.approx(-SPEC.v_z_m_s * 1.0)
    assert tr.setpoint[2] == pytest.approx(-(5.0 - 1.5))
    assert tr.target[2] == tr.setpoint[2]
    tr.update(10.0, (0.0, 0.0, -5.0), lower)
    assert tr.altitude_offset_m == pytest.approx(-3.0)
    tr.update(10.5, (0.0, 0.0, -5.0), NOMINAL)  # climbs back at the same rate
    assert tr.altitude_offset_m == pytest.approx(-3.0 + 0.75)


def test_mission_completes_at_lower_altitude():
    tr = MissionTracker(MISSION, SPEC)
    _ideal_flight(tr, Action(1.0, -3.0, False))
    assert tr.done


def test_time_going_backwards_does_not_move_the_carrot():
    tr = MissionTracker(MISSION, SPEC)
    tr._leg = 1
    tr.update(5.0, (0.0, 0.0, -5.0), NOMINAL)
    tr.update(4.0, (0.0, 0.0, -5.0), NOMINAL)
    assert tr.setpoint[0] == 0.0


def test_progress():
    tr = MissionTracker(MISSION, SPEC)
    tr.update(10.0, (0.0, 0.0, -5.0), NOMINAL)
    p = tr.progress(12.0, (3.0, 4.0, -4.5))
    assert (p.waypoint_index, p.n_waypoints) == (0, 3)
    assert p.distance_to_waypoint_m == pytest.approx(5.0)
    assert p.altitude_m == pytest.approx(4.5)
    assert p.elapsed_s == pytest.approx(2.0)
    _ideal_flight(tr)
    assert tr.progress(99.0, (15.0, 15.0, -5.0)).waypoint_index == 3


def test_empty_mission_rejected():
    with pytest.raises(ValueError):
        MissionTracker(dict(MISSION, waypoints=[]), SPEC)
