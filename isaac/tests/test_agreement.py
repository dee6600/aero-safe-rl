"""M8b task 5: the agreement check's summaries and gates, on hand-built
episodes (pure Python -- the flying needs Isaac Sim)."""
import math

import pytest

from aero_isaac.agreement import compare, crash50, summarize


def _ep(sev, outcome, touch=float("nan"), dur=40.0, peak=9.0, motor=0.75):
    return dict(severity=sev, outcome=outcome, touchdown_speed_m_s=touch, duration_s=dur,
                peak_hspeed_m_s=peak, mean_motor_command=motor)


def test_summarize_and_crash50():
    eps = [_ep(0.0, 0, 0.7)] * 4 + [_ep(0.4, 1, 1.5)] * 4 + [_ep(0.45, 2, 3.0)] * 2 + [_ep(0.45, 1, 1.8)] * 2
    s = summarize(eps)
    assert s["by_severity"]["0.45"]["crash"] == 0.5
    assert s["by_severity"]["0.45"]["median_touchdown_speed_m_s"] == pytest.approx(2.4)
    assert crash50(s) == pytest.approx(0.45)
    assert math.isnan(crash50(summarize([_ep(0.0, 0)])))


def test_gates():
    isaac = summarize([_ep(0.0, 0, 0.7, dur=42.0, peak=9.5, motor=0.74)] + [_ep(0.45, 2, 3.4)])
    px4 = summarize([_ep(0.0, 0, 0.7, dur=43.4, peak=9.1, motor=0.75)] + [_ep(0.45, 2, 2.9)])
    rows = {r["measure"]: r["passed"] for r in compare(isaac, px4)}
    assert rows["healthy mission time (s)"] and rows["healthy peak speed (m/s)"]
    assert rows["healthy mean motor command"]
    assert rows["median touchdown speed at s = 0.45 (m/s)"]                 # 0.5 apart
    px4_far = summarize([_ep(0.0, 0, 0.7, dur=60.0)] + [_ep(0.45, 2, 4.0)])
    rows = {r["measure"]: r["passed"] for r in compare(isaac, px4_far)}
    assert not rows["healthy mission time (s)"] and not rows["median touchdown speed at s = 0.45 (m/s)"]


# ---------------------------------------------------------------- M9 reward-ranking gate

def _ranked(returns: dict) -> dict:
    """returns: {severity: {policy: mean return}} -> rank() on 2 drones per cell."""
    from aero_isaac.agreement import rank
    eps = [dict(severity=s, policy=p, outcome=0, discounted_return=r)
           for s, row in returns.items() for p, r in row.items() for _ in range(2)]
    return rank(eps)


def _grid(nominal_low, react_low, nominal_high, react_high):
    row = lambda n, r: {"nominal": n, "react_slow_low": r, "react_land": r - 1}   # noqa: E731
    return {0.2: row(nominal_low, react_low), 0.3: row(nominal_low, react_low),
            0.4: row(nominal_high, react_high), 0.45: row(nominal_high, react_high)}


def test_ranking_gate_passes_when_the_reward_prefers_the_right_thing():
    out = _ranked(_grid(nominal_low=8.0, react_low=5.0, nominal_high=-9.0, react_high=2.0))
    assert out["passed"] and len(out["checks"]) == 4


def test_ranking_gate_fails_if_reacting_wins_where_the_mission_finishes_unaided():
    assert not _ranked(_grid(nominal_low=3.0, react_low=5.0, nominal_high=-9.0, react_high=2.0))["passed"]


def test_ranking_gate_fails_if_carrying_on_wins_where_the_drone_falls():
    assert not _ranked(_grid(nominal_low=8.0, react_low=5.0, nominal_high=3.0, react_high=2.0))["passed"]
