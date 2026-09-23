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
