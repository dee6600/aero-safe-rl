"""M9 task 4: experiments.metrics.wilson_interval, against published values
(Newcombe 1998, Table I: 81/263 -> 0.2553-0.3662) and its edge cases."""
import math

import pytest

from experiments.metrics import wilson_interval


def test_matches_a_published_value():
    lo, hi = wilson_interval(81, 263)
    assert (lo, hi) == pytest.approx((0.2553, 0.3662), abs=5e-4)


def test_all_or_nothing_cells_are_not_zero_width():
    lo, hi = wilson_interval(0, 8)
    assert lo == 0.0 and hi == pytest.approx(0.3244, abs=5e-4)
    lo, hi = wilson_interval(8, 8)
    assert hi == 1.0 and lo == pytest.approx(0.6756, abs=5e-4)


def test_symmetric_and_narrows_with_more_flights():
    lo, hi = wilson_interval(4, 8)
    assert lo == pytest.approx(1 - hi)
    lo2, hi2 = wilson_interval(40, 80)
    assert hi2 - lo2 < hi - lo


def test_empty_and_impossible():
    assert all(math.isnan(v) for v in wilson_interval(0, 0))
    with pytest.raises(ValueError):
        wilson_interval(9, 8)
