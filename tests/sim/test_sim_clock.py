"""Regression test for the M2 finding that decided PX4Clock's design
(@pytest.mark.sim -- needs a real worker; see simulation/sim_clock.py's
module docstring for the full story and the numbers from the original
investigation).

Two things are pinned here so a future PX4, Gazebo, or protobuf upgrade
can't silently reintroduce either problem:

  1. px4_msgs timestamps are NOT usable as a sim-time source -- they track
     wall clock almost exactly, regardless of speed factor.
  2. GzSimClock (reading Gazebo's own /world/<w>/stats directly) IS usable --
     it tracks the speed factor closely, including under a factor other
     than 1 (where the two would give very different answers if either
     measurement were wrong).

If (1) ever stops being true, PX4Clock could actually be simplified to use
px4_msgs timestamps directly -- but that must be a deliberate decision after
seeing this test fail, not something a future change does by accident.
"""
import time

import pytest

from simulation.sim_clock import GzSimClock

SPEED_FACTOR = 4
MEASURE_S = 8.0
WARMUP_S = 3.0


@pytest.fixture
def gz_clock_instance0(sim_worker):
    # sim_worker fixture (tests/sim/conftest.py) starts instance 0 at its own
    # default speed; override needed here since this test cares about a
    # specific, known factor. Restart at the factor this test needs.
    import subprocess
    from pathlib import Path
    repo = Path(__file__).resolve().parent.parent.parent
    subprocess.run([str(repo / 'scripts/sim_stop.sh'), '--all'],
                   capture_output=True, timeout=60)
    r = subprocess.run(
        [str(repo / 'scripts/sim_start.sh'), '-i', '0', '-s', str(SPEED_FACTOR)],
        capture_output=True, text=True, timeout=180)
    assert r.returncode == 0, r.stdout

    clock = GzSimClock(world='default', gz_partition='aero_0')
    yield clock
    clock.close()


def test_gz_sim_clock_tracks_speed_factor(gz_clock_instance0):
    clock = gz_clock_instance0

    deadline = time.monotonic() + 10.0
    while clock.now_us() is None:
        assert time.monotonic() < deadline, "no /stats message received"
        time.sleep(0.05)

    time.sleep(WARMUP_S)
    start_us = clock.now_us()
    wall_start = time.monotonic()
    time.sleep(MEASURE_S)
    wall_elapsed = time.monotonic() - wall_start
    end_us = clock.now_us()

    sim_elapsed_s = (end_us - start_us) / 1e6
    ratio = sim_elapsed_s / wall_elapsed

    # Not exact -- this machine's achieved RTF has its own jitter (M1
    # measured stdev growing with speed factor) -- but must be unambiguously
    # close to SPEED_FACTOR and unambiguously far from 1.0 (which is what a
    # wall-clock-tracking source, like px4_msgs timestamps, would give).
    assert ratio == pytest.approx(SPEED_FACTOR, rel=0.25), (
        f"GzSimClock ratio {ratio:.3f} is not close to the requested speed "
        f"factor {SPEED_FACTOR} -- sim_elapsed={sim_elapsed_s:.3f}s, "
        f"wall_elapsed={wall_elapsed:.3f}s"
    )
    assert ratio > 2.0, (
        "ratio is suspiciously close to 1.0 -- this is exactly the wrong "
        "answer px4_msgs timestamps give; GzSimClock may be reading the "
        "wrong source"
    )
