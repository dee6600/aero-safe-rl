"""M2's actual gate (@pytest.mark.sim): the flight logic works identically on
any instance number, including two running CONCURRENTLY. Everything before
this milestone was plumbing; this is the first test that would have caught
either of M2's two known bugs (docs/parallelism.md §2.1-2.2) by actually
flying instance 1, not just instance 0.

Each instance is flown in its OWN OS process (`python -m
aero_bridge.test_flight`), exactly as a user would run it via `ros2 run` --
not as two rclpy Contexts sharing one interpreter, which is not how any real
node in this project runs (CLAUDE.md's whole parallelism design is one
process per worker).
"""
import subprocess
import sys
import time

import pytest

HOVER_SECONDS = 2.0
PER_INSTANCE_TIMEOUT_S = 90

# Known, open reliability gap (2026-08-20) -- see docs/parallelism.md §2.6
# for the full investigation. Roughly 35-65% of concurrent two-worker flights
# hit offboard_control_signal_lost at least once (solo: ~10-20%), even
# though the publish loop was instrumented and NEVER missed a scheduled
# setpoint -- the message loss happens somewhere in the BEST_EFFORT DDS
# transport, not in application code. hold_position_until now re-engages
# offboard on loss, which roughly halves the failure rate but does not
# eliminate it (the loss can repeat periodically through a whole flight).
# Ruled out, with evidence, before finding the real mechanism: PX4's battery
# failsafe (misleading log line -- see §2.6) and GzSimClock's background
# gz-transport thread. Do not re-diagnose from scratch; read §2.6 first.
# The real fix belongs to M4 (WorkerSupervisor: detect, invalidate the
# episode, retry) or further transport-level investigation, not a wider
# timeout here. A failure with a DIFFERENT signature (not
# offboard_control_signal_lost) is a real, different bug.


def _fly(instance: int) -> subprocess.Popen:
    return subprocess.Popen(
        [sys.executable, '-m', 'aero_bridge.test_flight',
         '--instance', str(instance),
         '--hover-alt', '5.0', '--hover-seconds', str(HOVER_SECONDS)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )


def _wait(proc: subprocess.Popen, instance: int) -> tuple:
    try:
        out, _ = proc.communicate(timeout=PER_INSTANCE_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate()
        pytest.fail(f"instance {instance} test_flight did not finish within "
                    f"{PER_INSTANCE_TIMEOUT_S}s:\n{out}")
    return proc.returncode, out


def test_single_instance_flies(sim_worker):
    code, out = _wait(_fly(sim_worker), sim_worker)
    assert code == 0, f"instance {sim_worker} flight failed:\n{out}"
    assert 'landed and disarmed' in out


def test_two_instances_fly_concurrently(sim_workers_0_1):
    """The gate. Launch both flights at (as close as this process can manage
    to) the same moment, then wait for both -- each reaching ITS OWN target
    altitude while the other's world is also live, which is the scenario
    that would surface any cross-instance leakage.
    """
    a, b = sim_workers_0_1
    proc_a = _fly(a)
    proc_b = _fly(b)  # started before waiting on proc_a -- genuinely concurrent

    code_a, out_a = _wait(proc_a, a)
    code_b, out_b = _wait(proc_b, b)

    for instance, code, out in ((a, code_a, out_a), (b, code_b, out_b)):
        assert code == 0, f"instance {instance} flight failed:\n{out}"
        assert 'reached takeoff altitude' in out, f"instance {instance}:\n{out}"
        assert 'hover complete' in out, f"instance {instance}:\n{out}"
        assert 'landed and disarmed' in out, f"instance {instance}:\n{out}"
