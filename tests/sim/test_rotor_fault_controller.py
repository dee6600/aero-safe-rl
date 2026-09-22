"""M6 task 5 (@pytest.mark.sim): simulation/rotor_fault.py's
RotorFaultController against a real worker -- the actual "confirm, don't
assume" loop (set_rotor_fault -> read latest_applied/latest_severity back
from the plugin's own status echo), and the negative case that proves
wait_for_heartbeat() is actually checking something (a plain x500 worker,
with no RotorDegradationSystem plugin, must raise within its timeout, not
hang).

Also M6 task 7: the graded-severity relay math, isolated from flight
physics (the physics-level claim -- "does this actually cost the vehicle
thrust" -- is task 8's job, the cross-validation fixture). Commands a known
velocity directly onto the real command topic (bypassing PX4/arming
entirely, the same technique task 8's measurement script uses), and checks
the relayed topic's velocity against it -- exactly what the stock
MulticopterMotorModel plugins actually consume, so this is checking the
real input to real physics, not a step removed from it.
"""
import math
import os
import sys
import time

import pytest

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
_SYSTEM_SITE_PACKAGES = "/usr/lib/python3/dist-packages"
if _SYSTEM_SITE_PACKAGES not in sys.path:
    sys.path.append(_SYSTEM_SITE_PACKAGES)

import gz.transport13 as gz_transport  # noqa: E402
from gz.msgs10.actuators_pb2 import Actuators  # noqa: E402

from simulation.instance_spec import InstanceSpec
from simulation.rotor_fault import RotorFaultController, RotorFaultControllerError


def test_wait_for_heartbeat_succeeds_and_set_rotor_fault_is_confirmed(sim_worker_x500_aero):
    spec = InstanceSpec.for_instance(0, model="x500_aero")
    controller = RotorFaultController(spec)
    try:
        controller.wait_for_heartbeat(timeout_s=10.0)
        assert controller.latest_applied is False
        assert controller.latest_rotor_index == -1

        controller.set_rotor_fault(2, 0.5)
        deadline = time.monotonic() + 5.0
        while controller.latest_rotor_index != 2 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert controller.latest_rotor_index == 2
        assert controller.latest_applied is True
        assert controller.latest_severity == pytest.approx(0.5)

        controller.clear_rotor_fault()
        deadline = time.monotonic() + 5.0
        while controller.latest_applied and time.monotonic() < deadline:
            time.sleep(0.05)
        assert controller.latest_applied is False
        assert controller.latest_rotor_index == -1
    finally:
        controller.close()


_INJECTED_VELOCITY = 12345.0  # distinctive, so it's unmistakable among PX4's own real traffic


def test_graded_severity_scales_relayed_velocity(sim_worker_x500_aero):
    os.environ['GZ_PARTITION'] = 'aero_0'
    node = gz_transport.Node()
    relayed = {}

    def _on_relayed(msg: Actuators) -> None:
        if len(msg.velocity) == 4 and msg.velocity[1] != _INJECTED_VELOCITY:
            # A distinctive marker on a rotor we never fault in this test
            # (rotor 1) lets us tell our own injected messages apart from
            # PX4's simultaneous real traffic without needing to stop PX4.
            return
        relayed['velocity'] = list(msg.velocity)

    assert node.subscribe(Actuators, "/x500_aero_0/command/motor_speed_faulted", _on_relayed)
    pub = node.advertise("/x500_aero_0/command/motor_speed", Actuators)

    spec = InstanceSpec.for_instance(0, model="x500_aero")
    controller = RotorFaultController(spec)
    try:
        controller.wait_for_heartbeat(timeout_s=10.0)

        for severity in (0.0, 0.2, 0.5, 0.9):
            # Wait on rotor_index, not severity: severity's own default
            # (before any echo ever arrives) is 0.0, which would make the
            # first severity=0.0 iteration's wait a no-op against a stale
            # (pre-command) echo rather than a real confirmation.
            controller.latest_rotor_index = -1
            controller.set_rotor_fault(0, severity)
            deadline = time.monotonic() + 5.0
            while controller.latest_rotor_index != 0 and time.monotonic() < deadline:
                time.sleep(0.02)
            assert controller.latest_applied and controller.latest_rotor_index == 0
            assert controller.latest_severity == pytest.approx(severity)

            relayed.clear()
            cmd = Actuators()
            cmd.velocity.extend([_INJECTED_VELOCITY, _INJECTED_VELOCITY,
                                  _INJECTED_VELOCITY, _INJECTED_VELOCITY])
            deadline = time.monotonic() + 5.0
            while 'velocity' not in relayed and time.monotonic() < deadline:
                pub.publish(cmd)
                time.sleep(0.02)
            assert 'velocity' in relayed, f"no relayed message observed for severity={severity}"

            expected_ratio = math.sqrt(1.0 - severity)
            actual_ratio = relayed['velocity'][0] / _INJECTED_VELOCITY
            assert actual_ratio == pytest.approx(expected_ratio, abs=1e-6), (
                f"severity={severity}: expected rotor 0 velocity ratio "
                f"{expected_ratio}, got {actual_ratio}")
            # Every other rotor must be untouched by the fault.
            for i in (1, 2, 3):
                assert relayed['velocity'][i] == pytest.approx(_INJECTED_VELOCITY), (
                    f"severity={severity}: rotor {i} was not supposed to be faulted")
    finally:
        controller.clear_rotor_fault()
        controller.close()


def test_wait_for_heartbeat_raises_on_a_plain_x500_worker(sim_worker):
    """The negative case: a worker with no RotorDegradationSystem plugin at
    all must make wait_for_heartbeat() raise within its own timeout, not
    hang forever -- proves the positive test above is checking something
    plugin-specific, not just "some message showed up on some topic"."""
    spec = InstanceSpec.for_instance(0)  # default model: x500
    controller = RotorFaultController(spec)
    try:
        with pytest.raises(RotorFaultControllerError):
            controller.wait_for_heartbeat(timeout_s=3.0)
    finally:
        controller.close()
