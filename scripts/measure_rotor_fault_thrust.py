#!/usr/bin/env python3
"""M6 task 8: the CLAUDE.md §1.6 cross-validation fixture -- severity `s` ->
measured thrust reduction, from the real Gazebo plugin. A later milestone
(M8b, not built yet) will validate its Isaac-side rotor model's own
severity-vs-thrust-reduction curve against this file.

Measurement method and its honest limits
-----------------------------------------
Commands a known, fixed rotor velocity directly onto
`/<model>/command/motor_speed` (PX4 not armed, not even started against
this measurement -- no controller in the loop to confound a single-rotor
fault's effect, which is NOT a symmetric net-thrust loss: an asymmetric
single-rotor fault mostly shows up as attitude torque, and a real PX4
attitude controller would immediately start compensating for it within one
control cycle, which is exactly the confound this bypasses), then reads
back the RotorDegradationSystem-relayed velocity that
gz-sim-multicopter-motor-model-system actually consumes.

Thrust itself is not read from an independent physical sensor here (a
joint force-torque sensor was investigated for this but not completed this
session -- flagged as a documented follow-up, not silently skipped). Instead
`thrust_ratio` is DERIVED: `MulticopterMotorModel`'s thrust law is the
standard `thrust = motorConstant * omega^2` (its `<motorConstant>` SDF
parameter is, by definition across every known Gazebo multicopter plugin
lineage, exactly this proportionality constant -- this is documented
physics, not reimplemented physics, consistent with this project's D2
decision to reuse Gazebo's own motor model rather than re-derive one).
Since velocity_ratio is measured live and exactly
(tests/sim/test_rotor_fault_controller.py::test_graded_severity_scales_relayed_velocity
confirms it to 1e-6), `thrust_ratio = velocity_ratio ** 2` follows from that
law algebraically, not from a second, independent physical measurement.
**If M8b's own cross-validation later disagrees with this fixture by more
than measurement noise, an independent force-torque measurement is the
first thing to add** -- this method's derivation, not its arithmetic, would
be the first thing to re-examine.

Usage (worker must already be running: scripts/sim_start.sh -i 0 -m x500_aero):
    python scripts/measure_rotor_fault_thrust.py --instance 0 \
        --out tests/fixtures/rotor_fault_thrust_curve.json
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
_SYSTEM_SITE_PACKAGES = "/usr/lib/python3/dist-packages"
if _SYSTEM_SITE_PACKAGES not in sys.path:
    sys.path.append(_SYSTEM_SITE_PACKAGES)

import gz.transport13 as gz_transport  # noqa: E402
from gz.msgs10.actuators_pb2 import Actuators  # noqa: E402

from simulation.instance_spec import InstanceSpec  # noqa: E402
from simulation.rotor_fault import RotorFaultController  # noqa: E402

FAULTED_ROTOR = 0
UNFAULTED_MARKER_ROTOR = 1
INJECTED_VELOCITY = 12345.0
SEVERITIES = (0.0, 0.2, 0.5, 0.9)


def measure(instance: int) -> list[dict]:
    spec = InstanceSpec.for_instance(instance, model="x500_aero")
    os.environ['GZ_PARTITION'] = spec.gz_partition
    node = gz_transport.Node()
    relayed: dict = {}

    def _on_relayed(msg: Actuators) -> None:
        if len(msg.velocity) == 4 and msg.velocity[UNFAULTED_MARKER_ROTOR] != INJECTED_VELOCITY:
            return
        relayed['velocity'] = list(msg.velocity)

    topic = f"/{spec.model_name}/command/motor_speed_faulted"
    assert node.subscribe(Actuators, topic, _on_relayed), f"failed to subscribe to {topic}"
    pub = node.advertise(f"/{spec.model_name}/command/motor_speed", Actuators)

    controller = RotorFaultController(spec)
    controller.wait_for_heartbeat(timeout_s=10.0)

    results = []
    try:
        for severity in SEVERITIES:
            controller.latest_rotor_index = -1
            controller.set_rotor_fault(FAULTED_ROTOR, severity)
            deadline = time.monotonic() + 5.0
            while controller.latest_rotor_index != FAULTED_ROTOR and time.monotonic() < deadline:
                time.sleep(0.02)
            if controller.latest_rotor_index != FAULTED_ROTOR:
                raise RuntimeError(f"severity={severity}: fault command never confirmed applied")

            relayed.clear()
            cmd = Actuators()
            cmd.velocity.extend([INJECTED_VELOCITY] * 4)
            deadline = time.monotonic() + 5.0
            while 'velocity' not in relayed and time.monotonic() < deadline:
                pub.publish(cmd)
                time.sleep(0.02)
            if 'velocity' not in relayed:
                raise RuntimeError(f"severity={severity}: no relayed message observed")

            velocity_ratio = relayed['velocity'][FAULTED_ROTOR] / INJECTED_VELOCITY
            thrust_ratio = velocity_ratio ** 2
            results.append({
                "severity": severity,
                "measured_velocity_ratio": velocity_ratio,
                "derived_thrust_ratio": thrust_ratio,
                "expected_thrust_ratio": 1.0 - severity,
            })
            print(f"  severity={severity:.2f}  velocity_ratio={velocity_ratio:.6f}  "
                  f"derived_thrust_ratio={thrust_ratio:.6f}  "
                  f"expected={1.0 - severity:.6f}")
    finally:
        controller.clear_rotor_fault()
        controller.close()

    return results


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--out", default=str(REPO / "tests" / "fixtures" / "rotor_fault_thrust_curve.json"))
    args = ap.parse_args(argv)

    results = measure(args.instance)

    out = {
        "fixture_version": "1",
        "fault_type": "rotor_thrust_degradation",
        "method": (
            "commanded velocity ratio measured live via the RotorDegradationSystem "
            "relay (PX4 not in the loop); thrust_ratio derived from "
            "MulticopterMotorModel's documented thrust = motorConstant * omega^2 "
            "law, not independently sensor-measured -- see this script's module "
            "docstring for the full rationale and limits"
        ),
        "measured_at_utc": datetime.datetime.now(tz=datetime.timezone.utc).isoformat(),
        "curve": results,
    }

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, indent=2, sort_keys=False) + "\n")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
