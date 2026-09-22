"""M6 task 5: the Python-side control client for RotorDegradationSystem
(simulation/gz_plugins/src/RotorDegradationSystem.cc). Follows
simulation/sim_clock.py's / aero_bridge/reset.py's established pattern:
native gz.transport13 + gz.msgs10 Python bindings, not the `gz` CLI.

"Confirm, don't assume": a gz-transport publish is fire-and-forget, so
set_rotor_fault() returning tells a caller only that a command was queued,
never that it was applied. The plugin echoes the actually-applied
(rotor, severity) back on a status topic on every change plus a periodic
heartbeat; this class tracks the latest echo on plain attributes
(latest_rotor_index / latest_severity / latest_applied), which a caller
reads back to confirm -- this is the entire reason the plugin's status
topic exists (milestones.md M6).

One instance per worker process, same lifetime rule as PX4Clock/GzSimClock
(CLAUDE.md §3.3): never constructed in a SimFarm parent process, never
shared across workers.
"""
from __future__ import annotations

import os
import sys
import time
from typing import Optional

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

_SYSTEM_SITE_PACKAGES = "/usr/lib/python3/dist-packages"
if _SYSTEM_SITE_PACKAGES not in sys.path:
    # Appended, not inserted -- see simulation/sim_clock.py's module
    # docstring for the full reasoning (same pattern, same reason).
    sys.path.append(_SYSTEM_SITE_PACKAGES)

try:
    import gz.transport13 as _gz_transport
    from gz.msgs10.any_pb2 import Any as _GzAny
    from gz.msgs10.param_pb2 import Param as _GzParam
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    raise ImportError(
        "gz-transport/gz-msgs Python bindings not found. Install with: "
        "sudo apt install python3-gz-transport13 python3-gz-msgs10 "
        "(see scripts/env_report.sh)"
    ) from exc

from simulation.instance_spec import InstanceSpec


class RotorFaultControllerError(RuntimeError):
    """Raised when the RotorDegradationSystem plugin's status heartbeat
    never arrives within wait_for_heartbeat()'s deadline -- the plugin
    failed to load (e.g. the worker was started with plain x500 instead of
    x500_aero), not merely "no fault has been commanded yet"."""


def _params_to_dict(msg: _GzParam) -> dict:
    """gz.msgs10.param_pb2.Param's `params` field is a real
    ``map<string, Any>`` on the wire -- the plugin's C++ side, and `gz
    topic -e`, both read/write it correctly as a map. The apt-installed
    python3-gz-msgs10 Python bindings do NOT expose it as a Python
    dict-like map field, though: indexing it with a string key raises
    TypeError. Confirmed directly (M6, reproduced standalone, not assumed
    from a traceback): it deserializes as a plain repeated field of
    (key, value) ``ParamsEntry`` messages instead. This is the one place
    that translates it, so nothing else in this module needs to know."""
    return {entry.key: entry.value for entry in msg.params}


class RotorFaultController:
    def __init__(self, spec: InstanceSpec):
        # GZ_PARTITION is read by gz-transport at Node construction time --
        # same requirement simulation/sim_clock.py's GzSimClock documents.
        os.environ['GZ_PARTITION'] = spec.gz_partition
        self.spec = spec
        self._cmd_topic = f"/{spec.model_name}/rotor_fault/cmd"
        self._status_topic = f"/{spec.model_name}/rotor_fault/status"

        # Updated only from _on_status, delivered on a gz-transport
        # background thread (same reasoning as GzSimClock: reads of a
        # single int/float/bool are atomic under the GIL, so no lock).
        self.latest_rotor_index: int = -1
        self.latest_severity: float = 0.0
        self.latest_applied: bool = False
        self._last_status_wall_time: Optional[float] = None

        self._node = _gz_transport.Node()
        self._pub = self._node.advertise(self._cmd_topic, _GzParam)
        if not self._node.subscribe(_GzParam, self._status_topic, self._on_status):
            raise RotorFaultControllerError(
                f"failed to subscribe to {self._status_topic} in partition "
                f"{spec.gz_partition} -- is this worker's Gazebo server running?"
            )

    def _on_status(self, msg: _GzParam) -> None:
        by_key = _params_to_dict(msg)
        self.latest_rotor_index = by_key["rotor_index"].int_value
        self.latest_severity = by_key["severity"].double_value
        self.latest_applied = by_key["applied"].bool_value
        self._last_status_wall_time = time.monotonic()

    def set_rotor_fault(self, rotor_index: int, severity: float) -> None:
        """Commands rotor `rotor_index` to `severity` (fraction of thrust
        lost, planning.md §6). Fire-and-forget -- gz-transport gives no
        publish acknowledgement, and this returning does not mean the fault
        was applied. Read latest_applied / latest_rotor_index /
        latest_severity back afterward to confirm (they update from the
        plugin's own status echo, published on every change)."""
        msg = _GzParam()

        rotor_entry = msg.params.add()
        rotor_entry.key = "rotor_index"
        rotor_entry.value.type = _GzAny.INT32
        rotor_entry.value.int_value = rotor_index

        severity_entry = msg.params.add()
        severity_entry.key = "severity"
        severity_entry.value.type = _GzAny.DOUBLE
        severity_entry.value.double_value = severity

        self._pub.publish(msg)

    def clear_rotor_fault(self) -> None:
        """Commands "no rotor faulted" -- the same sentinel
        (rotor_index=-1, severity=0.0) FaultSpec.healthy() and the plugin's
        own idle state both use."""
        self.set_rotor_fault(-1, 0.0)

    def wait_for_heartbeat(self, timeout_s: float = 10.0) -> None:
        """Bounded wall-clock readiness gate, run once before any mission
        flies -- the same category as scripts/sim_start.sh's own
        DDS-readiness poll, not flight/mission logic (CLAUDE.md §4's
        wall-clock restriction is for the latter). Catches "plugin never
        loaded" (wrong --model, or the model.sdf's plugin block missing)
        before an episode's flight time is spent discovering a fault was
        never confirmed applied.
        """
        deadline = time.monotonic() + timeout_s
        while self._last_status_wall_time is None:
            if time.monotonic() > deadline:
                raise RotorFaultControllerError(
                    f"no rotor_fault/status heartbeat on {self._status_topic} "
                    f"within {timeout_s}s -- RotorDegradationSystem did not "
                    f"load for model '{self.spec.model}' (wrong --model, or "
                    f"its model.sdf has no RotorDegradationSystem plugin block?)"
                )
            time.sleep(0.05)

    def close(self) -> None:
        self._node.unsubscribe(self._status_topic)
