"""M6 task 3 (@pytest.mark.sim): RotorDegradationSystem actually loads
inside a real gz sim process and publishes its status heartbeat -- the
"proves the plugin loads at all" bar the milestone's own build order asks
for before anything about graded severity is trusted (task 7).

Uses gz.transport13 directly (the same native-binding pattern
simulation/sim_clock.py and aero_bridge/reset.py already establish), not
simulation/rotor_fault.py (M6 task 5, not built yet) -- a test needing its
own subscription for a test-only purpose is the same choice
test_telemetry_sanity.py's _Recorder already makes.
"""
import os
import sys
import time

import pytest

# Same bootstrap as simulation/sim_clock.py / aero_bridge/reset.py: the gz
# Python bindings are apt-installed system-wide, not in this conda env, and
# the protobuf C++ backend those apt packages link against isn't ABI-
# compatible with conda's newer protobuf wheel (see sim_clock.py's module
# docstring for the full story) -- both must be set up before the import.
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
_SYSTEM_SITE_PACKAGES = "/usr/lib/python3/dist-packages"
if _SYSTEM_SITE_PACKAGES not in sys.path:
    sys.path.append(_SYSTEM_SITE_PACKAGES)

import gz.transport13 as gz_transport  # noqa: E402
from gz.msgs10.param_pb2 import Param  # noqa: E402


def _wait_for_status(topic: str, timeout_s: float = 10.0) -> dict:
    os.environ['GZ_PARTITION'] = 'aero_0'
    node = gz_transport.Node()
    received = {}

    def _cb(msg: Param) -> None:
        # gz.msgs10.param_pb2.Param's `params` field is declared
        # `map<string, Any>` on the wire (confirmed: the C++ plugin side
        # writes/reads it as a real protobuf map, and `gz topic -e` prints
        # it correctly) -- but the apt-installed python3-gz-msgs10 bindings
        # do NOT expose it as a Python dict-like map field: indexing it with
        # a string key raises TypeError, because it deserializes as a plain
        # repeated field of ParamsEntry(key, value) messages instead
        # (confirmed directly, M6 -- this is a real quirk of this specific
        # compiled binding, not a misunderstanding of the wire format).
        # simulation/rotor_fault.py (task 5) must use this same linear-scan
        # pattern, not dict-style access.
        by_key = {entry.key: entry.value for entry in msg.params}
        received['rotor_index'] = by_key['rotor_index'].int_value
        received['severity'] = by_key['severity'].double_value
        received['applied'] = by_key['applied'].bool_value

    assert node.subscribe(Param, topic, _cb), f"failed to subscribe to {topic}"
    deadline = time.monotonic() + timeout_s
    while not received and time.monotonic() < deadline:
        time.sleep(0.05)
    node.unsubscribe(topic)
    return received


def test_rotor_fault_status_heartbeat_arrives(sim_worker_x500_aero):
    status = _wait_for_status("/x500_aero_0/rotor_fault/status")
    assert status, "no rotor_fault/status heartbeat arrived -- plugin did not load or configure"
    assert status['rotor_index'] == -1
    assert status['severity'] == 0.0
    assert status['applied'] is False


def test_rotor_fault_plugin_does_not_load_on_plain_x500(sim_worker):
    """Negative case: the plain x500 model has no RotorDegradationSystem
    plugin block at all, so no heartbeat should ever arrive on its
    (nonexistent) status topic -- proves the previous test is actually
    checking something plugin-specific, not just "some message showed up"."""
    status = _wait_for_status("/x500_0/rotor_fault/status", timeout_s=5.0)
    assert not status, f"unexpected rotor_fault/status on plain x500: {status}"
