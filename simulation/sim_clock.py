"""Reads Gazebo's own simulated time directly over native gz-transport,
bypassing PX4 and ROS 2 entirely. This is the ONLY correct source of
simulated time in this project -- see the finding below before reaching for
anything else.

Why this module exists (found empirically during M2, 2026-08-20):

The obvious design -- read the `timestamp` field PX4 stamps on every
px4_msgs message -- is WRONG. It looks like it should work: PX4 SITL's
internal hrt_absolute_time() genuinely is lockstep sim time (Gazebo drives
PX4's CLOCK_MONOTONIC directly; see GZBridge.cpp:331-345 and
docs/parallelism.md). But that raw value is not what ends up on the ROS 2
side. uxrce_dds_client resynchronizes every published message's timestamp to
the MicroXRCEAgent's wall clock via a session-level Timesync handshake
(uxrce_dds_client.cpp, `_synchronize_timestamps` / `on_time` /
`session->time_offset`), because the bridge is designed for real hardware,
where a MAVLink-style epoch-ish timestamp is the useful convention.

Measured directly, with a worker running at PX4_SIM_SPEED_FACTOR=4: over a
10-second wall-clock window, `vehicle_status.timestamp` advanced by 9.938s
-- a ratio of 0.991 to wall-clock elapsed, i.e. it tracks REAL time almost
exactly, not the 4x sim rate actually in effect. Every px4_msgs timestamp
goes through the same session-level offset, so this is not specific to one
topic -- none of them are usable as a sim-time source.

Gazebo's own `/world/<world>/stats` topic, read directly (not through the
DDS bridge), does not have this problem. The same experiment against it, at
the same speed factor 4, gave a ratio of 3.945 -- matching the requested
factor almost exactly, and unlike a value derived from the requested
speed_factor alone, it reports what ACTUALLY happened, which matters under
contention (M4's multi-worker throughput measurements showed achieved RTF
falling below the requested factor).

**CLAUDE.md D10 ("sim time is the only clock in flight logic") means "GzSimClock
via this module", never a px4_msgs timestamp.** PX4Interface.last_timestamp_us
still exists as a link-freshness indicator (has anything arrived recently?)
but must never be used as a duration source.

Requires the apt-installed python3-gz-transport13 + python3-gz-msgs10
bindings. They live under the system Python's site-packages
(/usr/lib/python3/dist-packages), not the conda env -- confirmed importable
from conda's Python 3.10 (matching cpython-310 ABI tag). This module is the
one place that path gets added, rather than scattering the shim across every
caller. scripts/env_report.sh checks both are present.

Second environment wrinkle, also found during M2: gz-transport13's compiled
extension links the SYSTEM libprotobuf, while this project's conda env
carries a much newer standalone `protobuf` package (pulled in transitively
by TensorBoard/PyTorch, per environment.yml). Whichever loads first decides
the process-wide protobuf backend; if the newer one wins, gz-msgs10's
generated _pb2.py files (built by an older protoc) fail with "Descriptors
cannot be created directly" the moment they're imported. Forcing the pure-
Python protobuf implementation (protobuf's own documented workaround for
exactly this class of conflict) fixes it. The performance note in that
workaround's warning does not apply here -- WorldStatistics is a handful of
scalar fields published at ~10-20 Hz, not a bulk-throughput path -- and the
processes that need this module are per-worker flight processes, not the
process doing TensorBoard logging (that's the learner, a different OS
process under this project's one-process-per-worker design), so no other
protobuf-heavy consumer shares this cost. setdefault, not a stomp: an
explicit override some other part of the process already made is respected.
"""

import os
import sys
from typing import Optional

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

_SYSTEM_SITE_PACKAGES = "/usr/lib/python3/dist-packages"
if _SYSTEM_SITE_PACKAGES not in sys.path:
    # Appended, not inserted -- if the conda env ever ships its own gz
    # bindings, those must win over this fallback, not be shadowed by it.
    sys.path.append(_SYSTEM_SITE_PACKAGES)

try:
    import gz.transport13 as _gz_transport
    from gz.msgs10.world_stats_pb2 import WorldStatistics as _WorldStatistics
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    raise ImportError(
        "gz-transport/gz-msgs Python bindings not found. Install with: "
        "sudo apt install python3-gz-transport13 python3-gz-msgs10 "
        "(see scripts/env_report.sh)"
    ) from exc


class GzSimClockError(RuntimeError):
    """Raised when the world's stats topic cannot be subscribed to at all --
    distinct from simply not having received a message yet, which is a
    normal (if brief) startup state, not an error."""


class GzSimClock:
    """Subscribes to /world/<world>/stats in one worker's GZ_PARTITION and
    exposes Gazebo's own sim_time as microseconds -- suitable as PX4Clock's
    now_us_fn.

    Delivery happens on a background thread owned by gz-transport, updating
    a plain int attribute; reads of a single int are atomic under the GIL,
    so no lock is used.
    """

    def __init__(self, world: str, gz_partition: str):
        # GZ_PARTITION is read by gz-transport at Node construction time, the
        # same requirement ROS_DOMAIN_ID has for rclpy (see test_flight.py).
        # This sets it for the WHOLE process -- fine under this project's one
        # worker per OS process rule (CLAUDE.md §3.3), but would be wrong for
        # a process juggling multiple workers' clocks at once, which the
        # architecture deliberately never does.
        os.environ['GZ_PARTITION'] = gz_partition
        self.world = world
        self.gz_partition = gz_partition
        self._sim_us: Optional[int] = None
        self._node = _gz_transport.Node()
        self._topic = f"/world/{world}/stats"
        if not self._node.subscribe(_WorldStatistics, self._topic, self._on_stats):
            raise GzSimClockError(
                f"failed to subscribe to {self._topic} in partition "
                f"{gz_partition} -- is this worker's Gazebo server running?"
            )

    def _on_stats(self, msg: "_WorldStatistics") -> None:
        self._sim_us = msg.sim_time.sec * 1_000_000 + msg.sim_time.nsec // 1000

    def now_us(self) -> Optional[int]:
        """Latest known simulated time, in microseconds. None until the
        first /stats message has arrived."""
        return self._sim_us

    def close(self) -> None:
        self._node.unsubscribe(self._topic)
