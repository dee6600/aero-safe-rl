"""The project's one clock abstraction for flight logic (CLAUDE.md D10).

PX4 SITL runs in lockstep: its own notion of time is driven entirely by
Gazebo's clock, not the wall clock of the machine it runs on. At
PX4_SIM_SPEED_FACTOR=8, one wall-clock second is eight seconds of flight. A
plain time.sleep(1.0) in mission logic means something different at every
speed factor, which is exactly the kind of bug that only shows up once
training moves off 1x.

PX4Clock itself is deliberately source-agnostic: sleep_sim(seconds) blocks
until now_us_fn() reports that many seconds have passed, without caring where
that value comes from. The obvious source -- the timestamp field PX4 stamps
on every message it publishes -- turned out to be WRONG: uxrce_dds_client
resynchronizes every published timestamp to the agent's wall clock before it
reaches ROS 2, so px4_msgs timestamps track real time almost exactly
regardless of speed factor (measured ratio 0.991 at requested 4x). Every real
caller in this project (test_flight.py, and everything built on it from M3
on) wires now_us_fn to simulation/sim_clock.py's GzSimClock instead, which
reads Gazebo's own clock directly over gz-transport and gave the correct
ratio (3.945 at the same speed factor) in the same experiment. Full story:
GzSimClock's module docstring and docs/parallelism.md §2.5.

The one place wall-clock time is legitimate is the safety deadline: if no
message ever arrives, sim time never advances and a naive implementation
would hang forever. That deadline is a hang watchdog, not mission timing, and
CLAUDE.md §4 carves it out explicitly for exactly that reason.

Deliberately has no rclpy import. now_us_fn and pump_fn are injected by the
caller (in practice GzSimClock.now_us and rclpy.spin_once) so this class can
be unit tested with a fake message stream and no ROS runtime at all.
"""

import time
from typing import Callable, Optional

DEFAULT_WALL_MARGIN_S = 10.0
DEFAULT_POLL_INTERVAL_S = 0.02


class ClockTimeout(RuntimeError):
    """Sim time did not advance far enough before the wall-clock safety
    deadline. Either no PX4 messages are arriving at all (link down, wrong
    ROS_DOMAIN_ID, wrong namespace) or the simulator itself has stalled."""


class PX4Clock:
    """now_us_fn() -> latest known sim time in microseconds, or None if no
    timestamped message has been received yet. pump_fn(timeout_s) performs
    one step of message reception (normally rclpy.spin_once); it must return
    within roughly timeout_s so the wall-clock deadline stays meaningful."""

    def __init__(self,
                 now_us_fn: Callable[[], Optional[int]],
                 pump_fn: Optional[Callable[[float], None]] = None,
                 wall_margin_s: float = DEFAULT_WALL_MARGIN_S,
                 poll_interval_s: float = DEFAULT_POLL_INTERVAL_S):
        self._now_us_fn = now_us_fn
        self._pump_fn = pump_fn if pump_fn is not None else (lambda t: time.sleep(t))
        self._wall_margin_s = wall_margin_s
        self._poll_interval_s = poll_interval_s

    def now_us(self) -> Optional[int]:
        """Latest known simulated time, in microseconds. None if nothing has
        been received yet."""
        return self._now_us_fn()

    def now_s(self) -> Optional[float]:
        us = self.now_us()
        return us / 1e6 if us is not None else None

    def pump(self, timeout_s: Optional[float] = None) -> None:
        """One step of message reception. Callers implementing their own wait
        loop (e.g. arming_sequence's wall-clock-bounded handshake) use this
        instead of calling rclpy.spin_once directly, so there is exactly one
        place that knows how this project pumps its ROS executor.

        Defaults to this instance's configured poll_interval_s -- NOT a
        module-level constant -- so a caller that customised poll_interval_s
        at construction time doesn't get silently overridden by every bare
        clock.pump() call.
        """
        self._pump_fn(timeout_s if timeout_s is not None else self._poll_interval_s)

    def sleep_sim(self, seconds: float) -> None:
        """Block until `seconds` of PX4 simulated time have elapsed.

        Bounded by a wall-clock deadline of `seconds + wall_margin_s`: since
        this project never runs slower than real time, waiting for `seconds`
        of sim time can never legitimately take longer than that in wall
        time. Exceeding it means the link is down, not that sim time is
        merely slow -- hence raising rather than continuing to wait.
        """
        if seconds < 0:
            raise ValueError(f"seconds must be >= 0, got {seconds}")

        wall_deadline = time.monotonic() + seconds + self._wall_margin_s
        start_us = self._wait_for_first_timestamp(wall_deadline)
        target_us = start_us + int(round(seconds * 1e6))

        while True:
            now_us = self.now_us()
            if now_us is not None and now_us >= target_us:
                return
            if time.monotonic() > wall_deadline:
                raise ClockTimeout(
                    f"sleep_sim({seconds}) timed out: sim time reached "
                    f"{now_us} us, needed {target_us} us, within "
                    f"{seconds + self._wall_margin_s:.1f}s wall-clock budget"
                )
            self.pump(self._poll_interval_s)

    def _wait_for_first_timestamp(self, wall_deadline: float) -> int:
        while True:
            now_us = self.now_us()
            if now_us is not None:
                return now_us
            if time.monotonic() > wall_deadline:
                raise ClockTimeout(
                    "sleep_sim: no PX4 timestamp received before the wall-clock "
                    "deadline -- check the link is up (ROS_DOMAIN_ID, namespace)"
                )
            self.pump(self._poll_interval_s)
