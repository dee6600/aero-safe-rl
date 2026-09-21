"""Background whole-machine memory/CPU sampling, shared by M4 task 6's
throughput sweep (experiments/benchmark_throughput.py) and task 7's soak test
(tests/slow/test_soak.py) -- CLAUDE.md §1.4: exactly one implementation, not
one per script that happens to need it.

Samples whole-machine memory/CPU rather than per-PID: a worker's actual
process tree (px4, gz sim, MicroXRCEAgent, plus whatever gz sim itself forks)
is awkward to enumerate correctly and reparenting would silently under-count
it. System-wide used memory is simpler and good enough for calibrating an
operating point or checking for a leak -- the same approach
docs/isaac_feasibility.md used for its own RSS numbers.
"""
from __future__ import annotations

import statistics
import threading
from typing import Optional


class ResourceSampler:
    """Context manager: samples every `interval_s` on a background thread for
    as long as the `with` block is open."""

    def __init__(self, interval_s: float = 1.0):
        import psutil
        self._psutil = psutil
        self.interval_s = interval_s
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.peak_rss_mb = 0.0
        self.rss_samples_mb: list[float] = []
        self.cpu_percent_samples: list[float] = []

    def _run(self) -> None:
        while not self._stop.is_set():
            used_mb = self._psutil.virtual_memory().used / (1024 * 1024)
            self.peak_rss_mb = max(self.peak_rss_mb, used_mb)
            self.rss_samples_mb.append(used_mb)
            self.cpu_percent_samples.append(self._psutil.cpu_percent(interval=None))
            self._stop.wait(self.interval_s)

    def __enter__(self) -> "ResourceSampler":
        self._psutil.cpu_percent(interval=None)  # prime it -- first call is always 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    @property
    def mean_cpu_percent(self) -> float:
        return statistics.mean(self.cpu_percent_samples) if self.cpu_percent_samples else 0.0

    def grew_meaningfully(self, *, threshold_fraction: float = 0.15) -> bool:
        """Coarse "is memory growing" check for the soak test: compares the
        mean of the first quarter of samples against the mean of the last
        quarter. Not a strict monotonic check -- normal allocator/GC noise
        would make that flaky -- so this only flags a growth larger than
        `threshold_fraction` of the first quarter's mean."""
        n = len(self.rss_samples_mb)
        if n < 8:
            return False  # too few samples to say anything meaningful
        quarter = n // 4
        first_q_mean = statistics.mean(self.rss_samples_mb[:quarter])
        last_q_mean = statistics.mean(self.rss_samples_mb[-quarter:])
        if first_q_mean <= 0:
            return False
        return (last_q_mean - first_q_mean) / first_q_mean > threshold_fraction
