"""M7 task 5: the online fault detector -- one tick in, one estimate out.

DetectorRuntime consumes the same per-step telemetry row mission_executor's
record_step() produces (and EpisodeRunner hands to `on_step`), runs it
through the one feature extractor and the trained ensemble, and carries the
GRU state to the next tick. Its outputs are identical to batch inference
over the recorded episode (ai.detector.model.predict_episode), and its alarm
identical to experiments.metrics.sustained() at the checkpoint's threshold
-- tests/test_detector_runtime.py asserts both, so what M7 measures offline
is what M8/M10 run online.

Call reset() at the start of every episode. Pure Python + torch on CPU; no
ROS import, so it runs inside a worker process without touching rclpy.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional

import torch

from ai.detector.model import StreamingEnsemble, load_checkpoint
from ai.features.feature_extractor import FeatureExtractor, TelemetryWindow
from experiments.metrics import ALARM_HOLD_TICKS


@dataclass(frozen=True)
class DetectorOutput:
    p_fault: float        # probability a rotor is degraded now
    rotor: int            # most likely degraded rotor, 0..3 (meaningful when p_fault is high)
    severity: float       # estimated fraction of that rotor's thrust lost
    uncertainty: float    # ensemble disagreement on p_fault (std across members)
    alarm: bool           # p_fault >= threshold for the last ALARM_HOLD_TICKS ticks


class DetectorRuntime:
    """Stateful per-episode detector. `torch_threads` caps torch's intra-op
    threads for this process: the network is tiny, and extra threads only
    compete with the simulator for CPU."""

    def __init__(self, checkpoint_path: str | Path, *, threshold: Optional[float] = None,
                 hold_ticks: int = ALARM_HOLD_TICKS, torch_threads: int = 1):
        torch.set_num_threads(torch_threads)
        self.ensemble, self.meta = load_checkpoint(checkpoint_path)
        self._stream = StreamingEnsemble(self.ensemble)
        self.threshold = float(self.meta["threshold"] if threshold is None else threshold)
        self.hold_ticks = hold_ticks
        self.extractor = FeatureExtractor()
        self.reset()

    def reset(self) -> None:
        self._window = TelemetryWindow()
        self._hidden: Optional[torch.Tensor] = None
        self._run = 0

    @torch.no_grad()
    def step(self, frame: Mapping[str, float]) -> DetectorOutput:
        self._window.append(frame)
        v = self.extractor.extract_vector(self._window)
        out, self._hidden = self._stream.step(torch.from_numpy(v), self._hidden)
        p = float(out["p_fault"])
        self._run = self._run + 1 if p >= self.threshold else 0
        return DetectorOutput(
            p_fault=p, rotor=int(out["rotor"]), severity=float(out["severity"]),
            uncertainty=float(out["uncertainty"]), alarm=self._run >= self.hold_ticks)
