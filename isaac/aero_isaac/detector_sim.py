"""The simulated detector (M8b task 6). During Isaac training the real M7
detector cannot run -- it needs PX4 telemetry -- so the policy sees this
stand-in's output instead. It is built entirely from
configs/rl/detector_sim_v1.yaml, which experiments/fit_detector_sim.py fits to
the real detector's recorded output (the model is described there);
isaac/tests/test_detector_sim.py checks it against flights held out of the fit.

It is given the true fault, as a simulator of the detector must be; what it
returns is the only thing that reaches the observation (observation.py never
receives the true fault).

Per drone and episode: when the fault is detected (a delay for a step; for
a ramp, when the true severity reaches a drawn level) or a miss, the rotor it will name,
and how strongly it over-reads severity after a commanded descent starts
(the measured over-read profile, scaled). False alarms are drawn as observed
(length, height) pairs. Per 0.1 s tick: the
fault probability, rotor, severity estimate, uncertainty and alarm.
"""
from __future__ import annotations

import math
from pathlib import Path

import torch
import yaml

from aero_isaac.contracts import CONFIGS

PARAMS_PATH = CONFIGS / "rl" / "detector_sim_v1.yaml"
DESCENT_TRIGGER_M = -0.05     # fit_detector_sim.py: a descent starts when the commanded offset first goes below this


def load_params(path: Path = PARAMS_PATH) -> dict:
    return yaml.safe_load(Path(path).read_text())


def sample(quantiles: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
    """Inverse-CDF sampling from evenly spaced quantiles (last dim), with
    linear interpolation. quantiles (..., K), u (...,) in [0, 1]."""
    k = quantiles.shape[-1] - 1
    quantiles = quantiles.expand(*u.shape, k + 1)      # one shared table, or one per draw
    x = u.clamp(0.0, 1.0) * k
    lo = x.floor().long().clamp(max=k - 1)
    w = x - lo
    q_lo = torch.gather(quantiles, -1, lo.unsqueeze(-1)).squeeze(-1)
    q_hi = torch.gather(quantiles, -1, (lo + 1).unsqueeze(-1)).squeeze(-1)
    return q_lo + w * (q_hi - q_lo)


class DetectorSim:

    def __init__(self, num_envs: int, device=None, generator: torch.Generator | None = None,
                 params: dict | None = None):
        p = params or load_params()
        self.n, self.device = num_envs, device
        self.gen = generator
        t = lambda v: torch.tensor(v, dtype=torch.float32, device=device)  # noqa: E731
        self.tick_s = float(p["tick_s"])
        self.threshold, self.hold = float(p["alarm"]["threshold"]), int(p["alarm"]["hold_ticks"])
        self.bands = t(p["severity_bands"])
        bg, fa, det = p["background"], p["false_alarm"], p["detection"]
        self.bg_p, self.bg_sev, self.bg_unc = t(bg["p_quantiles"]), t(bg["severity_quantiles"]), \
            t(bg["uncertainty_quantiles"])
        self.burst_rate = float(fa["rate_per_hour"]) / 3600.0 * self.tick_s   # per tick
        self.bursts = t(fa["bursts"])                                          # (K, 2) duration s, peak p
        self.burst_sev = t(fa["severity_quantiles"])
        self.det_p, self.det_unc = t(det["p_quantiles"]), t(det["uncertainty_quantiles"])
        self.rotor_acc = t(det["rotor_accuracy"])
        self.step_delay_q = t(det["step"]["delay_s_quantiles"])               # (bands, 21) seconds
        self.ramp_level_q = t(det["ramp"]["level_quantiles"])                 # (bands, 21) severity at detection
        self.miss = torch.stack([t(det["step"]["miss_rate"]), t(det["ramp"]["miss_rate"])])   # (2, bands)
        se = p["severity_error"]
        self.bias, self.std, self.phi = t(se["bias"]), t(se["std"]), float(se["ar1"])
        self.descent_bin_s = float(p["descent"]["profile_bin_s"])
        self.descent_profile = t(p["descent"]["profile"])
        self.descent_scale = t(p["descent"]["scale_quantiles"])

        z = lambda dtype=torch.float32: torch.zeros(num_envs, dtype=dtype, device=device)  # noqa: E731
        self.detect_at = z() + math.inf
        self.rotor_named = z(torch.long)
        self.band = z(torch.long)
        self.ar = z()
        self.descent_at = z() + math.inf
        self.descent_a = z()
        self.burst_left = z()
        self.burst_level = z()
        self.run = z(torch.long)

    def _u(self, *shape) -> torch.Tensor:
        return torch.rand(*shape, generator=self.gen, device=self.device)

    def reset(self, env_ids: torch.Tensor, target: torch.Tensor, rotor: torch.Tensor, onset: torch.Tensor,
              ramp_s: torch.Tensor) -> None:
        """New episode for env_ids: target severity (0 = healthy), faulted
        rotor (-1 = none), onset time, ramp length (0 = a step)."""
        k = len(env_ids)
        band = (torch.bucketize(target, self.bands, right=True) - 1).clamp(0, len(self.bands) - 2)
        is_ramp = ramp_s > 0
        step_delay = sample(self.step_delay_q[band], self._u(k))
        level = sample(self.ramp_level_q[band], self._u(k))
        # a ramp is detected when it reaches the level; one it never reaches, a step delay after it ends
        reach = (level / target.clamp(min=1e-6)).clamp(max=1.0) * ramp_s
        delay = torch.where(is_ramp, reach + torch.where(level >= target, step_delay, 0.0), step_delay)
        missed = self._u(k) < self.miss[is_ramp.long(), band]
        faulted = (rotor >= 0) & (target > 0)
        self.detect_at[env_ids] = torch.where(faulted & ~missed, onset + delay,
                                              torch.full_like(onset, math.inf))
        wrong = self._u(k) >= self.rotor_acc[band]
        other = (rotor.clamp(min=0) + 1 + torch.randint(0, 3, (k,), generator=self.gen, device=self.device)) % 4
        self.rotor_named[env_ids] = torch.where(wrong, other, rotor.clamp(min=0))
        self.band[env_ids] = band
        self.ar[env_ids] = torch.randn(k, generator=self.gen, device=self.device) * self.std[band]
        self.descent_at[env_ids] = math.inf
        self.descent_a[env_ids] = sample(self.descent_scale, self._u(k))
        self.burst_left[env_ids] = 0.0
        self.run[env_ids] = 0

    def step(self, t: torch.Tensor, severity: torch.Tensor, offset_cmd: torch.Tensor, **_) -> dict[str, torch.Tensor]:
        """One 0.1 s tick. severity: the true severity now; offset_cmd: the
        commanded altitude offset (for the descent over-read)."""
        n = self.n
        self.descent_at = torch.where(torch.isinf(self.descent_at) & (offset_cmd < DESCENT_TRIGGER_M), t,
                                      self.descent_at)
        detected = t >= self.detect_at

        start = (~detected) & (self.burst_left <= 0) & (self._u(n) < self.burst_rate)
        pick = self.bursts[torch.randint(0, len(self.bursts), (n,), generator=self.gen, device=self.device)]
        self.burst_left = torch.where(start, pick[:, 0], self.burst_left)
        self.burst_level = torch.where(start, pick[:, 1], self.burst_level)
        in_burst = (~detected) & (self.burst_left > 0)
        self.burst_left = (self.burst_left - self.tick_s).clamp(min=0.0)

        self.ar = self.phi * self.ar + math.sqrt(1 - self.phi ** 2) * self.std[self.band] * \
            torch.randn(n, generator=self.gen, device=self.device)
        since_descent = t - self.descent_at
        idx = (since_descent.clamp(min=0) / self.descent_bin_s).long().clamp(max=len(self.descent_profile) - 1)
        excess = torch.where(since_descent >= 0, self.descent_a * self.descent_profile[idx], torch.zeros_like(t))
        sev_detected = (severity + self.bias[self.band] + self.ar + excess).clamp(0.0, 1.0)

        p = torch.where(detected, sample(self.det_p, self._u(n)),
                        torch.where(in_burst, self.burst_level, sample(self.bg_p, self._u(n))))
        sev = torch.where(detected, sev_detected,
                          torch.where(in_burst, sample(self.burst_sev, self._u(n)), sample(self.bg_sev, self._u(n))))
        unc = torch.where(detected, sample(self.det_unc, self._u(n)), sample(self.bg_unc, self._u(n)))
        rotor = torch.where(detected, self.rotor_named,
                            torch.randint(0, 4, (n,), generator=self.gen, device=self.device))
        self.run = torch.where(p >= self.threshold, self.run + 1, torch.zeros_like(self.run))
        return dict(p_fault=p, rotor=rotor.float(), severity=sev, uncertainty=unc,
                    alarm=(self.run >= self.hold).float())


class NoDetector:
    """'Nothing detected' on every tick; ignores the fault. For runs that
    must not see any detector (the agreement check's scripted flights)."""

    def __init__(self, num_envs: int, device=None):
        self.n, self.device = num_envs, device

    def reset(self, env_ids: torch.Tensor, **_) -> None:
        pass

    def step(self, **_) -> dict[str, torch.Tensor]:
        z = torch.zeros(self.n, device=self.device)
        return dict(p_fault=z, rotor=z.clone(), severity=z.clone(), uncertainty=z.clone(), alarm=z.clone())
