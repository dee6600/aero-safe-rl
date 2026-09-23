"""M7 task 3: the fault detector model -- a rotor-symmetric streaming GRU with
a deep-ensemble head (milestones.md M7, "Model choice").

  RotorFrameTransform   raw feature vector -> each rotor's own view of it
                        (fixed, no parameters; x500 geometry below)
  RotorGRU              one shared GRU over the four rotor views, a 5-way
                        {healthy, rotor 0..3} head and per-rotor severity
  DetectorEnsemble      N independently seeded RotorGRUs + one temperature;
                        the only thing callers use, in batch (evaluation)
                        and in streaming (ai/detector/runtime.py) mode alike

Symmetry. For each rotor i, the transform expresses tilt, body rate and
horizontal acceleration along that rotor's arm direction d_i ("towards me")
and across it, the across-arm and yaw terms signed by the rotor's spin
direction. Under the x500's left-right mirror (rotor 0 <-> 2, 1 <-> 3,
roll/yaw/p/r/a_y/v_y negated) each rotor's view maps exactly onto its mirror
partner's, so the model's rotor outputs permute and p(fault) / severity are
unchanged: tests/test_detector_model.py checks this. Front-back symmetry
holds only approximately (the arms are 0.22 m vs 0.20 m).

Causal by construction: every layer before the GRU is per-tick and the GRU is
unidirectional, so an output never depends on a later tick.

No ROS import. Runs on CPU for inference; train.py may use CUDA.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import torch
import yaml
from torch import nn

from ai.features.feature_extractor import all_feature_names, feature_version

REPO = Path(__file__).resolve().parent.parent.parent
NORMALIZATION_PATH = REPO / "configs" / "rl" / "normalization_v1.yaml"

# PX4's own x500 control-allocation geometry,
# PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500
# (CA_ROTOR{i}_PX / _PY / _KM), FRD body frame: x forward, y right.
X500_ROTOR_GEOMETRY = {
    "px": (0.13, -0.13, 0.13, -0.13),
    "py": (0.22, -0.20, -0.22, 0.20),
    "km": (0.05, 0.05, -0.05, -0.05),
}

N_ROTORS = 4
N_LOCAL = 9     # see RotorFrameTransform.forward
N_GLOBAL = 7
N_CLASSES = 1 + N_ROTORS


class CheckpointMismatch(ValueError):
    """A checkpoint was trained under a different feature contract or
    normalisation than the one this checkout would feed it."""


def file_digest(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16]


def load_normalization(path: str | Path = NORMALIZATION_PATH) -> dict[str, tuple[float, float]]:
    """{feature: (mean, std)} from the frozen normalisation file."""
    cfg = yaml.safe_load(Path(path).read_text())
    if str(cfg["feature_version"]) != feature_version():
        raise CheckpointMismatch(
            f"{path} is for feature_version {cfg['feature_version']}, "
            f"configs/features.yaml is {feature_version()}")
    return {k: (float(v["mean"]), float(v["std"])) for k, v in cfg["stats"].items()}


class RotorFrameTransform(nn.Module):
    """(B, T, F) raw features -> (B, T, 4, N_LOCAL) rotor views and
    (B, T, N_GLOBAL) rotor-independent context. NaN inputs become 0 after
    scaling, i.e. the healthy-flight mean."""

    def __init__(self, norm: dict[str, tuple[float, float]],
                 geometry: dict = X500_ROTOR_GEOMETRY):
        super().__init__()
        names = all_feature_names()
        self.idx = {n: i for i, n in enumerate(names)}
        mean = lambda *ks: float(np.mean([norm[k][0] for k in ks]))  # noqa: E731
        std = lambda *ks: float(np.mean([norm[k][1] for k in ks]))   # noqa: E731
        motors = [f"motor_{i}_output" for i in range(N_ROTORS)]
        self.s_ang = std("roll_rad", "pitch_rad")
        self.s_rate = std("rate_p_rad_s", "rate_q_rad_s")
        self.s_yaw = std("rate_r_rad_s")
        self.s_acc = std("accel_x_m_s2", "accel_y_m_s2")
        self.s_vh = std("vel_x", "vel_y")
        self.mu_mot, self.s_mot = mean(*motors), std(*motors)
        self.glob_norm = {k: norm[k] for k in
                          ("vel_z", "position_error_m", "accel_z_m_s2",
                           "battery_remaining", "thrust_accel_residual")}
        px, py = np.array(geometry["px"]), np.array(geometry["py"])
        r = np.hypot(px, py)
        self.register_buffer("dx", torch.tensor(px / r, dtype=torch.float32))
        self.register_buffer("dy", torch.tensor(py / r, dtype=torch.float32))
        self.register_buffer("spin", torch.tensor(np.sign(geometry["km"]), dtype=torch.float32))

    def _f(self, x: torch.Tensor, name: str) -> torch.Tensor:
        return x[..., self.idx[name]]

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        f = lambda n: self._f(x, n).unsqueeze(-1)  # noqa: E731  (B, T, 1)
        u = torch.stack([self._f(x, f"motor_{i}_output") for i in range(N_ROTORS)], -1)
        u_mean = u.mean(-1, keepdim=True)
        dx, dy, k = self.dx, self.dy, self.spin

        def along_across(tx, ty):
            """Components of the horizontal (tx, ty) along each arm and
            across it, the across term spin-signed so the mirror maps
            rotor views onto each other."""
            along = dx * tx + dy * ty
            across = k * (dx * ty - dy * tx)
            return along, across

        # Tilt / rate "towards" a point (x, y) is (-pitch, roll) / (-q, p).
        tilt = along_across(-f("pitch_rad") / self.s_ang, f("roll_rad") / self.s_ang)
        rate = along_across(-f("rate_q_rad_s") / self.s_rate, f("rate_p_rad_s") / self.s_rate)
        acc = along_across(f("accel_x_m_s2") / self.s_acc, f("accel_y_m_s2") / self.s_acc)
        local = torch.stack([
            (u - u_mean) / self.s_mot,
            (u - self.mu_mot) / self.s_mot,
            tilt[0], tilt[1], rate[0], rate[1],
            k * f("rate_r_rad_s") / self.s_yaw,
            acc[0], acc[1],
        ], dim=-1)                                                   # (B, T, 4, N_LOCAL)

        g = lambda n: (self._f(x, n) - self.glob_norm[n][0]) / self.glob_norm[n][1]  # noqa: E731
        glob = torch.stack([
            (u_mean[..., 0] - self.mu_mot) / self.s_mot,
            torch.hypot(self._f(x, "vel_x"), self._f(x, "vel_y")) / self.s_vh,
            g("vel_z"), g("position_error_m"), g("accel_z_m_s2"),
            g("battery_remaining"), g("thrust_accel_residual"),
        ], dim=-1)                                                   # (B, T, N_GLOBAL)
        return torch.nan_to_num(local), torch.nan_to_num(glob)


class RotorGRU(nn.Module):
    """One ensemble member. forward() returns 5-way logits (B, T, 5), per-rotor
    severity in [0, 1] (B, T, 4) and the GRU state (1, B * 4, H) to carry to
    the next call when streaming."""

    def __init__(self, transform: RotorFrameTransform, embed: int = 32, hidden: int = 48):
        super().__init__()
        self.transform = transform
        self.hidden = hidden
        self.embed = nn.Sequential(nn.Linear(N_LOCAL + N_GLOBAL, embed), nn.Tanh())
        self.gru = nn.GRU(embed, hidden, batch_first=True)
        self.rotor_head = nn.Sequential(nn.Linear(2 * hidden, hidden), nn.Tanh(),
                                        nn.Linear(hidden, 2))
        self.healthy_head = nn.Sequential(nn.Linear(2 * hidden, hidden), nn.Tanh(),
                                          nn.Linear(hidden, 1))

    def forward(self, x: torch.Tensor, h0: Optional[torch.Tensor] = None):
        local, glob = self.transform(x)
        B, T = x.shape[:2]
        z = torch.cat([local, glob.unsqueeze(2).expand(-1, -1, N_ROTORS, -1)], -1)
        z = self.embed(z).permute(0, 2, 1, 3).reshape(B * N_ROTORS, T, -1)
        h, hn = self.gru(z, h0)
        h = h.reshape(B, N_ROTORS, T, self.hidden).permute(0, 2, 1, 3)   # (B, T, 4, H)
        pooled_mean, pooled_max = h.mean(2), h.max(2).values
        r = self.rotor_head(torch.cat([h, pooled_mean.unsqueeze(2).expand_as(h)], -1))
        healthy = self.healthy_head(torch.cat([pooled_mean, pooled_max], -1))
        logits = torch.cat([healthy, r[..., 0]], -1)
        return logits, torch.sigmoid(r[..., 1]), hn


def combine_members(probs_m: torch.Tensor, sev_m: torch.Tensor) -> dict[str, torch.Tensor]:
    """Ensemble output from per-member class probabilities (M, ..., 5) and
    per-rotor severities (M, ..., 4). The one combination rule, shared by
    DetectorEnsemble (batch) and StreamingEnsemble (online)."""
    p = probs_m.mean(0)
    rotor = p[..., 1:].argmax(-1)
    severity = torch.gather(sev_m.mean(0), -1, rotor.unsqueeze(-1))[..., 0]
    return {
        "probs": p,
        "p_fault": 1.0 - p[..., 0],
        "rotor": rotor,
        "severity": severity,
        "uncertainty": (1.0 - probs_m[..., 0]).std(0, unbiased=False),
    }


class DetectorEnsemble(nn.Module):
    """Members averaged in probability space after a shared temperature.
    forward() -> dict of (B, T) tensors plus the members' carried states."""

    def __init__(self, members: Sequence[RotorGRU], temperature: float = 1.0):
        super().__init__()
        self.members = nn.ModuleList(members)
        self.temperature = float(temperature)

    def forward(self, x: torch.Tensor, hidden: Optional[list] = None):
        hidden = hidden or [None] * len(self.members)
        probs, sevs, new_hidden = [], [], []
        for m, h0 in zip(self.members, hidden):
            logits, sev, hn = m(x, h0)
            probs.append(torch.softmax(logits / self.temperature, -1))
            sevs.append(sev)
            new_hidden.append(hn)
        return combine_members(torch.stack(probs), torch.stack(sevs)), new_hidden


class StreamingEnsemble:
    """One-tick-at-a-time inference for a trained DetectorEnsemble, with
    every member's weights stacked so the five members run as one set of
    batched ops, and the (member-independent) input transform computed once.
    Online a tick is dominated by per-op overhead, and under simulator CPU
    load the member-by-member path measured p99 34 ms live against a 20 ms
    budget. Same maths as RotorGRU.forward (nn.GRU's gate equations);
    tests/test_detector_runtime.py asserts it matches batch inference.

    step(x (F,), h (M, 4, H) or None) -> (output dict of scalars' tensors, h).
    """

    def __init__(self, ensemble: DetectorEnsemble):
        ms = list(ensemble.members)
        st = lambda get: torch.stack([get(m).detach() for m in ms])  # noqa: E731
        self.transform = ms[0].transform
        self.temperature = ensemble.temperature
        self.hidden = ms[0].hidden
        self.w_e, self.b_e = st(lambda m: m.embed[0].weight), st(lambda m: m.embed[0].bias)
        self.w_ih, self.b_ih = st(lambda m: m.gru.weight_ih_l0), st(lambda m: m.gru.bias_ih_l0)
        self.w_hh, self.b_hh = st(lambda m: m.gru.weight_hh_l0), st(lambda m: m.gru.bias_hh_l0)
        self.r1w, self.r1b = st(lambda m: m.rotor_head[0].weight), st(lambda m: m.rotor_head[0].bias)
        self.r2w, self.r2b = st(lambda m: m.rotor_head[2].weight), st(lambda m: m.rotor_head[2].bias)
        self.h1w, self.h1b = st(lambda m: m.healthy_head[0].weight), st(lambda m: m.healthy_head[0].bias)
        self.h2w, self.h2b = st(lambda m: m.healthy_head[2].weight), st(lambda m: m.healthy_head[2].bias)
        self.n_members = len(ms)
        for m in ms[1:]:  # computing the transform once is only valid if it is shared
            if not (torch.equal(m.transform.dx, self.transform.dx)
                    and torch.equal(m.transform.dy, self.transform.dy)
                    and torch.equal(m.transform.spin, self.transform.spin)):
                raise ValueError("ensemble members use different rotor geometries")

    @staticmethod
    def _linear(w, b, x):
        """Per-member linear layer: w (M, O, I), b (M, O), x (M, ..., I)."""
        shape = x.shape
        y = torch.bmm(x.reshape(shape[0], -1, shape[-1]), w.transpose(1, 2)) + b[:, None, :]
        return y.reshape(*shape[:-1], w.shape[1])

    @torch.no_grad()
    def step(self, x: torch.Tensor, h: Optional[torch.Tensor] = None):
        M, H = self.n_members, self.hidden
        local, glob = self.transform(x.reshape(1, 1, -1))
        z = torch.cat([local[0, 0], glob[0, 0].expand(N_ROTORS, -1)], -1)       # (4, 16)
        e = torch.tanh(self._linear(self.w_e, self.b_e, z.expand(M, -1, -1)))   # (M, 4, E)
        if h is None:
            h = torch.zeros(M, N_ROTORS, H)
        gi = self._linear(self.w_ih, self.b_ih, e)
        gh = self._linear(self.w_hh, self.b_hh, h)
        i_r, i_z, i_n = gi.split(H, -1)
        h_r, h_z, h_n = gh.split(H, -1)
        r = torch.sigmoid(i_r + h_r)
        zg = torch.sigmoid(i_z + h_z)
        n = torch.tanh(i_n + r * h_n)
        h = (1 - zg) * n + zg * h                                               # (M, 4, H)

        mean, mx = h.mean(1), h.max(1).values                                   # (M, H)
        ro = self._linear(self.r2w, self.r2b, torch.tanh(self._linear(
            self.r1w, self.r1b, torch.cat([h, mean[:, None].expand_as(h)], -1))))  # (M, 4, 2)
        he = self._linear(self.h2w, self.h2b, torch.tanh(self._linear(
            self.h1w, self.h1b, torch.cat([mean, mx], -1)[:, None])))[:, 0]    # (M, 1)
        logits = torch.cat([he, ro[..., 0]], -1)                                # (M, 5)
        out = combine_members(torch.softmax(logits / self.temperature, -1),
                              torch.sigmoid(ro[..., 1]))
        return out, h


def build_ensemble(n_members: int, norm: dict, embed: int = 32, hidden: int = 48,
                   seeds: Optional[Sequence[int]] = None) -> DetectorEnsemble:
    seeds = list(seeds) if seeds is not None else list(range(n_members))
    members = []
    for s in seeds:
        torch.manual_seed(s)
        members.append(RotorGRU(RotorFrameTransform(norm), embed=embed, hidden=hidden))
    return DetectorEnsemble(members)


def save_checkpoint(path: str | Path, ensemble: DetectorEnsemble, *, seeds: Sequence[int],
                    split_digest: str, threshold: float, extra: Optional[dict] = None) -> None:
    m0 = ensemble.members[0]
    torch.save({
        "members": [m.state_dict() for m in ensemble.members],
        "temperature": ensemble.temperature,
        "arch": {"embed": m0.embed[0].out_features, "hidden": m0.hidden},
        "seeds": list(seeds),
        "feature_version": feature_version(),
        "feature_names": list(all_feature_names()),
        "norm_digest": file_digest(NORMALIZATION_PATH),
        "geometry": X500_ROTOR_GEOMETRY,
        "split_digest": split_digest,
        "threshold": float(threshold),
        "extra": extra or {},
    }, path)


def load_checkpoint(path: str | Path) -> tuple[DetectorEnsemble, dict]:
    """The ensemble (eval mode, CPU) and its metadata. Raises
    CheckpointMismatch if the checkpoint was trained under a different
    feature contract or normalisation than this checkout's."""
    ckpt = torch.load(path, map_location="cpu", weights_only=False)
    if ckpt["feature_version"] != feature_version() or \
            ckpt["feature_names"] != list(all_feature_names()):
        raise CheckpointMismatch(
            f"{path}: trained on feature_version {ckpt['feature_version']}, "
            f"this checkout is {feature_version()}")
    if ckpt["norm_digest"] != file_digest(NORMALIZATION_PATH):
        raise CheckpointMismatch(f"{path}: trained under different normalisation statistics")
    if ckpt["geometry"] != X500_ROTOR_GEOMETRY:
        raise CheckpointMismatch(f"{path}: trained for a different rotor geometry")
    ens = build_ensemble(len(ckpt["members"]), load_normalization(), **ckpt["arch"],
                         seeds=ckpt["seeds"])
    for m, sd in zip(ens.members, ckpt["members"]):
        m.load_state_dict(sd)
    ens.temperature = ckpt["temperature"]
    ens.eval()
    return ens, ckpt


@torch.no_grad()
def predict_episode(ensemble: DetectorEnsemble, features: np.ndarray):
    """Batch (whole-episode) inference -> experiments.metrics.DetectorTrace.
    Identical to streaming the same ticks one at a time through
    ai/detector/runtime.py (tests/test_detector_runtime.py)."""
    from experiments.metrics import DetectorTrace
    device = next(ensemble.parameters()).device
    out, _ = ensemble(torch.as_tensor(features, dtype=torch.float32, device=device)[None])
    get = lambda k: out[k][0].cpu().numpy()  # noqa: E731
    return DetectorTrace(score=get("p_fault"), is_probability=True, rotor=get("rotor"),
                         severity=get("severity"), uncertainty=get("uncertainty"))
