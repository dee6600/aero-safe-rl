#!/usr/bin/env python3
"""M7 task 3: train the detector ensemble on an M6 fault-dataset run.

Each member is a RotorGRU trained on whole episodes (full-sequence BPTT,
hidden state starting from zero at the episode's first tick, exactly as the
online runtime starts), with per-tick cross-entropy over {healthy, rotor
0..3} plus a severity regression on fault-active ticks. Early stopping,
the ensemble temperature and the alarm threshold are all chosen on the
validation split; the test split is never read here (evaluate.py reads it
once).

Usage:
    python ai/detector/train.py results/m6_dataset_v1 --out results/m7_detector_v1
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Sequence

import numpy as np
import torch
import torch.nn.functional as F

REPO = Path(__file__).resolve().parent.parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from ai.detector.dataset import EpisodeData, load_episodes, split_by_episode  # noqa: E402
from ai.detector.model import (  # noqa: E402
    DetectorEnsemble, RotorFrameTransform, RotorGRU, load_normalization, predict_episode,
    save_checkpoint,
)
from experiments.metrics import threshold_at_healthy_quantile  # noqa: E402

SPLIT_SEED = 20260923
SEVERITY_LOSS_WEIGHT = 10.0
TEMPERATURE_GRID = np.round(np.linspace(0.5, 3.0, 26), 2)


def collate(episodes: Sequence[EpisodeData], device) -> dict[str, torch.Tensor]:
    """Pads whole episodes to the longest one. Padding sits after each
    episode's last tick, so (the GRU being causal) it never changes a real
    tick's output; `mask` removes it from the loss."""
    T = max(len(e.elapsed_s) for e in episodes)
    B = len(episodes)
    x = np.zeros((B, T, episodes[0].features.shape[1]), np.float32)
    y = np.zeros((B, T), np.int64)
    sev = np.zeros((B, T), np.float32)
    mask = np.zeros((B, T), bool)
    for b, e in enumerate(episodes):
        n = len(e.elapsed_s)
        x[b, :n], y[b, :n], sev[b, :n], mask[b, :n] = e.features, e.rotor_class, e.severity, True
    t = lambda a: torch.as_tensor(a, device=device)  # noqa: E731
    return {"x": t(x), "y": t(y), "sev": t(sev), "mask": t(mask)}


def member_loss(net: RotorGRU, batch: dict) -> torch.Tensor:
    logits, sev, _ = net(batch["x"])
    mask, y = batch["mask"], batch["y"]
    ce = F.cross_entropy(logits[mask], y[mask])
    active = mask & (y > 0)
    if not active.any():
        return ce
    sev_true_rotor = torch.gather(sev, -1, (y.clamp(min=1) - 1).unsqueeze(-1))[..., 0]
    return ce + SEVERITY_LOSS_WEIGHT * F.mse_loss(sev_true_rotor[active], batch["sev"][active])


@torch.no_grad()
def eval_loss(net: RotorGRU, batches: list[dict]) -> float:
    net.eval()
    return float(np.mean([member_loss(net, b).item() for b in batches]))


def train_member(train_eps, val_eps, norm, *, seed: int, device, epochs: int, patience: int,
                 batch_size: int, lr: float, log) -> tuple[RotorGRU, dict]:
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    net = RotorGRU(RotorFrameTransform(norm)).to(device)
    opt = torch.optim.Adam(net.parameters(), lr=lr, weight_decay=1e-4)
    val_batches = [collate(val_eps[i:i + batch_size], device)
                   for i in range(0, len(val_eps), batch_size)]

    best, best_state, best_epoch, history = float("inf"), None, -1, []
    for epoch in range(epochs):
        net.train()
        order = rng.permutation(len(train_eps))
        for i in range(0, len(order), batch_size):
            batch = collate([train_eps[j] for j in order[i:i + batch_size]], device)
            opt.zero_grad()
            loss = member_loss(net, batch)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0)
            opt.step()
        v = eval_loss(net, val_batches)
        history.append(v)
        if v < best:
            best, best_epoch = v, epoch
            best_state = {k: t.detach().clone() for k, t in net.state_dict().items()}
        log(f"  seed {seed} epoch {epoch:3d}  val loss {v:.4f}  (best {best:.4f} @ {best_epoch})")
        if epoch - best_epoch >= patience:
            break
    net.load_state_dict(best_state)
    net.eval()
    return net, {"seed": seed, "best_epoch": best_epoch, "best_val_loss": best,
                 "val_loss_history": history}


@torch.no_grad()
def fit_temperature(ensemble: DetectorEnsemble, val_eps: Sequence[EpisodeData], device) -> float:
    """Scalar temperature minimising the ensemble's per-tick NLL on val."""
    logits = [[m(torch.as_tensor(e.features, device=device)[None])[0][0] for m in ensemble.members]
              for e in val_eps]
    y = torch.cat([torch.as_tensor(e.rotor_class, device=device) for e in val_eps])
    nll = {}
    for T in TEMPERATURE_GRID:
        p = torch.cat([torch.stack([torch.softmax(l / T, -1) for l in ls]).mean(0) for ls in logits])
        nll[float(T)] = float(-torch.log(p[torch.arange(len(y)), y].clamp_min(1e-12)).mean())
    return min(nll, key=nll.get)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--out", default=str(REPO / "results" / "m7_detector_v1"))
    ap.add_argument("--members", type=int, default=5)
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--patience", type=int, default=12)
    ap.add_argument("--batch-size", type=int, default=16)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args(argv)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    log_lines: list[str] = []

    def log(msg: str) -> None:
        print(msg, flush=True)
        log_lines.append(msg)

    t0 = time.time()
    episodes, counts = load_episodes(args.run_dir)
    split = split_by_episode(episodes, seed=SPLIT_SEED)
    train_eps, val_eps = split.select(episodes, "train"), split.select(episodes, "val")
    log(f"loaded {len(episodes)} episodes in {time.time() - t0:.0f}s; categories {counts}")
    log(f"split {split.digest()}: train {len(train_eps)}  val {len(val_eps)}  "
        f"test {len(split.test)} (not read here)")

    norm = load_normalization()
    seeds = [SPLIT_SEED + k for k in range(args.members)]
    members, member_logs = [], []
    for s in seeds:
        net, info = train_member(train_eps, val_eps, norm, seed=s, device=args.device,
                                 epochs=args.epochs, patience=args.patience,
                                 batch_size=args.batch_size, lr=args.lr, log=log)
        members.append(net)
        member_logs.append(info)

    ensemble = DetectorEnsemble(members).eval()
    ensemble.temperature = fit_temperature(ensemble, val_eps, args.device)
    log(f"temperature {ensemble.temperature}")
    val_traces = [predict_episode(ensemble, e.features) for e in val_eps]
    threshold = threshold_at_healthy_quantile(val_eps, val_traces)
    log(f"alarm threshold (val healthy-tick quantile) {threshold:.4f}")

    ensemble.cpu()
    save_checkpoint(out / "detector.pt", ensemble, seeds=seeds, split_digest=split.digest(),
                    threshold=threshold,
                    extra={"run_dir": str(args.run_dir), "split_seed": SPLIT_SEED,
                           "members": member_logs, "args": vars(args)})
    (out / "train_log.json").write_text(json.dumps(
        {"split_digest": split.digest(), "temperature": ensemble.temperature,
         "threshold": threshold, "members": member_logs, "category_counts": counts,
         "wall_s": time.time() - t0}, indent=2))
    log(f"saved {out / 'detector.pt'} ({time.time() - t0:.0f}s total)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
