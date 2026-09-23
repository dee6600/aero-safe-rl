#!/usr/bin/env python3
"""M7 task 4: score the trained detector and the three baselines on the held-
out TEST episodes, and write the error model M8b's detector-output simulator
is fitted to.

Protocol (experiments/metrics.py): baselines are fitted on train; every
detector's alarm threshold is the same healthy-tick quantile of its own
score on val (the model's was fixed at training time by the same rule); the
test split is read here, once, and nothing is tuned on it.

Writes, under --out:
  metrics.json       every metric, every detector, overall and per group
  error_model.json   the model's measured error characteristics (M8b input)
  report.md          the tables docs/detector_results.md is written from

Usage:
    python ai/detector/evaluate.py results/m6_dataset_v1 \
        --checkpoint results/m7_detector_v1/detector.pt --out results/m7_detector_v1
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parent.parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from ai.detector.baselines import (  # noqa: E402
    MotorImbalanceDetector, RandomForestDetector, ResidualThresholdDetector,
)
from ai.detector.dataset import SEVERITY_BUCKETS, load_episodes, split_by_episode, stratum  # noqa: E402
from ai.detector.model import file_digest, load_checkpoint, predict_episode  # noqa: E402
from ai.detector.runtime import DetectorRuntime  # noqa: E402
from ai.detector.train import SPLIT_SEED  # noqa: E402
from ai.features.feature_extractor import RAW_FRAME_FIELDS  # noqa: E402
from experiments.metrics import (  # noqa: E402
    ALARM_HOLD_TICKS, HEALTHY_QUANTILE, evaluate_detector, threshold_at_healthy_quantile,
)

MODEL_NAME = "rotor_gru_ensemble"
BUCKETS = [f"sev_{lo:.1f}_{hi:.1f}" for lo, hi in SEVERITY_BUCKETS]
QUANTILES = (0.05, 0.25, 0.5, 0.75, 0.95)


def _quantiles(x: np.ndarray) -> dict:
    return {str(q): float(np.quantile(x, q)) for q in QUANTILES} if len(x) else {}


def measure_latency(ckpt_path: Path, run_dir: Path, keys: list[str]) -> dict:
    """Wall-clock per-tick cost of DetectorRuntime.step() on recorded test
    episodes (feature extraction + ensemble), CPU, one torch thread."""
    rt = DetectorRuntime(ckpt_path)
    times = []
    for key in keys:
        worker, ep = key.split("/")
        steps = pd.read_parquet(run_dir / f"worker_{worker}" / f"episode_{ep}_steps.parquet")
        rt.reset()
        for frame in steps[list(RAW_FRAME_FIELDS)].to_dict("records"):
            t0 = time.perf_counter()
            rt.step(frame)
            times.append(time.perf_counter() - t0)
    t = np.array(times[50:]) * 1e3  # drop warm-up
    return {"n_ticks": int(len(t)), "median_ms": float(np.median(t)),
            "p99_ms": float(np.quantile(t, 0.99)), "max_ms": float(t.max())}


def error_model(test_eps, traces, result: dict, meta: dict, ckpt_path: Path) -> dict:
    """What M8b needs to simulate this detector: detection delay and miss
    rate per severity, false-alarm rate, p_fault distributions, severity
    error, rotor-ID accuracy, uncertainty distribution -- all on test."""
    healthy_p = np.concatenate([t.score[~e.fault_active] for e, t in zip(test_eps, traces)])
    per_bucket = {}
    for b in BUCKETS:
        sel = [(e, t) for e, t in zip(test_eps, traces)
               if stratum(e) == b]
        if not sel:
            continue
        g = result["groups"][b]
        per_bucket[b] = {
            "n_episodes": g["n_episodes"],
            "detection_rate": g["detection_rate"],
            "delays_s": result["delays_s"][b],
            "rotor_accuracy": g["rotor_accuracy"],
            "severity_bias": g["severity_bias"],
            "severity_err_std": g["severity_err_std"],
            "active_tick_p_fault_quantiles": _quantiles(
                np.concatenate([t.score[e.fault_active] for e, t in sel])),
            "active_tick_uncertainty_quantiles": _quantiles(
                np.concatenate([t.uncertainty[e.fault_active] for e, t in sel])),
        }
    return {
        "detector": MODEL_NAME,
        "checkpoint_digest": file_digest(ckpt_path),
        "split_digest": meta["split_digest"],
        "evaluated_on": "test split",
        "tick_rate_hz": 10.0,
        "threshold": result["threshold"],
        "hold_ticks": result["hold_ticks"],
        "false_alarms_per_healthy_hour": result["false_alarms_per_healthy_hour"],
        "healthy_episodes_with_any_alarm": result["healthy_episodes_with_any_alarm"],
        "healthy_tick_p_fault_quantiles": _quantiles(healthy_p),
        "healthy_tick_uncertainty_quantiles": _quantiles(np.concatenate(
            [t.uncertainty[~e.fault_active] for e, t in zip(test_eps, traces)])),
        "per_severity_bucket": per_bucket,
    }


def _fmt(v, pct=False, digits=2) -> str:
    if v is None:
        return "—"
    return f"{100 * v:.1f}%" if pct else f"{v:.{digits}f}"


def report_md(results: dict) -> str:
    names = list(results)
    lines = []
    head = "| detector | " + " | ".join(BUCKETS + ["step", "ramp"]) + " |"
    sep = "|---" * (len(BUCKETS) + 3) + "|"

    def table(title, key, **fmt):
        lines.extend([f"### {title}", "", head, sep])
        for n in names:
            g = results[n]["groups"]
            cells = [_fmt(g.get(b, {}).get(key), **fmt)
                     for b in BUCKETS + ["profile_step", "profile_ramp"]]
            lines.append(f"| {n} | " + " | ".join(cells) + " |")
        lines.append("")

    lines += ["### Overall", "",
              "| detector | tick AUROC | false alarms / healthy h | healthy flights with any alarm "
              "| ECE | uncertainty→error AUROC | threshold |",
              "|---|---|---|---|---|---|---|"]
    for n in names:
        r = results[n]
        lines.append(f"| {n} | {_fmt(r['auroc_tick'], digits=4)} | "
                     f"{_fmt(r['false_alarms_per_healthy_hour'], digits=1)} | "
                     f"{_fmt(r['healthy_episodes_with_any_alarm'], pct=True)} | "
                     f"{_fmt(r.get('ece'), digits=3)} | {_fmt(r.get('uncertainty_error_auroc'), digits=3)} | "
                     f"{_fmt(r['threshold'], digits=4)} |")
    lines.append("")
    table("Detection rate (episodes alarmed after onset)", "detection_rate", pct=True)
    table("Detection delay, median (s)", "delay_s_p50")
    table("Detection delay, 90th percentile (s)", "delay_s_p90")
    table("Tick AUROC (fault-active ticks of the group vs all healthy ticks)", "auroc_tick", digits=4)
    table("Rotor-ID accuracy (all fault-active ticks)", "rotor_accuracy", pct=True)
    table("Severity MAE (all fault-active ticks)", "severity_mae", digits=3)
    table("Severity bias (predicted − true)", "severity_bias", digits=3)
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--checkpoint", default=str(REPO / "results" / "m7_detector_v1" / "detector.pt"))
    ap.add_argument("--out", default=str(REPO / "results" / "m7_detector_v1"))
    args = ap.parse_args(argv)
    run_dir, out, ckpt_path = Path(args.run_dir), Path(args.out), Path(args.checkpoint)
    out.mkdir(parents=True, exist_ok=True)

    episodes, counts = load_episodes(run_dir)
    split = split_by_episode(episodes, seed=SPLIT_SEED)
    ensemble, meta = load_checkpoint(ckpt_path)
    if meta["split_digest"] != split.digest():
        raise SystemExit(f"checkpoint was trained on split {meta['split_digest']}, "
                         f"this dataset gives {split.digest()} -- refusing to evaluate")
    train_eps, val_eps, test_eps = (split.select(episodes, p) for p in ("train", "val", "test"))
    print(f"train {len(train_eps)}  val {len(val_eps)}  test {len(test_eps)}  split {split.digest()}")

    results, test_traces = {}, {}
    for det in (MotorImbalanceDetector(), ResidualThresholdDetector(), RandomForestDetector(seed=0)):
        t0 = time.time()
        det.fit(train_eps)
        thr = threshold_at_healthy_quantile(val_eps, [det.predict(e.features) for e in val_eps])
        test_traces[det.name] = [det.predict(e.features) for e in test_eps]
        results[det.name] = evaluate_detector(test_eps, test_traces[det.name], thr)
        print(f"{det.name}: fitted + scored in {time.time() - t0:.0f}s, threshold {thr:.4f}")

    test_traces[MODEL_NAME] = [predict_episode(ensemble, e.features) for e in test_eps]
    results[MODEL_NAME] = evaluate_detector(test_eps, test_traces[MODEL_NAME], meta["threshold"])

    latency = measure_latency(ckpt_path, run_dir, list(split.test[:10]))
    print(f"runtime latency: {latency}")

    protocol = {"split_digest": split.digest(), "split_seed": SPLIT_SEED,
                "healthy_quantile": HEALTHY_QUANTILE, "alarm_hold_ticks": ALARM_HOLD_TICKS,
                "category_counts": counts, "checkpoint_digest": file_digest(ckpt_path),
                "runtime_latency": latency}
    (out / "metrics.json").write_text(json.dumps({"protocol": protocol, "detectors": results},
                                                 indent=2))
    (out / "error_model.json").write_text(json.dumps(
        error_model(test_eps, test_traces[MODEL_NAME], results[MODEL_NAME], meta, ckpt_path),
        indent=2))
    md = report_md(results)
    (out / "report.md").write_text(md + "\n")
    print(md)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
