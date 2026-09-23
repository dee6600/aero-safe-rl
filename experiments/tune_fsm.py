"""M8 tasks 4-5: tuning the rule-based FSM, by rules fixed before the data.

    # task 4 -- detection side, offline, on M7's VALIDATION episodes only
    python experiments/tune_fsm.py detection results/m6_dataset_v1 --out results/m8_fsm_tuning

    # task 5 -- response side: write the candidate configs, fly each with
    # run_recovery.py fly (run id m8_fsm_sweep_<name>), then pick by the rule
    python experiments/tune_fsm.py candidates
    python experiments/tune_fsm.py responses

Detection side: the detector is replayed over recorded flights (batch
inference, identical to streaming -- tests/test_detector_runtime.py) and the
real RuleBasedPolicy is stepped at 5 Hz on its output. Caveat, stated in the
report: once the FSM acts on a suspicion it changes the flight, which replay
cannot show; the detection thresholds only decide *when* it acts.
The selection rules are written in configs/rl/fsm_v1.yaml.
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
from dataclasses import replace
from pathlib import Path
from typing import Sequence

import numpy as np

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

FSM_PATH = REPO / "configs" / "rl" / "fsm_v1.yaml"
CHECKPOINT = REPO / "results" / "m7_detector_v1" / "detector.pt"
DECISION_STRIDE = 2               # 10 Hz ticks -> 5 Hz decisions
CONFIRM_HOLDS_S = (0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0)
SUSPECT_PS = (0.097, 0.3, 0.5)
TUNE_BAND = (0.3, 0.5)            # severities the delay criterion is judged on


def replay_fsm(policy, elapsed_s: np.ndarray, trace) -> tuple[list[float], float | None]:
    """Steps `policy` at 5 Hz over one episode's detector trace. Returns
    (times it entered SUSPECTED from NORMAL, time it confirmed or None)."""
    from ai.detector.runtime import DetectorOutput
    from rl.policies.base_policy import MissionProgress, PolicyInput
    from rl.policies.rule_based import FsmState
    policy.reset()
    suspicions, prev = [], FsmState.NORMAL
    progress = MissionProgress(0, 5, 0.0, 5.0, 0.0)
    for i in range(0, len(elapsed_s), DECISION_STRIDE):
        det = DetectorOutput(p_fault=float(trace.score[i]), rotor=int(trace.rotor[i]),
                             severity=float(trace.severity[i]),
                             uncertainty=float(trace.uncertainty[i]), alarm=False)
        policy.act(PolicyInput(t_sim_s=float(elapsed_s[i]), features={}, detector=det,
                               mission=progress))
        if prev == FsmState.NORMAL and policy.state == FsmState.SUSPECTED:
            suspicions.append(float(elapsed_s[i]))
        prev = policy.state
        if policy.confirmed_at_s is not None:
            return suspicions, policy.confirmed_at_s
    return suspicions, None


def score_detection(episodes, traces, config) -> dict:
    """False confirmations/suspicions on healthy time, and confirmation
    delay on faulty episodes, for one FsmConfig."""
    from ai.detector.dataset import EpisodeCategory
    from rl.policies.rule_based import RuleBasedPolicy
    policy = RuleBasedPolicy(config)
    false_conf, false_susp, healthy_s = 0, 0, 0.0
    delays, band_delays, missed = [], [], 0
    for e, tr in zip(episodes, traces):
        suspicions, confirmed = replay_fsm(policy, e.elapsed_s, tr)
        active = np.flatnonzero(e.fault_active)
        onset = float(e.elapsed_s[active[0]]) if len(active) else None
        healthy_end = onset if onset is not None else float(e.elapsed_s[-1])
        healthy_s += healthy_end - float(e.elapsed_s[0])
        false_susp += sum(t < healthy_end for t in suspicions)
        if confirmed is not None and (onset is None or confirmed < onset):
            false_conf += 1
            continue
        if e.category == EpisodeCategory.FAULT_APPLIED and onset is not None:
            if confirmed is None:
                missed += 1
                continue
            delays.append(confirmed - onset)
            if TUNE_BAND[0] <= e.severity_commanded <= TUNE_BAND[1]:
                band_delays.append(confirmed - onset)
    n_faulty = len(delays) + missed
    return dict(
        suspect_p=config.suspect_p, confirm_hold_s=config.confirm_hold_s,
        false_confirmations=false_conf,
        false_suspicions_per_healthy_hour=false_susp / (healthy_s / 3600) if healthy_s else None,
        confirm_rate=len(delays) / n_faulty if n_faulty else None,
        delay_s_p50=float(np.median(delays)) if delays else None,
        band_delay_s_p50=float(np.median(band_delays)) if band_delays else None,
        band_delay_s_p90=float(np.quantile(band_delays, 0.9)) if band_delays else None,
        n_band=len(band_delays))


def healthy_runs_s(p_fault: np.ndarray, t_s: np.ndarray, threshold: float) -> list[float]:
    """Durations of the runs of p_fault >= threshold as the FSM sees them
    (5 Hz decisions): first to last decision of each run, so a single-decision
    spike is 0 s. The FSM confirms when a run's duration reaches its hold."""
    out, start, last = [], None, None
    for i in range(0, len(p_fault), DECISION_STRIDE):
        if p_fault[i] >= threshold:
            start = t_s[i] if start is None else start
            last = t_s[i]
        elif start is not None:
            out.append(float(last - start))
            start = None
    if start is not None:
        out.append(float(last - start))
    return out


def pick_detection(rows: Sequence[dict], longest_healthy_run_s: dict[float, float],
                   margin_s: float = 0.2) -> dict:
    """fsm_v1.yaml's rule (revised 2026-09-23, see there): zero false
    confirmations on validation, AND a hold at least `margin_s` (one
    decision) longer than the longest healthy run above suspect_p seen in
    any non-test healthy data; then the smallest median band delay; ties
    towards the longer hold."""
    ok = [r for r in rows if r["false_confirmations"] == 0 and r["band_delay_s_p50"] is not None
          and r["confirm_hold_s"] >= longest_healthy_run_s[r["suspect_p"]] + margin_s - 1e-9]
    if not ok:
        raise SystemExit("no detection setting clears the healthy false-alarm runs")
    return min(ok, key=lambda r: (round(r["band_delay_s_p50"], 3), -r["confirm_hold_s"]))


def _longest_runs(parts: dict[str, list[tuple[np.ndarray, np.ndarray]]]) -> dict:
    """{suspect_p: {"longest_s": ..., "<part>": longest in that part}} over
    healthy (p_fault, t) series."""
    out = {}
    for p in SUSPECT_PS:
        per = {name: max((max(healthy_runs_s(sc, t, p), default=0.0) for sc, t in series),
                         default=0.0) for name, series in parts.items()}
        out[p] = dict(per, longest_s=max(per.values()))
    return out


def live_healthy_series(run_dir: Path) -> list[tuple[np.ndarray, np.ndarray]]:
    """Mission-phase (p_fault, t) of every healthy episode of a run flown
    with the detector attached (run_recovery.py fly --detector ...)."""
    import pandas as pd
    series = []
    for sp in sorted(run_dir.glob("worker_*/episode_*_summary.parquet")):
        s = pd.read_parquet(sp).iloc[0]
        if s.fault_applied or not s.valid or s.n_steps == 0:
            continue
        st = pd.read_parquet(sp.with_name(sp.name.replace("_summary", "_steps")))
        m = st[st.flight_phase == "mission"]
        series.append((m.det_p_fault.to_numpy(float), m.t_sim_s.to_numpy(float)))
    return series


def detection(args) -> None:
    import torch

    from ai.detector.dataset import load_episodes, split_by_episode
    from ai.detector.model import load_checkpoint, predict_episode
    from ai.detector.train import SPLIT_SEED
    from rl.policies.rule_based import load_fsm_config
    torch.set_num_threads(2)

    episodes, _ = load_episodes(args.run_dir)
    split = split_by_episode(episodes, seed=SPLIT_SEED)
    ensemble, meta = load_checkpoint(CHECKPOINT)
    if meta["split_digest"] != split.digest():
        raise SystemExit("checkpoint split does not match this dataset")
    val = split.select(episodes, "val")
    traces = [predict_episode(ensemble, e.features) for e in val]

    def healthy_part(eps, trs):
        series = []
        for e, tr in zip(eps, trs):
            active = np.flatnonzero(e.fault_active)
            end = active[0] if len(active) else len(e.elapsed_s)
            series.append((tr.score[:end], e.elapsed_s[:end]))
        return series

    train = split.select(episodes, "train")
    parts = {"val": healthy_part(val, traces),
             "train": healthy_part(train, [predict_episode(ensemble, e.features) for e in train])}
    for live in args.live_healthy:
        parts[str(live)] = live_healthy_series(Path(live))
    longest = _longest_runs(parts)

    base = load_fsm_config(FSM_PATH)
    rows = [score_detection(val, traces, replace(base, suspect_p=p, confirm_hold_s=h))
            for p, h in itertools.product(SUSPECT_PS, CONFIRM_HOLDS_S)]
    best = pick_detection(rows, {p: v["longest_s"] for p, v in longest.items()})
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "detection_sweep.json").write_text(json.dumps(
        dict(split_digest=split.digest(), n_val_episodes=len(val), rule="fsm_v1.yaml detection",
             longest_healthy_run_s={str(k): v for k, v in longest.items()},
             grid=rows, chosen=best), indent=2))
    for p, v in longest.items():
        print(f"longest healthy run above {p}: {v}")
    print(f"{len(val)} validation episodes, split {split.digest()}")
    print(f"{'suspect_p':>9} {'hold_s':>6} {'false_conf':>10} {'susp/h':>7} {'conf_rate':>9} "
          f"{'band_p50':>8} {'band_p90':>8}")
    for r in rows:
        print(f"{r['suspect_p']:>9.3f} {r['confirm_hold_s']:>6.1f} {r['false_confirmations']:>10} "
              f"{r['false_suspicions_per_healthy_hour']:>7.1f} {r['confirm_rate']:>9.3f} "
              f"{r['band_delay_s_p50']:>8.2f} {r['band_delay_s_p90']:>8.2f}")
    print(f"chosen: suspect_p={best['suspect_p']} confirm_hold_s={best['confirm_hold_s']}")


# Task 5: one-at-a-time variations around fsm_v1.yaml's responses. Each is
# written as its own config file so every sweep episode records the digest of
# exactly what it flew.
RESPONSE_CANDIDATES = {
    "base":            {},
    "degraded_high":   {"degraded": {"altitude_offset_m": 0.0}},
    "degraded_fast":   {"degraded": {"speed_scale": 0.6}},
    "land_0.35":       {"land_severity": 0.35},
    "land_0.45":       {"land_severity": 0.45},
    "suspect_descend": {"suspected": {"altitude_offset_m": -3.0}},
}
SWEEP_SEVERITIES = (0.35, 0.40, 0.45, 0.50)


def write_candidates(out_dir: Path) -> list[Path]:
    import yaml
    base = yaml.safe_load(FSM_PATH.read_text())
    paths = []
    for name, override in RESPONSE_CANDIDATES.items():
        cfg = json.loads(json.dumps(base))
        for k, v in override.items():
            if isinstance(v, dict):
                cfg["responses"][k].update(v)
            else:
                cfg["responses"][k] = v
        path = Path(out_dir) / f"fsm_{name}.yaml"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"# M8 task 5 sweep candidate '{name}': fsm_v1.yaml with {override}\n"
                        + yaml.safe_dump(cfg, sort_keys=False))
        paths.append(path)
    return paths


def pick_response(results: dict[str, dict]) -> str:
    """fsm_v1.yaml's rule: lowest crash rate over SWEEP_SEVERITIES; ties by
    mission success, then the simpler setting (base, then candidate order).
    `results` maps candidate name -> run_recovery.summarize() output."""
    order = list(RESPONSE_CANDIDATES)

    def key(name):
        cells = [results[name]["by_severity"][f"{s:.2f}"] for s in SWEEP_SEVERITIES]
        n = sum(c["n"] for c in cells)
        crash = sum(c["crash"] * c["n"] for c in cells) / n
        success = sum(c["mission_success"] * c["n"] for c in cells) / n
        return (round(crash, 6), -round(success, 6), order.index(name))
    return min(results, key=key)


def responses(args) -> None:
    import pandas as pd

    from experiments.run_recovery import episode_rows, report_text, summarize
    results = {}
    for name in RESPONSE_CANDIDATES:
        # The sweep run, pooled with its tie-break run when one was flown
        # (more flights for the leading candidates; same rule).
        dirs = [Path(args.sweep_root) / f"m8_fsm_{kind}_{name}" for kind in ("sweep", "tiebreak")]
        dirs = [d for d in dirs if d.exists()]
        if not dirs:
            print(f"no runs for {name}, skipped")
            continue
        rows = pd.concat([episode_rows(d) for d in dirs], ignore_index=True)
        results[name] = summarize("+".join(d.name for d in dirs), rows)
        print(f"== {name}\n{report_text(results[name])}")
    nominal = Path(args.sweep_root) / "m8_fsm_sweep_nominal"
    ref = summarize(nominal) if nominal.exists() else None
    if ref:
        print(f"== nominal (no recovery, reference)\n{report_text(ref)}")
    best = pick_response(results)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "response_sweep.json").write_text(json.dumps(
        dict(rule="fsm_v1.yaml responses", severities=SWEEP_SEVERITIES,
             candidates=RESPONSE_CANDIDATES, results=results, nominal_reference=ref,
             chosen=best), indent=2))
    print(f"chosen: {best}")


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("detection")
    d.add_argument("run_dir")
    d.add_argument("--out", default=str(REPO / "results" / "m8_fsm_tuning"))
    d.add_argument("--live-healthy", nargs="*", default=[],
                   help="run dirs whose healthy episodes (flown with the detector) add to the "
                        "false-alarm run lengths")
    c = sub.add_parser("candidates")
    c.add_argument("--out", default=str(REPO / "results" / "m8_fsm_sweep_configs"))
    r = sub.add_parser("responses")
    r.add_argument("--sweep-root", default=str(REPO / "results"))
    r.add_argument("--out", default=str(REPO / "results" / "m8_fsm_tuning"))
    args = ap.parse_args(argv)
    if args.cmd == "detection":
        detection(args)
    elif args.cmd == "candidates":
        for p in write_candidates(Path(args.out)):
            print(p)
    else:
        responses(args)


if __name__ == "__main__":
    main()
