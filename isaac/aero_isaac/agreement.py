"""M8b task 5: does the Isaac environment fly like the PX4 stack?

Flies a severity grid under one scripted policy (the same constant action the
PX4 side flew), computes the measures the PX4 side's
experiments/run_recovery.py report computes, and checks the gates written
into milestones.md M8b before anything was measured. The PX4 numbers arrive
as a summary file written by that report -- a file exchange, not an import
(CLAUDE.md §0.1).

    source scripts/activate_isaac.sh
    python -m aero_isaac.agreement --policy nominal \\
        --px4 results/m8b_agreement/px4_nominal.json --out results/m8b_agreement/isaac_nominal.json

Run from isaac/. The PX4 side is never adjusted to match; a failed gate is a
finding (milestones.md M8b, "Risk").
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
from pathlib import Path

POLICIES = {"nominal": (1.0, 0.0, 0.0), "slow_low": (0.3, -3.0, 0.0)}
SEVERITIES = (0.0, 0.2, 0.3, 0.35, 0.4, 0.45, 0.5, 0.7)
OUTCOMES = ("mission_success", "safe_landing", "crash", "incomplete")
TOUCHDOWN_SEVERITIES = (0.40, 0.45, 0.50, 0.70)
GATES = dict(duration_rel=0.10, peak_speed_rel=0.15, motor_abs=0.05, crash50_abs=0.05, touchdown_abs=0.6)


def _median(xs):
    xs = [x for x in xs if x is not None and not math.isnan(x)]
    return statistics.median(xs) if xs else float("nan")


def summarize(episodes: list[dict]) -> dict:
    """Same cells and fields as run_recovery.summarize()."""
    out = {"n_valid": len(episodes), "by_severity": {}}
    for sev in sorted({round(e["severity"], 2) for e in episodes}):
        g = [e for e in episodes if round(e["severity"], 2) == sev]
        cell = {"n": len(g)}
        for i, name in enumerate(OUTCOMES):
            cell[name] = sum(e["outcome"] == i for e in g) / len(g)
        cell["median_duration_s"] = _median([e["duration_s"] for e in g])
        cell["median_touchdown_speed_m_s"] = _median([e["touchdown_speed_m_s"] for e in g])
        cell["median_peak_hspeed_m_s"] = _median([e["peak_hspeed_m_s"] for e in g])
        cell["median_mean_motor_command"] = _median([e["mean_motor_command"] for e in g])
        out["by_severity"][f"{sev:.2f}"] = cell
    return out


def crash50(summary: dict) -> float:
    """Severity where the crash rate first crosses 50%, linearly interpolated."""
    pts = sorted((float(k), c["crash"]) for k, c in summary["by_severity"].items())
    for (s0, c0), (s1, c1) in zip(pts, pts[1:]):
        if c0 < 0.5 <= c1:
            return s0 + (0.5 - c0) / (c1 - c0) * (s1 - s0)
    return float("nan")


def compare(isaac: dict, px4: dict) -> list[dict]:
    """The gates (milestones.md M8b task 5)."""
    rows = []

    def add(name, a, b, ok):
        rows.append(dict(measure=name, isaac=a, px4=b, passed=bool(ok)))

    hi, hp = isaac["by_severity"].get("0.00"), px4["by_severity"].get("0.00")
    if hi and hp:
        a, b = hi["median_duration_s"], hp["median_duration_s"]
        add("healthy mission time (s)", a, b, abs(a - b) <= GATES["duration_rel"] * b)
        if "median_peak_hspeed_m_s" in hp:
            a, b = hi["median_peak_hspeed_m_s"], hp["median_peak_hspeed_m_s"]
            add("healthy peak speed (m/s)", a, b, abs(a - b) <= GATES["peak_speed_rel"] * b)
        if "median_mean_motor_command" in hp:
            a, b = hi["median_mean_motor_command"], hp["median_mean_motor_command"]
            add("healthy mean motor command", a, b, abs(a - b) <= GATES["motor_abs"])
    a, b = crash50(isaac), crash50(px4)
    if not (math.isnan(a) and math.isnan(b)):
        add("severity where crash rate crosses 50%", a, b,
            not math.isnan(a) and not math.isnan(b) and abs(a - b) <= GATES["crash50_abs"])
    for s in TOUCHDOWN_SEVERITIES:
        ci, cp = isaac["by_severity"].get(f"{s:.2f}"), px4["by_severity"].get(f"{s:.2f}")
        if ci and cp:
            a, b = ci["median_touchdown_speed_m_s"], cp["median_touchdown_speed_m_s"]
            add(f"median touchdown speed at s = {s:.2f} (m/s)", a, b, abs(a - b) <= GATES["touchdown_abs"])
    return rows


def fly(policy: str, per_severity: int, seed: int) -> list[dict]:
    from isaaclab.app import AppLauncher
    AppLauncher(headless=True)
    import torch
    from aero_isaac.env import AeroEnv, AeroEnvCfg
    sev = [s for s in SEVERITIES for _ in range(per_severity)]
    cfg = AeroEnvCfg()
    cfg.scene.num_envs = len(sev)
    cfg.fixed_severities = tuple(sev)
    cfg.seed = seed
    cfg.detector = "none"   # scripted policies ignore it; keeps the run about the vehicle
    env = AeroEnv(cfg)
    env.reset()
    act = torch.tensor([POLICIES[policy]], device=env.device).repeat(len(sev), 1)
    first: dict[int, dict] = {}
    steps = 0
    while len(first) < len(sev) and steps < 900:
        env.step(act)
        steps += 1
        for e in env.episode_log:
            first.setdefault(e["env"], e)
        env.episode_log.clear()
    return [first[i] for i in sorted(first)]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", choices=sorted(POLICIES), required=True)
    ap.add_argument("--per-severity", type=int, default=16)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--px4", default=None, help="run_recovery.py report --json output for the same policy")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    episodes = fly(args.policy, args.per_severity, args.seed)
    summary = summarize(episodes)
    result = dict(policy=args.policy, action=POLICIES[args.policy], gates=GATES, summary=summary,
                  episodes=episodes)
    if args.px4:
        result["px4_source"] = args.px4
        result["comparison"] = compare(summary, json.loads(Path(args.px4).read_text()))
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(result, indent=2))
    for k, c in summary["by_severity"].items():
        print(f"s={k} n={c['n']} " + " ".join(f"{o}={c[o]:.2f}" for o in OUTCOMES)
              + f" touch={c['median_touchdown_speed_m_s']:.2f} dur={c['median_duration_s']:.1f}"
              + f" peak={c['median_peak_hspeed_m_s']:.2f} motor={c['median_mean_motor_command']:.3f}", flush=True)
    for r in result.get("comparison", []):
        print(f"{'PASS' if r['passed'] else 'FAIL'}  {r['measure']}: isaac {r['isaac']:.3f}  px4 {r['px4']:.3f}",
              flush=True)
    sys.stdout.flush()
    os._exit(0)   # Isaac Sim's shutdown never returns headless


if __name__ == "__main__":
    main()
