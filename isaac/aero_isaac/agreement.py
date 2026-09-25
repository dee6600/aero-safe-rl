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

M9 adds the reward-ranking gate, run before any training:

    python -m aero_isaac.agreement --ranking --per-severity 32 --out ../results/m9_reward_ranking_v2.json

Three scripted policies fly the same severity grid with the fitted simulated
detector: always nominal; nominal until the detector's probability first
reaches 0.5, then slow and low; nominal until then, then land. The reward
(each drone's first episode, discounted from take-off with the training
discount) must rank them the way their outcomes do: nominal best at
severities 0.2 and 0.3, where the mission finishes unaided, and a reaction
best at 0.40 and 0.45, where slowing down and getting low turns falls into
landings. If it does not, the reward is wrong, and it is fixed before any
training (milestones.md M9 task 2).

And the Isaac half of the transfer table (M9 task 5):

    python -m aero_isaac.agreement --checkpoint ../results/m9_train/seed_1/policy.pt \
        --per-severity 256 --out ../results/m9_train/seed_1/isaac_scores.json

flies an exported policy -- the same file the PX4 side loads -- on the PX4
check's severity grid, with the fitted simulated detector, no training
randomisation, and the PX4 schedule's fault timing (onset 5-25 s, ramps
1-5 s, half sudden). The mean action, no sampling, as on the PX4 side.
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
# The reward-ranking gate's scripted policies: None = always nominal, else the
# action taken from the first detection on.
REACTIONS = {"nominal": None, "react_slow_low": (0.3, -3.0, 0.0), "react_land": (1.0, 0.0, 1.0)}
NOMINAL_BEST_AT = (0.2, 0.3)
REACTION_BEST_AT = (0.40, 0.45)


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


def fly_ranking(per_cell: int, seed: int) -> list[dict]:
    """Every (severity, scripted policy) cell, per_cell drones each; one
    record per drone's first episode, with its discounted return."""
    from isaaclab.app import AppLauncher
    AppLauncher(headless=True)
    import torch
    from aero_isaac.env import AeroEnv, AeroEnvCfg
    names = list(REACTIONS)
    cells = [(s, name) for s in SEVERITIES for name in names for _ in range(per_cell)]
    cfg = AeroEnvCfg()
    cfg.scene.num_envs = len(cells)
    cfg.fixed_severities = tuple(s for s, _ in cells)
    cfg.seed = seed
    env = AeroEnv(cfg)                                          # the fitted simulated detector, no randomisation
    env.reset()
    dev, n = env.device, len(cells)
    nominal = torch.tensor([POLICIES["nominal"]], device=dev).repeat(n, 1)
    react = torch.tensor([REACTIONS[name] or POLICIES["nominal"] for _, name in cells], device=dev)
    reactive = torch.tensor([REACTIONS[name] is not None for _, name in cells], device=dev)
    reacted = torch.zeros(n, dtype=torch.bool, device=dev)
    ret, disc = torch.zeros(n, device=dev), 1.0
    finished = torch.zeros(n, dtype=torch.bool, device=dev)
    first: dict[int, dict] = {}
    for _ in range(900):
        reacted |= reactive & (env._det["p_fault"] >= 0.5)         # what the policy observes
        _, rew, terminated, truncated, _ = env.step(torch.where(reacted[:, None], react, nominal))
        ret += torch.where(finished, torch.zeros_like(rew), disc * rew)
        disc *= env.reward.discount
        finished |= terminated | truncated
        reacted &= ~(terminated | truncated)
        for e in env.episode_log:
            first.setdefault(e["env"], e)
        env.episode_log.clear()
        if bool(finished.all()):
            break
    ret = ret.tolist()
    return [dict(first[i], policy=cells[i][1], discounted_return=ret[i]) for i in sorted(first)]


def fly_checkpoint(path: str, per_severity: int, seed: int) -> tuple[list[dict], dict]:
    from isaaclab.app import AppLauncher
    AppLauncher(headless=True)
    import torch
    from aero_isaac.env import AeroEnv, AeroEnvCfg
    from aero_isaac.train import run_exported
    policy = torch.load(path, map_location="cpu", weights_only=True)
    sev = [s for s in SEVERITIES for _ in range(per_severity)]
    cfg = AeroEnvCfg()
    cfg.scene.num_envs = len(sev)
    cfg.fixed_severities = tuple(sev)
    cfg.seed = seed
    env = AeroEnv(cfg)
    dev = env.device
    on_dev = dict(policy, layers=[{k: v.to(dev) for k, v in layer.items()} for layer in policy["layers"]])
    obs, _ = env.reset()
    first: dict[int, dict] = {}
    for _ in range(900):
        with torch.no_grad():
            obs, *_ = env.step(run_exported(on_dev, obs["policy"]))
        for e in env.episode_log:
            first.setdefault(e["env"], e)
        env.episode_log.clear()
        if len(first) == len(sev):
            break
    meta = {k: policy.get(k) for k in ("seed", "update", "code_commit", "fingerprints")}
    return [first[i] for i in sorted(first)], meta


def rank(episodes: list[dict]) -> dict:
    """Per (severity, policy): mean discounted return and outcome shares;
    then the gate."""
    table: dict[str, dict] = {}
    for sev in sorted({round(e["severity"], 2) for e in episodes}):
        row = {}
        for name in REACTIONS:
            g = [e for e in episodes if round(e["severity"], 2) == sev and e["policy"] == name]
            row[name] = dict(n=len(g), mean_return=statistics.fmean(e["discounted_return"] for e in g),
                             **{o: sum(e["outcome"] == i for e in g) / len(g) for i, o in enumerate(OUTCOMES)})
        table[f"{sev:.2f}"] = row
    checks = []
    for s in NOMINAL_BEST_AT:
        row = table[f"{s:.2f}"]
        best_reaction = max(row[k]["mean_return"] for k in REACTIONS if REACTIONS[k] is not None)
        checks.append(dict(severity=s, expect="nominal best", passed=row["nominal"]["mean_return"] > best_reaction))
    for s in REACTION_BEST_AT:
        row = table[f"{s:.2f}"]
        best_reaction = max(row[k]["mean_return"] for k in REACTIONS if REACTIONS[k] is not None)
        checks.append(dict(severity=s, expect="a reaction best", passed=best_reaction > row["nominal"]["mean_return"]))
    return dict(table=table, checks=checks, passed=all(c["passed"] for c in checks))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--policy", choices=sorted(POLICIES))
    mode.add_argument("--ranking", action="store_true", help="the reward-ranking gate (M9)")
    mode.add_argument("--checkpoint", help="score an exported policy.pt (M9 transfer table, Isaac half)")
    ap.add_argument("--per-severity", type=int, default=16)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--px4", default=None, help="run_recovery.py report --json output for the same policy")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    if args.ranking:
        episodes = fly_ranking(args.per_severity, args.seed)
        from aero_isaac.contracts import load_train_config
        train = load_train_config()
        result = dict(reactions=REACTIONS, reward=train["reward"], reward_fingerprint=train["reward_spec"].digest,
                      discount=train["algorithm"]["discount"], **rank(episodes), episodes=episodes)
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(result, indent=2))
        for sev, row in result["table"].items():
            print(f"s={sev} " + "  ".join(
                f"{k}: R={v['mean_return']:+.2f} ok/land/crash={v['mission_success']:.2f}/{v['safe_landing']:.2f}/"
                f"{v['crash']:.2f}" for k, v in row.items()), flush=True)
        for c in result["checks"]:
            print(f"{'PASS' if c['passed'] else 'FAIL'}  s={c['severity']:.2f}: {c['expect']}", flush=True)
        sys.stdout.flush()
        os._exit(0)
    if args.checkpoint:
        episodes, meta = fly_checkpoint(args.checkpoint, args.per_severity, args.seed)
        summary = summarize(episodes)
        result = dict(checkpoint=args.checkpoint, policy=meta, simulator="isaac", summary=summary,
                      episodes=episodes)
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(result, indent=2))
        for k, c in summary["by_severity"].items():
            print(f"s={k} n={c['n']} " + " ".join(f"{o}={c[o]:.2f}" for o in OUTCOMES)
                  + f" touch={c['median_touchdown_speed_m_s']:.2f} dur={c['median_duration_s']:.1f}", flush=True)
        sys.stdout.flush()
        os._exit(0)
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
