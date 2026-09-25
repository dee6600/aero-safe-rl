"""M8: fly recovery episodes on the PX4 stack and score them.

One command for every M8 simulator run -- the live check of the policy-driven
flight (task 3), the recovery controller's response sweep (task 5) and the validation run
(task 6) -- and, from M10, each condition's cells:

    # fly: N episodes per severity (0 = healthy) under one recovery config
    python experiments/run_recovery.py fly --run-id m8_check --policy nominal \\
        --detector results/m7_detector_v1/detector.pt \\
        --severities 0 0.5 --episodes-per-severity 4 --seed 8001

    # score a finished run (outcome per severity, experiments.metrics)
    python experiments/run_recovery.py report results/m8_check

    # M9: the transfer table -- each learned policy's Isaac score beside its
    # PX4 score, per severity, plus the baselines flown on the same faults
    python experiments/run_recovery.py transfer \
        --policy seed_1 results/m9_px4_seed_1 results/m9_train/seed_1/isaac_scores.json \
        --baseline "no recovery" results/m9_px4_nominal --json results/m9_transfer.json

Faults are single-rotor, step or ramp, with onset 5-25 s into the mission --
early enough that every fault happens while the mission is still flying
(M6's 5-40 s range let ~14% of faults land after the mission had ended).
Severities are fixed per cell rather than sampled, so each cell is a clean
point on the severity axis. The schedule is a pure function of the seed.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from experiments.episode_schema import digest  # noqa: E402
from experiments.fault_schedule import FaultProfile, FaultSpec, FaultType  # noqa: E402

ONSET_RANGE_S = (5.0, 25.0)
RAMP_RANGE_S = (1.0, 5.0)
ROTORS = (0, 1, 2, 3)


def recovery_schedule(severities: Sequence[float], n_per_severity: int,
                      rng: np.random.Generator) -> list[FaultSpec]:
    """n_per_severity episodes per severity (0 = healthy), shuffled so every
    worker's contiguous block covers every cell. Faulty episodes alternate
    step / ramp within each cell."""
    cells = []
    for sev in severities:
        for j in range(n_per_severity):
            cells.append((float(sev), FaultProfile.STEP if j % 2 == 0 else FaultProfile.RAMP))
    order = rng.permutation(len(cells))
    schedule = []
    for i, k in enumerate(order):
        sev, profile = cells[k]
        if sev <= 0.0:
            schedule.append(FaultSpec.healthy(i))
            continue
        schedule.append(FaultSpec(
            episode_index=i, fault_applied=True, fault_type=FaultType.ROTOR_THRUST_DEGRADATION,
            rotor_index=int(rng.choice(ROTORS)), severity=sev,
            onset_time_s=float(rng.uniform(*ONSET_RANGE_S)), profile=profile,
            ramp_duration_s=float(rng.uniform(*RAMP_RANGE_S)) if profile == FaultProfile.RAMP else 0.0))
    return schedule


def fly(args) -> None:
    from experiments.generate_fault_dataset import build_fault_specs_by_worker
    from experiments.sim_farm import SimFarm
    from rl.policy_driver import RecoveryConfig

    recovery = RecoveryConfig(policy=args.policy, policy_config=args.policy_config,
                              detector_checkpoint=args.detector,
                              constant_action=tuple(args.constant_action) if args.constant_action else None)
    n_total = len(args.severities) * args.episodes_per_severity
    per_worker = -(-n_total // args.worker_count)
    schedule = recovery_schedule(args.severities, args.episodes_per_severity,
                                 np.random.default_rng(args.seed))
    schedule += [FaultSpec.healthy(len(schedule) + i)
                 for i in range(per_worker * args.worker_count - len(schedule))]
    schedule_cfg = dict(severities=list(args.severities), n=args.episodes_per_severity,
                        seed=args.seed, onset=ONSET_RANGE_S, ramp=RAMP_RANGE_S)

    print(f"{args.run_id}: {recovery} -- {len(schedule)} episodes on {args.worker_count} workers",
          flush=True)
    with SimFarm(worker_count=args.worker_count, mission_id=args.mission,
                 n_episodes_per_worker=per_worker, model=args.model, headless=True,
                 seed_base=args.seed * 1000, run_id=args.run_id, resume=args.resume,
                 results_dir=args.results_dir, enable_rotor_fault=True,
                 fault_specs_by_worker=build_fault_specs_by_worker(
                     schedule, args.worker_count, per_worker),
                 fault_config_digest=digest(schedule_cfg), recovery=recovery) as farm:
        done = [sum(farm._completed_per_worker.values())]

        def on_result(r: dict) -> None:
            done[0] += 1
            print(f"  [{done[0]}/{len(schedule)}] w{r['worker_id']} {r['episode_id']} "
                  f"s={r['fault_severity_commanded']:.2f} -> {r['termination_reason']} "
                  f"({r['t_sim_duration_s']:.0f}s sim)", flush=True)

        farm.run(on_result=on_result,
                 on_restart=lambda i: print(f"  *** instance {i} restarted ***", flush=True))
    print(report_text(summarize(Path(args.results_dir) / args.run_id)))


def episode_rows(run_dir: Path) -> pd.DataFrame:
    """One row per valid episode: its requested fault, termination, outcome,
    and what the policy did -- everything the recovery tables need."""
    from experiments.metrics import classify_outcome
    from rl.policies.base_policy import load_outcome_spec
    spec = load_outcome_spec()
    rows = []
    for sp in sorted(Path(run_dir).glob("worker_*/episode_*_summary.parquet")):
        s = pd.read_parquet(sp).iloc[0]
        if not s.valid or s.n_steps == 0:
            continue
        steps = pd.read_parquet(sp.with_name(sp.name.replace("_summary", "_steps")))
        o = classify_outcome(s.termination_reason, steps, spec)
        sens = {f"crash_at_{v:g}": classify_outcome(s.termination_reason, steps, spec,
                                                    crash_touchdown_speed_m_s=v).outcome.value == "crash"
                for v in spec.sensitivity_touchdown_speeds_m_s}
        mission = steps[steps.flight_phase == "mission"]
        airborne = mission[-mission.pos_z > 1.0]
        motors = airborne[[f"motor_{i}_output" for i in range(4)]].to_numpy(float)
        states = [x for x in steps.policy_state.unique() if x]
        rows.append(dict(
            key=f"{s.worker_id}/{s.episode_id}", severity=float(s.fault_severity_commanded),
            profile=s.fault_profile, termination=s.termination_reason, outcome=o.outcome.value,
            touchdown_speed_m_s=o.touchdown_speed_m_s, max_tilt_deg=o.max_tilt_deg,
            waypoints_reached=int(s.waypoints_reached), t_sim_duration_s=float(s.t_sim_duration_s),
            position_rmse_m=float(s.position_rmse_m),
            peak_hspeed_m_s=float(np.hypot(mission.vel_x, mission.vel_y).max()) if len(mission) else np.nan,
            mean_motor_command=float(np.nanmean(motors)) if motors.size else np.nan,
            policy_landed=bool(steps.action_land.any()), fsm_states=",".join(states),
            detector_alarm_edges=int((steps.det_alarm.astype(bool)
                                      & ~steps.det_alarm.astype(bool).shift(fill_value=False)).sum()),
            **sens))
    return pd.DataFrame(rows)


def summarize(run_dir: Path, rows: Optional[pd.DataFrame] = None) -> dict:
    df = episode_rows(run_dir) if rows is None else rows
    out = {"run_dir": str(run_dir), "n_valid": int(len(df)), "by_severity": {}}
    for sev, g in df.groupby("severity"):
        cell = {"n": int(len(g))}
        for o in ("mission_success", "safe_landing", "crash", "incomplete"):
            cell[o] = float((g.outcome == o).mean())
        for c in [c for c in g.columns if c.startswith("crash_at_")]:
            cell[c] = float(g[c].mean())
        cell["policy_landed"] = float(g.policy_landed.mean())
        cell["median_duration_s"] = float(g.t_sim_duration_s.median())
        cell["median_touchdown_speed_m_s"] = float(g.touchdown_speed_m_s.median())
        for col in ("peak_hspeed_m_s", "mean_motor_command"):
            if col in g:
                cell[f"median_{col}"] = float(g[col].median())
        out["by_severity"][f"{sev:.2f}"] = cell
    return out


OUTCOME_NAMES = ("mission_success", "safe_landing", "crash", "incomplete")


def transfer_table(policies: Sequence[tuple[str, dict, dict]], baselines: Sequence[tuple[str, dict]] = ()) -> dict:
    """M9's transfer table (research question 5). `policies`: (label, PX4
    summary, Isaac summary) per learned policy; `baselines`: (label, PX4
    summary). Per severity: each policy's outcome rates in Isaac and on PX4
    (PX4 with 95% Wilson intervals), the PX4-minus-Isaac gap, the spread of
    the PX4 rates across the policies (the training seeds), and the
    baselines' PX4 rates on the same faults. Rates only -- no reward is
    computed on the PX4 side."""
    from experiments.metrics import wilson_interval

    def cell(c: dict, o: str) -> dict:
        n = int(c["n"])
        k = int(round(c[o] * n))
        lo, hi = wilson_interval(k, n)
        return dict(rate=c[o], n=n, lo=lo, hi=hi)

    sevs = sorted({k for _, px4, isaac in policies for k in (*px4["by_severity"], *isaac["by_severity"])}
                  | {k for _, b in baselines for k in b["by_severity"]}, key=float)
    rows = []
    for sev in sevs:
        row: dict = {"severity": sev, "policies": {}, "baselines": {}, "seed_spread": {}}
        for label, px4, isaac in policies:
            pc, ic = px4["by_severity"].get(sev), isaac["by_severity"].get(sev)
            entry = {}
            for o in OUTCOME_NAMES:
                entry[o] = dict(px4=cell(pc, o) if pc else None, isaac=ic[o] if ic else None,
                                gap=(pc[o] - ic[o]) if pc and ic else None)
            entry["px4_median_touchdown_speed_m_s"] = pc.get("median_touchdown_speed_m_s") if pc else None
            entry["isaac_median_touchdown_speed_m_s"] = ic.get("median_touchdown_speed_m_s") if ic else None
            row["policies"][label] = entry
        for o in OUTCOME_NAMES:
            vals = [px4["by_severity"][sev][o] for _, px4, _ in policies if sev in px4["by_severity"]]
            row["seed_spread"][o] = dict(mean=float(np.mean(vals)), min=float(min(vals)), max=float(max(vals))) \
                if vals else None
        for label, b in baselines:
            bc = b["by_severity"].get(sev)
            row["baselines"][label] = {o: cell(bc, o) for o in OUTCOME_NAMES} if bc else None
        rows.append(row)
    return dict(policies=[p[0] for p in policies], baselines=[b[0] for b in baselines], rows=rows)


def transfer_text(table: dict) -> str:
    pct = lambda x: "  -  " if x is None else f"{100 * x:4.0f}%"   # noqa: E731
    lines = ["Transfer table: Isaac vs PX4 (success / crash), PX4 95% Wilson interval in brackets"]
    for row in table["rows"]:
        lines.append(f"s={row['severity']}")
        for label, e in row["policies"].items():
            s, c = e["mission_success"], e["crash"]
            ps, pc = s["px4"], c["px4"]
            lines.append(
                f"  {label:>12}: isaac {pct(s['isaac'])} / {pct(c['isaac'])}   px4 "
                + (f"{pct(ps['rate'])} [{pct(ps['lo'])}-{pct(ps['hi'])}] / {pct(pc['rate'])} "
                   f"[{pct(pc['lo'])}-{pct(pc['hi'])}] (n {ps['n']})" if ps else "  -"))
        for label, b in row["baselines"].items():
            if b:
                lines.append(f"  {label:>12}: px4 {pct(b['mission_success']['rate'])} / {pct(b['crash']['rate'])} "
                             f"(n {b['crash']['n']})")
    return "\n".join(lines)


def report_text(summary: dict) -> str:
    lines = [f"{summary['run_dir']}: {summary['n_valid']} valid episodes",
             f"{'sev':>5} {'n':>3} {'success':>8} {'landed':>7} {'crash':>6} {'incompl':>8} "
             f"{'pol_land':>8} {'dur_s':>6}"]
    for sev, c in summary["by_severity"].items():
        lines.append(f"{sev:>5} {c['n']:>3} {c['mission_success']:>8.2f} {c['safe_landing']:>7.2f} "
                     f"{c['crash']:>6.2f} {c['incomplete']:>8.2f} {c['policy_landed']:>8.2f} "
                     f"{c['median_duration_s']:>6.1f}")
    return "\n".join(lines)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fly")
    f.add_argument("--run-id", required=True)
    f.add_argument("--policy", default="nominal", choices=["nominal", "rule_based", "constant", "learned"])
    f.add_argument("--constant-action", type=float, nargs=3, default=None,
                   help="the constant policy's action: speed_scale altitude_offset_m land")
    f.add_argument("--policy-config", default=None,
                   help="rule_based: its yaml; learned: the exported policy.pt")
    f.add_argument("--detector", default=None)
    f.add_argument("--severities", type=float, nargs="+", required=True)
    f.add_argument("--episodes-per-severity", type=int, required=True)
    f.add_argument("--seed", type=int, required=True)
    f.add_argument("--worker-count", type=int, default=2)
    f.add_argument("--mission", default="square_circuit")
    f.add_argument("--model", default="x500_aero")
    f.add_argument("--results-dir", default=str(REPO / "results"))
    f.add_argument("--resume", action="store_true")
    r = sub.add_parser("report")
    r.add_argument("run_dir")
    r.add_argument("--json", default=None, help="also write the summary here")
    t = sub.add_parser("transfer", help="M9: Isaac vs PX4 per learned policy, plus baselines")
    t.add_argument("--policy", nargs=3, action="append", required=True, metavar=("LABEL", "PX4_RUN", "ISAAC_JSON"))
    t.add_argument("--baseline", nargs=2, action="append", default=[], metavar=("LABEL", "PX4_RUN"))
    t.add_argument("--json", default=None)
    args = ap.parse_args(argv)
    if args.cmd == "fly":
        fly(args)
    elif args.cmd == "transfer":
        table = transfer_table(
            [(label, summarize(Path(run)), json.loads(Path(isaac).read_text())["summary"])
             for label, run, isaac in args.policy],
            [(label, summarize(Path(run))) for label, run in args.baseline])
        print(transfer_text(table))
        if args.json:
            Path(args.json).write_text(json.dumps(table, indent=2))
    else:
        summary = summarize(Path(args.run_dir))
        print(report_text(summary))
        if args.json:
            Path(args.json).write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
