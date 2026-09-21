#!/usr/bin/env python3
"""M4 task 6: the throughput sweep M9's training-time budget check is
supposed to be based on (rescoped by D12 to calibrate an *evaluation* farm's
throughput, not a training farm's -- see milestones.md M4's rescope note).

Sweeps worker count x speed factor, running a small, fixed number of real
episodes per configuration through the project's one real path (SimFarm ->
EpisodeRunner -> fly_mission), and records the aggregate throughput number
that actually matters: simulated-seconds delivered per wall-second, plus
per-worker RTF, peak memory, CPU utilization, and failure rate.

This is a CLI script, not a library import -- like run_episodes.py, its
SimFarm(...) construction happens inside `if __name__ == "__main__":`,
because multiprocessing's "spawn" start method re-imports the launching
script in every worker child it creates (see sim_farm.py's own module
docstring for the failure mode an unguarded top-level SimFarm(...) call
produces).

Usage:
    python experiments/benchmark_throughput.py --workers 1,2,3,4 --speeds 1,2,4,8
    python experiments/benchmark_throughput.py --workers 1,2 --speeds 1,4 --episodes-per-worker 2
"""
from __future__ import annotations

import argparse
import datetime
import json
import statistics
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from experiments.resource_sampler import ResourceSampler


def _sim_stop_all() -> None:
    subprocess.run([str(REPO / "scripts" / "sim_stop.sh"), "--all"],
                    capture_output=True, text=True, timeout=120)


def run_one_config(*, worker_count: int, speed_factor: float, mission_id: str,
                    episodes_per_worker: int, results_dir: str, cfg_label: str) -> dict:
    """Runs one (worker_count, speed_factor) configuration for real through
    SimFarm and returns its measured stats. Printed progress makes this
    visible line-by-line in the terminal rather than going silent for the
    whole configuration's duration."""
    from experiments.sim_farm import SimFarm

    total_planned = worker_count * episodes_per_worker
    completed = 0
    restarts = 0

    def on_result(record: dict) -> None:
        nonlocal completed
        completed += 1
        print(f"  [{cfg_label}] [{completed}/{total_planned}] worker {record['worker_id']} "
              f"{record['episode_id']} -> {record['termination_reason']} "
              f"(t_sim={record['t_sim_duration_s']:.1f}s t_wall={record['t_wall_duration_s']:.1f}s "
              f"valid={record['valid']})", flush=True)

    def on_restart(instance: int) -> None:
        nonlocal restarts
        restarts += 1
        print(f"  [{cfg_label}] *** worker (instance {instance}) restarted "
              f"(restart #{restarts} this config) ***", flush=True)

    run_id = f"throughput_{cfg_label}_{datetime.datetime.now(tz=datetime.timezone.utc):%Y%m%dT%H%M%SZ}"

    t_start = time.perf_counter()
    with ResourceSampler() as sampler:
        with SimFarm(worker_count=worker_count, mission_id=mission_id,
                     n_episodes_per_worker=episodes_per_worker, speed_factor=speed_factor,
                     headless=True, run_id=run_id, results_dir=results_dir) as farm:
            results = farm.run(on_result=on_result, on_restart=on_restart)
    wall_s = time.perf_counter() - t_start

    rtfs = [r["t_sim_duration_s"] / r["t_wall_duration_s"]
            for r in results if r["valid"] and r["t_wall_duration_s"] > 0]
    total_sim_s = sum(r["t_sim_duration_s"] for r in results)
    invalid = sum(1 for r in results if not r["valid"])

    return dict(
        worker_count=worker_count, speed_factor=speed_factor,
        n_episodes=len(results), total_planned=total_planned,
        wall_s=round(wall_s, 2),
        aggregate_sim_s_per_wall_s=round(total_sim_s / wall_s, 3) if wall_s > 0 else 0.0,
        episodes_per_wall_hour=round(len(results) / wall_s * 3600, 1) if wall_s > 0 else 0.0,
        rtf_mean=round(statistics.mean(rtfs), 3) if rtfs else 0.0,
        rtf_stdev=round(statistics.stdev(rtfs), 3) if len(rtfs) > 1 else 0.0,
        peak_rss_mb=round(sampler.peak_rss_mb, 1),
        mean_cpu_percent=round(sampler.mean_cpu_percent, 1),
        restarts=restarts,
        invalid_episodes=invalid,
        failure_rate=round(invalid / len(results), 3) if results else 0.0,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workers", default="1,2,3,4", help="comma-separated worker counts")
    parser.add_argument("--speeds", default="1,2,4,8", help="comma-separated speed factors")
    parser.add_argument("--episodes-per-worker", type=int, default=3,
                         help="episodes per worker per configuration (default 3: enough to "
                              "see hard reset's fixed wall-clock cost clearly)")
    parser.add_argument("--mission", default="square_circuit")
    parser.add_argument("--results-dir", default="results/throughput_sweep")
    parser.add_argument("--resume-from", default=None,
                         help="path to a previous (partial) sweep_*.json -- configurations "
                              "already present there (by worker_count, speed_factor) are "
                              "skipped rather than re-run")
    args = parser.parse_args()

    workers = [int(w) for w in args.workers.split(",")]
    speeds = [float(s) for s in args.speeds.split(",")]
    configs = [(w, s) for w in workers for s in speeds]

    sweep_results: list[dict] = []
    already_done: set[tuple[int, float]] = set()
    if args.resume_from:
        resume_path = Path(args.resume_from)
        sweep_results = json.loads(resume_path.read_text())
        already_done = {(r["worker_count"], r["speed_factor"]) for r in sweep_results}
        print(f"Resuming from {resume_path}: {len(sweep_results)} configuration(s) "
              f"already done, will be kept as-is and not re-run.")

    remaining = [(w, s) for (w, s) in configs if (w, s) not in already_done]

    print(f"Throughput sweep: {len(configs)} configurations total "
          f"(workers={workers} x speeds={speeds}), "
          f"{args.episodes_per_worker} episodes/worker each. "
          f"{len(remaining)} left to run.")
    print("Confirming a clean slate before starting...", flush=True)
    _sim_stop_all()

    out_dir = REPO / args.results_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"sweep_{datetime.datetime.now(tz=datetime.timezone.utc):%Y%m%dT%H%M%SZ}.json"
    out_path.write_text(json.dumps(sweep_results, indent=2))  # seed it with the resumed rows immediately

    sweep_start = time.perf_counter()
    for i, (w, s) in enumerate(remaining, start=1):
        cfg_label = f"cfg{i}of{len(remaining)}_w{w}_s{s:g}"
        print(f"\n=== [{i}/{len(remaining)} remaining] worker_count={w} speed_factor={s:g} ===",
              flush=True)
        cfg_start = time.perf_counter()
        try:
            stats = run_one_config(worker_count=w, speed_factor=s, mission_id=args.mission,
                                    episodes_per_worker=args.episodes_per_worker,
                                    results_dir=str(REPO / "results" / "throughput_sweep_episodes"),
                                    cfg_label=cfg_label)
        except Exception as exc:  # noqa: BLE001 -- one bad configuration must not abort the whole sweep
            print(f"--- config worker_count={w} speed_factor={s:g} FAILED: {exc!r} ---", flush=True)
            stats = dict(worker_count=w, speed_factor=s, error=repr(exc))
        finally:
            _sim_stop_all()  # sanctioned --all teardown between configs, no workers alive at this point
        cfg_elapsed = time.perf_counter() - cfg_start
        stats["config_wall_s_measured"] = round(cfg_elapsed, 2)
        sweep_results.append(stats)
        out_path.write_text(json.dumps(sweep_results, indent=2))  # incremental, same pattern as the manifest
        if "error" in stats:
            continue
        print(f"--- config done in {cfg_elapsed:.1f}s: "
              f"aggregate={stats['aggregate_sim_s_per_wall_s']} sim-s/wall-s, "
              f"rtf_mean={stats['rtf_mean']}, peak_rss={stats['peak_rss_mb']}MB, "
              f"cpu={stats['mean_cpu_percent']}%, restarts={stats['restarts']}, "
              f"failure_rate={stats['failure_rate']} ---", flush=True)

    total_elapsed = time.perf_counter() - sweep_start
    print(f"\nSweep finished in {total_elapsed / 60:.1f} min. Results: {out_path}")
    print("\n%-8s %-8s %10s %10s %10s %10s %9s" %
          ("workers", "speed", "sim-s/wall", "rtf_mean", "peak_rss", "cpu%", "restarts"))
    for r in sweep_results:
        if "error" in r:
            print(f"{r['worker_count']:<8d} {r['speed_factor']:<8g} FAILED: {r['error']}")
            continue
        print("%-8d %-8g %10.2f %10.3f %10.1f %10.1f %9d" %
              (r["worker_count"], r["speed_factor"], r["aggregate_sim_s_per_wall_s"],
               r["rtf_mean"], r["peak_rss_mb"], r["mean_cpu_percent"], r["restarts"]))


if __name__ == "__main__":
    main()
