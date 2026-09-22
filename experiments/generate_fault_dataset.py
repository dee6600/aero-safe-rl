#!/usr/bin/env python3
"""M6 task 9: the dataset generator -- flies the labelled dataset M7's
detector trains on, by threading a sampled fault schedule
(experiments/fault_schedule.py, task 1) through SimFarm + EpisodeRunner's
fault-injection integration (task 6). No new "fly one episode" or "N
workers" implementation lives here -- this script's only job is sampling
the schedule once, partitioning it deterministically across workers, and
handing SimFarm the pieces it already knows how to use.

Partitioning: the schedule is sampled once, flat, of length
`worker_count * n_episodes_per_worker` (n_episodes rounded up to a multiple
of worker_count so every worker gets an equal, full share), then split into
CONTIGUOUS per-worker blocks -- worker k gets
`schedule[k*n_episodes_per_worker : (k+1)*n_episodes_per_worker]`. Each
block is indexed by LOCAL episode index (0..n_episodes_per_worker-1) inside
SimFarm/_worker_main, matching how episode ids/seeds are already assigned
per worker (experiments/sim_farm.py) -- a respawned worker resumes with the
correct remaining slice automatically, since _worker_main always receives
the worker's FULL block and indexes into it by
`start_index + i` (see sim_farm.py's _worker_main docstring), the same
resume mechanism episode ids already use.

This is a CLI script, not a library import -- like run_episodes.py and
benchmark_throughput.py, its SimFarm(...) construction happens inside
`if __name__ == "__main__":` (see sim_farm.py's own module docstring for
the recursive-reimport failure mode an unguarded top-level call produces
under multiprocessing's spawn start method).

Usage:
    python experiments/generate_fault_dataset.py \\
        --fault-config configs/faults/rotor_thrust_degradation_v1.yaml \\
        --model x500_aero --worker-count 2 --speed-factor 1 \\
        --n-episodes 750 --run-id m6_dataset_v1
"""
from __future__ import annotations

import argparse
import datetime
import sys
from pathlib import Path
from typing import Optional

import numpy as np

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from experiments.episode_schema import digest
from experiments.fault_schedule import FaultSpec, load_fault_config, sample_fault_schedule


def build_fault_specs_by_worker(schedule: list[FaultSpec], worker_count: int,
                                 n_episodes_per_worker: int) -> list[list[FaultSpec]]:
    """Pure function: splits a flat schedule into `worker_count` contiguous
    blocks of `n_episodes_per_worker` each. `schedule` must already have
    exactly `worker_count * n_episodes_per_worker` entries."""
    expected = worker_count * n_episodes_per_worker
    if len(schedule) != expected:
        raise ValueError(f"schedule has {len(schedule)} entries, expected {expected} "
                          f"(worker_count * n_episodes_per_worker)")
    return [schedule[k * n_episodes_per_worker:(k + 1) * n_episodes_per_worker]
            for k in range(worker_count)]


def generate(*, fault_config_path: str, worker_count: int, speed_factor: float,
             model: str, n_episodes: Optional[int] = None, seed: Optional[int] = None,
             run_id: Optional[str] = None, results_dir: str = "results",
             resume: bool = False) -> list[dict]:
    from experiments.sim_farm import SimFarm

    cfg = load_fault_config(fault_config_path)
    fault_config_digest = digest(cfg)
    n_total = n_episodes if n_episodes is not None else cfg["dataset"]["n_episodes"]
    seed_value = seed if seed is not None else cfg["dataset"]["seed"]

    # Round up to a full multiple of worker_count -- every worker flies an
    # equal share; a caller who wants an exact total picks n_episodes
    # divisible by worker_count.
    n_episodes_per_worker = -(-n_total // worker_count)
    total_planned = worker_count * n_episodes_per_worker

    # Deterministic: resuming with the SAME seed, worker_count and n_episodes
    # reproduces the IDENTICAL flat schedule and per-worker partition as the
    # original run (sample_fault_schedule/build_fault_specs_by_worker are
    # both pure functions of their inputs) -- so a resumed run's
    # already-completed episodes are simply skipped over (via SimFarm's own
    # resume=True, below), never re-derived or re-sampled.
    rng = np.random.default_rng(seed_value)
    schedule = sample_fault_schedule(cfg, rng, total_planned)
    fault_specs_by_worker = build_fault_specs_by_worker(schedule, worker_count, n_episodes_per_worker)

    if resume and run_id is None:
        raise ValueError("resume=True requires the SAME --run-id as the run being resumed")
    run_id = run_id or (
        f"fault_dataset_{datetime.datetime.now(tz=datetime.timezone.utc):%Y%m%dT%H%M%SZ}")

    with SimFarm(worker_count=worker_count, mission_id=cfg["mission_id"],
                 n_episodes_per_worker=n_episodes_per_worker, speed_factor=speed_factor,
                 model=model, headless=True, run_id=run_id, resume=resume,
                 results_dir=results_dir,
                 enable_rotor_fault=True, fault_specs_by_worker=fault_specs_by_worker,
                 fault_config_digest=fault_config_digest) as farm:
        # completed is seeded from what's already on disk when resuming, so
        # the live progress line reads against the true total rather than
        # restarting from 0/750 for a run that is actually most of the way
        # done. this_run_count/faulty_total/confirmed_ok/detector_silent_ok
        # cover only episodes flown in THIS invocation -- the true rates
        # across the WHOLE dataset (including a prior invocation's episodes)
        # are experiments/analysis/fault_dataset_report.py's job, reading
        # every episode back from disk rather than an in-memory counter that
        # does not survive a resume.
        completed = sum(farm._completed_per_worker.values())
        this_run_count = 0
        faulty_total = 0
        confirmed_ok = 0
        detector_silent_ok = 0

        def on_result(record: dict) -> None:
            nonlocal completed, this_run_count, faulty_total, confirmed_ok, detector_silent_ok
            completed += 1
            this_run_count += 1
            if record["fault_applied"]:
                faulty_total += 1
                if record["fault_confirmed_applied"]:
                    confirmed_ok += 1
            if record["px4_failure_detector_silent"]:
                detector_silent_ok += 1
            print(f"  [{completed}/{total_planned}] worker {record['worker_id']} "
                  f"{record['episode_id']} -> {record['termination_reason']} "
                  f"fault_applied={record['fault_applied']} "
                  f"confirmed={record.get('fault_confirmed_applied')} "
                  f"detector_silent={record['px4_failure_detector_silent']}", flush=True)

        def on_restart(instance: int) -> None:
            print(f"  *** worker (instance {instance}) restarted ***", flush=True)

        results = farm.run(on_result=on_result, on_restart=on_restart)

    print(f"done: {completed}/{total_planned} episodes total "
          f"({this_run_count} flown this invocation), "
          f"{confirmed_ok}/{faulty_total} faults confirmed applied this invocation, "
          f"{detector_silent_ok}/{this_run_count} episodes with PX4 FailureDetector "
          f"silent this invocation, "
          f"results under {Path(results_dir) / run_id}")
    return results


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fault-config", required=True,
                     help="e.g. configs/faults/rotor_thrust_degradation_v1.yaml -- its own "
                          "mission_id is always used (not overridable here), since the config's "
                          "own onset/ramp-fits-mission validation is against that mission")
    ap.add_argument("--model", default="x500_aero")
    ap.add_argument("--worker-count", type=int, default=2)
    ap.add_argument("--speed-factor", type=float, default=1.0)
    ap.add_argument("--n-episodes", type=int, default=None,
                     help="overrides the fault config's own dataset.n_episodes")
    ap.add_argument("--seed", type=int, default=None,
                     help="overrides the fault config's own dataset.seed")
    ap.add_argument("--run-id", default=None)
    ap.add_argument("--results-dir", default=str(REPO / "results"))
    ap.add_argument("--resume", action="store_true",
                     help="resume an interrupted run: requires the SAME --run-id, "
                          "--seed/--n-episodes/--worker-count as the original invocation "
                          "(the fault schedule is re-derived deterministically from these, "
                          "not stored separately -- a mismatch silently produces a "
                          "different schedule for the remaining episodes). Each worker "
                          "picks up at its own next unused episode index, found from "
                          "results/<run-id>/worker_<k>/'s existing files.")
    args = ap.parse_args(argv)

    generate(fault_config_path=args.fault_config, worker_count=args.worker_count,
             speed_factor=args.speed_factor, model=args.model, n_episodes=args.n_episodes,
             seed=args.seed, run_id=args.run_id, results_dir=args.results_dir,
             resume=args.resume)


if __name__ == "__main__":
    main()
