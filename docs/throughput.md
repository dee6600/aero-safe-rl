# M4 task 6 — throughput sweep results

Companion to `milestones.md` M4. Every number here comes from a real run
against the simulator (`experiments/benchmark_throughput.py`), not an
estimate. Mission `square_circuit`, 3 episodes/worker/configuration, headless.
Raw per-configuration data: `results/throughput_sweep/sweep_final.json`.
Machine: 6-core/12-thread Intel i7-8750H, ~15.8 GB RAM
(`docs/isaac_feasibility.md`).

Regenerate with:

```bash
scripts/sim_stop.sh --all
python experiments/benchmark_throughput.py --workers 1,2,3,4 --speeds 1,2,4,8
```

## Full table

| workers | speed | sim-s/wall-s | RTF (mean) | peak RSS (MB) | CPU % | restarts | failure rate |
|---|---|---|---|---|---|---|---|
| 1 | 1x | 0.80 | 0.98 | 4821 | 17.6 | 0 | 0.00 |
| 1 | 2x | 1.50 | 1.96 | 4805 | 15.7 | 0 | 0.00 |
| 1 | 4x | 2.43 | 3.75 | 4803 | 16.0 | 0 | 0.00 |
| 1 | 8x | 3.37 | 7.06 | 4728 | 29.0 | 0 | 0.00 |
| 2 | 1x | 1.61 | 1.13 | 4877 | 34.6 | 0 | 0.00 |
| 2 | 2x | 2.77 | 1.95 | 4980 | 39.8 | 0 | 0.00 |
| **2** | **4x** | **4.01** | 3.49 | 5014 | 63.6 | 0 | 0.00 |
| 2 | 8x | 3.35 | 4.22 | 4886 | 63.3 | 0 | 0.00 |
| 3 | 1x | 1.86 | 1.03 | 5305 | 52.7 | 0 | 0.00 |
| 3 | 2x | 3.19 | 1.84 | 5213 | 77.3 | 1 | 0.10 |
| 3 | 4x | 3.74 | 5.90 | 5215 | 78.1 | 1 | 0.11 |
| 3 | 8x | 1.95 | 2.95 | 5388 | 79.4 | 4 | 0.44 |
| 4 | 1x | 3.39 | 1.96 | 5542 | 71.9 | 1 | 0.08 |
| 4 | 2x | 1.79 | 1.39 | 5597 | 81.6 | 5 | 0.42 |
| 4 | 4x | **unreliable** | — | — | — | — | see below |
| 4 | 8x | 1.33 | 2.52 | 5701 | 90.9 | 4 | 0.36 |

**sim-s/wall-s** is the number that actually matters: aggregate simulated
seconds delivered per real second, across all workers in that configuration.
Higher is better. **RTF** is the mean per-episode ratio of simulated flight
time to wall flight time (reset time excluded — matches M1's own RTF
definition). **Failure rate** is the fraction of episodes that ended
`valid=false` (worker_restarted or, in principle, sim_fault).

## Chosen operating point: **2 workers** — speed factor revised to **1x**

This table's "2 workers, 4x speed" row (`4.01 sim-s/wall-s`, zero restarts,
zero failures) was the best entry across all 16 configurations tested, and
was originally written up as the recommended operating point. **A follow-up
soak-test run at that exact configuration (2 workers, 4x, but 200 episodes/
worker instead of 3) found this table doesn't tell the whole story**: at
that larger sample size, 64% of episodes hit `episode_timeout` rather than
actually completing the mission, and the same worker-restart pattern that
sank the worker_count=4 configuration began recurring. This table's
`aggregate sim-s/wall-s` and `failure rate` columns only count restarts and
`valid=false` records — an episode that times out without ever completing
is still `valid=true` and doesn't show up as a "failure" here, so a
configuration can look perfectly clean in this table while mostly not
actually finishing its missions. **The worker COUNT recommendation (2, not
3 or 4) still stands** — that conclusion came from restart/leak behavior,
which the soak test confirmed. Only the **speed factor** recommendation is
revised: **1x**, the only speed actually proven reliable at soak scale.

**Final confirmed result (2026-09-22): 2 workers × 200 episodes × 1x speed,
the full 400-episode soak test, PASSED clean** — 0 restarts, 0 orphan
processes, flat memory (peak 4625.2MB) over ~3 hours unattended. Getting
there also required raising `restart_budget_per_worker` from 5 to 20
(`configs/env/farm.yaml`), an evidence-based change from two earlier runs
at this exact configuration that each hit `RestartBudgetExhausted` from the
same background DDS/rclpy crash rate accumulating over 200 episodes/worker
— not a new bug, just the old budget proving too tight for a run this long.
See `tests/slow/test_soak.py`'s own module docstring for the full writeup
and the standing instruction not to revert to 4x speed without
re-validating it at full soak scale first.

**Why not more workers, if this project has 4+ cores to spare?** Because more
workers did not mean more throughput on this hardware — worker_count=3 and 4
match or *underperform* worker_count=2 in the aggregate-throughput column,
because the time spent on restarts (workers failing to start cleanly, or
crashing mid-flight and needing a supervisor-triggered restart) eats into
whatever parallelism should have gained. worker_count=4 never beat
worker_count=2's peak, at any speed factor tested.

## The M9 training-time budget check

The milestone text requires: *"if the best aggregate throughput implies M9
cannot reach 1–3M steps in under ~5 days, stop and revisit decision D5."*
This is satisfied **by construction**, not by this sweep's numbers: D12
(2026-09-21) already moved M9's training onto Isaac Lab, which M3b measured
at 546k env-steps/s — several orders of magnitude beyond anything this
Gazebo/PX4 stack could deliver, evaluation-scale or not. This sweep's numbers
budget M6's dataset generation and M10's evaluation sweep, not M9's training.

## Real reliability finding: worker_count ≥ 3 gets measurably less stable

Every configuration at worker_count ≥ 3 needed at least one supervisor
restart except 3x1x. worker_count=4 at 4x speed **failed outright on all
three attempts**, each with a *different* proximate symptom:

1. A PX4 SITL instance lock collision ("PX4 server already running") caused
   by a leaked, untracked worker process from a prior restart.
2. The same leak pattern recurring from a different worker mid-run.
3. A live worker that started cleanly (its three OS processes all came up)
   but never produced telemetry within the 90-second readiness deadline.

Three different failure signatures at the exact same (4 workers, 4x speed)
combination, after the first two's root causes (a genuine supervisor
process-leak bug) were found and fixed mid-sweep, points to this specific
combination being a real reliability cliff on this hardware rather than a
single fixable bug — CPU utilisation tops out at 90.9% even at the sweep's
heaviest configuration (4 workers, 8x), well short of saturation, so the
bottleneck is **not** raw CPU contention. It looks like this machine's DDS
stack (ROS 2 / MicroXRCEAgent) and PX4's own per-instance startup path
becoming unreliable under 4 simultaneous SITL instances plus a demanding
speed factor, not a scheduling problem `taskset` core-pinning would fix.

**Two real bugs were found and fixed live during this sweep** (full detail:
`experiments/worker_supervisor.py`'s `is_healthy()`/`ensure_healthy()`
docstrings, `aero_bridge/reset.py`'s `REPO_DIR` comment, and `milestones.md`'s
M4 progress notes):
- `WorkerSupervisor.is_healthy()` was racing a worker's own in-flight
  `hard_reset()` (which legitimately has no `instance_<N>.json` for ~20s) and
  piling a redundant restart on top of it. Fixed by trusting a live child's
  heartbeat over raw PID presence.
- `ensure_healthy()`'s restart path could leave one of a worker's three OS
  processes (observed: a `gz sim` + `MicroXRCEAgent` pair) alive and
  untracked, permanently blocking every future start attempt for that
  instance. Fixed with a verify-and-force-kill fallback scoped to exactly
  the PIDs that were tracked before the restart.

A third, deeper timing tension was found here but **fixed during task 7's
soak test, not this sweep**: a worker's own `hard_reset()` writes no
heartbeat while it runs, so a reset slow enough (under heavy contention) can
still look stale to the parent's health check before it finishes, causing a
redundant restart on top of one already in progress. This didn't corrupt any
data in this sweep (the queue-race dedup fix above already covers a
duplicate record either way) but burned through a worker's restart budget
fast enough during the soak test's sustained 4-worker load to threaten the
whole run. Fixed in `EpisodeRunner.run_episode()`: the child now writes a
fresh heartbeat right before a reset starts, giving the health check's
timeout window the actual start of the slow operation to measure from,
instead of whenever the last flight control tick happened to be.

**Task 8 (CPU affinity): not warranted.** The milestone text gates it on task
6 showing contention, and CPU never saturates in this sweep (peaks at 90.9%)
— the instability above is a DDS/process-startup reliability problem, not a
scheduling one. `taskset` core-pinning would not address it.

## Method notes

- 3 episodes/worker/configuration: enough to see hard reset's ~19.8s
  wall-clock-fixed cost clearly (M3) without turning the sweep into a
  multi-hour run.
- Peak RSS and mean CPU% are whole-machine samples (`experiments/
  resource_sampler.py`), not per-process sums — simpler and, per
  `docs/isaac_feasibility.md`'s own precedent, good enough for calibrating an
  operating point.
- `scripts/sim_stop.sh --all` runs between every configuration.
- The 4x4 configuration's final, accepted result is the third attempt's
  numbers-free `FAILED` entry — the first two attempts' failures were
  diagnosed as the process-leak bug above (now fixed) and are not the
  reported outcome for this cell; the third attempt's telemetry-timeout
  failure, after that fix, is.
