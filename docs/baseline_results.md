# M3 baseline results

Companion to `milestones.md` M3. Every number here comes from a real run
against the simulator (`experiments/run_episodes.py` +
`experiments/analysis/noise_floor.py`), not an estimate. Raw episode records
are under `results/<run_id>/worker_0/`; re-derive any number below with:

```bash
python experiments/analysis/noise_floor.py results/<run_id>/
```

Runs referenced below (mission `square_circuit`, instance 0, speed factor
1x, seed 1):

| run_id | reset tier requested | episodes |
|---|---|---|
| `m3_healthy_20` | soft | 20 completed (task 8, doubles as task 7's fixed-seed repeats) |
| `m3_reset_hard_20` | hard | 20 completed (task 6's hard-reset comparison group) |

---

## Task 8 — 20 healthy missions

**20/20 completed.** `m3_healthy_20` needed 22 total attempts: 2 episodes
hit the reset-ladder issue described under Task 5 below and were retried
(escalated to hard reset, which recovered both immediately) rather than
counted as a mission failure — see "A note on retries" at the end of this
file for why that is the honest way to report this, not a way to hide a
problem. `m3_reset_hard_20` needed zero retries (20/20 on the first attempt
each).

**Noise floor** (position RMSE and final position error against the
mission's own commanded trajectory, over all 40 completed episodes across
both runs, mission is `square_circuit.yaml`: a ~15m square with 2s holds at
each corner plus a 3s final hover, speed factor 1x):

| metric | mean | std | min | max |
|---|---|---|---|---|
| position RMSE (m) | 6.44 | 0.57 | 5.50 | 9.09 |
| final position error (m) | 0.29 | 0.05 | 0.18 | 0.51 |

This is every later result's noise floor: a fault-injected (M6+) run that
differs from this by less than the spread above has told us nothing.

---

## Task 7 — run-to-run divergence at a fixed seed (D11)

Same mission, same seed (1), 20 repeats, reported separately per reset tier
since Task 6 (below) found they are **not** interchangeable for this
purpose. This defines what "reproducible" means for the rest of the
project — PX4 SITL + Gazebo is not bitwise deterministic, so reproducibility
here means "future runs fall within this measured spread," not "identical."

**Following hard reset** (`m3_reset_hard_20`, n=19 pure-hard episodes —
the cleaner, recommended baseline for this purpose; see Task 6):

| metric | std | range |
|---|---|---|
| position RMSE (m) | 0.083 | [6.13, 6.53] |
| final x position (m) | 0.016 | [0.167, 0.229] |
| final y position (m) | 0.017 | [0.172, 0.245] |
| final z position (m) | 0.006 | [-5.00, -4.98] |
| flight time, wall (s) | 0.10 | mean 42.5 |
| flight time, sim (s) | 0.13 | mean 41.7 |

**Following soft reset** (`m3_healthy_20`, n=17 pure-soft episodes):

| metric | std | range |
|---|---|---|
| position RMSE (m) | 0.82 | [5.50, 9.09] |
| final x position (m) | 0.12 | [-0.34, 0.22] |
| final y position (m) | 0.05 | [0.16, 0.38] |
| final z position (m) | 0.03 | [-5.08, -4.98] |
| flight time, wall (s) | 5.82 | mean 52.9 |
| flight time, sim (s) | 5.84 | mean 51.9 |

Hard reset's spread is roughly an order of magnitude tighter on every
metric. **Use the hard-reset numbers as the project's reproducibility
tolerance** for comparisons in M6+ unless a specific comparison is itself
about soft-reset behaviour.

---

## Task 6 — does soft reset leak state?

**Finding: yes, measurably, even though position/velocity/arming-state all
pass the tolerances `aero_bridge/reset.py` checks.** This is exactly the
"it looks fine is not evidence" scenario `milestones.md` warns about for
this task.

Comparing the two groups above (pure soft-tier vs pure hard-tier completed
episodes, same mission/seed):

| metric | soft (n=17) | hard (n=19) | ratio |
|---|---|---|---|
| RMSE mean | 6.65m | 6.33m | ~1.05x (small) |
| RMSE std | 0.82m | 0.083m | **~10x** |
| final error mean | 0.28m | 0.29m | ~1.0x (none) |
| final error std | 0.072m | 0.017m | **~4x** |
| flight wall-time mean | 52.9s | 42.5s | ~1.24x |
| flight wall-time std | 5.82s | 0.10s | **~58x** |

The **means** are close — soft reset does not make the vehicle fly a
systematically worse circuit. The **spreads** are not close: flights
following a soft reset take longer and vary far more in both duration and
RMSE than flights following a hard reset, which are strikingly consistent
(wall-time std of 0.10s over 19 episodes). The step-count distributions make
the same point directly: hard-reset episodes cluster tightly at 266-286
control steps; soft-reset episodes range from 262 to 536.

The most likely mechanism, consistent with everything measured: `soft_reset`
(module docstring, `aero_bridge/reset.py`) repositions the vehicle and
confirms position/velocity/arming settle within tolerance, but PX4's own
attitude/rate controller integrators, EKF covariance and bias estimates are
never given a real restart the way `PREFLIGHT_REBOOT_SHUTDOWN` (medium) or a
full process restart (hard) provide. `_wait_until_settled`'s checks do not
and cannot see that internal state. This is precisely the leak the milestone
asked this experiment to look for, and neither soft nor medium turned out to
be free of it.

**Consequence for M4/M9's throughput budget** (flagged here because it
changes M4's task 6, not because M3 owns fixing it): `milestones.md`'s
guidance was "if soft leaks, budget from medium instead" — but medium is
*also* not usable post-flight in this stack (Task 5 below), independently of
this finding. **Until one of the two cheaper tiers is fixed, M4 should
budget parallel throughput assuming hard reset (~19.8s, Task 5) between
every episode**, not soft's ~0.6s. That is a substantial difference for the
throughput sweep M4 task 6 runs — worth deciding early rather than
discovering under a training run.

---

## Task 5 — reset ladder, measured cost

| tier | measured wall-clock cost | reliable? |
|---|---|---|
| soft | 0.59s ± 0.44 (n=19, `m3_healthy_20`) | Fast, but Task 6 shows it leaks controller/EKF state. Also independently triggered a **latched compass fault** (see below) on 2/22 attempts before the fix landed. |
| medium | 0.5s to recover, *when it works* | **Not usable post-flight.** See finding below — kept implemented (the milestone asks for all three tiers to exist) but not used by `run_episodes.py`. |
| hard | 19.78s ± 0.13 (n=19, `m3_reset_hard_20`) | Always correct. The only tier this baseline actually relies on when a retry is needed. |

**Medium reset finding.** `VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN` reliably
recovers `pre_flight_checks_pass` (~0.5s) on a PX4 instance that has never
armed in that session, but was measured live to **never** recover it
(tested to 75s wall-clock) once the vehicle has actually armed and flown --
PX4's own log stops emitting any further module output at all right after
the command in that case, suggesting an incomplete internal reinit for a
PX4 that has already flown, under this exact stack (PX4 v1.17.0 SITL +
Gazebo Harmonic via gz-bridge). Not further root-caused within M3's scope.
Documented and tested as expected behaviour in
`tests/sim/test_reset.py::test_medium_reset_after_flight_is_unreliable`
rather than silently left broken.

**Compass-fault finding (fixed during M3, still worth recording).** Early
runs of Task 8 found that repeated `gz set_pose` teleports between
episodes -- even small ones -- eventually and permanently trip PX4's
magnetometer consistency check ("Preflight Fail: Compass 0 fault"), which
does not clear short of a hard reset. Fixed by having `soft_reset`/
`medium_reset` skip the teleport entirely when the vehicle is already within
tolerance of the spawn pose (`_already_at_spawn`, `aero_bridge/reset.py`) --
true on almost every M3 episode, since `square_circuit.yaml` always returns
to its own start point before landing. This cut the fault rate from
"permanent after ~8 cumulative resets" to 2 occurrences in 42 total
completed-run attempts across both baseline runs, both of which the
run-level retry-and-escalate logic in `experiments/run_episodes.py`
recovered from automatically. Not fully eliminated -- an episode that lands
meaningfully off-spawn (a real possibility once M6 injects faults) will
still need a genuine teleport and could still occasionally trip it. Left as
a known, monitored residual risk rather than further chased within M3.

---

## A note on retries

`run_episodes.py` retries an episode that did not end in `completed`,
capped at `n // 2` (or 5, whichever is larger) extra attempts, and always
escalates the reset before a retry to hard. This is not "19 of 20, the
other one was a fluke" (`CLAUDE.md`'s explicit warning against exactly that)
-- every attempt, including the ones that did not complete, is written as
its own real episode record with its true `termination_reason`
(`preflight_failed`), nothing is discarded, and the retry mechanism itself
is the thing being measured and reported here (2/22 and 0/20 above). What
the retry cap guards against is a *systemic* problem masquerading as 20
individually-reasonable-looking failures; hitting the cap raises loudly
rather than reporting a partial run as if it were complete.
