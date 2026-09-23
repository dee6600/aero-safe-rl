# M6 task 10 — fault dataset generation results

Companion to `milestones.md` M6. Every number here comes from a real run
against the simulator (`experiments/generate_fault_dataset.py`), not an
estimate. Raw episode records are under `results/m6_dataset_v1/worker_{0,1}/`;
re-derive any number below with:

```bash
python experiments/analysis/fault_dataset_report.py results/m6_dataset_v1/
```

Run: `run_id=m6_dataset_v1`, mission `square_circuit`, model `x500_aero`,
2 workers, 1x speed, fault config `configs/faults/rotor_thrust_degradation_v1.yaml`
(`fault_schema_version: "1"`, seed `20260922`, `healthy_fraction: 0.2`,
`severity_range_s: [0.2, 0.9]`, `onset_time_range_s: [5.0, 40.0]`,
`profiles: [step, ramp]`). 750 episodes planned; **750/750 delivered**.

The run was interrupted twice by real, external events (a full disk, and a
Claude Code session ending mid-run) and resumed both times via
`SimFarm(resume=True)`/`--resume` (M6 task 10's own build, found needed
live) — each worker picked up at its own next unused episode index from
what was already on disk, with zero episodes lost, re-flown, or
double-counted. See `milestones.md` M6 task 10 for the full story.

---

## Headline numbers

| | |
|---|---|
| episodes | 750 |
| valid | 741/750 (98.8%) |
| faulty (positive examples) | 597 |
| healthy (negative examples) | 153 (20.4% of total — matches the configured 20% `healthy_fraction`) |
| overall fault confirmation rate | 465/597 (77.9%) |
| overall PX4 FailureDetector-silent rate (all episodes) | 655/750 (87.3%) |
| PX4 FailureDetector-silent rate (faulty episodes only) | 502/597 (84.1%) |

**By termination_reason:**

| reason | count |
|---|---|
| `completed` | 397 |
| `hold_timeout` | 332 |
| `worker_restarted` | 9 |
| `preflight_failed` | 7 |
| `aborted_error` | 5 |

The 9 `worker_restarted` records are the invalid placeholders
`SimFarm._synthesize_lost_episode_record` writes for an episode a dead
worker never got to report (M4 task 4.2) — background DDS/rclpy
instability already documented in `docs/parallelism.md` §2.6, not a new
finding. `hold_timeout` outnumbering `completed` is itself notable: a
meaningful fraction of injected faults are severe enough that the vehicle
cannot hold a waypoint within the mission's acceptance radius/hold time —
this is real flight behaviour under fault, not a bug in the mission or the
fault plugin.

---

## The actual research-premise check: does PX4 notice?

This is what M6 exists to measure: whether the severities this project
targets are genuinely sub-threshold for PX4's own (naive) `FailureDetector`.
**The answer is nuanced, not a flat yes** — and the honest, monotonic trend
below is a real finding, not something to round off:

| severity range | n (faulty) | fault confirmed applied | PX4 FailureDetector silent |
|---|---|---|---|
| [0.2, 0.4) | 178 | 126/178 (70.8%) | **178/178 (100.0%)** |
| [0.4, 0.6) | 169 | 132/169 (78.1%) | 150/169 (88.8%) |
| [0.6, 0.8) | 152 | 126/152 (82.9%) | 112/152 (73.7%) |
| [0.8, 1.0] | 98 | 81/98 (82.7%) | **62/98 (63.3%)** |

At low severity (0.2–0.4), PX4's own detector is silent **every single
time** in this dataset — exactly the "genuinely stealthy" premise the
project's research question rests on. But that premise **does not hold
uniformly across the full `[0.2, 0.9]` range**: by the top severity bucket,
PX4 itself notices in over a third of episodes. This is worth carrying
into M7/M10 explicitly rather than treating `severity_range_s: [0.2, 0.9]`
as uniformly "PX4-blind" — the detection problem this project's own AI
detector needs to solve is genuinely harder (more informative) at low
severity and genuinely easier (PX4 already half-notices) at high severity,
which is itself a reasonable thing to report rather than a flaw to fix.

---

## Fault confirmation rate

77.9% of commanded faults were confirmed applied (the plugin's own status
echo matched the commanded severity within tolerance — CLAUDE.md's
"confirm, don't assume" requirement, not an assumption). The 132 faulty
episodes that were **not** confirmed break down as:

| termination_reason | count |
|---|---|
| `completed` | 125 |
| `preflight_failed` | 5 |
| `aborted_error` | 2 |

**Not a bug — a real, understood edge case, found first at small scale
during task 9's own verification and confirmed here at full scale.** A
fault's `onset_time_s` is sampled from `[5.0, 40.0]`; if the mission
*completes* (all waypoints + final hover) before that time is reached, the
fault genuinely never had a chance to onset — `completed` accounts for
nearly all (125/132) of the unconfirmed cases. `preflight_failed`/
`aborted_error` account for the rest: the fault can't apply during a flight
that never got airborne. None of these represent the plugin silently
failing to apply a fault that should have landed.

**By profile:**

| profile | n | confirmation rate |
|---|---|---|
| `ramp` | 298 | 74.2% |
| `step` | 299 | 81.6% |

Ramp's lower rate is a direct, expected consequence of how confirmation is
defined: `fault_confirmed_applied` requires the echoed severity to match
the **final target** severity within tolerance, and a ramp only reaches
that target after `ramp_duration_s`. A mission that completes partway
through a ramp is correctly *not* counted as "confirmed" even though the
fault was genuinely (partially) applied — a definitional consequence, not
evidence the ramp mechanism itself is unreliable (task 7's own dedicated
test confirms the ramp's velocity-scaling arithmetic is exact).

**By rotor index** — no systematic bias, as expected:

| rotor | n | confirmation rate |
|---|---|---|
| 0 | 147 | 81.0% |
| 1 | 160 | 77.5% |
| 2 | 163 | 77.3% |
| 3 | 127 | 75.6% |

---

## What this means for M7

*(M7 is done. It applied the points below with a finer inclusion rule: a
fault whose onset time fell after the mission ended is labelled healthy, and
ramps cut short are labelled at their instantaneous severity. See
`milestones.md` M7 task 1 and `docs/detector_results.md`.)*

- The 465 confirmed-applied faulty episodes (spanning the full severity/
  rotor/profile space above) plus the 153 healthy episodes are the dataset's
  real, trustworthy labelled examples. An unconfirmed faulty episode is not
  mislabelled (its `fault_severity_commanded` is still what was
  *requested*), but M7's training/eval split should treat
  `fault_confirmed_applied` as a filter, not ignore it — an unconfirmed
  "faulty" episode where the fault never actually onset is, physically, a
  healthy flight with a fault label that never took effect.
- `valid=False` episodes (9 `worker_restarted` placeholders) must stay
  excluded from training, same as any other milestone's convention.
- The severity-dependent FailureDetector-silence trend above is worth a
  sentence in M7's own writeup when discussing why detection difficulty is
  not uniform across the severity range.
