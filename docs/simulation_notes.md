# Simulation notes — M1

Measured on this machine (RTX 2070 Max-Q, see `docs/environment.md`) on
2026-08-20, using `scripts/sim_start.sh` / `scripts/sim_stop.sh`, PX4 v1.17.0,
Gazebo Harmonic 8.15.0, x500 quad, headless.

## How the multi-instance launch actually works (PX4 v1.17.0)

Worth recording since it's not obvious from the outside and shapes the M8
parallel-training design later:

- `PX4_SIM_MODEL=gz_x500` + `PX4_GZ_WORLD=<world>` + `HEADLESS=1` on the
  first instance starts **one** `gz sim -s` (server-only, no GUI) process
  and spawns a model into it.
- A second instance (`px4 -i 1`) detects the already-running world via
  `gz topic -l` and **joins the same Gazebo process**, spawning its own
  model (`<name>_<instance>`) into it, rather than starting a second Gazebo
  server. Confirmed by instance 1's log: `gazebo already running world:
  default`.
- Passing `-i N` to the `px4` binary (built-in behavior, not something we
  implemented) automatically uses `build/px4_sitl_default/rootfs/<N>/` as
  the per-instance working directory, and offsets all internal MAVLink UDP
  ports by `N` (GCS link: `18570+N`, offboard: `14580+N`, etc.) — no manual
  port bookkeeping needed on our side. `ROS_DOMAIN_ID` isolation is a
  separate concern for M2, not addressed here.
- So "N parallel SITL instances" on this stack means **one Gazebo process
  with N spawned vehicles**, not N separate Gazebo servers. This matters for
  M8's resource planning — the shared physics server is one process to
  budget, not N.

## Real-time factor (RTF)

`PX4_SIM_SPEED_FACTOR` is a *request*; actual RTF was measured independently
by sampling `/world/default/stats` (Gazebo's own `real_time_factor` field)
over ~35 samples per run, discarding the first 5 as startup transient.

| Requested | Achieved (mean) | stdev | min | max |
|---|---|---|---|---|
| 1× | 1.000× | 0.0001 | 0.9996 | 1.0003 |
| 2× | 2.000× | 0.0005 | 1.9985 | 2.0010 |
| 4× | 4.035× | 0.209 | 3.998 | 5.252 |
| 8× | 7.883× | 0.361 | 6.293 | 8.026 |
| 16× | **8.348×** | 0.686 | 6.726 | 10.118 |

**Finding: this machine is compute-bound at roughly 8× real-time.** Requesting
16× does not produce 16× — it produces about the same ~8.3× as requesting 8×,
just with more jitter (stdev nearly doubles). 1× and 2× are essentially exact
and rock-solid; 4× starts showing minor jitter; 8×+ is where the ceiling
actually bites.

## Flight stability across speed factors

At each speed factor: armed, took off with `MIS_TAKEOFF_ALT=5`, held until
reaching ≥85% of target altitude (or timeout), landed, waited for disarm.
Validated via a temporary MAVLink connection (pymavlink, acting as a GCS
heartbeat source) plus the PX4 shell client (`px4-commander`) — no ROS 2
involved, keeping this milestone self-contained ahead of M2.

| Speed | Armed | Max alt reached | Hover altitude stdev | Landed | Failsafe seen |
|---|---|---|---|---|---|
| 1× | ✅ | 4.49 m | 0.069 m | ✅ | no |
| 2× | ✅ | 4.43 m | 0.052 m | ✅ | no |
| 4× | ✅ | 4.45 m | 0.061 m | ✅ | no |
| 8× | ✅ | 4.48 m | 0.070 m | ✅ | no |
| 16× (request) | ✅ | 4.41 m | 0.050 m | ✅ | no |

**Flight never became unstable at any tested speed factor** — hover altitude
stdev stayed in a tight 0.05–0.07 m band throughout, and PX4's own EKF/
failsafe system never complained, even at the "16×" request. This is
expected given the RTF finding above: since the actual achieved factor never
exceeds ~8.3× regardless of what's requested, we never actually stress-tested
flight dynamics beyond that point. The ceiling here is a **throughput**
ceiling, not a **stability** ceiling.

**Conclusion: maximum useful stable speed factor ≈ 8×.** This lands
comfortably inside the "expect 3–8×" range `planning.md` §7.4 already
budgeted for M8, and is well above the 4× threshold that would have
triggered a re-plan per `milestones.md`'s M1 "watch out for" note — so no
change to the M8 plan or decision D5 is needed.

## Two concurrent instances

Started instance 0 (pose `0,0,0,0,0,0`) and instance 1 (pose `3,0,0,0,0,0`)
back-to-back via `sim_start.sh`. Both came up cleanly:

- One shared `gz sim` process (confirmed via `pgrep`), two independently
  spawned models (`x500_0`, `x500_1`) in it.
- Distinct GCS MAVLink ports (18570 / 18571), confirmed from each instance's
  log.
- `px4-commander status --instance 0` and `--instance 1` both respond
  independently.
- Clean shutdown via `sim_stop.sh` (kills every tracked PID, then sweeps for
  any `px4`/`gz sim` process by name — a plain `kill` on the `px4` process
  does **not** cascade to the `gz sim` server it spawned, confirmed by
  testing; `sim_stop.sh` handles both explicitly).

## Orphan process handling

Confirmed during testing: killing only the `px4` process leaves `gz sim`
running in the background, silently consuming CPU — exactly the failure mode
`milestones.md` warns about. `scripts/sim_stop.sh` always kills both by name,
not just tracked PIDs, specifically to guard against this.
