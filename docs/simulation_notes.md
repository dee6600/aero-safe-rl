# Simulation notes — M1

Measured on this machine (RTX 2070 Max-Q, see `docs/environment.md`) on
2026-08-20, using `scripts/sim_start.sh` / `scripts/sim_stop.sh`, PX4 v1.17.0,
Gazebo Harmonic 8.15.0, x500 quad, headless.

## How the multi-instance launch actually works (PX4 v1.17.0)

Worth recording since it is not obvious from the outside and shapes the M9
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
  with N spawned vehicles**, not N separate Gazebo servers.

> ⚠️ **Superseded 2026-08-20 — see [`parallelism.md`](parallelism.md).**
> The observation above is correct: that *is* what PX4 does by default. The
> conclusion drawn from it ("the shared physics server is one process to budget,
> not N") is wrong for this project. A shared world means one physics thread for
> all vehicles, one clock, one crash domain, and a **world-level** speed factor —
> `px4-rc.gzsim:154` applies `PX4_SIM_SPEED_FACTOR` through a
> `set_physics` service call on the world, so the last instance to start silently
> overrides every earlier one's speed factor.
>
> Decision **D7** now requires one isolated Gazebo server per worker, via a
> distinct `GZ_PARTITION`. Verified: two partitions produce two independent
> `gz sim -s` processes with independently honoured RTF. Milestone **M1b**
> reworks `sim_start.sh` accordingly.
>
> The RTF and flight-stability measurements below are unaffected — they were
> single-instance.

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
budgeted for M9, and is well above the 4× threshold that would have
triggered a re-plan per `milestones.md`'s M1 "watch out for" note — so no
change to the M9 plan or decision D5 is needed. (M8 renumbered to M9 on 2026-08-20.)

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

**A second, more subtle instance of the same bug was found and fixed**
(2026-08-20): with the GUI enabled (`--gui`), Gazebo runs as *two* separate
processes — the physics server (`gz sim --verbose=1 -r -s ...`) and a
separate GUI client (`gz sim -g`). `sim_stop.sh`'s original sweep pattern
only matched `--verbose`, so it killed the server but silently orphaned the
GUI process every time. Fixed by broadening the match to `^gz sim ` (anchored
to the start of the command line, so it can't accidentally match unrelated
processes). Confirmed clean on a full watch → verify → stop cycle afterward.

## Watching a flight

`scripts/sim_watch.sh` starts an instance with the Gazebo GUI on, flies an
arm/takeoff/hover/land sequence via a temporary MAVLink connection, and
prints a pass/fail checklist (`scripts/fly_demo.py` does the actual flying).
It targets the machine's real logged-in graphical session (`DISPLAY=:1`)
explicitly, since a remote/SSH shell has no `DISPLAY` of its own. If that
session is locked, the Gazebo window still exists and renders — you just
won't see it until you're at the machine and unlock the screen.
