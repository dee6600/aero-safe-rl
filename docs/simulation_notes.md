# Simulation notes — M1 / M1b

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

`scripts/watch_worlds.sh` starts N isolated workers with the Gazebo GUI on,
flies each one over ROS 2 (`aero_bridge.test_flight` — the same code path
the project actually flies with, no separate demo implementation), and
prints a pass/fail summary per worker. It targets the machine's real
logged-in graphical session (`DISPLAY=:1`) explicitly, since a remote/SSH
shell has no `DISPLAY` of its own. If that session is locked, the Gazebo
window still exists and renders — you just won't see it until you're at the
machine and unlock the screen.

(M1's original visual check flew over a temporary MAVLink connection,
before the ROS 2 bridge existed. Removed once `watch_worlds.sh` could do the
same job over the project's actual flight path instead of a second,
parallel one.)


---

# M1b — isolated workers (2026-08-20)

Re-measured after `sim_start.sh` was rewritten for one Gazebo server per worker
(`GZ_PARTITION`, `PX4_GZ_STANDALONE=1`). This supersedes the shared-world
behaviour described at the top of this file.

## Two concurrent workers, independent worlds

```
$ scripts/sim_start.sh -i 0 -s 4
$ scripts/sim_start.sh -i 1 -s 8
$ pgrep -af "^gz sim " | grep -c " -s "
2
$ scripts/sim_status.sh
INST PARTITION  NS       DOMAIN PORT   SYSID    px4/agent/gz           RTF
0    aero_0     px4_0    0      8888   1        up/up/up               4.00x
1    aero_1     px4_1    1      8889   2        up/up/up               5.26x
```

**Independent speed factors confirmed.** Worker 0 held exactly 4.00× while
worker 1 ran alongside it. Under the previous shared-world launcher the speed
factor was a world-level `set_physics` call, so starting worker 1 at 8× would
have silently changed worker 0 too.

Worker 1 requested 8× and achieved 5.26×. That is expected contention, not a
fault: two independent physics servers plus two PX4 stacks on 12 threads. The
single-worker ceiling of ~8.3× measured in M1 still stands. **Quantifying that
drop-off across worker counts is M4's throughput benchmark**, and the M9 sample
budget comes from that measurement rather than from extrapolating the
single-worker number.

## Identity verified live

```
ROS_DOMAIN_ID=0 ... /px4_0/fmu/out/vehicle_status_v1  ->  system_id: 1
ROS_DOMAIN_ID=1 ... /px4_1/fmu/out/vehicle_status_v1  ->  system_id: 2
```

Two things this proves, both previously only read from source:

- **`MAV_SYS_ID = instance + 1`.** This is the value a `VehicleCommand`'s
  `target_system` must match; a hardcoded `1` is silently dropped on instance 1.
- **The namespace override works.** Instance 0 publishes on `/px4_0/fmu/...`,
  not the bare `/fmu/...` that PX4 defaults to for instance 0 only. There is now
  one topic-naming code path for every worker.

## Isolated shutdown

`sim_stop.sh -i 1` stopped worker 1's three processes by recorded PID and left
worker 0 running and still at 4.00×. `sim_stop.sh --all` then exited 0 with no
`px4`, `gz sim` or `MicroXRCEAgent` processes remaining and an empty run
directory.

## Failure injection

Starting a worker whose uXRCE-DDS agent dies during startup: the launcher
detected the dead agent within the readiness loop, printed the port it was
trying to use, exited **1**, tore down the Gazebo server it had already started,
and left no `instance_2.json` behind. Previously a failure here would have
reported success and left an orphaned Gazebo server consuming a core.

## Notes

- Workers survive the exit of the shell that launched them (`setsid`), which is
  what makes them supervisable by a longer-lived process in M4.
- `results/sim_logs/` now also carries `gz_world_<N>_*.log`, since we own the
  Gazebo server process and can capture its output.
- The old `*.pid` files in the run directory are superseded by
  `instance_<N>.json`, which carries identity and PIDs together.
