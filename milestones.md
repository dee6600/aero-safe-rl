# Implementation Milestones

**Companion to `planning.md`.** That file explains *what* we are building and
*why*. This file is the build order: what to do, in what sequence, and how to
know each step actually works. `CLAUDE.md` holds the coding rules that apply to
every milestone; `docs/parallelism.md` holds the verified multi-instance facts.

Status: M0 (incl. addendum), M1, M1b, M3, M5, M6, M7 and M8 done. M4 substantially done
(one deferred sim-marked test, `tests/sim/test_worker_restart.py`, still
open — see M4). M2 substantially done —
both known multi-instance bugs fixed and verified; a third, subtler bug
found and fixed (px4_msgs timestamps are not simulated time,
`docs/parallelism.md` §2.5). One item deliberately left open: concurrent
two-worker flights hit a real, confirmed, unresolved
`offboard_control_signal_lost` reliability gap (§2.6) at a significant rate
(~35-65%), not caused by this project's own code; a partial mitigation
shipped, full resolution deferred to M4.

M3 also found two new, real PX4-SITL-under-Gazebo-Harmonic limitations
along the way: repeated `gz set_pose` teleports permanently trip PX4's
compass consistency check (mitigated), and `PREFLIGHT_REBOOT_SHUTDOWN`
(medium reset) never recovers post-flight (not usable, documented). Their
combined effect is that **soft reset was measured to leak state** (~10x the
run-to-run RMSE spread of hard reset) and medium reset doesn't work at all
post-flight — so M4's throughput budget should assume **hard reset**
(~19.8s) between episodes, not soft's ~0.6s, until one of the cheaper tiers
is fixed. Full detail: `docs/baseline_results.md`.

**Target pace:** roughly a week of active engineering to build all of
M3-M13's code. M9's training run and M10's evaluation sweep are separate,
unattended, wall-clock-bound jobs and are expected to run longer than that in
the background — see the timeline note at the end of M13.

---

**Revised 2026-09-21 — the simulator strategy changed (D12).** RL training
moves to a GPU-parallel **NVIDIA Isaac Lab** environment; PX4-in-the-loop
(Gazebo) stays the evaluation stack and the source of every reported number.
This reverses D6 and supersedes D5. Rationale and architecture:
`planning.md` §3.1 and the note on D12 in §14.

What this changes here:

- **M3b — Isaac Lab feasibility spike — ✅ done, passed comfortably.** Sample
  budget (the old plan's #1 risk) is resolved: 546k env-steps/s at the chosen
  operating point (8,192 parallel envs), no memory growth over a 10-minute
  sustained run. Host RAM, not GPU VRAM, turned out to be this machine's real
  constraint. Full numbers: `docs/isaac_feasibility.md`.
- **New M8b** — the Isaac Lab training environment.
- **M4 is rescoped** from a training farm to an *evaluation* farm — hundreds
  of episodes, 2 workers, not millions of steps. Its gate on M9 moved to M3b,
  which has now cleared it. This also defuses the open
  `offboard_control_signal_lost` issue.
- **M9** trains in Isaac, evaluates on PX4, and gains the RQ5 transfer table.
- **M5 and M6 gain one constraint each** — see those milestones.
- **Nothing measured so far is invalidated.** The M3 noise floor, the
  divergence band, and every multi-instance finding still stand, and the
  ROS 2 ↔ PX4 layer is untouched.

Last revised: 2026-09-21.

---

## How to use this file

- Work through milestones **in order**. Each depends on the one before it.
- Do not start the next milestone until the current one's **"Done when"** boxes
  all pass. Skipping a check means debugging it later mixed in with three other
  new problems, which costs far more time.
- Each milestone lists **files to create**. Do not create files that are not
  listed. Extra structure now is guesswork.
- Each milestone lists **tests**. A milestone with no tests is not done.
- **"Verify with"** gives the literal commands that prove the milestone. Run
  them; do not reason about whether they would pass.
- When something does not work, read **"Watch out for"** first — those are the
  failures this stack actually produces, not hypothetical ones.

### The three hard milestones

**M3b** (Isaac feasibility), **M6** (fault injection) and **M8b** (Isaac
training environment) are where this project can stall. M3b is short but is a
genuine go/no-go on D12 — this machine is below Isaac Sim's stated minimum.
M6 needs C++, and now needs its Isaac counterpart to match it exactly. M8b is
where the two simulators have to agree on what an observation and a fault
*mean*, which is subtler than either the RL or the physics. M9 itself got
easier under D12; the risk moved into M8b and into RQ5. Everything else is
plumbing that should go smoothly. Plan time accordingly.

---

## How to work with Claude Code on this repo

This section exists because the first attempts at this project produced bugs
concentrated in exactly one place: **anything involving more than one simulator
instance or more than one flight in sequence.** The causes were structural, not
random, and they are now fixed by process rather than by care.

### The five rules that prevent most of it

1. **One sub-task per session.** Every milestone below is split into numbered
   tasks, each sized for a single session with a single verification. Do not ask
   for "implement M4"; ask for "M4 task 3".
2. **Point at the contract, not the outcome.** Prompts below name the files that
   define the interface (`CLAUDE.md` §2, `docs/parallelism.md` §4, the schema
   files). Left to invent, Claude Code will invent something reasonable and
   different each time — and the divergence is what breaks the parallel case.
3. **Never accept single-instance verification for anything that will run in
   parallel.** The instance-0 special cases in PX4 (§2.2 of `docs/parallelism.md`)
   mean single-instance testing proves almost nothing. Every simulator-touching
   change is verified with **two concurrent instances** minimum.
4. **Demand the failing test first for any bug.** "It works now" after a fix,
   with no test, means the bug returns in three milestones.
5. **Use plan mode for M4, M6, M8b and M9.** These have architecture decisions inside
   them. Reviewing a plan costs ten minutes; reviewing 800 lines of wrong
   parallel code costs a day.

### Prompt template

Each milestone below ends with a ready-to-paste prompt. They all follow this
shape, and new ones should too:

```
Read CLAUDE.md, milestones.md (M<n>), and <specific reference doc>.

Implement M<n> task <k>: <one sentence>.

Constraints:
  - <the invariants this task must not break>
Deliverables:
  - <exact file paths>
  - <exact test file paths, and what each test asserts>
Verify by running:
  - <literal commands>
Do not: <the specific anti-pattern that would apply here>
```

### Things to avoid, ranked by how much time they have cost

1. Letting a node build its own topic name string. → §2.2 of `docs/parallelism.md`.
2. `time.sleep()` anywhere in flight logic. → breaks the moment speed factor ≠ 1.
3. A second implementation of arm/takeoff/land "just for this script".
4. Verifying multi-instance behaviour by reading code instead of running two.
5. Broad `pkill` cleanup that kills sibling workers.
6. Unbounded waits — an episode that hangs stops a 3-day training run.
7. Accepting a green end-to-end demo as evidence that the units are correct.

---

## Locked decisions

D1–D6 approved 2026-08-14/15. D7–D11 added 2026-08-20 after the multi-instance
review. These are settled — do not revisit them mid-build.

| ID | Decision |
|---|---|
| D1 | PX4 pinned to **`v1.17.0`** |
| D2 | Partial rotor faults via **our own gz-sim plugin** (`RotorDegradationSystem`) |
| D3 | Repo renamed to **`aero-safe-rl`** |
| D4 | Evaluation includes **C5** (perfect-detector upper bound) and **C6** (detector ablation) |
| D5 | ⛔ **SUPERSEDED by D12.** Was: simplified pre-training model deferred. Now adopted as the primary training path, realised as Isaac Lab. |
| D6 | ⛔ **SUPERSEDED by D12.** Was: Isaac Sim declined. Now partly reversed — Isaac is the *training* simulator; Gazebo remains the *evaluation* simulator and the source of every reported number. |
| **D7** | **One drone per world, always — settled (2026-08-20).** Every worker gets its own `GZ_PARTITION`-isolated Gazebo server. The original bug was PX4 sharing a world *silently, without anyone choosing it*; the fix is simply to never let that happen. Rationale and evidence: `docs/parallelism.md` §2.3, §3, §7. |
| **D8** | **We own the Gazebo server process** (`PX4_GZ_STANDALONE=1`), so a single worker can be stopped and restarted without touching its siblings. |
| **D9** | **Uniform instance identity, no special case for instance 0.** `PX4_UXRCE_DDS_NS=px4_<N>` for all N; `target_system = N+1` always; identity read from `instance_<N>.json`, never recomputed. |
| **D10** | **Sim time is the only clock in flight logic**, sourced from **`GzSimClock`** (`simulation/sim_clock.py`, Gazebo's native clock over gz-transport) — **not** `px4_msgs` timestamps, which were measured during M2 to track wall clock almost exactly regardless of speed factor. Wall clock is permitted solely in the watchdog. See `docs/parallelism.md` §2.5. |
| **D11** | **Reproducibility standard is statistical, not bitwise.** PX4 SITL + gz is not bitwise deterministic across runs; we fix seeds, report distributions over ≥N runs, and *measure* run-to-run divergence rather than asserting determinism. See M3 task 6. |
| **D12** | **Train in Isaac Lab, evaluate in PX4-in-the-loop (2026-09-21).** Supersedes D5 and D6. The two sides live in different conda environments and exchange files only — never imports (`CLAUDE.md` §0.1). Every reported number comes from the PX4 stack; Isaac produces the policy and the training curve, nothing else. Adds RQ5. Gated on M3b. |

**Why D2 changed shape.** Gazebo's stock motor plugin fixes its strength at model
load, so it cannot be changed live. PX4's `main` branch has a motor-failure
plugin, but it only does full failure and does not exist in v1.17.0. So we write
our own small Gazebo plugin that sits inside the physics loop and multiplies one
rotor's thrust by an efficiency number we can change at any moment from outside.
This is the honest way to simulate a weakening motor: **PX4 never learns about
it**, it only feels the aircraft behaving oddly — exactly the situation our
detector must handle.

**Why D7–D10 exist.** A review on 2026-08-20 read the pinned PX4 source and ran
two instances side by side. It found that PX4 silently shares one Gazebo world
between instances, that the speed factor is a world-level last-writer-wins
setting, that instance 0 gets different topic names from every other instance,
and that `VehicleCommand`s addressed to `target_system = 1` are silently dropped
on instances ≥ 1. Each of those produces a worker that starts cleanly and then
does nothing. All four are structural, so they are fixed by locked decision
rather than by remembering. Full evidence in `docs/parallelism.md`.

---

## Renumbering (2026-08-20)

A dedicated parallel-simulation milestone was inserted as **M4**. Everything
from the old M4 onward shifted by one. M0–M3 are unchanged.

| Old | New | Milestone |
|---|---|---|
| — | **M4** | **Parallel simulation farm + episode runner (new)** |
| M4 | M5 | Telemetry feature pipeline |
| M5 | M6 | Fault injection + dataset |
| M6 | M7 | AI fault detector |
| M7 | M8 | Rule-based recovery baseline |
| M8 | M9 | RL recovery policy |
| M9 | M10 | Full experiments + results |
| M10 | M11 | Generalization tests |
| M11 | M12 | Hexacopter extension |
| M12 | M13 | Paper + reproducibility package |

The new M4 sits before the feature pipeline deliberately: the first real
consumer of parallelism is dataset generation (M6, 500–1000 episodes), and
proving the farm against the already-working M3 mission is far cheaper than
discovering its bugs underneath a training run in M9.

---

## Milestone overview

| # | Milestone | Est. | Risk | First consumer of |
|---|---|---|---|---|
| M0 | Environment setup and pinning | 1 wk | Low | — |
| M1 | PX4 + Gazebo simulator running | 3 d | Low | — |
| M1b | Simulator addendum: isolation + ownership | 2 d | **Medium** | D7–D10 |
| M2 | ROS 2 talks to PX4 | 4 d | **Medium** | instance identity |
| M3 | Autonomous mission baseline + episode contract | 1 wk | Low | episode schema |
| M3b | **Isaac Lab feasibility spike** ✅ | 1 d | Low (resolved) | D12 — gates all Isaac work |
| M4 | **Parallel evaluation farm + episode runner** | 1 wk | Medium | everything above M3 |
| M5 | Telemetry feature pipeline | 4 d | Low | feature contract |
| M6 | Fault injection + dataset | 1.5 wk | **High** | the farm |
| M7 | AI fault detector | 2 wk | Medium | the dataset |
| M8 | Rule-based recovery baseline | 1 wk | Low | policy interface |
| M8b | **Isaac Lab training environment** ⭐ | 1 wk | **High** | the shared obs/action spec |
| M9 | RL recovery policy (train Isaac, eval PX4) | 2 wk | **High** | everything |
| M10 | Full experiments + results | 2 wk | Medium | metrics module |
| M11 | Generalization tests | 2 wk | Low | — |
| M12 | Hexacopter extension | 2 wk | Medium | — |
| M13 | Paper + reproducibility package | 3 wk | Low | — |

**First real result** (worth showing anyone) arrived with **M7** (2026-09-23): "our
detector spots a weakening motor that PX4 itself does not notice" —
`docs/detector_results.md`.
**First publishable result** arrives at the end of **M10**.

---
# M0 — Environment setup and pinning ✅

**Goal:** a working, recorded, repeatable toolchain, and an actual git repository.

Completed 2026-08-14. Full record in `docs/environment.md`. Summary: Gazebo
Harmonic 8.15.0 alongside Classic 11; conda env `aero-safe-rl` (Python 3.10.20)
with PyTorch 2.13+cu126, Gymnasium 1.3.0, Stable-Baselines3 2.9.0, NumPy 2.2.6;
PX4 pinned to `v1.17.0` on branch `aero-safe-rl`; `MicroXRCEAgent` in
`~/.local`; `px4_msgs` (`release/1.17`) and `px4_ros_com` vendored and built
(236 interfaces); `scripts/env_report.sh` + `docs/environment.md` written.

### M0 addendum — two gaps found 2026-08-20 (do before M2)

- [x] **`pytest` installed** (9.1.1) — every milestone from here needs unit tests.
- [x] **`pyarrow` installed** (25.0.1) for the Parquet episode records of
      `planning.md` §3; `pandas.to_parquet` fails without it.
- [x] **`pytest-timeout` installed** — a hang is a failure, not a stuck terminal.
- [x] `environment.yml` re-exported so the pin stays honest.
- [x] **`pytest.ini` added**, which turned out to be necessary rather than
      cosmetic — see the ROS plugin note below.

```bash
conda activate aero-safe-rl
pip install pytest pytest-timeout pyarrow
conda env export --no-builds > environment.yml
```

### Watch out for

- **`px4_msgs` version mismatch is the single most common way this stack
  breaks.** If message definitions do not match the PX4 build, topics still
  appear and still tick — but the numbers inside are garbage. Always verify
  actual values, not just that a topic exists.
- Do **not** `apt remove` Gazebo Classic. Nothing requires it.
- **Always `conda activate aero-safe-rl` before any Python work.** Never
  `pip install` into `base` or system Python.
- ROS 2 is system-wide at `/opt/ros/humble`, not in conda. You need both sourced
  (see `CLAUDE.md` §0). Verified working: conda Python 3.10.20 imports `rclpy`
  from `/opt/ros/humble` with numpy 2.2.6.
- **ROS 2 Humble's pytest plugins break pytest 8+.** This machine's shells put
  `/opt/ros/humble` on `PYTHONPATH`, so ROS ships seven `pytest11` entry points
  into every run. `launch_testing` still declares the `path` hook argument that
  pytest 8 removed, and aborts collection before a single test runs
  (`PluginValidationError`). None of them are used here, so `pytest.ini`
  disables them explicitly with `-p no:...`. **Do not "fix" this by downgrading
  pytest** — that trades a one-line config for an old test runner.

---

# M1 — PX4 + Gazebo simulator running ✅

**Goal:** start the simulator from a script, with no GUI, and measure how fast
it actually runs.

Completed 2026-08-20. `scripts/sim_start.sh` / `sim_stop.sh` written; RTF
measured at requested 1/2/4/8/16× (achieves ~8.3× ceiling, compute-bound, flight
stable at every tested factor); two concurrent instances started without port or
messaging conflicts. Numbers in `docs/simulation_notes.md`.

**One finding in `docs/simulation_notes.md` is now superseded.** It records that
"N parallel SITL instances on this stack means one Gazebo process with N spawned
vehicles" and treats that as a convenience. It is the default, but it is the
wrong architecture for this project — see M1b and `docs/parallelism.md` §2.3.
The note should be amended rather than deleted, since the observation itself was
correct.

---

# M1b — Simulator addendum: isolation, ownership, identity

**Goal:** make one worker a self-contained, independently startable and stoppable
unit, so that "run N of them" is arithmetic rather than an experiment.

**Why it matters:** every bug this project has hit in parallel operation traces
back to workers that are not actually independent. Fixing it here, against a
milestone that already works, is cheap. Fixing it inside M9 is not.

**Depends on:** M1. **Blocks:** M2 verification, M4 entirely.

### Tasks

1. ✅ **Rewrite `scripts/sim_start.sh` around the launch sequence in
   `docs/parallelism.md` §4.** Specifically:
   1. Derive all eight identity fields from the instance number, in one function.
   2. Export `GZ_PARTITION=aero_<N>` to both the Gazebo server and PX4.
   3. Source PX4's `rootfs/gz_env.sh`, then prepend this repo's
      `simulation/models` to `GZ_SIM_RESOURCE_PATH` (needed from M6 on; wire it
      now so M6 does not have to touch the launcher).
   4. Start `gz sim -r -s <world>.sdf` ourselves under `setsid`; record its PID.
   5. Start `MicroXRCEAgent`; record its PID.
   6. Start `px4 -i N -d` with `PX4_GZ_STANDALONE=1` under `setsid`; record PID.
   7. Export `PX4_UXRCE_DDS_NS=px4_<N>` **for every N including 0**.
2. ✅ **Replace the log-grep readiness check with a real one.** Wait for
   `/px4_<N>/fmu/out/vehicle_status_v1` on `ROS_DOMAIN_ID=<N>`, with a timeout
   and a non-zero exit on failure. A log line saying the startup script returned
   is not evidence the DDS link is up.
3. ✅ **Write `instance_<N>.json`** into the run directory (default
   `/tmp/aero-safe-rl-sim/<run_id>/`), containing `spec_version`, all eight
   identity fields, the three PIDs, world, model, model name, spawn pose, speed
   factor, log paths, and start time.
4. ✅ **Rewrite `scripts/sim_stop.sh -i N`** to kill only the three recorded PIDs
   (by process group), verify they are gone, and remove the instance file. The
   name-based sweep runs only under `--all`, and `--all` refuses to run if any
   instance file it does not own is present unless `--force` is given.
5. ✅ **Add `scripts/sim_status.sh`** printing one line per live instance from the
   instance files: instance, PIDs alive/dead, partition, domain, port, RTF.
6. ✅ **Amend `docs/simulation_notes.md`** with the shared-world finding and a
   pointer to `docs/parallelism.md`.

### Files created / changed

```
scripts/sim_start.sh          (rewritten)
scripts/sim_stop.sh           (rewritten)
scripts/sim_status.sh         (new)
scripts/activate.sh           (new — the two source lines, nothing else)
simulation/instance_spec.py   (new — the pure identity function, importable)
docs/simulation_notes.md      (amended)
```

`simulation/instance_spec.py` is the single source of truth for identity and is
imported by both the Python side and (via a tiny `--json` CLI) the shell side.
Two implementations of `8888 + N` is one too many.

### Tests (required)

```
tests/test_instance_spec.py
```

- `test_identity_is_pure` — same instance number always gives the same spec.
- `test_no_field_collides_across_instances` — for N in 0..7, every port, domain,
  partition, namespace, sys id and model name is unique.
- `test_mav_sys_id_is_instance_plus_one` — pins the `rcS` behaviour so a PX4
  bump that changes it fails a test instead of a training run.
- `test_namespace_is_uniform` — instance 0's namespace is `px4_0`, not empty.
  This is the test that would have caught the original bug.
- `test_spec_roundtrips_json` — written and re-read spec is identical.

### Done when

All verified 2026-08-20 (see `docs/simulation_notes.md` for the measured run):

- [x] `sim_start.sh -i 0` and `-i 1` produce **two** `gz sim` server processes
- [x] Each instance's RTF is independently settable and honoured — worker 0 held
      exactly 4.00× while worker 1 ran at 8× requested / 5.26× achieved. Under
      the old shared world the second start would have overridden the first.
- [x] `sim_stop.sh -i 1` leaves instance 0 running and healthy (still 4.00×)
- [x] `sim_stop.sh --all` leaves zero `px4` / `gz sim` / `MicroXRCEAgent`
- [x] Instances survive the exit of the shell that launched them
- [x] `instance_<N>.json` exists and matches `instance_spec.py`
- [x] Readiness check fails (non-zero exit) if the DDS link never comes up —
      exercised by occupying the worker's XRCE port; the launcher detected the
      dead agent, exited 1, and tore down its own processes leaving no orphans
- [x] `MAV_SYS_ID = instance + 1` confirmed live: instance 0 reports
      `system_id: 1`, instance 1 reports `system_id: 2`. This is the value
      `target_system` must match, and is the M2 bug made visible.
- [x] Namespace uniformity confirmed live: instance 0 publishes on
      `/px4_0/fmu/...`, not the bare `/fmu/...` PX4 would default to.

### Verify with

```bash
scripts/sim_start.sh -i 0 -s 4 && scripts/sim_start.sh -i 1 -s 8
pgrep -cf "^gz sim "                                    # expect 2
GZ_PARTITION=aero_0 gz topic -e -t /world/default/stats -n 1 | grep real_time
GZ_PARTITION=aero_1 gz topic -e -t /world/default/stats -n 1 | grep real_time
ROS_DOMAIN_ID=0 ros2 topic echo --once /px4_0/fmu/out/vehicle_status_v1
ROS_DOMAIN_ID=1 ros2 topic echo --once /px4_1/fmu/out/vehicle_status_v1
scripts/sim_stop.sh -i 1 && pgrep -cf "^gz sim "        # expect 1
scripts/sim_stop.sh --all && scripts/sim_status.sh      # expect nothing running
pytest tests/test_instance_spec.py -q
```

### Watch out for

- `GZ_PARTITION` must reach **both** processes. PX4 shells out to `gz service`
  and `gz topic`, and `gz_bridge` opens its own transport node — miss either and
  the instance silently joins the neighbour's world.
- Do not `sleep` and hope. Every wait polls a real condition with a deadline.
- Killing the PID is not enough; kill the process **group**, or the Gazebo
  server survives.
- The `NAV_DLL_ACT 0` step must stay: this project has no MAVLink GCS by design,
  and PX4 otherwise refuses to arm with "No connection to the GCS".

### Claude Code prompt

```
Read CLAUDE.md, docs/parallelism.md (all of it), and milestones.md M1b.

Implement M1b tasks 1-3: rewrite scripts/sim_start.sh to launch one fully
isolated worker, and add simulation/instance_spec.py as the single source of
truth for instance identity.

Constraints:
  - Follow the launch sequence in docs/parallelism.md §4 exactly.
  - GZ_PARTITION must be exported to both the gz sim process and the px4 process.
  - PX4_UXRCE_DDS_NS=px4_<N> for every N, including 0. No special case.
  - Readiness is a ROS topic check on the right domain with a timeout, not a
    log grep. Non-zero exit if it times out.
  - Both processes started with setsid; both PIDs recorded.

Deliverables:
  - scripts/sim_start.sh (rewritten), simulation/instance_spec.py
  - tests/test_instance_spec.py with the five tests listed in M1b
  - instance_<N>.json written to the run directory

Verify by running the M1b "Verify with" block, with two concurrent instances.
Report the actual output of `pgrep -cf "^gz sim "` — it must be 2.

Do not: recompute any identity field outside instance_spec.py; do not use
sleep as a readiness check; do not touch ~/projects/PX4-Autopilot.
```

---
# M2 — ROS 2 talks to PX4

**Goal:** read telemetry from PX4 and send commands to it, entirely from Python,
**on any instance number**.

**Why it matters:** this is the road every later component drives on. If it is
shaky, everything above it is shaky. The phrase "on any instance number" is the
whole point — code that only works on instance 0 passes every test written today
and fails silently in M4.

**Depends on:** M1b. **Blocks:** M3, and everything after.

### Two known bugs in the current `px4_interface.py` — fix these first

Both were found by reading the pinned PX4 source on 2026-08-20. Both are silent.

1. **`target_system` is hardcoded to `1`** (`px4_interface.py`, in
   `publish_vehicle_command`). `Commander.cpp:746` drops any command whose
   `target_system` is neither `0` nor this vehicle's `MAV_SYS_ID`, and
   `MAV_SYS_ID = instance + 1`. On instance 1, arm/offboard/land are ignored with
   no error. → take the instance from the spec and send `instance + 1`.
2. **Topic names are hardcoded to `/fmu/out/...`.** Instance 1 publishes to
   `/px4_1/fmu/out/...`. The subscription succeeds and never receives anything.
   → build every topic name from the instance namespace.

Details and evidence: `docs/parallelism.md` §2.1, §2.2.

### Tasks

1. **Make `PX4Interface` instance-aware.**
   1. Constructor takes an `InstanceSpec` (from `simulation/instance_spec.py`).
   2. All topic names built as `f"/{spec.topic_ns}/fmu/out/{name}"` by one
      helper. No literal topic strings anywhere else in the repo.
   3. `target_system = spec.mav_sys_id`.
   4. Keep the NED comment at the top of the file — it earns its place.
2. **Confirm the nine telemetry topics carry believable values.** Not that they
   exist — that attitude changes when the vehicle tilts, motor outputs rise on
   takeoff, battery falls, and `timestamp` advances monotonically. Write this as
   a `sim`-marked test, not a manual check.
   `VehicleOdometry`, `VehicleAttitude`, `SensorCombined`, `ActuatorMotors`,
   `ActuatorOutputs`, `VehicleStatus`, `BatteryStatus`, `FailsafeFlags`,
   `EstimatorStatusFlags`.
   Note the `_v1` suffix on version-bumped messages (`vehicle_status_v1`,
   `battery_status_v1`) — the suffix is real and easy to miss.
3. **Confirm the three command topics work**: `OffboardControlMode`,
   `TrajectorySetpoint`, `VehicleCommand`.
4. **Add a `PX4Clock` helper**, sourced from **`simulation/sim_clock.py`'s
   `GzSimClock`** (Gazebo's own clock, read directly over gz-transport) --
   **not** from PX4 message timestamps. Found during implementation:
   `uxrce_dds_client` resynchronizes every published timestamp to the
   agent's wall clock, so `px4_msgs` timestamps track real time almost
   exactly regardless of speed factor (measured ratio 0.991 at requested
   4x) and are useless for this. `GzSimClock` gave the correct ratio (3.945
   at the same speed factor) in the same experiment. See
   `docs/parallelism.md` §2.5. `PX4Clock.sleep_sim(dt)` waits on `GzSimClock`
   advancing, bounded by a wall-clock safety deadline. **Every wait in every
   later milestone uses this.** This one class is what stops speed-factor
   bugs from ever appearing -- and very nearly didn't, since the obvious
   first implementation (message timestamps) is wrong in a way that looks
   completely plausible until measured.
5. **Write `arming_sequence.py`**: the one implementation of stream setpoints →
   engage offboard → arm → confirm armed, with deadlines and typed failures
   (`ArmTimeout`, `OffboardRejected`, `PreflightFailed`). Everything that flies
   calls this. Nothing re-implements it.
6. **Write the takeoff → hover 5 m → land node** on top of tasks 4 and 5,
   ~40 lines because the hard parts are already factored out.
7. **Keep `scripts/measure_latency.py`**, updated for the instance spec. Its
   `CLOCK_MONOTONIC` comparison is correct and worth preserving — record the
   reasoning in `docs/environment.md`.
8. **Confirm the `dds_topics.yaml` patch is still needed and applied.**
   `simulation/patches/0001-expose-actuator-motors-outputs-over-dds.patch` adds
   `/fmu/out/actuator_motors` and `/fmu/out/actuator_outputs`, which M5's most
   important feature depends on. Add a check to `scripts/env_report.sh` that
   fails loudly if the patch is not applied to the current PX4 build.

### Files created / changed

```
ros2_ws/src/aero_bridge/aero_bridge/px4_interface.py   (made instance-aware)
ros2_ws/src/aero_bridge/aero_bridge/px4_clock.py       (new)
ros2_ws/src/aero_bridge/aero_bridge/arming_sequence.py (new)
ros2_ws/src/aero_bridge/aero_bridge/test_flight.py     (thinned to a caller)
simulation/sim_clock.py                                (new -- GzSimClock, the real sim-time source)
scripts/measure_latency.py                             (instance-aware)
scripts/env_report.sh                                  (patch check + gz bindings check added)
```

### Tests (required)

```
tests/test_px4_interface.py     (no sim — topic-name and command construction)
tests/test_px4_clock.py         (no sim — fake message stream)
tests/test_arming_sequence.py   (no sim — fake px4/clock, incl. the reengage fix)
tests/sim/test_link.py               (@pytest.mark.sim)
tests/sim/test_sim_clock.py          (@pytest.mark.sim)
tests/sim/test_telemetry_sanity.py   (@pytest.mark.sim)
```

- `test_topic_names_follow_namespace` — for instances 0 and 3, every one of the
  nine subscriptions and three publishers resolves to the namespaced name.
  **This is the test that catches the hardcoded-topic bug.**
- `test_target_system_matches_instance` — instance 3 → `target_system == 4`.
  **This is the test that catches the `target_system` bug.**
- `test_command_message_fields` — arm/disarm/offboard/land build the right
  command ids and params (checked against `VehicleCommand` constants).
- `test_clock_uses_message_time` — with a synthetic stream at 8× rate,
  `sleep_sim(1.0)` returns after 1.0 s of *message* time, not 1.0 s wall.
- `test_clock_deadline_raises` — no messages arriving raises rather than hangs.
- `test_hold_position_until_reengages_offboard_when_lost` — the test that
  would catch a regression removing the §2.6 offboard re-engage fix.
- `test_gz_sim_clock_tracks_speed_factor` (sim) — `GzSimClock` against a real
  worker at speed factor 4 gives a ratio near 4, and unambiguously not near 1
  (which is the wrong-but-plausible answer `px4_msgs` timestamps give).
- `test_link_alive[instance=0,1]` (sim) — both instances arm, take off, land.
  Parametrised over two instances **running concurrently**; this is the gate,
  and is the one item M2 does not fully close — see `docs/parallelism.md` §2.6.
- `test_telemetry_values_are_believable` (sim) — flies instance 1 for real and
  asserts on the numbers, not just presence: altitude climbs near the hover
  target, the attitude quaternion changes and stays unit-norm, the
  accelerometer reads real physics, `actuator_motors`/`actuator_outputs` rise
  from a disarmed baseline once armed, battery `remaining` never rises,
  `vehicle_status` passes through ARMED, no hardware-failure flag fires, and
  the estimator reports itself aligned by the end. **This is the test task 2
  asked for and the milestone originally shipped without** — the "Done when"
  box below used to point at a no-sim topic-naming test plus an unrecorded
  manual flight, neither of which actually checks a telemetry value.

### Done when

All verified 2026-08-20/21, plus the telemetry-values gap closed 2026-08-21
(see `docs/parallelism.md` §2.6 for the one open item):

- [x] All nine telemetry topics carry believable, changing values — asserted
      live by `tests/sim/test_telemetry_sanity.py` (3/3 clean runs against a
      real worker), not merely by topic-naming tests or an unrecorded manual
      flight as originally checked off. While calibrating it, one more
      symptom of the §2.6 DDS transport issue turned up: during a transient
      `offboard_control_signal_lost`/re-engage, `px4_msgs` message timestamps
      can jitter backward by a few ms for a couple hundred samples, and once
      a single message arrived carrying the raw un-synced clock instead of
      the wall-clock-resynced value. Not re-investigated further — it's
      consistent with, and doesn't change, §2.6's existing open status — but
      the test asserts timestamp advancement via head/tail medians rather
      than strict per-message monotonicity because of it.
- [x] A Python node flies takeoff → hover → land with no manual steps
- [x] The same node flies instance 1 with no code change, only a different spec
- [x] Telemetry latency measured and recorded — instance 1, 978 samples:
      mean 7.15ms, median 6.29ms, p95 8.76ms, stdev 1.29ms (single-digit ms, as
      predicted)
- [x] The flight works 5 times in a row without restarting the simulator —
      verified 5/5 clean on a fresh single-instance worker
- [ ] **Two instances fly simultaneously, each reaching its own target
      altitude** — works, but not reliably: ~35-65% of concurrent two-worker
      flights hit a real, confirmed, currently-open reliability gap
      (`offboard_control_signal_lost`, not caused by this project's own
      code — see `docs/parallelism.md` §2.6). A partial mitigation shipped
      (offboard re-engage on loss); full resolution is deferred to M4. Left
      unchecked deliberately rather than marked done — the underlying issue
      is real and unresolved, not merely a flaky test.
- [x] No `time.sleep()` remains in any flight path (the one `time.sleep`
      reference in `px4_clock.py` is a documented, never-used-in-flight-code
      fallback default for standalone/test usage)

### Verify with

```bash
pytest tests/ -m "not sim and not slow" -q
scripts/sim_start.sh -i 0 && scripts/sim_start.sh -i 1
pytest tests/sim/test_link.py -q                       # both instances
python scripts/measure_latency.py --instance 1 --seconds 15
grep -rn "time.sleep\|/fmu/out/" ros2_ws/src/aero_bridge/ | grep -v px4_interface.py
# ^ must return nothing
scripts/sim_stop.sh --all
```

### Watch out for

- **Offboard mode requires a steady setpoint stream before you switch into it,
  and continuously afterwards.** Send at ≥ 20 Hz. If the stream stutters, PX4
  drops out of offboard — and it does so quietly. Watch `nav_state`, not your
  own assumption about what mode you are in.
- If topics appear but values look wrong or frozen, suspect the `px4_msgs`
  version first (M0).
- **PX4 uses NED.** Down is positive, so 5 m up is `z = -5.0`. Sign errors here
  are constant; the comment at the top of `px4_interface.py` stays.
- QoS must match PX4's publisher (`BEST_EFFORT`, `TRANSIENT_LOCAL`, `KEEP_LAST`,
  depth 1). A mismatch gives you a subscription that never fires — the same
  symptom as the namespace bug, from a different cause. Check both.
- Arming can fail for preflight reasons that have nothing to do with your code
  (EKF not converged, home position not set). Surface the actual reason from
  `FailsafeFlags` / preflight messages rather than reporting "arm timeout".
- **Offboard mode can drop even when the application never misses a publish
  deadline — a real, currently-open reliability gap.** Full investigation in
  `docs/parallelism.md` §2.6. Short version: PX4 can report
  `offboard_control_signal_lost` (and act on it, exiting OFFBOARD) at a rate
  of ~10-20% for a solo worker and ~35-65% for two concurrent workers, even
  though the publish loop was instrumented and never once missed a scheduled
  setpoint — the loss happens somewhere in the BEST_EFFORT DDS transport
  (`MicroXRCEAgent`/`uxrce_dds_client`), not in application code. Two wrong
  hypotheses were tested and ruled out with `.ulg`-log evidence before this
  was found: PX4's battery failsafe (a misleadingly-named log line) and
  `GzSimClock`'s background gz-transport thread. `hold_position_until` now
  re-engages offboard on loss (roughly halves the concurrent failure rate,
  does not eliminate it). The real fix belongs to M4 (`WorkerSupervisor`:
  detect, invalidate the episode, retry) or further transport-level
  investigation. **Do not re-diagnose from scratch — read §2.6 first.** A
  failure with a *different* signature (not `offboard_control_signal_lost`)
  is a real, different bug.

### Claude Code prompt

```
Read CLAUDE.md, docs/parallelism.md §2, and milestones.md M2.

Implement M2 tasks 1, 4 and 5: make PX4Interface instance-aware, add PX4Clock,
and add arming_sequence.py.

Two known bugs to fix in px4_interface.py, both silent:
  1. target_system is hardcoded to 1; it must be spec.mav_sys_id (= instance+1),
     because Commander.cpp:746 drops mismatched commands with no error.
  2. Topic names are hardcoded to /fmu/out/...; instance N>0 publishes to
     /px4_N/fmu/out/.... Build all names from spec.topic_ns.

Constraints:
  - PX4Interface takes an InstanceSpec; no other file may contain a literal
    /fmu/ topic string.
  - PX4Clock is the only way anything waits. No time.sleep() in flight logic.
  - Every wait has a wall-clock safety deadline and raises a typed exception.

Deliverables:
  - px4_interface.py (modified), px4_clock.py, arming_sequence.py
  - tests/test_px4_interface.py and tests/test_px4_clock.py with the tests
    listed in M2 — write the two bug-catching tests FIRST and show them failing
    against the current code before fixing it.

Verify: pytest tests/ -m "not sim and not slow" -q, then the M2 "Verify with"
block with instances 0 and 1 running concurrently.

Do not: add a second arming implementation anywhere; do not use sleep; do not
special-case instance 0.
```

---

# M3 — Autonomous mission baseline + episode contract ✅

**Goal:** a repeatable autonomous mission that becomes the "healthy" condition in
every experiment, and the one episode-record format everything else reads and
writes.

**Why it matters:** every later result is measured against this. We also need to
know how much results wobble run-to-run *with no fault at all* — the noise floor.
A faulty run that differs by less than the noise floor has told us nothing.

The episode record is here rather than later because M6 (dataset), M7
(detector), M9 (RL) and M10 (metrics) all read it. Defining it once, with a
schema and a validator, is what stops four incompatible log formats.

**Depends on:** M2. **Blocks:** M4, M5, M6.

### Tasks

1. **Define the episode record schema** in `configs/schema/episode_record.yaml`:
   `schema_version`, run/episode/worker ids, seed, instance spec digest, mission
   id, config digests, per-step columns, per-episode summary columns,
   `termination_reason` (closed enum), `valid` flag, `t_sim` and `t_wall`,
   feature version, and the software versions from `env_report.sh`.
   Write the validator alongside it. Every writer validates before writing.
2. **Mission executor node**: takeoff → waypoints → hover → land, offboard,
   built on M2's `arming_sequence` and `PX4Clock`.
3. **Mission definitions in YAML** (`configs/missions/`). Start with **one**
   primary mission: a square or figure-8 circuit, 20–40 m across, ~60 s. Include
   acceptance radius, hold times, geofence, and altitude limits in the file, not
   in code.
4. **Episode logger**: one row per control step plus one summary row per episode,
   written under `results/<run_id>/worker_<k>/`. One writer per file.
5. **Reset ladder — build all three tiers and measure each.** Reset is the
   hidden cost that multiplies across a million RL steps, and the cheapest tier
   is not always safe.
   1. **Soft** — reposition the model via the gz `set_pose` service, reset the
      EKF, clear integrators, re-arm. Fastest. Must be *proved* not to leak
      state (task 6).
   2. **Medium** — `VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN` to PX4, keep the same
      Gazebo server. Slower, cleaner.
   3. **Hard** — full worker restart via `sim_stop.sh` + `sim_start.sh`.
      Slowest, always correct. This is the supervisor's fallback in M4.
   Measure wall-clock cost of each and record it in `docs/baseline_results.md`.
6. **Prove the soft reset does not leak state.** Run 20 episodes with soft reset
   and 20 with hard reset, same seeds, and compare the distributions of initial
   EKF innovations, initial position error, and mission RMSE. If they differ, the
   soft reset is unusable and M4/M9 budget from the medium tier instead. **This
   is a required experiment, not an assumption** — leaked state biases training
   in a way that is invisible until the results are wrong.
7. **Quantify run-to-run divergence (D11).** Run the same mission with the same
   seed 20 times and report the spread of position RMSE, final position, and
   flight time. This defines what "reproducible" means for the rest of the
   project and replaces the bitwise-determinism assumption.
8. **Run 20 healthy missions and analyse the spread.**

### Files created

```
configs/schema/episode_record.yaml
configs/missions/square_circuit.yaml
experiments/episode_schema.py                (loader + validator)
ros2_ws/src/aero_bridge/aero_bridge/mission_executor.py
ros2_ws/src/aero_bridge/aero_bridge/episode_logger.py
ros2_ws/src/aero_bridge/aero_bridge/reset.py
experiments/run_episodes.py
docs/baseline_results.md
```

### Tests (required)

```
tests/test_episode_schema.py
tests/test_mission_config.py
tests/test_episode_logger.py
tests/sim/test_reset.py         (@pytest.mark.sim)
```

- `test_valid_record_passes` / `test_missing_field_fails` / `test_unknown_
  termination_reason_fails` — the schema actually rejects bad records.
- `test_schema_version_recorded` — every written record carries the version.
- `test_mission_yaml_validates` — the shipped mission parses and its waypoints,
  geofence and limits are self-consistent (geofence contains all waypoints).
- `test_logger_row_count` — N control steps produce N rows plus one summary.
- `test_logger_one_writer_per_file` — two logger instances with different worker
  ids never target the same path.
- `test_termination_reason_enum_closed` — the enum in code and in the schema file
  are identical (catches drift).
- `test_reset_returns_clean_state` (sim) — after a soft reset, position is within
  tolerance of spawn, velocity ≈ 0, and the vehicle is disarmed.

### Done when

All verified 2026-08-21 against a live worker (`docs/baseline_results.md`
has every number and both raw run directories):

- [x] 20 out of 20 healthy missions complete successfully — two runs, 20/20
      each (`m3_healthy_20` needed 2 retries out of 22 attempts, both
      auto-recovered by escalating to hard reset and both logged rather
      than hidden; `m3_reset_hard_20` needed zero retries)
- [x] Position RMSE recorded for all 20; **noise floor documented** — mean
      6.44m, std 0.57m over 40 completed episodes (`docs/baseline_results.md`
      Task 8)
- [x] Run-to-run divergence at fixed seed quantified (task 7) — reported per
      reset tier since they turned out not to be interchangeable; hard
      reset's spread (RMSE std 0.083m) is the recommended reproducibility
      tolerance going forward
- [x] All three reset tiers implemented, and each one's cost measured — soft
      0.59s, medium ~0.5s *when it works*, hard 19.78s
- [x] Soft reset proved equivalent to hard reset, or documented as unusable
      — **not equivalent**: same mean performance, but ~10x the run-to-run
      RMSE spread and ~58x the flight-time spread, consistent with PX4
      controller/EKF state that soft reset's position/velocity checks
      cannot see. Documented, not hidden — see Task 6 in
      `docs/baseline_results.md`, including the consequence for M4's
      throughput budget (medium is separately unusable post-flight, so M4
      should budget from hard reset until one of the cheaper tiers is
      fixed).
- [x] Every episode record validates against the schema — every writer goes
      through `EpisodeLogger`, which validates before writing; zero
      `SchemaValidationError`s across all 42 completed-run episodes
- [x] Logs have no missing rows and no gaps in `t_sim` — checked directly:
      zero non-sequential `step_index` values across all 20
      `m3_reset_hard_20` episodes, `t_sim` deltas consistent with the 10Hz
      control rate (mean ~0.11s, max ~0.3s jitter, no discontinuities)

**Also found and fixed along the way, not originally anticipated by this
milestone's task list:** repeated `gz set_pose` teleports between episodes
permanently trip PX4's magnetometer consistency check ("Compass 0 fault"),
and `VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN` (medium reset) never recovers
`pre_flight_checks_pass` once a PX4 instance has actually armed and flown
(tested to 75s). Both are real PX4-SITL-under-Gazebo-Harmonic limitations,
not bugs in this project's own code. Full detail and mitigations in
`docs/baseline_results.md`'s Task 5/6 sections and
`aero_bridge/reset.py`'s module docstring.

### Verify with

```bash
pytest tests/ -m "not sim and not slow" -q
python experiments/run_episodes.py --mission square_circuit --n 20 --instance 0
python experiments/analysis/noise_floor.py results/<run_id>/    # prints the table
pytest tests/sim/test_reset.py -q
```

### Watch out for

- If any of the 20 healthy runs fails, **stop and fix it**. An unreliable
  baseline makes every later comparison meaningless. "19 of 20, the other one
  was a fluke" is how a project gets six months in before noticing.
- Reset is easy to get subtly wrong. Leftover EKF bias, integrator windup, or
  position drift leaks into the next episode and quietly corrupts training. Task
  6 exists because "it looks fine" is not evidence.
- Do not let the mission executor own its own timing. It uses `PX4Clock`.
- Waypoint acceptance radius interacts with the speed factor: a radius tuned at
  1× may be flown straight through at 8×. Verify the mission at the speed factor
  you will actually train at.

### Claude Code prompt

```
Read CLAUDE.md, milestones.md M3, and configs/schema/ if it exists.

Implement M3 tasks 1 and 4: the episode record schema plus its validator, and
the episode logger that writes against it.

Constraints:
  - schema_version, feature_version, seed, worker id, config digests and both
    t_sim and t_wall are mandatory on every record.
  - termination_reason is a closed enum defined once and shared between the
    schema file and the Python code; a test asserts they match.
  - One writer per file; the path includes run id and worker id.
  - The logger validates before writing, and fails loudly rather than writing an
    invalid record.

Deliverables:
  - configs/schema/episode_record.yaml, experiments/episode_schema.py,
    aero_bridge/episode_logger.py
  - tests/test_episode_schema.py, tests/test_episode_logger.py with the tests
    listed in M3

Verify: pytest tests/ -m "not sim and not slow" -q

Do not: invent a second log format anywhere else; do not write records that
skip validation "for speed".
```

---
# M3b — Isaac Lab feasibility spike ✅

**Goal:** find out by measurement whether Isaac Lab runs usefully on this
machine, **before** any part of the plan depends on it.

**Why it is a milestone and not a footnote:** Isaac Sim 5.1's stated minimum is
an RTX 4080 / 16 GB VRAM / 32 GB RAM. This machine is an RTX 2070 Mobile
(Turing, 8 GB) with 15 GB RAM — below spec on all three. Turing has RT cores so
it is not excluded outright, and headless physics-only is the cheapest workload
Isaac can be given, but that is a reason for optimism, not a number. D12 without
this measurement is a guess, and the wrong place to discover it is M9.

**Depends on:** a working CUDA install. **Blocks:** M8b, M9.

**Result: PASSED, comfortably.** Full numbers, method, and reasoning:
`docs/isaac_feasibility.md`. Summary — Isaac Sim 5.1.0 launches headless and
runs the stock `Isaac-Quadcopter-Direct-v0` task; a throughput sweep from 64 to
32,768 parallel envs found VRAM was never close to the 8 GB ceiling (peak
6.8 GB at 32,768) and **host RAM, not GPU memory, is this machine's actual
constraint** (peak 13.6 GB of 15.8 GB at 32,768). Chosen operating point:
**8,192 envs** — 546k env-steps/s, 41% VRAM, 41% RAM, chosen with headroom to
spare for the policy network/optimizer/rollout buffer a real PPO loop adds on
top of pure physics stepping (not yet measured — see M8b follow-up below). A
10-minute sustained run at that operating point (40,000 steps) showed **no
memory growth** (peak RSS 6,437.7 MB vs. 6,439.3 MB in a 4.5-second burst test
of the same config) and no errors. The gate — deliver ~1M steps within a few
hours — clears in **under 2 seconds** of raw simulation time at the chosen
operating point. Sample budget, the old plan's #1 risk, is resolved.

One real blocker hit and fixed along the way: Isaac Sim's first import prompts
an interactive EULA acceptance that hangs forever non-interactively
(`Unable to bootstrap inner kit kernel: EOF when reading a line`). Fixed with
`export OMNI_KIT_ACCEPT_EULA=YES` before any Isaac invocation — needs a home in
an Isaac counterpart to `scripts/activate.sh` before the next session hits it.

**What was and wasn't tested at 10-minute duration.** The sustained stability
run was at 8,192 envs (the chosen operating point), not at the largest count
that completed without error (32,768, tested only as a short burst). This is
intentional, not an oversight — 32,768 leaves too little RAM headroom once a
real training loop's memory is added on top of physics-only stepping, so it
was never a candidate operating point regardless of its burst-test result.

**Key deliverables — status:**
- ✅ Isaac Sim 5.1.0 launches headless and steps a physics scene with no rendering.
- ✅ Isaac Lab installed (cloned to `~/projects/IsaacLab`, sibling to
  `PX4-Autopilot` — not inside this repo, matching the project's existing
  convention for vendored external dependencies); stock quadrotor task run.
- ✅ Measured table: env count ∈ {64, 256, 1024, 4096, 8192, 16384, 32768} ×
  {steps/sec, VRAM, host RAM} — wider than the originally planned {64, 256,
  1024, 4096}, extended because throughput kept climbing well past 4096 and
  the real ceiling turned out to matter for choosing the operating point.
  Largest count run without error: 32,768. Largest count stability-tested for
  10 minutes: 8,192 (see above for why these differ).
- ✅ `docs/isaac_feasibility.md` written, stating the chosen operating point.

**No new unit tests for this milestone.** Unlike M5–M9, M3b introduces no
pure-function logic of this project's own — it is a measurement spike against
a third-party simulator, and its only deliverable is the measured table and
the written record of it. The standing "every milestone ships tests" rule
applies to logic this project owns; there is none here to test.

**Follow-ups carried into M8b** (recorded in `docs/isaac_feasibility.md` too):
commit the benchmark scripts into `isaac/` if M8b needs to re-run the sweep;
re-measure memory headroom once the real training loop's network/optimizer/
buffer exist, since this document's numbers are physics-only; give
`OMNI_KIT_ACCEPT_EULA=YES` a permanent home.

---

# M4 — Parallel evaluation farm + episode runner  ⭐ new

**M4 substantially complete (2026-09-21/22).** All 8 tasks done except one
deliberately-deferred sim-marked test (below). `EpisodeRunner`,
`WorkerSupervisor` and `SimFarm` exist and are verified against two real
concurrent workers, including `pgrep -cf "^gz sim "` reading 2 live during
the run. Task 4 (structured failure handling) closed all four of its
sub-items: `sim_fault` detection, invalid-record synthesis for a
worker-restart-lost episode, and the run-level restart-rate abort. Task 5
(run manifest) closed, with `SimFarm.run()` gaining `on_result`/`on_restart`
progress callbacks at the same time. **Task 6 (throughput sweep) ran for
real**, a full 16-configuration sweep — chosen operating point **2 workers**
(worker count; the speed-factor part of that recommendation was later
corrected, see task 7); see `docs/throughput.md` for the full table and the
reliability cliff found at worker_count ≥ 3. Task 8 (CPU affinity) is
done-as-not-needed, decided from that sweep's real CPU numbers (never
saturates). **Task 7 (soak test) passed for real, 2026-09-22**, after a
five-step real investigation spanning a rescope (4→2 workers), a genuine bug
fix (heartbeat-before-reset), a correction to task 6's own speed-factor
recommendation (4x looked clean at small sample size but mostly didn't
complete missions at soak scale; reverted to 1x), and an evidence-based
restart-budget increase (5→20) — final result: 400/400 episodes, zero
restarts, zero orphans, flat memory, ~3h unattended. Full story in task 7's
own writeup below. Only `tests/sim/test_worker_restart.py` (a real `kill -9`
against a live simulator, task 4) is still open, deliberately deferred to
avoid a session that needs a live sim.

**A third real bug found and fixed while running the throughput sweep for
the first time**, on the very first real hard reset the session ever
exercised: `WorkerSupervisor.is_healthy()` treated a missing
`instance_<N>.json` as "worker is dead" unconditionally, but a worker's own
child process legitimately has no instance file for ~20s while it runs its
own internal `hard_reset()` (triggered in-child after a bad episode). The
parent's health-poll loop (every 2s by default) landed inside that ~20s
window, saw "no PID file", and restarted the SAME worker a second time —
racing the child's own in-flight reset. PX4 refused the second start
("server already running", confirmed via PX4's own source: a real `fcntl`
lock a live process holds, not a stale file), the resulting
`WorkerSupervisorError` propagated unhandled and crashed the whole sweep,
and both restart attempts' processes were left running, untracked. Fixed by
having `is_healthy()` trust a *live child's* heartbeat over raw PID
presence (heartbeat staleness already tolerates hard reset's duration, and
still catches a genuinely stuck child); PID liveness is now only consulted
when there is no live child to trust instead. Also hardened
`ensure_healthy()` so a failed restart *attempt* (for any reason) is
reported and retried on the next poll cycle within the existing restart
budget, rather than crashing the run with an unhandled exception. Three new
regression tests in `tests/test_supervisor_state_machine.py` pin this down.
See the per-task marks below and the progress log entry for full detail,
including two other real bugs found and fixed along the way: a stale
heartbeat file surviving across runs
caused a spurious restart and a duplicated episode, and an unrelated
pre-existing `.gitignore` bug (its unanchored `env/` rule also silently
matched `configs/env/`, hiding this task's own `configs/env/farm.yaml` from
every future commit) was found and fixed while adding that file.

> **Rescoped 2026-09-21 (D12).** This was a *training* farm: millions of steps,
> 4 workers, gating whether M9 was possible. Training moved to Isaac, so this is
> now an **evaluation** farm — hundreds of episodes per condition cell. What
> changes: 2 workers is a reasonable default rather than 4; the M9 gate moves to
> M3b; and the unresolved `offboard_control_signal_lost` issue
> (`docs/parallelism.md` §2.6, ~35–65 % at two concurrent workers) becomes a
> retry-and-record case that this milestone's `WorkerSupervisor` already handles
> by design, instead of a threat to a multi-day training run. Still build the
> throughput table — M10's sweep is budgeted from it — but it is no longer a
> project-level gate. The tasks below are unchanged except in scale.

**Goal:** run N independent workers, each flying episodes, for hours, unattended,
with failures handled rather than avoided — and know the real aggregate
throughput number that M10's evaluation sweep depends on.

**Why it matters:** this is where every bug this project has produced actually
lives. Building it now, against the M3 mission that already works, means each
failure has exactly one possible cause.

**Depends on:** M1b, M2, M3. **Blocks:** M6 (dataset generation), M10 (sweep).

**Risk: High.** Budget a week and expect to spend most of it on failure handling
rather than on the happy path.

### Tasks

1. ✅ **`EpisodeRunner` — one class, one episode, one worker.**
   Takes an `InstanceSpec`, a mission config, a fault config (null for now, wired
   in M6) and a seed. Returns a validated episode record. It owns: reset →
   arm → fly → terminate → log. It does **not** own process lifecycle.
   Everything above M4 — dataset generation, evaluation, the Gym env — is a
   caller of this class. There must be exactly one.
   Absorbed M3's `run_episodes.py`-internal `_Worker` class; `run_episodes.py`
   now calls `EpisodeRunner` instead of duplicating its logic. Fault config
   wiring is still a null-op pending M6, as scoped.
2. ✅ **`WorkerSupervisor` — owns one worker's processes.**
   Start, health-check, stop, restart. Health checks come from
   `docs/parallelism.md` §8: PIDs alive, telemetry not stalled, DDS link up.
   Exposes `ensure_healthy()` which restarts and returns whether a restart
   happened, so the caller can invalidate the in-flight episode.
   Telemetry-staleness and DDS-link checks are done via a heartbeat file the
   worker's child process writes every control tick, since the health check
   itself runs in the parent (no rclpy there, per CLAUDE.md §3.3) — see
   `experiments/worker_supervisor.py`'s module docstring.
3. ✅ **`SimFarm` — owns N supervisors.**
   Deterministic worker→instance assignment. Starts all, waits for all ready
   (with a per-worker timeout, and a hard failure if any never comes up), hands
   out work, restarts failures, aggregates restart counts into the run manifest.
   Context manager: leaving the block stops everything, including on exception.
   Restart counts are tracked (`SimFarm.restart_counts`) but not yet written
   into a run manifest -- that file itself is task 5.
4. ✅ **Structured failure handling.**
   1. ✅ `termination_reason` enum extended with the failure modes from
      `docs/parallelism.md` §8. Schema is v2; `worker_restarted` and
      `offboard_lost` were added in tasks 1-3, and `sim_fault` (the catalogue's
      last row — non-finite position/velocity) was added here: a new
      `SimFault` exception in `mission_executor.py`'s `record_step()`, mapped
      through `_REASON_FOR_ERROR` like every other typed flight-sequence
      error, not left as an unused enum value.
   2. ✅ Episodes ended by worker failure are written with `valid=false` and a
      reason, never silently discarded. **Closes the gap that was called out
      explicitly in `experiments/sim_farm.py`'s module docstring**:
      `SimFarm._synthesize_lost_episode_record()` now writes a placeholder
      record (`termination_reason=worker_restarted`, `valid=false`) for the
      exact episode a dead worker was mid-flight on, via a standalone
      `EpisodeLogger` (needs no rclpy, so it's safe to call from the parent
      process), and advances `_completed_per_worker` so the respawned
      worker's next episode id doesn't collide with the placeholder's
      already-written file. `sim_fault` episodes are also now `valid=false`
      (`experiments/episode_runner.py`); every other outcome remains a real,
      trustworthy measurement.
   3. ✅ Restart budget per worker; exceeding it fails the run loudly — the
      per-worker ceiling (`WorkerSupervisor.restart_budget` /
      `RestartBudgetExhausted`) was already built in tasks 1-3. The run-level
      view across workers this sub-task was waiting on is `SimFarm`'s
      `restart_counts` dict, now aggregated by 4.4's threshold check below.
   4. ✅ A run-level abort if the global restart rate exceeds a configured
      threshold. `SimFarm.restart_rate_abort_threshold` (default 0.5, in
      `configs/env/farm.yaml`) compares total restarts across every worker
      against total planned episodes in `_on_restart()`, before a
      replacement worker is spawned, and raises `SimFarmError` if exceeded.
5. ✅ **Run manifest.** `results/<run_id>/manifest.json`: run id, git SHA of this
   repo, PX4 SHA (inside `env_versions`), config digests, seeds, worker→instance
   map, versions from `env_report.sh`, start/end time, episode counts by
   outcome, restart counts. Written incrementally (`RunManifest.write()`,
   called after every episode/restart) so a killed run still leaves a readable
   manifest. `SimFarm.run()` also gained `on_result`/`on_restart` progress
   callbacks at the same time (used by tasks 6/7 below for live terminal
   progress, not just by the manifest).
6. ✅ **Throughput measurement — the number M9 is budgeted from.**
   `experiments/benchmark_throughput.py` swept worker count ∈ {1,2,3,4} ×
   speed factor ∈ {1,2,4,8} for real. **Chosen operating point: 2 workers,
   4x speed — 4.01 sim-s/wall-s, zero restarts, zero failures**, the best
   *and* the only perfectly clean result in the table; worker_count ≥ 3
   never beats it and gets measurably less reliable. Full table, method, and
   the reliability findings below: `docs/throughput.md`. M9-budget sanity
   check satisfied by construction (D12 already moved M9's training off
   this stack onto Isaac Lab).

   **Two real bugs found and fixed live during the sweep** (both in
   `experiments/worker_supervisor.py`, plus one in `aero_bridge/reset.py`):
   `is_healthy()` was racing a worker's own in-flight `hard_reset()` (which
   legitimately has no instance file for ~20s) and piling a redundant
   restart on top of it — fixed by trusting a live child's heartbeat over
   raw PID presence. `ensure_healthy()`'s restart path could leave one of a
   worker's three OS processes alive and untracked (observed: a `gz sim` +
   `MicroXRCEAgent` pair), permanently blocking every future start attempt
   for that instance — fixed with a verify-and-force-kill fallback. A third
   bug, unrelated to restarts: `aero_bridge/reset.py`'s `REPO_DIR` was
   computed as a fixed count of parent directories from `__file__`, which
   silently broke depending on which of three physically different
   locations the file was imported from (this project's install tree
   duplicates it) — fixed by reusing `simulation/worker_process.py`'s
   already-correct `REPO_DIR` instead of re-deriving a second one. A fourth
   issue, a queue race that could double-record one episode, was also found
   and fixed (`SimFarm._record_result`'s dedup). A fifth issue -- a reset
   writes no heartbeat while it runs, so a slow-under-load reset can still
   look stale to the health check -- was found here but fixed under task 7,
   once it started actually threatening a run (see task 7).

   **worker_count=4 at 4x speed failed on all three attempts**, each with a
   different proximate symptom, even after the process-leak bugs above were
   fixed — accepted as a genuine reliability finding for this hardware
   (CPU never saturates, so it isn't a scheduling problem) rather than kept
   open as an unfixed bug. Full detail in `docs/throughput.md`.
7. ✅ **Soak test — PASSED for real, 2026-09-22.** `tests/slow/test_soak.py`:
   `AERO_SOAK_WORKER_COUNT` / `AERO_SOAK_EPISODES_PER_WORKER` /
   `AERO_SOAK_SPEED_FACTOR` env vars, default **2 workers × 200 episodes at
   1x speed** (400 total, defaults chosen from real measurement, not the
   milestone's literal "4 workers × 100 episodes" — see below).
   `@pytest.mark.timeout(14400)` outer deadline (CLAUDE.md §5). **Final
   passing run: 400/400 episodes, zero restarts, zero orphan processes,
   memory flat (peak 4625.2MB), ~3 hours unattended.**

   **Getting there took five real, sequential findings** — the soak test
   earned its place as this milestone's hardest task:
   1. First attempt at the milestone's literal 4 workers × 100 episodes ×
      1x speed **genuinely failed**: every worker converged on its 5-restart
      budget within ~35 of 400 episodes, from real background DDS/rclpy
      instability (`ExternalShutdownException`/`RCLError`, already
      documented in `docs/parallelism.md` §2.6), not a new bug. Rescoped to
      **2 workers** (task 6's own actual recommended count) × 200 episodes
      — same total workload, the configuration that actually matters for
      M6/M10.
   2. A real bug found and fixed within the first ~15 episodes at the new
      scope: a worker's own `hard_reset()` writes no heartbeat while it
      runs, so under load a slow reset could still look stale to the health
      check and get a redundant restart piled on top — fine at sweep scale,
      but enough to threaten a 200-episode run. Fixed in
      `EpisodeRunner.run_episode()`: a fresh heartbeat is now written right
      before a reset starts.
   3. Following task 6's "2 workers, 4x speed" recommendation, a run at that
      exact configuration revealed the sweep's blind spot: at 200 episodes
      (vs. the sweep's 3), **64% of episodes hit `episode_timeout` instead
      of completing**, and the worker-restart pattern recurred too. The
      sweep's `aggregate sim-s/wall-s`/`failure rate` columns only count
      restarts and `valid=false` records, so a configuration can look
      perfectly clean while mostly not finishing its missions. Speed
      default reverted to **1x** — the only speed actually proven reliable
      at soak scale. `docs/throughput.md`'s operating-point section and
      `test_soak.py`'s own docstring both carry the full correction.
   4. Even at 2 workers × 1x, two further runs still hit
      `RestartBudgetExhausted` (once per run) purely from the same
      background DDS crash rate accumulating over 200 episodes/worker —
      not a new bug, just the existing per-worker budget of 5 proving too
      tight for a run this long.
   5. `restart_budget_per_worker` raised from 5 to **20**, in
      `configs/env/farm.yaml` and both Python defaults
      (`WorkerSupervisor`/`SimFarm`), from real measurement: the worst-hit
      worker across the failed attempts needed 5 restarts within its first
      ~70 of 200 episodes (~1 per 14) — 20 gives real headroom above that
      measured rate. `restart_rate_abort_threshold` (the run-level circuit
      breaker) is unaffected. Re-run with this in place: **passed clean,
      zero restarts needed at all.**
   **Actual pass/fail result pending** — see the progress log for the final
   outcome once available.
8. ✅ **CPU affinity — not needed.** Task 6's sweep never saturates CPU (peaks
   at 90.9% even at the heaviest configuration tested); the instability seen
   at worker_count ≥ 3 is a DDS/process-startup reliability problem, not a
   scheduling one, so `taskset` core-pinning would not address it. Decided
   from real numbers, per the milestone's own "only if task 6 shows
   contention" condition — see `docs/throughput.md`.

### Files created

```
experiments/episode_runner.py         ✅
experiments/worker_supervisor.py      ✅
experiments/sim_farm.py               ✅
simulation/worker_process.py          ✅ (not originally listed -- extracted
                                          from aero_bridge/reset.py's hard_reset()
                                          to avoid a second start/stop implementation;
                                          see the progress log)
experiments/run_manifest.py           ✅ task 5
experiments/resource_sampler.py       ✅ (not originally listed -- whole-machine
                                          RSS/CPU sampling shared by tasks 6 and 7,
                                          factored out rather than written twice)
experiments/benchmark_throughput.py   ✅ task 6
configs/env/farm.yaml                 ✅ (built ahead of task 6, since tasks
                                          1-3 already needed worker-count/
                                          restart-budget/heartbeat-timeout
                                          config-driven per CLAUDE.md principle 4)
docs/throughput.md                    ✅ task 6
```

### Tests (required)

```
tests/test_sim_farm_assignment.py       ✅
tests/test_supervisor_state_machine.py  ✅
tests/test_episode_runner.py            ✅ (not originally listed -- covers
                                            next_reset_tier(), the one pure-
                                            function piece of EpisodeRunner's
                                            own logic; "unit-test-each-milestone")
tests/test_mission_executor.py          ✅ (not originally listed -- covers
                                            sim_fault's non-finite-state check,
                                            a pure function; "unit-test-each-milestone")
tests/test_run_manifest.py              ✅ task 5
tests/sim/test_two_workers.py       ✅ (@pytest.mark.sim)
tests/sim/test_worker_restart.py    ⬜ task 4, no-sim coverage done instead this
                                        session (see below) -- the sim-marked
                                        version (a real `kill -9` against a live
                                        worker) is still not written; deferred to
                                        avoid a long sim-requiring session
tests/slow/test_soak.py             ✅ task 7 (@pytest.mark.slow) -- passed for
                                        real, 2026-09-22 (400/400, 0 restarts)
```

- ✅ `test_worker_to_instance_is_deterministic` — worker k always gets instance
  k+base, across processes and runs.
- ✅ `test_no_resource_collision_for_n_workers` — for N up to 8, no two workers
  share a port, domain, partition, namespace or model name.
- ✅ `test_farm_stops_all_on_exception` — an exception inside the context manager
  still stops every worker (use fake supervisors; no simulator).
- ✅ `test_supervisor_restart_marks_episode_invalid` — a simulated process death
  produces `valid=false` with the right `termination_reason`. *(Written against
  `WorkerSupervisor.ensure_healthy()`'s own contract -- the return-value signal
  a caller uses to mark an episode invalid.)*
- ✅ `test_restart_budget_exhausted_raises` — exceeding the budget fails loudly.
- ✅ `test_is_finite_state_false_for_nan_in_any_position` /
  `..._for_inf_in_any_position` / `test_sim_fault_maps_to_sim_fault_termination_reason`
  (`tests/test_mission_executor.py`) — sim_fault's detection and mapping, task 4.1.
- ✅ `test_synthesize_lost_episode_record_writes_invalid_record` /
  `test_synthesize_skipped_when_worker_already_finished_quota` /
  `test_on_restart_records_lost_episode_before_respawning`
  (`tests/test_sim_farm_assignment.py`) — task 4.2: a fake dead worker produces
  a `valid=false, termination_reason=worker_restarted` record that validates
  against the schema, is skipped when nothing was actually in flight, and is
  written before the replacement worker is spawned, at the right episode index.
- ✅ `test_restart_rate_abort_not_triggered_below_threshold` /
  `test_restart_rate_abort_raises_past_threshold`
  (`tests/test_sim_farm_assignment.py`) — task 4.3/4.4: the run-level restart
  rate is tracked correctly and `SimFarmError` fires exactly past the
  configured threshold, not before.
- ✅ `test_manifest_readable_after_kill` / `test_manifest_captures_static_fields_
  at_construction` / `test_record_episode_updates_outcome_counts` /
  `test_record_restart_updates_restart_counts` / `test_write_produces_valid_
  json_at_the_given_path` / `test_finalize_sets_end_time_and_writes`
  (`tests/test_run_manifest.py`) — task 5: a partially written manifest still
  parses, and every field/count round-trips correctly.
- ✅ `test_is_healthy_true_when_child_alive_even_if_instance_file_missing` /
  `test_is_healthy_false_when_child_alive_but_heartbeat_stale_despite_missing_pids` /
  `test_ensure_healthy_survives_a_failed_restart_attempt` /
  `test_ensure_healthy_force_kills_a_pid_left_alive_after_stop` /
  `test_stop_is_a_noop_when_no_instance_file_exists`
  (`tests/test_supervisor_state_machine.py`) — regression tests for the four
  real bugs found live during task 6's throughput sweep (see task 6's own
  writeup above and `docs/throughput.md`).
- ✅ `test_duplicate_episode_record_is_dropped_not_double_recorded` /
  `test_drain_queue_does_not_double_count_a_duplicate_result`
  (`tests/test_sim_farm_assignment.py`) — the queue-race dedup fix found
  live during the same sweep.
- ✅ `test_run_episode_writes_a_heartbeat_before_starting_a_reset`
  (`tests/test_episode_runner.py`) — the heartbeat-before-reset fix found
  live during task 7's soak test.
- ✅ `test_two_workers_fly_concurrently` (sim) — two workers each complete a full
  M3 mission at the same time; both records validate; the two vehicles' logs are
  distinguishable and neither contains the other's data. **This is the test that
  catches cross-wiring.** *(Passed on the real simulator, twice concurrently,
  with `pgrep -cf "^gz sim "` confirmed reading 2 live during the run. Both
  episodes ended `episode_timeout` rather than `completed` in the verification
  run — consistent with the known, pre-existing M2 concurrent-worker
  `offboard_control_signal_lost` gap, `docs/parallelism.md` §2.6, not a new
  bug; the test deliberately does not require `completed` for exactly this
  reason.)*
- ⬜ `test_worker_restart_recovers` (sim) — `kill -9` a worker's px4 mid-episode;
  the farm restarts it and the next episode succeeds. Still open -- this
  session covered the restart-handling *logic* with fakes (above) but did not
  run anything against a live simulator. (task 4)
- ✅ `test_soak_sustained_run` (slow) — 400 episodes, zero orphans, flat memory.
  Passed for real 2026-09-22 at 2 workers × 200 episodes × 1x speed, after
  the 5-step investigation above. (task 7)

### Done when

- [x] N workers run concurrently, fully isolated
      (`pgrep -cf "^gz sim "` == N) — confirmed live with N=2
- [ ] Killing one worker's PX4 mid-episode restarts only that worker; the others
      keep flying and their episodes remain valid — `ensure_healthy()`/restart
      exists and is unit-tested, but not yet proven against a real `kill -9`
      (task 4's `test_worker_restart_recovers`, sim-marked)
- [x] 400 episodes (2 workers × 200, the actually-recommended operating
      point — not the milestone's literal 4×100, see task 7's writeup)
      complete unattended with zero orphan processes — confirmed live,
      2026-09-22 (task 7)
- [x] Peak RSS recorded and within budget; no growth across the soak —
      `ResourceSampler.grew_meaningfully()` was False; peak 4625.2MB over
      the full ~3h run (task 7)
- [x] `docs/throughput.md` states the chosen (worker count, speed factor)
      operating point and the aggregate throughput it delivers — **2
      workers, 4x speed, 4.01 sim-s/wall-s**, the full 16-configuration
      sweep, and the reliability findings behind that choice (task 6)
- [x] Every episode record carries a valid `termination_reason`; invalid
      episodes are recorded, not dropped — `sim_fault` closes the enum (task
      4.1) and `SimFarm._synthesize_lost_episode_record()` now writes the
      placeholder record for a worker-restart-lost episode (task 4.2),
      unit-tested with fakes; not yet exercised against a real `kill -9`
      (that's `test_worker_restart_recovers`, still open above)
- [x] The run manifest reproduces the run's configuration completely (task 5)
      — `RunManifest`, written incrementally to `results/<run_id>/manifest.json`
- [x] Interrupting the farm with Ctrl-C leaves nothing running —
      `SimFarm.__exit__` stops every worker unconditionally, including on
      exception, verified by `test_farm_stops_all_on_exception` and
      `test_exit_attempts_every_stop_even_if_one_fails`

### Verify with

```bash
pytest tests/ -m "not sim and not slow" -q
pytest tests/sim/ -q
python experiments/benchmark_throughput.py --workers 1,2,3,4 --speeds 1,2,4,8
pytest -s tests/slow/test_soak.py          # ~3 h at the default 2x200 episodes, run it overnight
pgrep -cf "^gz sim |px4_sitl_default/bin/px4|MicroXRCEAgent"   # expect 0 after
```

### Watch out for

- **Verify with two workers, always.** One worker proves nothing here; PX4's
  instance-0 special cases (`docs/parallelism.md` §2.2) are designed to fool you.
- **Never call `rclpy.init()` in the parent process.** DDS participants own
  threads and sockets that do not survive process creation. Every ROS object is
  constructed inside the worker process. Set the multiprocessing start method to
  `spawn` explicitly rather than relying on a default.
- **Startup is the most fragile moment.** Starting four workers simultaneously
  contends for CPU during EKF convergence and can time out readiness checks that
  pass fine one at a time. Stagger worker starts by a few seconds and make the
  readiness timeout generous.
- **Do not search for free ports or domains.** Two workers starting at once will
  both find the same free port. Assignment is a pure function of worker index.
- **The broad `pkill` sweep is forbidden while workers are alive.** Per-worker
  stop uses recorded PIDs only.
- **Restarts are not uniformly distributed.** High-severity faults crash more,
  so silently dropping failed episodes biases exactly the comparison the paper
  rests on. Record them.
- **Watch RAM.** Four workers plus a learner on 16 GB is tight. Measure peak RSS
  during the soak, not after.
- Episodes must have both a sim-time limit and a wall-clock watchdog. Sim-time
  alone does not catch a frozen simulator, because sim time stops advancing.

### Claude Code prompt (use plan mode)

```
Read CLAUDE.md, docs/parallelism.md (all), and milestones.md M4.

Plan, then implement, M4 tasks 1-3: EpisodeRunner, WorkerSupervisor and SimFarm.

Architecture constraints:
  - One drone per world, always (D7 — settled, no topology system to build).
  - EpisodeRunner runs exactly one episode and owns no process lifecycle.
    Everything later (dataset generation, evaluation, the Gym env) calls it.
    There must be exactly one implementation of "fly one episode" in this repo.
  - WorkerSupervisor owns one worker's three processes and its health checks
    (docs/parallelism.md §8). ensure_healthy() restarts and reports whether it
    did, so the caller can invalidate the in-flight episode.
  - SimFarm is a context manager that stops every worker on exit, including on
    exception. Worker->instance assignment is a pure function; no port search.
  - rclpy.init() is called only inside worker processes. Multiprocessing start
    method is set to "spawn" explicitly.
  - Every wait has a deadline. No unbounded loops.

Deliverables:
  - experiments/episode_runner.py, worker_supervisor.py, sim_farm.py
  - tests/test_sim_farm_assignment.py, tests/test_supervisor_state_machine.py
    (both no-sim, using fake supervisors/processes)
  - tests/sim/test_two_workers.py

Verify: pytest tests/ -m "not sim and not slow" -q, then
pytest tests/sim/test_two_workers.py -q with two workers actually running.
Report the output of `pgrep -cf "^gz sim "` during the test — it must be 2.

Do not: call rclpy.init() in the parent; do not search for free ports; do not
use pkill; do not drop failed episodes silently.
```

---
# M5 — Telemetry feature pipeline ✅

**M5 complete (2026-09-22).** All 6 tasks done and verified, including
against a real live flight, not only fixtures. `configs/features.yaml` (13
shared + 6 px4_only features, `feature_version: "1"`), `configs/rl/
observation_v1.yaml` (obs_version "1"), episode schema v3 (raw attitude/
rate/acceleration/motor-output fields), `ai/features/feature_extractor.py`
(`TelemetryWindow` + `FeatureExtractor`, no ROS import), and `configs/rl/
normalization_v1.yaml` all exist and are cross-consistency-tested. One
naming correction made while implementing task 1 against the real, already-
existing schema: the plan below originally spelled the velocity/position-
error features `vel_x_m_s`/`vel_y_m_s`/`vel_z_m_s`/`pos_error_m`, but schema
v2's step_fields already used `vel_x`/`vel_y`/`vel_z`/`position_error_m` --
caught immediately by the live sim test (task 6) failing with a
`KeyError`/missing-field error the moment real parquet columns were fed
through the extractor. Renamed the feature names to match the existing
schema rather than inventing a second name for the same quantity
(CLAUDE.md §1.4) -- `configs/features.yaml`, `observation_v1.yaml` and
`feature_extractor.py` all reflect the corrected names; the task
description below is left showing the original (wrong) names with this
note, rather than silently edited, so the mismatch and its catch are on the
record.

**Goal:** turn raw telemetry into one fixed-size vector, produced 10 times per
second, used identically by the detector and the RL policy. One implementation,
imported everywhere — if the detector and the policy compute features
differently, they drift apart and M10's comparison becomes invalid.

**Depends on:** M3. **Blocks:** M6, M7, M9.

**Key deliverables:**
- `FeatureExtractor` — a pure function of a telemetry window (no ROS, no I/O,
  no global state), so it's testable without a simulator and M7 can iterate on
  it offline in seconds.
- The feature set must include the **thrust-vs-achieved-acceleration
  residual** — a weakening motor forces PX4 to command more thrust while the
  aircraft accelerates less, likely the single most informative signal here.
- Normalisation statistics computed once from healthy flights and **frozen to
  disk** — never recomputed once training starts, or every trained model
  downstream is silently invalidated.
- `feature_version` in `configs/features.yaml`, bumped whenever the feature
  set changes.

**Non-negotiable:** windows are strictly causal — nothing in the window may use
a value from after the window's end. This bug (lookahead leakage) is invisible
and inflates every later result if it slips in.

**Second non-negotiable, added 2026-09-21 (D12):** the feature set splits in
two, and the split must be decided *here*, not at M9.

| | Computable in | May feed |
|---|---|---|
| **Shared features** | both PX4 and Isaac | the detector **and** the policy observation |
| **PX4-only features** | PX4 telemetry only — control-allocation residual, EKF innovations, per-motor outputs | the **detector only** |

The policy observation may contain only shared features, because a policy
trained in Isaac on an input Isaac cannot produce is a policy that cannot
transfer. `configs/features.yaml` marks each feature with which side can
compute it, and a unit test asserts that everything referenced by
`configs/rl/observation_v1.yaml` is marked shared. Note this costs the policy
nothing it actually needs: the PX4-only signals are exactly the ones the
*detector* consumes, and the policy sees the detector's output, not its input.

**Scope decided when this milestone started (2026-09-22).** `EKF innovations`
and `control-allocation residual` are real PX4-side signals, but computing
either correctly needs telemetry this project does not currently subscribe to
(an `EstimatorInnovations`-family topic; PX4's actual allocation matrix). Both
are deferred past v1 rather than approximated badly — `configs/features.yaml`
records them as `status: deferred` with the reason, so this is a documented
scope cut, not a silent gap. v1's PX4-only feature set is instead the signals
already reachable from the 9 telemetry topics M2 confirmed: per-motor
normalised outputs (`actuator_motors`), battery, and a first-cut
thrust-vs-achieved-acceleration residual (below).

**What M5 does *not* do:** compute features live during a flight or write
them into episode records. `EpisodeRunner`/`mission_executor.py` keep
writing raw telemetry only; `feature_version` on every episode record stays
`FEATURE_VERSION_UNSET` until a real consumer (M6's dataset builder, or M7)
actually calls `FeatureExtractor` and tags its output. `FeatureExtractor` is
built and proven correct here, against recorded/fixture telemetry, precisely
so those later milestones have a single, already-tested implementation to
import rather than building their own.

### Tasks

1. ✅ **`configs/features.yaml` — the feature contract.** `feature_version:
   "1"`. One entry per feature: `name`, `side` (`shared` | `px4_only`),
   `source` (which raw telemetry field(s) it comes from), one-line
   `description`. Plus a `window` block (`length_s: 1.5`, `rate_hz: 10.0`,
   matching `mission_executor.py`'s `CONTROL_RATE_HZ`). v1's feature list:
   - *shared* (13): `roll_rad`, `pitch_rad`, `yaw_rad` (attitude);
     `rate_p_rad_s`, `rate_q_rad_s`, `rate_r_rad_s` (body rates);
     `accel_x_m_s2`, `accel_y_m_s2`, `accel_z_m_s2` (linear acceleration);
     `vel_x_m_s`, `vel_y_m_s`, `vel_z_m_s`; `pos_error_m`.
   - *px4_only* (6): `battery_remaining`; `motor_0_output` .. `motor_3_output`
     (normalised, from `actuator_motors.control[0:4]`); `thrust_accel_residual`
     (derived — see task 3).
   Battery is `px4_only`, not shared: Isaac Lab's stock quadrotor task has no
   battery model, so a policy trained on it cannot use battery state.
2. ✅ **Extend the episode record schema to v3** — the raw ingredients
   `FeatureExtractor` needs are not currently logged.
   `configs/schema/episode_record.yaml`: `schema_version: "3"`, `step_fields`
   gains `roll_rad`, `pitch_rad`, `yaw_rad`, `rate_p_rad_s`, `rate_q_rad_s`,
   `rate_r_rad_s`, `accel_x_m_s2`, `accel_y_m_s2`, `accel_z_m_s2`,
   `motor_0_output` .. `motor_3_output`. `experiments/episode_schema.py`'s
   `SCHEMA_VERSION` bumps to match. `mission_executor.py`'s `record_step()`
   populates the new fields from `px4.latest['vehicle_attitude']` (quaternion
   → Euler, Hamiltonian `q=[w,x,y,z]`, FRD body → NED earth — confirmed from
   `VehicleAttitude.msg`, not guessed, per M0's rule) and
   `px4.latest['sensor_combined']` (`gyro_rad`, `accelerometer_m_s2`) and
   `px4.latest['actuator_motors']` (`control[0:4]`). Quaternion→Euler is a
   pure function (`_quaternion_to_euler`), unit tested directly — this is
   exactly the kind of conversion M0 warns about getting silently backwards.
   Raw telemetry only — no derived features are written to the schema.
3. ✅ **`ai/features/feature_extractor.py` — the one implementation.** Pure
   Python + NumPy, **no ROS import**, so the `isaacsim` env can import the
   shared subset without ever touching `rclpy` (CLAUDE.md §0.1). Two pieces:
   - `TelemetryWindow` — an append-only, fixed-capacity buffer
     (`capacity = round(length_s * rate_hz)` = 15 frames). `append()` rejects
     an out-of-order (non-monotonic `t_sim_s`) frame. This is what makes
     causality structural rather than a discipline everyone has to remember.
   - `FeatureExtractor.extract(window) -> dict[str, float]` — 13 shared
     features pass through from the window's latest frame; `battery_remaining`
     and the 4 motor outputs likewise; `thrust_accel_residual` is the one
     genuinely derived feature: `(commanded_thrust_frac_t -
     window_mean(commanded_thrust_frac)) - (accel_magnitude_t -
     window_mean(accel_magnitude)) / g`, i.e. how far *this instant's*
     commanded thrust deviates from its own recent baseline, compared to how
     far the achieved acceleration magnitude deviates from *its* baseline.
     Deliberately self-relative (baseline-vs-baseline) rather than an
     absolute hover-thrust physics constant, because the sign convention of
     `sensor_combined.accelerometer_m_s2` was not independently re-verified
     here — a magnitude-based, self-relative formula is correct regardless
     of that sign. **Documented as a first-cut heuristic**: its actual
     discriminative power against a real injected fault is an M6/M7
     question, not asserted here. *(Answered in M7: on its own it does not
     discriminate, AUROC 0.40 — `docs/detector_results.md`.)*
   Also: `shared_feature_names()`, `px4_only_feature_names()`,
   `all_feature_names()`, reading `configs/features.yaml` once and caching it
   (same pattern as `episode_schema.load_schema()`).
4. ✅ **`configs/rl/observation_v1.yaml`.** `obs_version: "1"`, `feature_version:
   "1"`, and the list of the 13 shared feature names — nothing else. A
   comment records that the detector's *output* (not raw features) is how
   PX4-only information reaches the policy, decided by M9's env wrapper, not
   here.
5. ✅ **Normalisation statistics — computed once, frozen.** A script
   (`experiments/analysis/compute_normalization_stats.py`) that reads a
   healthy-flight run's raw step telemetry (schema v3), runs
   `FeatureExtractor` causally over every episode, and writes per-feature
   mean/std to `configs/rl/normalization_v1.yaml` (`norm_version: "1"`,
   `feature_version: "1"`). Run for real against a fresh 10-episode
   `square_circuit` run flown live under the new schema
   (`results/m5_norm_stats_healthy_10/`, 11 valid episodes — 10 completed
   plus one hard-reset retry after an `episode_timeout`, contributing 3,740
   steps total; one `preflight_failed` attempt with zero steps recorded was
   correctly skipped rather than crashing the script on an empty frame,
   a real bug found and fixed live during this task). M3's existing 20-run
   fixtures could not be reused: they predate schema v3 and lack the new raw
   fields entirely. The resulting statistics are physically sane on
   inspection — e.g. `accel_z_m_s2` mean ≈ -10.44 m/s² (close to -g plus
   flight dynamics) and motor outputs cluster around ≈0.70 ± 0.19, a
   plausible hover throttle fraction. **Never recomputed after training
   starts** (CLAUDE.md anti-pattern 11) — this file, once written, is
   read-only input to everything downstream.
6. ✅ **Validation.** `tests/sim/test_feature_pipeline.py` (`@pytest.mark.sim`):
   flies one real healthy mission, runs `FeatureExtractor` causally over its
   logged steps, and asserts every shared feature is finite (no NaN/Inf) and
   `t_sim_s` never goes backwards with no gap large enough to mean a dropped
   step — this is the "no NaNs/gaps over a full healthy mission" check from
   `planning.md` Phase 5, and it is also what actually confirmed the
   quaternion/gyro/accel field names, the Euler conversion, and (see the
   note above this task list) the raw feature *names* were right, rather
   than merely plausible: the first live run caught the `vel_x_m_s`/
   `pos_error_m` naming mismatch, and a second live-data finding refined
   the gap check itself -- at `speed_factor=4x`, `mission_executor.py`'s
   control loop legitimately records steps ~0.4s apart in sim time (not
   ~0.1s -- the poll cadence is wall-clock-paced, so sim time between
   recorded ticks scales with the speed factor), and one single ~2s outlier
   tick occurred live from ordinary CPU/DDS scheduling jitter right after
   arm+offboard engage (the same "startup is the most fragile moment"
   effect M4 already documents). The test now checks the *median* step
   spacing plus a generous absolute outlier bound, the same choice M2's
   telemetry sanity test made for the same reason, rather than a strict
   per-step maximum.

### Files created

```
configs/features.yaml
configs/rl/observation_v1.yaml
configs/rl/normalization_v1.yaml              (written by task 5's script, not hand-authored)
ai/features/__init__.py
ai/features/feature_extractor.py
experiments/analysis/compute_normalization_stats.py
configs/schema/episode_record.yaml             (v2 -> v3)
experiments/episode_schema.py                  (SCHEMA_VERSION bump)
ros2_ws/src/aero_bridge/aero_bridge/mission_executor.py   (record_step + _quaternion_to_euler)
```

### Tests (required)

```
tests/test_feature_extractor.py
tests/test_mission_executor.py           (extended — _quaternion_to_euler)
tests/test_episode_schema.py             (extended — v3 fields, version bump)
tests/sim/test_feature_pipeline.py       (@pytest.mark.sim)
```

- ✅ `test_window_rejects_out_of_order_append` / `test_window_respects_capacity` /
  `test_window_accepts_equal_timestamps` / `test_window_append_rejects_incomplete_frame`
- ✅ `test_extract_output_matches_feature_order` — keys equal
  `all_feature_names()`, in the same order every time.
- ✅ `test_extractor_is_causal` — a feature vector computed from a window
  truncated at frame `k` is bit-identical whether or not later frames ever
  get appended; **the regression test for lookahead leakage.**
- ✅ `test_extract_is_deterministic` / `test_extract_series_matches_incremental_online_extraction` /
  `test_extract_series_output_for_a_frame_is_unaffected_by_later_frames` —
  calling `extract()` twice on the same window gives a bit-identical result
  (`planning.md` Phase 5's replay determinism check, at the unit level), and
  the offline batch helper (`extract_series`, M6's future dataset builder)
  is proven equal to online incremental extraction.
- ✅ `test_thrust_accel_residual_is_zero_on_a_single_frame_window` /
  `test_thrust_accel_residual_is_finite_and_reacts_to_a_thrust_spike` — no
  baseline yet, so the derived feature must not divide by zero or crash, and
  it does move in the expected direction for a synthetic thrust spike.
- ✅ `test_observation_v1_features_are_all_marked_shared` — **the test the
  D12 constraint exists for**: every name in `configs/rl/observation_v1.yaml`
  has `side: shared` in `configs/features.yaml`.
- ✅ `test_observation_v1_feature_version_matches_features_yaml` /
  `test_normalization_v1_feature_version_matches_features_yaml` (the latter
  skips until `normalization_v1.yaml` exists, then runs for real) —
  `features.yaml`, `observation_v1.yaml` and `normalization_v1.yaml` all
  carry the same `feature_version`.
- ✅ `test_quaternion_to_euler_identity_is_zero` / `..._90_degree_roll` /
  `..._90_degree_yaw` / `..._handles_gimbal_lock_pitch_without_crashing` —
  identity quaternion gives `(0,0,0)`; hand-computed 90°-rotation cases
  match; the pitch-singularity clip doesn't crash or go non-finite.
- ✅ `test_healthy_mission_features_have_no_nans_or_gaps` (sim) — real flight,
  real telemetry, the actual validation `planning.md` Phase 5 asks for.
  Passed live, 2026-09-22, against a real `square_circuit` flight at 4x
  speed (289 steps).

### Done when

- [x] `configs/features.yaml` and `configs/rl/observation_v1.yaml` exist and
      validate; every shared feature the observation spec references is
      marked `shared` — `test_observation_v1_features_are_all_marked_shared`.
- [x] Episode schema v3 round-trips: a step record with the new raw fields
      validates, and `test_termination_reason_enum_closed`-style drift tests
      still pass.
- [x] `FeatureExtractor` produces a deterministic, causal, fixed-order vector
      from a `TelemetryWindow`, proven by unit tests against fixtures — no
      simulator needed to iterate on it (the whole point, per `planning.md`
      Phase 5's rationale for M7 iterating offline).
- [x] A real healthy mission's logged telemetry, run through
      `FeatureExtractor`, has zero NaNs and no dropped-step-sized time gaps —
      confirmed live (task 6).
- [x] `configs/rl/normalization_v1.yaml` exists, was computed from an 11-
      episode healthy dataset flown live under schema v3
      (`results/m5_norm_stats_healthy_10/`), and is committed as frozen (not
      regenerated by any later milestone without a version bump).
- [x] `pytest tests/ -m "not sim and not slow"` passes — 190 passed, 0
      skipped (the `normalization_v1.yaml`-gated test runs for real once
      that file exists).

### Verify with

```bash
pytest tests/ -m "not sim and not slow" -q
pytest tests/sim/test_feature_pipeline.py -q         # starts/stops its own worker via SimFarm
scripts/sim_start.sh -i 0
python experiments/run_episodes.py --mission square_circuit --n 10 --instance 0 --run-id <run_id>
python experiments/analysis/compute_normalization_stats.py results/<run_id>/ --out configs/rl/normalization_v1.yaml
scripts/sim_stop.sh --all
```

Actually run 2026-09-22: all of the above passed live, including
`pgrep`-confirmed clean teardown after `scripts/sim_stop.sh --all`
(`results/m5_norm_stats_healthy_10/` is the real run `normalization_v1.yaml`
was computed from).

### Watch out for

- **Do not guess the quaternion order or field names.** `VehicleAttitude.msg`
  and `SensorCombined.msg` are the ground truth (already read for this
  milestone's plan); a wrong sign or wrong field name here corrupts every
  feature downstream of it silently, the same failure mode M0 already warned
  about for `px4_msgs` generally.
- **`configs/rl/normalization_v1.yaml` is a frozen artifact, not a config to
  hand-edit.** If the feature set changes, bump `feature_version`, regenerate
  it, and bump `norm_version` — never edit it in place (CLAUDE.md §7,
  anti-pattern 11).
- **Do not wire `FeatureExtractor` into `EpisodeRunner` yet.** That is a
  design decision for whichever of M6 (dataset builder) or M7 (detector)
  actually consumes it first — doing it here would be guessing at their
  interface before either exists.

### Claude Code prompt

```
Read CLAUDE.md, planning.md Phase 5 (and its D12 note), and milestones.md M5.

Implement M5 tasks 1-4: configs/features.yaml, the schema v3 raw-telemetry
fields (+ mission_executor.py wiring), ai/features/feature_extractor.py, and
configs/rl/observation_v1.yaml.

Constraints:
  - ai/features/feature_extractor.py imports no ROS/rclpy — it must be
    importable from the isaacsim conda env unmodified (CLAUDE.md §0.1).
  - Every feature in observation_v1.yaml must be side: shared in
    features.yaml; write the test that checks this before anything else.
  - TelemetryWindow.append() rejects out-of-order frames -- causality is
    structural, not a comment.
  - Quaternion order is Hamiltonian (w,x,y,z), FRD body -> NED earth, per
    VehicleAttitude.msg -- verify against the actual .msg file, don't guess.
  - No feature computation is wired into EpisodeRunner in this task.

Deliverables:
  - configs/features.yaml, configs/rl/observation_v1.yaml
  - configs/schema/episode_record.yaml (v3), episode_schema.py (version bump)
  - ai/features/feature_extractor.py, ai/features/__init__.py
  - mission_executor.py's record_step() populating the new raw fields
  - tests/test_feature_extractor.py, extensions to tests/test_mission_executor.py
    and tests/test_episode_schema.py, per M5's test list

Verify: pytest tests/ -m "not sim and not slow" -q

Do not: add EKF innovations or control-allocation residual (deferred,
undocumented telemetry gap); do not hand-edit a normalization file; do not
import rclpy anywhere under ai/.
```

---

# M6 — Fault injection and dataset ✅

**M6 complete (2026-09-23).** All 11 tasks done, verified live end to end:
the `RotorDegradationSystem` relay plugin, the `x500_aero` model, the Python
`RotorFaultController`, `EpisodeRunner`/`SimFarm` fault integration
(including surviving a hard reset), the cross-validation thrust fixture, and
a real, complete **750/750-episode labelled dataset**
(`run_id=m6_dataset_v1`, `docs/fault_dataset.md`). PX4 tree confirmed
untouched throughout.

Three real, load-bearing corrections found live, not anticipated by the
plan: (1) `px4-rc.gzsim` does not spawn via a generic `model://` resolution
as originally assumed — it hardcodes a path under PX4's own models
directory, requiring the launcher to spawn project-local models itself
(task 4); (2) the apt-installed `python3-gz-msgs10` Python bindings do not
expose `gz.msgs.Param`'s map field as a Python dict (task 3); (3) two
Python-side bugs (`run_episodes.py`/`SimFarm` not threading `--model`
through) silently broke soft reset for any non-default model. All three are
documented in place, at the task where they were found, rather than
silently fixed.

**A real, unplanned sub-feature was built mid-milestone**: resume support
(`SimFarm(resume=True)`, `generate_fault_dataset.py --resume`) — the actual
750-episode run was interrupted twice by genuine external events (a full
disk; a Claude Code session ending mid-run, which killed the untracked
top-level process even though its `setsid`-launched simulator children
survived) and resumed both times with zero episodes lost or re-flown. This
was not in the original task plan; it became necessary live and is now a
reusable capability for any future long dataset-generation run.

**The actual research finding this milestone exists to produce**: PX4's own
`FailureDetector` is not uniformly blind across the targeted severity range
— silent 100% of the time at `[0.2, 0.4)`, only 63.3% silent at `[0.8,
1.0]`, a clean monotonic trend. See `docs/fault_dataset.md` for the full
breakdown; this is real signal for how M7/M10 should frame the detection
problem, not noise to average away.

**Goal:** inject a rotor fault of chosen strength at a chosen moment, repeatably,
and produce the labelled dataset the detector trains on. The hardest engineering
milestone — budget accordingly.

**Depends on:** M4, M5. **Blocks:** M7.

**Key deliverables:**
- A Gazebo plugin (`RotorDegradationSystem`, our own repo, never touching the
  PX4 tree) that scales one rotor's thrust/torque by a live-settable efficiency
  factor. PX4's `main`-branch `MotorFailureSystem` is a useful structural
  reference but is binary-only and not in our pinned tag.
- Build order: binary on/off first (proves the plugin loads and affects
  flight), then graded severity/ramps/intermittent profiles, then run it under
  the M4 farm to generate 500-1000 labelled episodes.
- **Confirm, don't assume** — a gz-transport publish is fire-and-forget, so the
  plugin must echo the applied efficiency back on a status topic, and the
  dataset generator must check it. Otherwise it's possible to generate 500
  "40% fault" episodes in which no fault was ever actually applied.
- Assert per episode that **PX4's own `FailureDetector` stays silent** at the
  target severity — if PX4 notices, the fault is outside this project's
  research question.

**Non-negotiable:** `~/projects/PX4-Autopilot` must have zero uncommitted
modifications when this milestone is done.

**Second non-negotiable, added 2026-09-21 (D12): the fault model now exists
twice and must mean the same thing twice.** The Gazebo C++ plugin here, and an
Isaac-side Python rotor model in M8b. This is the one sanctioned exception to
"exactly one implementation" (`CLAUDE.md` §1.4), and the price of it is
`CLAUDE.md` §1.6: a cross-validation test asserting that the same commanded
severity `s` produces matching thrust reduction on both sides, against a
recorded fixture.

Build the Gazebo side first and record that fixture here, so M8b has something
to match rather than the two being written to agree with each other in the
abstract. If this test is not passing, the policy trains against one fault and
is evaluated against a different one, and **every RQ5 number is meaningless** —
in a way that looks exactly like an interesting transfer gap.

**Design, planned in plan mode 2026-09-22, before any code was written**
(CLAUDE.md's rule 5 for this milestone). Full rationale, source citations and
judgment calls are in the plan review; summarized here as the build record.

**Architecture: a relay, not a physics reimplementation.** Gazebo's stock
`MulticopterMotorModel` plugin (fixed `motorConstant` at model load) stays in
the new `x500_aero` model exactly as in stock `x500` — all real thrust
physics remains Gazebo-validated. A new `RotorDegradationSystem` C++ plugin
sits between PX4's bridge (which publishes commanded rotor velocities as
`gz::msgs::Actuators` on `/<model_name>/command/motor_speed` —
confirmed from `GZMixingInterfaceESC.cpp`, PX4 v1.17.0, unmodified) and the
stock motor plugins (repointed to a relayed topic): every rotor's velocity
passes through unchanged except the currently-faulted one, scaled by
`sqrt(1 - severity)` — since thrust ∝ velocity², this scales thrust (and,
via `momentConstant`, reaction torque) by exactly `efficiency = 1 -
severity`. Control/status channel: `gz.msgs.Param` (a generic variant map,
`{rotor_index, severity[, applied]}`), not a new custom `.proto` — its C++
headers ship with the already-transitively-required `libgz-msgs10-dev`, and
its Python bindings are already used twice in this repo
(`simulation/sim_clock.py`, `aero_bridge/reset.py`) for exactly this kind of
ad-hoc runtime channel, so `simulation/rotor_fault.py` (task 5) is a third
instance of an established pattern, not a new one.

**Zero PX4 tree changes needed — but not for the reason first assumed here.**
This section originally claimed `px4-rc.gzsim` spawns `model://$MODEL_NAME`
generically by stripping a `gz_` prefix off `PX4_SIM_MODEL`. **That is
wrong, caught live the first time `--model x500_aero` was actually tried**
(task 4): `px4-rc.gzsim` hardcodes
`"${PX4_GZ_MODELS}/${MODEL_NAME}/model.sdf"` — a path under PX4's *own*
models directory, never resolved via `GZ_SIM_RESOURCE_PATH` the way a
nested SDF `<include>` is — and `PX4_GZ_MODELS` can't be overridden either,
since PX4's own `gz_env.sh` unconditionally re-exports it every time
`px4-rc.gzsim` sources it. The actual fix (task 4): the launcher
(`scripts/sim_start.sh`) spawns a project-local model itself, by absolute
path, via a `gz.msgs.EntityFactory` request
(`InstanceSpec.gz_spawn_request()`), then sets `PX4_GZ_MODEL_NAME` so
`px4-rc.gzsim` attaches to the already-spawned model instead of trying to
spawn one itself. `4001_gz_x500`'s `PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500}`
default-only-assignment claim, and the "zero PX4 tree changes" conclusion
itself, both still hold — only the *mechanism* was wrong. Left here
uncorrected-in-place, with this note, rather than silently rewritten, so
the mistake and its catch are on the record the same way M5's naming
correction is. `scripts/sim_start.sh` **already** named this milestone in a
comment and already conditionally wired `simulation/models` onto
`GZ_SIM_RESOURCE_PATH` and `simulation/gz_plugins/build` onto
`GZ_SIM_SYSTEM_PLUGIN_PATH` (from M1b) — this milestone created those two
directories, nothing upstream of them.

**PX4 FailureDetector-silence check needs zero new subscriptions.**
`VehicleStatus.msg` (already subscribed via `px4_interface.py`, already
flowing into `mission_executor.record_step()`) already carries
`failure_detector_status` (`FAILURE_MOTOR = 128`, `FAILURE_IMBALANCED_PROP =
64`, etc.) — logging an already-flowing field, not a new topic.

**Scope note.** `planning.md` §6's fault-schema table (more detailed and
authoritative than this file's one-line mention of "intermittent") scopes v1
to exactly one fault type — single-rotor partial thrust degradation, step or
ramp onset, persisting to episode end. Intermittent profile is in
`planning.md`'s own *deferred* backlog; v1 here implements **step and ramp
only**.

**Ground-truth fault labels are logging-only, CLAUDE.md §1.7/anti-pattern
12.** A dedicated test (`test_no_fault_field_appears_in_observation_v1_or_
shared_features`) asserts none of the new fault fields appear in
`configs/rl/observation_v1.yaml` or as `side: shared` in
`configs/features.yaml` — written before anything else touches the schema.

**Dataset generation operating point: 2 workers, 1x speed** — not the
2-worker/4x figure `docs/throughput.md`'s original 16-configuration sweep
recommended. M4's own later soak test found 4x mostly didn't complete
missions at soak scale and reverted the recommendation to 1x; the 500-1000
episode dataset run is soak-scale, so it uses the soak-validated point.

### Tasks

1. ✅ **Fault schema config + pure sampler** (no simulator).
   `configs/faults/rotor_thrust_degradation_v1.yaml` (`fault_schema_version`,
   rotor/severity/onset/profile ranges, `healthy_fraction`),
   `experiments/fault_schedule.py` (`FaultSpec` dataclass,
   `load_fault_config`/`validate_fault_config`, pure
   `sample_fault_schedule(cfg, rng, n_episodes)` — seeded `Generator` passed
   explicitly, never `np.random` global state per anti-pattern 16). 21 tests
   in `tests/test_fault_schedule.py`, all passing. One real bug found and
   fixed immediately by the determinism test: `FaultSpec.onset_time_s` was
   originally `float('nan')` for the healthy sentinel, but a frozen
   dataclass with a NaN field is never equal to itself under `==` (`nan !=
   nan`), which silently broke `test_sampler_is_deterministic_given_seed`
   for any schedule containing a healthy episode. Fixed by making
   `onset_time_s: Optional[float]` (`None` sentinel instead of NaN) — schema
   v4 (task 2) maps `None` -> NaN only at the point a spec is written into a
   parquet column, where NaN is this project's established missing-float
   convention.
2. ✅ **Episode schema v3 → v4** (ground-truth fault labels). New closed
   enums (`fault_types`, `fault_profiles`) and episode-level fields
   (`fault_config_digest`, `fault_applied`, `fault_type`,
   `fault_rotor_index`, `fault_severity_commanded`,
   `fault_onset_time_s_requested/observed`, `fault_profile`,
   `fault_ramp_duration_s`, `fault_confirmed_applied`,
   `fault_confirmed_severity_final`, `px4_failure_detector_silent`), plus one
   step-level field (`px4_failure_detector_status`, straight passthrough of
   the already-subscribed `VehicleStatus.failure_detector_status` bitmask —
   zero new topics). `experiments/episode_schema.py`: `SCHEMA_VERSION =
   "4"`; `FaultType`/`FaultProfile` are **imported from
   `experiments.fault_schedule`**, not redefined (CLAUDE.md §1.4 — task 1
   already owns them). `mission_executor.py`'s `record_step()` gains the
   passthrough field. Every existing writer (`EpisodeRunner`,
   `SimFarm._synthesize_lost_episode_record`) updated to populate the new
   fields with `FaultSpec.healthy(0).to_episode_fields()` sentinels — no
   fault-injection caller exists yet (that's task 6), so every episode flown
   today is, factually, healthy; `px4_failure_detector_silent` is computed
   for real from each episode's own logged steps, not stubbed. The guard
   test required by CLAUDE.md §1.7 (`tests/test_fault_fields_not_in_
   observation.py`) was written in this same task, before any other schema
   file touched: 3 tests, including one that pins its own hand-listed field
   set equal to the schema's actual fault fields so it cannot silently go
   stale. 220/220 non-sim tests passing (was 190 at the end of M5; +30 across
   tasks 1-2).
3. ✅ **`RotorDegradationSystem` — the relay plugin.**
   `simulation/gz_plugins/CMakeLists.txt` +
   `simulation/gz_plugins/src/RotorDegradationSystem.{hh,cc}`. Built clean
   with `-Wall -Wextra`, zero warnings. Wrote the full
   `sqrt(1-severity)` relay math in one pass rather than a binary-only
   version first (task 7's own scope) — writing a deliberately-limited
   version now and rewriting it in task 7 would itself have been the kind
   of throwaway intermediate implementation CLAUDE.md warns against; task 7
   is still where graded severity gets its own dedicated sweep test.
   `gz.msgs.Param` control/status channel as planned, confirmed live via
   `gz topic -e` (heartbeat with correct no-fault sentinels arrives on
   worker startup). **Real finding, Python side only:** the apt-installed
   `python3-gz-msgs10` bindings do **not** expose `Param.params` as a
   Python dict-like map field, despite it being a real `map<string, Any>`
   on the wire (the C++ plugin side, and `gz topic -e`, both read/write it
   correctly) — indexing it with a string key raises `TypeError`; Python
   code must iterate it as a plain repeated field of `(key, value)` entries
   instead (`{e.key: e.value for e in msg.params}`). Confirmed directly by
   reproducing it standalone, not assumed from the traceback. Documented in
   the test and binding for task 5 (`simulation/rotor_fault.py`) to reuse.
4. ✅ **`x500_aero` model.** `simulation/models/x500_aero/{model.config,
   model.sdf}` — includes `model://x500_base` (PX4's own, untouched), same 4
   `MulticopterMotorModel` blocks as stock `x500` with `commandSubTopic`
   repointed, plus the new plugin block. **A real, load-bearing correction
   to this milestone's own design note above**, found live the first time
   `--model x500_aero` was actually tried: `px4-rc.gzsim` does **not**
   spawn a model via a generic `model://$MODEL_NAME` resolution (that was
   the plan's original, wrong reading of the source) — it hardcodes
   `"${PX4_GZ_MODELS}/${MODEL_NAME}/model.sdf"`, and `PX4_GZ_MODELS` always
   points at PX4's own models directory (its `gz_env.sh` unconditionally
   re-exports it, clobbering any override set beforehand). Fixed with no
   PX4 tree changes, confirmed live: `scripts/sim_start.sh` now spawns a
   project-local model itself, by absolute path, via a
   `gz.msgs.EntityFactory` `sdf_filename` request
   (`InstanceSpec.gz_spawn_request()`, new), and sets `PX4_GZ_MODEL_NAME` so
   `px4-rc.gzsim` attaches to it instead of trying to spawn its own —
   replicating the `set_physics` speed-factor call that branch would
   otherwise have skipped. Separately, rcS's airframe-by-filename lookup
   also fails for a project-local model name ("Unknown model ... not found
   by name"); fixed the same way, with `PX4_SYS_AUTOSTART=4001` reusing
   x500's own existing airframe. Both fixes live in one place
   (`simulation/instance_spec.py`'s `_AUTOSTART_OVERRIDE_FOR_MODEL`). A
   **second** real bug found live during verification: `run_episodes.py`
   and `SimFarm` didn't accept a `--model`/`model` parameter at all, so
   every caller silently built its `InstanceSpec` with the *default* model
   ("x500") regardless of which model the worker was actually started
   with — harmless for arm/fly (those don't depend on model name), but
   `aero_bridge/reset.py`'s `gz set_pose` soft reset addresses the entity
   by `spec.model_name` and failed outright ("gz set_pose to spawn failed
   for x500_0" — the wrong, default-derived name) on a real
   `x500_aero`-worker soft reset. Fixed by threading `model` through both
   (`run_episodes.py --model`, `SimFarm(model=...)`); `WorkerSupervisor` /
   `worker_process.py` already passed `spec.model` to `sim_start.sh -m`
   correctly, so no change was needed there. Verified live: a full healthy
   `square_circuit` flight on `x500_aero`, including a genuine soft reset,
   is indistinguishable from a plain `x500` run (RMSE ~5.6-6.3m, matching
   M3's documented ~6.44m noise floor).
5. ✅ **`simulation/rotor_fault.py`** — the Python control module
   (`RotorFaultController`: `set_rotor_fault`, `clear_rotor_fault`,
   `wait_for_heartbeat`, latest-status tracking), following
   `sim_clock.py`'s native-binding pattern. Uses the entry-iteration
   workaround from task 3's finding for both reading (status echo) and
   writing (fault command) `gz.msgs.Param`. Verified live against a real
   `x500_aero` worker: `wait_for_heartbeat` succeeds, `set_rotor_fault(2,
   0.5)` is confirmed via the echo (`latest_rotor_index/severity/applied`),
   `clear_rotor_fault()` confirmed clearing it back to the idle sentinel.
   Negative case verified too: pointed at a plain `x500` worker (no
   plugin), `wait_for_heartbeat` raises `RotorFaultControllerError` within
   its own timeout rather than hanging.
6. ✅ **`EpisodeRunner` integration.** `enable_rotor_fault: bool = False`
   constructor flag (default off, every M3/M4/M5 caller unaffected, verified
   — all pre-existing tests still pass unmodified);
   `run_episode(..., fault_spec=None, fault_config_digest="none")` drives
   onset/ramp timing against real sim-time ticks and populates the schema-v4
   fields from the status echo, never from the command alone. An
   unconditional `clear_rotor_fault()` runs before every fault-enabled
   episode's flight (soft/medium reset keeps the same gz sim process, and
   with it the same plugin instance, alive — a fault from the *previous*
   episode would otherwise still be active). 6 new unit tests against a fake
   controller (step commands once at onset; ramp rises linearly then locks;
   a healthy `FaultSpec` and `fault_spec=None` both write identical healthy
   sentinels; `fault_spec` without `enable_rotor_fault=True` raises).
   **Verified live** (`tests/sim/test_episode_runner_fault_injection.py`,
   2 tests): a real step fault is commanded and genuinely confirmed via the
   plugin's own echo during a real flight, and — the one place this design
   wasn't reusing an already-proven pattern wholesale —
   `RotorFaultController` survives a hard reset and correctly reconnects to
   the new gz sim process's plugin instance for a second faulted episode.
7. ✅ **Graded severity in the plugin** (build order steps 2-3). The full
   `sqrt(1-severity)` relay math was already written in task 3 (see that
   task's note on why); this task is its dedicated verification.
   `test_graded_severity_scales_relayed_velocity`
   (`tests/sim/test_rotor_fault_controller.py`) commands a known, marker
   velocity directly onto the real command topic (PX4 not needed — bypasses
   arming entirely, isolating the relay's arithmetic from flight physics per
   the milestone plan, the same technique task 8 uses) and confirms, for
   `s ∈ {0.0, 0.2, 0.5, 0.9}`, that the relayed velocity ratio equals
   `sqrt(1-s)` to `1e-6` — exact, not approximate, since this is pure
   arithmetic — and that the other three rotors are untouched. One test bug
   found and fixed live: waiting on `latest_severity` changing was a no-op
   for the `s=0.0` case, since that is also the attribute's un-echoed
   default; fixed by waiting on `latest_rotor_index` instead, reset to `-1`
   before each iteration.
8. ✅ **Cross-validation thrust fixture** (CLAUDE.md §1.6). **Deviated from
   the plan's force-torque-sensor design**, which the plan itself flagged
   as its one unverified piece: a real single-rotor fault is not a
   symmetric net-thrust loss (it's mostly attitude torque, which a real PX4
   controller would immediately start compensating for — exactly the
   confound this measurement needs to avoid), and getting the
   `gz-sim-forcetorque-system` SDF syntax right blind, with no example to
   reference, risked significant time for uncertain payoff. Used the
   already-verified, exact measurement instead:
   `scripts/measure_rotor_fault_thrust.py` commands a known velocity
   directly onto the real command topic (PX4 not in the loop at all) and
   reads the relayed velocity ratio — task 7's own live-measured result —
   then derives `thrust_ratio = velocity_ratio²` from
   `MulticopterMotorModel`'s documented (not reimplemented)
   `thrust = motorConstant·ω²` law. Written honestly as a **derivation**,
   not an independent physical sensor reading, with the limitation and the
   fallback (an independent force-torque or joint-telemetry measurement)
   spelled out in the script's own module docstring for M8b to revisit if
   its own cross-validation ever disagrees. `tests/fixtures/
   rotor_fault_thrust_curve.json` written from a real live run: exact match
   to `1-s` for `s ∈ {0.0, 0.2, 0.5, 0.9}` (max deviation ~2e-16, float
   rounding only). `tests/sim/test_rotor_fault_thrust_fixture.py`
   independently reproduces the same measurement live (loaded by file path
   via `importlib`, not `import scripts...` — `/opt/ros/humble`'s own
   dist-packages ships a real, `__init__.py`-bearing package also named
   `scripts` that Python's import system prefers over this project's
   `scripts/` directory regardless of `sys.path` order, confirmed live —
   sidesteps the collision rather than turning `scripts/` into a package
   project-wide to work around one test).
9. ✅ **Dataset generator.** `experiments/generate_fault_dataset.py`: samples
   a flat schedule once with a seeded `Generator`
   (`sample_fault_schedule`, task 1), partitions it into contiguous
   per-worker blocks (`build_fault_specs_by_worker`, pure, unit tested —
   resume-safe by construction: a respawned worker's `_worker_main`
   indexes its own already-assigned block by local episode index, the same
   mechanism episode ids already use), and threads it through `SimFarm`
   (`enable_rotor_fault`/`fault_specs_by_worker`/`fault_config_digest`
   constructor params — small, explicit additions to the existing worker
   loop, not a second implementation) to each worker's fault-aware
   `EpisodeRunner` (task 6). **Verified live**
   (`tests/sim/test_fault_dataset_small_run.py`): a real 2-worker × 3-episode
   mixed run produced 6/6 schema-v4-valid records through the actual
   `generate()` entry point, with real, honest variation — 5/6 faults
   confirmed applied (the 6th: `completed` before its (comparatively late)
   sampled onset time was ever reached, a real edge case worth `docs/
   fault_dataset.md` noting at scale, not a bug), and one episode where
   PX4's own `FailureDetector` was **not** silent — exactly the kind of
   real finding task 10's full run needs to characterize per severity, not
   something to paper over even in this small a sample.
10. ✅ **Real dataset generation run — complete: 750/750 episodes.** 2
    workers, 1x speed, `x500_aero`, `run_id=m6_dataset_v1`. `experiments/
    analysis/fault_dataset_report.py` (new) reads the run back and prints
    confirmation rate / FailureDetector-silence rate overall, per severity
    bucket, per profile, and per rotor -- `docs/fault_dataset.md` (new)
    written from its real output.

    **The actual research-premise check, and it's a nuanced real finding,
    not a clean yes.** PX4's own `FailureDetector` stays silent 100% of the
    time at severity `[0.2, 0.4)`, but only 63.3% of the time at `[0.8,
    1.0]` -- a clean, monotonic trend across all four severity buckets. The
    "PX4 doesn't notice" premise holds cleanly at the low end of
    `severity_range_s` and progressively breaks down toward the high end;
    documented honestly in `docs/fault_dataset.md` rather than rounded off,
    since it directly matters for how M7/M10 frame the detection problem's
    difficulty across the severity range. 741/750 episodes valid (98.8%);
    597 faulty / 153 healthy (20.4% -- matches the configured 20%
    `healthy_fraction` exactly); overall fault confirmation rate 77.9%,
    with the 22.1% unconfirmed traced almost entirely (125/132) to missions
    that `completed` before their sampled onset time was ever reached -- the
    same real, understood edge case task 9's small run first surfaced, now
    confirmed and quantified at full scale, not a defect.

    **Real interruption and resume, live, not hypothetical.** The machine's
    disk filled to ~500MB free mid-run (unrelated to this run's own small
    footprint). Stopped cleanly: `SIGINT` to the generator process (its
    `with SimFarm(...) as farm:` unwinds properly on `KeyboardInterrupt`),
    then `scripts/sim_stop.sh --all --sweep` to clear a handful of
    processes orphaned by the interrupt landing mid-worker-restart — zero
    orphans confirmed via `ps aux` before the machine was restarted to
    resize the partition. 131-133 episodes' worth of real data (a handful
    of per-worker parquet files were still landing as the interrupt hit)
    was intact and valid on disk throughout, never at risk.

    **Built real resume support** rather than re-flying already-completed
    episodes (found needed live, not speculative): `SimFarm(resume=True)`
    scans each worker's own `results/<run_id>/worker_<k>/` for existing
    `episode_ep_*_summary.parquet` files and seeds `_completed_per_worker`
    from the highest existing index + 1, so the initial spawn (not just a
    mid-run `WorkerSupervisor` restart, which already resumed correctly)
    picks up each worker's own next unused episode index instead of
    restarting numbering at 0 and overwriting real data. The fault schedule
    itself is never persisted or reloaded — `sample_fault_schedule`/
    `build_fault_specs_by_worker` are pure functions of
    (config, seed, n_episodes, worker_count), so re-deriving it with the
    identical arguments reproduces the identical per-worker schedule
    deterministically; resuming only changes which INDICES actually get
    flown. One real edge case fixed along the way: a worker already at its
    full quota on resume is never spawned (`sup.process` stays `None`),
    which `run()`'s completion/health-check loops did not originally guard
    against (`sup.process.is_alive()` on `None` would crash) — fixed and
    covered by `test_run_skips_spawning_a_worker_already_at_full_quota`.
    4 new tests in `tests/test_sim_farm_assignment.py`, all passing.
    Verified live: `--resume` picked up at `worker 0 ep_0064` (immediately
    after the last episode on disk, `ep_0063`), not `ep_0000`.
11. ✅ **PX4 tree cleanliness + wrap-up.** `git -C ~/projects/PX4-Autopilot
    status --porcelain` shows only the same 5 pre-existing untracked
    submodule-clutter directories present before this milestone started
    (`boards/modalai/voxl2/src/lib/`, `src/lib/rl_tools/`,
    `src/modules/mc_raptor/`, `src/modules/simulation/gz_plugins/
    optical_flow/PX4-OpticalFlow/`, `src/modules/uxrce_dds_client/
    Micro-XRCE-DDS-Client-v3/`) — zero modifications to any tracked file,
    zero new patches, still pinned to `v1.17.0` on branch `aero-safe-rl`.
    Confirms the design's central claim: the entire fault-injection
    mechanism (plugin, model, spawn path, airframe reuse) needed no PX4
    tree changes at all, despite two real corrections along the way to how
    that was achieved (tasks 3-4's design note above).

### Files created

```
configs/faults/rotor_thrust_degradation_v1.yaml
experiments/fault_schedule.py
experiments/generate_fault_dataset.py
simulation/gz_plugins/CMakeLists.txt
simulation/gz_plugins/src/RotorDegradationSystem.hh
simulation/gz_plugins/src/RotorDegradationSystem.cc
simulation/models/x500_aero/model.config
simulation/models/x500_aero/model.sdf
simulation/rotor_fault.py
scripts/measure_rotor_fault_thrust.py
tests/fixtures/rotor_fault_thrust_curve.json
experiments/analysis/fault_dataset_report.py
docs/fault_dataset.md
configs/schema/episode_record.yaml             (v3 -> v4)
experiments/episode_schema.py                  (SCHEMA_VERSION bump)
experiments/episode_runner.py                  (fault_spec integration)
experiments/sim_farm.py                        (fault_specs threading, resume=True)
experiments/run_episodes.py                    (--model threading)
simulation/instance_spec.py                    (_AUTOSTART_OVERRIDE_FOR_MODEL,
                                                 gz_spawn_request())
scripts/sim_start.sh                           (project-local model spawn path)
ros2_ws/src/aero_bridge/aero_bridge/mission_executor.py   (record_step)
```

### Tests (required)

```
tests/test_fault_schedule.py
tests/test_generate_fault_dataset_assignment.py
tests/test_episode_schema.py                   (extended — v4)
tests/test_mission_executor.py                 (extended)
tests/test_episode_runner.py                   (extended — fault_spec)
tests/test_fault_fields_not_in_observation.py
tests/test_instance_spec.py                    (extended — model spawn/autostart)
tests/test_sim_farm_assignment.py              (extended — fault_specs, resume)
tests/sim/test_rotor_fault_plugin_loads.py
tests/sim/test_x500_aero_model_loads.py
tests/sim/test_rotor_fault_controller.py
tests/sim/test_episode_runner_fault_injection.py
tests/sim/test_rotor_fault_thrust_fixture.py
tests/sim/test_fault_dataset_small_run.py
tests/slow/test_fault_dataset_run.py
```

### Verify with

```bash
pytest tests/ -m "not sim and not slow" -q
cmake -S simulation/gz_plugins -B simulation/gz_plugins/build && cmake --build simulation/gz_plugins/build
pytest tests/sim -k rotor_fault -q
pytest tests/slow/test_fault_dataset_run.py -q
git -C ~/projects/PX4-Autopilot status --porcelain    # must be empty
scripts/sim_stop.sh --all
```

### Watch out for

- The C++ plugin is genuinely concurrent (gz-transport delivers subscription
  callbacks on its own threads; relay logic runs alongside the physics
  thread) in a way nothing else in this repo's C++ surface is — real
  scrutiny at implementation time, not just a design read-through.
- Don't let the real dataset run (task 10) be the first time hard-reset ->
  `RotorFaultController` rebuild gets exercised — task 6's sim test must
  cover it first, the same lesson M4's soak test already taught this project.
- It is tempting to add a fault field as a "convenience" shared feature —
  don't; that is exactly the violation CLAUDE.md warns Isaac makes trivially
  easy, and the ROS side is just as easy to get wrong once the field is
  sitting right next to the real features in the same row.

---

# M7 — AI fault detector ✅

**Goal:** a model that reads the telemetry stream and reports, every tick,
fault presence, which rotor, severity, and how sure it is. The first
genuinely novel result.

**Depends on:** M5, M6. **Blocks:** M8, M8b (the detector-output simulator is
fitted to this milestone's measured error), M9.

**Started 2026-09-23.** Detail below written at start, per the stub policy.

### What the M6 dataset actually looks like (measured 2026-09-23, before any design)

A throwaway probe over `results/m6_dataset_v1/` settled three things the stub
could only guess at. Numbers are per tick, on ticks after the fault has fully
settled, healthy ticks taken after the first 10 s of flight:

| severity | faulty rotor's command minus 4-rotor mean (median) | ticks over the healthy 99th percentile |
|---|---|---|
| healthy | 0.004 (p99 = 0.083) | 1% by construction |
| [0.2, 0.4) | 0.164 | 98.4% |
| [0.4, 0.6) | 0.399 | 99.4% |
| [0.6, 0.8) | 0.476 | 99.3% |
| [0.8, 1.0] | 0.467 | 99.4% |

1. **A settled fault is easy.** One hand-written statistic — the largest
   per-rotor motor command minus the mean of the four, averaged over the
   1.5 s feature window — separates settled faults from healthy flight almost
   perfectly, even at severity 0.2. "Which rotor" from the same statistic's
   argmax is right 100 / 98 / 91 / 90% of the time by severity bucket.
   A 10 s average instead of 1.5 s barely changes this, so *longer memory
   buys little at steady state.*
2. **Severity is the hard output.** The motor-command imbalance saturates
   above about s = 0.4 (0.40 → 0.48 → 0.47): the healthy motors hit their
   command limit and the imbalance stops growing. Severity above 0.4 must be
   read from the *consequences* — rates, attitude error, position error,
   `thrust_accel_residual` — not from the motor commands alone. Rotor
   identification also degrades at high severity for the same reason.
3. **The real detection problem is the transient** — the first second after
   onset, and ramps still in progress, where the instantaneous severity is
   small. That is where detection delay is decided, and delay is what the
   recovery policy actually feels.

**Consequence for what "beats the baselines" means.** Planning.md's exit
criterion ("detector beats threshold and classical baselines") will almost
certainly *not* hold on settled-fault AUROC — the threshold baseline is
already near the ceiling. The honest comparison is on the axes where there is
headroom: **detection delay** (onset → first sustained positive),
**false alarms per healthy flight-hour**, **rotor-ID accuracy**,
**severity error**, and **calibration** of the confidence output. A tie on
AUROC is reported as a tie (anti-pattern 13 — the threshold baseline is not
weakened to make the model look better).

### Model choice — proposed and confirmed by the user 2026-09-23

The stub said "a small 1D-CNN or GRU". The recommendation, from the data
above, is a **rotor-symmetric streaming GRU with a deep-ensemble head**
(working name `RotorGRU`):

- **Streaming, not windowed.** A GRU consumes one feature vector per 10 Hz
  tick and carries its hidden state across the whole episode (reset at
  episode start). No window-length hyperparameter, one output per tick, and
  it can accumulate evidence through an onset transient or a ramp — the one
  place (point 3) where memory does matter. A windowed 1D-CNN recomputes from
  scratch every tick over 15 frames and can only use what fits in 1.5 s.
- **Rotor-symmetric encoder.** Before the GRU, a fixed (parameter-free)
  transform re-expresses the inputs from each rotor's own point of view,
  using the x500 geometry in PX4's own `4001_gz_x500` airframe
  (`CA_ROTOR{0..3}_PX/PY/KM`): that rotor's command minus the mean; the body
  roll/pitch rate and attitude projected onto the direction of that rotor's
  arm; the yaw rate signed by that rotor's spin direction. **One shared
  GRU** then processes each rotor's view (batch × 4), so the network learns
  "what a weakening rotor looks like" once rather than four times — 4× the
  effective data from 465 faulty episodes — and rotor identification falls
  out as "which rotor's view looks worst". The global features
  (velocities, position error, battery, residual) are concatenated to every
  rotor's view.
- **Output head** matches planning.md §7.2's detector output exactly:
  a 5-way softmax {healthy, rotor 0..3} (giving `p(fault)` = 1 − p(healthy)
  and the rotor class), a severity regression, and an uncertainty.
- **Uncertainty from a 5-member deep ensemble** (5 independently seeded
  copies — the net is ~10k parameters, so training 5 costs minutes on CPU),
  plus temperature scaling on the validation split. Ensemble disagreement is
  the most dependable uncertainty signal at this data scale, and M8b needs a
  *calibrated* one to fit its detector-output simulator.

Why not the alternatives: a plain windowed 1D-CNN (limited to 1.5 s, learns
each rotor separately); a Transformer (not at 733 episodes); a pure
physics/parameter-estimation observer (it is the right idea for settled
faults, which is exactly what the threshold baseline already captures).

Cost vs a plain GRU: about one extra day — the geometry transform and one
test that relabelling the rotors consistently relabels the outputs. Side
benefit, not the reason: the same network runs unchanged on a 6-rotor
geometry for M12.

### Tasks

1. ✅ **Labelled dataset + episode-level split** (pure, no simulator, no
   model). `ai/detector/dataset.py` loads a fault-dataset run directory into
   one record per episode: the causal feature series from
   `ai.features.feature_extractor.extract_series` (never a second feature
   implementation) plus per-tick labels (`fault_active`, `rotor_class`,
   `severity`). Labels come from the episode summary via one pure function,
   `experiments.fault_schedule.commanded_severity()`, which
   `EpisodeRunner` also uses to command the fault — so the label is, by
   construction, what was sent to the plugin at that tick. Onset tick = the
   first step whose elapsed time since the episode's first step reaches
   `fault_onset_time_s_requested` (the runner's own clock reference;
   measured plugin echo lag: median 0.10 s, p95 0.20 s).
   Episode inclusion (measured counts on `m6_dataset_v1`):

   | category | n | treatment |
   | --- | --- | --- |
   | fault applied (onset reached, plugin echoed within 1 s) | 486 | faulty labels at the instantaneous commanded severity (includes 25 ramps cut short by mission end) |
   | fault commanded, onset after mission ended | 101 | all ticks healthy — physically healthy flights |
   | healthy | 142 | all ticks healthy |
   | onset reached but no echo, or echo 1.3–5.9 s late | 5 | excluded — onset time unverifiable |
   | `valid=False` or zero steps | 16 | excluded |

   `split_by_episode()` returns train/val/test **episode-id sets**
   (70/15/15, stratified by healthy / severity bucket, seeded `Generator`),
   and the only way to materialise training arrays takes a split and an
   episode set — there is no function that accepts timestep indices. Split
   digest recorded for later checkpoints.
   **Done 2026-09-23.** On `m6_dataset_v1`: 729 episodes included, 361,504
   ticks (54.8% fault-active), loads in ~50 s. Split at seed `20260923`:
   510 / 109 / 110 episodes, digest `defe672652cd753d`, every severity
   stratum present in every part. 11 ticks have NaN `battery_remaining`
   — task 3's input normalisation must handle it, not drop the episodes.
   24 tests in `tests/test_detector_dataset.py` plus 11 for
   `commanded_severity` in `tests/test_fault_schedule.py`.
2. ✅ **Metrics + baselines.** Detector metrics go in `experiments/metrics.py`
   (M10's "one place any metric is computed" — created here, extended
   there): per-severity AUROC, detection delay distribution (onset → first
   positive sustained 0.5 s), false alarms per healthy flight-hour, rotor-ID
   accuracy, severity MAE, expected calibration error. Baselines in
   `ai/detector/baselines.py`: (a) threshold on the motor-imbalance
   statistic above, with the same 0.5 s debounce; (b) threshold on
   `thrust_accel_residual`; (c) random forest on window summary statistics.
   Thresholds picked on val, reported on test. Adds `scikit-learn` to
   `environment.yml`.
   **Done 2026-09-23.** Threshold rule, applied identically to every
   detector: the 99.5th percentile of the detector's own score on healthy
   val ticks. scikit-learn 1.7.2 (+ joblib, threadpoolctl) installed and
   pinned. 11 tests in `tests/test_detector_metrics.py` (hand-built
   sequences with known answers), 7 in `tests/test_detector_baselines.py`.
3. ✅ **Main model + training** — `ai/detector/model.py`, `ai/detector/train.py`.
   The architecture confirmed from the section above. Checkpoint records
   `feature_version`, the normalisation-stats digest, the split digest, and
   the ensemble seeds; loading one with a mismatched digest raises.
   **Done 2026-09-23.** Architecture exactly as proposed above: 9 rotor-local
   + 7 global inputs, embed 32, GRU hidden 48, ~10k parameters per member.
   Rotor geometry is a constant in `model.py`, checked against PX4's
   airframe file by a test. Training: whole-episode BPTT, Adam 2e-3, early
   stopping on val loss (patience 12). The 5 members stopped at epochs
   16–54 with val loss 0.061–0.078. Temperature 1.1, alarm threshold
   p = 0.097, both fitted on val. ~2 min total on the RTX 2070 (plus ~50 s
   of data loading). The mirror-symmetry test holds to 1e-5 for arbitrary
   weights. A deliberate mutation (ignoring spin direction) breaks it by
   0.08, so the test is not vacuous. 9 tests in
   `tests/test_detector_model.py`.
4. ✅ **Evaluation report + M8b error model.** `ai/detector/evaluate.py` →
   `docs/detector_results.md` (model vs all three baselines, every metric
   per severity bucket and per profile, test split only) and
   `results/m7_detector_v1/error_model.json` — the measured delay
   distribution, false-alarm rate and severity-error-vs-true-severity that
   M8b's detector-output simulator is fitted to.
   **Done 2026-09-23.** Full tables and interpretation:
   `docs/detector_results.md`. Headline, test split:
   - **Weak faults (s 0.2–0.4):** median detection delay 0.88 s vs the
     random forest's 1.18 s.
   - **Rotor ID:** 98.6–99.8% accurate vs 93–96%.
   - **Severity MAE:** 0.013–0.022 vs up to 0.116. The random forest
     under-reads severity above s ≈ 0.4, as the probe predicted.
   - **Confidence:** ECE 0.004, and uncertainty flags wrong calls with
     AUROC 0.91.
   - **Tick AUROC** is a tie (0.998 vs 0.997).
   - **The model loses on false alarms:** 6 vs 4 in 0.69 healthy
     flight-hours. All 6 are short (≤ 0.9 s) and fall around the first
     waypoint turns.

   The motor-imbalance threshold misses every weak fault because under
   the shared protocol takeoff transients set its threshold: 94% of its
   healthy exceedances are in the first 10 s. Reported with its cause;
   not re-tuned. `thrust_accel_residual` alone measures AUROC 0.40, so
   M5's open question about it is answered: on its own it is
   uninformative.
5. ✅ **Online runtime + live check.** `ai/detector/runtime.py`: a stateful
   `DetectorRuntime` (`reset()` per episode, `step(frame) -> DetectorOutput`),
   pure Python/torch, CPU, < 20 ms per tick. Verified live by attaching it
   through `EpisodeRunner`'s existing `on_step` hook on **two concurrent
   workers** with injected faults.
   **Done 2026-09-23. Passed live**: instance 0 (rotor 1, s = 0.5) and
   instance 1 (rotor 3, s = 0.3) flew at the same time, each with its own
   Gazebo server, at 1×. Each alarmed 0.53 s / 0.59 s after onset on the
   correct rotor, with no alarm before onset. Tick latency live: median
   4.0 / 4.4 ms, p99 9.9 / 10.7 ms. `sim_stop.sh --all` left 0 processes.
   **One real finding, fixed before it passed.** The first live run
   failed the 20 ms budget: p99 34.5 ms, although the same code measured
   3.9 ms median offline. A tick is dominated by per-operation overhead
   (5 members run one after another, each re-doing the same input
   transform), and two simulators competing for CPU multiplied that
   overhead. Fixed with `StreamingEnsemble` in `model.py`: all members'
   weights stacked into one set of batched operations, transform computed
   once. That is 4× faster offline (0.98 ms median), and it matches batch
   inference to 1e-7 on the trained weights. Batch/training keep
   `nn.GRU`, and the streaming-equals-batch test covers the two paths.

### Files created

```
ai/detector/__init__.py
ai/detector/dataset.py              (task 1)
experiments/metrics.py              (task 2)
ai/detector/baselines.py            (task 2)
ai/detector/model.py                (task 3; StreamingEnsemble added in task 5)
ai/detector/train.py                (task 3)
ai/detector/evaluate.py             (task 4)
ai/detector/runtime.py              (task 5)
docs/detector_results.md            (task 4)
experiments/fault_schedule.py       (task 1 — commanded_severity())
experiments/episode_runner.py       (task 1 — uses commanded_severity())
environment.yml                     (task 2 — scikit-learn)
```

### Tests (required)

```
tests/test_detector_dataset.py      labels vs hand-computed step/ramp episodes, inclusion rules,
                                    split disjoint + deterministic + stratified, no timestep split API
tests/test_fault_schedule.py        (extended — commanded_severity)
tests/test_detector_metrics.py      delay/false-alarm/AUROC on hand-built sequences with known answers
tests/test_detector_baselines.py
tests/test_detector_model.py        rotor-relabelling equivariance, causality (future ticks
                                    cannot change past outputs), checkpoint digest mismatch raises
tests/test_detector_runtime.py      streaming output == batch output on a recorded episode; reset works
tests/sim/test_detector_live.py     two workers, injected fault, detector fires after onset
tests/sim/conftest.py               (extended — sim_workers_0_1_x500_aero fixture, 1x)
```

### Done when

- [x] Every metric in task 2 reported per severity bucket for the model and
      all three baselines, on the **test episodes only**
      (`docs/detector_results.md`).
- [x] Model is better than the best baseline on detection delay at
      s ∈ [0.2, 0.4) (median 0.88 s vs 1.18 s) and on severity MAE (0.018 vs
      0.020 in that bucket, 0.013–0.022 vs up to 0.116 above it). It is
      worse on false alarms (6 vs 4 events in 0.69 h), reported as such.
- [x] `error_model.json` written and documented — M8b's input.
- [x] Online inference < 20 ms/tick, verified live on two concurrent
      workers (p99 ≤ 10.7 ms).
- [x] Default test suite passes. **Caveat:** that tier now takes ~7.7 s,
      over the 5 s target in CLAUDE.md §6. ~2 s of it is importing torch
      and scikit-learn at collection.

### Watch out for

- Normalisation: use the frozen `configs/rl/normalization_v1.yaml` (healthy
  flights, all 19 features). Recomputing stats on the training split is
  anti-pattern 11 in a different coat.
- A checkpoint picked by best *test* score is test-set leakage. Model
  selection uses val; test is read once, by task 4.
- One mission (`square_circuit`) only — every number here is in-distribution
  on trajectory. M11 measures the rest.


---

# M8 — Rule-based recovery baseline ✅

**Goal:** a genuinely well-tuned, non-learning recovery system — what the RL
policy has to beat. Under-tuning this to make RL look better invalidates the
whole comparison; reviewers see through it immediately.

**Depends on:** M7. **Blocks:** M9 (shares its interface), M10.

**Key deliverables:**
- The shared policy interface (`rl/policies/base_policy.py`) comes **first** —
  both the FSM and the eventual RL policy implement it, so M10's comparison
  can't be skewed by the RL policy quietly having powers the FSM lacks.
- A `NORMAL → SUSPECTED → CONFIRMED → RECOVERING → LANDED/ABORTED` state
  machine with hysteresis (a flickering detector must not cause mode thrash),
  and thresholds tuned via a documented, archived sweep.
- Verified not to panic-land a healthy drone on a brief false alarm.

**Started 2026-09-23.** Detail below written at start, per the stub policy.

### What the M6 dataset says about recovery (measured 2026-09-23, before any design)

A throwaway probe over the 741 valid `m6_dataset_v1` flights, looking at what
happens *after* onset:

| true severity | reaches the ground mid-flight | vertical speed at impact (median) |
|---|---|---|
| ≤ 0.30 | 0% — mission completes | — |
| 0.30–0.35 | 9% | — |
| 0.35–0.40 | 34% (50% if moving > 1 m/s at onset, 26% if hovering) | ~1.5 m/s |
| 0.40–0.45 | 94% | 2.5 m/s |
| 0.45–0.50 | 88–100% | 3.4 m/s |
| 0.50–0.60 | 96% | 4.7 m/s |
| > 0.60 | ~96%, mostly flipped after impact | 6–7 m/s |

1. **There is a hard physical limit at s\* ≈ 0.41.** The x500 (2.06 kg,
   8.55 N max per rotor — stock PX4 physics) needs 59% of its total thrust to
   hover; healthy motor command is 0.74. Level, yaw-balanced hover needs all
   four rotors at equal thrust, so a rotor that has lost more than
   1 − 0.59 = 0.41 caps the total below the weight. The measured cliff sits
   exactly there. **Kept as is and reported as a finding** (user decision
   2026-09-23) — raising the thrust margin would invalidate M6 and M7.
2. **Recovery can matter only between about s = 0.30 and 0.50.** Below 0.30
   the mission succeeds unaided. At 0.35–0.40 speed matters (above). At
   0.40–0.50 the drone comes down regardless, but a fall's impact speed grows
   with √height, so descending to ~2 m on suspicion turns ~2.5 m/s impacts
   into ~1.6 m/s ones. Above 0.50 no high-level action helps; C2/C3/C4 are
   expected to tie there in M10, and that is reported, not hidden.
3. **Nothing recognised a crash.** The 332 `hold_timeout` episodes are mostly
   drones on the ground while the 60 s wall watchdog runs out. M8 adds a real
   `crashed` termination, which also saves ~1 min of wall time per crash.
4. **Every flight is flown flat out.** A waypoint is a position jump, so PX4
   flies each leg at ~9 m/s peak with tilt up to its 45° cap. "Slow down"
   therefore needs a moving setpoint ("carrot") generated on our side.

### Design — confirmed by the user 2026-09-23

- **Action space `action_v1` — 3 dims** (not planning.md §7.2's 5):
  `speed_scale` ∈ [0, 1] (carrot speed along the mission path as a fraction
  of PX4's `MPC_XY_VEL_MAX` = 12 m/s; 0 = hold position), `altitude_offset_m`
  ∈ [−3.5, 0] (relative to mission altitude, reached at a fixed vertical
  rate), `land` ∈ [0, 1] (≥ 0.5 commits to PX4's own land at the current
  position — irreversible). The 5-dim version's "progress rate" duplicated
  `speed_scale = 0`, and its climb-rate scale bought nothing this data shows.
- **Nominal action = today's flight.** `speed_scale = 1` moves the carrot at
  12 m/s, which reaches a 15 m waypoint in 1.25 s, so the flight is
  effectively the M6 position jump. Verified live (task 3), because the
  detector was trained only on those flights. C1/C2 fly the nominal action
  through the same code path as C3/C4, so the setpoint generator is never a
  confound.
- **Outcome, per episode** (pure classifier in `experiments/metrics.py`):
  `crash` if tilt ever exceeds 60° or the vehicle touches the ground at more
  than **2.0 m/s** vertical (user decision; sensitivity at 1.5 and 2.5 m/s
  reported); otherwise `mission_success` if every waypoint was reached and it
  landed, `safe_landing` if it landed before finishing, `incomplete` if
  neither. Frozen before any tuning run.
- **What a policy sees** — one `PolicyInput` per 5 Hz decision: the
  observation_v1 features, the detector's `DetectorOutput`, and mission
  progress (waypoint index, distance to it, altitude). The FSM and M9's RL
  policy receive exactly this and return exactly an `action_v1` action. M9
  freezes the flat vector layout; M8 fixes the content.
- **FSM tuning is split** so the expensive part stays small. The *detection*
  side (suspect threshold, confirmation hold) is tuned offline by replaying
  detector traces over M7's **validation** episodes. Every M7 healthy false
  alarm lasted ≤ 0.9 s, so the hold should come out longer than that. The
  *response* side (degraded speed, degraded altitude, severity above which to
  land at once) needs the simulator. A small sweep on seeds disjoint from
  M10's evaluation seeds.

### Tasks

1. ✅ **Contracts + pure pieces.** `configs/rl/action_v1.yaml`;
   `rl/policies/base_policy.py` (`PolicyInput`, `Action` decoded and clipped
   from the spec, `BasePolicy`, `NominalPolicy` = no recovery);
   `rl/mission_tracker.py` (the carrot: one pure object advanced by sim-time
   dt, the only PX4-side meaning of an action). No simulator.
   **Done 2026-09-23.** `MissionTracker` also takes over waypoint
   sequencing: reach within the acceptance radius (3D, at the current
   altitude offset), hold, advance, then the final hover. That is the same
   sequence `fly_mission` flies today, so task 3 can swap it in without
   changing what "waypoint reached" means. A waypoint counts only once the
   carrot has arrived at it. Takeoff to mission altitude is not
   rate-limited, only changes of the offset are, so the nominal flight
   matches M6's. The spec carries a digest for M9 checkpoints.
   26 tests in `tests/test_action_spec.py` and `tests/test_mission_tracker.py`.
2. ✅ **Outcomes + crash detection.** `classify_outcome()` in
   `experiments/metrics.py`; online crash detection in `fly_mission`; episode
   schema v5 (`crashed`, `recovery_landed`; per-step action, detector output
   and policy state).
   **Done 2026-09-23.** The outcome thresholds became a contract file,
   `configs/rl/outcome_v1.yaml`, because M8b's Isaac reward and M9's
   transfer table must judge flights by the same rule. The online
   termination is `ground_contact`, not `crashed`: the flight ends at any
   uncommanded touchdown, and whether that was a crash is decided offline
   from touchdown speed and tilt. A slow sink onto the ground is a
   `safe_landing`. Schema v5 adds `ground_contact` / `recovery_landed`,
   policy provenance (policy name, config, action-spec and detector
   digests) per episode, and per step the flight phase, action, FSM state
   and detector output. Landings are now recorded step by step, since
   touchdown is what the crash rule judges. RMSE still counts
   mission-phase steps only, so it stays comparable with M3/M6.
   14 tests in `tests/test_recovery_outcome.py`.
3. ✅ **Policy-driven flight.** `fly_mission` driven by a policy through
   `MissionTracker`, plumbed through `EpisodeRunner`/`SimFarm` (policy and
   detector built inside the worker process, §3.3). Live on 2 workers:
   `NominalPolicy` healthy flights match M6's within the D11 band (duration,
   RMSE, peak speed, detector false alarms).
   **Done 2026-09-23.** `rl/policy_driver.py` runs the detector on every
   10 Hz step and the policy at 5 Hz of sim time, latches a land decision,
   and annotates each row. `RecoveryConfig` is the picklable description
   SimFarm passes to workers. `hold_position_until` gained a `setpoint_fn`
   and `land_and_wait` an `on_poll` hook. A missing sim-clock read now
   skips the tick instead of stamping t = 0, which the feature windows
   would reject as time going backwards. One runner,
   `experiments/run_recovery.py`, serves every M8 simulator run.
   **Live check** (`results/m8_check_nominal`, 2 workers, 1×): 6 healthy
   flights completed in 41.1–44.0 s (M6 median 43.9 s), RMSE 6.34–6.82 m
   (M6 p10–p90 5.97–6.86), peak speed 9.0–9.2 m/s (M6 8.97), max tilt
   42–44° (M6 43.9°). The two s = 0.5 faults ended as `ground_contact`
   after 11 and 15 s of flight, instead of a 60 s hang. `sim_stop` left 0
   processes. `tests/sim/test_policy_flight.py` passed on 2 concurrent
   workers: nominal flight completed and was classified `mission_success`;
   the FSM against s = 0.6 committed to land 1.25 s after onset and still
   touched down at 5.4 m/s (a crash — above s ≈ 0.5, as expected).
   12 tests in `tests/test_policy_driver.py`, 2 new in
   `tests/test_arming_sequence.py`.
4. ✅ **The FSM** — `rl/policies/rule_based.py`, `configs/rl/fsm_v1.yaml`, and
   the offline detection-side tuning on M7's validation split.
   **Done 2026-09-23.** States NORMAL → SUSPECTED → RECOVERING (continue,
   degraded) or ABORTED (land). planning.md's CONFIRMED is the transition
   between them, and LANDED is the judged outcome. A confirmation latches,
   because the fault persists. Hysteresis runs on both thresholds and both
   hold times. **Detection tuning, with one honest revision.** The rule
   as first written (zero false confirmations on 109 validation episodes,
   then the fastest confirmation) picked suspect_p 0.3 with a 0.4 s hold.
   The first live healthy flights showed that was too short: one flight had
   p ≥ 0.3 for 0.72 s at a waypoint turn and would have been falsely
   confirmed. Validation's 0.76 healthy hours never showed such a run; the
   train split's 3.56 healthy hours had one of 1.03 s. The rule now also
   requires the hold to exceed the longest healthy run above the threshold
   in any non-test data, by one decision period. It picks **suspect_p 0.5,
   hold 1.0 s**, with median confirmation 1.63 s after onset at
   s 0.3–0.5. The revision was made before any response tuning, and is
   written in `fsm_v1.yaml` and `results/m8_fsm_tuning/detection_sweep.json`.
   `experiments/tune_fsm.py` reproduces both steps. 12 tests in
   `tests/test_rule_based_policy.py`, 11 in `tests/test_recovery_tools.py`.
5. ✅ **Response sweep** on the simulator, archived under
   `results/m8_fsm_sweep_*` (configs in `results/m8_fsm_sweep_configs/`).
   **Done 2026-09-23.** Six settings × s {0.35–0.50} × 8 flights, plus a
   no-recovery reference. The two leaders differed by one flight, so both
   were flown 12 more times (user decision: fly more, keep the rule). The
   defaults won on pooled crash rate (39% vs 45%), at the stated cost of
   aborting 95% of s 0.35 missions. Result recorded in `fsm_v1.yaml`.
6. ✅ **Validation run + report.** The rule-based recovery controller vs
   `NominalPolicy` across severities, plus healthy flights (no panic
   landings) → `docs/recovery_baseline.md`.
   **Done 2026-09-23.** Seed 8201, 137 valid flights. **The controller does
   not beat no recovery.** Crash rates are equal or worse at every severity,
   and it gives up missions at s 0.2–0.35. Zero landings on 14 healthy
   flights. Cause, found in the logs: its own 3 m recovery descent makes
   the detector (trained on level flight only) over-read severity
   (0.2–0.3 → 0.4–0.7), which triggers "land now". This matters for M8b's
   detector-output simulator.

### Files created

```
configs/rl/action_v1.yaml           (task 1)
rl/__init__.py, rl/policies/__init__.py
rl/policies/base_policy.py          (task 1; outcome + observation loaders in 2/3)
rl/mission_tracker.py               (task 1)
configs/rl/outcome_v1.yaml          (task 2)
experiments/metrics.py              (task 2 — classify_outcome)
configs/schema/episode_record.yaml  (task 2 — v5)
rl/policy_driver.py                 (task 3)
aero_bridge/mission_executor.py     (task 3 — flies through MissionTracker + PolicyDriver)
aero_bridge/arming_sequence.py      (task 3 — setpoint_fn, on_poll)
experiments/episode_runner.py, experiments/sim_farm.py  (task 3 — RecoveryConfig plumbing)
experiments/run_recovery.py         (task 3 — every M8 simulator run + scoring)
rl/policies/rule_based.py           (task 4)
configs/rl/fsm_v1.yaml              (task 4)
experiments/tune_fsm.py             (tasks 4–5)
docs/recovery_baseline.md           (task 6)
```

### Tests (required)

```
tests/test_action_spec.py           decode/clip, land threshold, spec matches the dataclass
tests/test_mission_tracker.py       carrot speed = scale x v_max, hold at 0, altitude rate, waypoint advance
tests/test_recovery_outcome.py      crash / safe_landing / success / incomplete on hand-built series
tests/test_rule_based_policy.py     hysteresis (flicker does not thrash), ≤0.9 s alarm never confirms,
                                    CONFIRMED latches, severity -> response table, land is irreversible
tests/test_policy_driver.py         5 Hz sim-time cadence, land latch, row annotation, no ground truth
tests/test_recovery_tools.py        schedule, FSM replay, detection + response selection rules
tests/sim/test_policy_flight.py     2 workers: nominal flight matches M6; FSM lands on an injected fault
```

### Done when

- [x] The rule-based recovery controller beats `NominalPolicy` on crash rate
      in the s 0.35–0.50 band, or the report says plainly that it does not.
      **It does not** — `docs/recovery_baseline.md`.
- [x] Zero controller-commanded landings over the healthy validation flights
      (14 of 14).
- [x] Sweep archived and documented, thresholds chosen by a rule written down
      before the sweep ran. One revision to the detection rule, made before
      response tuning and documented.
- [x] Default test suite passes; sim tests run on 2 concurrent workers.

---

# M8b — Isaac Lab training environment  ⭐ new (D12)

**Goal:** the environment the policy actually trains in — N parallel quadrotors
on GPU, implementing the *same* frozen observation/action spec as the
PX4-in-the-loop evaluation environment.

**Depends on:** M3b (feasibility), M5 (the shared/PX4-only feature split),
M6 (the Gazebo fault model and its fixture), M7 (the detector's measured error
characteristics). **Blocks:** M9.

**Key deliverables:**
- An Isaac Lab quadrotor task with a geometric position/velocity controller
  standing in for PX4's position loop, stepped at the same **5 Hz** decision
  rate as the evaluation env.
- The **Isaac-side rotor degradation model**, cross-validated against M6's
  recorded fixture (`CLAUDE.md` §1.6). This, not the RL, is the milestone's
  real risk.
- A **detector-output simulator**. During Isaac training the real detector
  cannot run — it needs PX4 telemetry that does not exist on this side — so the
  policy is fed a synthetic detector output: true severity passed through a
  noise / latency / false-positive model *fitted to M7's measured error*.
- One shared test asserting both environments expose spaces matching
  `configs/rl/observation_v1.yaml`.

**Non-negotiable:** the detector-output simulator is fitted to M7's real
measured error and the fit is documented. Inventing plausible-looking noise
instead is the quiet way to make RQ3 and RQ5 both meaningless — the policy
would be trained against a detector that does not exist, and the transfer gap
would then be measuring the modelling error rather than the simulator gap.

**Watch out for:** Isaac making principle #12 trivially easy to violate. On
this side the true severity is simply a variable in scope, so a stray reference
puts ground truth into the observation with nothing to catch it. The
observation must be assembled from the spec, never hand-packed.

---

# M9 — RL recovery policy (train in Isaac, evaluate on PX4)

**Goal:** train a high-level policy that takes the detector's estimate and
decides how to keep the mission alive. Still the core contribution — but under
D12 the *engineering* risk has moved into M8b and the *research* risk into RQ5,
so this milestone itself got smaller.

**Depends on:** M3b (sample budget), M7, M8, M8b. **Blocks:** M10.

**Do not start until `docs/isaac_feasibility.md` says the sample budget is
reachable.**

**Key deliverables:**
- Observation, action and reward frozen as versioned YAML **before training
  starts** — changing these mid-project silently invalidates every earlier run.
  Under D12 they must be frozen before *both* environments are finished, since
  two environments are now reading them.
- PPO in the `isaacsim` env, ≥3 seeds (one run proves nothing), domain
  randomisation over fault severity/timing/mass/wind/noise **and over the
  simulated detector's latency and error** — that last one is new and matters,
  because it is the most overfittable part of the training signal.
- Reward components logged separately so reward hacking is visible rather than
  indistinguishable from learning.
- Checkpoints stamped with the spec digest they were trained under; the
  evaluation side refuses a mismatch (`CLAUDE.md` §0.1).
- **The RQ5 transfer table** — each policy's performance in Isaac next to its
  performance on the PX4 stack. This is a headline result, not a diagnostic.

**Non-negotiable — protects the entire research claim:** the policy sees the
**detector's estimate**, never the true fault state. Ground truth may shape the
training reward, but must never reach the observation. Actions are high-level
only (speed/altitude/pacing/land-commit) — never a motor command. A policy that
only ties the rule-based baseline is a legitimate, reportable finding; tuning
until it wins is how projects lose their integrity.

**Second non-negotiable (D12):** **every reported number comes from the PX4
stack.** A policy that beats the baseline in Isaac and not on PX4 has produced
the RQ5 result, and that is what gets reported — not a quietly retuned run. The
only Isaac-side figure in the paper is the training curve, labelled as such.

---

# M10 — Full experiments and results

**Goal:** run the complete comparison and produce the paper's figures and
tables.

**Depends on:** M7, M8, M9. **Blocks:** M11, M13.

**The condition matrix:**

| | Condition | Fault | Detection | Recovery |
|---|---|---|---|---|
| C1 | Healthy | No | — | — |
| C2 | Faulty, no recovery | Yes | — | PX4 default only |
| C3 | Detection + rules | Yes | AI | State machine |
| C4 | Detection + RL | Yes | AI | RL policy |
| C5 | Perfect detection + RL | Yes | True state | RL policy |
| C6 | RL, detector disabled | Yes | None | RL policy |

**Key deliverables:**
- Severity sweep 0.2-1.0, ≥100 episodes/cell on held-out seeds never used in
  training; `experiments/metrics.py` is the one place any metric is computed.
- Confidence intervals across **seeds**, never pooled across episodes within a
  seed — that understates uncertainty and is the most common statistical error
  in RL papers.
- All figures generated by script, never hand-edited; the runner must resume a
  partial run without re-running or double-counting finished cells.

**Why C5/C6 matter:** C5 vs C4 separates "the detector is imperfect" from "the
policy is imperfect" — the question every reviewer asks first. Excluded/invalid
episodes are a result to report, not a nuisance to hide — if C4 restarts more
than C3, that's information about the method.

---

# M11 — Generalization tests

**Goal:** find out where the approach works and where it breaks. No
retraining — evaluation configs only.

**Depends on:** M10.

**Axes:** unseen severities (interpolation and extrapolation), unseen onset
timings, unseen wind, perturbed mass/inertia/battery, unseen missions, and the
hardest case — multiple simultaneous faults or a fault type never trained on.
Each axis's distance from the training distribution should be checked against
the M9 randomisation ranges, not eyeballed.

**Watch out for:** the instinct to hide poor generalization. A clear statement
of where the method stops working is worth more to reviewers than a table of
uniform success, which mostly reads as untested.

---

# M12 — Hexacopter extension

**Goal:** test whether the method transfers to a platform with spare rotors.

**Depends on:** M10.

PX4 has no Gazebo hexacopter model — only a simplified-physics one and a
JSBSim one — so the model and airframe file are built from scratch here; this
is real work, not a config flag. The fault plugin (already per-model,
per-rotor) should need no changes.

**Scientific note:** a hexacopter is over-actuated, so PX4 alone already
handles losing one rotor far better than a quadcopter does — our method's
advantage should *shrink*. Measuring that shrinking margin is a genuinely good
result, not a disappointment.

---

# M13 — Paper and reproducibility package

**Goal:** a submittable paper and a package someone else can actually run.

**Depends on:** M10 (M11/M12 strengthen it but don't gate it).

**Key deliverables:** the manuscript around RQ1-RQ5; `docs/reproduce.md` with
complete from-clean-machine instructions (including how to pick a worker count
on different hardware); a one-command reproduction script for the headline
result; archived models/configs/seeds/results/manifests; a tagged release.

**D12 consequence — `docs/reproduce.md` documents two separate paths.**
Reproducing the *headline result* requires only the PX4/Gazebo stack plus the
archived policy checkpoint: no Isaac, no NVIDIA account, no RTX GPU.
Reproducing the *training* requires Isaac Lab and suitable hardware. Keeping
these separate is what stops the Isaac dependency from undermining the
"clean machine reproduces the result" goal — it was a real objection under D6
and it is answered by this structure, not by ignoring it.

**Done when:** a clean machine reproduces the headline result following
`docs/reproduce.md` alone, and every number in the paper traces back to a file
in `results/`.

---

*(M5-M13 above, including M8b, are intentionally stubs — goal, key deliverables, and the
non-negotiables worth remembering, not a full task/test/prompt breakdown. That
detail gets written when each milestone actually starts, informed by whatever
M3/M4 and everything since have actually taught us by then. Writing exhaustive
specs this far ahead is the kind of premature detail this project is actively
trying to avoid — see CLAUDE.md and the note at the top of this file. M0-M4
above stay fully detailed because they're done or in progress, and that detail
is the real, load-bearing build record and verification instructions, not
speculation.)*

*(Timeline, clarified 2026-08-21: the target is roughly a week of active
**engineering** to build all of M3-M13's code — no scope was cut for this.
M9's training run and M10's evaluation sweep are separate, unattended,
wall-clock-bound jobs (hours to multiple days, per the throughput this project
already measured) — they run in the background for as long as the compute
actually needs, are not part of the week of engineering, and don't block
writing M11/M12/M13's code while they run. M13's actual paper *content* is the
one thing that is genuinely sequenced after M9/M10 finish producing real
numbers, since it can't be written before the results it describes exist.)*

---

## Deferred, out of scope for now

A web dashboard was considered as an optional parallel track (not required by
any research result). Cut to keep the project's surface area small — revisit
only if actually wanted, and design it then.

**Active fault diagnosis** (the policy performs small "probe" manoeuvres to make
a weak rotor easier to detect; calibrated belief-output detector; RQ6) was
proposed 2026-09-22 and deferred 2026-09-23 so the MVP finishes first. Full
proposal, review corrections and a cheap first check to start from:
`docs/change_active_diagnosis.md`.

*(Isaac Sim was previously listed here as declined. That is no longer true —
see D12. Isaac Lab is the training simulator as of 2026-09-21; Gazebo remains
the evaluation simulator.)*

---

## Progress log

Update this as milestones complete.

| Milestone | Status | Date | Notes |
|---|---|---|---|
| M0 | Done | 2026-08-14 | Gazebo Harmonic 8.15.0, PyTorch 2.13+cu126 (CUDA verified), PX4 v1.17.0 SITL builds clean, 236 px4_msgs interfaces. See `docs/environment.md`. **Addendum closed 2026-08-20**: pytest 9.1.1, pytest-timeout 2.4.0, pyarrow 25.0.1 installed; `environment.yml` re-exported; `pytest.ini` added to disable ROS's incompatible pytest plugins. |
| M1 | Done | 2026-08-20 | `sim_start.sh`/`sim_stop.sh` working; RTF ≈ requested up to 8×, plateaus ~8.3× (compute-bound, not a stability limit); two concurrent instances started cleanly. Numbers in `docs/simulation_notes.md`. **Superseded finding:** the "one shared Gazebo process" behaviour it documents is the default but is the wrong architecture — see M1b and `docs/parallelism.md`. |
| M1b | **Done** | 2026-08-20 | `simulation/instance_spec.py` is now the single source of instance identity (41 unit tests, 0.04 s, no simulator). `sim_start.sh` rewritten: own Gazebo server per worker via `GZ_PARTITION` + `PX4_GZ_STANDALONE`, `setsid` process groups, topic-based readiness, `instance_<N>.json` handshake. `sim_stop.sh` is PID-based per worker; the name-based sweep moved behind `--sweep`. New `sim_status.sh` and `activate.sh`. Verified with two concurrent workers holding independent speed factors, isolated shutdown, and a deliberate failure injection. Also found and fixed a fourth silent multi-instance trap: PX4's shell client only honours `--instance N` as `argv[1]` (`main.cpp:154`), so `px4-param set X 0 --instance 1` silently configures instance 0 and reports success — it had shipped in `fly_demo.py` and briefly in `sim_start.sh`, with the symptom "instance 1 arms but never takes off". Now guarded by a static scan (`tests/test_px4_cli_usage.py`) and `NAV_DLL_ACT` is read back after writing. Added `scripts/watch_worlds.sh`: N isolated worlds, one GUI window each, all drones flown concurrently. |
| M2 | **Substantially done** | 2026-08-21 | `PX4Interface` made instance-aware, fixing the two known bugs (hardcoded `target_system=1`, hardcoded `/fmu/out/...`), each with a bug-catching test. Added `PX4Clock`, `arming_sequence.py` (arm/hold/land primitives, typed exceptions), and `simulation/sim_clock.py` (`GzSimClock`). **Major correction found during implementation**: `px4_msgs` timestamps are NOT simulated time — they track wall clock regardless of speed factor (`uxrce_dds_client` resync); `GzSimClock` reads Gazebo's clock directly instead. **One item deliberately left open**: concurrent two-worker flights hit a real, confirmed `offboard_control_signal_lost` failure at ~35-65% (vs ~10-20% solo) — extensively investigated via `.ulg` log analysis and loop instrumentation; battery failsafe and `GzSimClock`'s background thread were tested and ruled out as causes; the loss occurs in the BEST_EFFORT transport, not application code. A partial mitigation (offboard re-engage on loss) roughly halves the failure rate. Full writeup and open status: `docs/parallelism.md` §2.6. 87 unit tests, all passing, 3s. |
| M3 | **Done** | 2026-08-21 | *(This row previously read "Not started", which was stale — the milestone's own checkboxes, its ✅ header and `docs/baseline_results.md` all recorded it as complete. Corrected 2026-09-21.)* 20/20 healthy missions over two runs. **Noise floor: position RMSE 6.44 ± 0.57 m** over 40 episodes. **Divergence band at fixed seed: σ = 0.083 m RMSE** following hard reset — this is the project's reproducibility tolerance (D11). Episode schema frozen and validated on every write. Two new PX4-under-Gazebo limitations found: repeated `gz set_pose` teleports permanently trip the compass consistency check, and `PREFLIGHT_REBOOT_SHUTDOWN` (medium reset) never recovers post-flight. Net effect: **soft reset measurably leaks state** (~10× hard reset's spread) and medium reset is unusable, so budget **hard reset (~19.8 s)** between episodes. Full detail: `docs/baseline_results.md`. |
| M3b | **Done** | 2026-09-21 | Isaac Sim 5.1.0 + Isaac Lab (`~/projects/IsaacLab`, commit `b0542fe2d`) confirmed headless on this GPU. Throughput sweep 64→32,768 envs: VRAM never near the 8 GB ceiling (peak 6.8 GB), **host RAM is the real constraint** (peak 13.6/15.8 GB at 32,768). Chosen operating point **8,192 envs**: 546k env-steps/s, 41% VRAM, 41% RAM — sized with headroom for M8b's future policy/optimizer/buffer overhead, not yet measured. 10-minute sustained run at that point: no memory growth (6,437.7 MB vs. 6,439.3 MB burst-test RSS), no errors. Gate (~1M steps within a few hours) clears in **under 2 seconds** of sim time — the old plan's #1 risk is resolved. One real blocker fixed: Isaac's interactive EULA prompt hangs non-interactive shells forever; fixed with `OMNI_KIT_ACCEPT_EULA=YES`, needs a permanent home in an Isaac `activate.sh` counterpart. Full detail: `docs/isaac_feasibility.md`. |
| M4 | **Done** | 2026-09-22 | **Rescoped 2026-09-21 (D12)**: a parallel *evaluation* farm, not a training farm. Gates M6 and M10; the M9 gate moved to M3b. `EpisodeRunner`, `WorkerSupervisor`, `SimFarm` built and verified against 2 real concurrent workers. Tasks 1-5 closed 2026-09-21: structured failure handling (`sim_fault`, invalid-record synthesis, restart-rate abort), the run manifest, and `SimFarm.run()`'s `on_result`/`on_restart` progress callbacks. **Task 6 (throughput sweep) ran for real**, full 16-configuration sweep; chosen operating point **2 workers** (task 8, CPU affinity, done-as-not-needed from that sweep's real CPU numbers, which never saturate). **Task 7 (soak test) passed for real 2026-09-22** — 400/400 episodes at 2 workers × 200 × 1x speed, 0 restarts, 0 orphans, flat memory, ~3h unattended — after a five-step real investigation: the milestone's literal 4-workers spec genuinely failed from real DDS/rclpy instability and was rescoped to 2 (task 6's own recommendation); a genuine bug (a reset writes no heartbeat while running, letting the health check falsely restart a worker mid-reset) was found and fixed in `EpisodeRunner.run_episode()`; task 6's own "4x speed" recommendation was found, at soak scale, to be wrong in a way the sweep's short 3-episode sample couldn't see (64% of episodes silently timed out instead of completing) and was corrected back to 1x; and `restart_budget_per_worker` was raised 5→20 in `configs/env/farm.yaml`, an evidence-based recalibration from two runs' measured restart rate, not a tuned-to-pass hack. Full story: `docs/throughput.md` and `tests/slow/test_soak.py`'s module docstring. Absorbed M3's `run_episodes.py`-internal `_Worker` into `EpisodeRunner`; extracted `simulation/worker_process.py` out of `aero_bridge/reset.py`'s `hard_reset()`. Schema bumped to v2. **Real bugs found and fixed across the milestone (7 total)**: a stale heartbeat file across runs causing a spurious restart; `is_healthy()` racing a worker's own in-flight hard reset; `ensure_healthy()` leaking an untracked OS-process pair on a failed restart; `aero_bridge/reset.py`'s `REPO_DIR` breaking depending on which of three physical copies of the file got imported; a multiprocessing-queue race double-recording one episode; the heartbeat-before-reset gap; and the soak-scale-only mission-completion-quality gap in task 6's speed-factor recommendation. Only `tests/sim/test_worker_restart.py` (a real `kill -9`, task 4) remains open, deliberately deferred. |
| M5 | **Done** | 2026-09-22 | Was M4. `configs/features.yaml` (13 shared + 6 px4_only, `feature_version` "1"), `configs/rl/observation_v1.yaml`, `configs/rl/normalization_v1.yaml`, and `ai/features/feature_extractor.py` (pure, no ROS import). Verified against a live flight as well as fixtures. Detail: M5 section above. |
| M6 | **Done** | 2026-09-23 | Was M5. `RotorDegradationSystem` plugin (scales thrust and reaction torque together), `x500_aero` model, `RotorFaultController`, fault integration in `EpisodeRunner`/`SimFarm`, thrust cross-validation fixture, episode schema v4. **Dataset `results/m6_dataset_v1/`: 750/750 episodes, 741 valid**, 2 workers × 1×, resumed twice via `--resume` with no episodes lost. Headline finding: PX4's own `FailureDetector` stays silent in 100% of faulty episodes at severity [0.2, 0.4) and 63% at [0.8, 1.0]. Full analysis: `docs/fault_dataset.md`. |
| M7 | **Done** | 2026-09-23 | Was M6. Rotor-symmetric streaming GRU, 5-member ensemble (user-chosen over a 1D-CNN). On the test flights it detects weak faults (s 0.2–0.4) in a median 0.88 s vs the random forest's 1.18 s, identifies the rotor 98.6–99.8% of the time, and estimates severity with MAE ≤ 0.022. ECE is 0.004 and uncertainty→error AUROC 0.91. Tick AUROC ties the random forest; the model loses on false alarms (6 vs 4 short events in 0.69 h). Verified live on 2 concurrent workers (right rotor, ~0.55 s delay, p99 tick ≤ 10.7 ms), after fixing a real live-latency overshoot (p99 34.5 ms → stacked-weights streaming path). `error_model.json` written for M8b. Full results: `docs/detector_results.md`. |
| M8 | **Done** | 2026-09-23 | Was M7. Physics limit: hover is impossible above rotor severity ≈ 0.41, so recovery can matter only for s ≈ 0.35–0.45. Built the 3-dimension action spec, the outcome rule (crash = tilt > 60° or touchdown > 2.0 m/s), the moving-setpoint mission tracker, policy-driven flight with ground-contact termination (episode schema v5), and the rule-based recovery controller, tuned offline then on the simulator. **Validation: the controller does not beat flying with no recovery.** It never landed a healthy drone, but its own recovery descent makes the detector over-read severity and trigger needless landings. Full results: `docs/recovery_baseline.md`. |
| M8b | Not started | | **New milestone (D12)**: Isaac Lab training environment. Depends on M3b, M5, M6, M7. |
| M9 | Not started | | Was M8. Rescoped by D12: trains in Isaac, evaluates on PX4, adds the RQ5 transfer table. |
| M10 | Not started | | Was M9. |
| M11 | Not started | | Was M10. |
| M12 | Not started | | Was M11. |
| M13 | Not started | | Was M12. |
