# Implementation Milestones

**Companion to `planning.md`.** That file explains *what* we are building and
*why*. This file is the build order: what to do, in what sequence, and how to
know each step actually works. `CLAUDE.md` holds the coding rules that apply to
every milestone; `docs/parallelism.md` holds the verified multi-instance facts.

Status: M0 (incl. addendum), M1, M1b and M3 done. M2 substantially done —
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

**First real result** (worth showing anyone) arrives at the end of **M7**: "our
detector spots a weakening motor that PX4 itself does not notice."
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

**Tasks 1–3 done (2026-09-21).** `EpisodeRunner`, `WorkerSupervisor` and
`SimFarm` exist and are verified against two real concurrent workers,
including `pgrep -cf "^gz sim "` reading 2 live during the run. Tasks 4
(structured failure handling), 5 (run manifest), 6 (throughput sweep), 7
(soak test) and 8 (CPU affinity, conditional) remain — see the per-task marks
below and the progress log entry for full detail, including two real bugs
found and fixed along the way: a stale heartbeat file surviving across runs
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
4. ⬜ **Structured failure handling.**
   1. `termination_reason` enum extended with the failure modes from
      `docs/parallelism.md` §8. **Partly done as part of tasks 1-3**: schema
      bumped to v2, `worker_restarted` and `offboard_lost` added (the minimum
      those tasks' own required tests needed — see the progress log). Still
      missing: `sim_fault` and any other remaining §8 rows.
   2. ⬜ Episodes ended by worker failure are written with `valid=false` and a
      reason, never silently discarded. **Known gap, called out explicitly in
      `experiments/sim_farm.py`'s module docstring**: when `ensure_healthy()`
      restarts a worker mid-episode, that episode's result is currently just
      never produced (not silently marked invalid — restart_counts makes the
      event visible — but not written as a record either). Closing this is
      this sub-task's job.
   3. ⬜ Restart budget per worker; exceeding it fails the run loudly. *(The
      per-worker ceiling itself already exists —
      `WorkerSupervisor.restart_budget` / `RestartBudgetExhausted`, driven by
      `configs/env/farm.yaml`'s `restart_budget_per_worker` — built as part of
      tasks 1-3 since the required test `test_restart_budget_exhausted_raises`
      needed it. What remains here is the run-level view across workers.)*
   4. ⬜ A run-level abort if the global restart rate exceeds a configured
      threshold — a biased dataset is worse than no dataset.
5. ⬜ **Run manifest.** `results/<run_id>/manifest.json`: run id, git SHA of this
   repo, PX4 SHA, config digests, seeds, worker→instance map, versions from
   `env_report.sh`, start/end time, episode counts by outcome, restart counts.
   Written incrementally so a killed run still leaves a readable manifest.
6. ⬜ **Throughput measurement — the number M9 is budgeted from.**
   One drone per world, always (D7 — settled, not reopened here). Sweep worker
   count ∈ {1, 2, 3, 4} × speed factor ∈ {1, 2, 4, 8}. Record per configuration:
   **aggregate simulated-seconds per wall-second** (the number that actually
   matters), episodes per wall hour, per-worker RTF mean and stdev, peak RSS,
   total CPU utilisation, and failure rate. Write to `docs/throughput.md` with
   the chosen operating point stated explicitly.

   **If the best aggregate throughput implies M9 cannot reach 1–3 M steps in
   under ~5 days, stop and revisit decision D5 before building anything else.**
   (If it also looks like this machine is memory-, not CPU-, bound, that is the
   one condition under which sharing a world between drones would be worth
   reconsidering — but that is a decision to make from real numbers if it ever
   comes up, not something to pre-build a topology system for now.)
7. ⬜ **Soak test.** 4 workers × 100 episodes at the chosen operating point,
   unattended, no manual intervention. Zero orphan processes at the end. Memory
   flat, not growing.
8. ⬜ **CPU affinity (only if task 6 shows contention).** Pin each worker's px4 and
   gz processes to disjoint core sets with `taskset`, leaving cores for the
   learner. Measure before and after; keep it only if it actually helps.

### Files created

```
experiments/episode_runner.py         ✅
experiments/worker_supervisor.py      ✅
experiments/sim_farm.py               ✅
simulation/worker_process.py          ✅ (not originally listed -- extracted
                                          from aero_bridge/reset.py's hard_reset()
                                          to avoid a second start/stop implementation;
                                          see the progress log)
experiments/run_manifest.py           ⬜ task 5
experiments/benchmark_throughput.py   ⬜ task 6
configs/env/farm.yaml                 ✅ (built ahead of task 6, since tasks
                                          1-3 already needed worker-count/
                                          restart-budget/heartbeat-timeout
                                          config-driven per CLAUDE.md principle 4)
docs/throughput.md                    ⬜ task 6
```

### Tests (required)

```
tests/test_sim_farm_assignment.py       ✅
tests/test_supervisor_state_machine.py  ✅
tests/test_episode_runner.py            ✅ (not originally listed -- covers
                                            next_reset_tier(), the one pure-
                                            function piece of EpisodeRunner's
                                            own logic; "unit-test-each-milestone")
tests/test_run_manifest.py              ⬜ task 5
tests/sim/test_two_workers.py       ✅ (@pytest.mark.sim)
tests/sim/test_worker_restart.py    ⬜ task 4 (@pytest.mark.sim)
tests/slow/test_soak.py             ⬜ task 7 (@pytest.mark.slow)
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
  a caller uses to mark an episode invalid. `SimFarm` actually building that
  invalid record for the lost in-flight episode is task 4.2's job, not yet done.)*
- ✅ `test_restart_budget_exhausted_raises` — exceeding the budget fails loudly.
- ⬜ `test_manifest_readable_after_kill` — a partially written manifest still
  parses. (task 5)
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
  the farm restarts it and the next episode succeeds. (task 4)
- ⬜ `test_soak_4x100` (slow) — 400 episodes, zero orphans, flat memory. (task 7)

### Done when

- [x] N workers run concurrently, fully isolated
      (`pgrep -cf "^gz sim "` == N) — confirmed live with N=2
- [ ] Killing one worker's PX4 mid-episode restarts only that worker; the others
      keep flying and their episodes remain valid — `ensure_healthy()`/restart
      exists and is unit-tested, but not yet proven against a real `kill -9`
      (task 4's `test_worker_restart_recovers`, sim-marked)
- [ ] 4 × 100 episodes complete unattended with zero orphan processes (task 7)
- [ ] Peak RSS recorded and within budget; no growth across the soak (task 7)
- [ ] `docs/throughput.md` states the chosen (worker count, speed factor)
      operating point and the aggregate throughput it delivers (task 6)
- [ ] Every episode record carries a valid `termination_reason`; invalid
      episodes are recorded, not dropped — schema/enum support exists (v2);
      `SimFarm` synthesizing the invalid record for a worker-restart-lost
      episode is task 4.2, not yet done
- [ ] The run manifest reproduces the run's configuration completely (task 5)
- [x] Interrupting the farm with Ctrl-C leaves nothing running —
      `SimFarm.__exit__` stops every worker unconditionally, including on
      exception, verified by `test_farm_stops_all_on_exception` and
      `test_exit_attempts_every_stop_even_if_one_fails`

### Verify with

```bash
pytest tests/ -m "not sim and not slow" -q
pytest tests/sim/ -q
python experiments/benchmark_throughput.py --workers 1,2,3,4 --speeds 1,2,4,8
pytest tests/slow/test_soak.py -q          # ~1-2 h, run it overnight
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
# M5 — Telemetry feature pipeline

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

---

# M6 — Fault injection and dataset

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

---

# M7 — AI fault detector

**Goal:** a model that reads a telemetry window and reports fault presence,
type, severity, and confidence. The first genuinely novel result.

**Depends on:** M5, M6. **Blocks:** M8, M9.

**Key deliverables:**
- Split the dataset **by episode, never by timestep** — this is the classic
  data-leakage mistake in this kind of work (overlapping windows from one
  flight landing in both train and test), and it's worth a dedicated test that
  makes the wrong split structurally inexpressible, not just discouraged.
- Baselines (threshold-on-residual, random forest) trained first, so the bar is
  honest and dataset problems surface while the model is still simple.
- Main model: a small 1D-CNN or GRU — not a Transformer, not at this data
  scale.
- Outputs include an uncertainty estimate, not just a point prediction — the
  recovery policy needs to know when to distrust the detector.
- Report accuracy **per severity level**, plus detection-delay distribution. A
  single averaged accuracy number hides everything that actually matters here.

---

# M8 — Rule-based recovery baseline

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
| M4 | **Tasks 1-3 done** | 2026-09-21 | **Rescoped 2026-09-21 (D12)**: now a parallel *evaluation* farm, not a training farm. Gates M6 and M10; the M9 gate moved to M3b. `EpisodeRunner`, `WorkerSupervisor`, `SimFarm` built and verified against 2 real concurrent workers (`pgrep -cf "^gz sim "` == 2 confirmed live; zero orphans after; 136 no-sim unit tests pass). Absorbed M3's `run_episodes.py`-internal `_Worker` into `EpisodeRunner`; extracted `simulation/worker_process.py` out of `aero_bridge/reset.py`'s `hard_reset()` so starting/stopping a worker's OS processes has exactly one implementation. Schema bumped to v2 (`worker_restarted`, `offboard_lost` added) and `arming_sequence.py` gained a distinct `OffboardLost` exception, separating "hold timed out while genuinely stuck" from "hold timed out because PX4 never came back from an offboard drop" -- the latter being the M2-documented concurrent-worker DDS gap (`docs/parallelism.md` §2.6), which the verification run's two workers both hit (`termination_reason=episode_timeout`, not a new bug -- see the test's own note). **One real bug found and fixed**: a heartbeat file surviving in the shared `run_dir` across runs made a perfectly healthy, still-flying worker look stale to `ensure_healthy()`, triggering a spurious restart and a duplicated episode (3 results for 2 workers) on first attempt; fixed by clearing a worker's heartbeat file at the start of `WorkerSupervisor.start()`. Tasks 4 (full structured-failure handling), 5 (run manifest), 6 (throughput sweep), 7 (soak test) and 8 (CPU affinity) remain -- see the milestone's own task list for exact status. |
| M5 | Not started | | Was M4. |
| M6 | Not started | | Was M5. |
| M7 | Not started | | Was M6. |
| M8 | Not started | | Was M7. |
| M8b | Not started | | **New milestone (D12)**: Isaac Lab training environment. Depends on M3b, M5, M6, M7. |
| M9 | Not started | | Was M8. Rescoped by D12: trains in Isaac, evaluates on PX4, adds the RQ5 transfer table. |
| M10 | Not started | | Was M9. |
| M11 | Not started | | Was M10. |
| M12 | Not started | | Was M11. |
| M13 | Not started | | Was M12. |
