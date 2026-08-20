# Implementation Milestones

**Companion to `planning.md`.** That file explains *what* we are building and
*why*. This file is the build order: what to do, in what sequence, and how to
know each step actually works. `CLAUDE.md` holds the coding rules that apply to
every milestone; `docs/parallelism.md` holds the verified multi-instance facts.

Status: M0 (incl. addendum), M1 and M1b done. M2 substantially done — both
known multi-instance bugs fixed and verified; a third, subtler bug found and
fixed (px4_msgs timestamps are not simulated time, `docs/parallelism.md`
§2.5). One item deliberately left open: concurrent two-worker flights hit a
real, confirmed, unresolved `offboard_control_signal_lost` reliability gap
(§2.6) at a significant rate (~35-65%), not caused by this project's own
code; a partial mitigation shipped, full resolution deferred to M4.

**Target pace:** roughly a week of active engineering to build all of
M3-M13's code. M9's training run and M10's evaluation sweep are separate,
unattended, wall-clock-bound jobs and are expected to run longer than that in
the background — see the timeline note at the end of M13.
Last revised: 2026-08-21.

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

**M4** (parallel simulation farm), **M6** (fault injection) and **M9** (RL
training) are where this project can stall. M4 is systems work that has already
produced silent bugs; M6 needs C++; M9 needs patience and compute. Everything
else is plumbing that should go smoothly. Plan time accordingly.

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
5. **Use plan mode for M4, M6 and M9.** These have architecture decisions inside
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
| D5 | Simplified pre-training model **deferred** — revisit only if M4 shows training is impossible |
| D6 | **Gazebo Harmonic stays primary** for M1–M13; Isaac Sim was considered and declined (`planning.md` §14 D6) |
| **D7** | **One drone per world, always — settled (2026-08-20).** Every worker gets its own `GZ_PARTITION`-isolated Gazebo server. The original bug was PX4 sharing a world *silently, without anyone choosing it*; the fix is simply to never let that happen. Rationale and evidence: `docs/parallelism.md` §2.3, §3, §7. |
| **D8** | **We own the Gazebo server process** (`PX4_GZ_STANDALONE=1`), so a single worker can be stopped and restarted without touching its siblings. |
| **D9** | **Uniform instance identity, no special case for instance 0.** `PX4_UXRCE_DDS_NS=px4_<N>` for all N; `target_system = N+1` always; identity read from `instance_<N>.json`, never recomputed. |
| **D10** | **Sim time is the only clock in flight logic**, sourced from **`GzSimClock`** (`simulation/sim_clock.py`, Gazebo's native clock over gz-transport) — **not** `px4_msgs` timestamps, which were measured during M2 to track wall clock almost exactly regardless of speed factor. Wall clock is permitted solely in the watchdog. See `docs/parallelism.md` §2.5. |
| **D11** | **Reproducibility standard is statistical, not bitwise.** PX4 SITL + gz is not bitwise deterministic across runs; we fix seeds, report distributions over ≥N runs, and *measure* run-to-run divergence rather than asserting determinism. See M3 task 6. |

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
| M4 | **Parallel simulation farm + episode runner** | 1 wk | **High** | everything above M3 |
| M5 | Telemetry feature pipeline | 4 d | Low | feature contract |
| M6 | Fault injection + dataset | 1.5 wk | **High** | the farm |
| M7 | AI fault detector | 2 wk | Medium | the dataset |
| M8 | Rule-based recovery baseline | 1 wk | Low | policy interface |
| M9 | RL recovery policy | 3–4 wk | **High** | everything |
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

# M3 — Autonomous mission baseline + episode contract

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

- [ ] 20 out of 20 healthy missions complete successfully
- [ ] Position RMSE recorded for all 20; **noise floor documented**
- [ ] Run-to-run divergence at fixed seed quantified (task 7)
- [ ] All three reset tiers implemented, and each one's cost measured
- [ ] Soft reset proved equivalent to hard reset, or documented as unusable
- [ ] Every episode record validates against the schema
- [ ] Logs have no missing rows and no gaps in `t_sim`

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
# M4 — Parallel simulation farm + episode runner  ⭐ new

**Goal:** run N independent workers, each flying episodes, for hours, unattended,
with failures handled rather than avoided — and know the real aggregate
throughput number that M9's budget depends on.

**Why it matters:** this is the milestone that decides whether M9 is possible.
It is also where every bug this project has produced actually lives. Building it
now, against the M3 mission that already works, means each failure has exactly
one possible cause. Building it inside M9 means debugging parallelism, reward
shaping and PPO convergence simultaneously — which is how RL projects die.

**Depends on:** M1b, M2, M3. **Blocks:** M6 (dataset generation), M9 (training).

**Risk: High.** Budget a week and expect to spend most of it on failure handling
rather than on the happy path.

### Tasks

1. **`EpisodeRunner` — one class, one episode, one worker.**
   Takes an `InstanceSpec`, a mission config, a fault config (null for now, wired
   in M6) and a seed. Returns a validated episode record. It owns: reset →
   arm → fly → terminate → log. It does **not** own process lifecycle.
   Everything above M4 — dataset generation, evaluation, the Gym env — is a
   caller of this class. There must be exactly one.
2. **`WorkerSupervisor` — owns one worker's processes.**
   Start, health-check, stop, restart. Health checks come from
   `docs/parallelism.md` §8: PIDs alive, telemetry not stalled, DDS link up.
   Exposes `ensure_healthy()` which restarts and returns whether a restart
   happened, so the caller can invalidate the in-flight episode.
3. **`SimFarm` — owns N supervisors.**
   Deterministic worker→instance assignment. Starts all, waits for all ready
   (with a per-worker timeout, and a hard failure if any never comes up), hands
   out work, restarts failures, aggregates restart counts into the run manifest.
   Context manager: leaving the block stops everything, including on exception.
4. **Structured failure handling.**
   1. `termination_reason` enum extended with the failure modes from
      `docs/parallelism.md` §8.
   2. Episodes ended by worker failure are written with `valid=false` and a
      reason, never silently discarded.
   3. Restart budget per worker; exceeding it fails the run loudly.
   4. A run-level abort if the global restart rate exceeds a configured
      threshold — a biased dataset is worse than no dataset.
5. **Run manifest.** `results/<run_id>/manifest.json`: run id, git SHA of this
   repo, PX4 SHA, config digests, seeds, worker→instance map, versions from
   `env_report.sh`, start/end time, episode counts by outcome, restart counts.
   Written incrementally so a killed run still leaves a readable manifest.
6. **Throughput measurement — the number M9 is budgeted from.**
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
7. **Soak test.** 4 workers × 100 episodes at the chosen operating point,
   unattended, no manual intervention. Zero orphan processes at the end. Memory
   flat, not growing.
8. **CPU affinity (only if task 6 shows contention).** Pin each worker's px4 and
   gz processes to disjoint core sets with `taskset`, leaving cores for the
   learner. Measure before and after; keep it only if it actually helps.

### Files created

```
experiments/episode_runner.py
experiments/worker_supervisor.py
experiments/sim_farm.py
experiments/run_manifest.py
experiments/benchmark_throughput.py
configs/env/farm.yaml
docs/throughput.md
```

### Tests (required)

```
tests/test_sim_farm_assignment.py
tests/test_supervisor_state_machine.py
tests/test_run_manifest.py
tests/sim/test_two_workers.py       (@pytest.mark.sim)
tests/sim/test_worker_restart.py    (@pytest.mark.sim)
tests/slow/test_soak.py             (@pytest.mark.slow)
```

- `test_worker_to_instance_is_deterministic` — worker k always gets instance
  k+base, across processes and runs.
- `test_no_resource_collision_for_n_workers` — for N up to 8, no two workers
  share a port, domain, partition, namespace or model name.
- `test_farm_stops_all_on_exception` — an exception inside the context manager
  still stops every worker (use fake supervisors; no simulator).
- `test_supervisor_restart_marks_episode_invalid` — a simulated process death
  produces `valid=false` with the right `termination_reason`.
- `test_restart_budget_exhausted_raises` — exceeding the budget fails loudly.
- `test_manifest_readable_after_kill` — a partially written manifest still
  parses.
- `test_two_workers_fly_concurrently` (sim) — two workers each complete a full
  M3 mission at the same time; both records validate; the two vehicles' logs are
  distinguishable and neither contains the other's data. **This is the test that
  catches cross-wiring.**
- `test_worker_restart_recovers` (sim) — `kill -9` a worker's px4 mid-episode;
  the farm restarts it and the next episode succeeds.
- `test_soak_4x100` (slow) — 400 episodes, zero orphans, flat memory.

### Done when

- [ ] N workers run concurrently, fully isolated
      (`pgrep -cf "^gz sim "` == N)
- [ ] Killing one worker's PX4 mid-episode restarts only that worker; the others
      keep flying and their episodes remain valid
- [ ] 4 × 100 episodes complete unattended with zero orphan processes
- [ ] Peak RSS recorded and within budget; no growth across the soak
- [ ] `docs/throughput.md` states the chosen (worker count, speed factor)
      operating point and the aggregate throughput it delivers
- [ ] Every episode record carries a valid `termination_reason`; invalid
      episodes are recorded, not dropped
- [ ] The run manifest reproduces the run's configuration completely
- [ ] Interrupting the farm with Ctrl-C leaves nothing running

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

# M9 — RL recovery policy

**Goal:** train a high-level policy that takes the detector's estimate and
decides how to keep the mission alive. The core contribution, and the
milestone most likely to consume time — which is why the design stays small
and why M4 exists first.

**Depends on:** M4 (throughput budget), M7, M8. **Blocks:** M10.

**Do not start until `docs/throughput.md` says the sample budget is reachable.**

**Key deliverables:**
- Observation, action and reward frozen as versioned YAML **before training
  starts** — changing these mid-project silently invalidates every earlier run.
- The Gym environment is a thin wrapper over M4's `EpisodeRunner`; it must not
  re-implement flying, reset, or logging.
- PPO, ≥3 seeds (one run proves nothing), domain randomisation over fault
  severity/timing/mass/wind/noise, reward components logged separately so
  reward hacking is visible rather than indistinguishable from learning.

**Non-negotiable — protects the entire research claim:** the policy sees the
**detector's estimate**, never the true fault state. Ground truth may shape the
training reward, but must never reach the observation. Actions are high-level
only (speed/altitude/pacing/land-commit) — never a motor command. A policy that
only ties the rule-based baseline is a legitimate, reportable finding; tuning
until it wins is how projects lose their integrity.

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

**Key deliverables:** the manuscript around RQ1-RQ4; `docs/reproduce.md` with
complete from-clean-machine instructions (including how to pick a worker count
on different hardware); a one-command reproduction script for the headline
result; archived models/configs/seeds/results/manifests; a tagged release.

**Done when:** a clean machine reproduces the headline result following
`docs/reproduce.md` alone, and every number in the paper traces back to a file
in `results/`.

---

*(M5-M13 above are intentionally stubs — goal, key deliverables, and the
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
only if actually wanted, and design it then. Isaac Sim was considered and
declined entirely (`planning.md` §14 D6) — Gazebo is the only simulator in
this plan.

---

## Progress log

Update this as milestones complete.

| Milestone | Status | Date | Notes |
|---|---|---|---|
| M0 | Done | 2026-08-14 | Gazebo Harmonic 8.15.0, PyTorch 2.13+cu126 (CUDA verified), PX4 v1.17.0 SITL builds clean, 236 px4_msgs interfaces. See `docs/environment.md`. **Addendum closed 2026-08-20**: pytest 9.1.1, pytest-timeout 2.4.0, pyarrow 25.0.1 installed; `environment.yml` re-exported; `pytest.ini` added to disable ROS's incompatible pytest plugins. |
| M1 | Done | 2026-08-20 | `sim_start.sh`/`sim_stop.sh` working; RTF ≈ requested up to 8×, plateaus ~8.3× (compute-bound, not a stability limit); two concurrent instances started cleanly. Numbers in `docs/simulation_notes.md`. **Superseded finding:** the "one shared Gazebo process" behaviour it documents is the default but is the wrong architecture — see M1b and `docs/parallelism.md`. |
| M1b | **Done** | 2026-08-20 | `simulation/instance_spec.py` is now the single source of instance identity (41 unit tests, 0.04 s, no simulator). `sim_start.sh` rewritten: own Gazebo server per worker via `GZ_PARTITION` + `PX4_GZ_STANDALONE`, `setsid` process groups, topic-based readiness, `instance_<N>.json` handshake. `sim_stop.sh` is PID-based per worker; the name-based sweep moved behind `--sweep`. New `sim_status.sh` and `activate.sh`. Verified with two concurrent workers holding independent speed factors, isolated shutdown, and a deliberate failure injection. Also found and fixed a fourth silent multi-instance trap: PX4's shell client only honours `--instance N` as `argv[1]` (`main.cpp:154`), so `px4-param set X 0 --instance 1` silently configures instance 0 and reports success — it had shipped in `fly_demo.py` and briefly in `sim_start.sh`, with the symptom "instance 1 arms but never takes off". Now guarded by a static scan (`tests/test_px4_cli_usage.py`) and `NAV_DLL_ACT` is read back after writing. Added `scripts/watch_worlds.sh`: N isolated worlds, one GUI window each, all drones flown concurrently. |
| M2 | **Substantially done** | 2026-08-21 | `PX4Interface` made instance-aware, fixing the two known bugs (hardcoded `target_system=1`, hardcoded `/fmu/out/...`), each with a bug-catching test. Added `PX4Clock`, `arming_sequence.py` (arm/hold/land primitives, typed exceptions), and `simulation/sim_clock.py` (`GzSimClock`). **Major correction found during implementation**: `px4_msgs` timestamps are NOT simulated time — they track wall clock regardless of speed factor (`uxrce_dds_client` resync); `GzSimClock` reads Gazebo's clock directly instead. **One item deliberately left open**: concurrent two-worker flights hit a real, confirmed `offboard_control_signal_lost` failure at ~35-65% (vs ~10-20% solo) — extensively investigated via `.ulg` log analysis and loop instrumentation; battery failsafe and `GzSimClock`'s background thread were tested and ruled out as causes; the loss occurs in the BEST_EFFORT transport, not application code. A partial mitigation (offboard re-engage on loss) roughly halves the failure rate. Full writeup and open status: `docs/parallelism.md` §2.6. 87 unit tests, all passing, 3s. |
| M3 | Not started | | Now also owns the episode record schema and the reset ladder. |
| M4 | Not started | | **New milestone**: parallel simulation farm + episode runner. Gates M6 and M9. |
| M5 | Not started | | Was M4. |
| M6 | Not started | | Was M5. |
| M7 | Not started | | Was M6. |
| M8 | Not started | | Was M7. |
| M9 | Not started | | Was M8. |
| M10 | Not started | | Was M9. |
| M11 | Not started | | Was M10. |
| M12 | Not started | | Was M11. |
| M13 | Not started | | Was M12. |
