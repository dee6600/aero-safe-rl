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
| D6 | **Gazebo Harmonic stays primary** for M1–M13; Isaac Sim is an optional, non-blocking learning track |
| **D7** | **One drone per world — settled for the build (2026-08-20).** Workers are grouped into `GZ_PARTITION`-isolated worlds with `drones_per_world` as a config parameter, **fixed at 1** for M4–M13. The parameter exists so the shared and hybrid topologies can be *measured* in M4's benchmark, not so they can be built: no milestone depends on `drones_per_world > 1`. The original bug was PX4 sharing a world *silently, without anyone choosing it* — that stays prohibited regardless. Rationale and evidence: `docs/parallelism.md` §2.3, §3, §7. |
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
tests/sim/test_link.py          (@pytest.mark.sim)
tests/sim/test_sim_clock.py     (@pytest.mark.sim)
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

### Done when

All verified 2026-08-20/21 (see `docs/parallelism.md` §2.6 for the one open item):

- [x] All nine telemetry topics carry believable, changing values — asserted by
      `tests/test_px4_interface.py` (topic naming/target_system, both instance 0
      and 3) and flown live on instance 1
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
6. **Make world topology a parameter, not an assumption.**
   The farm config gains `worlds` and `drones_per_world`; total workers is their
   product. `drones_per_world = 1` (fully isolated) and `worlds = 1` (fully
   shared) are the two ends of the same single code path — **not two code
   paths.** The instance spec gains `world_index` and `slot_index`; the launcher
   starts one Gazebo server per *world* rather than per *drone*, and drones
   within a world share its partition, clock and speed factor.
   Building this as one parameterised design costs almost nothing now. Adding it
   later means touching the launcher, the spec, the supervisor and the farm at
   once — the exact rework this milestone exists to prevent.
7. **Support shared worlds — benchmark only, deferred by default.**
   `drones_per_world` stays at 1 for the build (D7). Do only what the benchmark
   arm needs, and stop there. Items 1 and 3–4 below are **deferred unless task 8
   shows the hybrid winning**; item 2 is the cheap path that makes the arm
   measurable at all.
   1. *(Deferred)* **Disable drone–drone collision.** `collide_bitmask` is supported by
      sdformat14 and the dartsim plugin ships a `BitmaskContactFilter`
      (verified 2026-08-20). Note the gotcha: masks collide when
      `maskA & maskB != 0`, so a *shared* drone bitmask still self-collides.
      Each drone in a world needs a **distinct bit** (`1 << (slot+1)`), with the
      ground left at `0xFFFF` so every drone still lands on it. That requires
      per-slot SDF templating at spawn time.
   2. **Use spatial separation** — spawn slots ≥ 200 m apart, far beyond the
      20–40 m mission envelope. Simpler and certain, needs no SDF templating,
      and is enough to make the benchmark arm meaningful. **This is the only
      collision work to do now.**
   3. *(Deferred)* Verify PX4 sets each vehicle's EKF origin at its own spawn point, so
      mission waypoints stay in local coordinates and do not need per-slot
      offsetting. If they do need offsetting, that is a per-slot special case
      and must go in the spec, not in mission code.
   4. *(Deferred)* Confirm the motor model applies no aerodynamic coupling between vehicles
      (no downwash interaction). If it does, shared worlds are scientifically
      unusable and the comparison arm is dropped.
8. **Throughput measurement — the number M9 is budgeted from.**
   Sweep the **topology grid**, not just worker count:

   | Topology | Workers | Physics threads | What it tests |
   |---|---|---|---|
   | 1 world × 1 drone | 1 | 1 | Baseline; matches M1's ~8.3× ceiling |
   | 4 worlds × 1 drone | 4 | 4 | Full isolation (current default) |
   | 2 worlds × 2 drones | 4 | 2 | **Hybrid** |
   | 1 world × 4 drones | 4 | 1 | Full sharing |
   | 2 worlds × 3 drones | 6 | 2 | Hybrid, RAM-favourable |

   × speed factor ∈ {1, 2, 4, 8}. Record per configuration: **aggregate
   simulated-seconds per wall-second** (the number that actually matters),
   episodes per wall hour, per-worker RTF mean and stdev, peak RSS, total CPU
   utilisation, and failure rate. Write to `docs/throughput.md` with the chosen
   operating point stated explicitly and the runner-up noted.

   **Expected result, stated in advance so the measurement can falsify it:**
   isolated worlds should win, because gz-sim steps one world on one thread and
   M1 already showed a single drone nearly saturates that thread at 8×. Packing
   a second drone into a world should roughly halve that world's achievable
   speed factor, leaving aggregate throughput similar but isolation worse. The
   hybrid's real advantage should be **RAM and process count**, so it matters
   only if the machine turns out to be memory-bound before it is core-bound.
   If the data contradicts this, D7's default changes — that is the point of
   measuring.

   **If the best aggregate throughput implies M9 cannot reach 1–3 M steps in
   under ~5 days, stop and revisit decision D5 before building anything else.**
9. **Soak test.** 4 workers × 100 episodes at the chosen operating point,
   unattended, no manual intervention. Zero orphan processes at the end. Memory
   flat, not growing.
10. **CPU affinity (only if task 8 shows contention).** Pin each worker's px4 and
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
configs/env/topologies.yaml        (the benchmark grid)
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
- `test_topology_grouping` — for every `(worlds, drones_per_world)` in the
  benchmark grid, worker count equals the product, each world has exactly
  `drones_per_world` members, and workers in the same world share a partition
  while workers in different worlds never do.
- `test_shared_world_slots_are_separated` — within a world, spawn poses are at
  least the configured minimum distance apart, and (if the bitmask path is
  built) every slot has a distinct collision bit while the ground keeps `0xFFFF`.
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

- [ ] N workers run concurrently in the configured topology
      (`pgrep -cf "^gz sim "` == number of **worlds**, not workers)
- [ ] `drones_per_world = 1` and `drones_per_world > 1` run through the **same
      code path**, selected by config alone
- [ ] Killing one worker's PX4 mid-episode restarts only that worker; the others
      keep flying and their episodes remain valid
- [ ] 4 × 100 episodes complete unattended with zero orphan processes
- [ ] Peak RSS recorded and within budget; no growth across the soak
- [ ] `docs/throughput.md` states the chosen **(worlds × drones-per-world,
      speed factor)** operating point, the aggregate throughput it delivers, and
      the runner-up
- [ ] The isolated-vs-hybrid-vs-shared comparison is a measured table, not an
      argument — including the case where the measurement contradicts the
      prediction written in task 8
- [ ] Every episode record carries a valid `termination_reason`; invalid
      episodes are recorded, not dropped
- [ ] The run manifest reproduces the run's configuration completely
- [ ] Interrupting the farm with Ctrl-C leaves nothing running

### Verify with

```bash
pytest tests/ -m "not sim and not slow" -q
pytest tests/sim/ -q
python experiments/benchmark_throughput.py --grid configs/env/topologies.yaml \
  --speeds 1,2,4,8            # sweeps isolated / hybrid / shared topologies
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
- **Shared worlds couple more than they look like they do.** PX4 SITL runs in
  lockstep (`boards/px4/sitl/sitl.cmake:12`) and takes its entire clock from the
  world's `/clock` topic (`GZBridge.cpp:345`). Every drone in a world therefore
  shares one heartbeat — if one flight stack stalls, the whole world waits and
  every drone in it stalls too. The speed factor is also world-level. Neither
  shows up in a short test; both show up in a multi-day run.
- **A shared world is a shared crash domain.** One `gz sim` segfault loses every
  in-flight episode in that world, not one. That is the cost the hybrid trades
  against its RAM saving.
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
  - World topology is ONE parameterised design, not two code paths. The farm
    config has `worlds` and `drones_per_world`; total workers is their product.
    drones_per_world=1 and worlds=1 are just two points in the same design.
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

*(was M4)*

**Goal:** turn raw telemetry into one fixed-size vector, produced 10 times per
second, used identically by the detector, the RL policy, and later the dashboard.

**Why it matters:** if the detector and the policy each compute features their
own way, they drift apart and the M10 comparison becomes invalid. One
implementation, used everywhere.

**Depends on:** M3 (episode records to test against). **Blocks:** M6, M7, M9.

### Tasks

1. **`FeatureExtractor`**: sliding window of telemetry (start 1–2 s) → fixed
   length vector at 10 Hz. Pure function of the window; no ROS, no I/O, no
   global state. That is what makes it testable without a simulator.
2. **Feature set, at minimum:**
   - attitude, angular rates, estimated angular acceleration
   - position and velocity error against the current setpoint
   - each motor's normalised output
   - **commanded thrust vs achieved acceleration (the residual)**
   - control-allocation residual
   - EKF innovation values
   - battery current, vibration metrics
3. **Handle missing and late messages explicitly.** Topics arrive at different
   rates and occasionally drop. Decide per feature: hold-last, interpolate, or
   emit a validity mask — and put the choice in `features.yaml`. Silent
   zero-filling is how a detector learns to detect dropouts instead of faults.
4. **Compute normalisation statistics from healthy flights and freeze them** to
   `ai/features/normalization.json`. Never recompute after training starts.
5. **`configs/features.yaml`** with an explicit `feature_version` string and the
   ordered feature-name list. The names are part of the contract — a reordering
   is a version bump.
6. **Offline replay path**: extract features from a stored episode record with no
   simulator. Everything downstream develops against this, which makes M7
   iteration fast.

### Files created

```
ai/features/extractor.py
ai/features/normalization.json         (frozen statistics)
ai/features/replay.py
configs/features.yaml
tests/fixtures/healthy_window.npz      (recorded, committed)
tests/fixtures/healthy_episode.parquet (recorded, committed)
```

### Tests (required)

```
tests/test_features.py
tests/test_feature_replay.py
```

- `test_output_length_matches_config` — vector length equals the declared feature
  list length; a mismatch is caught here, not in training.
- `test_no_nans_on_healthy_fixture` — the committed healthy window produces a
  finite vector.
- `test_deterministic_on_fixture` — same input, same output, bit for bit.
- `test_window_is_causal` — feeding a window with future samples zeroed changes
  nothing. **This is the test that catches lookahead leakage**, which is
  otherwise invisible and inflates every later result.
- `test_missing_message_handling` — dropping one topic for 200 ms produces the
  documented behaviour (mask set / hold-last), not silent zeros.
- `test_normalisation_is_frozen` — the extractor loads stats from disk and does
  not recompute them.
- `test_feature_version_in_output` — every emitted record carries the version.
- `test_thrust_residual_sign` — with commanded thrust above achieved
  acceleration, the residual has the expected sign. The single most important
  feature deserves its own test.

### Done when

- [ ] Feature vector logged across a full healthy mission with no gaps, no NaNs
- [ ] Replaying the same recorded episode produces **exactly** the same vectors
- [ ] Normalisation statistics computed from healthy data and frozen to disk
- [ ] `feature_version` recorded in every log file
- [ ] Causality test passes
- [ ] Feature extraction runs comfortably inside the 10 Hz budget with margin

### Verify with

```bash
pytest tests/test_features.py tests/test_feature_replay.py -q
python -m ai.features.replay results/<run_id>/worker_0/episode_0001.parquet \
  --check-determinism --repeat 3
```

### Watch out for

- **The thrust-versus-acceleration residual is probably the most important
  feature in the project.** A weakening motor forces PX4 to command more thrust
  while the aircraft accelerates less. Make sure the sign and the units are
  right, and test it.
- Never recompute normalisation statistics after training starts. Doing so
  silently invalidates every trained model.
- Anything using a *future* value inside the window is a bug — the real drone
  cannot see the future. Windows look backwards only, and a test enforces it.
- Feature extraction must not import ROS. Keeping it pure is what lets M7 iterate
  offline in seconds instead of minutes.

### Claude Code prompt

```
Read CLAUDE.md and milestones.md M5.

Implement M5 tasks 1, 2 and 6: FeatureExtractor, the feature set, and the
offline replay path.

Constraints:
  - FeatureExtractor is a pure function of a telemetry window. No ROS imports,
    no file I/O at call time, no global state.
  - Windows are strictly causal.
  - The ordered feature-name list and feature_version live in
    configs/features.yaml; the code reads them and never hardcodes an order.
  - Missing-message policy is explicit per feature and declared in the config.

Deliverables:
  - ai/features/extractor.py, ai/features/replay.py, configs/features.yaml
  - tests/test_features.py with the eight tests listed in M5, using a committed
    fixture in tests/fixtures/ rather than a live simulator

Verify: pytest tests/test_features.py -q

Do not: recompute normalisation statistics at call time; do not use any sample
later than the window's end; do not zero-fill missing data silently.
```

---

# M6 — Fault injection and dataset

*(was M5)*

**Goal:** inject a rotor fault of any chosen strength, at any chosen moment,
repeatably — and produce the labelled dataset the detector learns from.

**Why it matters:** this is the foundation of the research. If faults are not
controllable and repeatable, nothing after this point is science.

**This is the hardest engineering milestone. Budget accordingly.**

**Depends on:** M4 (the farm generates the dataset), M5 (features validate the
fault is visible). **Blocks:** M7.

### Tasks

1. **Write the Gazebo plugin `RotorDegradationSystem`** (C++, ~250–350 lines).
   1. Use PX4's `MotorFailureSystem`
      (`src/modules/simulation/gz_plugins/motor_failure/` on `main`, 344 lines)
      as a structural reference — it shows the correct pattern.
   2. Holds one efficiency value per rotor, 0 (dead) to 1 (healthy).
   3. Listens on a gz-transport topic for updates, so severity can change
      mid-flight.
   4. Multiplies that rotor's thrust and torque contribution each physics step.
   5. Lives in **our** repo, loaded via `GZ_SIM_SYSTEM_PLUGIN_PATH`.
   6. **The command topic is per-model, not global** —
      `/model/<model_name>/rotor_efficiency`. With one world per worker this is
      belt-and-braces, but a global topic would make the plugin unusable the
      moment two vehicles share a world, and M12's hexacopter work may do exactly
      that.
   7. Echo the applied efficiency back on a status topic, so the Python side can
      *confirm* a fault took effect rather than assuming the message arrived.
2. **Copy the x500 model into `simulation/models/x500_aero/`** and add our plugin
   to the SDF. **Do not edit anything inside `~/projects/PX4-Autopilot`.**
   `GZ_SIM_RESOURCE_PATH` was already wired to find it in M1b.
3. **Python control interface**: `inject_fault(rotor, severity, profile,
   onset_time)` with step, ramp and intermittent profiles. Onset time is in
   **sim** time. Verify application via the status topic from task 1.7.
4. **Fault configuration files**, severity sampled from a **seeded** generator
   passed explicitly (never `np.random` global state).
5. **Ground-truth fault labels written into every episode record**, next to the
   features, using the M3 schema. The label includes the *commanded* and the
   *confirmed applied* severity — if they ever differ, the dataset is wrong and
   we need to know.
6. **Confirm PX4's `FailureDetector` stays silent** at target severities. Assert
   it in the dataset generator, per episode, and record the result in the
   episode summary. If PX4 notices the fault, that episode is outside the
   research question and must be flagged, not quietly included.
7. **Generate the dataset**: 500–1000 episodes, healthy and faulty, balanced
   across severity, onset time and rotor index. Uses the M4 farm. Expect this to
   take hours — that is what M4 was for.

### Suggested build order

Get the loop closed before making it precise:

- **Step A:** binary failure only (efficiency 0 or 1), one rotor, single
  instance. Proves the plugin loads, receives messages, and affects flight.
- **Step B:** graded severity, all rotors, ramps and intermittent profiles.
- **Step C:** run it under the farm and generate the dataset.

### Files created

```
simulation/gz_plugins/rotor_degradation/     (C++ source + CMakeLists.txt)
simulation/models/x500_aero/model.sdf
simulation/faults/injector.py
simulation/faults/schedule.py
configs/faults/rotor_degradation.yaml
experiments/generate_dataset.py
docs/fault_injection.md
```

### Tests (required)

```
tests/test_fault_schedule.py
tests/test_fault_config.py
tests/sim/test_plugin_loads.py        (@pytest.mark.sim)
tests/sim/test_fault_visible.py       (@pytest.mark.sim)
tests/slow/test_dataset_balance.py    (@pytest.mark.slow)
```

- `test_step_profile` / `test_ramp_profile` / `test_intermittent_profile` — the
  efficiency-vs-sim-time curve is exactly what the config asks for. Pure
  function, no simulator.
- `test_severity_sampling_is_seeded` — same seed, same sequence of faults;
  different seeds, different sequences.
- `test_onset_time_is_sim_time` — the schedule is unaffected by speed factor.
- `test_fault_config_validates` — severity in range, rotor index valid, onset
  before episode end.
- `test_plugin_loads` (sim) — the plugin appears in the gz server's system list
  and answers on its status topic.
- `test_commanded_equals_applied` (sim) — commanded severity is echoed back
  within tolerance.
- `test_fault_visible_in_features` (sim) — 40% loss on rotor 2 at t=20 s
  produces a feature-space deviation exceeding the M3 noise floor by a stated
  margin.
- `test_px4_failure_detector_silent` (sim) — at target severities, PX4's own
  detector does not trigger.
- `test_dataset_balance` (slow) — the generated corpus is balanced across
  severity bins, onset times and rotor indices, and label distribution matches
  the config.

### Done when

- [ ] Plugin loads without errors and responds to its gz topic
- [ ] Commanded severity is confirmed applied, not assumed
- [ ] 40% loss on rotor 2 at t = 20 s produces a **clear, repeatable signature**
      in the M5 features, exceeding the M3 noise floor
- [ ] Fault timing is in sim time and unaffected by speed factor
- [ ] Same seed produces the same *fault schedule*; trajectory divergence is
      within the band measured in M3 task 7 (D11 — not bitwise)
- [ ] **PX4's own `FailureDetector` stays silent** at target severities, asserted
      per episode
- [ ] Dataset of 500+ labelled episodes, balanced across severities
- [ ] `~/projects/PX4-Autopilot` has **zero** uncommitted modifications

### Verify with

```bash
pytest tests/test_fault_schedule.py -q
cmake --build simulation/gz_plugins/rotor_degradation/build
pytest tests/sim/ -q
python experiments/generate_dataset.py --config configs/faults/rotor_degradation.yaml \
  --episodes 500 --workers 4
pytest tests/slow/test_dataset_balance.py -q
cd ~/projects/PX4-Autopilot && git status --porcelain   # must be empty
```

### Watch out for

- **Plugin ordering in the SDF matters.** Ours must be declared *after* the motor
  model plugin, or the motor model overwrites our changes each step. PX4's plugin
  README states this explicitly.
- **Confirm, do not assume.** A gz-transport publish is fire-and-forget. Without
  the status echo you can generate 500 episodes labelled "40% fault" in which no
  fault was ever applied, and the detector will learn nothing while looking like
  it is underfitting.
- If PX4's `FailureDetector` *does* trigger, the severities are too high. Lower
  them. The interesting research zone is faults PX4 cannot see.
- Do not use PX4's `failure motor N off` command for the main experiments. It is
  binary, and it is internal to PX4, which makes the fault partly
  self-announcing.
- Keep every fault parameter in YAML. This dataset will be regenerated more than
  once and you must be able to say exactly how.
- Dataset generation is the first long unattended run. If M4's soak test was
  skipped, this is where you discover it.

### Claude Code prompt (use plan mode for task 1)

```
Read CLAUDE.md, milestones.md M6, and PX4's
src/modules/simulation/gz_plugins/motor_failure/ on the main branch as a
structural reference (read only — do not modify the PX4 tree).

Plan, then implement, M6 task 1 step A: a gz-sim system plugin
RotorDegradationSystem that applies a per-rotor efficiency factor, starting with
binary on/off only.

Constraints:
  - Lives in simulation/gz_plugins/rotor_degradation/ in THIS repo. The PX4 tree
    is not touched.
  - Command topic is per-model: /model/<model_name>/rotor_efficiency.
  - The plugin echoes the applied efficiency on a status topic so Python can
    confirm application rather than assume it.
  - The plugin element must appear after the motor model plugin in the SDF.

Deliverables:
  - simulation/gz_plugins/rotor_degradation/ (source + CMakeLists.txt)
  - simulation/models/x500_aero/model.sdf (our copy, plugin added)
  - tests/sim/test_plugin_loads.py asserting the plugin loads and echoes status

Verify: build the plugin, start one instance with GZ_SIM_SYSTEM_PLUGIN_PATH
pointing at the build output, publish an efficiency of 0 to rotor 2, and show
from the telemetry that the vehicle's behaviour changed. Then confirm
`cd ~/projects/PX4-Autopilot && git status --porcelain` is empty.

Do not: use a global gz topic; do not modify the PX4 tree; do not add graded
severity yet.
```

---
# M7 — AI fault detector

*(was M6)*

**Goal:** a model that reads the telemetry window and reports whether a fault is
present, which one, how severe, and how confident it is.

**Why it matters:** this is the first genuinely novel result. Target the region
where PX4 is blind.

**Depends on:** M5, M6. **Blocks:** M8, M9.

### Tasks

1. **Split the dataset by episode, never by time step.** Write the splitter so
   that splitting by timestep is not expressible, and assert the property in a
   test rather than trusting the caller.
2. **Train the comparison baselines first** — a threshold on the residual and a
   random forest on the same features. Doing these first sets the bar honestly
   and catches dataset problems while the model is still simple enough to debug.
3. **Train the main model** — start with a **1D CNN or a small GRU**. A
   Transformer is not justified at this data size.
4. **Model outputs**: `p(fault)`, fault class, severity estimate, and an
   **uncertainty measure** — the recovery policy needs to know when to distrust
   it. Decide the uncertainty mechanism explicitly (ensemble, MC-dropout, or a
   predicted variance head) and record the choice.
5. **Wrap it in a ROS 2 node running live at 10 Hz**, sharing `ai/features/` with
   everything else. The node takes an `InstanceSpec` like every other node.
6. **Evaluation report**: accuracy per severity, ROC curves, and the **detection
   delay** distribution.
7. **Freeze the detector artifact**: checkpoint plus the exact `feature_version`,
   normalisation file digest and config it was trained under. M9 and M10 must be
   able to prove which detector produced which result.

### Files created

```
ai/models/detector.py
ai/train.py    ai/evaluate.py
ai/baselines/threshold.py    ai/baselines/random_forest.py
ai/dataset.py                (split, load, batch)
ros2_ws/src/aero_bridge/aero_bridge/detector_node.py
configs/detector/cnn_v1.yaml
results/detector/report.md
```

### Tests (required)

```
tests/test_dataset_split.py
tests/test_detector_io.py
tests/test_detector_inference_budget.py
tests/sim/test_detector_node.py    (@pytest.mark.sim)
```

- `test_split_is_by_episode` — no episode id appears in more than one split.
  **The single most important test in this milestone.**
- `test_split_is_stratified` — severity distribution is preserved across splits.
- `test_split_is_reproducible` — same seed, same split.
- `test_no_window_crosses_episode_boundary` — windows are built within an
  episode, never across two.
- `test_model_output_shapes` — the four outputs have the declared shapes and
  ranges (`p(fault)` in [0,1], severity in [0,1]).
- `test_normalisation_matches_training` — the node refuses to run if the
  normalisation digest differs from the one the checkpoint was trained with.
- `test_inference_under_budget` — a single forward pass is < 20 ms on this
  machine.
- `test_detector_node_rate` (sim) — the live node sustains 10 Hz.

### Done when

- [ ] Splits are by episode — verified by test, not assumed
- [ ] Main model beats **both** baselines on held-out episodes
- [ ] Accuracy reported **per severity level** (a single average hides
      everything interesting)
- [ ] Detection delay measured: onset → first sustained alarm (mean, median, p95)
- [ ] False alarm rate on healthy flights, per minute of flight
- [ ] Uncertainty output is calibrated well enough to be useful, and its
      calibration is reported
- [ ] Live inference under 20 ms per step
- [ ] Detector artifact frozen with its feature/normalisation digests

### Verify with

```bash
pytest tests/test_dataset_split.py tests/test_detector_io.py -q
python ai/train.py --config configs/detector/cnn_v1.yaml --seed 0
python ai/evaluate.py --checkpoint ai/checkpoints/cnn_v1_seed0.pt --report
pytest tests/sim/test_detector_node.py -q
```

### Watch out for

- **Splitting by time step instead of episode leaks data catastrophically.**
  Overlapping windows from the same flight land in both train and test, accuracy
  looks superb, and it means nothing. This is the classic mistake in this kind of
  work, and it is why `test_split_is_by_episode` exists.
- Report per severity. Strong faults are easy; weak ones are the point.
- A model that is 99% accurate but takes 4 seconds to notice may be useless in
  flight. Delay matters as much as accuracy.
- Class balance: if healthy episodes dominate, accuracy is meaningless. Report
  precision/recall and use a balanced or weighted objective.
- The detector must never see ground-truth fault state as an input, at train or
  test time. Only labels, only in the loss.

---

# M8 — Rule-based recovery baseline

*(was M7)*

**Goal:** a sensible, well-tuned, non-learning recovery system.

**Why it matters:** this is what the RL policy must beat. A weak baseline that
RL then "beats" is worthless, and reviewers see through it immediately. Build
this one honestly.

**Depends on:** M7. **Blocks:** M9 (shares the policy interface), M10.

### Tasks

1. **Define the shared policy interface first** (`rl/policies/base_policy.py`).
   Both the FSM and the RL policy implement it. The interface fixes what a policy
   can see (the detector's estimate, vehicle state, mission context) and what it
   can command (the M9 action space). Building this first is what makes the C3
   vs C4 comparison fair — the RL policy must not be able to quietly acquire
   powers the FSM lacks.
2. **State machine**: `NORMAL → SUSPECTED → CONFIRMED → RECOVERING →
   LANDED/ABORTED`.
3. **Responses**: cap horizontal speed, cap climb rate, lower the altitude
   ceiling, fly less aggressively, hold position, divert to a safe point, descend
   under control.
4. **Hysteresis and debouncing** so a flickering detector does not cause
   thrashing.
5. **Tune the thresholds with a documented sweep**, run on the M4 farm. Save the
   sweep results as evidence.
6. **Verify behaviour on false alarms** — the FSM must not panic-land a healthy
   drone.

### Files created

```
rl/policies/base_policy.py        (shared interface — RL uses this too)
rl/policies/rule_based.py
configs/recovery/rule_based.yaml
experiments/tune_rule_based.py
results/recovery/tuning_sweep.md
```

### Tests (required)

```
tests/test_base_policy_interface.py
tests/test_rule_based_fsm.py
```

- `test_fsm_transitions` — every legal transition fires on its condition and no
  illegal transition is reachable. Table-driven, no simulator.
- `test_hysteresis_prevents_thrash` — an alternating detector signal at the
  threshold produces at most one transition per debounce window.
- `test_false_alarm_does_not_land` — a brief spurious detection on an otherwise
  healthy flight does not reach `LANDED`.
- `test_action_within_bounds` — every action the FSM emits is inside the declared
  action space. **This is what keeps C3 and C4 comparable.**
- `test_policy_interface_conformance` — the FSM and a stub RL policy both satisfy
  the same interface, checked by the same test.
- `test_config_sweep_is_reproducible` — a seeded sweep produces the same grid.

### Done when

- [ ] Clearly beats the no-recovery condition across the severity range
- [ ] Tuning sweep saved as evidence that it was given a fair chance
- [ ] Uses the identical command interface the RL policy will use
- [ ] Behaves sensibly on false alarms
- [ ] Every FSM parameter lives in YAML, none in code

### Watch out for

- Resist the temptation to under-tune this so RL looks better. A strong baseline
  makes an RL win *credible*; a weak one makes it worthless.
- The shared interface is what makes the M10 comparison fair. Build it here, and
  do not let the RL policy quietly gain extra powers later.
- Tune against training-fold seeds only. Tuning on the held-out evaluation seeds
  is the same leakage mistake as splitting by timestep, one level up.

---

# M9 — RL recovery policy

*(was M8)*

**Goal:** train a policy that takes the detector's estimate and chooses
high-level actions that keep the mission alive.

**Why it matters:** the core contribution, and the milestone most likely to
consume time — which is why the design is deliberately small and why M4 exists.

**Depends on:** M4 (throughput), M7 (detector), M8 (interface). **Blocks:** M10.

**Do not start this milestone until `docs/throughput.md` says the sample budget
is reachable.**

### Tasks

1. **Freeze the observation, action and reward specification before training
   starts.** Write them to `configs/rl/observation_v1.yaml`,
   `action_v1.yaml`, `reward_v1.yaml` with version strings, and add a test that
   the environment's spaces match the files. Full specification in
   `planning.md` §7.2. Changing these mid-project silently invalidates every
   earlier run; the version string is what makes that visible.
2. **Build the Gymnasium environment** wrapping the stack at **5 Hz**. It is a
   thin wrapper over M4's `EpisodeRunner` — it must not re-implement flying,
   reset, or logging.
3. **Environment contract tests before any training.**
   `gymnasium.utils.env_checker`, plus a random-action smoke test of 1000 steps
   across a worker restart.
4. **Wire the farm into `SubprocVecEnv`.** Start method `spawn`, explicitly.
   `rclpy.init()` only inside workers. Worker→instance assignment from M4.
5. **Train with PPO.** Domain randomisation over fault severity, timing, mass,
   wind and sensor noise. Randomisation ranges live in config and are recorded
   in the run manifest.
6. **Train at least 3 seeds.** A single run proves nothing.
7. **Log to TensorBoard, checkpoint often**, and make training resumable — a
   multi-day run will be interrupted.
8. **Watch for reward hacking explicitly.** Log the components of the reward
   separately, not just the total. A policy that maximises reward by exploiting a
   shaping term looks identical to a policy that learned the task, unless you can
   see the breakdown.

### Files created

```
rl/envs/uav_fault_env.py
rl/rewards/mission_reward.py
rl/policies/rl_policy.py
rl/train.py    rl/evaluate.py
configs/rl/ppo_v1.yaml
configs/rl/observation_v1.yaml  action_v1.yaml  reward_v1.yaml
configs/env/train_env.yaml
```

### Key design points (repeated because they are easy to lose)

- The policy sees the **detector's estimate**, never the true fault state. True
  state may shape the reward during training, but must never be an input.
  Violating this destroys hardware transferability and any claim about RQ3.
- Actions are high-level only: speed limits, altitude offset, mission pacing, and
  a decision to commit to landing. **Never motor commands.**
- Give **partial credit for a safe landing**. Without it, the policy learns to
  gamble on completing the mission instead of protecting the aircraft.
- Rough target: **1–3 M environment steps**. Budget from `docs/throughput.md`,
  not from an assumption.

### Tests (required)

```
tests/test_obs_action_spec.py
tests/test_reward_function.py
tests/test_env_contract.py
tests/sim/test_env_smoke.py        (@pytest.mark.sim)
tests/slow/test_vecenv_4.py        (@pytest.mark.slow)
```

- `test_spaces_match_config` — observation and action spaces equal the frozen
  YAML, dimension for dimension and bound for bound.
- `test_observation_excludes_ground_truth` — no ground-truth fault field can
  reach the observation vector. **This test protects the entire research claim.**
- `test_reward_components_sum` — the logged components sum to the total.
- `test_reward_safe_landing_beats_crash` — for matched trajectories, a controlled
  landing scores strictly above a crash.
- `test_reward_is_bounded` — no input produces an unbounded or NaN reward.
- `test_env_checker` — passes `gymnasium.utils.env_checker` with a stub backend,
  no simulator.
- `test_env_smoke_random_actions` (sim) — 1000 random-action steps without hang
  or crash, including at least one induced worker restart.
- `test_vecenv_4_workers` (slow) — 4 parallel envs step together for 10 k steps;
  no cross-talk, no leaks, no orphans.

### Done when

- [ ] Observation, action and reward specs frozen and version-stamped **before**
      the first training run
- [ ] Environment passes the contract tests and the random-action smoke test
      across a worker restart
- [ ] Training runs stably for 3+ seeds
- [ ] Learning curves show real improvement, not noise
- [ ] Reward components logged separately and inspected for hacking
- [ ] Policy beats the M8 rule-based baseline on mission success and crash rate
      with **non-overlapping confidence intervals** across seeds — or the null
      result is documented with evidence
- [ ] Checkpoints, configs, seeds and the run manifest saved together

### Watch out for

- **If training is too slow, fix it by shortening episodes and ending failed ones
  early — not by raising the decision rate.** The 5 Hz rate is what makes the
  problem small enough to learn.
- If PPO plateaus, **check the reward first.** Reward bugs look exactly like
  learning failures. Only after a genuine tuning effort should SAC be considered,
  and then it is reported as an extra comparison, not a silent swap.
- **A result showing RL only ties the rule-based baseline is a legitimate finding
  — report it.** Tuning endlessly until RL wins is how projects lose their
  integrity, and reviewers usually notice.
- Watch RAM. Four workers plus a learner on 16 GB is tight; M4 measured the
  actual number, use it.
- A worker that dies mid-rollout must not corrupt the batch. Decide explicitly
  whether the partial episode is discarded or truncated with bootstrapping, and
  make it consistent.
- Never call `rclpy.init()` in the learner process.

### Claude Code prompt (use plan mode)

```
Read CLAUDE.md, planning.md §7, docs/throughput.md, and milestones.md M9.

Plan, then implement, M9 tasks 1-3: freeze the observation/action/reward specs
as versioned YAML, build the Gymnasium environment as a thin wrapper over
experiments/episode_runner.py, and write the contract tests.

Constraints:
  - The env must NOT re-implement flying, reset, or logging. It calls
    EpisodeRunner.
  - Ground-truth fault state must be structurally unable to reach the
    observation. Write the test that proves this.
  - Spaces are read from the YAML; a test asserts they match.
  - Every episode has both a sim-time limit and a wall-clock watchdog, and
    returns truncated=True with a termination_reason rather than hanging.

Deliverables:
  - rl/envs/uav_fault_env.py, configs/rl/{observation,action,reward}_v1.yaml
  - tests/test_obs_action_spec.py, tests/test_reward_function.py,
    tests/test_env_contract.py (all no-sim, using a stub backend)

Verify: pytest tests/ -m "not sim and not slow" -q, then
pytest tests/sim/test_env_smoke.py -q

Do not: start training in this session; do not call rclpy.init() outside a
worker; do not put ground-truth fault state anywhere near the observation.
```

---
# M10 — Full experiments and results

*(was M9)*

**Goal:** run the complete comparison and produce the paper's figures and tables.

**Depends on:** M7, M8, M9. **Blocks:** M11, M13.

### Tasks

1. **Batch runner for the full matrix**, on the M4 farm:

   | | Condition | Fault | Detection | Recovery |
   |---|---|---|---|---|
   | C1 | Healthy | No | — | — |
   | C2 | Faulty, no recovery | Yes | — | PX4 default only |
   | C3 | Detection + rules | Yes | AI | State machine |
   | C4 | Detection + RL | Yes | AI | RL policy |
   | C5 | **Perfect detection + RL** | Yes | True state | RL policy |
   | C6 | **RL, detector disabled** | Yes | None | RL policy |

2. **Severity sweep**: 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0.
3. **100+ episodes per cell**, using held-out seeds never seen in training.
   The held-out seed set is generated once, committed, and never regenerated.
4. **`experiments/metrics.py` is the one place a metric is computed.** Detector,
   FSM and RL conditions are never measured by different code.
5. **Statistics**: means, 95% confidence intervals, significance tests, effect
   sizes. Report CIs across *seeds*, not across episodes within a seed — the
   episodes within one training seed are not independent samples of "the method".
6. **The RQ3 study**: artificially inject delay and false alarms into the
   detector output, and measure how recovery degrades.
7. **All figures generated by script** — never hand-edited.
8. **Resumability.** The full matrix is many hours of simulation. The runner must
   be able to resume from a partially complete results directory without
   re-running finished cells, and without double-counting.

### Files created

```
experiments/run_matrix.py
experiments/metrics.py          (the ONE place metrics are computed)
experiments/analysis/figures.py
experiments/analysis/stats.py
configs/experiments/main_comparison.yaml
configs/experiments/heldout_seeds.json    (generated once, committed)
results/main/                             (raw logs + generated figures)
```

### Tests (required)

```
tests/test_metrics.py
tests/test_matrix_runner.py
tests/test_stats.py
```

- `test_metrics_on_known_episode` — a hand-constructed episode with a known
  answer produces exactly that answer for every metric. Golden-file style.
- `test_metrics_handle_invalid_episodes` — episodes marked `valid=false` are
  excluded from headline metrics and counted separately.
- `test_success_definition_is_single_sourced` — "mission success" is defined
  once and used by every condition.
- `test_matrix_resume_skips_done_cells` — resuming a partial run does not re-run
  or double-count.
- `test_matrix_cell_isolation` — each cell's results land in its own directory
  and no cell writes into another's.
- `test_ci_across_seeds_not_episodes` — the CI helper refuses to be handed raw
  episodes without a seed grouping.
- `test_figures_regenerate_deterministically` — running the figure script twice
  produces identical output.

### Done when

- [ ] All six conditions run across all severities
- [ ] Every figure and table regenerates from raw logs with **one command**
- [ ] Confidence intervals reported everywhere — no bare averages
- [ ] The detection-delay study (RQ3) complete
- [ ] Re-running with the same seeds reproduces the numbers within the M3 task 7
      divergence band
- [ ] Invalid/restarted episodes accounted for explicitly in every table

### Watch out for

- **C5 versus C4 is the most informative comparison in the paper**: it separates
  "our detector is imperfect" from "our policy is imperfect". Reviewers always
  ask; we will already have the answer.
- All conditions must use identical PX4 parameters. Any difference is a
  confound that invalidates the comparison.
- Excluded episodes are a result, not a nuisance. If C4 has a higher restart
  rate than C3, that is information about the method, and hiding it is
  misconduct.
- Do not compute CIs over pooled episodes from all seeds — it understates
  uncertainty dramatically and is the most common statistical error in RL papers.

---

# M11 — Generalization tests

*(was M10)*

**Goal:** find out where the approach works and where it breaks. **No retraining
— evaluation configs only.**

**Depends on:** M10.

### Axes

Unseen severities (interpolation *and* extrapolation beyond the training range),
unseen fault onset timings, unseen wind profiles, perturbed mass / inertia /
battery, unseen missions and initial conditions, and the hardest case — multiple
simultaneous faults or a fault type never trained on.

### Tests (required)

```
tests/test_generalization_configs.py
```

- `test_heldout_axes_disjoint_from_training` — every generalization config uses
  parameter values outside the training distribution, checked automatically
  against the M9 randomisation ranges. Doing this by eye is how "held-out" sets
  quietly stop being held out.
- `test_no_retraining_in_eval_path` — the evaluation entry point cannot load an
  optimiser or take a gradient step.

### Done when

- [ ] Generalization table complete across all axes
- [ ] Failure modes described honestly, in plain terms
- [ ] Each axis's distance from the training distribution stated quantitatively

**Watch out for:** the instinct to hide poor generalization. A clear statement of
*where the method stops working* is worth more to reviewers than a table of
uniform success, which mostly reads as untested.

---

# M12 — Hexacopter extension

*(was M11)*

**Goal:** test whether the method transfers to an aircraft with spare rotors.

**Important:** PX4 has **no Gazebo hexacopter model** — only a simplified-physics
one and a JSBSim one. We build the model and airframe ourselves. This is real
work, not a configuration flag.

**Depends on:** M10.

### Tasks

1. Build a hexacopter SDF model, including our degradation plugin (which is
   already per-model and per-rotor, so it should need no changes — verify that).
2. Create the PX4 airframe file with correct six-rotor mixing, in
   `simulation/airframes/`, loaded without modifying the PX4 tree.
3. Extend `instance_spec.py` and the feature config for six rotors — the feature
   vector length changes, so this is a `feature_version` bump, not an edit.
4. Retrain or fine-tune the detector for six rotors.
5. Evaluate the policy zero-shot, then after fine-tuning.

### Tests (required)

```
tests/test_hex_airframe_config.py
tests/test_feature_version_bump.py
```

- `test_rotor_count_drives_feature_length` — the feature vector length follows
  the airframe's rotor count from config, with no hardcoded 4.
- `test_quad_checkpoint_rejected_on_hex` — loading a quad-trained detector
  against hex features fails loudly rather than silently misaligning columns.

### Done when

- [ ] Hexacopter flies the baseline mission when healthy
- [ ] The full fault → detect → recover loop works on it
- [ ] Feature version bumped; quad and hex artifacts cannot be confused

**Scientific note:** a hexacopter has spare rotors, so PX4 alone already handles
losing one far better than a quadcopter does. Our method's advantage should
*shrink*. Measuring that shrinking margin is a genuinely good result — report it
as a finding, not a disappointment.

---

# M13 — Paper and reproducibility package

*(was M12)*

**Goal:** a submittable paper and a package someone else can actually run.

**Depends on:** M10 (M11 and M12 strengthen it but do not gate it).

### Tasks

1. Write the manuscript around RQ1–RQ4.
2. `docs/reproduce.md` — complete instructions from a clean machine, including
   the parallelism setup, since a reader with a different core count needs to
   know how to pick N.
3. One-command reproduction script for the headline result.
4. Archive trained models, configs, seeds, raw results and run manifests.
5. Tag a release; record a short demo video (the dashboard's replay mode is the
   easiest way to produce this).

### Done when

- [ ] A clean machine reproduces the headline result following `docs/reproduce.md`
      alone
- [ ] Every number in the paper traces back to a file in `results/`
- [ ] Every result's run manifest identifies the exact code and config that
      produced it
- [ ] Repository tagged and archived

---

## Parallel track — Web dashboard

Can start any time after **M5**. Not required for any research result, so it must
never delay M6–M10.

FastAPI backend with a ROS 2 node → WebSocket → React frontend. Shows position,
attitude, velocity, battery, per-motor health, detected fault and severity,
recovery state, mission progress, and telemetry plots.

Build the **replay mode** (playing back a saved episode record) before the live
mode — it is more useful for paper figures and the demo video, it does not
require a running simulator to develop against, and it exercises the episode
schema, which is a useful check on M3.

No database, no login, no containers. The backend imports `ai/features/`; it does
not reimplement it.

---

## Parallel track — Isaac Sim (learning, optional)

Can start any time — no dependency on any other milestone, and it must never
delay M6–M10. See `planning.md` §10b and §14 (D6) for the full reasoning.

Stack: Isaac Sim (pip install, its own conda env, kept separate from
`aero-safe-rl`) + Pegasus Simulator's PX4 MAVLink backend, driving the same
pinned PX4 `v1.17.0` binary. Goal: get the x500 quad flying the M3 baseline
mission in Isaac Sim, single instance. Stretch goal: port the rotor degradation
fault via an `omni.physx` physics-step callback instead of a compiled plugin —
plausibly *simpler* than the Gazebo version.

This machine's RTX 2070 (8 GB) is under Isaac Sim's stated minimum spec. Expect
real performance ceilings and treat them as expected, not as a problem to solve.
This track exists for learning, not for a number that ends up in the paper.

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
