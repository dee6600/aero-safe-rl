# Autonomous UAV AI Fault Detection and Fault-Tolerant Control using Reinforcement Learning

**Planning document — roadmap only. No implementation.**

Status: Phases 0–1 done. Toolchain pinned and verified (Phase 0); headless
SITL scripted and RTF measured (Phase 1). Phase 2 (ROS 2 ↔ PX4) in progress.

**Revised 2026-08-20** after a multi-instance review that read the pinned PX4
source and ran two instances side by side. It found four structural problems in
how parallel simulation was planned, all of which produce workers that start
cleanly and then silently do nothing. The roadmap now contains a dedicated
**Phase 4 — parallel simulation farm**, decisions **D7–D11**, and a
reproducibility standard that matches what this stack can actually deliver.
Evidence: `docs/parallelism.md`. Coding rules: `CLAUDE.md`.

Companion documents: `milestones.md` (build order), `CLAUDE.md` (coding rules),
`docs/parallelism.md` (verified multi-instance behaviour).
Last updated: 2026-08-20

---

## 0. Verified environment state (inspected, not assumed)

This section records what is *actually* on the machine today, because several
assumptions in the original project brief did not hold. Phase 0 exists to close these gaps.

| Item | Assumed | Actual (verified) |
|---|---|---|
| Project repo path | `~/projects/aero-safe-rl/` | ✅ Renamed from `aero-safe-rf/`; now matches. `git init` done. |
| Repo contents | README, `project_setup.md`, 11 subdirs | Currently just `planning.md`, `milestones.md`, `.gitignore` — subdirs created in M0 |
| ROS 2 | Humble | Humble present at `/opt/ros/humble` |
| Gazebo Harmonic | target stack | **Not installed yet.** Only Gazebo Classic 11 + old `libignition-*` (Fortress-era) |
| Python environment manager | — | ✅ **Miniconda installed** at `~/miniconda3`; empty env `aero-safe-rl` created (Python 3.10.20, no packages yet) |
| PyTorch + CUDA | "verified" | **Not installed yet** — will be installed into the `aero-safe-rl` conda env in M0 |
| GPU / driver | RTX 2070 Max-Q, 580.173.02 | Confirmed. 8 GB VRAM. `nvcc` absent (fine — PyTorch wheels ship their own CUDA runtime) |
| Python | 3.10.12 (system) | Confirmed; project now uses conda env `aero-safe-rl` (Python 3.10.20) instead of system Python |
| PX4 | `~/projects/PX4-Autopilot`, main | Confirmed, on `main` @ `v1.18.0-beta1-307-gbb59c637cd`, 35 submodules initialised, **not built** |
| PX4↔ROS 2 bridge | — | **`MicroXRCEAgent` absent, `px4_msgs`/`px4_ros_com` absent** |
| Docker | not installed | Confirmed absent — and we keep it that way |

**Package manager decision:** the project uses **conda** (Miniconda), not a
plain `venv`, for the Python environment. The environment is named
`aero-safe-rl` and currently contains only the Python 3.10 interpreter — no
packages installed yet. All Python tooling (PyTorch, ROS 2 Python bindings via
`colcon`, Gymnasium, Stable-Baselines3, etc.) will be installed into this env
during M0, not into system Python.

Other findings that shape the plan:

- **PX4 fault injection is built in.** `src/systemcmds/failure` supports units
  `motor`, `esc`, `servo`, `battery`, `gyro`, `accel`, `mag`, `baro`, `gps`,
  `rc_signal`, `mavlink_signal` and types `off`, `stuck`, `garbage`, `wrong`,
  `slow`, `delayed`, `intermittent`, gated by `SYS_FAILURE_EN`. Present in
  v1.16.2 and v1.17.0.
- **`failure_injection_manager` is a `main`-only module** (not in any stable
  tag). Do not build the fault framework on it if we pin a stable release.
- **PX4 `main` ships a Gazebo `MotorFailureSystem` plugin**
  (`src/modules/simulation/gz_plugins/motor_failure/`, 344 lines, driven by a
  gz-transport `Int32` topic). It is **binary only** (motor N fully dead) and
  **absent from v1.16.2 and v1.17.0**. It is a useful reference implementation,
  not something we can depend on. See D2.
- Stable PX4 tags available locally: `v1.16.0/.1/.2`, `v1.17.0`. Both install
  **Gazebo Harmonic** via `Tools/setup/ubuntu.sh`, both ship the `gz_x500` quad
  airframe family.
- **No Gazebo hexacopter airframe exists in PX4** (only `sihsim_hex` and a
  JSBSim hexarotor). Phase 13 therefore requires a custom SDF model + airframe
  file, and must be budgeted as real work, not a config flag.

---

## 1. Project objective

Build a reproducible, research-grade simulation platform in which an autonomous
quadcopter (1) detects actuator and sensor faults from telemetry using a learned
model, and (2) recovers from them using a **high-level** reinforcement learning
policy that commands the PX4 flight controller rather than replacing it.

The platform must produce paper-quality, reproducible experiments, and must be
architected so the same policy interface could later drive real hardware.

**Explicitly out of scope for v1:** motor-level RL control, sim-to-real flight
tests, hexacopter, dashboard, multi-vehicle.

---

## 2. Research question

> Does combining telemetry-based AI fault detection with a high-level RL
> recovery policy improve UAV mission resilience under actuator and sensor
> faults, compared to (a) no recovery and (b) fixed rule-based recovery?

Sub-questions the experiment design must be able to answer:

- **RQ1 (Detection)** — How accurately and how quickly can a learned detector
  identify partial actuator degradation from PX4 telemetry alone, at severities
  below PX4's own failure-detector threshold?
- **RQ2 (Recovery)** — Given a fault estimate, does a learned high-level policy
  outperform a hand-tuned rule-based policy on mission success and safety?
- **RQ3 (Coupling)** — How sensitive is recovery performance to detection
  latency and false positives? (i.e. is the *combination* more than the sum?)
- **RQ4 (Generalization)** — Does the policy transfer to unseen fault
  severities, timings, wind, and vehicle parameters?

### Framing the novelty honestly

PX4 already contains a `FailureDetector` and control-allocation-based actuator
failure handling. A paper claiming "RL beats PX4 at motor mixing" would be weak
and hard to defend. The defensible contribution is at a **different layer**:

1. **Sub-threshold, partial degradation** — PX4 handles binary motor loss
   reasonably; it does not reason about a rotor at 60% effectiveness. That is
   where a learned detector earns its place.
2. **Mission-level recovery decisions** — what PX4 does *not* do: decide
   whether to continue, degrade speed, re-plan, loiter, or land *given an
   uncertain fault estimate*. This is a sequential decision problem under
   partial observability, which is a legitimate RL problem.
3. **The detection–recovery coupling (RQ3)** — studying how detector latency
   and error propagate into closed-loop recovery outcomes. This is the most
   publishable angle and is under-studied.

Keep the low-level controller fixed (PX4) in all conditions so the comparison
isolates the contribution.

---

## 3. Core architecture

```
                    ┌──────────────────────────────────────────┐
                    │      Gazebo Harmonic (gz-sim 8)          │
                    │  x500 quad model + fault injection       │
                    └──────────────┬───────────────────────────┘
                                   │ gz-transport
                    ┌──────────────┴───────────────────────────┐
                    │           PX4 SITL (pinned tag)          │
                    │  EKF2 · Commander · Control Allocation   │
                    └──────────────┬───────────────────────────┘
                                   │ uXRCE-DDS (MicroXRCEAgent)
                    ┌──────────────┴───────────────────────────┐
                    │              ROS 2 Humble                │
                    │  px4_msgs · telemetry/state pipeline      │
                    └──────────────┬───────────────────────────┘
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
┌───────┴────────┐      ┌──────────┴──────────┐    ┌──────────┴─────────┐
│ Fault Detector │─────▶│  Recovery Policy    │    │  Logging / Metrics │
│  (AI, ~10 Hz)  │ fault│  rule-based OR RL   │    │  (episode records) │
│  window → ŷ    │ state│      (~5 Hz)        │    └────────────────────┘
└────────────────┘      └──────────┬──────────┘
                                   │ high-level command
                                   │ (setpoint / mode / limits)
                                   ▼
                            back to PX4 offboard
```

**Control hierarchy — non-negotiable:**

| Layer | Rate | Owner |
|---|---|---|
| Motor mixing, rate & attitude control, EKF2 | 250–1000 Hz | **PX4 (untouched)** |
| Position/velocity setpoint tracking | 50 Hz | **PX4** |
| Fault detection | ~10 Hz | Ours (AI) |
| High-level recovery decisions | ~5 Hz | Ours (RL) |

The RL policy's action is a **desired high-level command**, never a motor
command. This is what makes the design hardware-transferable: the same
ROS 2 node could publish to a real PX4 over the same interface.

### The worker — the unit everything is built from

A **worker** is one fully isolated simulation stack. Everything that runs more
than one flight — dataset generation, evaluation, RL training — is N workers
plus a supervisor, never a special mode.

```
worker k  ┌─────────────────────────────────────────────────┐
          │ gz sim -s          GZ_PARTITION=aero_k          │  own world,
          │ MicroXRCEAgent     udp port 8888+k              │  own clock,
          │ px4 -i k           ROS_DOMAIN_ID=k              │  own crash
          │                    PX4_UXRCE_DDS_NS=px4_k       │  domain
          │                    MAV_SYS_ID=k+1               │
          │ python env node    topics /px4_k/fmu/...        │
          └─────────────────────────────────────────────────┘
```

Three points that are not optional, each of which was a bug before it was a
rule (see D7–D9 and `docs/parallelism.md`):

- **One Gazebo server per worker**, isolated by `GZ_PARTITION`. PX4's default is
  to join an already-running world, which shares one physics thread, one clock
  and one crash domain across every worker.
- **We own the Gazebo server process**, so one worker can be restarted without
  disturbing its siblings.
- **Instance identity is uniform and read from a file.** No special case for
  instance 0, no value recomputed in a second place.

Above the workers sits a small supervision layer, and nothing else:

```
SimFarm  →  N × WorkerSupervisor  →  EpisodeRunner  →  one episode record
```

`EpisodeRunner` is the only implementation of "fly one episode" in the project.
The Gymnasium environment, the dataset generator and the evaluation harness are
all thin callers of it.

No Docker, no message broker, no database. Episode records go to Parquet files
under `results/`, one writer per file.

---

## 4. Development principles

1. **Nothing is "working" until it is validated by a checkpoint.** Every phase
   below has an explicit, runnable acceptance test.
2. **Pin everything.** PX4 tag, `px4_msgs` commit, Gazebo version, Python
   package versions, seeds. A result that cannot be regenerated is not a result.
3. **PX4 stays stock.** Do not patch PX4 source. Configure it via parameters,
   airframe files, and the `failure` command. If a patch becomes unavoidable,
   it lives as a tracked patch file in `simulation/patches/`, never as an
   in-place edit of `~/projects/PX4-Autopilot`.
4. **Config-driven, not code-driven.** Faults, missions, RL hyperparameters, and
   evaluation suites are YAML. One code path, many configs.
5. **The fault must be invisible to PX4's ground truth.** If we inject a fault
   in a way that PX4 already knows about, detection becomes trivial and the
   research question evaporates.
6. **Baselines before novelty.** The rule-based recovery baseline (Phase 8) must
   be genuinely well-tuned. A strawman baseline invalidates the paper.
7. **Simplicity over generality.** Quadcopter, one fault type, one algorithm
   first. Generalize only after the pipeline produces a real result.
8. **Sample efficiency is the #1 project risk** — see §7.4. Design around it
   from the start, not after training fails.
9. **Every interface is a versioned file, not a convention.** The feature
   vector, episode record, observation, action, reward and fault schemas each
   live in `configs/` with a version string, and a test asserts that what gets
   written validates against them. Four incompatible log formats is the normal
   outcome otherwise.
10. **Sim time is the only clock in flight logic.** At speed factor 8, a
    one-second wall-clock sleep is eight simulated seconds. Wall clock is
    permitted in exactly one place: the watchdog that decides a worker is hung.
11. **Anything that will run in parallel is verified with at least two
    concurrent instances.** PX4 gives instance 0 different topic names from
    every other instance, so single-instance testing is close to no testing.
12. **Failures are recorded, not discarded.** Over a multi-day run every worker
    dies repeatedly. Crashes are not uniformly distributed across fault
    severities, so silently dropping failed episodes biases exactly the
    comparison the paper depends on.

---

## 5. Phased roadmap

Ordering follows the brief with two deliberate changes:

- **Dashboard moved to a parallel track** (see §10) rather than a hard Phase 11
  gate. It has no research dependency and can be built any time after Phase 5.
- **Labelled-dataset generation folded into Phase 6.** The fault injection
  framework's first deliverable *is* the detector's training data — treating
  these as separate phases invites building the injector without the logging
  the detector needs.
- **Phase 4 — parallel simulation farm — added 2026-08-20**, shifting the old
  Phases 4–12 to 5–13. Parallel SITL was previously assumed to be a detail
  inside the RL phase; it is the project's largest source of silent bugs and the
  gate on whether the RL phase is feasible at all, so it is now a phase with its
  own acceptance criteria. It sits before the feature pipeline because its first
  real consumer is dataset generation (Phase 6), and proving it against the
  already-working Phase 3 mission is far cheaper than debugging it underneath a
  PPO run.

| Old | New | | Old | New |
|---|---|---|---|---|
| — | **4 (new)** | | 8 | 9 |
| 4 | 5 | | 9 | 10 |
| 5 | 6 | | 10 | 11 |
| 6 | 7 | | 11 | 12 |
| 7 | 8 | | 12 | 13 |

Rough effort estimates assume part-time work; they are for sequencing, not commitments.

---

### Phase 0 — Environment and reproducibility
**Effort: ~1 week**

- **Goal** — A pinned, documented, reproducible toolchain, and a real repository.
- **Deliverables**
  - ✅ Git repo initialised at `~/projects/aero-safe-rl` (renamed from `aero-safe-rf`).
  - ✅ Miniconda installed (`~/miniconda3`); empty conda env `aero-safe-rl` created (Python 3.10).
  - ✅ Directory scaffold created: `configs/ simulation/ ros2_ws/ ai/ rl/ experiments/ scripts/ tests/ results/ docs/`
    (`dashboard/` deliberately deferred — not needed until its parallel track starts, §10).
  - ✅ Gazebo Harmonic 8.15.0 installed from `packages.osrfoundation.org`, coexisting with Classic 11.
  - ✅ Pinned package list installed **into the `aero-safe-rl` conda env** (not system Python), recorded via `environment.yml`:
    PyTorch 2.13+cu126, Gymnasium, Stable-Baselines3, NumPy, SciPy, pandas, PyYAML, matplotlib, TensorBoard.
  - ✅ PX4 pinned to `v1.17.0` on branch `aero-safe-rl` (**decision D1, §14**).
  - ✅ `MicroXRCEAgent` built (to `~/.local`); `px4_msgs` (`release/1.17`) + `px4_ros_com` (`main`) vendored at commits matching the PX4 tag.
  - ✅ `docs/environment.md` recording every version; `scripts/env_report.sh` emitting them as JSON.
- **Tech** — apt, git, **conda**, colcon, CMake.
- **Validation** — ✅ all passed: `scripts/env_report.sh` runs clean; `conda activate aero-safe-rl && python -c "import torch; torch.cuda.is_available()"` → `True` on the RTX 2070; `gz sim --versions` reports 8.15.0; PX4 builds `make px4_sitl` without errors.

> ⚠️ `px4_msgs` **must** match the pinned PX4 tag. Mismatched uORB message
> definitions fail silently — topics appear but fields are garbage. This is the
> single most common way this stack breaks.

---

### Phase 1 — PX4 + Gazebo quadcopter simulation
**Effort: ~3 days**

- **Goal** — Reliable, headless, scriptable, deterministic-as-possible SITL.
- **Deliverables**
  - ✅ `scripts/sim_start.sh` (+ `sim_stop.sh`) launching PX4 SITL + Gazebo headless,
    with configurable instance, world, speed factor, and spawn pose.
  - ✅ Verified `PX4_SIM_SPEED_FACTOR` behaviour and the max stable factor on this
    machine: ~8× (compute-bound ceiling — requesting 16× does not exceed it;
    flight itself stayed stable at every tested factor). See `docs/simulation_notes.md`.
  - ✅ Documented instance-isolation scheme (ports, `PX4_GZ_MODEL_POSE`) for parallel
    SITL later — `ROS_DOMAIN_ID` isolation deferred to Phase 2, where ROS 2 is introduced.
  - ⚠️ **Partly superseded.** The notes record that N instances share one Gazebo
    process and treat that as a convenience. It is the PX4 default, but it is the
    wrong topology for this project (D7). Phase 1b reworks the launcher for
    per-worker isolation and ownership; the RTF numbers themselves stand.
- **Tech** — PX4 SITL, gz-sim 8, gz-transport.
- **Validation** — ✅ all passed: vehicle arms and holds altitude (5 m) in headless mode;
  RTF measured and logged at 1/2/4/8/16× requested; two SITL instances run concurrently
  without port or messaging collisions.

---

### Phase 1b — Worker isolation, ownership and identity
**Effort: ~2 days — opened 2026-08-20, blocks everything above it**

- **Goal** — Make one worker a self-contained unit that starts, stops and
  restarts independently, so "run N of them" becomes arithmetic rather than an
  experiment.
- **Deliverables**
  - `sim_start.sh` rewritten around the launch sequence in
    `docs/parallelism.md` §4: `GZ_PARTITION` per worker, Gazebo server started
    and PID-tracked by us (`PX4_GZ_STANDALONE=1`), `setsid` process groups,
    uniform `PX4_UXRCE_DDS_NS=px4_<N>`.
  - Readiness determined by a ROS topic arriving on the right domain, with a
    timeout and a non-zero exit — not by grepping a log for a startup string.
  - `simulation/instance_spec.py`: the pure identity function, plus an
    `instance_<N>.json` handshake file that both the shell and Python layers read
    instead of recomputing anything.
  - `sim_stop.sh -i N` kills only that worker's recorded PIDs; the name-based
    sweep is reserved for `--all`. `sim_status.sh` reports live workers.
- **Tech** — bash, gz-transport partitions, `setsid`, ROS 2 CLI.
- **Validation** — two workers produce two `gz sim` servers with independently
  honoured speed factors; stopping one leaves the other healthy; `--all` leaves
  zero processes; workers survive the exit of the launching shell; unit tests pin
  the identity mapping including PX4's `MAV_SYS_ID = N + 1` behaviour.

---

### Phase 2 — ROS 2 ↔ PX4 integration
**Effort: ~4 days**

- **Goal** — Bidirectional, verified ROS 2 link to PX4.
- **Deliverables**
  - `ros2_ws` builds `px4_msgs` + a project package (`aero_bridge`).
  - Confirmed subscriptions: `VehicleOdometry`, `VehicleAttitude`,
    `SensorCombined`, `ActuatorOutputs` / `ActuatorMotors`, `VehicleStatus`,
    `BatteryStatus`, `FailsafeFlags`, `EstimatorStatusFlags`.
  - Confirmed publications: `OffboardControlMode`, `TrajectorySetpoint`, `VehicleCommand`.
  - Latency measurement tool (`scripts/measure_latency.py`) for the round trip.
  - **Every node takes an `InstanceSpec`.** Topic names are built from the
    instance namespace and `target_system` from `MAV_SYS_ID`; no literal
    `/fmu/...` string and no hardcoded `1` survives anywhere (D9).
  - `PX4Clock` — the project's only wait primitive, timed from PX4 message
    timestamps with a wall-clock safety deadline (D10).
  - `arming_sequence` — the single implementation of stream setpoints → engage
    offboard → arm → confirm, with typed failures. Nothing re-implements it.
- **Tech** — uXRCE-DDS, `px4_msgs`, rclpy.
- **Validation** — A Python node commands takeoff → 5 m hover → land entirely
  over ROS 2, **on instance 1 as well as instance 0, with no code change**;
  measured PX4→ROS 2 telemetry latency documented (expect single-digit ms).

---

### Phase 3 — Autonomous flight baseline
**Effort: ~1 week**

- **Goal** — A repeatable autonomous mission that will serve as every experiment's nominal condition.
- **Deliverables**
  - Mission executor node: takeoff → waypoint sequence → hover → land, in offboard mode.
  - Mission definition in YAML (`configs/missions/`), starting with one primary mission (recommend a square or figure-8 circuit, 20–40 m, ~60 s).
  - **Three-tier reset ladder**, all built and each costed in wall-clock time:
    soft (reposition + EKF reset), medium (PX4 reboot, same Gazebo server), hard
    (full worker restart). Reset cost multiplies across a million RL steps, so it
    is measured, not estimated.
  - **Proof that the soft reset does not leak state** — 20 soft-reset episodes
    versus 20 hard-reset episodes at matched seeds, comparing initial EKF
    innovations, initial position error and mission RMSE. Leaked state biases
    training invisibly, so "it looks fine" is not evidence.
  - **The episode record schema**, versioned in `configs/schema/`, with a
    validator every writer runs before writing. Phases 6, 7, 9 and 10 all read
    this format; defining it once is what stops four incompatible log formats.
  - Episode logger writing one row per control step + one summary row per
    episode, one writer per file.
  - **Run-to-run divergence at a fixed seed**, measured over 20 repeats. This
    number becomes the tolerance for every later reproducibility claim (D11).
- **Tech** — PX4 offboard mode, rclpy, YAML.
- **Validation** — 20 consecutive healthy missions with 100% success; position RMSE variance across seeds documented as the noise floor for later comparisons. **This noise floor is a required number — every later result is measured against it.**

---

### Phase 4 — Parallel simulation farm
**Effort: ~1 week — this phase decides whether Phase 9 is possible**

- **Goal** — N independent workers flying episodes unattended for hours, with
  failures handled rather than avoided, and a measured aggregate throughput
  number to budget the RL phase from.
- **Why it is a phase and not a detail** — parallel SITL is where this project's
  bugs actually live, and the first real consumer of it is dataset generation
  (Phase 6), not training. Proving the farm against the Phase 3 mission, which
  already works, means each failure has exactly one possible cause. Discovering
  the same bugs underneath a PPO run means debugging parallelism, reward shaping
  and convergence simultaneously.
- **Deliverables**
  - `EpisodeRunner` — the single implementation of "fly one episode". The
    Gymnasium env, dataset generator and evaluation harness are all callers.
  - `WorkerSupervisor` — owns one worker's three processes, health-checks them,
    restarts them, and reports whether a restart invalidated the episode.
  - `SimFarm` — owns N supervisors, deterministic worker→instance assignment,
    context-managed teardown that survives exceptions and Ctrl-C.
  - Structured failure handling: a closed `termination_reason` enum, invalid
    episodes recorded rather than dropped, per-worker restart budgets, and a
    run-level abort when the restart rate would bias the dataset.
  - Run manifest: run id, repo and PX4 SHAs, config digests, seeds,
    worker→instance map, toolchain versions, outcome and restart counts.
  - **World topology as a parameter, not an assumption** — `worlds` and
    `drones_per_world` in the farm config, **fixed at 1 drone per world for the
    build** (D7). Fully isolated and fully shared are two points on one code
    path, never two code paths; the parameter exists so the benchmark can sweep
    them, not so later phases can depend on them.
  - **Measured throughput table** across the topology grid (isolated / hybrid /
    shared) × speed ∈ {1,2,4,8}: aggregate simulated-seconds per wall-second,
    episodes per hour, per-worker RTF stdev, peak RSS, CPU utilisation. Written
    to `docs/throughput.md` with the chosen operating point stated, and with the
    prediction it was testing recorded in advance so the data can falsify it.
- **Tech** — Python multiprocessing (`spawn`, never `fork`-after-`rclpy.init`),
  `GZ_PARTITION` isolation, `setsid` process groups, `psutil`.
- **Validation** — 4 workers × 100 episodes unattended, zero orphan processes,
  flat memory; killing one worker's PX4 mid-episode restarts only that worker
  while the others keep flying; `pgrep -cf "^gz sim "` equals the worker count.

> **Gate.** If the best aggregate throughput implies Phase 9 cannot reach 1–3 M
> environment steps in roughly five days, stop and revisit decision D5 before
> building anything above this phase. That is the whole reason this measurement
> happens here rather than in Phase 9.

---

### Phase 5 — Telemetry / state pipeline
**Effort: ~4 days**

- **Goal** — A single, versioned feature vector consumed identically by the detector, the RL policy, and the dashboard.
- **Deliverables**
  - `FeatureExtractor` producing a fixed-length vector at 10 Hz from a sliding window (recommend 1.0–2.0 s).
  - Candidate features: attitude + rates, angular acceleration estimates,
    velocity/position error vs setpoint, per-motor normalised outputs, thrust
    setpoint vs achieved acceleration residual, control-allocation residual,
    EKF innovations, current draw, vibration metrics.
  - Normalisation statistics computed from healthy flights and **frozen** to disk.
  - `configs/features.yaml` with an explicit `feature_version` string.
- **Tech** — NumPy, ROS 2 message filters.
- **Validation** — Feature vector logged during a healthy mission with no NaNs/gaps; replaying a recorded flight reproduces the identical vector (bit-for-bit determinism check).

> The **thrust-setpoint vs achieved-acceleration residual** is likely the single
> most informative feature for actuator degradation — a degraded rotor forces the
> allocator to command more while delivering less. Ensure it is in v1.

---

### Phase 6 — Fault injection framework (+ labelled dataset)
**Effort: ~1.5 weeks — this phase is the research foundation**

- **Goal** — Reproducible, parameterised, severity-controllable faults, plus the labelled dataset the detector needs.
- **Deliverables**
  - Fault injection interface supporting: fault type, affected component index,
    onset time, severity ∈ [0,1], profile (step / ramp / intermittent), duration.
  - **v1 fault: single-rotor partial thrust degradation** (see §6).
  - `configs/faults/*.yaml`; deterministic sampling from a seeded RNG.
  - Ground-truth fault labels written into every episode log alongside features.
  - Dataset generator producing a balanced corpus across severities and onset times (target: 500–1000 episodes, healthy + faulty).
- **Tech** — a project-owned gz-sim system plugin (`RotorDegradationSystem`)
  that scales rotor thrust by a severity factor received over gz-transport
  (primary); PX4 `failure` command (secondary, for binary and sensor faults),
  `SYS_FAILURE_EN`.
- **Validation** — A commanded 40% degradation on rotor 2 at t=20 s produces a
  visible, repeatable signature in the Phase 5 features, exceeding the Phase 3
  noise floor by a stated margin; the commanded severity is **confirmed applied**
  via the plugin's status echo rather than assumed (a gz-transport publish is
  fire-and-forget — without the echo it is entirely possible to generate 500
  episodes labelled "40% fault" in which no fault was ever applied); the same
  seed reproduces the same *fault schedule* bitwise and the same trajectory
  within the Phase 3 divergence band (D11); PX4's own `FailureDetector` does
  **not** trigger at the severities we target, asserted per episode.

> **Design decision D2 (§14).** Partial degradation is *not* supported by PX4's
> `failure` command, which is binary. Achieving graded severity requires acting
> at the Gazebo motor model instead. Recommended: scale the rotor's thrust
> coefficient via gz-transport at runtime. This is physically faithful and, critically,
> **PX4 has no knowledge of it** — preserving the detection problem.

---

### Phase 7 — AI fault detection
**Effort: ~2 weeks**

- **Goal** — A learned detector producing fault presence, type, and severity from the telemetry window.
- **Deliverables**
  - Offline-trained model on the Phase 6 dataset, with strict train/val/test
    splits **by episode, never by timestep** (timestep-level splits leak
    catastrophically across a sliding window).
  - Baselines for comparison: threshold-on-residual, and a classical model
    (random forest / SVM) on the same features.
  - Model outputs: `p(fault)`, fault class, severity estimate, plus an
    uncertainty proxy — the recovery policy needs to know when the detector is unsure.
  - Online inference node meeting the 10 Hz budget.
  - `ai/` module with training script, eval script, saved checkpoints, metrics report.
- **Tech** — PyTorch; recommend starting with a **1D-CNN or small GRU** over the
  window. A Transformer is not justified at this data scale.
- **Validation** — Test-set detection accuracy, per-severity ROC/AUC, and a
  **detection latency distribution** (time from fault onset to first sustained
  positive). Must beat both baselines. Online inference latency < 20 ms.

---

### Phase 8 — Rule-based fault recovery baseline
**Effort: ~1 week**

- **Goal** — A strong, honestly-tuned classical baseline. **Do not shortchange this.**
- **Deliverables**
  - Finite state machine: `NOMINAL → SUSPECTED → CONFIRMED → RECOVERING → LANDED/ABORTED`.
  - Hand-designed responses: reduce max velocity, lower altitude ceiling,
    reduce aggressiveness, hold position, divert to nearest safe point, controlled descent.
  - Hysteresis and debounce on detector output.
  - Thresholds tuned via a documented sweep — not guessed.
- **Tech** — Python FSM, same command interface the RL policy will use.
- **Validation** — Measurably improves mission success and crash rate over the
  no-recovery condition across the fault severity sweep. The tuning sweep is
  archived so reviewers can see the baseline was given a fair chance.

---

### Phase 9 — High-level RL fault recovery
**Effort: ~3–4 weeks — highest risk phase**

- **Goal** — An RL policy that consumes the fault estimate and outputs high-level commands, trained in the PX4-in-the-loop environment.
- **Deliverables**
  - Gymnasium environment wrapping the full stack (§7.1).
  - Observation, action, and reward specification frozen and documented before training starts (§7.2–7.3).
  - Observation, action and reward specs frozen as versioned YAML **before the
    first training run**, with a test asserting the environment's spaces match.
  - A Gymnasium environment that is a thin wrapper over Phase 4's
    `EpisodeRunner` — it must not re-implement flying, reset or logging.
  - PPO training pipeline over the Phase 4 farm, checkpointing, resumable,
    TensorBoard logging with **reward components logged separately** so reward
    hacking is visible rather than indistinguishable from learning.
  - Domain randomisation over fault severity, onset, mass/inertia, wind, sensor noise.
  - Trained policy checkpoints + training curves, ≥3 seeds.
- **Tech** — Stable-Baselines3 PPO, `SubprocVecEnv` (start method `spawn`,
  `rclpy.init()` only inside workers), PyTorch, CUDA.
- **Precondition** — `docs/throughput.md` exists and shows the sample budget is
  reachable. Budget from that measurement, never from an assumption.
- **Validation** — Policy exceeds the Phase 8 rule-based baseline on mission
  success rate and crash rate at matched fault severities, with non-overlapping
  confidence intervals across ≥3 training seeds. **A policy that merely
  ties the baseline is a legitimate finding — report it rather than tuning until it wins.**

---

### Phase 10 — Evaluation and paper-quality experiments
**Effort: ~2 weeks**

- **Goal** — The results tables and figures for the paper.
- **Deliverables**
  - Batch evaluation harness running the full condition matrix (§8).
  - ≥100 evaluation episodes per (condition × severity) cell, fixed held-out seeds never used in training.
  - Statistical analysis: means, 95% CIs, bootstrap or Mann-Whitney tests, effect sizes.
  - Publication figures generated by script from `results/`, never hand-edited.
  - The RQ3 ablation: detection latency and false-positive rate injected artificially to map their effect on recovery.
- **Tech** — pandas, SciPy, matplotlib, seeded evaluation configs.
- **Validation** — Every figure and table regenerates from raw logs with one command. Independent re-run with the same seeds reproduces the numbers.

---

### Phase 11 — Generalization and robustness
**Effort: ~2 weeks**

- **Goal** — Establish where the approach holds and where it breaks.
- **Deliverables** — Held-out evaluations across: unseen fault severities
  (interpolation and extrapolation), unseen onset timings, unseen wind profiles,
  perturbed mass/inertia/battery, unseen missions and initial conditions, and
  **multi-fault or unseen fault type** as the hardest case.
- **Tech** — Evaluation configs only; no retraining.
- **Validation** — A generalization table with an honest account of failure
  modes. Reviewers value a clear boundary far more than uniform success.

---

### Phase 12 — Hexacopter extension
**Effort: ~2 weeks — expect real work, not a config change**

- **Goal** — Test whether the method transfers to a platform with actuator redundancy.
- **Deliverables** — Custom hexacopter SDF model + PX4 airframe file (none
  exists for Gazebo in PX4); retrained or fine-tuned detector; policy evaluated
  zero-shot and after fine-tuning.
- **Tech** — SDF, PX4 airframe config, PX4 control allocation for hex geometry.
- **Validation** — Hex flies the baseline mission healthy; the fault→detect→recover
  loop closes. Scientific interest: a hexacopter is over-actuated, so PX4 alone
  handles single-rotor loss far better — quantifying the *shrinking* margin for
  our method is itself a strong result, and should be reported as such.

---

### Phase 13 — Research paper / publication package
**Effort: ~3 weeks**

- **Goal** — Submittable paper and an artifact others can run.
- **Deliverables** — Manuscript; complete reproducibility appendix; tagged repo
  release; one-command reproduction script; archived trained models, configs,
  seeds and raw results; short demo video.
- **Validation** — A clean machine, following `docs/reproduce.md` only,
  regenerates the headline result.

---

### Parallel track — Web dashboard (any time after Phase 5)
See §10. Not a research dependency; do not let it block Phases 6–10.

---

## 6. Initial fault model

**v1 — implement exactly one:**

> **Single-rotor partial thrust degradation.** Rotor `i ∈ {0..3}`, severity
> `s ∈ [0.2, 0.9]` (fraction of thrust lost), step or ramp onset at a randomised
> mission time, persisting to episode end.

Rationale: continuous severity gives a natural difficulty axis for every
experiment; it is sub-threshold for PX4's own detector at low `s`, so detection
is genuinely non-trivial; and it is physically realistic (prop damage, ESC
derating, motor wear, icing).

**Deferred fault backlog** (add only after the full pipeline works end-to-end,
roughly in this order):

| Priority | Fault | Mechanism |
|---|---|---|
| 2 | Complete rotor failure (`s = 1.0`) | Same interface, extreme severity |
| 3 | IMU noise / bias injection | PX4 `failure gyro`/`accel`, or sensor-level noise |
| 4 | GPS degradation / dropout | PX4 `failure gps` |
| 5 | Telemetry loss / delay | PX4 `failure mavlink_signal`, or ROS 2-side delay |
| 6 | Simultaneous multi-fault | Composition of the above |

Every fault is described by the same schema: `{type, component_index, onset_time,
severity, profile, duration}`. Adding a fault type must never require changing
the detector, environment, or evaluation code.

---

## 7. RL strategy

### 7.1 Environment
A Gymnasium `Env` wrapping PX4 SITL + Gazebo + the ROS 2 pipeline. Step rate
**5 Hz**. Episodes: 60–90 s of simulated time, terminated early on crash,
geofence breach, landing, or mission completion.

### 7.2 Observation, action, reward

**Observation** (recommend ~40–60 dims, all normalised):
- Vehicle state: attitude, angular rates, body velocity, position error vs setpoint, altitude AGL.
- Mission context: distance/bearing to next waypoint, waypoints remaining, elapsed time, battery.
- **Detector output**: `p(fault)`, fault class one-hot, severity estimate, uncertainty.
- Short history of the above (stack ~4 frames) to expose trends.

> The policy sees the *detector's estimate*, never ground truth. Ground-truth
> fault state may be used for reward shaping during training only — never as an
> input. Violating this destroys hardware transferability and any claim about RQ3.

**Action** — continuous, low-dimensional. Recommended v1 (5 dims):
1. Max horizontal velocity scale ∈ [0.1, 1.0]
2. Max climb/descent rate scale ∈ [0.1, 1.0]
3. Altitude setpoint offset ∈ [−5, +5] m
4. Mission-progress rate / hold ∈ [0, 1]
5. Abort–land commitment ∈ [0, 1] (thresholded to trigger controlled descent)

This is deliberately small. A large or discrete-heavy action space will not
train within this project's sample budget.

**Reward** — sparse terminal + dense shaping:
- `+R_success` mission completed
- `−R_crash` crash / geofence breach / uncontrolled descent
- `+r_progress` per-step waypoint progress
- `−λ₁ ·` tracking error, `−λ₂ ·` control effort/energy, `−λ₃ ·` attitude deviation
- Small `−r_time` to discourage indefinite loitering
- **Safe-landing partial credit**: a controlled landing after a severe fault is a
  good outcome, not a failure. Without this the policy learns to gamble.

All weights live in `configs/rl/*.yaml`, never in code.

### 7.3 Algorithm
**PPO only for v1.** On-policy, stable, robust to hyperparameters, well-suited to
a slow non-resettable simulator. SAC is off-policy and more sample-efficient in
principle, but replay-buffer benefits are undercut here by the environment's
wall-clock cost and it is far more sensitive to tuning.

Add SAC **only** if PPO fails to learn after a genuine tuning effort, and report
it as an ablation rather than swapping silently.

### 7.4 Sample budget — the project's main risk

Blunt assessment: PX4 SITL is slow, and this machine (12 threads, 16 GB) will
support roughly **3–4 workers**, not 32. Mitigations, in order of importance:

1. **Low decision rate (5 Hz)** — a 90 s episode is ~450 steps, not 22,500. This
   alone makes the problem tractable and is the main reason for the high-level
   design.
2. **Headless** — no GUI during training, ever.
3. **Speed factor** — measured single-instance ceiling on this machine is ~8×,
   compute-bound (Phase 1). Note that the speed factor is applied by a
   **world-level** `set_physics` service call, so it is per-worker only to the
   extent that a worker owns its world — drones sharing a world share one speed
   factor and one clock (D7).
4. **3–4 isolated workers** via `SubprocVecEnv` over the Phase 4 farm.
5. **Short episodes** — terminate early and decisively on crash.

Order-of-magnitude target: **1–3 M environment steps** in roughly 2–5 days.

**Do not budget from an assumption.** The tempting arithmetic — 4 workers × 8×
= 32× aggregate — is not supported by any measurement, and is almost certainly
wrong: each worker now runs its own Gazebo physics server, and gz-sim physics is
effectively single-threaded per world, so four workers contend for cores and
per-worker RTF falls. Phase 4 measures aggregate throughput directly across
N × speed-factor and picks the operating point. **Phase 9 is budgeted from that
one number and nothing else.**

If the measurement shows 1–3 M steps is unreachable, the fallback is a
reduced-order quadrotor model for policy pre-training with PX4-in-the-loop
fine-tuning — **deferred by default** (D5), since it adds a second dynamics
model and a sim-to-sim gap. Revisit only if Phase 4 forces it.

### 7.5 Reproducibility standard (D11)

PX4 SITL plus gz-sim is **not bitwise deterministic** across runs. Thread
scheduling, EKF timing and physics stepping all vary, and asserting otherwise
produces a "reproducibility check" that fails randomly and gets disabled.

What we guarantee instead, in decreasing order of strength:

| Level | What is fixed | Where it applies |
|---|---|---|
| **Bitwise** | Feature extraction, metrics, splits, fault schedules — every pure function of recorded data | Phases 5, 6, 7, 10 |
| **Distributional** | Same seed set → same distribution of outcomes, within a band measured in Phase 3 | Whole-pipeline results |
| **Provenance** | Every artifact records the code SHA, config digests, seeds and toolchain versions that produced it | Everything |

Phase 3 measures the run-to-run divergence band at a fixed seed and that number
becomes the tolerance every later reproducibility claim is stated against.
"Reproduces the headline result" in Phase 13 means *within that band*, and the
band is published alongside the result.

### 7.6 Failure handling contract

Over a multi-day run each of these happens many times: PX4 hangs, the EKF never
converges, the vehicle never arms, offboard drops out, the Gazebo server dies,
an episode never terminates. The design assumes them rather than hoping.

- Every wait has a deadline and a defined outcome. No unbounded loops.
- Episodes have **both** a sim-time limit and a wall-clock watchdog — sim-time
  alone cannot catch a frozen simulator, because sim time stops advancing.
- An episode ended by failure is written with `valid=false` and a
  `termination_reason` from a closed enum. It is never silently dropped.
- Restart counts go in the run manifest and are reported in results tables. A
  condition with a higher restart rate is telling you something about the
  method.

The full failure catalogue and the response to each is in
`docs/parallelism.md` §8.

---

## 8. Experiment methodology

The four-condition comparison, all sharing identical missions, seeds, and PX4 configuration:

| # | Condition | Fault injected | Detection | Recovery |
|---|---|---|---|---|
| C1 | Healthy baseline | No | — | — |
| C2 | Faulty, no recovery | Yes | — | PX4 default only |
| C3 | Detection + rule-based | Yes | AI detector | FSM (Phase 8) |
| C4 | Detection + RL | Yes | AI detector | RL policy (Phase 9) |

**Recommended additional conditions** (cheap to run, substantially strengthen the paper):

| # | Condition | Purpose |
|---|---|---|
| C5 | Oracle detection + RL | Upper bound: isolates detector error from policy quality (**answers RQ3 directly**) |
| C6 | Detection + RL, detector disabled at test | Measures how much the policy actually relies on the detector |

**Protocol**
- Severity sweep: `s ∈ {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0}`.
- ≥100 episodes per (condition × severity) cell.
- Fixed held-out evaluation seed set, disjoint from training seeds.
- ≥3 training seeds per learned condition; report mean ± 95% CI across seeds,
  never a single best run.
- Identical PX4 parameters across all conditions; any difference is a confound.

---

## 9. Evaluation metrics

**Detection**
- Accuracy, precision, recall, F1, ROC-AUC — reported *per severity level*
- **Detection latency** — onset → first sustained positive (mean, median, p95)
- False positive rate on healthy flights (per flight-minute)
- Severity estimation error (MAE)

**Recovery / flight**
- Mission success rate (primary headline metric)
- Crash / loss-of-control rate (primary safety metric)
- Safe-landing rate (partial success)
- Position RMSE and max deviation, pre- and post-fault
- Attitude error (RMS, max)
- Recovery time — fault onset → stable flight re-established
- Altitude loss during recovery
- Control effort / energy proxy
- Time-to-completion

**Robustness** — every metric above, reported across the generalization axes in Phase 11.

All metrics computed by one shared module (`experiments/metrics.py`) so that
detector, rule-based, and RL conditions are never measured by different code.

---

## 10. Dashboard concept (later, parallel track)

Read-only live monitoring. **Not on the research critical path — do not build it before Phase 10.**

```
PX4 → ROS 2 → FastAPI backend → WebSocket → React frontend
```

- **Backend** — one FastAPI process with a ROS 2 node, throttling telemetry to
  10–20 Hz and broadcasting JSON over a WebSocket. No database; keep a bounded
  in-memory ring buffer. Historical analysis is offline from `results/`.
- **Frontend** — React + a light charting library. Panels: 3D/2D position,
  attitude indicator, velocity, battery, per-motor health bars, sensor health,
  detected fault + severity + confidence, recovery FSM / RL state, mission
  progress, telemetry timeseries.
- **Modes** — live (attached to a running sim) and replay (from a saved episode log).
  Replay is the more useful mode for paper figures and demo video.

Explicitly excluded: authentication, multi-user, cloud deployment, databases,
containerisation.

---

## 10b. Isaac Sim track (later, parallel, optional — decision D6)

Not a research dependency. Exists for hands-on learning and, if it goes well,
a small cross-simulator validation data point in the paper. **Must never gate
Phases 6–13**, and must never be the environment M9 trains on.

- **Stack** — Isaac Sim (pip-installed, own conda env, isolated from
  `aero-safe-rl`) + Pegasus Simulator's PX4 `MavlinkBackend`, driving the same
  pinned PX4 `v1.17.0` SITL binary over MAVLink instead of gz-transport.
- **Scope** — port the x500 quadrotor and, if time allows, a single-instance
  version of the `RotorDegradationSystem` fault (via an `omni.physx`
  per-physics-step callback rather than a compiled plugin) far enough to fly
  the Phase 3 baseline mission. No RL training here.
- **Known constraint** — this machine's RTX 2070 (8 GB) is below Isaac Sim's
  stated minimum spec (see the D6 note above). Expect to run single-instance,
  possibly with reduced rendering fidelity, and to hit real performance
  ceilings — that is expected and fine for a learning track, not a blocker to
  fix.
- **Not required for**: any number in the paper. If it never produces a
  usable result, nothing above it changes.

---

## 11. Repository / module responsibilities

```
aero-safe-rl/
├── planning.md              # this document
├── README.md                # what it is, how to run it
├── environment.yml          # pinned conda env spec (env name: aero-safe-rl)
├── CLAUDE.md                # coding rules that apply to every phase
├── configs/                 # ALL experiment configuration (YAML)
│   ├── schema/              #   versioned record + interface schemas
│   ├── env/                 #   simulation + episode settings
│   ├── faults/              #   fault definitions and sweeps
│   ├── missions/            #   waypoint missions
│   ├── rl/                  #   algorithm + reward hyperparameters
│   ├── detector/            #   model + feature configuration
│   └── experiments/         #   full experiment matrices
├── simulation/              # PX4/Gazebo layer
│   ├── instance_spec.py     #   THE single source of instance identity
│   ├── gz_plugins/          #   RotorDegradationSystem (C++)
│   ├── models/              #   custom SDF (hex, modified x500)
│   ├── airframes/           #   PX4 airframe files
│   ├── faults/              #   fault injection control + schedules
│   └── patches/             #   tracked PX4 patches (avoid if possible)
├── ros2_ws/                 # colcon workspace
│   └── src/aero_bridge/     #   telemetry pipeline, mission executor, command interface
├── ai/                      # fault detection
│   ├── features/            #   feature extraction (shared with rl/ and dashboard/)
│   ├── models/              #   architectures
│   ├── train.py  eval.py
│   └── checkpoints/
├── rl/                      # reinforcement learning
│   ├── envs/                #   Gymnasium environment
│   ├── rewards/             #   reward functions
│   ├── policies/            #   rule-based FSM + RL policy wrappers
│   ├── train.py  eval.py
│   └── checkpoints/
├── experiments/             # orchestration + analysis
│   ├── episode_runner.py    #   THE single "fly one episode" implementation
│   ├── worker_supervisor.py #   one worker's processes + health
│   ├── sim_farm.py          #   N supervisors, restart policy, teardown
│   ├── run_manifest.py      #   provenance for every run
│   ├── episode_schema.py    #   episode record loader + validator
│   ├── run_matrix.py        #   batch runner
│   ├── metrics.py           #   THE single metrics implementation
│   └── analysis/            #   figure and table generation
├── dashboard/               # later: backend/ + frontend/
├── scripts/                 # env_report.sh, sim_start.sh, utilities
├── tests/                   # unit (default), sim/ (@sim), slow/ (@slow)
│   └── fixtures/            #   recorded telemetry — most tests need no sim
├── results/                 # raw logs, trained artifacts (git-ignored, except manifests)
└── docs/                    # environment.md, parallelism.md, throughput.md,
                             # simulation_notes.md, reproduce.md, paper drafts
```

**Boundaries that matter:**
- `ai/features/` is imported by `rl/` and `dashboard/`. Feature extraction is
  defined exactly once, and imports no ROS — which is what lets Phase 7 iterate
  offline in seconds.
- `experiments/metrics.py` is the only place a metric is computed.
- `experiments/episode_runner.py` is the only place an episode is flown. The
  Gymnasium env, the dataset generator and the evaluation harness are callers.
- `simulation/instance_spec.py` is the only place instance identity is derived,
  and both the Python and shell layers read it.
- `simulation/` knows nothing about learning; `ai/` and `rl/` know nothing about Gazebo.
- The recovery interface (`rl/policies/`) is identical for FSM and RL policies —
  they are swappable behind one API. This is what makes C3 vs C4 a fair comparison.

---

## 12. Definition of Done — major phases

| Phase | Done when |
|---|---|
| **0** | ✅ `scripts/env_report.sh` emits complete version JSON; CUDA verified inside the `aero-safe-rl` conda env; PX4 pinned tag builds; `px4_msgs` matched to that tag; `docs/environment.md` written |
| **1** | ✅ Headless SITL arms, hovers, lands from a script; measured RTF and max stable speed factor documented (~8×, compute-bound, see `docs/simulation_notes.md`); 2 concurrent instances verified |
| **1b** | One worker starts, stops and restarts independently; `pgrep -cf "^gz sim "` equals the instance count; identity read from `instance_<N>.json` |
| **2** | Takeoff→hover→land driven entirely from a ROS 2 Python node, **on instance 1 as well as 0**; telemetry latency measured; all required topics confirmed carrying valid data; no wall-clock sleeps in flight logic |
| **3** | 20/20 healthy missions succeed; position RMSE noise floor documented; run-to-run divergence band at fixed seed measured; all three reset tiers implemented and costed; episode schema frozen and validated |
| **4** | 4 workers × 100 episodes unattended, zero orphans, flat memory; single-worker restart proven not to disturb siblings; `docs/throughput.md` states the chosen operating point |
| **5** | Feature vector logged for a full healthy mission with no gaps; replay determinism verified bitwise; causality test passes; normalisation stats frozen |
| **6** | Graded severity injected and **confirmed applied**, visible in features above the Phase 3 noise floor; PX4 `FailureDetector` confirmed silent at target severities; ≥500-episode labelled dataset generated; PX4 tree unmodified |
| **7** | Detector beats threshold and classical baselines on held-out **episodes**; per-severity ROC and latency distribution reported; online inference < 20 ms |
| **8** | FSM measurably beats no-recovery across the severity sweep; tuning sweep archived; shared policy interface in place |
| **9** | Obs/action/reward specs frozen and version-stamped before training; PPO policy beats the tuned FSM on success and crash rate with non-overlapping CIs over ≥3 seeds — **or** the null result is documented with evidence |
| **10** | Full condition matrix (C1–C6) executed; all figures/tables regenerate from raw logs by one command; RQ3 ablation complete; invalid episodes accounted for explicitly |
| **11** | Generalization table complete, including honest failure-mode analysis |
| **12** | Hexacopter flies healthy mission and closes the fault→detect→recover loop |
| **13** | Clean machine reproduces the headline result from `docs/reproduce.md` alone, within the Phase 3 divergence band |

---

## 13. Immediate next steps

Phases 0 and 1 are complete. Phase 2 is in progress. The 2026-08-20 review
opened work that must land before Phase 2 can be closed.

1. ✅ **Decisions D1–D6 resolved** (§14) — all approved.
2. ✅ **Repo renamed and initialised** (`aero-safe-rf` → `aero-safe-rl`), `git init` done.
3. ✅ **Miniconda installed**, empty conda env `aero-safe-rl` (Python 3.10) created.
4. ✅ **`.gitignore` added** and committed.
5. ✅ **Directory scaffold created**: `configs/ simulation/ ros2_ws/ ai/ rl/ experiments/ scripts/ tests/ results/ docs/`.
6. ✅ **Gazebo Harmonic installed** from the OSRF apt repository alongside Classic 11; `gz sim --versions` reports 8.15.0.
7. ✅ **Packages installed into the `aero-safe-rl` conda env**: PyTorch 2.13+cu126, Gymnasium, Stable-Baselines3, NumPy, SciPy, pandas, PyYAML, matplotlib, TensorBoard; frozen to `environment.yml`; GPU access verified.
8. ✅ **PX4 pinned**: branch `aero-safe-rl` from tag `v1.17.0`, `make px4_sitl` builds clean.
9. ✅ **Bridge built**: `Micro-XRCE-DDS-Agent` (installed to `~/.local`, see `docs/environment.md`), `px4_msgs` (`release/1.17`) and `px4_ros_com` (`main`) vendored and colcon-built.
10. ✅ **`docs/environment.md` and `scripts/env_report.sh` written**, Phase 0 validation checklist passed.
11. ✅ **Phase 1**: `scripts/sim_start.sh`/`sim_stop.sh` written; RTF measured at 1/2/4/8/16× requested (achieves ~8.3× ceiling, compute-bound); flight stayed stable at every tested factor; 2 concurrent instances confirmed conflict-free. Full numbers in `docs/simulation_notes.md`.

### Open now, in order

1. **Install `pytest` and `pyarrow`** into the conda env and re-export
   `environment.yml`. Every phase from here requires unit tests, and the Parquet
   episode records of §3 cannot be written without `pyarrow`. Neither is
   currently installed.
2. **Phase 1b — worker isolation and ownership** (`milestones.md` M1b). Rewrite
   `sim_start.sh`/`sim_stop.sh` around `GZ_PARTITION`, `PX4_GZ_STANDALONE`,
   uniform identity and an `instance_<N>.json` handshake file. Add
   `simulation/instance_spec.py` as the single source of truth.
3. **Fix the two silent multi-instance bugs already present in
   `ros2_ws/src/aero_bridge/aero_bridge/px4_interface.py`**, each with a test
   that fails before the fix:
   - `target_system` is hardcoded to `1`; PX4's `Commander.cpp:746` drops any
     command whose `target_system` is neither `0` nor `MAV_SYS_ID`, and
     `MAV_SYS_ID = instance + 1`. Arm, offboard and land are therefore ignored
     on every instance except 0, with no error anywhere.
   - Topic names are hardcoded to `/fmu/out/...`; instance 1 publishes to
     `/px4_1/fmu/out/...`, so the subscription succeeds and receives nothing
     forever.
4. **Finish Phase 2** — the `PX4Clock` and `arming_sequence` helpers, and a
   `sim`-marked test that flies instances 0 and 1 concurrently.
5. **Phase 3**, which now also owns the episode record schema, the three-tier
   reset ladder, and the run-to-run divergence measurement that D11 depends on.
6. **Phase 4** — the parallel simulation farm, and the throughput measurement
   that Phase 9's budget is derived from.

**Do not start Phase 9 until `docs/throughput.md` exists and says the sample
budget is reachable.**

---

## 14. Decisions — ALL APPROVED (2026-08-14)

| ID | Decision | Resolution |
|---|---|---|
| **D1** | PX4 version to pin | ✅ **`v1.17.0`** — newest stable, closest to the current `main` checkout, full Gazebo Harmonic support, `failure` command present. |
| **D2** | Partial-degradation injection mechanism | ✅ **Gazebo-side rotor thrust scaling**, implemented as a **project-owned gz-sim system plugin** (`RotorDegradationSystem`) living in `simulation/gz_plugins/`, not in the PX4 tree. See the note below. |
| **D3** | Repo directory name | ✅ Rename `aero-safe-rf` → **`aero-safe-rl`**. |
| **D4** | Conditions C5 (oracle detection) and C6 (detector ablation) | ✅ **Included** in the evaluation matrix. |
| **D5** | Reduced-order pre-training model | ✅ **Deferred.** Revisit only if Phase 4 throughput measurements prove PX4-in-the-loop training unreachable. |
| **D6** | Isaac Sim as the simulator backend | ✅ **Declined for the research pipeline.** Gazebo Harmonic stays primary for M1–M13. Isaac Sim adopted only as an optional, non-blocking parallel learning track. See §10b and the note below. |

### Added 2026-08-20 after the multi-instance review

| ID | Decision | Resolution |
|---|---|---|
| **D7** | Simulator topology for parallel runs | ✅ **One drone per world**, `GZ_PARTITION`-isolated (settled 2026-08-20). `drones_per_world` exists as a parameter so Phase 4 can *measure* the hybrid and shared topologies; no phase depends on building them. Silent, unchosen sharing stays prohibited. |
| **D8** | Who owns the Gazebo server process | ✅ **We do** — `PX4_GZ_STANDALONE=1`, server started and PID-tracked by our launcher, so one worker can be restarted without touching its siblings. |
| **D9** | Instance identity | ✅ **Uniform, no special case for instance 0.** `PX4_UXRCE_DDS_NS=px4_<N>` for every N; `target_system = N+1` always; identity computed once in `simulation/instance_spec.py` and published as `instance_<N>.json`. |
| **D10** | Timing source in flight logic | ✅ **Simulated time only, sourced from `GzSimClock`** (Gazebo's native clock over gz-transport) — not `px4_msgs` timestamps, which M2 measured to track wall clock almost exactly regardless of speed factor (`uxrce_dds_client`'s session-level resync). Wall clock is permitted solely in the hang watchdog. See `docs/parallelism.md` §2.5. |
| **D11** | Reproducibility standard | ✅ **Statistical, not bitwise** — see §7.5. Pure functions of recorded data are bitwise reproducible; whole-pipeline results are reproducible within a divergence band measured in Phase 3. |

### Note on D7–D11 — what the review found

The review read the pinned PX4 v1.17.0 source and ran two instances side by side
on this machine. Four findings, each of which produces a worker that starts
cleanly and then silently does nothing:

1. **PX4 shares one Gazebo world between instances by default.**
   `px4-rc.gzsim` runs `gz topic -l`, finds an existing world and joins it. N
   instances then share one physics thread, one clock, one crash domain — and
   one `set_physics` setting, so the speed factor is world-level and
   last-writer-wins. Starting worker 2 at 1× silently drops worker 1 from 8× to
   1×. Setting a distinct `GZ_PARTITION` per instance was verified to produce N
   independent servers with independent RTF. → **D7**
2. **PX4 does not tell us its Gazebo server's PID**, so stopping one worker
   required a name-based `pkill` that also killed its siblings. Killing the
   `px4` process alone was verified *not* to cascade to `gz sim`, which then
   keeps running and consuming a core. → **D8**
3. **Instance 0 is a special case in PX4's own startup.** `rcS` applies the DDS
   namespace `-n px4_N` only when `N != 0`, so instance 0 publishes to
   `/fmu/out/...` and instance 1 to `/px4_1/fmu/out/...`. Separately,
   `MAV_SYS_ID = instance + 1` and `Commander.cpp:746` drops commands addressed
   to the wrong system id. Both mean code that passes every single-instance test
   fails silently the moment it is scaled up. → **D9**
4. **Wall-clock waits change meaning with the speed factor.** At 8×, a
   one-second sleep is eight simulated seconds — so mission logic debugged at 1×
   behaves differently in training. → **D10**

Full evidence, with source line references and measured output, is in
`docs/parallelism.md`. The coding rules that follow from these decisions are in
`CLAUDE.md`.

### Note on D2 — refinement after inspecting the code

The approved decision stands, but the mechanism needs one correction. There is
no runtime service to change `motorConstant` on the stock
`MulticopterMotorModel`, so "scale it via gz-transport" is not available
out of the box. Three options were considered:

| Option | Verdict |
|---|---|
| Edit the model SDF per episode | ✗ Severity fixed at spawn — no mid-flight fault onset |
| Python relay on the `command/motor_speed` topic | ✗ Adds latency and jitter inside the ~250 Hz actuator loop, risks breaking lockstep, and degrades *all* conditions — a confound |
| **Own gz-sim system plugin** | ✅ **Chosen** |

We write a small C++ gz-sim system plugin (~250–350 lines, closely modelled on
PX4's `MotorFailureSystem`) that holds a per-rotor efficiency factor in `[0,1]`,
receives updates over a gz-transport topic, and scales the rotor's contribution
each physics step. It runs *inside* gz-sim, so it adds no latency and cannot
disturb lockstep.

It lives in **our** repository and is loaded via `GZ_SIM_SYSTEM_PLUGIN_PATH`
plus a line in our own copy of the x500 SDF. That keeps PX4 completely stock
(principle #3) and decouples the fault framework from the PX4 version — the
plugin survives a future PX4 bump.

### Note on D6 — Isaac Sim considered and declined for the primary pipeline (2026-08-15)

The question was raised after M0 completed: switch the simulator backend from
Gazebo Harmonic to NVIDIA Isaac Sim, partly to learn the tool. Investigated
before deciding rather than guessing:

- **No native PX4 integration.** The pinned PX4 `v1.17.0` source has no Isaac
  Sim SITL target (checked directly — only Gazebo/jMAVSim/FlightGear/JSBSim
  are wired into the Makefile). The real integration path is **Pegasus
  Simulator**, a third-party framework with a MAVLink-based PX4 backend. This
  part is solid: actively maintained, committed through 2027, synced to
  current Isaac Sim releases.
- **GPU is under Isaac Sim's own stated minimum.** Isaac Sim 5.1's minimum is
  an RTX 4080 with 16 GB VRAM; even the older 4.5 baseline was RTX 3070 8 GB.
  This machine's RTX 2070 Max-Q (8 GB, Turing) sits below both — real risk of
  poor performance or instability, not just "slower."
- **Conflicts with the Phase 9 parallelism plan.** Isaac Sim's efficient
  parallelism (thousands of robots in one GPU context via Isaac Lab) doesn't
  apply here: this project deliberately keeps a full separate PX4 process per
  vehicle, so PX4 never learns about the fault (principle #5). That means N
  genuinely separate heavy GPU contexts, not vectorized envs — NVIDIA's own
  guidance is one instance per 8 GB card. The planned 4 parallel SITL
  instances (§7.4) would likely collapse to 1, stretching Phase 9's
  already-flagged risk budget considerably.
- **Reproducibility cost.** Gazebo is apt-installable, free, no login. Isaac
  Sim gates readers behind an NVIDIA account and a much heavier GPU
  requirement, working against Phase 13's "clean machine reproduces the
  result" goal.
- **One genuine upside, noted for the record.** Isaac Sim's Python
  physics-step callback API (`omni.physx`) could make fault injection
  *easier* than the planned C++ gz-sim plugin — no compiled plugin needed.
  This doesn't outweigh the GPU/parallelism risk above, but it's worth
  remembering if D2 is ever revisited.

**Resolution:** Gazebo remains the backbone for every research milestone.
Isaac Sim + Pegasus Simulator is being set up as a separate, optional track
(§10b) — useful for hands-on learning and possibly a small cross-simulator
validation note in the paper later, but it must never gate M6–M13.

---

## Appendix — Known risks

| Risk | Severity | Mitigation |
|---|---|---|
| RL training too slow on this hardware | **High** | Low decision rate, headless, speed factor, 3–4 isolated workers; **Phase 4 measures aggregate throughput and gates Phase 9 on it**; D5 fallback |
| Silent multi-instance bugs (shared world, namespace, `target_system`) | **High** | D7–D9; identity computed once in `instance_spec.py`; every simulator-touching change verified with ≥2 concurrent instances; unit tests pin PX4's `rcS` behaviour so a version bump fails a test rather than a training run |
| Worker failures biasing the dataset | **High** | Failures recorded with `valid=false` and a reason, never dropped; restart counts in the run manifest and in results tables; run-level abort above a restart-rate threshold |
| Wall-clock timing bugs appearing only at speed factor > 1 | Medium | D10; `PX4Clock` is the only wait primitive; a grep check in the Phase 2 verification block |
| Divergent implementations of "fly one episode" / features / metrics | Medium | Exactly one implementation of each, enforced by principle #4 and by phase ordering — `EpisodeRunner` (Phase 4), `ai/features/` (Phase 5), `experiments/metrics.py` (Phase 10) |
| Interface drift between detector, policy and evaluation | Medium | Versioned schema files in `configs/` with validation tests (principle #9) |
| `px4_msgs` / PX4 version mismatch | **High** | Pin both together in Phase 0; verify field values, not just topic presence |
| PX4's own failure handling masks our contribution | **High** | Target sub-threshold severities; verify `FailureDetector` stays silent (Phase 6 validation); frame contribution at the mission layer |
| Weak rule-based baseline undermines the paper | **High** | Documented tuning sweep, archived as evidence |
| SITL non-determinism blocks reproducibility | Medium | D11 — statistical standard with a divergence band measured in Phase 3; bitwise determinism required only of pure functions over recorded data |
| Episode reset requires full SITL restart | Medium | Three-tier reset ladder built and costed in Phase 3; soft reset *proved* equivalent to hard reset before it is trusted, since leaked state biases training invisibly |
| 8 GB VRAM limits model size | Low | Models here are small (1D-CNN/GRU, MLP policy); VRAM is not the bottleneck — wall-clock simulation is |
| Scope creep (dashboard, hexacopter, extra faults) | Medium | Phases 11–12 and the dashboard are explicitly gated behind a complete Phase 10 |
```
