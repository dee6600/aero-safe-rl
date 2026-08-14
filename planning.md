# Autonomous UAV AI Fault Detection and Fault-Tolerant Control using Reinforcement Learning

**Planning document — roadmap only. No implementation.**

Status: pre-Phase-0. Repository initialised, environment tooling in place;
no simulation, ROS, or ML code written yet.
Last updated: 2026-08-14

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
  JSBSim hexarotor). Phase 12 therefore requires a custom SDF model + airframe
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

### Two-process design (keep it simple)

- **Process A** — `PX4 SITL + gz-sim + MicroXRCEAgent` (launched by a script).
- **Process B** — Python: ROS 2 node wrapping a Gymnasium environment, plus
  detector and policy.

No Docker, no message broker, no database. Episode records go to Parquet/CSV
files under `results/`.

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
6. **Baselines before novelty.** The rule-based recovery baseline (Phase 7) must
   be genuinely well-tuned. A strawman baseline invalidates the paper.
7. **Simplicity over generality.** Quadcopter, one fault type, one algorithm
   first. Generalize only after the pipeline produces a real result.
8. **Sample efficiency is the #1 project risk** — see §7.4. Design around it
   from the start, not after training fails.

---

## 5. Phased roadmap

Ordering follows the brief with two deliberate changes:

- **Dashboard moved to a parallel track** (see §10) rather than a hard Phase 10
  gate. It has no research dependency and can be built any time after Phase 4.
- **Labelled-dataset generation folded into Phase 5.** The fault injection
  framework's first deliverable *is* the detector's training data — treating
  these as separate phases invites building the injector without the logging
  the detector needs.

Rough effort estimates assume part-time work; they are for sequencing, not commitments.

---

### Phase 0 — Environment and reproducibility
**Effort: ~1 week**

- **Goal** — A pinned, documented, reproducible toolchain, and a real repository.
- **Deliverables**
  - ✅ Git repo initialised at `~/projects/aero-safe-rl` (renamed from `aero-safe-rf`).
  - ✅ Miniconda installed (`~/miniconda3`); empty conda env `aero-safe-rl` created (Python 3.10).
  - Directory scaffold: `configs/ simulation/ ros2_ws/ ai/ rl/ experiments/ dashboard/ scripts/ tests/ results/ docs/`.
  - Gazebo Harmonic installed from `packages.osrfoundation.org`, coexisting with Classic 11.
  - Pinned package list installed **into the `aero-safe-rl` conda env** (not system Python), recorded via `environment.yml`:
    PyTorch+CUDA, Gymnasium, Stable-Baselines3, NumPy, SciPy, pandas, PyYAML, matplotlib.
  - PX4 pinned to a stable tag on a project branch (**decision D1, §14**).
  - `MicroXRCEAgent` built; `px4_msgs` + `px4_ros_com` cloned at commits matching the PX4 tag.
  - `docs/environment.md` recording every version; `scripts/env_report.sh` emitting them as JSON.
- **Tech** — apt, git, **conda**, colcon, CMake.
- **Validation** — `scripts/env_report.sh` runs clean; `conda activate aero-safe-rl && python -c "import torch; torch.cuda.is_available()"` → `True` on the RTX 2070; `gz sim --versions` reports 8.x; PX4 builds `make px4_sitl` without errors.

> ⚠️ `px4_msgs` **must** match the pinned PX4 tag. Mismatched uORB message
> definitions fail silently — topics appear but fields are garbage. This is the
> single most common way this stack breaks.

---

### Phase 1 — PX4 + Gazebo quadcopter simulation
**Effort: ~3 days**

- **Goal** — Reliable, headless, scriptable, deterministic-as-possible SITL.
- **Deliverables**
  - `scripts/sim_start.sh` launching `make px4_sitl gz_x500` headless.
  - Verified `PX4_SIM_SPEED_FACTOR` behaviour and the max stable factor on this machine (measure it — do not assume).
  - Documented instance-isolation scheme (ports, `PX4_GZ_MODEL_POSE`, `ROS_DOMAIN_ID`) for parallel SITL later.
- **Tech** — PX4 SITL, gz-sim 8, gz-transport.
- **Validation** — Vehicle arms and holds altitude in headless mode; RTF measured and logged; two SITL instances run concurrently without port or DDS collisions.

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
- **Tech** — uXRCE-DDS, `px4_msgs`, rclpy.
- **Validation** — A Python node commands takeoff → 5 m hover → land entirely over ROS 2; measured PX4→ROS 2 telemetry latency documented (expect single-digit ms).

---

### Phase 3 — Autonomous flight baseline
**Effort: ~1 week**

- **Goal** — A repeatable autonomous mission that will serve as every experiment's nominal condition.
- **Deliverables**
  - Mission executor node: takeoff → waypoint sequence → hover → land, in offboard mode.
  - Mission definition in YAML (`configs/missions/`), starting with one primary mission (recommend a square or figure-8 circuit, 20–40 m, ~60 s).
  - Episode reset that returns SITL to a clean state without restarting the process where possible; documented fallback if a full restart is required.
  - Episode logger writing one row per control step + one summary row per episode.
- **Tech** — PX4 offboard mode, rclpy, YAML.
- **Validation** — 20 consecutive healthy missions with 100% success; position RMSE variance across seeds documented as the noise floor for later comparisons. **This noise floor is a required number — every later result is measured against it.**

---

### Phase 4 — Telemetry / state pipeline
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

### Phase 5 — Fault injection framework (+ labelled dataset)
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
  visible, repeatable signature in the Phase 4 features; the same seed reproduces
  the same trajectory; PX4's own `FailureDetector` does **not** trigger at the
  severities we target (confirming they are sub-threshold and therefore worth detecting).

> **Design decision D2 (§14).** Partial degradation is *not* supported by PX4's
> `failure` command, which is binary. Achieving graded severity requires acting
> at the Gazebo motor model instead. Recommended: scale the rotor's thrust
> coefficient via gz-transport at runtime. This is physically faithful and, critically,
> **PX4 has no knowledge of it** — preserving the detection problem.

---

### Phase 6 — AI fault detection
**Effort: ~2 weeks**

- **Goal** — A learned detector producing fault presence, type, and severity from the telemetry window.
- **Deliverables**
  - Offline-trained model on the Phase 5 dataset, with strict train/val/test
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

### Phase 7 — Rule-based fault recovery baseline
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

### Phase 8 — High-level RL fault recovery
**Effort: ~3–4 weeks — highest risk phase**

- **Goal** — An RL policy that consumes the fault estimate and outputs high-level commands, trained in the PX4-in-the-loop environment.
- **Deliverables**
  - Gymnasium environment wrapping the full stack (§7.1).
  - Observation, action, and reward specification frozen and documented before training starts (§7.2–7.3).
  - PPO training pipeline with parallel SITL instances, checkpointing, TensorBoard logging.
  - Domain randomisation over fault severity, onset, mass/inertia, wind, sensor noise.
  - Trained policy checkpoints + training curves, ≥3 seeds.
- **Tech** — Stable-Baselines3 PPO, `SubprocVecEnv`, PyTorch, CUDA.
- **Validation** — Policy exceeds the Phase 7 rule-based baseline on mission
  success rate and crash rate at matched fault severities, with non-overlapping
  confidence intervals across ≥3 training seeds. **A policy that merely
  ties the baseline is a legitimate finding — report it rather than tuning until it wins.**

---

### Phase 9 — Evaluation and paper-quality experiments
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

### Phase 10 — Generalization and robustness
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

### Phase 11 — Hexacopter extension
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

### Phase 12 — Research paper / publication package
**Effort: ~3 weeks**

- **Goal** — Submittable paper and an artifact others can run.
- **Deliverables** — Manuscript; complete reproducibility appendix; tagged repo
  release; one-command reproduction script; archived trained models, configs,
  seeds and raw results; short demo video.
- **Validation** — A clean machine, following `docs/reproduce.md` only,
  regenerates the headline result.

---

### Parallel track — Web dashboard (any time after Phase 4)
See §10. Not a research dependency; do not let it block Phases 5–9.

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
support roughly **4 parallel SITL instances**, not 32. Mitigations, in order of
importance:

1. **Low decision rate (5 Hz)** — a 90 s episode is ~450 steps, not 22,500. This
   alone makes the problem tractable and is the main reason for the high-level design.
2. **Headless** — no GUI during training, ever.
3. **`PX4_SIM_SPEED_FACTOR`** — measure the maximum stable factor in Phase 1
   (expect 3–8×) and budget from the measured number.
4. **4 parallel instances** via `SubprocVecEnv`, each with its own ports and `ROS_DOMAIN_ID`.
5. **Short episodes** — terminate early and decisively on crash.

Order-of-magnitude target: **1–3 M environment steps**, reachable in roughly
2–5 days of wall-clock training. If Phase 1 measurements show this is
unreachable, the fallback is a reduced-order quadrotor model for policy
pre-training with PX4-in-the-loop fine-tuning — **deferred by default**, since it
adds a second dynamics model and a sim-to-sim gap. Revisit only if forced.

---

## 8. Experiment methodology

The four-condition comparison, all sharing identical missions, seeds, and PX4 configuration:

| # | Condition | Fault injected | Detection | Recovery |
|---|---|---|---|---|
| C1 | Healthy baseline | No | — | — |
| C2 | Faulty, no recovery | Yes | — | PX4 default only |
| C3 | Detection + rule-based | Yes | AI detector | FSM (Phase 7) |
| C4 | Detection + RL | Yes | AI detector | RL policy (Phase 8) |

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

**Robustness** — every metric above, reported across the generalization axes in Phase 10.

All metrics computed by one shared module (`experiments/metrics.py`) so that
detector, rule-based, and RL conditions are never measured by different code.

---

## 10. Dashboard concept (later, parallel track)

Read-only live monitoring. **Not on the research critical path — do not build it before Phase 9.**

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

## 11. Repository / module responsibilities

```
aero-safe-rl/
├── planning.md              # this document
├── README.md                # what it is, how to run it
├── environment.yml          # pinned conda env spec (env name: aero-safe-rl)
├── configs/                 # ALL experiment configuration (YAML)
│   ├── env/                 #   simulation + episode settings
│   ├── faults/              #   fault definitions and sweeps
│   ├── missions/            #   waypoint missions
│   ├── rl/                  #   algorithm + reward hyperparameters
│   ├── detector/            #   model + feature configuration
│   └── experiments/         #   full experiment matrices
├── simulation/              # PX4/Gazebo layer
│   ├── launch/              #   SITL startup, parallel-instance management
│   ├── models/              #   custom SDF (hex, modified x500)
│   ├── airframes/           #   PX4 airframe files
│   ├── faults/              #   fault injection backends
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
│   ├── run_matrix.py        #   batch runner
│   ├── metrics.py           #   THE single metrics implementation
│   └── analysis/            #   figure and table generation
├── dashboard/               # later: backend/ + frontend/
├── scripts/                 # env_report.sh, sim_start.sh, utilities
├── tests/                   # unit + integration tests
├── results/                 # raw logs, trained artifacts (git-ignored, except manifests)
└── docs/                    # environment.md, reproduce.md, design notes, paper drafts
```

**Boundaries that matter:**
- `ai/features/` is imported by `rl/` and `dashboard/`. Feature extraction is
  defined exactly once.
- `experiments/metrics.py` is the only place a metric is computed.
- `simulation/` knows nothing about learning; `ai/` and `rl/` know nothing about Gazebo.
- The recovery interface (`rl/policies/`) is identical for FSM and RL policies —
  they are swappable behind one API. This is what makes C3 vs C4 a fair comparison.

---

## 12. Definition of Done — major phases

| Phase | Done when |
|---|---|
| **0** | `scripts/env_report.sh` emits complete version JSON; CUDA verified inside the `aero-safe-rl` conda env; PX4 pinned tag builds; `px4_msgs` matched to that tag; `docs/environment.md` written |
| **1** | Headless SITL arms, hovers, lands from a script; measured RTF and max stable speed factor documented; 2 concurrent instances verified |
| **2** | Takeoff→hover→land driven entirely from a ROS 2 Python node; telemetry latency measured; all required topics confirmed carrying valid data |
| **3** | 20/20 healthy missions succeed; position RMSE noise floor across seeds documented |
| **4** | Feature vector logged for a full healthy mission with no gaps; replay determinism verified; normalisation stats frozen |
| **5** | Graded severity reproducibly injected and visible in features; identical seed → identical trajectory; PX4 `FailureDetector` confirmed silent at target severities; ≥500-episode labelled dataset generated |
| **6** | Detector beats threshold and classical baselines on held-out **episodes**; per-severity ROC and latency distribution reported; online inference < 20 ms |
| **7** | FSM measurably beats no-recovery across the severity sweep; tuning sweep archived |
| **8** | PPO policy beats the tuned FSM on success and crash rate with non-overlapping CIs over ≥3 seeds — **or** the null result is documented with evidence |
| **9** | Full condition matrix (C1–C6) executed; all figures/tables regenerate from raw logs by one command; RQ3 ablation complete |
| **10** | Generalization table complete, including honest failure-mode analysis |
| **11** | Hexacopter flies healthy mission and closes the fault→detect→recover loop |
| **12** | Clean machine reproduces the headline result from `docs/reproduce.md` alone |

---

## 13. Immediate next steps

In order. Do not begin Phase 1 until Phase 0's validation passes.

1. ✅ **Decisions D1–D5 resolved** (§14) — all approved.
2. ✅ **Repo renamed and initialised** (`aero-safe-rf` → `aero-safe-rl`), `git init` done.
3. ✅ **Miniconda installed**, empty conda env `aero-safe-rl` (Python 3.10) created.
4. **Add `.gitignore`** (`results/`, `build/`, `install/`, `log/`, `*.pt`, `__pycache__/`) and make the first commit.
5. **Create the directory scaffold**: `configs/ simulation/ ros2_ws/ ai/ rl/ experiments/ scripts/ tests/ results/ docs/`.
6. **Install Gazebo Harmonic** from the OSRF apt repository alongside Classic 11; verify `gz sim --versions` reports 8.x.
7. **Install packages into the `aero-safe-rl` conda env**: PyTorch with CUDA, Gymnasium, Stable-Baselines3, NumPy, SciPy, pandas, PyYAML, matplotlib; freeze to `environment.yml`; **verify GPU access**.
8. **Pin PX4**: create a project branch from the chosen tag (`v1.17.0`) in `~/projects/PX4-Autopilot`, run `Tools/setup/ubuntu.sh`, update submodules, and build `make px4_sitl`.
9. **Build the bridge**: `Micro-XRCE-DDS-Agent`, plus `px4_msgs` and `px4_ros_com` at commits matching the pinned PX4 tag.
10. **Write `docs/environment.md` and `scripts/env_report.sh`**, then run the Phase 0 validation checklist.

---

## 14. Decisions — ALL APPROVED (2026-08-14)

| ID | Decision | Resolution |
|---|---|---|
| **D1** | PX4 version to pin | ✅ **`v1.17.0`** — newest stable, closest to the current `main` checkout, full Gazebo Harmonic support, `failure` command present. |
| **D2** | Partial-degradation injection mechanism | ✅ **Gazebo-side rotor thrust scaling**, implemented as a **project-owned gz-sim system plugin** (`RotorDegradationSystem`) living in `simulation/gz_plugins/`, not in the PX4 tree. See the note below. |
| **D3** | Repo directory name | ✅ Rename `aero-safe-rf` → **`aero-safe-rl`**. |
| **D4** | Conditions C5 (oracle detection) and C6 (detector ablation) | ✅ **Included** in the evaluation matrix. |
| **D5** | Reduced-order pre-training model | ✅ **Deferred.** Revisit only if M1 throughput measurements prove PX4-in-the-loop training unreachable. |

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

---

## Appendix — Known risks

| Risk | Severity | Mitigation |
|---|---|---|
| RL training too slow on this hardware | **High** | Low decision rate, headless, speed factor, 4 parallel envs; D5 fallback |
| `px4_msgs` / PX4 version mismatch | **High** | Pin both together in Phase 0; verify field values, not just topic presence |
| PX4's own failure handling masks our contribution | **High** | Target sub-threshold severities; verify `FailureDetector` stays silent (Phase 5 validation); frame contribution at the mission layer |
| Weak rule-based baseline undermines the paper | **High** | Documented tuning sweep, archived as evidence |
| SITL non-determinism blocks reproducibility | Medium | Lockstep mode; report distributions over seeds rather than single runs |
| Episode reset requires full SITL restart | Medium | Measure reset cost in Phase 3; budget it into the sample plan |
| 8 GB VRAM limits model size | Low | Models here are small (1D-CNN/GRU, MLP policy); VRAM is not the bottleneck — wall-clock simulation is |
| Scope creep (dashboard, hexacopter, extra faults) | Medium | Phases 10–11 and the dashboard are explicitly gated behind a complete Phase 9 |
```
