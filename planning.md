# Autonomous UAV AI Fault Detection and Fault-Tolerant Control using Reinforcement Learning

**Planning document — roadmap only. No implementation.**

Status: Phases 0–1, 1b, 3, 3b, 4, 5, 6 and 7 done; Phase 2 substantially done
with one open reliability item (`docs/parallelism.md` §2.6). Phase 4's
evaluation farm passed a 400-episode soak at 2 workers (`docs/throughput.md`);
Phase 6 delivered the 750-episode labelled fault dataset
(`docs/fault_dataset.md`); Phase 7 delivered the fault detector and the error
model Phase 8b needs (`docs/detector_results.md`). Next: Phase 8 (rule-based
recovery baseline). An active-fault-diagnosis extension was
proposed and deferred until after the MVP (`docs/change_active_diagnosis.md`).

**Revised 2026-09-21 — the simulator strategy changed.** RL training moves to a
GPU-parallel **NVIDIA Isaac Lab** environment; PX4-in-the-loop (Gazebo) remains
the evaluation stack and the source of every reported number. This reverses
**D6** and supersedes **D5**; the new decision is **D12** (§14). It adds
**RQ5** (§2) and a thirteenth development principle (§4). Nothing measured so
far is invalidated — the Phase 3 noise floor, the divergence band and the
multi-instance findings all still stand, and the ROS 2 ↔ PX4 layer is untouched.

**Revised 2026-08-20** after a multi-instance review that read the pinned PX4
source and ran two instances side by side. It found four structural problems in
how parallel simulation was planned, all of which produce workers that start
cleanly and then silently do nothing. The roadmap now contains a dedicated
**Phase 4 — parallel simulation farm**, decisions **D7–D11**, and a
reproducibility standard that matches what this stack can actually deliver.
Evidence: `docs/parallelism.md`. Coding rules: `CLAUDE.md`.

Companion documents: `milestones.md` (build order), `CLAUDE.md` (coding rules),
`docs/parallelism.md` (verified multi-instance behaviour).
Last updated: 2026-09-23

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
- **RQ5 (Sim-to-sim transfer)** — Does a high-level recovery policy trained in
  a massively-parallel *reduced-order* simulator transfer to a full
  autopilot-in-the-loop stack, and what is lost in the crossing? (§3.1, D12)

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
4. **Honest sim-to-sim accounting (RQ5)** — a great deal of published drone RL
   is trained in a GPU-parallel reduced-order simulator and evaluated in that
   same simulator, leaving the transfer question unasked. Training in Isaac Lab
   and reporting *only* PX4-in-the-loop numbers turns that unasked question
   into a measured one. The gap we report is a contribution, not an apology.

Keep the low-level controller fixed (PX4) in all conditions so the comparison
isolates the contribution.

**Why this survives the simulator change.** Points 1–3 all depend on PX4 being
in the loop — the sub-threshold framing is defined against *PX4's* detector,
and the hardware-transferability claim rests on the policy speaking PX4's
command interface. That is precisely why evaluation stays on PX4 and only
training moves (D12). A version of this project that trained *and* evaluated in
Isaac would forfeit all three.

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

### 3.1 Two simulators, one policy (D12)

Training and evaluation run in different simulators, on purpose.

```
        TRAIN                                 EVALUATE  /  REPORT
┌───────────────────────────┐        ┌────────────────────────────────┐
│ Isaac Lab (env: isaacsim) │        │ PX4 SITL + Gazebo (aero-safe-rl)│
│ N parallel quadrotors,GPU │        │ the real autopilot, 8.3× RTF    │
│ reduced-order dynamics +  │        │ the stack Phases 1–4 already    │
│ geometric position ctrl   │        │ built, entirely unchanged       │
│ PPO, millions of steps    │        │ hundreds of episodes            │
└─────────────┬─────────────┘        └───────────────▲────────────────┘
              │                                       │
              └──── policy.pt + obs/action spec ──────┘
                    + frozen normalisation stats
                    (files only — never a shared import)
```

**Why.** The project's own §7.4 names sample budget as risk #1: PX4-in-the-loop
gives roughly 8× real-time on 3–4 workers, which makes 1–3 M steps a multi-day
job and a single bad hyperparameter an expensive mistake. A GPU-parallel
reduced-order environment removes that constraint outright. Meanwhile
evaluation needs fidelity, not volume — a few hundred episodes per cell — which
is exactly what the existing stack is good at.

**What makes it plausible.** The action space is high-level (velocity scale,
altitude offset, mission pacing, land-commit) at 5 Hz, and the thing that
differs most between the two simulators is the low-level controller we
deliberately never touch. A motor-level policy would have no chance of
crossing this gap; a mission-level one has a real one. That is an argument, not
a guarantee — **RQ5 exists to measure it rather than assume it.**

**The two constraints this imposes**, both of which must hold from the start
rather than be discovered at training time:

1. **The policy observation must be computable in both simulators.** Anything
   PX4-specific — control-allocation residual, EKF innovations — may feed the
   *detector*, which runs only on the PX4 side, but may not enter the policy's
   observation vector. `configs/rl/observation_v1.yaml` is where this is
   enforced, and a test checks both sides produce the same dimensions.
2. **The fault model exists twice and must mean the same thing twice.** A C++
   gz-sim plugin and an Isaac-side Python rotor model, validated against each
   other for matching thrust reduction at matched severity. This is the one
   sanctioned exception to "exactly one implementation" (`CLAUDE.md` §1.4),
   and §1.6 is the price of it.

**If RQ5 comes out badly** — the policy trains well in Isaac and transfers
poorly — that is a reportable result, not a failed project, and it is a more
interesting one than most papers in this area publish. The fallback is
PX4-in-the-loop fine-tuning of an Isaac-pretrained policy, which is the
original D5 proposal and is now cheap because the pretraining already happened.

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
13. **Every reported number comes from the PX4-in-the-loop stack.** Isaac is
    where the policy is trained, never where a result is measured. The single
    exception is the training curve, which is labelled as an Isaac-side
    diagnostic. The two environments exchange files and never import each
    other (`CLAUDE.md` §0.1).

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

Effort estimates below are rough and are for sequencing, not commitments.
**Updated target (2026-08-21):** roughly a week of active engineering to build
all of Phase 5-13's code — this is faster, AI-assisted development, not the
part-time-human pace the original per-phase estimates assumed. Phase 9's RL
training run and Phase 10's evaluation sweep are separate, unattended,
wall-clock-bound jobs (hours to multiple days, per this project's own measured
throughput) — they are expected to run longer than the week, in the
background, and that is fine; only the engineering effort is targeted at a
week. See `milestones.md`'s M13 timeline note for the full framing.

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

### Phase 3b — Isaac Lab feasibility spike ✅
**Effort: ~1 day — added 2026-09-21; gates everything D12 depends on**

- **Goal** — Find out, by measurement, whether Isaac Lab runs usefully on this
  machine before any part of the plan is built on the assumption that it does.
- **Why it is a phase** — Isaac Sim 5.1's stated minimum is an RTX 4080 / 16 GB
  VRAM / 32 GB RAM; this machine is an RTX 2070 Mobile (Turing, 8 GB) with
  15 GB RAM. Turing has RT cores so it is not excluded outright, and a headless
  physics-only workload is far cheaper than the full application — but "far
  cheaper" is not a number. D12 without this measurement is a guess.
- **Result: passed, comfortably.** Full numbers and method:
  `docs/isaac_feasibility.md`. VRAM was never near the 8 GB ceiling at any
  scale tested (peak 6.8 GB at 32,768 parallel envs); **host RAM turned out to
  be this machine's real constraint** (peak 13.6/15.8 GB at 32,768). Chosen
  operating point — 8,192 envs — delivers 546k env-steps/s at 41% VRAM / 41%
  RAM, with no memory growth over a 10-minute sustained run. The gate (below)
  clears in under 2 seconds of simulated time. Sample budget, §7.4's old #1
  risk, is resolved rather than merely mitigated.
- **Deliverables**
  - ✅ Isaac Sim 5.1.0 (pip-installed in the `isaacsim` env) launches
    **headless**, with no rendering, and steps a physics scene.
  - ✅ Isaac Lab installed (cloned to `~/projects/IsaacLab`, sibling to
    `PX4-Autopilot` — outside this repo, per this project's existing
    convention for vendored dependencies); the stock `Isaac-Quadcopter-Direct-v0`
    task run.
  - ✅ **Measured table**, widened beyond the originally planned {64, 256,
    1024, 4096} to {..., 8192, 16384, 32768} once throughput kept climbing
    past 4096: steps/sec, VRAM, host RAM at each. Largest count run without
    error: 32,768 (burst only). Largest count stability-tested for 10 minutes:
    8,192 — the chosen operating point, deliberately smaller than the largest
    working count because it leaves headroom for a real training loop's
    network/optimizer/buffer, which this spike does not include.
  - ✅ Written to `docs/isaac_feasibility.md` with the chosen operating point.
- **One real blocker hit and fixed**: Isaac Sim's first import prompts an
  interactive EULA acceptance that hangs forever non-interactively. Fixed with
  `OMNI_KIT_ACCEPT_EULA=YES`; needs a permanent home in an Isaac counterpart to
  `scripts/activate.sh` before the next session hits the same hang.

---

### Phase 4 — Parallel evaluation farm
**Effort: ~1 week — rescoped by D12; see the note below**

> **Rescoped 2026-09-21.** This phase was originally sized as a *training*
> farm — millions of steps, 4 workers, with a gate deciding whether Phase 9 was
> possible at all. Under D12 training moved to Isaac, so this is now an
> **evaluation** farm: hundreds of episodes per condition cell, not millions of
> steps. Consequences: 2 workers is a reasonable default instead of 4; the
> Phase 9 throughput gate moves to Phase 3b; and the unresolved
> `offboard_control_signal_lost` issue (`docs/parallelism.md` §2.6, ~35–65 % at
> two concurrent workers) becomes a retry-and-record case that this phase's
> `WorkerSupervisor` already handles by design, rather than a threat to a
> multi-day training run. The throughput table is still worth producing — the
> evaluation sweep is budgeted from it — but it is no longer a project gate.

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
  - **Measured throughput table** across worker count ∈ {1,2,3,4} × speed ∈
    {1,2,4,8}, always one drone per world (D7): aggregate simulated-seconds per
    wall-second, episodes per hour, per-worker RTF stdev, peak RSS, CPU
    utilisation. Written to `docs/throughput.md` with the chosen operating
    point stated.
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

> The **thrust-setpoint vs achieved-acceleration residual** was expected to be
> the single most informative feature for actuator degradation — a degraded
> rotor forces the allocator to command more while delivering less. It is in v1
> (`thrust_accel_residual`).
>
> **Measured in Phase 7: on its own it is not.** As a threshold detector it
> scored tick AUROC 0.40. The informative signal is per-rotor: the degraded
> rotor's motor command rises above the other three. The residual stays in the
> feature vector as one of the detector's inputs (`docs/detector_results.md`).

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

### Phase 7 — AI fault detection ✅
**Effort: ~2 weeks — done 2026-09-23**

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
- **Tech** — PyTorch. The plan recommended a 1D-CNN or small GRU. What was built
  is a **rotor-symmetric streaming GRU with a 5-member deep ensemble**, chosen
  over a 1D-CNN after a look at the dataset:
  - **Streaming:** it carries its memory across the whole flight, which helps
    during the onset transient and slow ramps. A windowed CNN sees only the
    last 1.5 s.
  - **Rotor-symmetric:** the telemetry is re-expressed from each rotor's point
    of view using PX4's x500 geometry, and one shared network scores each
    rotor. This quadruples the effective data and gives rotor identification
    directly.
  - **Ensemble:** member disagreement is the uncertainty output.
  A Transformer is not justified at this data scale.
- **Validation** — Per-severity AUROC, detection rate and **detection delay
  distribution** (fault onset → first alarm sustained 0.5 s), false alarms per
  healthy flight-hour, rotor-ID accuracy, severity MAE and calibration. All
  are measured on held-out test episodes, with every detector's threshold set
  by the same rule on validation data. Online inference < 20 ms.

  **Exit criterion as revised.** The original "must beat both baselines"
  proved the wrong bar: a settled fault is near-trivially separable from
  motor-command imbalance, so every good detector sits near the AUROC
  ceiling. The criterion became: better detection delay on weak faults and
  better severity error than the best baseline, **or** the shortfall reported
  as the result, with no baseline retuned downward.
- **Outcome** (details: `docs/detector_results.md`; build record:
  `milestones.md` M7) — against the strongest baseline, a random forest:
  - **Weak faults (s 0.2–0.4):** faster detection, median 0.88 s vs 1.18 s.
  - **Rotor ID:** 98.6–99.8% accurate vs 93–96%.
  - **Severity MAE:** 0.013–0.022 vs up to 0.116.
  - **Calibration:** ECE 0.004.
  - **Tick AUROC:** a tie (0.998 vs 0.997).
  - **False alarms:** slightly more (6 vs 4 short events in 0.69 healthy
    hours).

  Online: at most 10.7 ms p99 per tick, verified live on two concurrent
  workers.

---

### Phase 8 — Rule-based fault recovery baseline
**Effort: ~1 week**

- **Goal** — A strong, honestly-tuned classical baseline. **Do not shortchange this.**
- **Deliverables**
  - Finite state machine: `NOMINAL → SUSPECTED → CONFIRMED → RECOVERING → LANDED/ABORTED`.
  - Hand-designed responses: reduce max velocity, lower altitude ceiling,
    reduce aggressiveness, hold position, divert to nearest safe point, controlled descent.
  - Hysteresis and debounce on detector output. Phase 7 measured the
    detector's false alarms as short: every one lasted ≤ 0.9 s and fell around
    waypoint turns. A 1–2 s confirmation hold is longer than all of them
    (`docs/detector_results.md`).
  - Thresholds tuned via a documented sweep — not guessed.
- **Tech** — Python FSM, same command interface the RL policy will use.
- **Validation** — Measurably improves mission success and crash rate over the
  no-recovery condition across the fault severity sweep. The tuning sweep is
  archived so reviewers can see the baseline was given a fair chance.

---

### Phase 8b — Isaac Lab training environment
**Effort: ~1 week — added 2026-09-21 (D12)**

- **Goal** — The environment the policy trains in: N parallel quadrotors on
  GPU, implementing the *same* frozen observation/action spec as the
  PX4-in-the-loop evaluation environment.
- **Deliverables**
  - An Isaac Lab task: quadrotor with a geometric position/velocity controller
    standing in for PX4's position loop, stepped at the same 5 Hz decision rate.
  - The **Isaac-side rotor degradation model**, matching the Gazebo plugin's
    severity semantics, with a cross-validation test against a recorded fixture
    (`CLAUDE.md` §1.6). This is the milestone's real risk, not the RL part.
  - A **detector-output stub** in the observation: during Isaac training the
    policy cannot run the real detector (which needs PX4 telemetry), so it is
    fed a *simulated* detector output — true severity passed through a
    calibrated noise/latency/false-positive model fitted to the Phase 7
    detector's measured error characteristics. This is the honest way to keep
    principle #12 (no ground truth as input) while training off-stack, and the
    fit must be documented, since RQ3 and RQ5 both lean on it. The measured
    error model is `results/m7_detector_v1/error_model.json`. Its main
    feature: the detector's output is near-binary, so what matters is *when* it
    switches, not the noise on its level. Model it as a detection-delay
    distribution plus rare short false alarms, not as Gaussian noise on the
    true severity.
  - Both environments asserted against `observation_v1.yaml` by one shared test.
- **Precondition** — Phase 3b's measurement, Phase 7's detector error model.
- **Validation** — The two environments agree dimension-for-dimension on the
  observation spec; a hand-written scripted policy (e.g. "always slow down")
  produces qualitatively similar outcomes in both; the fault model matches.

---

### Phase 9 — High-level RL fault recovery
**Effort: ~2 weeks — reduced by D12; risk moved to Phase 8b and RQ5**

- **Goal** — An RL policy that consumes the fault estimate and outputs
  high-level commands, **trained in Isaac Lab (Phase 8b) and evaluated in the
  PX4-in-the-loop stack**.
- **Deliverables**
  - Observation, action and reward specs frozen as versioned YAML **before the
    first training run**, with a test asserting *both* environments' spaces match.
  - PPO training pipeline in the `isaacsim` env, checkpointing, resumable,
    TensorBoard logging with **reward components logged separately** so reward
    hacking is visible rather than indistinguishable from learning.
  - Domain randomisation over fault severity, onset, mass/inertia, wind, sensor
    noise — and, newly important under D12, over the *simulated detector's*
    latency and error, since that is the main thing the policy could overfit to.
  - Trained policy checkpoints + training curves, ≥3 seeds. Checkpoints record
    the spec digest they were trained under (`CLAUDE.md` §0.1).
  - **The RQ5 transfer table**: each policy's performance in Isaac and on the
    PX4 stack, side by side. The gap is a headline result of this project.
- **Tech** — Isaac Lab + PPO (`rsl_rl` or `skrl`), PyTorch, CUDA. Note this
  replaces the previous Stable-Baselines3 `SubprocVecEnv` plan, which existed
  only to parallelise PX4 workers and has no purpose now.
- **Precondition** — `docs/isaac_feasibility.md` (Phase 3b) shows the sample
  budget is reachable. Budget from that measurement, never from an assumption.
- **Validation** — Policy exceeds the Phase 8 rule-based baseline **on the PX4
  stack** on mission success rate and crash rate at matched fault severities,
  with non-overlapping confidence intervals across ≥3 training seeds. **A policy
  that merely ties the baseline is a legitimate finding — report it rather than
  tuning until it wins.** A policy that beats it in Isaac and not on PX4 is the
  RQ5 result and is reported as such, not quietly retuned.

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

**Two environments implementing one spec.** Step rate **5 Hz** and episodes of
60–90 s simulated time in both, terminated early on crash, geofence breach,
landing, or mission completion.

| | Training env | Evaluation env |
|---|---|---|
| Backend | Isaac Lab, N parallel quadrotors on GPU | PX4 SITL + Gazebo + ROS 2 |
| Conda env | `isaacsim` (py3.11) | `aero-safe-rl` (py3.10) |
| Low-level control | geometric position/velocity controller | PX4 (EKF2, allocation, rate loops) |
| Built on | new, Phase 8b | a thin wrapper over Phase 4's `EpisodeRunner` |
| Used for | PPO training only | every reported number |

The evaluation env must not re-implement flying, reset or logging — it wraps
`EpisodeRunner`. The training env is necessarily a separate implementation and
is the reason `CLAUDE.md` §0.1 exists.

**The specs are frozen once and shared.** `configs/rl/observation_v1.yaml` and
`action_v1.yaml` are authored before either environment is built, and a test
asserts both environments expose spaces matching them. Two environments that
disagree about the observation layout produce a policy that appears to train
and then behaves randomly at evaluation — and it looks exactly like a
sim-to-sim transfer failure, which is how it would be misdiagnosed.

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

### 7.4 Sample budget — resolved by D12, at the cost of a new risk

**This was the project's #1 risk and is no longer.** Training moved to a
GPU-parallel Isaac Lab environment (§3.1, D12), where the constraint is GPU
memory rather than wall-clock simulation, and 1–3 M steps is hours rather than
days. The risk did not vanish so much as change shape: it is now **RQ5, the
sim-to-sim gap**, which is at least measurable, falsifiable, and interesting
enough to publish either way.

Two budgets now exist and must not be confused:

| | Where | Scale | Bounded by |
|---|---|---|---|
| **Training** | Isaac Lab, `isaacsim` env | 1–3 M+ steps | GPU memory, env count |
| **Evaluation** | PX4 + Gazebo, `aero-safe-rl` env | ~100 episodes × cell | wall clock, worker count |

The Isaac side's env count on this hardware (RTX 2070 Mobile, 8 GB) is
**unmeasured and must be measured before the plan depends on it** — that is
Phase 3b's only job, and it is a gate in the same sense Phase 4's throughput
table is. Isaac Sim 5.1's stated minimum is an RTX 4080 / 16 GB, so this
machine is below spec and a headless physics-only workload is the plausible
case, not a certain one.

The evaluation-side budget is much smaller than the old training budget, which
is what makes Phase 4 easier than originally scoped (see that phase's note).

#### The old analysis, retained — it still governs the evaluation side

PX4 SITL is slow, and this machine (12 threads, 16 GB) will
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

**Do not budget from an assumption.** The tempting arithmetic — 4 workers × 8×
= 32× aggregate — is not supported by any measurement, and is almost certainly
wrong: each worker now runs its own Gazebo physics server, and gz-sim physics is
effectively single-threaded per world, so four workers contend for cores and
per-worker RTF falls. Phase 4 measures aggregate throughput directly across
N × speed-factor and picks the operating point. **The evaluation sweep is
budgeted from that one number and nothing else.**

*(Superseded: this section previously named a reduced-order pre-training model
as a deferred fallback under D5, rejected for adding a second dynamics model
and a sim-to-sim gap. D12 adopts exactly that approach as the primary path —
the second dynamics model is now the Isaac Lab environment, and the sim-to-sim
gap is now RQ5. The cost was always real; what changed is that it buys a
tractable sample budget and a research question rather than only the former.)*

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

**All six conditions are executed on the PX4-in-the-loop stack** (principle
#13). C4/C5/C6's policies are trained in Isaac, but no condition is *measured*
there. The one Isaac-side table in the paper is the **RQ5 transfer table** —
each policy's Isaac performance next to its PX4 performance — which is a
separate result about the method's construction, not part of the C1–C6
comparison. Keeping these apart is what stops "trained in Isaac" from becoming
an unstated advantage for C4 over the C3 baseline.

---

## 9. Evaluation metrics

**Detection** (implemented in Phase 7; protocol in `experiments/metrics.py`)
- Tick-level ROC-AUC and episode detection rate — reported *per severity level*
  and per onset profile
- **Detection delay** — onset → first alarm sustained 0.5 s (median, p90)
- False alarms per healthy flight-hour, and share of healthy flights with any
  alarm
- Rotor-identification accuracy and severity estimation error (MAE, bias)
- Calibration (ECE) and whether the uncertainty output predicts errors

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

## 10. Dashboard (deferred — out of scope unless explicitly requested)

A read-only live-monitoring web UI was considered as an optional, non-blocking
parallel track (PX4 → ROS 2 → FastAPI → WebSocket → a simple frontend). It is
not a research dependency and no phase needs it. Cut from the active plan to
keep the project's surface area small; revisit only if actually wanted later,
and design it then, against whatever the pipeline looks like at that point.

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
│   ├── features/            #   feature extraction (shared with rl/ and isaac/)
│   └── detector/            #   dataset + split, baselines, model, train, evaluate,
│                            #   online runtime (checkpoint in results/m7_detector_v1/)
├── rl/                      # reinforcement learning — PX4 side (aero-safe-rl env)
│   ├── envs/                #   evaluation Gym env: thin wrapper over EpisodeRunner
│   ├── rewards/             #   reward functions (shared spec with isaac/)
│   ├── policies/            #   rule-based FSM + RL policy wrappers
│   ├── eval.py              #   loads a checkpoint, runs the PX4 stack
│   └── checkpoints/         #   archived trained policies (+ spec digests)
├── isaac/                   # TRAINING side — isaacsim env (py3.11). NEVER imports
│   ├── envs/                #   rl/, simulation/, aero_bridge or rclpy. CLAUDE.md §0.1
│   │                        #   Isaac Lab quadrotor task + geometric controller
│   ├── faults/              #   Isaac-side rotor degradation (matches the gz plugin)
│   ├── detector_model/      #   calibrated detector-output simulator (Phase 8b)
│   └── train.py             #   PPO; writes checkpoints rl/ later reads
├── experiments/             # orchestration + analysis
│   ├── episode_runner.py    #   THE single "fly one episode" implementation
│   ├── worker_supervisor.py #   one worker's processes + health
│   ├── sim_farm.py          #   N supervisors, restart policy, teardown
│   ├── run_manifest.py      #   provenance for every run
│   ├── episode_schema.py    #   episode record loader + validator
│   ├── run_matrix.py        #   batch runner
│   ├── metrics.py           #   THE single metrics implementation
│   └── analysis/            #   figure and table generation
├── scripts/                 # env_report.sh, sim_start.sh, utilities
├── tests/                   # unit (default), sim/ (@sim), slow/ (@slow)
│   └── fixtures/            #   recorded telemetry — most tests need no sim
├── results/                 # raw logs, trained artifacts (git-ignored, except manifests)
└── docs/                    # environment.md, parallelism.md, throughput.md,
                             # simulation_notes.md, reproduce.md, paper drafts
```

**Boundaries that matter:**
- `ai/features/` is imported by `rl/` and, for the shared subset, by `isaac/`.
  Feature extraction is defined exactly once, and imports no ROS — which is what
  lets Phase 7 iterate offline in seconds and what lets the Isaac side reuse it
  without dragging in the ROS 2 stack.
- `experiments/metrics.py` is the only place a metric is computed.
- `experiments/episode_runner.py` is the only place an episode is flown. The
  Gymnasium env, the dataset generator and the evaluation harness are callers.
- `simulation/instance_spec.py` is the only place instance identity is derived,
  and both the Python and shell layers read it.
- `simulation/` knows nothing about learning; `ai/` and `rl/` know nothing about Gazebo.
- The recovery interface (`rl/policies/`) is identical for FSM and RL policies —
  they are swappable behind one API. This is what makes C3 vs C4 a fair comparison.
- **`isaac/` and everything else are in different conda environments and never
  import each other** (`CLAUDE.md` §0.1). They communicate through three files:
  the frozen observation/action spec, the frozen normalisation statistics, and
  a policy checkpoint stamped with the spec digest it was trained under. This
  is the boundary that, if violated, silently invalidates every RQ5 number.

---

## 12. Definition of Done — major phases

| Phase | Done when |
|---|---|
| **0** | ✅ `scripts/env_report.sh` emits complete version JSON; CUDA verified inside the `aero-safe-rl` conda env; PX4 pinned tag builds; `px4_msgs` matched to that tag; `docs/environment.md` written |
| **1** | ✅ Headless SITL arms, hovers, lands from a script; measured RTF and max stable speed factor documented (~8×, compute-bound, see `docs/simulation_notes.md`); 2 concurrent instances verified |
| **1b** | One worker starts, stops and restarts independently; `pgrep -cf "^gz sim "` equals the instance count; identity read from `instance_<N>.json` |
| **2** | Takeoff→hover→land driven entirely from a ROS 2 Python node, **on instance 1 as well as 0**; telemetry latency measured; all required topics confirmed carrying valid data; no wall-clock sleeps in flight logic |
| **3** | 20/20 healthy missions succeed; position RMSE noise floor documented; run-to-run divergence band at fixed seed measured; all three reset tiers implemented and costed; episode schema frozen and validated |
| **3b** | Isaac Lab runs headless on this GPU; env-count × steps/sec × VRAM table measured; `docs/isaac_feasibility.md` states the operating point **or** states that D12 is not viable here |
| **4** | 2+ workers × 100 episodes unattended, zero orphans, flat memory; single-worker restart proven not to disturb siblings; `docs/throughput.md` states the chosen operating point |
| **5** | Feature vector logged for a full healthy mission with no gaps; replay determinism verified bitwise; causality test passes; normalisation stats frozen |
| **6** | Graded severity injected and **confirmed applied**, visible in features above the Phase 3 noise floor; PX4 `FailureDetector` confirmed silent at target severities; ≥500-episode labelled dataset generated; PX4 tree unmodified |
| **7** | ✅ Detector evaluated against threshold and classical baselines on held-out **episodes**, per severity: better detection delay and severity error than the best baseline (tick AUROC a tie, false alarms slightly worse — reported, not tuned away); per-severity ROC and delay distribution reported; online inference < 20 ms verified live on 2 workers. See `docs/detector_results.md` |
| **8** | FSM measurably beats no-recovery across the severity sweep; tuning sweep archived; shared policy interface in place |
| **8b** | Both environments assert against one frozen `observation_v1.yaml`; Isaac and Gazebo fault models validated to match at matched severity; detector-output simulator fitted to Phase 7's measured error and documented |
| **9** | Obs/action/reward specs frozen and version-stamped before training; PPO policy beats the tuned FSM **on the PX4 stack** on success and crash rate with non-overlapping CIs over ≥3 seeds — **or** the null result is documented with evidence; the RQ5 Isaac-vs-PX4 transfer table exists |
| **10** | Full condition matrix (C1–C6) executed; all figures/tables regenerate from raw logs by one command; RQ3 ablation complete; invalid episodes accounted for explicitly |
| **11** | Generalization table complete, including honest failure-mode analysis |
| **12** | Hexacopter flies healthy mission and closes the fault→detect→recover loop |
| **13** | Clean machine reproduces the headline result from `docs/reproduce.md` alone, within the Phase 3 divergence band |

---

## 13. Immediate next steps

The list below is the original Phase 0–1 setup log, kept as a record. Current
work is under **Open now**.

1. ✅ **Decisions D1–D6 resolved** (§14) — all approved. *(D5 and D6 were later superseded by D12 on 2026-09-21.)*
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

Phases 3b, 4, 5, 6 and 7 are **done** (see `milestones.md`'s progress log).
What remains for the MVP:

1. **Phase 8** — the rule-based recovery baseline, consuming the Phase 7
   detector through `ai/detector/runtime.py`.
2. **Phase 8b** — the Isaac Lab training environment. Its detector-output
   simulator is fitted to Phase 7's `results/m7_detector_v1/error_model.json`.
3. **Phases 9 → 10** — RL training in Isaac, evaluation on PX4, full
   experiments.

**Do not start Phase 8b or 9 until `docs/isaac_feasibility.md` exists and says
the sample budget is reachable.** The old gate on `docs/throughput.md` now
governs the evaluation sweep rather than training.

---

## 14. Decisions — ALL APPROVED (2026-08-14)

| ID | Decision | Resolution |
|---|---|---|
| **D1** | PX4 version to pin | ✅ **`v1.17.0`** — newest stable, closest to the current `main` checkout, full Gazebo Harmonic support, `failure` command present. |
| **D2** | Partial-degradation injection mechanism | ✅ **Gazebo-side rotor thrust scaling**, implemented as a **project-owned gz-sim system plugin** (`RotorDegradationSystem`) living in `simulation/gz_plugins/`, not in the PX4 tree. See the note below. |
| **D3** | Repo directory name | ✅ Rename `aero-safe-rf` → **`aero-safe-rl`**. |
| **D4** | Conditions C5 (oracle detection) and C6 (detector ablation) | ✅ **Included** in the evaluation matrix. |
| **D5** | Reduced-order pre-training model | ⛔ **SUPERSEDED by D12 (2026-09-21).** Was: deferred. Now adopted as the primary training path, realised as Isaac Lab. |
| **D6** | Isaac Sim as the simulator backend | ⛔ **SUPERSEDED by D12 (2026-09-21).** Was: declined. Now partially reversed — Isaac enters as the *training* simulator, not as a replacement for Gazebo. |

### Added 2026-08-20 after the multi-instance review

| ID | Decision | Resolution |
|---|---|---|
| **D7** | Simulator topology for parallel runs | ✅ **One drone per world, always**, `GZ_PARTITION`-isolated (settled 2026-08-20). Silent, unchosen sharing (PX4's default) stays prohibited. |
| **D8** | Who owns the Gazebo server process | ✅ **We do** — `PX4_GZ_STANDALONE=1`, server started and PID-tracked by our launcher, so one worker can be restarted without touching its siblings. |
| **D9** | Instance identity | ✅ **Uniform, no special case for instance 0.** `PX4_UXRCE_DDS_NS=px4_<N>` for every N; `target_system = N+1` always; identity computed once in `simulation/instance_spec.py` and published as `instance_<N>.json`. |
| **D10** | Timing source in flight logic | ✅ **Simulated time only, sourced from `GzSimClock`** (Gazebo's native clock over gz-transport) — not `px4_msgs` timestamps, which M2 measured to track wall clock almost exactly regardless of speed factor (`uxrce_dds_client`'s session-level resync). Wall clock is permitted solely in the hang watchdog. See `docs/parallelism.md` §2.5. |
| **D11** | Reproducibility standard | ✅ **Statistical, not bitwise** — see §7.5. Pure functions of recorded data are bitwise reproducible; whole-pipeline results are reproducible within a divergence band measured in Phase 3. |

### Added 2026-09-21 — the simulator strategy

| ID | Decision | Resolution |
|---|---|---|
| **D12** | Where the RL policy is trained, and where results are measured | ✅ **Hybrid: train in Isaac Lab, evaluate in PX4-in-the-loop.** Supersedes D5 and D6. See §3.1 and the note below. |

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

### Note on D12 — Isaac Lab as the training simulator (2026-09-21)

Supersedes D5 and D6. Three options were weighed: a **full swap** of Gazebo for
Isaac Sim with PX4 still in the loop (via Pegasus Simulator); **Isaac Lab
only**, dropping PX4 entirely; and the **hybrid** that was chosen. Full detail
of the architecture is in §3.1.

Full swap was rejected on throughput: Pegasus documents running *below*
real-time at its default 250 Hz physics with a sensor suite, against a measured
8.3× for Gazebo on this machine, and each Isaac worker is a multi-GB process,
so 3–4 parallel workers collapse to one. Isaac Lab only was rejected because it
removes PX4 from the loop, and with it the sub-threshold framing, the
fixed-low-level-controller comparison, and the hardware-transferability claim —
i.e. all three novelty arguments in §2.

**Which of D6's original objections survive:**

| D6's objection (2026-08-15) | Status under D12 |
|---|---|
| PX4 v1.17.0 has no native Isaac SITL target | **Still true, now irrelevant** — under D12 Isaac never talks to PX4. No bridge is needed. |
| Isaac's minimum GPU is above this machine's RTX 2070 | **Still true, and the main open risk.** Phase 3b measures it before anything depends on it. Headless physics-only is the cheapest possible Isaac workload, which is what makes it plausible at all. |
| GPU parallelism doesn't help, since we run one PX4 per vehicle | **Resolved** — the Isaac side has no PX4, so the parallelism applies fully. This was the objection that D12's shape exists to answer. |
| Gates reproducibility behind an NVIDIA account | **Partially stands, and constrains Phase 13.** Mitigation: the headline result is an *evaluation* result, reproducible on the PX4/Gazebo stack alone from an archived checkpoint. Only *retraining* needs Isaac. `docs/reproduce.md` must state both paths separately and the checkpoint must be archived, or this objection becomes real again. |

The honest summary: D6 was correctly argued for the question it was asked
("should Isaac replace Gazebo?"). D12 asks a different question and gets a
different answer. The one objection that was never about scope — the GPU — is
the one that still has to be settled empirically.

---

## Appendix — Known risks

| Risk | Severity | Mitigation |
|---|---|---|
| RL training too slow on this hardware | ~~High~~ **Low** | **Largely resolved by D12** — training moved to GPU-parallel Isaac Lab. Residual risk is now the row below. |
| Isaac Sim will not run usefully on an RTX 2070 / 8 GB / 15 GB RAM | **High** | The machine is below Isaac Sim 5.1's stated minimum. **Phase 3b measures it before anything depends on it**, headless and physics-only being the cheapest workload. Fallbacks if it fails: cloud GPU for training runs, or revert to PX4-in-the-loop training under the old Phase 4 budget. |
| Policy trains in Isaac and does not transfer to PX4 (RQ5 negative) | **Medium** | Mitigated by the high-level 5 Hz action space, which keeps the differing low-level controller out of the policy's job. Not eliminated — this is why RQ5 is a measured question. A negative result is reportable; the fallback is PX4-in-the-loop fine-tuning of the pretrained policy. |
| The two fault models silently diverge | **High** | They are one contract with two backends (`CLAUDE.md` §1.6); a cross-validation test against a recorded fixture asserts matching thrust reduction at matched severity. Without it, training and evaluation use different faults and every transfer number is meaningless. |
| Observation spec drifts between the two environments | **High** | One frozen `observation_v1.yaml`, one shared test asserting both environments' spaces; checkpoints carry the spec digest and evaluation refuses a mismatch. Failure here is indistinguishable from a transfer failure, which is what makes it dangerous. |
| Isaac dependency weakens Phase 13 reproducibility | Medium | Headline results are *evaluation* results, reproducible on the PX4/Gazebo stack alone from an archived checkpoint; only retraining needs Isaac. `docs/reproduce.md` states the two paths separately. |
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
| 8 GB VRAM limits model size | Low | Models here are small (the Phase 7 detector is ~10k parameters per ensemble member and trains in ~2 min; MLP policy); VRAM is not the bottleneck — wall-clock simulation is |
| Scope creep (dashboard, hexacopter, extra faults) | Medium | Phases 11–12 and the dashboard are explicitly gated behind a complete Phase 10 |
```
