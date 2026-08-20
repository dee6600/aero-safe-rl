# Implementation Milestones

**Companion to `planning.md`.** That file explains *what* we are building and
*why*. This file is the build order: what to do, in what sequence, and how to
know each step actually works.

Status: M0 done (2026-08-14). Next up: M1.

---

## How to use this file

- Work through milestones **in order**. Each one depends on the one before it.
- Do not start the next milestone until the current one's **"Done when"** checks
  all pass. Skipping a check means you will debug it later, mixed in with three
  other new problems, which costs far more time.
- Each milestone lists **files to create**. Do not create files that are not
  listed. Extra structure now is guesswork; we add it when we need it.
- When something does not work, check the **"Watch out for"** section first —
  these are the failures this stack actually produces.

### A note on the two hard milestones

M5 (fault injection) and M8 (RL training) are where this project can stall.
M5 needs C++ work; M8 needs patience and compute. Everything before M5 is
plumbing that should go smoothly. Plan your time accordingly.

---

## Locked decisions

All approved on 2026-08-14. These are settled — do not revisit them mid-build.

| ID | Decision |
|---|---|
| D1 | PX4 pinned to **`v1.17.0`** |
| D2 | Partial rotor faults via **our own gz-sim plugin** (`RotorDegradationSystem`) |
| D3 | Repo renamed to **`aero-safe-rl`** |
| D4 | Evaluation includes **C5** (perfect-detector upper bound) and **C6** (detector ablation) |
| D5 | Simplified pre-training model **deferred** — revisit only if M1 shows training is impossible |
| D6 | Simulator backend: **Gazebo Harmonic stays primary** for M1–M12; Isaac Sim added only as an optional, non-blocking parallel learning track |

**Why D2 changed shape.** The original idea was to change the motor's strength
setting live over Gazebo's messaging system. On inspection, Gazebo's stock motor
plugin does not allow that — its strength is fixed when the model loads. PX4's
`main` branch has a motor-failure plugin, but it only does full failure (motor
completely dead), and it does not exist in v1.17.0 anyway.

So we write our own small Gazebo plugin. It sits inside the physics loop and
multiplies one rotor's thrust by an efficiency number between 0 and 1 that we
can change at any moment from outside. This is the honest way to simulate a
weakening motor: **PX4 never learns about it**, it only feels the aircraft
behaving oddly — which is exactly the situation our detector must handle.

**Why D6 exists.** Isaac Sim was considered as a full replacement for Gazebo
(2026-08-15), partly to learn the tool. Declined for the research pipeline:
Isaac Sim 5.1's stated minimum GPU is an RTX 4080 16GB, this machine's RTX
2070 (8GB) sits below it, and M8's design deliberately runs a full separate
PX4 process per parallel instance (so PX4 never sees the fault) — which
conflicts with Isaac Sim's single-GPU-per-instance guidance and would likely
collapse 4 planned parallel instances down to 1. It also weakens M12's
reproducibility story (NVIDIA account + heavy GPU vs. free/apt-installable
Gazebo). Full reasoning in `planning.md` §14 (D6) and §10b. Isaac Sim +
Pegasus Simulator (a mature, actively-maintained third-party PX4 bridge for
it) is being set up as a separate learning track — see below — and must never
gate M5–M12.

---

## Milestone overview

| # | Milestone | Est. | Risk |
|---|---|---|---|
| M0 | Environment setup and pinning | 1 wk | Low |
| M1 | PX4 + Gazebo simulator running | 3 d | Low |
| M2 | ROS 2 talks to PX4 | 4 d | **Medium** |
| M3 | Autonomous mission baseline | 1 wk | Low |
| M4 | Telemetry feature pipeline | 4 d | Low |
| M5 | Fault injection + dataset | 1.5 wk | **High** |
| M6 | AI fault detector | 2 wk | Medium |
| M7 | Rule-based recovery baseline | 1 wk | Low |
| M8 | RL recovery policy | 3–4 wk | **High** |
| M9 | Full experiments + results | 2 wk | Medium |
| M10 | Generalization tests | 2 wk | Low |
| M11 | Hexacopter extension | 2 wk | Medium |
| M12 | Paper + reproducibility package | 3 wk | Low |

**First real result** (something worth showing anyone) arrives at the end of
**M6**: "our detector spots a weakening motor that PX4 itself does not notice."
**First publishable result** arrives at the end of **M9**.

---

# M0 — Environment setup and pinning

**Goal:** a working, recorded, repeatable toolchain, and an actual git repository.

**Why it matters:** three things the project brief assumed are not true on this
machine — Gazebo Harmonic is missing, PyTorch is missing, and the repository is
an empty folder. Also, once we start producing results, changing any tool
version invalidates them. We pin now so we never have to re-run experiments.

### Tasks

1. ✅ **Rename and initialise the repository** — done
   - `~/projects/aero-safe-rf` → `~/projects/aero-safe-rl`
   - `git init`, `.gitignore` created
   - First commit still pending — do it once the scaffold below exists
2. ✅ **Set up conda** — done
   - Miniconda installed at `~/miniconda3`, initialised for bash
   - Empty conda env `aero-safe-rl` created (Python 3.10.20, no packages yet)
   - Activate with `conda activate aero-safe-rl` before any Python work from here on
3. ✅ **Create the folder structure** (empty folders with `.gitkeep`) — done
   `configs/ simulation/ ros2_ws/src/ ai/ rl/ experiments/ scripts/ tests/ results/ docs/`
   (`dashboard/` is not created yet — it is not needed until much later)
4. ✅ **Install Gazebo Harmonic** from the OSRF apt repository — done, 8.15.0.
   Installed alongside Gazebo Classic 11; did not remove it.
5. ✅ **Install packages into the `aero-safe-rl` conda env** — done
   - `conda activate aero-safe-rl`
   - PyTorch 2.13+cu126, Gymnasium, Stable-Baselines3, NumPy, SciPy, pandas,
     PyYAML, matplotlib, TensorBoard installed; CUDA verified on the RTX 2070
   - Frozen to `environment.yml` (`conda env export`)
6. ✅ **Pin PX4** — done
   - Branch `aero-safe-rl` created from tag `v1.17.0` in `~/projects/PX4-Autopilot`
   - `Tools/setup/ubuntu.sh` run, submodules updated, `make px4_sitl` builds clean
7. ✅ **Build the ROS 2 bridge** — done
   - `Micro-XRCE-DDS-Agent` built (installed to `~/.local`, see `docs/environment.md`)
   - `px4_msgs` (`release/1.17`) and `px4_ros_com` (`main`) vendored into
     `ros2_ws/src/`, `colcon build` succeeds — 236 px4_msgs interfaces visible
8. ✅ **Record the environment** — done
   - `scripts/env_report.sh` prints every version as JSON
   - `docs/environment.md` written from its output

### Files created

```
.gitignore  environment.yml
scripts/env_report.sh
docs/environment.md
```

`.gitignore` must include: `results/`, `build/`, `install/`, `log/`,
`__pycache__/`, `*.pt`, `*.ulg`. (The conda env itself lives outside the repo,
at `~/miniconda3/envs/aero-safe-rl`, so it never needs ignoring.)

### Done when

- [x] `scripts/env_report.sh` prints complete version info with no blanks
- [x] `conda activate aero-safe-rl && python -c "import torch; print(torch.cuda.is_available())"` → `True`
- [x] `nvidia-smi` shows the RTX 2070 and the Python process can use it
- [x] `gz sim --versions` reports **8.x** (Harmonic) — 8.15.0
- [x] `cd ~/projects/PX4-Autopilot && git describe --tags` → **`v1.17.0`**
- [x] `make px4_sitl` finishes with no errors
- [x] `ros2 interface list | grep px4_msgs` returns messages — 236 interfaces
- [x] Everything committed to git

### Watch out for

- **`px4_msgs` version mismatch is the single most common way this stack breaks.**
  If the message definitions do not match the PX4 build, topics still appear and
  still tick — but the numbers inside are garbage. Always verify actual values,
  not just that a topic exists.
- Do **not** `apt remove` Gazebo Classic. Nothing requires removing it, and
  other things on this machine may depend on it.
- **Always `conda activate aero-safe-rl` before installing or running anything
  Python-related.** Never `pip install` into the `base` conda env or system
  Python — packages belong only in the project env.
- ROS 2 (`colcon`, `rclpy`) is installed system-wide via `/opt/ros/humble`, not
  through conda. When working with ROS 2, you'll typically `source
  /opt/ros/humble/setup.bash` *and* have the `aero-safe-rl` conda env active at
  the same time — conda supplies the ML packages, ROS supplies the middleware.
  Watch for the two Pythons disagreeing; if `colcon build` picks up the wrong
  interpreter, pin it explicitly.

---

# M1 — PX4 + Gazebo simulator running

**Goal:** start the simulator from a script, with no GUI, and measure how fast
it actually runs.

**Why it matters:** the speed measured here decides whether M8 (RL training) is
realistic. We need this number early, not after we have built everything else.

### Tasks

1. ✅ **Write `scripts/sim_start.sh`** — done. Launches PX4 SITL + Gazebo
   headless directly (env vars matching what the `make ... gz_x500` target
   does under the hood), with configurable instance number, world, speed
   factor, spawn pose, and model
2. ✅ **Measure the real-time factor** — done, at requested 1/2/4/8/16×
   (sampled from Gazebo's own `/world/<world>/stats` topic, not estimated)
3. ✅ **Find the highest stable speed factor** — done: ~8× (compute-bound
   ceiling on this machine; flight itself never became unstable at any
   tested factor — see `docs/simulation_notes.md`)
4. ✅ **Confirm two simulator instances run at once without clashing** — done
   (separate ports, separate model positions; `ROS_DOMAIN_ID` isolation is
   an M2 concern, not applicable yet since ROS 2 isn't wired in until M2)
5. ✅ **Write `docs/simulation_notes.md`** — done

### Files created

```
scripts/sim_start.sh
scripts/sim_stop.sh
docs/simulation_notes.md
```

### Done when

- [x] One command starts a headless simulation; the drone arms, takes off,
      hovers at 5 m, and lands
- [x] Real-time factor measured and recorded at each speed factor
- [x] Maximum stable speed factor identified and written in `docs/simulation_notes.md`
      (~8×; this machine is compute-bound there, requesting 16× doesn't exceed it —
      see `docs/simulation_notes.md` for the full breakdown)
- [x] Two instances run at the same time with no port or messaging conflicts
- [x] `scripts/sim_stop.sh` cleanly kills everything (no orphan processes left)

### Watch out for

- Headless means **no GUI at all** during training. The GUI can consume more
  compute than the physics.
- Orphan `px4` and `gz sim` processes accumulate and quietly steal CPU. Always
  stop through the script.
- **If the maximum stable speed factor is below about 4, tell me before
  continuing.** That changes the M8 plan and may trigger decision D5.

---

# M2 — ROS 2 talks to PX4

**Goal:** read telemetry from PX4 and send commands to it, entirely from Python.

**Why it matters:** this is the road every later component drives on. If it is
shaky, everything above it is shaky.

### Tasks

1. Create ROS 2 package `aero_bridge` in `ros2_ws/src/`
2. Start `MicroXRCEAgent` alongside the simulator (add it to `sim_start.sh`)
3. Confirm we receive and can read these topics **with sensible values**:
   `VehicleOdometry`, `VehicleAttitude`, `SensorCombined`, `ActuatorMotors`,
   `ActuatorOutputs`, `VehicleStatus`, `BatteryStatus`, `FailsafeFlags`,
   `EstimatorStatusFlags`
4. Confirm we can publish: `OffboardControlMode`, `TrajectorySetpoint`, `VehicleCommand`
5. Write a small node that flies takeoff → hover 5 m → land, over ROS 2 only
6. Write `scripts/measure_latency.py` to measure the PX4 → ROS 2 delay

### Files created

```
ros2_ws/src/aero_bridge/     (package: setup.py, package.xml)
ros2_ws/src/aero_bridge/aero_bridge/px4_interface.py
ros2_ws/src/aero_bridge/aero_bridge/test_flight.py
scripts/measure_latency.py
```

### Done when

- [ ] All listed topics appear **and carry believable numbers** (attitude changes
      when the drone tilts; motor outputs rise on takeoff)
- [ ] A Python node flies takeoff → hover → land with no manual steps
- [ ] Telemetry delay measured and recorded (expect a few milliseconds)
- [ ] The flight works reliably 5 times in a row

### Watch out for

- **Offboard mode requires a steady stream of setpoints before you switch into
  it, and continuously afterwards.** If the stream stutters, PX4 drops out of
  offboard. Send at 20 Hz or faster.
- If topics appear but values look wrong or frozen, suspect the `px4_msgs`
  version first (see M0).
- PX4 uses NED coordinates (north-east-**down**). Down is positive, so altitude
  is a negative number. This causes sign-error bugs constantly — write it in a
  comment at the top of the interface file.

---

# M3 — Autonomous mission baseline

**Goal:** a repeatable autonomous mission that becomes the "healthy" condition
in every experiment.

**Why it matters:** every later result is measured against this. We also need to
know how much the results wobble run-to-run *with no fault at all* — that is our
noise floor. If a faulty run differs by less than the noise floor, we have found
nothing.

### Tasks

1. Mission executor node: takeoff → fly waypoints → hover → land, in offboard mode
2. Define missions in YAML. Start with **one** primary mission — a square or
   figure-8 circuit, 20–40 m across, about 60 seconds
3. Build episode reset: return the simulator to a clean start state. Try to do
   this without restarting PX4; if a restart is unavoidable, **measure how long
   it takes** — that cost multiplies across millions of RL steps
4. Episode logger: one row per control step, plus one summary row per episode,
   written to `results/`
5. Run 20 healthy missions and analyse the spread

### Files created

```
configs/missions/square_circuit.yaml
ros2_ws/src/aero_bridge/aero_bridge/mission_executor.py
ros2_ws/src/aero_bridge/aero_bridge/episode_logger.py
experiments/run_episodes.py
docs/baseline_results.md
```

### Done when

- [ ] 20 out of 20 healthy missions complete successfully
- [ ] Position tracking error (RMSE) recorded for all 20 runs
- [ ] **Noise floor documented**: the spread of that error across seeds
- [ ] Episode reset works reliably; its time cost is measured and recorded
- [ ] Logs are written in a consistent format with no missing rows

### Watch out for

- If any of the 20 healthy runs fails, stop and fix it. An unreliable baseline
  makes every later comparison meaningless.
- Reset is easy to get subtly wrong — leftover state from the previous episode
  (EKF bias, integrator windup, drift in position) leaks into the next one and
  quietly corrupts training. Verify the drone truly starts fresh each time.

---

# M4 — Telemetry feature pipeline

**Goal:** turn raw telemetry into one fixed-size list of numbers, produced 10
times per second, used identically by the detector, the RL policy, and later the
dashboard.

**Why it matters:** if the detector and the policy each compute features their
own way, they will drift apart and the comparison in M9 becomes invalid. One
implementation, used everywhere.

### Tasks

1. `FeatureExtractor` class: takes a sliding window of telemetry (start with 1–2
   seconds) and outputs a fixed-length vector at 10 Hz
2. Include at minimum:
   - attitude, angular rates, estimated angular acceleration
   - position and velocity error against the current setpoint
   - each motor's normalised output
   - **commanded thrust versus achieved acceleration (the residual)**
   - control allocation residual
   - EKF innovation values
   - battery current, vibration metrics
3. Compute normalisation statistics from healthy flights, then **freeze them to
   disk**. Never recompute them later.
4. `configs/features.yaml` with an explicit `feature_version` string

### Files created

```
ai/features/extractor.py
ai/features/normalization.json     (frozen statistics)
configs/features.yaml
tests/test_features.py
```

### Done when

- [ ] Feature vector logged across a full healthy mission with no gaps and no NaNs
- [ ] Replaying the same recorded flight produces **exactly** the same vectors
- [ ] Normalisation statistics computed from healthy data and frozen
- [ ] `feature_version` recorded in every log file

### Watch out for

- **The thrust-versus-acceleration residual is probably the most important
  feature in the whole project.** A weakening motor forces PX4 to command more
  thrust while the aircraft accelerates less. Make sure it is correct.
- Never recompute normalisation statistics after training starts. Doing so
  silently invalidates every trained model.
- Anything using a *future* value inside the window is a bug — the real drone
  cannot see the future. Windows look backwards only.

---

# M5 — Fault injection and dataset

**Goal:** inject a rotor fault of any chosen strength, at any chosen moment,
repeatably — and produce the labelled dataset the detector learns from.

**Why it matters:** this is the foundation of the research. If faults are not
controllable and repeatable, nothing after this point is science.

**This is the hardest engineering milestone. Budget accordingly.**

### Tasks

1. **Write the Gazebo plugin** `RotorDegradationSystem` (C++, ~250–350 lines)
   - Use PX4's `MotorFailureSystem`
     (`src/modules/simulation/gz_plugins/motor_failure/`, on `main`) as a
     structural reference — it is only 344 lines and shows the correct pattern
   - Holds one efficiency value per rotor, from 0 (dead) to 1 (healthy)
   - Listens on a Gazebo topic for updates, so severity can change mid-flight
   - Multiplies that rotor's thrust and torque contribution each physics step
   - Lives in **our** repo, loaded via `GZ_SIM_SYSTEM_PLUGIN_PATH`
2. Copy the x500 model into `simulation/models/` and add our plugin to the SDF.
   **Do not edit anything inside `~/projects/PX4-Autopilot`.**
3. Python control interface: `inject_fault(rotor, severity, profile, onset_time)`
   with step, ramp, and intermittent profiles
4. Fault configuration files, with severity sampled from a **seeded** random
   generator so runs are reproducible
5. Write ground-truth fault labels into every episode log next to the features
6. Generate the dataset: **500–1000 episodes**, mixing healthy and faulty runs
   across the full severity range

### Files created

```
simulation/gz_plugins/rotor_degradation/    (C++ source + CMakeLists.txt)
simulation/models/x500_aero/model.sdf
simulation/faults/injector.py
configs/faults/rotor_degradation.yaml
experiments/generate_dataset.py
```

### Suggested build order

Do it in two steps — get the loop closed before making it precise:

- **Step A:** binary failure only (efficiency 0 or 1). Proves the plugin loads,
  receives messages, and affects flight.
- **Step B:** add graded severity, ramps, and intermittent profiles.

### Done when

- [ ] Plugin loads without errors and responds to Gazebo topic messages
- [ ] Commanding 40% loss on rotor 2 at t = 20 s produces a **clear, repeatable
      signature** in the M4 features
- [ ] The same seed produces the same trajectory
- [ ] **PX4's own `FailureDetector` stays silent** at our target severities —
      confirming the fault is genuinely hidden from PX4 and therefore worth detecting
- [ ] Dataset of 500+ labelled episodes generated, balanced across severities
- [ ] `~/projects/PX4-Autopilot` has **zero** uncommitted modifications

### Watch out for

- **Plugin ordering in the SDF matters.** Ours must be declared *after* the
  motor model plugin, or the motor model overwrites our changes each step.
  PX4's plugin README states this explicitly.
- If PX4's `FailureDetector` *does* trigger, our severities are too high. Lower
  them. The interesting research zone is faults PX4 cannot see.
- Do not use PX4's `failure motor N off` command for the main experiments. It is
  binary, and it is internal to PX4, which makes the fault partly self-announcing.
- Keep every fault parameter in YAML. You will regenerate this dataset more than
  once, and you must be able to say exactly how.

---

# M6 — AI fault detector

**Goal:** a model that reads the telemetry window and reports whether a fault is
present, which one, and how severe.

**Why it matters:** this is the first genuinely novel result. Target the region
where PX4 is blind.

### Tasks

1. Split the dataset **by episode, never by time step**
2. Train the simple comparison baselines first: a threshold on the residual, and
   a random forest on the same features
3. Train the main model — start with a **1D convolutional network or a small
   GRU**. A Transformer is not justified at this data size.
4. Model outputs: probability of fault, fault class, severity estimate, and an
   **uncertainty measure** (the recovery policy needs to know when to distrust it)
5. Wrap it in a ROS 2 node running live at 10 Hz
6. Produce the evaluation report: accuracy per severity, ROC curves, and the
   **detection delay** distribution

### Files created

```
ai/models/detector.py
ai/train.py    ai/evaluate.py
ai/baselines/threshold.py    ai/baselines/random_forest.py
ros2_ws/src/aero_bridge/aero_bridge/detector_node.py
configs/detector/cnn_v1.yaml
results/detector/report.md
```

### Done when

- [ ] Splits are by episode — verified, not assumed
- [ ] Main model beats **both** simple baselines on held-out data
- [ ] Accuracy reported **separately for each severity level** (a single average
      hides everything interesting)
- [ ] Detection delay measured: time from fault start to first sustained alarm
      (mean, median, 95th percentile)
- [ ] False alarm rate on healthy flights, per minute of flight
- [ ] Live inference runs under 20 ms per step

### Watch out for

- **Splitting by time step instead of episode leaks data catastrophically.**
  Overlapping windows from the same flight end up in both training and test sets,
  and accuracy looks superb but means nothing. This is the classic mistake in
  this kind of work.
- Report results per severity. Strong faults are easy; weak ones are the point.
- A model that is 99% accurate but takes 4 seconds to notice may be useless in
  flight. Delay matters as much as accuracy.

---

# M7 — Rule-based recovery baseline

**Goal:** a sensible, well-tuned, non-learning recovery system.

**Why it matters:** this is what the RL policy must beat. If we build a weak
baseline and then declare victory, reviewers will see through it immediately and
the paper is dead. Build this one honestly.

### Tasks

1. State machine: `NORMAL → SUSPECTED → CONFIRMED → RECOVERING → LANDED/ABORTED`
2. Responses: cap horizontal speed, cap climb rate, lower the altitude ceiling,
   fly less aggressively, hold position, divert to a safe point, descend under control
3. Add hysteresis and debouncing so a flickering detector does not cause thrashing
4. **Tune the thresholds with a documented sweep.** Save the sweep results.
5. Expose it through the **same interface** the RL policy will use

### Files created

```
rl/policies/base_policy.py        (shared interface — RL uses this too)
rl/policies/rule_based.py
configs/recovery/rule_based.yaml
results/recovery/tuning_sweep.md
```

### Done when

- [ ] Clearly beats the no-recovery condition across the severity range
- [ ] Tuning sweep saved as evidence that it was given a fair chance
- [ ] Uses the identical command interface the RL policy will use
- [ ] Behaves sensibly on false alarms (does not panic and land a healthy drone)

### Watch out for

- Resist the temptation to under-tune this so RL looks better. A strong baseline
  makes an RL win *credible*; a weak one makes it worthless.
- The shared interface is what makes the M9 comparison fair. Build it here, and
  do not let the RL policy quietly gain extra powers later.

---

# M8 — RL recovery policy

**Goal:** train a policy that takes the detector's estimate and chooses
high-level actions that keep the mission alive.

**Why it matters:** the core contribution. Also the milestone most likely to
consume time, so the design is deliberately kept small.

### Tasks

1. Build the Gymnasium environment wrapping the whole stack, stepping at **5 Hz**
2. **Freeze the observation, action, and reward design before training starts**
   (full specification in `planning.md` §7.2) — and write it down
3. Set up parallel training: **4 simulator instances** with separate ports and
   `ROS_DOMAIN_ID`s
4. Train with PPO. Add randomisation across fault severity, timing, aircraft
   mass, wind, and sensor noise
5. Train **at least 3 separate seeds** — a single run proves nothing
6. Log everything to TensorBoard, checkpoint often

### Files created

```
rl/envs/uav_fault_env.py
rl/rewards/mission_reward.py
rl/policies/rl_policy.py
rl/train.py    rl/evaluate.py
configs/rl/ppo_v1.yaml
configs/env/train_env.yaml
```

### Key design points (from `planning.md`, repeated because they are easy to lose)

- The policy sees the **detector's estimate**, never the true fault state. True
  state may shape the reward during training, but must never be an input.
- Actions are high-level only: speed limits, altitude offset, mission pacing,
  and a decision to commit to landing. **Never motor commands.**
- Give **partial credit for a safe landing**. Without it, the policy learns to
  gamble on completing the mission instead of protecting the aircraft.
- Rough target: **1–3 million environment steps**, which should be a few days of
  wall-clock training.

### Done when

- [ ] Environment passes a random-action smoke test without crashing or hanging
- [ ] Training runs stably for 3+ seeds
- [ ] Learning curves show real improvement, not noise
- [ ] Policy beats the M7 rule-based baseline on mission success and crash rate,
      with **non-overlapping confidence intervals** across seeds
- [ ] Checkpoints and full configs saved

### Watch out for

- **If training is too slow, fix it by shortening episodes and ending failed
  ones early — not by raising the decision rate.** The 5 Hz rate is what makes
  the problem small enough to learn.
- If PPO plateaus, check the reward first. Reward bugs look exactly like
  learning failures. Only after a real tuning effort should SAC be considered,
  and then it must be reported as an extra comparison, not a silent swap.
- **A result showing RL only ties the rule-based baseline is a legitimate
  finding — report it.** Tuning endlessly until RL wins is how projects lose
  their integrity, and reviewers usually notice.
- Watch RAM. Four simulator instances plus training on a 16 GB machine is tight.

---

# M9 — Full experiments and results

**Goal:** run the complete comparison and produce the paper's figures and tables.

### Tasks

1. Batch runner for the full matrix:

   | | Condition | Fault | Detection | Recovery |
   |---|---|---|---|---|
   | C1 | Healthy | No | — | — |
   | C2 | Faulty, no recovery | Yes | — | PX4 default only |
   | C3 | Detection + rules | Yes | AI | State machine |
   | C4 | Detection + RL | Yes | AI | RL policy |
   | C5 | **Perfect detection + RL** | Yes | True state | RL policy |
   | C6 | **RL, detector disabled** | Yes | None | RL policy |

2. Severity sweep: 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0
3. **100+ episodes per cell**, using held-out seeds never seen in training
4. Statistics: means, 95% confidence intervals, significance tests, effect sizes
5. The RQ3 study: artificially add delay and false alarms to the detector, and
   measure how recovery degrades
6. All figures generated by script — **never hand-edited**

### Files created

```
experiments/run_matrix.py
experiments/metrics.py          (the ONE place metrics are computed)
experiments/analysis/figures.py
configs/experiments/main_comparison.yaml
results/main/                   (raw logs + generated figures)
```

### Done when

- [ ] All six conditions run across all severities
- [ ] Every figure and table regenerates from raw logs with **one command**
- [ ] Confidence intervals reported everywhere — no bare averages
- [ ] The detection-delay study (RQ3) complete
- [ ] Re-running with the same seeds reproduces the same numbers

### Watch out for

- C5 versus C4 is the most informative comparison in the paper: it separates
  "our detector is imperfect" from "our policy is imperfect". Reviewers always
  ask this. We will already have the answer.
- All conditions must use identical PX4 parameters. Any difference is a
  confound that invalidates the comparison.

---

# M10 — Generalization tests

**Goal:** find out where the approach works and where it breaks. **No retraining
— evaluation configs only.**

Test on: unseen severities (both between and beyond the training range), unseen
fault timings, unseen wind, changed mass and battery, unseen missions and
starting positions, and the hardest case — multiple simultaneous faults or a
fault type never trained on.

### Done when

- [ ] Generalization table complete across all axes
- [ ] Failure modes described honestly, in plain terms

**Watch out for:** the instinct to hide poor generalization. A clear statement of
*where the method stops working* is worth more to reviewers than a table of
uniform success, which mostly reads as untested.

---

# M11 — Hexacopter extension

**Goal:** test whether the method transfers to an aircraft with spare rotors.

**Important:** PX4 has **no Gazebo hexacopter model** — only a simplified-physics
one and a JSBSim one. We must build the model and airframe ourselves. This is
real work, not a configuration flag.

### Tasks

1. Build a hexacopter SDF model, including our degradation plugin
2. Create the PX4 airframe file with correct six-rotor mixing
3. Retrain or fine-tune the detector for six rotors
4. Evaluate the policy with no retraining, then after fine-tuning

### Done when

- [ ] Hexacopter flies the baseline mission when healthy
- [ ] The full fault → detect → recover loop works on it

**Scientific note:** a hexacopter has spare rotors, so PX4 alone already handles
losing one far better than a quadcopter does. Our method's advantage should
*shrink*. Measuring that shrinking margin is a genuinely good result — report it
as a finding, not a disappointment.

---

# M12 — Paper and reproducibility package

**Goal:** a submittable paper and a package someone else can actually run.

### Tasks

1. Write the manuscript around RQ1–RQ4
2. `docs/reproduce.md` — complete instructions from a clean machine
3. One-command reproduction script for the headline result
4. Archive trained models, configs, seeds, and raw results
5. Tag a release; record a short demo video

### Done when

- [ ] A clean machine reproduces the headline result following `docs/reproduce.md` alone
- [ ] Every number in the paper traces back to a file in `results/`
- [ ] Repository tagged and archived

---

## Parallel track — Web dashboard

Can start any time after **M4**. It is not required for any research result, so
it must never delay M5–M9.

Simple design: FastAPI backend with a ROS 2 node → WebSocket → React frontend.
Shows position, attitude, velocity, battery, per-motor health, detected fault and
severity, recovery state, mission progress, and telemetry plots.

Build the **replay mode** (playing back a saved episode) before the live mode —
it is more useful for paper figures and the demo video, and it does not require
a running simulator to develop against.

No database, no login, no containers.

---

## Parallel track — Isaac Sim (learning, optional)

Can start any time — no dependency on any other milestone. Not required for
any research result, so it must never delay M5–M9. See `planning.md` §10b
and §14 (D6) for the full reasoning behind keeping this off the main path.

Stack: Isaac Sim (pip install, its own conda env, kept separate from
`aero-safe-rl`) + Pegasus Simulator's PX4 MAVLink backend, driving the same
pinned PX4 `v1.17.0` binary. Goal: get the x500 quad flying the M3 baseline
mission in Isaac Sim, single instance. Stretch goal: port the rotor
degradation fault via an `omni.physx` physics-step callback instead of a
compiled plugin — plausibly *simpler* than the Gazebo version.

This machine's RTX 2070 (8GB) is under Isaac Sim's stated minimum spec.
Expect real performance ceilings and treat them as expected, not a problem to
solve — this track exists for learning, not for a number that ends up in the
paper.

---

## Progress log

Update this as milestones complete.

| Milestone | Status | Date | Notes |
|---|---|---|---|
| M0 | Done | 2026-08-14 | Gazebo Harmonic 8.15.0, PyTorch 2.13+cu126 (CUDA verified), PX4 v1.17.0 SITL builds clean, 236 px4_msgs interfaces visible. See docs/environment.md for full toolchain record and setup deviations. |
| M1 | Done | 2026-08-20 | sim_start.sh/sim_stop.sh working (multi-instance, clean orphan cleanup verified). RTF ≈ requested up to 8×; requesting 16× plateaus at ~8.3× (compute-bound, not a stability issue — flight stayed clean and stable at every tested factor). Two concurrent instances confirmed conflict-free. Full numbers in docs/simulation_notes.md. |
| M2 | Not started | | |
| M3 | Not started | | |
| M4 | Not started | | |
| M5 | Not started | | |
| M6 | Not started | | |
| M7 | Not started | | |
| M8 | Not started | | |
| M9 | Not started | | |
| M10 | Not started | | |
| M11 | Not started | | |
| M12 | Not started | | |
