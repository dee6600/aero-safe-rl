<div align="center">

# aero-safe-rl

**AI fault detection + high-level reinforcement learning recovery for autonomous UAVs**

*Detect a weakening motor before the flight controller notices. Decide how to finish the mission safely — without ever touching the low-level controller.*

[![Status](https://img.shields.io/badge/status-active%20research-brightgreen)](#-project-status)
[![PX4](https://img.shields.io/badge/PX4-v1.17.0-blue)](https://github.com/PX4/PX4-Autopilot)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic%208.15-orange)](https://gazebosim.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB)](https://www.python.org/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.13-EE4C2C)](https://pytorch.org/)
[![License: TBD](https://img.shields.io/badge/license-TBD-lightgrey)](#license)

[Overview](#overview) · [Research questions](#research-questions) · [Architecture](#architecture) · [Project status](#-project-status) · [Getting started](#getting-started) · [Roadmap](#roadmap)

<br>

<img src="docs/media/square_circuit_mission.gif" alt="PX4 SITL flying the square_circuit mission in Gazebo: takeoff, four-corner square, hover, land" width="560">

<sub>The real M3 <code>square_circuit</code> mission (<a href="configs/missions/square_circuit.yaml">config</a>), flown through <code>EpisodeRunner</code> — the same code path every reported number in this project comes from, not a separate demo. 2× sped up.</sub>

</div>

---

## Overview

A quadcopter loses 40% of one rotor's thrust mid-flight. PX4's own failure
detector — tuned for *complete* motor loss — stays silent. The aircraft
compensates and keeps flying, worse than it should, with nobody told.

**aero-safe-rl** is a research platform built to catch exactly that: a
telemetry-based AI detector that spots sub-threshold actuator degradation
PX4 can't see, paired with a high-level reinforcement-learning policy that
decides how to finish the mission safely — slow down, divert, land — while
PX4's low-level control loop stays completely untouched and stock.

Everything here is built for one property: **every result must be
regenerable.** Tool versions, seeds, and configs are pinned; nothing is
hand-edited after the fact.

<details>
<summary><b>Why this is a real contribution, not "RL beats PX4 at motor mixing"</b></summary>

<br>

PX4 already contains a `FailureDetector` and control-allocation-based
actuator failure handling — a paper claiming "RL beats PX4 at motor mixing"
would be weak and easy to dismiss. The contribution here sits at a
**different layer**:

1. **Sub-threshold, partial degradation.** PX4 handles binary motor loss
   reasonably; it does not reason about a rotor running at 60% effectiveness.
   That's where a learned detector earns its place.
2. **Mission-level recovery decisions.** What PX4 does *not* do: decide
   whether to continue, slow down, re-plan, loiter, or land, *given an
   uncertain fault estimate*. That's a sequential decision problem under
   partial observability — a legitimate RL problem.
3. **The detection–recovery coupling.** How detector latency and error
   propagate into closed-loop recovery outcomes. Understudied, and the most
   publishable angle.

The low-level controller (PX4) stays fixed and stock across every condition,
so the comparison isolates the actual contribution. Full reasoning in
[`planning.md`](planning.md).

</details>

## Research questions

| | Question |
|---|---|
| **RQ1** — Detection | How accurately and how quickly can a learned detector identify partial actuator degradation from telemetry alone, at severities below PX4's own failure-detector threshold? |
| **RQ2** — Recovery | Given a fault estimate, does a learned high-level policy outperform a hand-tuned rule-based policy on mission success and safety? |
| **RQ3** — Coupling | How sensitive is recovery performance to detection latency and false positives — is the combination more than the sum of its parts? |
| **RQ4** — Generalization | Does the policy transfer to unseen fault severities, timings, wind, and vehicle parameters? |
| **RQ5** — Sim-to-sim transfer | Does a high-level recovery policy trained in a massively-parallel *reduced-order* simulator transfer to a full autopilot-in-the-loop stack, and what is lost in the crossing? |

## Architecture

```mermaid
flowchart TB
    subgraph SIM["Gazebo Harmonic 8 — simulation"]
        GZ["x500 quad model<br/>+ rotor-degradation fault injection"]
    end
    subgraph FC["PX4 SITL (v1.17.0, pinned, stock)"]
        PX4["EKF2 · Commander · Control Allocation<br/>250–1000 Hz — never touched"]
    end
    subgraph MW["ROS 2 Humble"]
        BRIDGE["px4_msgs · telemetry/state pipeline"]
    end
    DET["Fault Detector (AI)<br/>~10 Hz — window → fault estimate"]
    POL["Recovery Policy<br/>rule-based OR RL — ~5 Hz"]
    LOG["Episode logging / metrics"]

    GZ -- "gz-transport" --> PX4
    PX4 -- "uXRCE-DDS" --> BRIDGE
    BRIDGE --> DET
    BRIDGE --> LOG
    DET -- "fault estimate" --> POL
    POL -- "high-level command<br/>(setpoint / mode / speed limit)" --> PX4

    style PX4 fill:#22314E,color:#fff
    style GZ fill:#e86a2c,color:#fff
    style DET fill:#7c3aed,color:#fff
    style POL fill:#7c3aed,color:#fff
```

The RL policy's action is always a **high-level command** — speed limits,
altitude offset, mission pacing, a decision to land — **never a motor
command**. That's what keeps the design hardware-transferable: the same ROS 2
node could talk to a real PX4 flight controller unchanged.

<details>
<summary><b>Control hierarchy</b></summary>

<br>

| Layer | Rate | Owner |
|---|---|---|
| Motor mixing, rate & attitude control, EKF2 | 250–1000 Hz | **PX4 (untouched)** |
| Position/velocity setpoint tracking | 50 Hz | **PX4** |
| Fault detection | ~10 Hz | Ours (AI) |
| High-level recovery decisions | ~5 Hz | Ours (RL) |

</details>

## 🚧 Project status

**M0–M3 done.** M2 has one open item — a real, confirmed, currently
unresolved DDS transport reliability gap under concurrent worker load, not
caused by this project's own code (full writeup:
[`docs/parallelism.md`](docs/parallelism.md) §2.6). M3 established the
project's noise floor (position RMSE **6.44 ± 0.57 m**) and its reproducibility
tolerance (**σ = 0.083 m** at fixed seed after hard reset) — see
[`docs/baseline_results.md`](docs/baseline_results.md). **M3b (Isaac Lab
feasibility) passed comfortably** — 546k env-steps/s at the chosen operating
point (8,192 parallel envs), no memory growth over a 10-minute sustained run;
host RAM, not GPU VRAM, is this machine's real constraint (see
[`docs/isaac_feasibility.md`](docs/isaac_feasibility.md)). **M4 done** —
the parallel evaluation farm (`EpisodeRunner`, `WorkerSupervisor`, `SimFarm`)
passed a 400-episode soak test at 2 workers (see
[`docs/throughput.md`](docs/throughput.md)). **M5 done** — the telemetry feature
pipeline and the frozen policy observation spec. **M6 done** — rotor fault
injection plus a 750-episode labelled dataset; PX4's own failure detector
misses every fault at severity 0.2–0.4 (see
[`docs/fault_dataset.md`](docs/fault_dataset.md)). Next: **M7**, the AI fault detector. See
[`milestones.md`](milestones.md) for the full task-by-task build log and
[`docs/`](docs/) for measured numbers.

| # | Milestone | Status |
|---|---|---|
| M0 | Environment setup and pinning | ✅ Done |
| M1 | PX4 + Gazebo simulator running | ✅ Done |
| M1b | Worker isolation, ownership, identity | ✅ Done |
| M2 | ROS 2 talks to PX4 | ✅ Done (one open reliability item, §2.6) |
| M3 | Autonomous mission baseline + episode contract | ✅ Done |
| M3b | **Isaac Lab feasibility spike** | ✅ Done — passed comfortably, see `docs/isaac_feasibility.md` |
| M4 | **Parallel evaluation farm + episode runner** | ✅ Done (one sim test deferred) |
| M5 | Telemetry feature pipeline | ✅ Done |
| M6 | Fault injection + dataset | ✅ Done — 750 episodes, see `docs/fault_dataset.md` |
| M7 | AI fault detector | ⬜ Not started |
| M8 | Rule-based recovery baseline | ⬜ Not started |
| M8b | **Isaac Lab training environment** | ⬜ Not started |
| M9 | RL recovery policy (train Isaac, eval PX4) | ⬜ Not started |
| M10 | Full experiments + results | ⬜ Not started |
| M11 | Generalization tests | ⬜ Not started |
| M12 | Hexacopter extension | ⬜ Not started |
| M13 | Paper + reproducibility package | ⬜ Not started |

> **Renumbered 2026-08-20.** A dedicated parallel-simulation milestone was
> inserted as M4, shifting the old M4–M12 to M5–M13. Parallel SITL turned out to
> be the project's largest source of silent bugs and the gate on whether RL
> training is feasible at all, so it now has its own acceptance criteria instead
> of living inside the RL milestone. Mapping table in
> [`milestones.md`](milestones.md).

> **Simulator strategy changed 2026-09-21 (D12).** RL training moves to a
> GPU-parallel **NVIDIA Isaac Lab** environment; PX4 + Gazebo remains the
> evaluation stack and the source of **every reported number**. This removes
> the project's largest risk (sample budget: PX4-in-the-loop training would
> have taken days per run) and adds RQ5, which measures the resulting
> sim-to-sim gap rather than assuming it away. Two milestones were added,
> **M3b** (feasibility — this machine is below Isaac Sim's stated minimum, so
> it was measured before anything depended on it: **passed**, 546k
> env-steps/s at the chosen operating point, host RAM rather than GPU VRAM
> turned out to be the real constraint) and **M8b** (the training
> environment, not yet built). Nothing already measured is invalidated. Full
> rationale: [`planning.md` §3.1](planning.md); feasibility numbers:
> [`docs/isaac_feasibility.md`](docs/isaac_feasibility.md).

<details>
<summary><b>What's actually been verified so far</b></summary>

<br>

- **Toolchain pinned and reproducible** — PX4 `v1.17.0`, Gazebo Harmonic
  `8.15.0`, ROS 2 Humble, PyTorch `2.13+cu126` with CUDA verified on the
  target GPU. Full record: [`docs/environment.md`](docs/environment.md).
- **Headless SITL scripted** (`scripts/sim_start.sh` / `sim_stop.sh`) —
  arms, takes off, hovers, lands from a single command, with clean shutdown
  and no orphaned processes.
- **Real-time factor measured, not assumed** — this machine is compute-bound
  at **~8× real-time**; flight stayed stable and clean at every tested speed
  factor up to a 16× request. Full breakdown:
  [`docs/simulation_notes.md`](docs/simulation_notes.md).
- **Multi-instance simulation isolated correctly** — each worker owns its own
  Gazebo server (`GZ_PARTITION`), independent speed factor, independent
  shutdown. PX4's default (one shared world) was tried first and found wrong
  for parallel RL; the corrected design and evidence are in
  [`docs/parallelism.md`](docs/parallelism.md).
- **ROS 2 flies PX4 on any instance number** — the two multi-instance bugs
  that made this fail silently on instance ≥ 1 are fixed and covered by
  tests. Latency measured (mean 7.2 ms). One open item: a DDS transport
  reliability gap under concurrent worker load — confirmed not caused by
  this project's own publish timing (instrumented and verified), root cause
  not fully isolated. `docs/parallelism.md` §2.6.

</details>

## Getting started

> The full, reproducible environment setup lives in
> [`docs/environment.md`](docs/environment.md) — this is the short version.

```bash
# Toolchain: PX4 v1.17.0 · Gazebo Harmonic 8 · ROS 2 Humble · conda (Python 3.10)
git clone https://github.com/dee6600/aero-safe-rl.git
cd aero-safe-rl

conda env create -f environment.yml
conda activate aero-safe-rl

# Print the full pinned toolchain state as JSON
./scripts/env_report.sh

# Start a headless PX4 + Gazebo instance
./scripts/sim_start.sh -i 0
./scripts/sim_stop.sh

# ...or watch it fly over ROS 2: opens the Gazebo GUI, arms, takes off,
# hovers, lands, and prints a pass/fail summary
./scripts/watch_worlds.sh -n 1
```

## Tech stack

| Layer | Choice |
|---|---|
| Flight stack | [PX4 Autopilot](https://px4.io/) `v1.17.0`, stock, never patched |
| Simulator | [Gazebo Harmonic](https://gazebosim.org/) 8 |
| Middleware | ROS 2 Humble, `px4_msgs` / `px4_ros_com` over uXRCE-DDS |
| ML | PyTorch, Gymnasium, Stable-Baselines3 (PPO) |
| Environment | conda (`aero-safe-rl` env), pinned via `environment.yml` |

## Repository structure

<details>
<summary>Expand tree</summary>

```
aero-safe-rl/
├── planning.md      # research plan: questions, architecture, decisions
├── milestones.md    # the build order — what to do, in what sequence
├── CLAUDE.md        # coding rules every contributor and agent follows
├── configs/         # all experiment configuration (YAML)
├── simulation/      # PX4/Gazebo layer — models, fault injection
├── ros2_ws/src/     # colcon workspace — telemetry pipeline, mission executor
├── ai/              # fault detection — features, models, training
├── rl/              # reinforcement learning — env, rewards, policies
├── experiments/     # batch orchestration + analysis
├── scripts/         # env_report.sh, sim_start.sh, sim_stop.sh, ...
├── tests/           # unit (default), sim/ (@sim), slow/ (@slow), fixtures/
├── results/         # raw logs, trained artifacts (git-ignored)
└── docs/            # environment.md, parallelism.md, simulation_notes.md, ...
```

</details>

## Roadmap

This project is built in 14 ordered milestones, each with an explicit,
runnable acceptance test and its own unit tests — no milestone starts until the
previous one's checks all pass.

- **[`planning.md`](planning.md)** — the *what* and *why*: research
  questions, architecture, RL design, evaluation methodology, and every
  major decision with its reasoning.
- **[`milestones.md`](milestones.md)** — the *how* and *in what order*:
  concrete tasks, files created, required tests, done-when checklists, and a
  ready-to-paste implementation prompt per milestone.
- **[`CLAUDE.md`](CLAUDE.md)** — the coding rules: instance identity, timing,
  parallelism, testing tiers, and the anti-patterns that have cost time here.
- **[`docs/parallelism.md`](docs/parallelism.md)** — verified multi-instance
  PX4/Gazebo behaviour, with source references and measured output.

**First result worth showing anyone** lands at the end of **M7** — a
detector that spots a weakening motor PX4 itself never notices.
**First publishable result** lands at the end of **M10**.

<details>
<summary><b>Locked decisions</b></summary>

<br>

| ID | Decision |
|---|---|
| D1 | PX4 pinned to **`v1.17.0`** |
| D2 | Partial rotor faults via **our own gz-sim plugin**, not PX4's binary-only failure command |
| D3 | Repo named **`aero-safe-rl`** |
| D4 | Evaluation includes an oracle-detector upper bound and a detector-ablation condition |
| ~~D5~~ | ~~Simplified pre-training model **deferred**~~ — **superseded by D12** |
| ~~D6~~ | ~~Isaac Sim considered and **declined**~~ — **superseded by D12** |
| D7 | **One drone per world**, `GZ_PARTITION`-isolated; silent, unchosen sharing stays prohibited |
| D8 | **We own the Gazebo server process** so a single worker can be restarted without touching its siblings |
| D9 | **Uniform instance identity**, no special case for instance 0; derived once and published as a file |
| D10 | **Sim time is the only clock** in flight logic, from `GzSimClock` (Gazebo's own clock) — not `px4_msgs` timestamps, which track wall clock regardless of speed factor; wall clock only in the hang watchdog |
| D11 | **Reproducibility is statistical, not bitwise** — pure functions are exact, whole-pipeline results reproduce within a measured band |
| **D12** | **Train in Isaac Lab, evaluate in PX4-in-the-loop** (2026-09-21) — supersedes D5 and D6. GPU-parallel training removes the sample-budget risk; every *reported* number still comes from the real autopilot stack. Adds RQ5. |

Full reasoning for each: [`planning.md` §14](planning.md#14-decisions--all-approved-2026-08-14).

</details>

## License

Not yet decided — a license will be added before any external contributions
or reuse are expected.

## Acknowledgments

Built on [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot),
[Gazebo](https://gazebosim.org/), and [ROS 2](https://www.ros.org/).
