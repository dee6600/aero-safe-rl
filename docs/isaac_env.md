# The Isaac Lab training environment (M8b)

M9 trains the recovery policy in Isaac Lab, with thousands of drones in
parallel on the graphics card, and evaluates it on the PX4 + Gazebo stack.
This document records how the Isaac side was built to fly like the PX4 side,
and the checks showing that it does. Everything here was measured; nothing on
the PX4 side was adjusted to make a check pass.

## How to run it

```bash
source scripts/activate_isaac.sh                                # the isaacsim environment, never ROS
python -m pytest isaac/tests -m "not isaac and not slow"        # pure PyTorch, a few seconds
python -m pytest isaac/tests -m slow                            # closed-loop controller checks, ~1 min
python -m pytest isaac/tests -m isaac                           # starts Isaac Sim headless, ~5 min
cd isaac && python -m aero_isaac.agreement --policy nominal \
    --px4 ../results/m8b_agreement/px4_nominal.json --out ../results/m8b_agreement/isaac_nominal.json
cd isaac && python -m aero_isaac.probe throughput --num-envs 8192 --out ../results/m8b_throughput.json
cd isaac && python -m aero_isaac.probe clip --out ../results/m8b_isaac_clip.mp4
```

The PX4 side writes the files the Isaac side reads, in the `aero-safe-rl`
environment: `python experiments/write_isaac_fixtures.py` (the shared
contract fixture) and `python experiments/fit_detector_sim.py` (the simulated
detector's parameters and its held-out fixture).

## What is shared, and how

The two sides never import each other (`CLAUDE.md` §0.1; a static scan in
`isaac/tests/test_boundary.py` enforces it). They share files:

| File | What it fixes |
|---|---|
| `configs/rl/action_v1.yaml` | the 3 actions and the 0.2 s decision period |
| `configs/rl/outcome_v1.yaml` | crash = tilt over 60° or touchdown faster than 2.0 m/s |
| `configs/rl/observation_v2.yaml` | the 27 observation values, in order |
| `configs/missions/square_circuit.yaml` | the mission |
| `configs/rl/detector_sim_v1.yaml` | the simulated detector (below) |
| `tests/fixtures/isaac_contract_v1.json` | recorded input and output cases from the PX4-side code |

The fixture holds action decoding, three mission-tracker trajectories, 23
outcome cases and 12 observation vectors built from logged flights. The Isaac
side's PyTorch versions reproduce every case (`isaac/tests/test_mission.py`,
`test_outcome.py`, `test_observation.py`, `test_contracts.py`).

The observation is assembled from four named blocks only: sensor features,
detector output, mission progress and previous action. The true fault is not
one of them, so the builder has no way to receive it. A live check in
`isaac/tests/test_env.py` confirms this: with physics, the detector's last
output and the sensor-noise draw all held fixed, the true fault is changed on
every drone. The observation must not change at all. Changing the detector's
output instead must change it, which shows the check can see a change when
there is one.

## The vehicle

- **The same x500 that PX4 flies in Gazebo.** `isaac/aero_isaac/x500_asset.py`
  reads PX4's pinned `x500_base/model.sdf` and writes a robot description file
  for Isaac's importer. The frame and four propellers are merged into one
  rigid body with the combined mass (2.0643 kg) and inertia. Collision boxes,
  including the legs and skids that decide tip-over at touchdown, are copied
  as they are. The 3D meshes are only for looks. The drone rests with its
  centre 0.227 m above the ground, computed from the collision boxes.
- **Rotor model copied from Gazebo's** (`rotor.py`): speed = 150 + 850 ×
  command, thrust = 8.54858e-6 × speed², motor lag 0.0125 s speeding up and
  0.025 s slowing down, rotor drag included. The fault scales the commanded
  speed by √(1 − s), as the Gazebo plugin does. It is checked against M6's
  recorded plugin fixture at three commands (`CLAUDE.md` §1.6).
- **PX4's flight controller, ported to PyTorch** (`controller.py`), with
  PX4 v1.17's default gains read from the pinned source (`px4_model.py`),
  never retyped. The port covers position, velocity, attitude and rate
  control, and allocation with PX4's saturation handling: when motors
  saturate, total thrust is cut first, then roll and pitch, and yaw last.
  It also covers PX4's takeoff sequence: 1 s of motor spin-up, then a climb
  rate ramped over 3 s, with tilt limited to 12° until the drone is flying.
- **Timing.** Physics runs at 250 steps per second, Gazebo's own step
  (0.004 s in PX4's default world), not the 200 the plan assumed. The
  position loop runs every 5th step (50 per second). The mission tracker,
  outcome rule and detector tick every 0.1 s, as on the PX4 side. The policy
  decides every 0.2 s, and the environment refuses to start if that differs
  from `action_v1.yaml` (`CLAUDE.md` §4).
- **Sensor noise**, one standard deviation, measured from M6's hover
  stretches (an upper bound, since it includes some real motion): attitude
  0.0056 / 0.0051 / 0.0011 rad, body rates 0.0087 / 0.0076 / 0.0054 rad/s,
  specific force 0.0055 / 0.0056 / 0.0186 m/s², velocity 0.025 / 0.029 /
  0.004 m/s.

PX4's hover-thrust estimator and its state estimator were not ported. The
plan said to add them only if an agreement check failed because of them.
None did.

## Does it fly like PX4? (task 5)

Both simulators fly the same severity grid under the same fixed action, with
no detector. The gates were written into the plan before anything was
measured. PX4 values: "no recovery" pools M8's validation and sweep flights
with no recovery (102 flights); "slow and low" (speed 0.3, 3 m lower from the
start) is a new 48-flight run, `results/m8b_px4_slow_low`. Isaac flies 16
drones per severity. Results: `results/m8b_agreement/`.

| Measure (gate) | No recovery: Isaac | PX4 | Slow and low: Isaac | PX4 |
|---|---|---|---|---|
| healthy mission time (±10%) | 45.8 s | 43.4 s | 42.6 s | 44.7 s |
| healthy peak speed (±15%) | 9.12 m/s | 9.13 m/s | 4.10 m/s | 4.06 m/s |
| healthy mean motor command (±0.05) | 0.750 | 0.749 | 0.730 | 0.730 |
| severity where crash rate crosses 50% (±0.05) | 0.423 | 0.425 | 0.473 | 0.470 |
| median touchdown speed, s 0.40 (±0.6 m/s) | 1.58 | 1.46 | 0.72 | 0.69 |
| median touchdown speed, s 0.45 | 2.75 | 2.97 | 1.74 | 1.87 |
| median touchdown speed, s 0.50 | 4.05 | 4.14 | 2.65 | 2.27 |
| median touchdown speed, s 0.70 | 6.66 | 6.62 | 3.99 | 3.60 |

**All 16 gates pass.** The cliff near s ≈ 0.42 appears on both sides from
physics plus the ported saturation handling. Nothing was tuned to produce it.

One gate failed on the first attempt. Slow and low's healthy mission time
was 39.2 s against PX4's 44.7 s, while every no-recovery gate already passed.
The cause was PX4's takeoff sequence (spin-up, then a ramped climb), which
the first controller port skipped. It costs about 4 s, and a slow mission
cannot win that time back later. It was ported from the source, and both
checks were re-run. The largest remaining gap is slow and low's touchdown at
s 0.50: 2.65 against 2.27 m/s, inside the gate.

## The simulated detector (task 6)

During training the real detector cannot run, because it needs PX4
telemetry. The policy sees a stand-in instead: `isaac/aero_isaac/detector_sim.py`,
built entirely from `configs/rl/detector_sim_v1.yaml`, which
`experiments/fit_detector_sim.py` fits to the real detector's recorded output.
It is given the true fault, as any simulator of a detector must be. What it
returns is the only thing that reaches the observation.

### Model

Every 0.1 s it outputs what the real detector outputs: fault probability,
rotor, severity estimate, uncertainty, alarm.

- **Healthy flight:** background values drawn from the measured healthy
  distribution. False-alarm bursts arrive at the measured rate (192 per
  healthy hour, 80% of them a single tick). Each burst draws a length and
  peak together from the 335 observed bursts, because long bursts are also
  the high ones. Only 6% of bursts are the kind that matter (probability
  0.5 or more for at least 0.3 s), about 11 per hour.
- **Detection:** a sudden fault is detected after a delay drawn per
  severity band (median 0.10 s; 0.15 s below severity 0.3). A ramp is
  detected when the true severity reaches a drawn level: median 0.09 below
  severity 0.3, 0.11 at 0.3–0.4, 0.13 at 0.4–0.5, 0.19 at 0.5–0.7 and 0.15
  above. No misses were observed. The rotor is named correctly 99.9% of the
  time or better.
- **Severity estimate after detection:** true severity + measured bias (at
  most 0.02) + correlated noise (standard deviation 0.02–0.03, correlation
  0.95 from one tick to the next).
- **The descent over-read** that M8 found: when a commanded descent starts,
  an extra error with the measured shape over time is added. Its median
  rises to about +0.056 within 2 s, drops to about +0.03 by 3 s and stays
  between +0.024 and +0.043 up to 10 s. Each descent scales that shape by a
  draw from 169 measured descents.

### Fit data: only flights the detector never trained on

- M6's test portion (110 flights) replayed through the detector, plus 467
  live flights with the detector running: M8's check, sweep, tie-break and
  validation runs, and `m8b_px4_descents_low` (36 flights with the recovery
  controller at s 0.20–0.30). That last run was flown for this fit, because
  no earlier run had recovery descents below 0.35.
- **Healthy behaviour** is fitted from live flights only. Replayed and live
  flights have the same rate of false alarms that matter (10.8 against 10.7
  per hour), but the live detector blips for a single tick about twice as
  often, and the policy is evaluated live.
- **Fault behaviour** leaves out the M6 flights the detector trained on.
  Replayed on its own training flights, it catches ramps at severity 0.3–0.4
  in a median 0.62 s. On its test flights it takes 0.92 s, and live 1.13 s.
  Sudden faults are caught in one or two ticks either way.

### Held-out check

`isaac/tests/test_detector_sim.py` replays the held-out flights' conditions
(fault, onset, ramp, descent start) through the simulator, 20 times each, and
compares with the real detector on those flights. Tolerances, fixed before
the first comparison and never changed:

- detection delay, median per severity band and onset type, in cells with at
  least 8 flights: within 0.3 s;
- false alarms per healthy hour: within a factor of 1.5;
- false alarms that matter per healthy hour: within a factor of 2;
- severity over-read in each of the first 3 s of a descent, median: within
  0.05.

**Result on the confirmation run** (138 valid flights; checked once, after
freezing; the refit for the fixture reproduced the frozen file exactly):

| Check | Simulator | Real | Result |
|---|---|---|---|
| false alarms per healthy hour | 189 | 236 | pass (factor 0.80) |
| false alarms that matter per hour | 12.5 | 13.2 | pass |
| descent over-read, 0–1 s / 1–2 s / 2–3 s | 0.033 / 0.072 / 0.053 | 0.032 / 0.079 / 0.071 | pass |
| detection delay, 9 cells with 8+ flights | | | 8 pass |
| detection delay, ramps at severity 0.5–0.7 (8 flights) | 1.70 s | 1.12 s | **fail** (0.58 s late) |

Ramp delays in the other cells: 0.0–0.3 1.60 against 1.53 s; 0.3–0.4 1.10
against 0.93; 0.4–0.5 0.90 against 0.72; above 0.7 0.50 against 0.56. The
simulator tends to catch ramps a little late, and at 0.5–0.7 it is outside
the tolerance. At those severities the drone falls whatever the policy does
(the cliff is near 0.42), so the delay matters less there than anywhere
else.

### How the model got here, honestly

The first held-out set was M8's validation run. It was consulted four times
while the model was revised, so by the end it was no longer a fair test.
Every revision was fitted on fit data only, but each came after a failed
held-out check:

1. **First version:** burst length and height drawn separately, a fixed
   delay for ramps, and an exponential descent term. It failed three checks.
   False alarms that matter came out too rare, because separate draws rarely
   produce a burst that is both long and high. Ramp delays were wrong
   because a fixed delay ignores the ramp's length. The descent term had the
   wrong shape.
2. **Revision 2:** bursts drawn as observed (length, peak) pairs, ramps
   detected at a drawn fraction of their length, and the descent term given
   its measured shape. The false-alarm checks passed. The ramp and descent
   checks still failed. For the descent term, the fit data had no descents
   at low severity, so `m8b_px4_descents_low` was flown and added.
3. **Revision 3:** a measurement bug. The fit script read each ramp's length
   off the flight record. For a flight where the recovery controller landed
   before the ramp finished (17 of the 56 held-out ramps), it recorded
   length 0, and the check then replayed those flights as sudden faults.
   Lengths now come from the fault command (`tests/test_fit_detector_sim.py`).
   With the new descents added too, the descent check and the
   low-severity ramp cell passed. Ramps at 0.3–0.4 still failed (0.70 s
   against 1.02 s).
4. **Revision 4:** the M6 flights the detector trained on were removed from
   the fault fit, for the reason above. The 0.3–0.4 ramp cell then passed,
   and ramps at 0.5–0.7 failed (1.40 s against 0.82 s).
5. **Revision 5:** the ramp model, never checked against M8's validation
   run. "Detected at a fraction of the ramp's length" assumes the delay grows in proportion to the length, and it does
   not. The replacement was chosen by 5-fold cross-validation on fit flights
   only. The three candidates were detection at a severity level, at a
   fraction of the length, and a fitted formula in length and severity. The
   level model had the smallest typical error: 0.216 on a log scale, against
   0.245 and 0.226. That cross-validation also shows that, with 8 flights in
   a cell, a cell's median lands up to about 0.3 s off even for the best
   model. The 0.3 s tolerance is tight, but it stays.

After revision 5, the model was frozen and the check was moved to fresh
flights. M8's validation run joined the fit data. A confirmation run with the
same design was flown afterwards: seed 8501, used by no other run; 8 flights
per severity from 0.2 to 0.7 plus 16 healthy; once with no recovery and once
with the recovery controller; 144 flights on 2 workers. The parameter file's
content hash (`sha256sum`) at freezing, recorded at 2026-09-23 20:48 UTC
before any confirmation flight was read, was
`a2a0df587d61e86b679cc2e0ff24a69455aed38116ac458db04e9c50bccb9429`. The fit
script refuses to read a held-out run until that run has finished.

## The environment in Isaac Lab (task 4)

`isaac/aero_isaac/env.py` (`AeroEnv`, an Isaac Lab `DirectRLEnv`). Each drone
starts on the ground, takes off through the mission tracker, and flies the
square circuit. A fault is drawn per drone: 80% of episodes, one rotor,
severity 0.2–0.9, onset 5–25 s, step or ramp (1–5 s). M9 widens these. A land
decision is final and mimics PX4's land mode: hold position, descend at
0.7 m/s. An episode ends on uncommanded ground contact, after landing and
settling for 2 s, or at the mission time limit. The reward is zero; M9
freezes it. Per-episode results go to `env.episode_log`.

Headless smoke test (`isaac/tests/test_env.py`, 16 drones at s 0 / 0.3 /
0.45 / 0.7, about 5 minutes): observation shape (16, 27) and finite;
decision period 0.2 s; every drone finishes an episode and is reset on the
ground; healthy and s 0.3 drones complete the mission; s 0.7 drones crash;
the leak check above.

**Throughput and memory** (`results/m8b_throughput*.json`, 100 timed decisions,
nominal action, simulated detector):

| Drones | Decisions per second | Physics steps per second | Graphics-card memory | Computer memory (peak) |
|---|---|---|---|---|
| 8,192 | 8,688 | 434,000 | 3.1 of 8.2 GB | 4.9 GB |
| 16,384 | 16,342 | 817,000 | 3.5 GB | 5.8 GB |
| 32,768 | 24,297 | 1,215,000 | 4.6 GB | 7.4 GB |

At 8,192 a step takes as long as at 16,384: the time goes on the fixed cost
of launching many small computations, not on the drones. At 32,768 the
graphics card starts to be the limit. Memory grew by at most 10 MB while
stepping. M3b's stock quadcopter task did 546,000 steps per second at 8,192;
ours runs the full controller and rotor model on every physics step. Even at
8,192, 3 million decisions take about 6 minutes, so M9's sample budget can
be far larger than planned.

**Clip:** `results/m8b_isaac_clip.mp4` (and a smaller copy in the README,
`docs/media/isaac_four_drones.gif`): four drones at severities 0, 0.3, 0.45
and 0.7, fixed camera, 4× speed.
