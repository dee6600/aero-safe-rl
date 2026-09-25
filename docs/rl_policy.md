# M9 — the learned recovery policy: trained in Isaac, checked on PX4

Companion to `milestones.md` M9. This document gives the reward and the
check that caught its first version's flaw, the training setup, the
training curves, the Isaac-against-PX4 transfer table (research question 5),
and the PX4 comparison with no recovery and the rule-based controller.

Every number in the comparison sections comes from real flights on the PX4
and Gazebo stack. Isaac supplies only two things here: the training curves,
and each policy's Isaac score shown beside its PX4 score in the transfer
table.

## Headline

**On PX4, the learned policy never panics, and it finishes more missions
right below the physical limit. It does not save more drones above that
limit.** All three training seeds, flown on the same fresh fault schedule
as no recovery and the rule-based controller (seed 9101, 8 flights per cell,
16 healthy):

- **Faults of 0.20–0.35.** It finished 71 of 71 missions (95% interval
  95–100%). The rule-based controller finished 6 of 23 (26%, 13–46%), giving
  up the rest. Flying with no recovery also finished all 24.
- **Faults of 0.40.** It finished 9 of 23 missions (39%, 22–59%) and crashed
  1. No recovery and the rule-based controller finished none: all 16 of
  their drones came down, safely. With 8 flights per cell the intervals
  still just overlap.
- **Faults of 0.45 and above.** It crashed 22 of 24 at 0.45 (92%, 74–98%),
  against 6 of 6 for no recovery and 6 of 7 for the rule-based controller.
  Above 0.45 every method crashed every flight.
- **Healthy flights.** No landings commanded on any of its 45 healthy
  flights.

**The three seeds differ a lot.** Seed 1 learned the manoeuvre in Isaac:
there it crashed 67% at 0.45 against 100% for the others. On PX4 it saved 2
of 8 at 0.45. Seeds 2 and 3 learned mainly not to panic. Every seed was
still improving when the fixed 300-update budget ended.

**Extended to 600 updates (below).** Seed 1 improved again. On PX4 it
crashed 10 of 16 at 0.45–0.50, against 14 of 14 for no recovery, and saved
drones at 0.50 for the first time. It paid with 2 needless landings at
0.35. Seed 3 did not change. Seed 2 found a flaw in the reward and the
fault mix: it learned to refuse to take off, so it was not flown on PX4.

**Transfer (research question 5).** Every Isaac crash rate (24 of 24 cells)
falls inside the PX4 result's 95% interval. Mission success does too,
except at 0.40, where all three seeds finished more missions on PX4 than
Isaac predicted (14% against 1%, 38% against 12%, 62% against 21%). The
policy carries over. Right at the edge of what the airframe can do, PX4
turned out a little more forgiving than the training simulator.

These are M9's checks, with small cells. M10 repeats the comparison with at
least 100 flights per cell on held-out seeds.

```bash
source scripts/activate_isaac.sh && cd isaac
python -m aero_isaac.agreement --ranking --per-severity 32 --out ../results/m9_reward_ranking_v2.json
python -m aero_isaac.train train --seed 1 --config configs/rl/train_v3.yaml \
    --run ../results/m9_train/train_v3_seed_1                  # also seeds 2 and 3
python -m aero_isaac.train curves --run ../results/m9_train/train_v3_seed_1
python -m aero_isaac.agreement --checkpoint ../results/m9_train/train_v3_seed_1/policy.pt \
    --per-severity 256 --seed 3 --out ../results/m9_train/train_v3_seed_1/isaac_scores.json
# then, in the aero-safe-rl environment (the same schedule for every condition):
python experiments/run_recovery.py fly --run-id m9_px4_v3_seed_1 --policy learned \
    --policy-config results/m9_train/train_v3_seed_1/policy.pt --seed 9101 \
    --severities 0 0 0.2 0.3 0.35 0.4 0.45 0.5 0.7 --episodes-per-severity 8 \
    --detector results/m7_detector_v1/detector.pt  # also --policy nominal / rule_based
python experiments/run_recovery.py transfer ...                 # (below)
tensorboard --logdir results/m9_train                          # the training dashboard
```

## What the policy sees and does

It sees and does exactly what the rule-based controller does. It gets the
27 values of `observation_v2`:
- the 13 sensor features;
- the detector's estimate;
- mission progress;
- its own previous action.

The true fault is never among them. It answers with the 3 values of
`action_v1`: a speed scale, an altitude offset, and an irreversible
decision to land. It decides every 0.2 s.

The network has 27 inputs, two hidden layers of 128 units, and 3 outputs.
It is evaluated with its mean action, with no sampling. Its raw output u is
mapped to an action centred on the nominal flight:

- speed = 1 + 0.5u;
- altitude offset = 1.75u;
- land = −0.5 + 0.25u.

So an untrained policy flies the mission normally, and landing needs u ≥ 4,
which exploration almost never produces by accident. The mapping is folded
into the exported network's last layer, so the PX4 side runs the network
and `action_v1`'s own decode, nothing more.

## The reward, and the check that caught its first version

`configs/rl/reward_v2.yaml`, computed in `isaac/aero_isaac/reward.py`:

| Part | Value |
|---|---|
| mission success | +10, paid when the mission is completed |
| safe landing | +4 |
| incomplete (mission timer ran out) | 0 |
| crash | −10 |
| touchdown speed | −1 per m/s, at touchdown |
| progress along the mission | a weight of 2, as exact progress-difference shaping |

- **The four outcome values were chosen by the user.** They mean "keep
  flying only if at least 70% likely to finish": 10p − 10(1 − p) = 4 at
  p = 0.7.
- **The progress part cannot change which policy is best.** Its discounted
  sum over any episode from the ground is exactly zero. It only speeds up
  learning.
- **The reward never reads the fault.** A parse-tree test confirms that
  `reward.py` never names a fault, a severity or a rotor.

**The reward-ranking check, before any training.** Three scripted policies
flew every severity, with the fitted detector:

- always nominal;
- nominal until the detector's probability reached 0.5, then slow and low;
- nominal until then, then land.

The reward had to rank them the way their outcomes rank: flying normally
best at 0.2 and 0.3, and a reaction best at 0.40 and 0.45.

**Version 1 failed that check.** At 0.2 and 0.3, "slow down and descend"
scored 7.48 against 7.40 for flying normally, although both finished 100% of
missions (`results/m9_reward_ranking_v1.json`). The cause: version 1 paid
success only after PX4's final landing. A drone already down at 2 m finishes
that landing about 4 s sooner than one at 5 m, and under the training
discount a sooner payment is worth more. So the reward paid for flying low
at the end of a mission, for no reason to do with faults.

Version 2 pays success when the mission is completed, and takes it back if
the flight is not judged a success after landing. Re-run on the same faults,
it passes (`results/m9_reward_ranking_v2.json`):

| Severity | Flying normally | Slow and low | Land |
|---|---|---|---|
| 0.20 | **7.75** | 7.66 | 2.93 |
| 0.30 | **7.74** | 7.63 | 2.75 |
| 0.40 | 0.28 | **2.21** | 1.56 |
| 0.45 | −11.81 | −10.60 | **−8.95** |

The same check also moved the planned discount from 0.995 to 0.999. At
0.995 finishing a 45 s mission was worth about as much as landing at once
(3.2 against 3.1), which would have quietly undone the 70% setting.

## Training setup

`configs/rl/train_v2.yaml`, frozen at fingerprint `51b40d6cc3167530`, with
reward `reward_v2.yaml`, fingerprint `137e9b040436ad84`.

- **Algorithm:** `rsl_rl` 5.0.1 proximal policy optimisation.
- **Scale:** 16,384 drones; 24 decisions per update; discount 0.999.
- **Exploration bonus:** none (`entropy_coef` 0).
- **Budget:** 300 updates per seed, fixed in advance. Each seed reports its
  final checkpoint, never a "best" one. Seeds 1, 2 and 3.
- **No running observation normaliser:** the contract already normalises
  with frozen statistics.
- **Speed:** about 24.5 s per update whatever the drone count, so a seed
  takes about 2 hours. Memory: 4.2 GB graphics, 6.3 GB computer.

**Why version 2 (seed 1's first run was stopped).** Under `train_v1`, seed 1
was reviewed at update 62 of 300 and stopped. The run is kept at
`results/m9_train/stopped_train_v1_seed_1`.

- `train_v1`'s exploration bonus (`entropy_coef` 0.005) was widening every
  command's noise: speed 1.00 → 1.25, altitude → 1.22, land → 1.32.
- Nothing in the task pushes back on the land command. Its irreversible
  threshold only matters on the rare occasions noise crosses it.
- Accidental landings of healthy drones therefore rose from 0.2% to 7.9%.
- The noise parameters jumping every update also kept tripping the adaptive
  learning-rate schedule down to its floor, so the policy barely changed.
- Isaac Lab's own quadcopter task uses no bonus; `train_v2` changes only
  that.

**Randomised per episode, in training only:**

| What | Range |
|---|---|
| Faults | 20% healthy; severity 0.1–1.0; onset 3–35 s; half sudden, half ramped over 1–8 s |
| Vehicle | mass ×0.95–1.05, with inertia; a steady horizontal wind force of 0–1 N |
| Sensors | noise ×0.5–1.5 |
| Simulated detector | delay, false-alarm rate, severity noise and descent over-read each ×0.5–2.0; severity bias ±0.03 |

The detector ranges cover its one known gap: ramps at 0.5–0.7 detected
0.58 s late. When the multipliers are left at their neutral values, the
simulated detector's output is bit-identical to the fitted model's.

## Training: three versions, and what each one learned

Every change below was made on Isaac training evidence alone, before any PX4
number existed for the version concerned. Each is a new version file, with
its reason written at the top.

| Version | What happened | Where |
|---|---|---|
| `train_v1` | Stopped at update 62. The exploration bonus widened every command's noise, so accidental landings of healthy drones rose from 0.2% to 7.9%, and the learning rate kept collapsing. | `results/m9_train/stopped_train_v1_seed_1` |
| `train_v2` | No exploration bonus. Learned not to panic, but not the manoeuvre: from update 25 to 300 the crash rate for 0.35–0.45 stayed at about 33%, the same as flying normally. | `results/m9_train/train_v2_seed_1` |
| `train_v3` | Half of faulty episodes drawn from 0.3–0.5, and a fixed learning rate of 3e-4. The crash rate for 0.35–0.45 falls steadily from about 35% to about 20% after update 100. **It is still falling at update 300**, so the fixed budget stopped it before it converged. | `results/m9_train/train_v3_seed_{1,2,3}` |

Curves for each run are in its `curves.png` and `metrics.csv`, and the full
dashboard is in TensorBoard (`tensorboard --logdir results/m9_train`).

**Choosing between v2 and v3.** The rule was written before v3 ran. Both
seed-1 policies were scored in Isaac on the fixed evaluation grid (mean
action, fitted detector, 256 drones per cell). v3 had to lower the mean
crash rate over 0.40 and 0.45 by at least 5 points, while still finishing
at least 98% of missions at 0.20–0.35.

| Severity | v2: crash | v3: crash (safe landing) | Median touchdown, v2 → v3 (m/s) |
|---|---|---|---|
| 0–0.35 | 0% (100% success) | 0% (100% success) | 0.70 → 0.70 |
| 0.40 | 6% | 0% (99%) | 1.31 → 0.97 |
| 0.45 | 100% | 67% (33%) | 2.56 → 2.28 |
| 0.50 | 100% | 99% (1%) | 3.46 → 3.20 |
| 0.70 | 100% | 100% | 5.26 → 4.82 |

That is 53% → 33.5%, so v3 was adopted. Seeds 2 and 3 were trained only under
v3, unchanged, whatever seed 1 later scored on PX4.

**Why the manoeuvre is hard to learn.**

- **Little to gain.** Of 16,384 drones, the ones in the 0.35–0.45 band gain
  only a little from reacting. Even the best scripted reaction still crashed
  81% of flights at 0.45.
- **Exploration does not help.** The altitude command is rate-limited, so
  noise that jitters it every 0.2 s averages out and never produces a
  sustained descent. The policy has to move its mean command there
  deliberately.

v3 attacks the first by drawing faults from the band more often, and gives
learning a steadier step size.

## Isaac against PX4: the transfer table

Each `train_v3` policy was scored twice with its mean action and the same
severity grid and fault timing. In Isaac: 256 drones per cell, with the
fitted simulated detector and no training randomisation
(`results/m9_train/train_v3_seed_*/isaac_scores.json`). On PX4: 8 flights
per cell (16 healthy), with the real M7 detector running live
(`results/m9_px4_v3_seed_*`). The full table with 95% Wilson intervals is
in `results/m9_transfer.txt` and `.json`:

```bash
python experiments/run_recovery.py transfer \
    --policy seed_1 results/m9_px4_v3_seed_1 results/m9_train/train_v3_seed_1/isaac_scores.json \
    --policy seed_2 ... --policy seed_3 ... \
    --baseline "no recovery" results/m9_px4_nominal --baseline "rule-based" results/m9_px4_rule_based
```

Success / crash, Isaac → PX4:

| Severity | Seed 1 | Seed 2 | Seed 3 |
|---|---|---|---|
| healthy–0.35 | 100/0 → 100/0 | 100/0 → 100/0 | 100/0 → 100/0 |
| 0.40 | 1/0 → 14/0 | 12/15 → 38/12 | 21/12 → 62/0 |
| 0.45 | 0/67 → 0/75 | 0/92 → 0/100 | 0/100 → 0/100 |
| 0.50 | 0/99 → 0/100 | 0/100 → 0/100 | 0/100 → 0/100 |
| 0.70 | 0/100 → 0/100 | 0/99 → 0/100 | 0/100 → 0/100 |

- **Crash rates carry over.** All 24 Isaac crash rates fall inside the PX4
  result's 95% interval.
- **Mission success carries over except at 0.40.** 21 of 24 cells agree.
  The three that do not are all at 0.40 and all go the same way: PX4
  finished more missions than Isaac predicted. At 0.40 the weak rotor sits
  just below the hover limit (about 0.41), so whether a slowed, lowered
  drone keeps flying depends on small thrust margins. PX4's hover-thrust
  estimator adapts to the lost thrust, and it was not ported to Isaac (M8b
  found the agreement checks did not need it), which is a plausible reason
  PX4 has slightly more margin there. The gap is at the one place where
  that margin decides the outcome.
- **One Isaac-only artifact.** In seed 2's Isaac scoring, 22 of 2048 drones
  committed to land 1–2 s into the flight, while still on the ground
  spinning up. They came in blocks of ten neighbouring drones.
  - Isaac's contact physics can spike the sideways-acceleration feature
    there (up to 20 standard deviations), and seed 2 answers that with a
    land command.
  - On PX4, where accelerations come from its filtered sensor pipeline, it
    never happened: all 45 healthy flights finished.
  - A fix for later: filter the Isaac acceleration feature the way PX4
    does.

## Against no recovery and the rule-based controller, on PX4

Same fault schedule for every condition (seed 9101, never used before): 16
healthy flights, then 8 per severity at 0.2, 0.3, 0.35, 0.4, 0.45, 0.5 and
0.7. Onsets 5–25 s, half sudden and half ramped. The M7 detector ran live,
on 2 workers at 1× speed. 348 valid flights out of 360: 12 worker restarts
at start-up, each recorded as invalid rather than dropped.

Success / safe landing / crash, per condition:

| Severity | No recovery | Rule-based | Learned seed 1 | Seed 2 | Seed 3 |
|---|---|---|---|---|---|
| healthy | 100/0/0 | 100/0/0 | 100/0/0 | 100/0/0 | 100/0/0 |
| 0.20 | 100/0/0 | **38**/62/0 | 100/0/0 | 100/0/0 | 100/0/0 |
| 0.30 | 100/0/0 | **14**/86/0 | 100/0/0 | 100/0/0 | 100/0/0 |
| 0.35 | 100/0/0 | **25**/75/0 | 100/0/0 | 100/0/0 | 100/0/0 |
| 0.40 | 0/100/0 | 0/100/0 | 14/86/0 | 38/50/12 | **62**/38/0 |
| 0.45 | 0/0/100 | 0/14/86 | 0/25/**75** | 0/0/100 | 0/0/100 |
| 0.50 | 0/0/100 | 0/0/100 | 0/0/100 | 0/0/100 | 0/0/100 |
| 0.70 | 0/0/100 | 0/0/100 | 0/0/100 | 0/0/100 | 0/0/100 |

Pooled over the three seeds, with 95% Wilson intervals:

| | Learned (3 seeds) | No recovery | Rule-based |
|---|---|---|---|
| success at 0.20–0.35 | 71/71 = 100% (95–100%) | 24/24 = 100% (86–100%) | 6/23 = 26% (13–46%) |
| success at 0.40 | 9/23 = 39% (22–59%) | 0/8 (0–32%) | 0/8 (0–32%) |
| crash at 0.40 | 1/23 = 4% (1–21%) | 0/8 | 0/8 |
| crash at 0.45 | 22/24 = 92% (74–98%) | 6/6 = 100% (61–100%) | 6/7 = 86% (49–97%) |
| healthy landings commanded | 0 of 45 | 0 of 15 | 0 of 15 |

Median touchdown speed (m/s), for no recovery / rule-based / seeds 1, 2, 3:

| Severity | No recovery | Rule-based | Seed 1 | Seed 2 | Seed 3 |
|---|---|---|---|---|---|
| 0.40 | 1.40 | 0.88 | 1.23 | 0.71 | 0.71 |
| 0.45 | 3.58 | 2.71 | 2.56 | 3.06 | 3.34 |
| 0.70 | 6.73 | 6.81 | 5.00 | 6.88 | 6.55 |

**Against planning.md's validation criterion.** The criterion was: beat the
rule-based controller on success and crash rate, with confidence intervals
that do not overlap across three seeds.

- **Success: met.** At 0.20–0.35, 100% (95–100%) against 26% (13–46%).
- **Crash rate: not met.** No method crashes below 0.40. At 0.45, 92%
  against 86% is a tie at this sample size.

**Against flying with no recovery.** It ties everywhere except 0.40, where
it finishes missions that no recovery only lands. Since no recovery was the
stronger baseline in M8, that comparison is the more informative of the
two.

## Extension: 600 updates (`train_v4`)

Every v3 seed was still improving at update 300, so each was continued to
600 (user decision). `train_v4` is `train_v3` with only the budget changed.
Each run resumes from its v3 seed's final checkpoint, optimiser included.
With a fixed learning rate, that equals training 600 from the start. The v3
runs above stay as the 300-update results.

Training crash rate at 0.35–0.45 (sampled actions, the training fault mix):

| Updates | Seed 1 | Seed 2 | Seed 3 |
|---|---|---|---|
| 250–299 | 23% | 30% | 32% |
| 350–399 | 19% | 8% (see below) | 33% |
| 550–599 | 17% | 2% (see below) | 32% |

**Seed 2 learned to refuse to fly.** From about update 320 it commits to
land within the first second, before take-off, on almost every flight. In
training it landed 93% of healthy flights; in Isaac scoring 78% of healthy
drones never took off.

- A drone that never gets airborne ends "incomplete", worth 0.
- Under the training fault mix (80% of flights faulty, half of those at
  0.3–0.5), flying is worth about break-even on average (roughly −0.1),
  because every crash costs −10 plus its touchdown speed.
- So never flying is as good or better. A longer run gave seed 2 time to
  find that.

This is a flaw in the reward together with the fault mix, and fixing it is
a design decision (see "What to try next"). The low crash rates in the
table above are this exploit, not better recovery.

**Gate, written before any v4 policy was scored**, so Gazebo time is not
spent on a policy that will not fly: a v4 policy is flown on PX4 only if,
in Isaac, it finishes ≥ 98% of missions at every severity from 0 to 0.35.
Seeds 1 and 3 pass; seed 2 does not.

Isaac scores, success / safe landing / crash (`isaac_scores.json` in each
run):

| Severity | Seed 1 v3 → v4 | Seed 3 v3 → v4 |
|---|---|---|
| 0–0.30 | 100/0/0 → 100/0/0 | 100/0/0 → 100/0/0 |
| 0.35 | 100/0/0 → 98/2/0 | 100/0/0 → 100/0/0 |
| 0.40 | 1/99/0 → 3/97/0 | 21/66/12 → 20/66/14 |
| 0.45 | 0/33/67 → 0/43/57 | 0/0/100 → 0/0/100 |
| 0.50 | 0/1/99 → 0/2/98 | 0/0/100 → 0/0/100 |
| touchdown at 0.70 | 4.82 → 4.34 m/s | 6.63 → 6.62 m/s |

- **Seed 1 kept improving.** It now saves 43% of drones at 0.45, up from
  33%. Its mean crash rate over 0.40 and 0.45 fell from 33.5% to 28.5%.
- **Seed 3 did not change at all.**

**On PX4** (`results/m9_px4_v4_seed_{1,3}`), on the same seed-9101
schedule, with the no-recovery and rule-based runs reused. Success / safe
landing / crash:

| Severity | No recovery | Rule-based | Seed 1 v3 → **v4** | Seed 3 v3 → v4 |
|---|---|---|---|---|
| healthy | 100/0/0 | 100/0/0 | 100/0/0 → 100/0/0 | 100/0/0 → 94/0/0 (+1 incomplete) |
| 0.20 | 100/0/0 | 38/62/0 | 100/0/0 → 100/0/0 | 100/0/0 → 88/0/0 (+1 incomplete) |
| 0.30 | 100/0/0 | 14/86/0 | 100/0/0 → 100/0/0 | 100/0/0 → 100/0/0 |
| 0.35 | 100/0/0 | 25/75/0 | 100/0/0 → **75/25/0** | 100/0/0 → 100/0/0 |
| 0.40 | 0/100/0 | 0/100/0 | 14/86/0 → 12/88/0 | 62/38/0 → 57/43/0 |
| 0.45 | 0/0/100 | 0/14/86 | 0/25/75 → **0/50/50** | 0/0/100 → 0/0/100 |
| 0.50 | 0/0/100 | 0/0/100 | 0/0/100 → **0/25/75** | 0/0/100 → 0/0/100 |
| 0.70 | 0/0/100 | 0/0/100 | 0/0/100 → 0/0/100 | 0/0/100 → 0/0/100 |

- **Seed 1 at 600 updates is M9's best policy above the limit.** It
  crashed 10 of 16 flights at 0.45–0.50 (62%, interval 39–82%). No recovery
  crashed 14 of 14 (100%, 78–100%) and the rule-based controller 14 of 15
  (93%, 70–99%). It is the first policy to save any drone at 0.50. It also
  lands softer everywhere: median touchdown 2.14 m/s at 0.45 (no recovery
  3.58) and 4.17 at 0.70 (6.73).
- **It paid for that with caution.** At 0.35 it landed 2 of 8 drones that
  would have finished, so 22 of 24 missions at 0.20–0.35 were completed
  (92%), against 24 of 24 at 300 updates. The intervals at 0.45–0.50 still
  just overlap (39–82% against 78–100%), so M10's 100-flight cells decide
  whether the gain is real.
- **Seed 3 at 600 updates is the same policy as at 300** in the fault band.
  Its two new "incomplete" flights have different causes:
  - At 0.20, PX4's real detector over-read the severity (0.56). The policy
    then flew the rest of the mission at a quarter of full speed and ran
    out of the 120 s mission time. This is M8's over-read making the policy
    too cautious.
  - A healthy flight finished all 5 waypoints, then PX4's own final landing
    timed out. That was after the policy's part, a PX4 landing flake.
- **Transfer holds for crashes.** For the two v4 policies, 15 of 16 Isaac
  crash rates fall inside the PX4 interval. The exception is seed 1 at 0.50
  (98% in Isaac, 75% on PX4), again with PX4 the more forgiving.
  Mission success agrees in 12 of 16 cells, with the same pattern as v3 at
  0.40 plus the three cautious or failed flights above
  (`results/m9_transfer_v4.txt`).

## What to try next (Phase 2 or M10, not done here)

- **Remove the "refuse to fly" option before training longer.** Seed 2 found
  it at about update 320. Three candidate fixes, each a new version file:
  - no land decision before the drone is airborne, on both sides, the same
    rule;
  - a realistic share of healthy flights in training, so flying is clearly
    worth it on average;
  - a negative value for a flight that never took off.
- **A longer training budget, once that is fixed.** Seed 1 improved from
  update 300 to about 450, then levelled off. Seed 3 never moved.
- **Temporally correlated exploration**, or an action held over several
  decisions. The altitude command's per-step noise averages out under its
  rate limit, which is why the descent manoeuvre is hard to find.
- **A filtered Isaac acceleration feature**, to match PX4's sensor pipeline
  (the take-off artifact above).
- **Porting PX4's hover-thrust estimator to Isaac**, if the 0.40 gap should
  close.
