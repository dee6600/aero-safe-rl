# M8 — rule-based recovery baseline: results

Companion to `milestones.md` M8. Every number comes from real flights on the
PX4 + Gazebo stack (2 workers, 1× speed, `x500_aero`, mission
`square_circuit`). To re-derive any table:

```bash
python experiments/run_recovery.py report results/<run_id>
python experiments/tune_fsm.py responses          # sweep + tie-break scoring
```

## Headline

**The tuned rule-based recovery controller does not beat flying with no
recovery at all.** It never landed a healthy drone (14 of 14 healthy flights
completed). It crashed no less often than no recovery at any severity, and it
gave up missions that would have completed at severities 0.2–0.35. This is
reported as it stands. The reinforcement-learning policy in M9 is compared
against this controller exactly as tuned here.

Two findings explain it, and both matter beyond M8:

1. **Physics leaves a narrow window.** The x500 needs 59% of its total thrust
   to hover. So once one rotor has lost more than about 41% of its thrust
   (s\* ≈ 0.41), level hover is impossible. Above s ≈ 0.45 the drone comes
   down hard whatever the controller commands. Below about 0.35 the mission
   succeeds with no help. The only band where a high-level decision can
   change the outcome is roughly s = 0.35–0.45.
2. **The recovery manoeuvre fools the detector.** After confirming a mild
   fault, the controller slows down and descends 3 m. The M7 detector was
   trained only on level mission flights. Within 2–5 s of the descent
   starting, it reads the manoeuvre itself as a worse fault: severity
   estimates rise from 0.2–0.3 to 0.4–0.7. That crosses the controller's
   "land now" cut, so it lands. All 9 aborts at s 0.2–0.3 in the validation
   run began at 4.6–4.9 m altitude, right as the descent started. The
   detector's error therefore depends on what the policy does. The M8b
   detector-output simulator, fitted to M7's level-flight error, does not
   yet capture this.

## Validation run (task 6)

Fresh fault schedule (seed 8201, disjoint from tuning), 8 flights per cell,
16 healthy flights per condition. Faults are single-rotor, step or ramp, with
onset 5–25 s into the mission. Outcomes follow `configs/rl/outcome_v1.yaml`:
a crash is a tilt over 60° or a touchdown faster than 2.0 m/s.

| severity | no recovery: success / safe landing / crash | recovery controller: success / safe landing / crash |
|---|---|---|
| healthy | 100% / 0 / 0 (n 15) | 100% / 0 / 0 (n 14) |
| 0.20 | 100% / 0 / 0 | 57% / 43% / 0 |
| 0.30 | 100% / 0 / 0 | 14% / 86% / 0 |
| 0.35 | 100% / 0 / 0 | 0 / 100% / 0 |
| 0.40 | 0 / 100% / 0 | 0 / 88% / 12% |
| 0.45 | 0 / 0 / 100% | 0 / 0 / 100% |
| 0.50 | 0 / 0 / 100% | 0 / 0 / 100% |
| 0.70 | 0 / 0 / 100% | 0 / 0 / 100% |

Runs: `results/m8_validation_nominal` (70 valid), `results/m8_validation_recovery`
(67 valid). 7 worker restarts across both, all at worker start-up, when the
rotor-fault plugin did not report within its 10 s window. Each lost one
flight; the supervisor recovered as designed.

Median touchdown speed (m/s), no recovery → recovery controller: s 0.40
1.41 → 0.74, s 0.45 2.78 → 2.81, s 0.50 4.02 → 3.70, s 0.70 6.62 → 6.72.
The controller's commanded landing is gentler at 0.40. It makes no
difference where the fall is decided by physics.

Crash-line sensitivity: at 1.5 m/s the no-recovery crash rate at s 0.40
becomes 29% (recovery controller 25%). At 2.5 m/s the recovery controller's
crash rate at s 0.45 becomes 75%. Neither changes the conclusion.

**No panic landings:** across the 14 healthy flights, the recovery controller
briefly became suspicious 4 times and never confirmed or landed.

## How the controller was tuned

States: normal → suspected (slow to half speed) → confirmed, then either
*recovering* (continue at 0.3 speed, 3 m lower) or *aborted* (land now),
depending on the detector's severity estimate against `land_severity`.
Every number is in `configs/rl/fsm_v1.yaml`, with how it was chosen.

**Detection side (offline, task 4).** The M7 detector was replayed over
recorded flights and the controller stepped at 5 decisions per second.
- The first rule (zero false confirmations on the 109 validation flights,
  then the fastest confirmation) chose a 0.4 s hold above p = 0.3.
- The first 6 live healthy flights showed that was too short: one false alarm
  held for 0.72 s at a waypoint turn. The validation flights were too few to
  show how long healthy false alarms can last (up to 1.03 s in the training
  flights).
- The rule was revised before any response tuning: the hold must also exceed
  the longest healthy false-alarm run seen outside the test set, by one
  decision.
- Result: probability ≥ 0.5 held for 1.0 s. Median confirmation is 1.63 s
  after onset at s 0.3–0.5. Details: `results/m8_fsm_tuning/detection_sweep.json`.

**Response side (simulator, task 5).**
- Six settings were flown on one shared fault schedule: the defaults; full
  altitude when recovering; faster when recovering; "land now" cut at 0.35;
  "land now" cut at 0.45; descending already when suspicious. Each covered
  s ∈ {0.35, 0.40, 0.45, 0.50}, with 8 flights per cell.
- The rule, fixed beforehand, was lowest crash rate, then mission success.
- The two leaders differed by one flight, so both were flown 12 more times at
  s 0.35–0.45 (user decision).
- Pooled crash rate: defaults 26/67 (39%), "land only above 0.45" 28/62
  (45%). The defaults were kept.
- Stated trade-off: at s 0.35 the defaults gave up 95% of missions, whereas
  "land above 0.45" completed 89%. That comparison was measured before
  finding 2 above was understood. Details:
  `results/m8_fsm_tuning/response_sweep.json`.

## What this means for M9 and M10

- The comparison band is s ≈ 0.35–0.45. Above it, every method is expected to
  tie at "crash"; below it, at "success". That is a property of the airframe,
  reported with the result.
- Flying with no recovery is a stronger reference than the rule-based
  controller on this mission. M10 should show both.
- A learned policy trained against a detector simulator fitted only to level
  flight will not see finding 2. Either the detector-output simulator in M8b
  must model the error induced by the policy's own manoeuvres, or the
  detector needs training data that includes recovery manoeuvres. Resolved
  in M8b by the first: the simulated detector adds the measured descent
  over-read, and it passes the held-out check (`docs/isaac_env.md`).
