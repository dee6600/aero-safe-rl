# M7 — fault detector results

Companion to `milestones.md` M7. Every number here was produced by
`ai/detector/evaluate.py` on the held-out **test** episodes of
`results/m6_dataset_v1/`, which were read once, after training was finished.
Re-derive them with:

```bash
python ai/detector/train.py    results/m6_dataset_v1 --out results/m7_detector_v1   # ~2 min, GPU
python ai/detector/evaluate.py results/m6_dataset_v1 --out results/m7_detector_v1   # ~3 min
```

Outputs: `results/m7_detector_v1/{detector.pt, train_log.json, metrics.json,
error_model.json, report.md}`.

## Setup

| | |
|---|---|
| episodes used | 729 of 750 (inclusion rules: `milestones.md` M7 task 1) |
| split (by episode, stratified by severity) | 510 train / 109 val / 110 test, seed `20260923`, digest `defe672652cd753d` |
| test set | 73 faulty (21 / 21 / 19 / 12 per severity bucket; 32 step, 41 ramp), 37 without an applied fault, 0.69 healthy flight-hours |
| model | rotor-symmetric streaming GRU, 5-member ensemble, temperature 1.1 (fitted on val) |
| checkpoint | `results/m7_detector_v1/detector.pt`, digest `bec19e4ecee1edd3` |
| alarm rule (all detectors) | score ≥ threshold for 5 consecutive ticks (0.5 s). Threshold = 99.5th percentile of that detector's own score on healthy **val** ticks |

The shared alarm rule means every detector runs at the same per-tick
false-positive rate on healthy validation flight. They are then compared on
what that rate buys each of them.

## Results (test episodes)

### Overall

| detector | tick AUROC | false alarms (per healthy flight-hour) | healthy flights with any alarm | calibration error (ECE) | uncertainty predicts errors (AUROC) |
|---|---|---|---|---|---|
| motor-imbalance threshold | 0.982 | 10 (14.5/h) | 10.8% | — | — |
| residual threshold | 0.404 | 8 (11.6/h) | 8.1% | — | — |
| random forest | 0.997 | **4 (5.8/h)** | **2.7%** | 0.011 | — |
| **rotor GRU ensemble** | **0.998** | 6 (8.7/h) | 8.1% | **0.004** | **0.910** |

### Per severity bucket

| | s 0.2–0.4 | 0.4–0.6 | 0.6–0.8 | 0.8–1.0 |
|---|---|---|---|---|
| **Detection rate** — motor imbalance | 0% | 81% | 100% | 100% |
| random forest | 95.2% | 95.2% | 100% | 100% |
| rotor GRU | 95.2% | 95.2% | 100% | 100% |
| **Median delay (s)** — motor imbalance | — | 10.58 | 3.98 | 1.90 |
| random forest | 1.18 | 0.67 | 0.82 | 0.57 |
| rotor GRU | **0.88** | **0.61** | **0.62** | 0.57 |
| **90th-pct delay (s)** — random forest | 2.28 | 1.25 | 1.58 | 0.82 |
| rotor GRU | **1.67** | **1.15** | **1.11** | **0.72** |
| **Rotor-ID accuracy** — motor imbalance | 98.0% | 92.0% | 89.1% | 93.8% |
| random forest | 98.2% | 94.1% | 93.0% | 95.9% |
| rotor GRU | **98.6%** | **99.7%** | **99.6%** | **99.8%** |
| **Severity MAE** — motor imbalance | 0.184 | 0.114 | 0.090 | 0.201 |
| random forest | 0.020 | 0.075 | 0.082 | 0.116 |
| rotor GRU | **0.018** | **0.018** | **0.022** | **0.013** |

By profile, the GRU's median delay is 0.52 s on step faults and 0.92 s on
ramps. The random forest's is 0.52 s and 1.12 s. Every table, including the
residual baseline's rows and the severity bias, is in
`results/m7_detector_v1/report.md`.

Delays include the 0.5 s hold, so 0.4 s is the fastest possible alarm.

## What the numbers say

1. **The model is ahead on the axes that have headroom.** The dataset probe
   (`milestones.md` M7) predicted where a learned model could win, and it
   does win there:
   - **Faster detection of weak faults:** median 0.88 s vs 1.18 s, 90th
     percentile 1.67 s vs 2.28 s.
   - **Near-perfect rotor identification** at every severity: 98.6–99.8%
     vs 93–96%.
   - **Severity estimates that hold up above s ≈ 0.4:** MAE 0.013–0.022 vs
     up to 0.116. Above that point the motor commands saturate and the
     random forest under-reads by 0.11. The GRU reads severity from how the
     vehicle responds as well as from the command imbalance.
   - **Well calibrated:** ECE 0.004.
   - **An uncertainty output that means something:** ensemble disagreement
     separates wrong fault/no-fault calls from right ones with AUROC 0.91.
2. **Tick AUROC is effectively a tie** with the random forest (0.998 vs
   0.997), as expected: separating settled faults is near the ceiling for
   any detector that reads the motor commands.
3. **The model loses on false alarms**: 6 vs 4 in 0.69 healthy hours
   (3 vs 1 healthy flights affected). The counts are small, but the
   direction is reported as it is. All 6 GRU false alarms are short
   (0.1–0.9 s) and all fall 7–21 s into the flight, around the first
   waypoint turns. For M8: the recovery state machine's hold before
   acting (planned at 1–2 s) is longer than every one of them.
4. **Both learned detectors miss the same 2 faults.** Each became active
   only 0.4–0.5 s before the flight ended, which is shorter than the 0.5 s
   hold. They are effectively undetectable under any alarm rule with a hold.
   Every fault active for longer than the hold was detected.
5. **The motor-imbalance threshold misses every weak fault because of
   takeoff, not because its statistic is weak.** 94% of the healthy
   validation ticks above its threshold (0.34) fall in the first 10 s of
   flight. After 10 s its healthy 99.5th percentile is 0.13. Under the
   shared protocol, takeoff transients set its threshold, and weak faults
   (imbalance ≈ 0.16) never reach it. It was not tuned down: the same rule
   was applied to all four detectors (anti-pattern 13). The learned
   detectors see the same takeoff and learn to discount it.
6. **`thrust_accel_residual` alone is not a detector** (AUROC 0.40, worse
   than chance in its signed form). M5 flagged that its discriminative
   power was unmeasured; it has now been measured, and on its own it is
   uninformative. It stays in the feature vector as one of the model's
   19 inputs.

## Online inference

`DetectorRuntime.step()` (feature extraction + 5-member ensemble, CPU,
one torch thread):

| | median | p99 | max |
|---|---|---|---|
| offline, 4,375 recorded test ticks | 1.03 ms | 1.42 ms | 2.41 ms |
| **live, two workers flying at once** (instance 0 / 1) | 4.0 / 4.4 ms | 9.9 / 10.7 ms | — |

The budget is 20 ms. The first live run broke it (p99 34.5 ms): when the
five members ran one after another, per-operation overhead dominated a tick,
and two simulators competing for CPU multiplied it. The runtime now steps
all members at once through stacked weights (`StreamingEnsemble`). Streaming
output still equals batch output (to 1e-7 on the trained weights;
`tests/test_detector_runtime.py`).

**Live check** (`tests/sim/test_detector_live.py`, two concurrent workers,
1×):

| worker | fault | detection | pre-onset alarms |
|---|---|---|---|
| instance 0 | rotor 1, s = 0.5 | 0.53 s after onset, correct rotor | none |
| instance 1 | rotor 3, s = 0.3 | 0.59 s after onset, correct rotor | none |

## For M8b: the error model

`results/m7_detector_v1/error_model.json` is what M8b's detector-output
simulator is fitted to. It is measured on the test split and contains:

- the raw detection delays per severity bucket (`null` = missed)
- detection rate, rotor-ID accuracy, severity bias and error std per bucket
- false alarms per healthy hour, and the share of healthy flights with any
  alarm
- p(fault) and uncertainty quantiles on healthy ticks and on fault-active
  ticks per bucket

Healthy-tick p(fault) is small (median 0.014, 95th percentile 0.043).
Fault-active p(fault) is near 1 once detected (median > 0.9999 in every
bucket). The detector's behaviour is dominated by *when* it switches, not by
a noisy level, so M8b should model a delay distribution plus rare short
false alarms rather than Gaussian noise on the true severity.

## Limits

- One mission (`square_circuit`), one airframe, one fault type, flown at
  1× speed. Every number here is in-distribution on trajectory. M11 measures
  generalisation.
- 110 test episodes. Per-bucket rates rest on 12–21 episodes each, and the
  false-alarm comparison on 4–6 events.
