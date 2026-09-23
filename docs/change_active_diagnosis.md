# Change proposal — active fault diagnosis and belief-driven recovery

**Status: DEFERRED until after the MVP (decided 2026-09-23).** Nothing in this
document is part of the current plan, including Part A. The project finishes
its MVP first (M7–M10 as already planned in `milestones.md`; M7 is now done); this proposal is
kept as the starting point for a future update. When it is picked up, re-check
it against the repository first, since it describes the state as of 2026-09-23.

**Original status: proposal, not approved. Date: 2026-09-22. Revised 2026-09-23** after
checking it against the repository: M5 is committed, the M6 plugin exists and
already couples torque to thrust, and the 750-episode M6 dataset
(`results/m6_dataset_v1/`) is delivered without probes. The revision corrects
the parts written against the older state, fixes three technical claims, and
splits adoption into two parts (§0.1).
Companion to `planning.md` (§2, §3.1, §6, §7.2, §8, §9, appendix), `milestones.md`
(M5, M6, M7, **M7b new**, M8b, M9, M10) and `CLAUDE.md` (§1).

**What this changes.** The project currently treats detection as passive: the
detector reads whatever telemetry the mission happens to produce, and the policy
consumes whatever the detector says. This proposal closes that loop in the other
direction — the policy may spend a small amount of mission performance to *make
the fault observable*, and is rewarded for the belief it buys.

**What this does not change.** M0–M4 are untouched. PX4 stays stock. The action
space stays high-level and the low-level controller is still never ours. Every
reported number still comes from the PX4-in-the-loop stack (principle #13). D12
stands: training in Isaac, evaluation on PX4.

**Nothing already measured is invalidated.** The M3 noise floor, the divergence
band, the M3b Isaac feasibility numbers and the M4 throughput table all survive.

**One bookkeeping note before anything else** *(resolved 2026-09-23 — all three
status blocks now reflect M4–M6 as done)*: the status blocks were stale in
three places. `planning.md` §0 still says "M4 tasks 4–8 remain" and `README.md`
still says "M4 tasks 1-3 done, 4-8 remaining", although commit `35b2fba` landed
them and `milestones.md`'s own progress log marks M4 done. That progress log and
`README.md` also list M5 and M6 as "Not started", although M5 is committed
(`b999e82`) and M6's dataset is delivered (`docs/fault_dataset.md`). Reconcile
all three before adding the sections below, or the change history stops being
trustworthy.

---

## 0. The change in four parts

1. **RQ3 is sharpened** from a sensitivity sweep into a falsifiable claim about
   *which* detector property drives closed-loop outcome, and **RQ6 is added** for
   active diagnosis. RQ1, RQ2, RQ4 and RQ5 keep their numbers and their text, so
   nothing already written has to be renumbered.
2. **`action_v1` gains a probe channel** (D13) — a commit signal plus an
   intensity, triggering a bounded, pre-defined diagnostic primitive, gated by a
   safety guard that lives outside the policy.
3. **The M8b detector stub is replaced by a portable detector** that actually
   runs on both sides (D14). A fixed noise model cannot represent "information
   depends on what you do", and a stub that *does* model that effect explicitly
   would make RQ6 circular.
4. **A kill-test is inserted before any of it is built** (M7b), in two stages.
   Stage 0 flies scripted probes with the existing plugin and compares raw
   telemetry — no new detector, no dataset rerun, about one day. Only if that
   shows an effect does stage 1 repeat it with the belief-output detector.

M5 already anticipated most of part 3: the shared / PX4-only feature split it
defines is exactly the boundary the portable detector lives inside.

### 0.1 Adoption in two parts

**Part A — adopt now.** Cheap, and useful whether or not probing works:

- the RQ3 sharpening (§1);
- D15, the calibrated belief head, built as part of M7, with the matched-AUC
  detector variants it makes possible (§9, M7 addendum);
- recording the yaw-rate response in the M6 fault fixture, so the Isaac-side
  model must match it as well as the thrust curve (§2, D2 note);
- the status-block fixes above.

**Part B — adopt only if M7b stage 0 passes.** RQ6, D13, D14, D14a, D16, the
probe channel, the probe dataset run, C7–C9, schema v3 and the two new
`CLAUDE.md` rules. Until then these sections stay a proposal. D14 also needs
the environment-boundary question in §3.2 settled before it is adopted.

---

## 1. Research questions — drop-in replacement for `planning.md` §2

Replace the sub-question list and the novelty framing with the following. RQ3 is
rewritten; RQ6 is new; RQ1, RQ2, RQ4, RQ5 are unchanged and are reproduced only
so the block can be pasted whole.

---

Sub-questions the experiment design must be able to answer:

- **RQ1 (Detection)** — How accurately and how quickly can a learned detector
  identify partial actuator degradation from PX4 telemetry alone, at severities
  below PX4's own failure-detector threshold?
- **RQ2 (Recovery)** — Given a fault estimate, does a learned high-level policy
  outperform a hand-tuned rule-based policy on mission success and safety?
- **RQ3 (What a detector owes a policy)** — *Which* property of a detector
  determines closed-loop recovery outcome: ranking accuracy (AUC), calibration,
  or latency? The hypothesis this project sets out to falsify is that **AUC is
  the weakest of the three predictors**, and that a lower-AUC but well-calibrated
  detector beats a higher-AUC overconfident one, because a policy can act on an
  honest probability and cannot act on a confident wrong one.
- **RQ4 (Generalization)** — Does the policy transfer to unseen fault
  severities, timings, wind, and vehicle parameters?
- **RQ5 (Sim-to-sim transfer)** — Does a high-level recovery policy trained in a
  massively-parallel *reduced-order* simulator transfer to a full
  autopilot-in-the-loop stack, and what is lost in the crossing? (§3.1, D12)
- **RQ6 (Active diagnosis)** — A partial rotor fault is partly *masked* by PX4's
  own control allocation: in steady flight the allocator redistributes thrust and
  the signature shrinks. The fault's observability therefore depends on what the
  vehicle is doing. Can a high-level policy — restricted to an unmodified
  autopilot's setpoint interface — learn **when** to spend mission performance on
  a diagnostic maneuver, and does *selective, learned* probing beat both no
  probing and fixed-schedule probing on detection latency per unit mission cost?
  (§7.2, D13)

### Framing the novelty honestly

PX4 already contains a `FailureDetector` and control-allocation-based actuator
failure handling. A paper claiming "RL beats PX4 at motor mixing" would be weak
and hard to defend. The defensible contribution is at a **different layer**:

1. **Sub-threshold, partial degradation** — PX4 handles binary motor loss
   reasonably; it does not reason about a rotor at 60% effectiveness.
2. **Mission-level recovery decisions** — continue, degrade speed, re-plan,
   loiter, or land, *given an uncertain fault estimate*.
3. **The detection–recovery coupling, in both directions (RQ3, RQ6)** — not only
   how detector error propagates into recovery, but how recovery actions
   determine what the detector can see. The existing literature on active fault
   diagnosis for multirotors injects hand-designed excitation at the motor or
   allocator level, which requires modifying the flight stack and is
   conventionally regarded as difficult on an under-actuated quadrotor. Doing it
   as a *learned, budgeted, mission-level* decision through a stock autopilot's
   setpoint interface is, as far as this project's literature review has found,
   open. **Confirm that with a proper search before the paper claims it**
   (keywords: active fault diagnosis, auxiliary input design, dual control,
   information-gathering POMDP, excitation for diagnosability).
4. **Honest sim-to-sim accounting (RQ5)** — training in Isaac Lab and reporting
   *only* PX4-in-the-loop numbers turns an unasked question into a measured one.
5. **A task-relevant standard for detector quality (RQ3)** — detectors in this
   literature are graded with AUC and F1, which are properties of a ranking, not
   of a decision. If calibration and latency dominate closed-loop outcome, the
   field is optimising the wrong number, and that is a result worth stating
   plainly.

---

## 2. New decisions — append to `planning.md` §14

---

### Added 2026-09-22 — active diagnosis

| ID | Decision |
|---|---|
| **D13** | **The action space gains a probe channel.** `action_v1` carries a probe commit signal and an intensity; a commit triggers one bounded, pre-defined *probe primitive* rather than a policy-synthesised waveform. A **probe guard**, implemented once outside the policy and identical in both simulators, may refuse a probe. Denials are logged. |
| **D14** | **Two detectors, one interface.** `detector_px4_v1` (all M5 features, PX4 side only) is the RQ1 result. `detector_portable_v1` (shared features only) is what the *policy* consumes, and it runs on **both** sides. This replaces M8b's synthetic detector-output simulator as the reported configuration. The synthetic stub survives only as a training-side sanity check (D14a) and never produces a reported number. |
| **D14a** | The synthetic detector-output simulator is demoted to **one purpose**: confirming that PPO can learn to probe *at all*, in an environment where the benefit of probing is explicitly modelled. It is a check on the RL setup, run before the real thing is built. A result produced with it is circular by construction and is never reported. |
| **D15** | **The detector outputs a belief, not a point estimate.** A posterior over `K = 9` classes — healthy, plus eight severity bins over (0.1, 0.9] — calibrated by temperature scaling on a held-out split, with ECE reported. Entropy, NLL and Brier score all derive from it. The policy observes the belief; `p(fault)` and the severity estimate become derived quantities rather than the primary interface. |
| **D16** | **The information reward is a proper scoring rule, not entropy.** Rewarding entropy reduction pays the policy for becoming confident, including confidently wrong. The shaping term is the change in the score of the *true* class under the belief, which pays only for becoming correctly confident. Ground truth enters the reward only — never the observation (`CLAUDE.md` §1.7). The term is potential-based and **unclipped**, so it cannot change the optimal policy of the underlying mission reward (§5 explains what that implies). |

**Adoption:** D15 is Part A (§0.1). D13, D14, D14a and D16 are Part B and are
appended to `planning.md` §14 only if M7b stage 0 passes.

### Note on D2 — torque coupling (checked 2026-09-23)

The yaw signature depends on the fault weakening the rotor's **reaction torque
as well as its thrust**, and the Isaac-side model must do the same.

**The M6 plugin already does this.** `RotorDegradationSystem.cc` scales the
faulted rotor's commanded velocity by `sqrt(1 − s)`. In the stock gz motor
model thrust goes with velocity² and reaction torque with thrust (via
`momentConstant`), so both scale by exactly `1 − s`. No plugin change and no
new config field is needed.

A configurable `torque_factor` is **not** added. It would only serve fault
types (ESC derating, bearing wear) that nothing in the plan calls for. If one
is ever added, add the field then (see the project's keep-it-simple rule).

What *does* change: the `CLAUDE.md` §1.6 cross-validation fixture records the
**yaw-rate response to a commanded yaw doublet** as well as the thrust
reduction, so the Isaac-side model is held to both. This is Part A.

---

## 3. New subsection — insert as `planning.md` §3.2

---

### 3.2 The belief pipeline (D14, D15)

```
   PX4 side (evaluation, every reported number)        Isaac side (training only)
┌───────────────────────────────────────────┐   ┌──────────────────────────────────┐
│ all M5 features ──► detector_px4_v1       │   │                                  │
│   (allocation residual, EKF innovations,  │   │                                  │
│    per-motor outputs, + shared)           │   │                                  │
│            └─► RQ1 result, reported       │   │                                  │
│                                           │   │                                  │
│ shared features ──► detector_portable_v1 ─┼───┼─► detector_portable_v1 (retrained│
│                          │                │   │    on Isaac data, same arch,     │
│                          ▼                │   │    same feature names)           │
│                    belief b_t (K=9)       │   │           ▼                      │
│                          │                │   │     belief b_t (K=9)             │
│                          ▼                │   │           ▼                      │
│                   policy (5 Hz) ──────────┼───┼──── policy (5 Hz) — one weights  │
└───────────────────────────────────────────┘   └──────────────────────────────────┘
```

Two detectors, one **interface**: both emit a `K`-way belief over the same
severity bins, defined in `configs/ai/belief_v1.yaml`. The policy's observation
contract is the belief, not the model that produced it.

**Why the stub had to go.** M8b's original design fed the policy true severity
through a noise/latency/false-positive model fitted to M7's measured error. That
model is a function of severity and time. Under RQ6 the quantity that matters is
a function of severity, time **and the vehicle's recent actions** — a probe makes
the fault observable. A fixed stub cannot express that, so the policy would never
learn to probe. A stub *extended* to express it would be told, by us, exactly how
much probing helps, and would then "discover" it. That is circular, a reviewer
will say so, and they will be right.

**What it costs.** The two detectors will not have identical error
characteristics, so the RQ5 gap now contains a detector component as well as a
dynamics component. That is a price worth paying because it is *measurable*:
report the transfer table with an extra row in which the Isaac policy is
evaluated against an oracle belief on both sides, which separates "the dynamics
differ" from "the detectors differ". The alternative — an unmeasurable circular
result — is not cheaper, it is just quieter.

**What it costs nothing.** Principle #12 and `CLAUDE.md` §1.7 are unaffected:
the policy still never sees ground truth on either side. The belief is a model
output in both environments, which is strictly *more* faithful than the stub was.

**Two problems to settle before D14 is adopted.**

1. **It needs a second implementation of feature extraction.** The detector in
   Isaac needs the M5 features computed from batched GPU tensors. The existing
   extractor (`ai/features/feature_extractor.py`) works on PX4 episode records
   in the `aero-safe-rl` env and cannot be imported across the boundary
   (`CLAUDE.md` §0.1). An Isaac-side extractor is therefore a second
   implementation, which `CLAUDE.md` §1.4 forbids unless sanctioned. Adopting
   D14 means adding it as a second sanctioned exception beside the fault model,
   paid for the same way: a test asserting that both extractors produce
   matching features from a recorded fixture.
2. **"Commanded" means different things on each side.** On PX4, the command
   behind any "commanded vs achieved" residual comes from PX4's own position
   and rate controllers. In Isaac it comes from the geometric stand-in
   controller. The same residual name would measure two different
   controllers, which puts a controller gap inside the belief. Portable
   features must use quantities that mean the same thing on both sides: raw
   vehicle state, or residuals against the **policy-level** setpoint (the one
   input both sides share by construction), never against an inner-loop
   command.

---

## 4. `planning.md` §6 addendum — fault model

Add after the D2 note:

---

**Torque coupling (D2, checked 2026-09-23).** Already satisfied by the plugin
(see the D2 note in §2). The M6 validation gains one assertion: at `s = 0.3` on
rotor `i`, a commanded yaw-rate doublet produces a yaw response that **differs
from the healthy response** by more than the M3 noise floor. Expect the
difference to show as a weaker yaw response overall and as yaw leaking into
roll and pitch (the weak rotor delivers less than its share of a yaw
manoeuvre). **Do not assert that the +/− directions respond differently.**
Thrust and torque fall together, so for small doublets the response is close to
symmetric. It only becomes lopsided near motor saturation, so that assertion
could fail on a correct plugin. If the healthy-vs-faulty difference is missing,
either the severity is not reaching the physics or the doublet is too small.

**Dataset composition (Part B).** The delivered M6 dataset
(`results/m6_dataset_v1/`, 750 episodes) has no probes and **is not
regenerated**. If M7b stage 0 passes, a **supplementary** run
(`results/m6_probe_v1/`, same fault config, same farm) adds episodes with
scripted probes at randomised times, healthy and faulty, so that probe episodes
make up 30–40% of the combined set. Without them the detector sees probe
telemetry for the first time at evaluation, out of distribution, at exactly the
moment it is supposed to be most informative — and the resulting failure looks
like "probing doesn't help". Budget the run from `docs/fault_dataset.md`: the
750-episode run took several sessions at 2 workers × 1×.

---

## 5. `planning.md` §7.2 — replacement for Observation / Action / Reward

---

**Observation** (~50–70 dims, all normalised, all computable in both
simulators):

- Vehicle state: attitude, angular rates, body velocity, position error vs
  setpoint, altitude AGL.
- Mission context: distance/bearing to next waypoint, waypoints remaining,
  elapsed time, battery.
- **Belief** (D15): the `K = 9` posterior over severity classes, its normalised
  entropy `H(b_t)/log K`, and the time since the belief last changed class.
- **Probe state** (D13): probes remaining in budget (normalised), time since last
  probe (normalised, saturating), and a single `probe_available` flag reflecting
  the guard's current verdict.
- Short history of the above (stack ~4 frames) to expose trends.

> The policy sees the *belief*, never ground truth. Ground-truth fault state may
> be used for reward shaping during training only — never as an input.

**Action** — continuous, 7 dims:

| # | Action | Range | Note |
|---|---|---|---|
| 1 | Max horizontal velocity scale | [0.1, 1.0] | unchanged |
| 2 | Max climb/descent rate scale | [0.1, 1.0] | unchanged |
| 3 | Altitude setpoint offset | [−5, +5] m | unchanged |
| 4 | Mission-progress rate / hold | [0, 1] | unchanged |
| 5 | Abort–land commitment | [0, 1] | thresholded |
| 6 | **Probe commit** | [0, 1] | thresholded; triggers one probe primitive |
| 7 | **Probe intensity** | [0, 1] | scales the primitive's amplitude within its bounds |

**Probe primitives** (`configs/rl/probe_v1.yaml`). A commit triggers a
pre-defined, bounded, ~1–1.5 s sequence rather than a per-step waveform, for
three reasons: 5 Hz is too slow to shape a useful excitation; a fixed primitive
is identical in both simulators by construction; and a bounded primitive is
auditable, which a policy-synthesised excitation is not.

| Primitive | Command | Rationale |
|---|---|---|
| `yaw_doublet` (**v1**) | yaw-rate setpoint `+ω_p` then `−ω_p`, `T_p/2` each, position setpoint held | a degraded rotor delivers less than its share of the yaw torque, so yaw response weakens and leaks into roll/pitch (§4); the vehicle does not translate, so the probe is cheap and does not risk a position excursion |
| `climb_pulse` (deferred to M11) | vertical velocity `+v_p` for `T_p`, then return | exposes the reduced thrust ceiling; costs altitude and energy, so it is an ablation, not v1 |

Defaults: `ω_p ∈ [0.2, 0.6] rad/s` scaled by action 7, `T_p = 1.2 s`,
`N_max = 6` probes/episode, refractory `t_ref = 3 s`.

**Flight-code prerequisite.** Nothing in `aero_bridge` sends a yaw-rate setpoint
today. The doublet needs `PX4Interface` to publish `TrajectorySetpoint.yawspeed`
while holding position. This is also the first thing M7b stage 0 needs.

**The probe guard.** One rule set, in the shared command-mapping layer, used by
the FSM and the RL policy and identical in both environments. The boundary
(`CLAUDE.md` §0.1) means "identical" cannot be "one module imported by both
sides": the PX4 side runs it per vehicle in the `aero-safe-rl` env, the Isaac
side batched over tensors in the `isaacsim` env. It is therefore defined by
`configs/rl/probe_v1.yaml` (thresholds) plus a shared table of recorded
states → verdicts that **both** implementations are tested against. A probe is
refused when any of the following holds: altitude AGL below `h_min`; attitude
error or body rate above threshold; horizontal geofence margin below `d_min`;
battery below threshold; within the refractory window; budget exhausted;
land-commit already active. **Refusals are logged as events, not discarded** — a
policy that repeatedly asks for a denied probe is telling you your guard is
mistuned, and that is invisible if only granted probes are recorded.

Deliberately *not* in the guard for v1: blocking a probe when the belief is
already confident. Letting the policy learn that by itself is one of the
results — a probe rate that falls as belief entropy falls is the figure that
shows selectivity was learned rather than imposed.

**Reward** — sparse terminal + dense shaping:

- `+R_success` mission completed
- `−R_crash` crash / geofence breach / uncontrolled descent
- `+r_progress` per-step waypoint progress
- `−λ₁ ·` tracking error, `−λ₂ ·` control effort/energy, `−λ₃ ·` attitude deviation
- Small `−r_time` to discourage indefinite loitering
- **Safe-landing partial credit**
- **`+r_info` — belief shaping (D16).** With `Φ_t = log b_t[k*]`, the log-score of
  the true severity class under the current belief, the term is
  `r_info,t = β · (γ Φ_{t+1} − Φ_t)`, with `Φ = 0` at episode end. This is
  potential-based, so it provably leaves the optimal policy of the rest of the
  reward unchanged. **Do not clip it**: clipping breaks the potential-based
  form and with it that guarantee. If log-score is unstable near `b[k*] → 0`,
  switch to Brier score, which is bounded, rather than clipping. Whichever is
  used, it is a **proper scoring rule over the true class**, never the belief's
  entropy. Entropy alone pays for confidence, and the cheapest way to buy
  confidence is to be wrong about it.

  **What this term can and cannot do.** Because it cannot change the optimal
  policy, it cannot by itself make probing worthwhile. It only helps the
  learner find informative behaviour sooner. Probing is worth doing only if a
  better belief leads to better mission outcomes under the rest of the reward.
  That is the claim RQ6 tests. If the policy probes only because `r_info` pays
  for it, the term is doing something it provably cannot do in the limit, and
  the probing will fade with more training.
- **`−λ_probe` per probe initiated.** Small and explicit, so the mission cost of
  probing appears in the reward decomposition rather than only in the tracking
  error it happens to cause.

All weights live in `configs/rl/*.yaml`. `r_info` and `λ_probe` are logged as
separate reward components (M9), because a policy that probes constantly is the
most likely reward-hacking outcome of this design and must be visible as such.

---

## 6. `planning.md` §8 — condition matrix additions

Keep C1–C6 exactly as they are: C4 stays the *non-probing* RL policy, so the
original RQ2 comparison against the FSM is preserved unchanged. Add:

| # | Condition | Fault | Detection | Recovery | Purpose |
|---|---|---|---|---|---|
| C7 | **Detection + RL with learned probing** | Yes | portable belief | RL, probe channel live | the RQ6 headline arm |
| C8 | **Detection + FSM with fixed-schedule probing** | Yes | portable belief | FSM + probe every `T_probe` s while SUSPECTED | the honest probing baseline — "just probe periodically" is the first thing a reviewer will ask about |
| C9 | **C7's policy, probe channel clamped at test** | Yes | portable belief | RL, probe disabled | how much of C7's advantage is the probing itself rather than the training it received |

**Scope.** C7–C9 run only at `s ∈ {0.2, 0.3, 0.4}` plus the healthy cell —
probing is a sub-threshold story, and at `s ≥ 0.6` the fault is obvious without
it. That is 4 cells × 3 conditions × 100 episodes = **1,200 extra episodes**,
which the M4 throughput table can price exactly. The healthy cell is not
optional: it is where the **false-probe rate** is measured, and a policy that
probes a healthy aircraft is paying mission cost for nothing.

---

## 7. `planning.md` §9 — metrics additions

Under **Detection**, add:

- Belief quality over time: NLL and Brier score of the true class, and
  **calibration (ECE, reliability diagram)** — per severity. These are RQ3's
  primary instruments.
- Detection latency **with and without probing**, at `s ∈ {0.2, 0.3, 0.4}`.

New group, **Active diagnosis**:

- Probes per episode; probe duty cycle (fraction of flight time inside a probe).
- **False-probe rate** on healthy flights, per flight-minute.
- **Information efficiency**: change in belief NLL per probe, and per second of
  mission time spent probing.
- **Mission cost of probing**: Δ time-to-completion and Δ position RMSE, C7 vs C9
  at matched severity.
- **Probe-induced loss-of-control rate** — episodes where the departure follows a
  probe within `t` seconds. Reported even if zero; especially if zero.
- **Selectivity**: probe rate as a function of belief entropy. The headline RQ6
  figure is this curve next to C8's flat fixed-schedule line.

---

## 8. `planning.md` appendix — risk table additions

| Risk | Severity | Mitigation |
|---|---|---|
| Information reward is hacked — policy becomes confidently wrong | **High** | D16: proper scoring rule over the true class, not entropy; `r_info` logged separately; belief calibration (ECE) reported per condition |
| Circular RQ6 result from an explicitly-modelled information stub | **High** | D14a: the stub is a sanity check on the RL setup only and never produces a reported number; every reported probing result uses a real detector in the loop |
| Isaac-side rotor model scales thrust but not torque, erasing the yaw signature on the training side only | **High** | The Gazebo plugin already couples them (D2 note); the fixture records the yaw-doublet response, and M8b's §1.6 test holds the Isaac model to it as well as to the thrust curve |
| Isaac-side feature extraction drifts from the PX4-side extractor | **High** | Sanctioned second implementation (§3.2) with a parity test on a recorded fixture, like §1.6; portable features restricted to quantities that mean the same on both sides |
| Probing causes the very failure it is diagnosing | **High** | Probe guard outside the policy, bounded primitive amplitude, budget and refractory; probe-induced LOC rate reported as a first-class metric; `yaw_doublet` chosen partly because it holds position |
| Detector is out-of-distribution during probes | **High** | M6 dataset contains scripted probes in 30–40% of episodes, healthy and faulty |
| Two detectors widen the RQ5 gap and confound it | Medium | Oracle-belief row in the transfer table separates the dynamics gap from the detector gap |
| Action space growth hurts the sample budget | Low | +2 dims on 5, and the probe dims are near-binary in effect; Isaac's throughput has the headroom M3b measured |
| RQ6 turns out to be physically unavailable at sub-threshold severities | Medium | **M7b stage 0 kills it in about a day, before any Part B work is built.** The RQ3 sharpening costs nothing extra and survives on its own |

---

## 9. `milestones.md` changes

### M5 — addendum (Part B; M5 is already committed)

M5 shipped `feature_version: "1"` with 13 shared features, all raw vehicle
state, and no commanded quantities. `thrust_accel_residual` is **`px4_only`**,
not shared. Adding features now means `feature_version: "2"` and recomputing
`configs/rl/normalization_v1.yaml` as `normalization_v2.yaml`. That is allowed
because no model has been trained on v1 yet (anti-pattern #11 applies only
after training starts), but it is a version bump, not an in-place edit
(`CLAUDE.md` §7).

Candidate shared features, because they are the ones a probe excites. Per
§3.2, residuals are taken against the **policy-level setpoint** both sides
share, never against an inner-loop command:

- achieved **yaw rate** against the probe's commanded yaw-rate profile during a
  probe, and its per-window gain;
- **angular acceleration** per body axis, including roll/pitch activity during
  a yaw probe (the cross-axis leak described in §4);
- vertical acceleration against the policy's climb-rate setpoint.
  `thrust_accel_residual` stays `px4_only`.

New test: everything `configs/ai/detector_portable_v1.yaml` references is marked
shared in `configs/features.yaml` — the same test that already guards
`observation_v1.yaml`, pointed at the portable detector's input list.

### M6 — addendum

- *(Part A)* Fixture records the yaw-doublet response as well as the thrust
  reduction, so M8b has both to match. No schema or plugin change: torque is
  already coupled (§2 D2 note).
- *(Part A)* Validation gains the healthy-vs-faulty yaw-response assertion at
  `s = 0.3` (§4), not an asymmetry assertion.
- *(Part B)* The supplementary probe dataset run, with `probe_events` in the
  record (§4). The delivered 750-episode dataset is kept as is.

### M7 — addendum

*(Note, 2026-09-23: M7 shipped without this addendum. Its detector is a 5-way
{healthy, rotor 0..3} softmax plus a severity regression, from a
temperature-scaled 5-member ensemble; ECE is reported (0.004), but there is no
K = 9 severity-class head and no reliability diagram. Re-plan this addendum
against `ai/detector/` and `docs/detector_results.md` when the proposal is
picked up.)*

- The model's head becomes a `K = 9` softmax over severity classes (D15), not a
  scalar plus a hand-made uncertainty proxy. Temperature-scaled on a held-out
  split; **ECE and a reliability diagram reported alongside AUC**.
- *(Part B)* Train **two** models from one training script and one
  architecture: `detector_px4_v1` (all features — the RQ1 result) and
  `detector_portable_v1` (shared features only — what the policy consumes).
  Report both; the gap between them is itself informative about what the
  PX4-only features are worth. Under Part A alone, only `detector_px4_v1` is
  trained, with the belief head.
- The measured error model is still fitted, since D14a's sanity stub needs it.
- **RQ3's detector variants are generated here, and cost almost nothing:**
  temperature scaling is monotonic, so a mis-scaled variant has **exactly the
  same AUC** and different calibration — a matched-AUC arm by construction.
  Noise-injected variants give the reverse (lower AUC, re-calibrated). That is
  RQ3's whole experimental design, produced from one trained model.

---

### M7b — Probe observability spike  ⭐ new (gate for RQ6)

**Goal:** answer, without RL, whether a scripted probe actually makes a
sub-threshold fault more observable **on the PX4 stack**. This is the physical
assumption RQ6 rests on. Two stages, cheapest first. All of Part B waits on
stage 0.

**Blocks:** every Part B item (§0.1) — and nothing else.

#### Stage 0 — raw-telemetry spike (about one day)

**Depends on:** M6's existing plugin and farm only. No new detector, no dataset
run, no feature-version bump.

- Add yaw-rate setpoint support to `PX4Interface` (§5, flight-code
  prerequisite).
- Fly a scripted `yaw_doublet` through `EpisodeRunner` at `s ∈ {0, 0.2, 0.3}`,
  ≥30 episodes per cell, at least two concurrent workers, probe timing
  randomised, with matched no-probe control episodes at identical seeds.
- Compare healthy vs faulty separability in the 2 s window after the probe
  against the same window without one, using the existing M5 features plus the
  roll/pitch activity during the doublet. A small classifier fitted per window
  (e.g. logistic regression, cross-validated by episode) is enough. Report the
  AUC gain with a 95% CI.

**Pass:** the post-probe window separates healthy from faulty better than the
no-probe window at `s = 0.2` **and** `s = 0.3`, with non-overlapping 95% CIs.
Then Part B is adopted and stage 1 runs.

**Kill:** no significant gain at any tested severity. Then RQ6 is dropped, D13
is marked superseded, `action_v1` stays at 5 dims, and the project continues
with Part A only, having spent about a day. **Write the negative result into
`docs/probe_observability.md` anyway** — "the allocator masks it well enough
that mission-level excitation buys nothing" is a genuine finding about PX4, and
it belongs in the paper's discussion.

#### Stage 1 — detector-level confirmation

**Depends on:** stage 0 passed, the supplementary probe dataset (§4), M7's
`detector_portable_v1`.

- Rerun stage 0's protocol at `s ∈ {0, 0.2, 0.3, 0.4}`, scoring the belief
  instead of the raw features.
- One figure: belief NLL and severity MAE in the 2 s window after the probe,
  against the matched no-probe control.

**Pass criterion:** at `s = 0.2` **and** `s = 0.3`, the post-probe belief NLL is
lower than the matched control by a margin exceeding the M3 divergence band, with
a non-overlapping 95% CI over ≥30 episodes.

**Watch out for:** running stage 1 with a detector trained on probe-free data.
It will fail for the wrong reason. Check that the supplementary probe run went
into the detector's training split first.

**Deliverable for both stages:** `docs/probe_observability.md`, written like
`docs/isaac_feasibility.md` — the measurement, the method, and the verdict.
**Done when** the effect is quantified with a confidence interval, whichever way
it comes out.

---

### M8b — Isaac Lab training environment (rewritten under D14)

**Goal:** the environment the policy trains in — N parallel quadrotors on GPU,
implementing the same frozen observation/action spec as the PX4-in-the-loop
evaluation environment, **and producing a belief the same way the evaluation
environment does**.

**Depends on:** M3b, M5, M6, M7, **M7b**. **Blocks:** M9.

Split into two stages, cheapest first.

**M8b.1 — environment and spec parity.**
- Isaac Lab quadrotor task, geometric position/velocity controller, 5 Hz.
- Isaac-side rotor degradation model with **thrust and torque** scaling,
  cross-validated against M6's fixture on both quantities (`CLAUDE.md` §1.6).
- The v1 probe primitive implemented against the same
  `configs/rl/probe_v1.yaml`, and the probe guard implemented batched for Isaac
  and tested against the **same recorded states → verdicts table** as the PX4
  side (§5). It cannot be one imported module across the boundary.
- The Isaac-side feature extractor (§3.2), with its parity test against the
  PX4-side extractor on a recorded fixture.
- One shared test asserting both environments match `observation_v1.yaml` and
  `action_v1.yaml`.
- **D14a sanity check:** a short PPO run against the synthetic
  detector-output stub, extended so that its noise shrinks after a probe. The
  only question it answers is "can PPO find the probe behaviour when the benefit
  is handed to it?" If no, the RL setup is broken and that is far cheaper to
  learn here. **This run produces no reported number** and its checkpoints are
  marked `sanity_only` so they cannot be evaluated by mistake.

**M8b.2 — the portable detector in the loop.**
- `detector_portable_v1` retrained on Isaac telemetry, same architecture, same
  shared feature names, same `K = 9` belief interface.
- Runs inside the Isaac env at 10 Hz (batched across envs on GPU — budget it
  against M3b's throughput before committing; a per-env Python detector call will
  destroy the sample budget, so it must be one batched forward pass).
- **The new acceptance test — cross-simulator probe response.** Run M7b's
  scripted probe protocol in Isaac. Require that the post-probe belief-NLL
  improvement (a) exists, (b) has the same sign, and (c) falls within a stated
  band of the PX4-side effect measured in M7b. If Isaac shows a large effect and
  PX4 shows a small one, the policy will learn to probe for information that does
  not exist on the evaluation side, and RQ5 will report it as a transfer gap when
  it is a modelling error. This test is to RQ6 what §1.6's fixture test is to the
  fault model.

**Non-negotiable:** no policy that will produce a reported number is trained
against the synthetic stub (D14a). The stub answers one question and is then
retired.

**Watch out for:** Isaac making principle #12 trivially easy to violate — true
severity is a variable in scope there, and it is now *also* legitimately in scope
for the reward (D16). Assemble the observation from the spec; keep the reward's
ground-truth access in one clearly named function.

---

### M9 — addendum

- Train **two policy families from the same seeds**: `rl_noprobe` (action 6–7
  clamped, this is C4) and `rl_probe` (C7). Same everything else. Without the
  matched pair, every RQ6 number is confounded with ordinary training variance.
- Domain randomisation adds: probe guard thresholds, and the belief's latency and
  error — now arising from a real model rather than an injected one, so randomise
  the *detector's* training conditions instead where possible.
- Reward components logged separately **including `r_info` and `λ_probe`** — the
  expected reward-hacking mode of this design is constant probing, and it is
  visible in one curve.
- Curriculum: start training at `s ∈ [0.3, 0.6]` where probing pays and the fault
  is survivable, widen to `[0.2, 0.9]`. Document it; an undocumented curriculum is
  a hyperparameter that reviewers cannot reproduce.
- New RQ6 deliverable: the **selectivity figure** — probe rate against belief
  entropy for `rl_probe`, with C8's fixed schedule as a flat reference line.

### M10 — addendum

- Condition matrix gains C7, C8, C9 at `s ∈ {0.2, 0.3, 0.4}` + healthy
  (+1,200 episodes).
- RQ3's detector-variant sweep (matched-AUC/different-calibration and the
  reverse) runs against the **rule-based** policy as well as the RL policy, so
  the conclusion is about detectors and not about one policy's quirks.

### `CLAUDE.md` — additions to §1

- **§1.8** — *Never let the two probe guards drift.* The environment boundary
  forces one guard per side, so, like the fault model in §1.6, they are one
  contract with two backends, held together by a shared states → verdicts test
  table. A guard that differs between training and evaluation produces a policy
  whose probes are silently refused at evaluation time, which looks exactly
  like a transfer failure.
- **§1.4 amendment** — add Isaac-side feature extraction as the second
  sanctioned exception, paid for by a parity test against the PX4-side
  extractor on a recorded fixture (§3.2).
- **§1.9** — *Never reward belief entropy.* Ground truth may enter the reward
  only through a proper scoring rule over the true class (D16), and only in the
  reward.

### `configs/schema/episode_record.yaml` — bump to v3

- `step_fields`: add the `K`-way belief vector (or its digest plus argmax and
  entropy, if per-step width is a concern), and `probe_active`.
- New `probe_events[]`: `{t_sim, primitive, intensity, granted|denied,
  guard_reason}` — **denied probes included**, per §5.
- `episode_fields`: `n_probes`, `n_probes_denied`, `probe_time_s`,
  `belief_nll_mean`, `probe_policy_version`.
- Add `probe_induced_loc` to `termination_reasons`? **No** — keep the enum about
  what ended the episode; derive probe attribution in `metrics.py` from the probe
  event timestamps, so the classification can be revised without a schema bump.

---

## 10. Order of operations

**Part A — now:**

1. **Reconcile the status blocks** (§0 note above): `planning.md` §0,
   `README.md`, and `milestones.md`'s progress log for M4, M5 and M6.
2. **Extend the M6 fixture** with the yaw-doublet response, and add the
   healthy-vs-faulty yaw assertion (§4). No plugin change: torque is already
   coupled.
3. **M7: belief head and calibration** on `detector_px4_v1`. The RQ3 variants
   fall out of this for free.

**The gate:**

4. **M7b stage 0** — about a day, right after step 2 (it needs only the
   plugin, the farm and yaw-rate setpoint support). It decides whether any of
   Part B happens.

**Part B — only if stage 0 passes:**

5. Settle the two §3.2 questions (sanctioned Isaac-side extractor; features
   against the policy-level setpoint only), then M5's `feature_version: "2"`
   and `normalization_v2`.
6. The supplementary probe dataset run (§4), then `detector_portable_v1`, then
   M7b stage 1.
7. **Freeze `action_v1` at 7 dims and `observation_v1` with the belief**, then
   build M8b.1, run the D14a sanity check, then M8b.2.
8. M9 with the matched policy pair; M10 with the extended matrix.

**If stage 0 fails:** drop D13/RQ6, keep Part A — §1's RQ3, §7's calibration
metrics and D15. The paper is still meaningfully stronger than the original
plan, and about a day has been spent on the idea.

**Rough effort.** Part A: M6 +1 day, M7 +2 days. Gate: about 1 day. Part B:
M5 +1 day, a supplementary dataset run (several sessions, going by
`docs/fault_dataset.md`), M7 +2 days, M7b stage 1 +2 days, M8b +1 week (the
Isaac-side extractor and its parity test are the new cost), M9 +5 days, M10
+1,200 episodes. Part B is **four weeks or more**. M4 and M6 both ran well
past their estimates, so treat that figure as a floor.