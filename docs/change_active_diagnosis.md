# Change proposal — active fault diagnosis and belief-driven recovery

**Status: proposal, not approved. Date: 2026-09-22.**
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

**One bookkeeping note before anything else:** `README.md` and `planning.md` §0
still say "M4 tasks 4–8 remain", but commit `35b2fba` ("M4 (tasks 4-8): failure
handling, run manifest, throughput, soak test") appears to land them. Reconcile
the status blocks before adding the sections below, or the change history stops
being trustworthy.

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
4. **A kill-test is inserted before any of it is built** (M7b). It uses a
   scripted probe and the offline detector — no RL, no Isaac — and answers in
   two days whether the physics this idea rests on is real on the PX4 stack.

M5 already anticipated most of part 3: the shared / PX4-only feature split it
defines is exactly the boundary the portable detector lives inside.

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
| **D16** | **The information reward is a proper scoring rule, not entropy.** Rewarding entropy reduction pays the policy for becoming confident, including confidently wrong. The shaping term is the change in the log-score of the *true* class under the belief, which pays only for becoming correctly confident. Ground truth enters the reward only — never the observation (`CLAUDE.md` §1.7) — and the term is potential-based, so it cannot change the optimal policy of the underlying mission reward. |

### Note on D2 — refinement 2026-09-22 (torque coupling)

The degradation plugin must scale the rotor's **reaction torque as well as its
thrust**, with the ratio configurable, and the Isaac-side model must do the same.

This is not a detail. A quadrotor's yaw authority comes from the differential
drag torque of its two rotor pairs. If the plugin scales thrust alone, the
allocator compensates the thrust loss and the vehicle's yaw behaviour stays
almost exactly nominal — which is both physically wrong and, specifically, the
place where most of the diagnostic information lives. A thrust-only fault model
would make RQ6 untestable while looking like a negative result.

`configs/faults/*.yaml` therefore carries `thrust_factor` and `torque_factor`,
defaulting to equal (proportional degradation, e.g. blade damage), with the
decoupled case available for later fault types (e.g. ESC derating, bearing wear).
The `CLAUDE.md` §1.6 cross-validation fixture records **both** the thrust
reduction and the yaw-rate step response, on both sides.

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

---

## 4. `planning.md` §6 addendum — fault model

Add after the D2 note:

---

**Torque coupling (refinement to D2, 2026-09-22).** Every fault specification
carries `thrust_factor` and `torque_factor`. v1 sets them equal. The plugin's
status echo reports both as applied. The M6 validation gains one assertion: at
`s = 0.3` on rotor `i`, a commanded yaw-rate doublet produces a **measurably
asymmetric** yaw-rate response between the two directions, exceeding the M3 noise
floor. If it does not, either the torque scaling is not reaching the physics or
the severity is not being applied — both of which are silent failures that would
otherwise surface as a null result three milestones later.

**Dataset composition (new).** 30–40% of the M6 dataset's episodes must contain
scripted probe maneuvers at randomised times, in both healthy and faulty
episodes. Without them the detector sees probe telemetry for the first time at
evaluation, out of distribution, at exactly the moment it is supposed to be most
informative — and the resulting failure looks like "probing doesn't help".

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
| `yaw_doublet` (**v1**) | yaw-rate setpoint `+ω_p` then `−ω_p`, `T_p/2` each, position setpoint held | a degraded rotor changes the differential drag torque, so yaw authority becomes asymmetric; the vehicle does not translate, so the probe is cheap and does not risk a position excursion |
| `climb_pulse` (deferred to M11) | vertical velocity `+v_p` for `T_p`, then return | exposes the reduced thrust ceiling; costs altitude and energy, so it is an ablation, not v1 |

Defaults: `ω_p ∈ [0.2, 0.6] rad/s` scaled by action 7, `T_p = 1.2 s`,
`N_max = 6` probes/episode, refractory `t_ref = 3 s`.

**The probe guard.** One implementation, in the shared command-mapping layer,
used by the FSM and the RL policy and identical in both environments. A probe is
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
  `r_info,t = β · (γ Φ_{t+1} − Φ_t)`, clipped. This is potential-based, so it
  provably leaves the optimal policy of the rest of the reward unchanged; it
  shapes exploration toward informative behaviour without redefining the task.
  Brier score is the bounded alternative if log-score proves unstable near
  `b[k*] → 0`; whichever is used, it is a **proper scoring rule over the true
  class**, never the belief's entropy. Entropy alone pays for confidence, and the
  cheapest way to buy confidence is to be wrong about it.
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
| Thrust-only fault model erases the yaw signature | **High** | D2 refinement: `torque_factor` in the plugin and the Isaac model; M6 asserts asymmetric yaw-rate response at `s = 0.3`; the fixture records both sides |
| Probing causes the very failure it is diagnosing | **High** | Probe guard outside the policy, bounded primitive amplitude, budget and refractory; probe-induced LOC rate reported as a first-class metric; `yaw_doublet` chosen partly because it holds position |
| Detector is out-of-distribution during probes | **High** | M6 dataset contains scripted probes in 30–40% of episodes, healthy and faulty |
| Two detectors widen the RQ5 gap and confound it | Medium | Oracle-belief row in the transfer table separates the dynamics gap from the detector gap |
| Action space growth hurts the sample budget | Low | +2 dims on 5, and the probe dims are near-binary in effect; Isaac's throughput has the headroom M3b measured |
| RQ6 turns out to be physically unavailable at sub-threshold severities | Medium | **M7b kills it in 2–3 days, before M8b or M9 depend on it.** The RQ3 sharpening costs nothing extra and survives on its own |

---

## 9. `milestones.md` changes

### M5 — addendum

Three feature groups become **required and shared** (computable on both sides),
because they are the ones a probe excites:

- commanded vs achieved **yaw rate** residual, and its asymmetry over the window
  (signed, per direction);
- commanded vs achieved **angular acceleration** per body axis;
- commanded collective thrust vs achieved vertical acceleration (the existing
  thrust-residual feature, confirmed present in the shared set, not the PX4-only
  set).

New test: everything `configs/ai/detector_portable_v1.yaml` references is marked
shared in `configs/features.yaml` — the same test that already guards
`observation_v1.yaml`, pointed at the portable detector's input list.

### M6 — addendum

- `thrust_factor` **and** `torque_factor` in the fault schema and the plugin's
  status echo (D2 refinement).
- Fixture records the yaw-rate step response as well as the thrust reduction, so
  M8b has both to match.
- 30–40% of dataset episodes carry scripted probes at randomised times, healthy
  and faulty, with `probe_events` in the record.
- Validation gains the asymmetric-yaw assertion at `s = 0.3`.

### M7 — addendum

- The model's head becomes a `K = 9` softmax over severity classes (D15), not a
  scalar plus a hand-made uncertainty proxy. Temperature-scaled on a held-out
  split; **ECE and a reliability diagram reported alongside AUC**.
- Train **two** models from one training script and one architecture:
  `detector_px4_v1` (all features — the RQ1 result) and `detector_portable_v1`
  (shared features only — what the policy consumes). Report both; the gap between
  them is itself informative about what the PX4-only features are worth.
- The measured error model is still fitted, since D14a's sanity stub needs it.
- **RQ3's detector variants are generated here, and cost almost nothing:**
  temperature scaling is monotonic, so a mis-scaled variant has **exactly the
  same AUC** and different calibration — a matched-AUC arm by construction.
  Noise-injected variants give the reverse (lower AUC, re-calibrated). That is
  RQ3's whole experimental design, produced from one trained model.

---

### M7b — Probe observability spike  ⭐ new (gate for RQ6)

**Goal:** answer, offline and without RL, whether a scripted probe actually makes
a sub-threshold fault more observable **on the PX4 stack**. This is the physical
assumption RQ6 rests on. Two to three days.

**Depends on:** M6 (probe-containing dataset), M7 (`detector_portable_v1`).
**Blocks:** the RQ6 parts of M8b, M9, M10 — and nothing else.

**Key deliverables:**
- A scripted `yaw_doublet` flown through `EpisodeRunner` at `s ∈ {0, 0.2, 0.3,
  0.4}`, ≥30 episodes per cell, probe timing randomised, matched no-probe
  control episodes at identical seeds.
- One figure: belief NLL and severity MAE in the 2 s window after the probe,
  against the matched no-probe control.
- `docs/probe_observability.md`, written like `docs/isaac_feasibility.md` — the
  measurement, the method, and the verdict.

**Done when** the effect is quantified with a confidence interval, whichever way
it comes out.

**Pass criterion:** at `s = 0.2` **and** `s = 0.3`, the post-probe belief NLL is
lower than the matched control by a margin exceeding the M3 divergence band, with
a non-overlapping 95% CI over ≥30 episodes.

**Kill criterion:** no significant effect at any `s ≤ 0.4`. Then RQ6 is dropped,
D13 is marked superseded, `action_v1` stays at 5 dims, and the project continues
with RQ1–RQ5 plus the RQ3 sharpening, having spent three days instead of five
weeks. **Write the negative result into `docs/probe_observability.md` anyway** —
"the allocator masks it well enough that mission-level excitation buys nothing"
is a genuine finding about PX4, and it belongs in the paper's discussion.

**Watch out for:** running this with a detector trained on probe-free data. It
will fail for the wrong reason. Check the M6 dataset composition first.

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
- Both probe primitives implemented against the same
  `configs/rl/probe_v1.yaml`, and the **same probe guard module**, imported by
  both sides — the guard is flight logic, not env-specific glue.
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

- **§1.8** — *Never implement the probe guard twice.* It is flight logic shared
  by the FSM, the RL policy and both environments. A guard that differs between
  training and evaluation produces a policy whose probes are silently refused at
  evaluation time, which looks exactly like a transfer failure.
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

1. **Reconcile the M4 status blocks** with commit `35b2fba` (§0 note above).
2. **Apply §4 (torque coupling) before writing the M6 plugin.** This is the
   cheapest possible moment; after the plugin exists and a dataset has been
   generated, it costs a regeneration.
3. **M5: add the three shared residual groups.** Also cheap now, expensive after
   `feature_version` is referenced by a trained model.
4. **M7: belief head, calibration, and the portable detector.** The RQ3 variants
   fall out of this for free.
5. **M7b: the kill-test.** Two to three days, and it decides whether steps 6–8
   happen at all.
6. If M7b passes: **freeze `action_v1` at 7 dims and `observation_v1` with the
   belief**, then build M8b.1, run the D14a sanity check, then M8b.2.
7. M9 with the matched policy pair; M10 with the extended matrix.
8. If M7b fails: drop D13/RQ6, keep everything in §1's RQ3, §7 metrics for
   calibration, and D15. The paper is still meaningfully stronger than the
   original plan and nothing built so far is wasted.

**Rough effort delta:** M5 +1 day, M6 +2 days, M7 +2 days, M7b 3 days, M8b +4
days, M9 +5 days, M10 +1,200 episodes. Call it **three weeks**, of which the
first six days are the ones that decide whether the remaining two weeks happen.