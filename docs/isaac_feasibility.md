# M3b — Isaac Lab feasibility spike

Companion to `milestones.md` M3b and `planning.md` §3.1 / D12. This is the
measurement that gates every Isaac-dependent milestone (M8b, M9) — the
question was not "does Isaac Sim launch" but "does it run usefully on this
machine," and this document is the answer, with the numbers to back it.

**Bottom line: yes, comfortably.** This machine is below Isaac Sim 5.1's
*stated* minimum (RTX 4080/16GB vs. our RTX 2070 Mobile/8GB), but for a
headless, rendering-disabled, physics-only RL workload — which is what
training actually needs — that stated minimum turned out to be irrelevant.
Raw environment throughput was never the bottleneck at any scale tested; host
RAM, not GPU VRAM, is the real ceiling on this hardware, and even that has
comfortable headroom at the operating point chosen below.

---

## Hardware and versions

| | |
|---|---|
| GPU | NVIDIA GeForce RTX 2070 Mobile (Max-Q), 8192 MiB, Turing (sm_75) |
| Driver | 580.178.04 (CUDA 13.0 runtime, torch built against cu126) |
| CPU / RAM | Intel i7-8750H, 6 cores / 12 threads, 15831 MB total |
| Isaac Sim | 5.1.0.0, pip-installed into conda env `isaacsim` (Python 3.11) |
| Isaac Lab | 0.54.4, commit `b0542fe2d` (2026-07-24), cloned to `~/projects/IsaacLab` (sibling to `PX4-Autopilot`, not inside this repo — same convention as the other vendored dependencies) |
| Task | `Isaac-Quadcopter-Direct-v0` (stock Isaac Lab task, `isaaclab_tasks.direct.quadcopter`) |
| Mode | `--headless`, zero-action stepping (no learning) |

**One real blocker along the way, noted for the next person:** Isaac Sim's
first import prompts an interactive EULA acceptance and hangs forever under a
non-interactive shell (`Unable to bootstrap inner kit kernel: EOF when reading
a line`). Fixed by exporting `OMNI_KIT_ACCEPT_EULA=YES` before every Isaac
invocation. This has to be set in `scripts/activate.sh`'s Isaac counterpart
once one exists, or every future session hits the same hang.

**Isaac Lab install command actually used** (existing pip-installed Isaac Sim,
not the isaaclab-bundles-isaacsim path):
```bash
conda activate isaacsim
export OMNI_KIT_ACCEPT_EULA=YES
cd ~/projects/IsaacLab
./isaaclab.sh --install rl_games   # matches the framework Isaac-Quadcopter-Direct-v0 ships a config for
```
This produced non-fatal pip dependency-resolver warnings (`click`, `psutil`
version friction between `isaacsim-kernel`'s pins and `rl-games`'/`ipython`'s)
— both packages still imported and ran correctly; noted here so a future
session doesn't mistake the warnings for a real failure.

---

## Method

A purpose-built benchmark (not the stock `zero_agent.py`, which loops forever)
steps `Isaac-Quadcopter-Direct-v0` with zero actions for a fixed count, after a
50-step warmup excluded from the measurement. Two things are measured
externally rather than trusted from inside the process:

- **GPU memory** — via `nvidia-smi --query-gpu=memory.used`, polled at 1 Hz
  from outside the process. `torch.cuda.max_memory_allocated()` was tried
  first and reads near-zero at every scale — it only sees PyTorch's own
  allocator, not the PhysX/Warp GPU buffers Isaac Sim actually uses, so it is
  not a usable signal here.
- **Host RAM** — peak RSS of the Python process via `psutil`.

Scripts: `isaac_bench.py` and `run_bench.sh` (currently in the session
scratchpad, not yet committed to the repo — see "Follow-ups" below).

## Throughput and memory vs. env count

300 physics steps measured per row (after warmup), zero actions, headless.

| Env count | Physics steps/s | Env-steps/s (steps/s × envs) | Peak VRAM used | Peak host RSS |
|---:|---:|---:|---:|---:|
| 64 | 127 | 8,114 | 2,239 MB | 3,196 MB |
| 256 | 114 | 29,270 | 2,243 MB | 3,284 MB |
| 1,024 | 96 | 97,889 | 2,373 MB | 3,561 MB |
| 4,096 | 87 | 357,343 | 2,841 MB | 4,836 MB |
| 8,192 | 67 | 546,282 | 3,361 MB | 6,439 MB |
| 16,384 | 46 | 757,404 | 4,505 MB | 9,713 MB |
| 32,768 | 29 | 941,847 | 6,803 MB | 13,603 MB |

**Reading this table:**
- **Env-steps/s scales sub-linearly but keeps climbing all the way to 32,768** —
  physics steps/s per env falls as env count rises (CPU-side Python/gym-API
  overhead per `step()` call amortizes, but per-env GPU kernel work still
  costs something), yet the product keeps growing because env count grows
  faster than per-step cost does.
- **~2.2 GB of the VRAM figure is fixed overhead** (kernel, USD stage, physics
  engine init) present even at 64 envs — the actual per-env marginal cost is
  small. VRAM was never within sight of the 8 GB ceiling at any tested scale
  (6.8 GB peak at 32,768, ~83% of the card).
- **Host RAM is the real constraint on this machine**, not VRAM. It crosses 50%
  of the 15.8 GB total between 8,192 and 16,384 envs, and reaches ~86% at
  32,768 — too tight to trust for a sustained run, since this number is
  *physics-only* and doesn't yet include a policy network, optimizer state, or
  rollout buffer.

## 10-minute stability run

40,000 physics steps at 8,192 envs (chosen operating point, below), zero
actions, headless — the closest this benchmark gets to a real training
session's duration without an actual learning loop.

| | Burst test (300 steps) | 10-min run (40,000 steps) |
|---|---:|---:|
| Elapsed | 4.5 s | 612.6 s |
| Physics steps/s | 66.7 | 65.3 |
| Env-steps/s | 546,282 | 534,913 |
| Peak host RSS | 6,439.3 MB | 6,437.7 MB |
| Peak VRAM (nvidia-smi) | 3,361 MB | 3,361 MB |
| Exit | clean (0) | clean (0) |

**No memory growth over 10 minutes** — peak RSS differs by 1.6 MB (0.02%)
between a 4.5-second burst and a 10-minute sustained run, and VRAM peaked at
the identical value both times. Throughput held steady within 2%. No
errors, tracebacks, or OOM signatures anywhere in the run's log. This is the
result the milestone's gate actually asked for: not just "did it launch" but
"does it leak or degrade under sustained load," and the answer is no.

## Chosen operating point

**8,192 parallel environments**, physics-only throughput ≈546k env-steps/s,
~3.4 GB VRAM (41% of the card), ~6.4 GB host RSS (41% of total RAM).

This is deliberately well short of the highest count actually tested (32,768,
which also completed without error). The reason is what this benchmark does
*not* include: a real PPO run adds a policy network, an optimizer, and a
rollout buffer sized `num_envs × rollout_length × obs_dim`, all of which scale
with env count and none of which existed in this zero-action benchmark.
32,768 envs left only ~14% of host RAM free even before any of that — not a
safe number to build M8b's actual training environment on. 8,192 leaves
roughly 9 GB of headroom for exactly that overhead, which M8b should re-verify
once the real training loop exists, per the "Follow-ups" note below.

## The gate — did this pass?

**Yes, by a wide margin.** The gate (`milestones.md` M3b) was "deliver roughly
1M environment steps within a few hours." At the chosen operating point
(8,192 envs, ~546k env-steps/s physics-only), 1M steps is native-throughput
work of **under 2 seconds** — even the slowest tested configuration (64 envs,
8.1k env-steps/s) clears it in under 2 minutes. Raw simulation throughput is
not a constraint on this hardware for this task at any scale tested; **D12
stands**, and the sample-budget risk that motivated it is resolved, not merely
mitigated.

The caveat that matters going forward: this measures the simulator, not
training. Real PPO wall-clock time will be dominated by network
forward/backward passes and optimizer steps, not environment stepping — which
is exactly the situation Isaac's GPU-resident batching is designed for, but it
means this document's numbers are a floor on training speed, not a prediction
of it.

## Follow-ups for M8b

1. **Commit the benchmark script.** `isaac_bench.py` and `run_bench.sh` live
   in the session scratchpad right now, not in the repo. If M8b's environment
   work wants to re-run this sweep (e.g. after adding the real observation
   spec, which will change per-env memory cost), move them into `isaac/` first.
2. **Re-measure memory headroom once the real training loop exists.** This
   document's numbers are physics-only. `isaacsim-rl`'s own `rl_games` PPO
   config, the policy network, and the rollout buffer all add host RAM (and
   some VRAM) that scales with env count — 8,192 was chosen with that in mind,
   but it was not measured directly.
3. **`OMNI_KIT_ACCEPT_EULA=YES` needs a home.** Currently exported by hand each
   session; belongs in an Isaac counterpart to `scripts/activate.sh` (`CLAUDE.md`
   §0) before anyone else hits the interactive-hang trap.
