# Isaac Sim — learning track notes

This is the optional, non-blocking parallel track described in `planning.md`
§10b and decision D6. It does not gate any research milestone (M1–M13 stay on
Gazebo Harmonic). Recorded here so the setup doesn't need to be repeated or
re-diagnosed later.

## Install

- Isaac Sim **5.1.0.0**, installed via pip into its **own conda env**
  (`isaacsim`, Python 3.11 — separate from the `aero-safe-rl` env, which stays
  on Python 3.10), matching the version Pegasus Simulator currently targets.
- Requires Python 3.11 specifically (5.x rejects 3.9/3.12).

```bash
conda create -y -n isaacsim python=3.11
conda activate isaacsim
pip install "isaacsim[all]" --extra-index-url https://pypi.nvidia.com
```

- The EULA prompt blocks any non-interactive run. Set before first import or
  launch: `export OMNI_KIT_ACCEPT_EULA=YES`.

## Smoke test result (2026-08-20, this machine)

Ran a minimal headless scene (`SimulationApp(headless=True)` + default ground
plane, 200 physics steps, no rendering) to get real numbers instead of
guessing from spec-sheet minimums. Full log:
`docs/../` — see history; key facts below.

- **GPU is recognized and works.** Warp/PhysX initialized the CUDA device
  cleanly: `"NVIDIA GeForce RTX 2070 with Max-Q Design" (8 GiB, sm_75, mempool
  enabled)`, CUDA Toolkit 12.8 / driver 13.0. Despite sitting below Isaac Sim
  5.1's stated minimum (RTX 4080 16GB), it did not fail or refuse to start.
- **First launch took ~715s (~12 min)** — almost entirely one-time extension
  downloads via `omni.kit.registry.nucleus` into `~/.local/share/ov/data/`.
  Subsequent launches should be much faster since those are cached locally.
- **Physics-only stepping is fast**: 200 steps in 0.08s (~2400 steps/s) for a
  trivial scene with `render=False`. This is a good sign for RTF, but it's an
  empty world — not yet tested with an actual rotor/vehicle model or with
  rendering/sensors enabled, which is where the 8GB VRAM ceiling will actually
  bite.
- `simulation_app.close()` terminates the process before any Python code
  after it runs — a known Isaac Sim quirk, not a bug in our script. Exit code
  was clean (0).

## Known constraint, restated

This machine's 8GB VRAM is expected to support **one** Isaac Sim instance
comfortably, not several in parallel — confirmed by NVIDIA's own guidance,
not yet stress-tested here. That's fine for this track's scope (learning +
a single-instance port of the baseline mission); it is exactly why this
project's actual RL training (M9) stays on Gazebo. See `planning.md` D6 for
the full reasoning.

## Next step (not yet done)

Clone and set up **Pegasus Simulator** (the PX4 MAVLink bridge for Isaac
Sim) to actually fly the pinned PX4 `v1.17.0` SITL binary inside Isaac Sim.
Not started yet — this doc will be updated when it is.
