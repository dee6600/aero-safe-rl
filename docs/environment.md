# Environment

Recorded 2026-08-14, from `scripts/env_report.sh`. This is the pinned toolchain
for the project. If any of these versions change, note it here and re-verify
M0's "Done when" checks before trusting any later result.

## Host

- OS: Ubuntu 22.04.5 LTS (jammy), kernel 6.8.0-136-generic
- GPU: NVIDIA GeForce RTX 2070 with Max-Q Design, driver 580.173.02, 8 GB VRAM

## Simulator

- Gazebo Harmonic (`gz-harmonic`) 8.15.0, installed from the OSRF apt repo
  (`packages.osrfoundation.org/gazebo/ubuntu-stable`)
- Gazebo Classic 11 remains installed alongside it (not removed, per M0 notes)

## PX4

- Repo: `~/projects/PX4-Autopilot`
- Branch `aero-safe-rl`, created from tag `v1.17.0`
- `git describe --tags` → `v1.17.0`
- Built with `make px4_sitl` (NuttX toolchain skipped via `--no-nuttx` in
  `Tools/setup/ubuntu.sh` — this project only targets SITL, never real
  hardware, so the ARM cross-toolchain is unnecessary weight)

## ROS 2 / DDS bridge

- ROS 2 Humble, system install at `/opt/ros/humble` (not via conda)
- `Micro-XRCE-DDS-Agent`, built from source and installed to `~/.local`
  (not system-wide — see note below), binary at `~/.local/bin/MicroXRCEAgent`
- `px4_msgs`: cloned into `ros2_ws/src/`, on branch `release/1.17` (matches
  PX4 v1.17.0)
- `px4_ros_com`: cloned into `ros2_ws/src/`, on branch `main` — PX4 has not
  cut a `release/1.17` branch for this repo yet, so `main` is the closest
  match. Revisit if message compatibility issues appear.
- Built with `colcon build --symlink-install`; `ros2 interface list | grep
  px4_msgs` returns 236 interfaces.

## Simulated-time source (M2)

- `python3-gz-transport13` + `python3-gz-msgs10`, installed via apt, living
  under system Python's site-packages (`/usr/lib/python3/dist-packages`), NOT
  the conda env. Confirmed importable from conda's Python 3.10 (matching
  `cpython-310` ABI tag) via `simulation/sim_clock.py`, which appends that
  path to `sys.path` itself.
- Required because `px4_msgs` timestamps turned out to be unusable as a
  simulated-time source — see `docs/parallelism.md` §2.5 for the measurement.
  `simulation/sim_clock.py`'s `GzSimClock` reads Gazebo's own
  `/world/<world>/stats` directly instead, and is what `PX4Clock` (and every
  wait in the project from M2 onward, per D10) is built on.
- `scripts/env_report.sh` checks both the DDS actuator-topics patch and these
  bindings, and exits non-zero if either is missing.

## PX4 → ROS 2 telemetry latency (M2)

Measured with `scripts/measure_latency.py --instance 1 --seconds 10` against
`sensor_combined`, instance 1, speed factor 1×, 978 samples after warmup:

| Mean | Median | P95 | Min / Max | Stdev |
|---|---|---|---|---|
| 7.15 ms | 6.29 ms | 8.76 ms | 5.58 / 10.42 ms | 1.29 ms |

Single-digit milliseconds as `planning.md` §5/M2 predicted. Measured via
`time.time()` (`CLOCK_REALTIME`), which is the correct basis specifically
*because* `px4_msgs` timestamps are wall-clock-resynced by `uxrce_dds_client`
(see the sim-time note above and `docs/parallelism.md` §2.5) — both sides of
the comparison are already in the same clock domain.

## Python / ML (conda env `aero-safe-rl`)

- Python 3.10.20
- PyTorch 2.13.0+cu126, CUDA available: `True`
- Gymnasium 1.3.0, Stable-Baselines3 2.9.0
- NumPy 2.2.6, SciPy 1.15.3, pandas 2.3.3, PyYAML 6.0.3, matplotlib 3.10.9,
  TensorBoard 2.21.0
- Full frozen spec: `environment.yml` (repo root)

## Notes and deviations from a stock setup

- **Micro-XRCE-DDS-Agent is installed to `~/.local`, not `/usr/local`.**
  `sudo make install` wasn't available under the permission model used for
  this setup (only `apt`/`dpkg`/`gpg`/`tee` were pre-approved for
  passwordless sudo), so the agent was configured with
  `-DCMAKE_INSTALL_PREFIX=$HOME/.local` instead. `~/.local/bin` is already on
  `PATH` via the default `~/.profile`. `~/.local/lib` was added to
  `LD_LIBRARY_PATH` in `~/.bashrc` for interactive shells; scripts that
  launch the agent directly (e.g. `sim_start.sh`, from M1 onward) must set
  `LD_LIBRARY_PATH="$HOME/.local/lib:$LD_LIBRARY_PATH"` explicitly, since
  non-interactive script invocations don't source `.bashrc`.
- **First build of Micro-XRCE-DDS-Agent failed** because CMake picked up the
  conda `base` environment's `fmt` package (`fmt_DIR` resolved to
  `~/miniconda3/lib/cmake/fmt`), which is incompatible with the system
  `spdlog` this agent links against. Fixed by reconfiguring with
  `CONDA_PREFIX` and `CMAKE_PREFIX_PATH` unset and a restricted `PATH`, so
  CMake only sees system packages. Any future from-source build of a C++
  dependency on this machine should watch for the same conda/system
  collision if a conda environment is active in the shell.
- **NuttX toolchain was skipped** (`--no-nuttx`) when running PX4's
  `Tools/setup/ubuntu.sh`. This project is SITL-only; if real hardware
  targets are ever needed, re-run the setup script without that flag.
