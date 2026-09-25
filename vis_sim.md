# Visualizing the simulation

A run-from-here guide to actually **see** what this project has built so
far: drones flying, missions executing, multiple isolated worlds running side
by side. Companion to [`README.md`](README.md) (what/why) and
[`milestones.md`](milestones.md) (build log) — this file is only commands.

**Prerequisite: run these from a real desktop session, not a headless SSH
shell.** Gazebo's GUI opens a window on whatever `$DISPLAY` your shell has;
if you're on this machine's own desktop, that's already set correctly. If
you're SSHed in, use `ssh -X` (X forwarding) or run these at the machine's
own screen instead — otherwise `--gui` will hang waiting for a display that
isn't there (confirmed live while writing this file).

Every section below assumes the project environment is sourced first, every
time (see [`CLAUDE.md`](CLAUDE.md) §0 — each new terminal needs this again):

```bash
source scripts/activate.sh
```

**The camera default is already zoomed out enough to see a whole mission.**
Gazebo's own out-of-the-box default camera sits close enough that a
15×15 m mission like `square_circuit` doesn't fully fit in frame. This is
set once, in `~/.gz/sim/8/gui.config`'s `<camera_pose>` — a per-user Gazebo
preference file, not anything inside this repo or PX4's tree — so it applies
automatically to every GUI window in every section below; there's nothing to
do per-command. If you want it different (closer, an angle, following a
specific drone instead), edit that one line
(`<camera_pose>x y z roll pitch yaw</camera_pose>`, world frame, in metres
and radians) and it takes effect on the next `--gui` launch.

---

## 1. The fastest look: one drone, one window

Starts a single headed (GUI) worker and leaves it idle at its spawn point —
nothing flies yet, this just proves the picture: one Gazebo window, one x500
quadcopter, sitting on the ground.

```bash
scripts/sim_start.sh -i 0 --gui
```

You should see one Gazebo window with one drone in it. Check what's running:

```bash
scripts/sim_status.sh
```

Stop it when you're done looking:

```bash
scripts/sim_stop.sh -i 0
```

---

## 2. Watch the actual M3 mission fly

This is the real thing: the same `square_circuit` mission
([`configs/missions/square_circuit.yaml`](configs/missions/square_circuit.yaml))
every M3 baseline number in
[`docs/baseline_results.md`](docs/baseline_results.md) came from — takeoff to
5 m, a ~15×15 m square with a 2 s hold at each corner, a 3 s final hover, then
land. About 60–90 s of flight at 1× speed.

```bash
# terminal 1: start the worker with its GUI visible, at real-time speed so
# it's easy to follow
scripts/sim_start.sh -i 0 --gui -s 1

# terminal 2 (source scripts/activate.sh here too): fly it, through the
# exact same EpisodeRunner/mission_executor code path every real experiment
# uses -- this is not a separate demo flight implementation
python experiments/run_episodes.py --mission square_circuit --n 1 --instance 0 --speed 1
```

Watch the drone take off, trace the square, hold at each corner, hover, and
land. The second command prints a line per episode with the outcome
(`completed`, or a `termination_reason` if something interrupted it) and the
position RMSE against the commanded trajectory.

Fly it a few times in a row (each one resets between flights, per M3's
three-tier reset ladder):

```bash
python experiments/run_episodes.py --mission square_circuit --n 5 --instance 0 --speed 1
```

Stop when done:

```bash
scripts/sim_stop.sh -i 0
```

---

## 3. Multiple isolated worlds at once (M1b)

The thing M1b actually proves: N workers get N **independent** Gazebo
worlds — not N drones sharing one window, one clock and one speed factor
(the wrong, PX4-default topology this project deliberately avoids; see
[`docs/parallelism.md`](docs/parallelism.md) §2.3, decision D7).

```bash
scripts/watch_worlds.sh -n 2 -s 1
```

You should see **two separate Gazebo windows**, one drone in each. It also
flies both (a simple arm → takeoff → hover → land, over ROS 2, each on its
own `ROS_DOMAIN_ID`) and prints per-window confirmation once done. Leaves
everything running so you can keep looking — up to 4 windows are allowed
(`-n 4`), more is refused since each one costs real GPU.

```bash
scripts/sim_status.sh      # both workers, their PIDs, their measured RTF
scripts/sim_stop.sh --all  # stop everything this script started
```

---

## 4. Watch M4's SimFarm fly two workers concurrently

This is the actual milestone-4 gate test made visible: `SimFarm` starting and
supervising two workers, each independently flying the real M3 mission,
restarting either one automatically if it fails — the same code path
[`tests/sim/test_two_workers.py`](tests/sim/test_two_workers.py) runs
headless in CI, just with `headless=False` so you can watch it.

`SimFarm` doesn't have its own CLI script yet (that's `benchmark_throughput.py`,
M4 task 6, not yet built) — for now this is a short inline script:

```bash
cat > /tmp/watch_simfarm.py << 'EOF'
from experiments.sim_farm import SimFarm

def main():
    with SimFarm(worker_count=2, mission_id="square_circuit", n_episodes_per_worker=1,
                 speed_factor=1.0, headless=False, stagger_s=3.0) as farm:
        results = farm.run()

    for r in results:
        print(f"worker {r['worker_id']}: {r['termination_reason']} "
              f"rmse={r['position_rmse_m']:.2f}m reset={r['reset_tier']}")

# Required, not a style choice: SimFarm uses multiprocessing's "spawn" start
# method (CLAUDE.md §3.3), which re-imports this script in every worker
# process it creates. Without this guard, SimFarm(...) would run AGAIN
# inside each spawned worker, recursively -- see experiments/sim_farm.py's
# module docstring, which documents exactly this failure mode.
if __name__ == "__main__":
    main()
EOF
python /tmp/watch_simfarm.py
```

You should see two Gazebo windows come up a few seconds apart (the
"stagger" — starting workers simultaneously contends for CPU during EKF
convergence), both fly the square circuit independently, and a two-line
summary print at the end. Results are also written under
`results/<run_id>/worker_0/` and `results/<run_id>/worker_1/`.

```bash
scripts/sim_stop.sh --all
```

**Two real bugs lived here until 2026-09-21, now both fixed.** If you hit a
crash a little while after the two windows appeared, this was it — not
something wrong with your machine, and nothing to redo except pulling the fix.

1. **The missing `if __name__ == "__main__":` guard above, the primary
   cause.** `SimFarm` uses multiprocessing's "spawn" start method (required
   by CLAUDE.md §3.3, since rclpy can only be initialised inside a spawned
   worker process). Spawn re-imports the *launching script* in every child
   process it creates — so a `SimFarm(...)` sitting at a script's top level,
   without the guard, ran again inside each worker it spawned, recursively:
   each nested attempt raced the real run for CPU and tried (and failed) to
   start already-running instances. This produced exactly what "nothing
   happened for a while, then it crashed" looks like from the outside.
2. **A tight, badly-placed timeout, found while diagnosing #1.** `SimFarm`
   used to capture toolchain versions (`scripts/env_report.sh`, itself
   several subprocess calls) *after* starting the workers, inside `run()`,
   with a 30-second budget. The CPU contention from bug #1 alone was enough
   to blow through that timeout, which unwound the whole run via
   `SimFarm.__exit__` — stopping both workers, which is the "crashed" part.
   Fixed independently by moving that capture into `SimFarm.__init__()`,
   before any worker starts, regardless of what else is competing for CPU.

Two GUI-rendered workers is still genuinely heavy on a laptop GPU even with
both fixes — expect this to take noticeably longer wall-clock than the same
mission solo (section 2, confirmed: ~150s here vs. ~80s headless), and expect
your system to feel loaded while both windows are up. That slowness itself is
normal, not a bug.

**Known limitation, honestly stated:** the concurrent-worker
`offboard_control_signal_lost` gap documented in
[`docs/parallelism.md`](docs/parallelism.md) §2.6 is real and unresolved — at
two concurrent workers you have a meaningful chance of seeing a drone drop
out of offboard mid-flight (it recovers on its own most of the time; the
mission sometimes still finishes with `termination_reason=episode_timeout`
instead of `completed`). That is expected, already-documented behavior, not
something wrong with what you're watching.

---

## 5. Isaac Sim's own GUI — a different simulator, for a different purpose

Everything above is **Gazebo**, and that's deliberate, not incidental: every
piece of code you've seen so far (`EpisodeRunner`, `WorkerSupervisor`,
`SimFarm`, the mission executor) talks to PX4, and PX4's SITL only simulates
against Gazebo (or a couple of other simulators Isaac Sim isn't one of) — see
`docs/parallelism.md` and decision D7. None of this project's actual flight
code has ever spoken to Isaac Sim, and under the current plan (D12,
`planning.md` §3.1) it never will: Isaac Sim's role is training a policy in a
GPU-parallel *reduced-order* environment (M8b, not yet built), evaluated back
on this same PX4/Gazebo stack — not a second place to fly the same mission.

That said — Isaac Sim 5.1 and Isaac Lab are already installed and verified
working on this machine (M3b, `docs/isaac_feasibility.md`), and getting
comfortable with its GUI is a reasonable goal on its own. This opens Isaac
Lab's own stock quadrotor task in its windowed viewer — a *different*
quadrotor model than PX4's x500, with a simple built-in controller, not this
project's mission logic, and not on the same track as anything above:

This switches conda env, so if you're in a fresh terminal for this section
(especially over SSH), confirm the display is actually set before launching
— Isaac Sim needs it exactly the same way Gazebo did above, just easier to
miss here since it's a new shell:

```bash
echo $DISPLAY   # empty/wrong here means the window has nowhere to open
export DISPLAY=:1   # this machine's own desktop session; set explicitly if the above was empty
```

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate isaacsim
export OMNI_KIT_ACCEPT_EULA=YES   # first run only; see docs/isaac_feasibility.md

cd ~/projects/IsaacLab
python scripts/environments/random_agent.py --task Isaac-Quadcopter-Direct-v0 --num_envs 4
```

Omitting `--headless` is what opens the window — this is the same task M3b's
throughput sweep ran headless. `--num_envs 4` gives you four quadrotors side
by side in one Isaac Sim viewport; the actions are random (no trained policy
exists yet, that's M9), so expect them to drift and tumble, not fly
purposefully — this is for GUI familiarity, not a demo of anything trained.
Closing the window or Ctrl-C *usually* stops it cleanly.

To instead see it standing still (easier to look around the interface without
things moving): `scripts/environments/zero_agent.py` in place of
`random_agent.py`, same arguments.

**If it hangs instead — confirmed live, it can.** Unlike Gazebo's workers,
Isaac Sim has no `sim_stop.sh` equivalent and no supervised process tree; if
the Kit process wedges (observed: unresponsive to Ctrl-C and to `SIGTERM`),
you have to find and kill it by hand:

```bash
# find it -- the Kit process is the python interpreter running your script;
# there is usually also a separate omni.telemetry.transmitter process
ps -eo pid,etimes,cmd | grep -E "environments/random_agent.py|environments/zero_agent.py|omni.telemetry.transmitter" | grep -v grep

# try graceful first (replace PIDs with what the above printed)
kill -TERM <pid> <telemetry_pid>
sleep 3
ps -p <pid>   # still there? escalate:
kill -KILL <pid> <telemetry_pid>
```

Confirm the GPU is actually free afterward (Isaac Sim holding VRAM after its
process dies is the symptom that matters, not just "is the PID gone"):

```bash
nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader   # empty = clean
```

---

## 6. Just checking what's currently running

Any time, in any terminal with the environment sourced:

```bash
scripts/sim_status.sh          # every tracked worker: identity, PIDs, live RTF
scripts/sim_status.sh --no-rtf # same, skip the ~2s-per-worker RTF probe
```

---

## 7. Cleaning up

Always available, and safe to run even if nothing is up:

```bash
scripts/sim_stop.sh --all      # stops every worker this project's own scripts started
scripts/sim_stop.sh --all --sweep  # also kills any gz/px4/MicroXRCEAgent process
                                    # on the machine, tracked or not -- use this if
                                    # something got left behind (e.g. Ctrl-C'd mid-start)
```

Verify nothing is left:

```bash
pgrep -cf "^gz sim |px4_sitl_default/bin/px4|MicroXRCEAgent"   # expect 0
```

`sim_stop.sh` only knows about Gazebo/PX4 — it does **not** touch Isaac Sim.
If section 5 is hung, use its own kill commands there, not this section.

---

*(This file is a companion, not a milestone deliverable — it documents
existing, already-tested functionality for visual inspection. If a future
milestone adds a real visualization/dashboard track, that gets its own
design, not a rewrite of this file — see the "Phase 2" list in `milestones.md`.)*
