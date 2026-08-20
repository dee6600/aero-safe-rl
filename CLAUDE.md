# CLAUDE.md — operating rules for `aero-safe-rl`

Read this before touching anything. `planning.md` says *what and why*;
`milestones.md` says *in what order*; this file says *how to write code here
without breaking the things that have already broken once*.

Every rule below exists because violating it produces a bug that is silent,
reproduces only under parallelism, or only appears three milestones later.

---

## 0. Shell setup — every session, every terminal

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate aero-safe-rl
source /opt/ros/humble/setup.bash
source ~/projects/aero-safe-rl/ros2_ws/install/setup.bash
```

Order matters. Conda supplies the interpreter (3.10.20) and the ML stack; ROS
supplies `rclpy` from `/opt/ros/humble` on `PYTHONPATH`. Both are Python 3.10,
so the C-extension ABI matches. Verified working with numpy 2.2.6.

`scripts/activate.sh` should do exactly this and nothing else. If a script or
test fails with `No module named rclpy` or `No module named torch`, you sourced
one of the two and not the other.

---

## 1. Hard rules — never violate

1. **Never modify `~/projects/PX4-Autopilot` in place.** It is pinned to
   `v1.17.0` on branch `aero-safe-rl`. Changes that are genuinely unavoidable
   go in `simulation/patches/` as a tracked `.patch` file *and* get committed
   on that branch, so `git status` there stays clean.
2. **Never run `git add` or `git commit`.** The user commits their own work.
   Leave changes in the working tree and say what you changed.
3. **Never `pip install` outside the `aero-safe-rl` conda env.** If a package is
   missing, add it to `environment.yml` in the same change.
4. **Never introduce a second implementation of something that already exists.**
   Feature extraction, metrics, the PX4 interface, and the episode record schema
   each have exactly one implementation. Import it; do not re-derive it.
5. **Never write flight logic that special-cases instance 0.** See §2.

---

## 2. Instance identity — the single largest bug source in this project

A PX4 SITL "instance" is **eight** coupled resources. Getting one wrong gives you
a worker that starts fine, publishes nothing, and hangs forever.

| Resource | Value for instance `N` | Set by |
|---|---|---|
| PX4 instance id | `N` | `px4 -i N` |
| `MAV_SYS_ID` | **`N + 1`** | PX4 `rcS` (automatic) |
| uXRCE-DDS client key | `N + 1` | PX4 `rcS` (automatic) |
| uXRCE-DDS UDP port | `8888 + N` | `PX4_UXRCE_DDS_PORT` |
| ROS 2 / DDS domain | `N` (via `UXRCE_DDS_DOM_ID`) | `ROS_DOMAIN_ID` env on the **px4** process |
| ROS topic namespace | **`px4_N`** — force it for *all* N, including 0 | `PX4_UXRCE_DDS_NS` |
| gz-transport partition | `aero_N` | `GZ_PARTITION` env on **both** `gz sim` and `px4` |
| Gazebo model name | `<model>_N` | PX4 `rcS` (automatic) |

### The three rules that follow from that table

**2.1 — `target_system` is `N + 1`, never `1`.**
`Commander.cpp:746` rejects any `VehicleCommand` whose `target_system` is
neither `0` nor `MAV_SYS_ID`. Hardcoding `target_system = 1` means arm, offboard
and land are silently *ignored* on every instance except 0. Nothing logs an
error; the vehicle just sits there.

**2.2 — Topic names are namespaced, and instance 0 is namespaced too.**
PX4's `rcS` adds `-n px4_N` only when `N != 0`, so instance 0 gets bare
`/fmu/out/...` and instance 1 gets `/px4_1/fmu/out/...`. That asymmetry is what
lets code pass every single-instance test and then fail the moment you scale up.
**Set `PX4_UXRCE_DDS_NS="px4_${N}"` for every instance so there is exactly one
code path.** Build topic names with one helper; never write a literal
`/fmu/out/...` string in a node.

**2.3 — Identity is read from a file, not recomputed.**
`scripts/sim_start.sh` writes `${run_dir}/instance_<N>.json` with every field in
the table above plus the PIDs it owns. Python reads that file. Two places
computing `8888 + N` is one place too many.

---

## 3. Parallelism

**3.1 — One Gazebo server per worker, isolated by `GZ_PARTITION`.**
By default PX4 v1.17.0 (`px4-rc.gzsim`) runs `gz topic -l`, finds an existing
world, and *joins it*. N instances then share one physics process. That is wrong
for RL: shared clock, shared `set_physics` (speed factor is world-level and
last-writer-wins), vehicles that can collide, and one crash that poisons every
worker. Setting a distinct `GZ_PARTITION` per instance makes discovery fail to
find the neighbour, so each instance gets its own server. **Verified: two
partitions produce two independent `gz sim -s` processes with independent RTF.**

**3.2 — Launch `gz sim` yourself; use `PX4_GZ_STANDALONE=1`.**
If PX4 spawns the Gazebo server, we do not own its PID and cannot stop one
worker without a name-based `pkill` that kills the others too. Start the server
first (after sourcing PX4's `rootfs/gz_env.sh` for the resource paths), record
its PID, then start PX4 with `PX4_GZ_STANDALONE=1`.

**3.3 — `rclpy.init()` happens in the worker process only.**
Never call `rclpy.init()`, create a node, or import anything that does, in the
parent process of a `SubprocVecEnv` / `multiprocessing.Pool`. DDS participants
own threads and sockets that do not survive process creation intact. Set the
start method explicitly to `spawn` and construct every ROS object inside the
child's `__init__`.

**3.4 — Worker index → resources is a pure function, assigned once.**
No "find a free port", no random domain. Worker `k` gets instance `k + base`,
deterministically. Free-port search races when two workers start simultaneously.

**3.5 — One writer per file.** N workers never append to one CSV/Parquet.
Each writes `results/<run_id>/worker_<k>/...`; merging is a separate offline
step.

**3.6 — Stopping one worker must not touch the others.**
`sim_stop.sh -i N` kills only the three PIDs recorded in `instance_N.json`.
The broad `pkill -f "^gz sim "` sweep is for `--all` at the end of a session,
and must never run while other workers are alive.

---

## 4. Time

**Wall-clock time is not simulation time.** At `PX4_SIM_SPEED_FACTOR=8`, a
`time.sleep(1.0)` waits 8 simulated seconds. Every timeout, hold duration,
settling wait, and rate limit written against wall clock silently changes
meaning when the speed factor changes — and the speed factor *will* change
between debugging (1×) and training (4–8×).

- Mission and control logic times itself from **PX4 message timestamps** or the
  Gazebo `/clock`, never `time.time()` / `time.sleep()`.
- Wall clock is allowed for exactly one thing: the **watchdog** that decides a
  worker is hung. That one *must* be wall-clock, and must be generous enough to
  survive a 1× run.
- Every episode carries both `t_sim` and `t_wall` in its record so the two can
  never be confused after the fact.

---

## 5. Robustness — assume every worker will fail

Over a 1–3 M step training run, each of these happens many times: PX4 hangs, the
EKF refuses to converge, the vehicle never arms, offboard drops out, the Gazebo
server dies, the XRCE agent dies, an episode never terminates.

- Every wait has a deadline and a defined outcome. There is **no unbounded
  `while True:`** and no unbounded `rclpy.spin()` in episode code.
- An episode that exceeds its deadline returns `truncated=True` with a
  `termination_reason`, and is *recorded*, not silently dropped.
- The supervisor restarts a dead worker and marks the in-flight episode invalid.
  Restart counts go in the run manifest — a run that needed 400 restarts is a
  finding, not a footnote.
- `termination_reason` is a closed enum. Add a value deliberately; never write
  a free-text reason.

---

## 6. Tests

`pytest` is the runner. Three tiers, separated by marker, because a suite that
needs a simulator is a suite nobody runs.

| Marker | Needs | Runtime | When it runs |
|---|---|---|---|
| *(none)* | nothing | < 5 s total | every change |
| `@pytest.mark.sim` | one SITL instance | ~1 min each | before closing a milestone |
| `@pytest.mark.slow` | multiple instances / training | minutes+ | explicitly |

```bash
pytest tests/ -m "not sim and not slow"    # the default; must always pass
pytest tests/ -m sim                       # milestone gate
```

- **Most logic must be testable without a simulator.** Feature extraction,
  the fault schedule, the recovery FSM, metrics, config loading, and the
  instance-identity mapping all take data in and give data out. Test them
  against recorded fixtures in `tests/fixtures/`, not against a live sim.
- Record fixtures once (a real telemetry window, a real episode log) and commit
  them. Regenerating a fixture is a deliberate act that invalidates baselines.
- **Every milestone ships unit tests for the logic it introduced**, not only a
  visual demo or an end-to-end smoke script. A milestone without tests is not
  done.
- Tests must not depend on each other's order and must not leave processes
  behind. `sim`-marked tests use a fixture that guarantees cleanup on failure.

---

## 7. Contracts

These are files with schemas and version strings, not conventions:

| Contract | File | Versioned by |
|---|---|---|
| Feature vector | `configs/features.yaml` | `feature_version` |
| Episode record | `configs/schema/episode_record.yaml` | `schema_version` |
| RL observation | `configs/rl/observation_v1.yaml` | `obs_version` |
| RL action | `configs/rl/action_v1.yaml` | `action_version` |
| Fault spec | `configs/faults/*.yaml` | `fault_schema_version` |
| Instance spec | `${run_dir}/instance_<N>.json` | `spec_version` |

Every artifact written to `results/` records the versions it was produced under.
Changing a contract means bumping its version, not editing in place. A test
asserts that written records validate against the schema.

---

## 8. Anti-patterns — do not do these

1. Writing a literal `/fmu/out/...` topic string inside a node.
2. `target_system = 1`.
3. `time.sleep()` in flight or mission logic.
4. `while True:` without a deadline.
5. Searching for a free port or a free ROS domain at runtime.
6. `pkill -f` anything while other workers are running.
7. Calling `rclpy.init()` before forking/spawning workers.
8. A second copy of feature extraction, metrics, or arming logic "just for this
   script".
9. Splitting a dataset by timestep instead of by episode.
10. Recomputing normalisation statistics after training has started.
11. Using ground-truth fault state as a policy input (reward shaping only).
12. Tuning a baseline down so the learned method wins.
13. Grepping a log file for a readiness string when a topic or service can be
    queried instead.
14. Appending results from several workers to one file.
15. `np.random` global state instead of a seeded `Generator` passed explicitly.

---

## 9. Definition of done for any change

Before you say a task is complete:

- [ ] `pytest tests/ -m "not sim and not slow"` passes.
- [ ] New logic has unit tests that would fail if the logic were wrong.
- [ ] Anything touching the simulator was run with **at least two concurrent
      instances**, not one.
- [ ] No new literal topic strings, sleeps, or unbounded loops (§8).
- [ ] `scripts/sim_stop.sh --all` leaves zero `px4` / `gz sim` /
      `MicroXRCEAgent` processes.
- [ ] `milestones.md`'s progress log and checkboxes updated.
- [ ] Changes left uncommitted for the user.

---

## 10. How to scope a session

One milestone sub-task per session. `milestones.md` contains a ready-to-paste
prompt for each. Prefer plan mode for anything touching M4 (parallel farm),
M6 (fault plugin) or M9 (RL). When a task spans more than about three files,
stop and split it — long single-shot implementations in this stack are where the
silent parallel bugs come from.
