# Multi-instance PX4 + Gazebo — verified reference

Everything here was checked against the pinned build
(PX4 `v1.17.0`, Gazebo Harmonic 8.15.0, ROS 2 Humble) on this machine on
2026-08-20, either by reading the source at the pinned tag or by running it.
Statements are marked **[source]** or **[measured]**. Nothing here is inferred
from documentation or memory.

This document exists because parallel SITL is the single largest source of
silent bugs in this project. Read it before writing anything that starts more
than one simulator.

---

## 1. What PX4 does for you automatically

**[source]** `ROMFS/px4fmu_common/init.d-posix/rcS:140-141`

```sh
param set MAV_SYS_ID     $((px4_instance+1))
param set UXRCE_DDS_KEY  $((px4_instance+1))
```

**[source]** `rcS:295-298`

```sh
uxrce_dds_ns=""
if [ "$px4_instance" -ne "0" ]; then
    uxrce_dds_ns="-n px4_$px4_instance"      # note: NOT applied to instance 0
fi
```

**[source]** `rcS`, `PX4_UXRCE_DDS_NS` override block — the namespace can be
forced to any value, including empty, by setting that environment variable.

**[source]** `rcS` — `UXRCE_DDS_DOM_ID` is set from the `ROS_DOMAIN_ID`
environment variable of the **px4 process**. The client passes the domain to the
agent, which creates the DDS participant in it
(`uxrce_dds_client.cpp:255-261`). The `MicroXRCEAgent` process itself does not
need `ROS_DOMAIN_ID`.

**[source]** `px4 -i N` also gives the instance its own rootfs
(`build/px4_sitl_default/rootfs/N/`) and offsets the internal MAVLink UDP ports
by `N`.

---

## 2. The four traps

### 2.1 `target_system` must be `instance + 1`

**[source]** `src/modules/commander/Commander.cpp:746`

```cpp
if (((cmd.target_system != _vehicle_status.system_id) && (cmd.target_system != 0))
```

A `VehicleCommand` whose `target_system` matches neither `0` nor this vehicle's
`MAV_SYS_ID` is dropped. Since `MAV_SYS_ID == instance + 1`, code that hardcodes
`target_system = 1` works on instance 0 and is **silently ignored** on every
other instance. There is no error, no ack, no log line. The vehicle simply never
arms.

*Fix:* derive `target_system` from the instance spec (`instance + 1`).

### 2.2 Topic namespace is asymmetric — instance 0 is the odd one out

**[measured]** `results/sim_logs/px4_instance_1_*.log`:

```
INFO [uxrce_dds_client] successfully created rt/px4_1/fmu/out/vehicle_odometry data writer
```

Instance 0 publishes `/fmu/out/vehicle_odometry`; instance 1 publishes
`/px4_1/fmu/out/vehicle_odometry`. Any node with hardcoded `/fmu/out/...`
subscribes successfully to instance 1 and receives **nothing, forever** — a
subscription to a topic with no publisher is not an error in ROS 2.

*Fix:* set `PX4_UXRCE_DDS_NS="px4_${N}"` for **all** instances including 0, so
there is one uniform code path, and build topic names from the namespace.

### 2.3 Instances silently share one Gazebo world

**[source]** `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim:35-63`

```sh
gz_world=$( gz topic -l | grep -m 1 -e "^/world/.*/clock" | sed ... )
if [ -z "${gz_world}" ] && [ -n "${PX4_GZ_WORLD}" ]; then
    gz sim --verbose=1 -r -s "${PX4_GZ_WORLDS}/${PX4_GZ_WORLD}.sdf" &
else
    echo "INFO  [init] gazebo already running world: ${gz_world}"   # joins it
fi
```

The second instance finds the first instance's world and spawns into it. The
consequences are all bad for RL:

- **One physics thread for N vehicles.** Throughput does not scale with N.
- **Speed factor is world-level and last-writer-wins.** **[source]**
  `px4-rc.gzsim:154-158` calls `gz service .../set_physics --req
  "real_time_factor: ${PX4_SIM_SPEED_FACTOR}"`, which applies to the whole
  world. Starting worker 2 at 1× silently drops worker 1 from 8× to 1×.
- **Vehicles share a space** and can collide, or interact through ground effect
  if spawned close together.
- **One crash affects everyone**, and a world reset resets all vehicles.

---

### 2.4 `--instance` must be the PX4 shell client's *first* argument

**[source]** `platforms/posix/src/px4/common/main.cpp:154`

```cpp
if (argc >= 3 && strcmp(argv[1], "--instance") == 0) {
```

`px4-commander`, `px4-param` and friends only honour `--instance N` when it is
`argv[1]`. Written anywhere else it is silently ignored: the command goes to
**instance 0**, and `--instance`/`N` are additionally passed through as stray
arguments to the sub-command.

```bash
px4-param set NAV_DLL_ACT 0 --instance 1     # WRONG -- sets it on instance 0
px4-param --instance 1 set NAV_DLL_ACT 0     # right
```

**[measured]** With two workers running, `px4-commander arm -f --instance 1`
incremented instance **0**'s arm count and left instance 1 disarmed. Moving the
flag first armed instance 1 and left instance 0 alone.

This one is especially nasty because the client prints instance 0's reply, so a
follow-up `status` check *passes* while the vehicle you meant to command never
moves. The observed symptom was "instance 1 arms but never takes off", which
reads as a flight-control problem rather than a CLI problem.

It is a static property of the source, so `tests/test_px4_cli_usage.py` scans
the repository for call sites and fails if the flag is ever misplaced again.

---

## 3. The fix: `GZ_PARTITION` per instance

`GZ_PARTITION` scopes gz-transport discovery. Two servers in different
partitions cannot see each other, so `px4-rc.gzsim`'s `gz topic -l` probe finds
nothing and each instance starts its own world.

**[measured]** Two instances started with `GZ_PARTITION=aero0` and
`GZ_PARTITION=aero1`:

```
$ pgrep -af "gz sim"
34068 gz sim --verbose=1 -r -s .../worlds/default.sdf
34619 gz sim --verbose=1 -r -s .../worlds/default.sdf     # two servers, not one
```

**[measured]** RTF is then independent per partition:

```
$ GZ_PARTITION=aero0 gz topic -e -t /world/default/stats -n 3 | grep real_time_factor
real_time_factor: 4.0001800081003642     # requested 4x, honoured
```

`GZ_PARTITION` must be exported to **both** the `gz sim` process and the `px4`
process (PX4 shells out to `gz service` and `gz topic`, and `gz_bridge` opens
its own transport node).

**[measured]** `GZ_PARTITION` is supported by the installed gz-transport 13
(`strings libgz-transport13.so | grep GZ_PARTITION`).

---

## 4. Recommended launch sequence per worker

```
run_dir/                       ← one directory per training or evaluation run
  instance_0.json              ← identity + PIDs, written by sim_start.sh
  instance_1.json
  worker_0/…                   ← logs, episode records
```

1. Compute the identity for worker `k` — a pure function of `k`, no discovery.
2. `source $PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh` for
   `GZ_SIM_RESOURCE_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH`, then prepend this
   repo's `simulation/models` and the built `RotorDegradationSystem` plugin dir.
3. Start `gz sim -r -s <world>.sdf` with `GZ_PARTITION=aero_k`, in its own
   process group (`setsid`), record its PID.
4. Wait for `/world/<w>/scene/info` to answer **in that partition** — do not
   sleep, and do not grep a log.
5. Start `MicroXRCEAgent udp4 -p $((8888+k))`, record its PID.
6. Start `px4 -i k -d` with `PX4_GZ_STANDALONE=1`, `GZ_PARTITION=aero_k`,
   `ROS_DOMAIN_ID=k`, `PX4_UXRCE_DDS_NS=px4_k`, `PX4_UXRCE_DDS_PORT=$((8888+k))`,
   `PX4_SIM_SPEED_FACTOR`, `PX4_GZ_MODEL_POSE`, in its own process group.
   Record its PID.
7. Readiness = `/px4_k/fmu/out/vehicle_status_v1` received on domain `k`
   **and** its `pre_flight_checks_pass` reachable. Not a log grep.
8. Set `NAV_DLL_ACT 0` (this project has no MAVLink GCS by design; PX4 otherwise
   refuses to arm) via `px4-param set ... --instance k`.
9. Write `instance_k.json`.

Teardown is the reverse, by recorded PID, killing the process **group**.

---

## 5. Why `PX4_GZ_STANDALONE=1`

If PX4 starts the Gazebo server, that server is a child of the `px4` process's
shell and we never learn its PID. Two consequences already observed:

- **[measured]** `kill <px4_pid>` does not cascade to `gz sim`; the server keeps
  running and consuming CPU.
- Stopping one worker therefore requires a name-based `pkill`, which cannot
  distinguish workers and kills all of them.

Owning the server PID makes per-worker stop exact, which is a hard requirement
for a supervisor that restarts individual failed workers.

---

## 6. Process lifetime

**[measured]** A SITL instance backgrounded from a short-lived shell dies when
that shell exits, leaving its `gz sim` server orphaned. Both halves must be
started with `setsid` (own session and process group) so they survive the
launcher, and both PIDs must be recorded so they can be reaped deliberately.

Orphan accumulation is not cosmetic: each orphaned `gz sim` holds one to two
cores. Three orphans on a 12-thread machine is a 25% throughput loss that shows
up only as "training got slower".

---

## 7. Throughput budget

Per worker, roughly: `px4` ≈ 1 core, `gz sim` ≈ 1–2 cores, `MicroXRCEAgent`
≈ 0.2 core, plus the Python env node. On 12 threads with the PPO learner also
running, **4 workers is the realistic ceiling and 3 is the safe default.**

Single-instance RTF ceiling on this machine is ~8× **[measured, M1]**. With N
independent worlds the per-worker RTF falls as they contend; aggregate
throughput, not per-worker RTF, is the number that matters:

> **Required measurement (M4):** aggregate simulated-seconds-per-wall-second
> across the **topology grid** — isolated (N worlds × 1 drone), hybrid
> (N/2 worlds × 2 drones), and fully shared (1 world × N drones) — at speed
> factors {1, 2, 4, 8}, plus per-worker RTF stdev and peak RSS. Pick the
> configuration with the best aggregate throughput at acceptable jitter, and
> budget M9 from that single number.

The old assumption "4 workers × 8× = 32× aggregate" is not supported by any
measurement and should not be planned against.

### Why isolated worlds are the *default*, and why sharing is still worth measuring

**[source]** gz-sim advances one world on a single thread — there is no
multi-threaded per-model stepping in gz-sim 8. So a world with M vehicles
computes their physics one after another on one core, and its achievable speed
factor falls roughly as `1/M`. M1 measured a single drone nearly saturating that
thread at ~8.3×, which means at high speed factors one world realistically
carries one drone.

**[source]** The coupling goes further than physics. PX4 SITL is built with
`ENABLE_LOCKSTEP_SCHEDULER yes` (`boards/px4/sitl/sitl.cmake:12`), and
`GZBridge::clockCallback` sets PX4's `CLOCK_MONOTONIC` from the world's `/clock`
topic on every tick (`GZBridge.cpp:331-345`). **Every drone in a world runs off
that one clock.** A flight stack that stalls therefore stalls its whole world.

Against that, sharing has two genuine advantages worth measuring rather than
dismissing: **RAM** (one `gz sim` process instead of M) and **process count**
(fewer things to start, supervise and reap). If this machine turns out to be
memory-bound before it is core-bound, the hybrid becomes the right answer.

Sharing also requires drone–drone collision to be disabled, which is supported
here: `collide_bitmask` exists in sdformat14 and the dartsim plugin ships a
`BitmaskContactFilter` **[measured — symbols present in
`libgz-physics-dartsim-plugin.so`]**. The gotcha is that masks collide when
`maskA & maskB != 0`, so a single shared drone mask still self-collides — each
drone needs a **distinct bit**, with the ground left at `0xFFFF`. Large spatial
separation (≥ 200 m between spawn slots, against a 20–40 m mission envelope) is
the simpler alternative and needs no SDF templating.

---

## 8. Failure catalogue

Things that will happen during a long run, and how a worker must respond:

| Symptom | Detection | Response |
|---|---|---|
| PX4 process gone | PID not alive | Restart worker; invalidate episode |
| `gz sim` gone | PID not alive | Restart worker; invalidate episode |
| XRCE agent gone | PID not alive | Restart agent only; invalidate episode |
| Telemetry stalls | no message on odometry for `T_stall` wall-seconds | Restart worker |
| Never arms | arm not confirmed within deadline | Restart worker; log preflight failure reason |
| Offboard drops out | `nav_state` leaves offboard unexpectedly | End episode, `termination_reason=offboard_lost` |
| EKF never converges | `pre_flight_checks_pass` false past deadline | Restart worker |
| Episode never ends | sim-time deadline exceeded | `truncated=True`, `termination_reason=timeout` |
| Vehicle below ground / NaN state | position or attitude non-finite | End episode, `termination_reason=sim_fault`, invalidate |

Every restart is counted in the run manifest. A run whose restart rate exceeds a
configured threshold should abort loudly rather than quietly produce a biased
dataset — restarts are not uniformly distributed across fault severities, so
silently dropping them biases exactly the comparison the paper depends on.

---

## 9. Quick self-check

Before trusting any multi-instance code:

```bash
source scripts/activate.sh

# 1. Two workers, two servers (start them one at a time; readiness is checked)
scripts/sim_start.sh -i 0 -s 4
scripts/sim_start.sh -i 1 -s 8
pgrep -af "^gz sim " | grep -c " -s "      # must be 2, not 1

# 2. Everything at a glance, including per-worker RTF
scripts/sim_status.sh

# 3. Both vehicles reachable, same code path, own domains.
#    --qos-reliability best_effort is required: PX4 publishes BEST_EFFORT, and a
#    RELIABLE subscriber waits forever against it with no error.
ROS_DOMAIN_ID=0 ros2 topic echo --once --qos-reliability best_effort \
  /px4_0/fmu/out/vehicle_status_v1 | grep system_id     # expect 1
ROS_DOMAIN_ID=1 ros2 topic echo --once --qos-reliability best_effort \
  /px4_1/fmu/out/vehicle_status_v1 | grep system_id     # expect 2  (= instance+1)

# 4. Per-worker stop does not disturb the other
scripts/sim_stop.sh -i 1
pgrep -af "^gz sim " | grep -c " -s "      # must be 1

# 5. Full stop is clean
scripts/sim_stop.sh --all                  # exits 0 and reports "Clean:"
```

If step 1 prints `1`, partitions are not being applied and everything downstream
is running in a shared world. If step 3 prints the same `system_id` twice, the
namespace override is not reaching PX4.
