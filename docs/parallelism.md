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

## 2. The six traps

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

### 2.5 `px4_msgs` timestamps are not simulated time -- they track wall clock

**[measured]** This is the finding that decided `PX4Clock`'s actual wiring
(M2, 2026-08-20). It looks like it shouldn't be true: PX4 SITL's internal
`hrt_absolute_time()` genuinely IS lockstep sim time (§2.4's neighbour,
Gazebo drives PX4's `CLOCK_MONOTONIC` directly -- `GZBridge.cpp:331-345`).
But that is not what arrives over the ROS 2 bridge.

**[source]** `uxrce_dds_client.cpp`'s `_synchronize_timestamps` mechanism
runs a session-level `Timesync` handshake against the `MicroXRCEAgent`
(`on_time` / `session->time_offset`), and applies that offset to every
message serialized over the session -- designed for real hardware, where a
roughly-epoch timestamp is the useful convention.

**[measured]** With a worker running at `PX4_SIM_SPEED_FACTOR=4`, sampling
`vehicle_status.timestamp` over a 10-second wall-clock window:

```
wall elapsed:        10.030 s
msg.timestamp delta:  9.938 s
ratio:                0.991      <- tracks WALL time, not the 4x sim rate
first timestamp:  1787248217.9   <- an ordinary Unix epoch value
```

The same experiment against Gazebo's own `/world/default/stats`, read
directly over gz-transport (bypassing PX4 and ROS 2 entirely):

```
wall elapsed:    10.010 s
sim_time delta:  39.488 s
ratio:            3.945          <- matches the requested factor
```

**Consequence:** no `px4_msgs` topic is usable as a source of "how much
simulated flight time has passed" -- the session-level offset is applied
uniformly, so this isn't specific to one topic. `simulation/sim_clock.py`'s
`GzSimClock` is the project's only correct source, and CLAUDE.md D10 means
"`GzSimClock`", not "any PX4 message timestamp". `PX4Interface.last_
timestamp_us` still exists as a link-freshness indicator, explicitly *not*
as a duration source (see its docstring).

**Practical confirmation:** `test_flight.py`'s hover, requested as 8 seconds
of *mission* time at a worker running speed factor 4, measured 1.97 real
seconds elapsed -- matching the expected 8/4 = 2.0s almost exactly. Wiring
`PX4Clock` to `vehicle_status.timestamp` instead would have made every
hover, and every future M3+ mission duration, take full wall-clock length
regardless of speed factor -- silently defeating the entire reason M8/M9's
sample-efficiency plan wants speed factor > 1 in the first place.

**Environment note:** `gz-transport13`'s compiled extension links the
system's protobuf; this project's conda env carries a much newer standalone
`protobuf` package. Whichever initializes first decides the process-wide
implementation, and if the newer one wins, `gz-msgs10`'s generated code
(built by an older protoc) fails to import with "Descriptors cannot be
created directly". `sim_clock.py` forces the pure-Python protobuf
implementation via `os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_
IMPLEMENTATION', 'python')` before importing anything gz-related -- the
performance caveat in protobuf's own warning doesn't matter here (a handful
of scalar fields at ~10-20 Hz), and the processes needing this module are
per-worker flight processes, not the learner process that does TensorBoard
logging.

---

### 2.6 Offboard mode can drop even when the publish loop never misses a beat

**[measured, confirmed via `.ulg` log analysis]** This is the single most
time-consuming finding in the project so far, so the full trail is recorded
here rather than just the conclusion.

**Symptom.** A worker arms and engages offboard successfully, then the
vehicle never reaches its commanded altitude and the flight logic times out.
Reading `failsafe_flags` at the moment of failure shows
`offboard_control_signal_lost = true`. Roughly 10-20% of solo flights hit
this at least once; **roughly 35-65% of concurrent two-worker flights do**
(sample sizes: 6-15 trials per configuration -- noisy, but the gap between
solo and concurrent is consistent and large).

**Three wrong hypotheses, each ruled out empirically before the real cause
was found:**

1. *"It's the battery failsafe."* The PX4 log prints `battery warning (fast)`
   immediately before every failure. **Wrong** -- that tune name is
   misleading. `Commander::updateTunes()` (`Commander.cpp`) plays that exact
   sound for the GENERIC `_vehicle_status.failsafe && isArmed()` case, not
   only for a real battery condition; it happens to reuse the battery-warning
   audio file. Parsing the `.ulg` from a failing flight
   (`pyulog.ULog(...).get_dataset('failsafe_flags')`) showed
   `battery_warning = 0` and `battery_low_remaining_time = false` at every
   single failure. Two PX4 battery parameters (`BAT_AVRG_CURRENT`,
   `COM_LOW_BAT_ACT`) were tuned and tested over 15+ flights each on the
   strength of this wrong hypothesis; neither changed the failure rate.
2. *"It's `GzSimClock`'s background gz-transport thread starving the GIL."*
   Plausible, since `simulation/sim_clock.py` forces the pure-Python protobuf
   implementation (§ below), which is slower per message than the compiled
   backend. **Ruled out**: a flight flown with a stub sim-time source (no
   gz-transport subscription at all, no `GzSimClock` construction) hit the
   identical failure signature at a similar rate (3/15 solo, including the
   same `offboard_control_signal_lost` message).
3. *"The publish loop itself is stalling under load."* The natural
   suspicion given the concurrency correlation. **Ruled out directly**: the
   publish loop was instrumented to log any gap over 150ms between
   successive `publish_position_setpoint` calls. Across multiple concurrent
   two-worker runs, including runs that failed with
   `offboard_control_signal_lost` moments later, **zero gaps over 150ms were
   ever recorded**. The application never misses a scheduled publish.

**What the `.ulg` actually shows.** `pyulog` parsing of `vehicle_status` and
`failsafe_flags` from a failing flight gives a precise timeline:
`offboard_control_signal_lost` flips true, `nav_state` switches from
OFFBOARD (14) to AUTO_RTL (5), and -- under sustained concurrent load -- this
was observed to **repeat periodically, roughly every 6-7 seconds, for the
remainder of the flight**, not as a single one-off blip.

**The actual mechanism.** `ROMFS/px4fmu_common/init.d-posix/rcS` scales
several failsafe timeouts by `PX4_SIM_SPEED_FACTOR` on startup:

```sh
COM_OF_LOSS_T_LONGER=$(echo "$PX4_SIM_SPEED_FACTOR * 1.0" | bc)
param set COM_OF_LOSS_T $COM_OF_LOSS_T_LONGER
```

`COM_OF_LOSS_T` is compared against PX4's internal `hrt_absolute_time()`,
which **is** genuine lockstep sim time (unlike the DDS-bridged message
timestamps in §2.5). Since sim time advances at `speed_factor`× the wall
rate, this formula holds the **wall-clock** tolerance for a missing offboard
heartbeat at a **constant ~1.0 real second, regardless of speed factor** --
that's the intended effect, matching how `COM_DL_LOSS_T`/`COM_RC_LOSS_T`/
`COM_OBC_LOSS_T` are scaled the same way for the same reason. A ~1-real-second
tolerance is workable when nothing else is competing for the machine, but
under concurrent-worker CPU contention, a message can apparently sit
undelivered somewhere in the BEST_EFFORT transport (DDS discovery /
`MicroXRCEAgent` / `uxrce_dds_client`, all separate OS processes/threads from
the publishing application) for longer than that, even though the
application published it on schedule.

**What was tried and what shipped.**

- **Shipped**: `hold_position_until` (`arming_sequence.py`) now re-engages
  offboard mode (re-issues `VEHICLE_CMD_DO_SET_MODE`) whenever it observes
  `nav_state != OFFBOARD`, the same "resend a BEST_EFFORT command until
  confirmed" reasoning `arm_and_engage_offboard` already applies to arming.
  Measured effect: roughly halves the concurrent-flight failure rate (66%→33%
  across paired 6-trial samples) by recovering individual loss events, but
  does **not** eliminate the underlying periodic loss -- streaming setpoints
  alone does not bring PX4 back into OFFBOARD once it has left; the explicit
  re-engage command is what does.
- **Tried, not shipped**: widening `COM_OF_LOSS_T` well beyond the rcS
  default (giving a ~5 real-second tolerance instead of ~1s). Across 8
  concurrent trials this showed no clear improvement (4/8 failed) over the
  re-engage fix alone -- within the noise of these small samples, but not a
  demonstrated win, so it was not kept. A parameter change without evidence
  it helps is worse than no change.

**Status: open.** This is a real, confirmed, currently-unresolved reliability
gap under concurrent-worker load. It is exactly the class of problem M4's
`WorkerSupervisor`/`EpisodeRunner` design already exists to handle --
detect, record as an invalid episode with a `termination_reason`, and retry
-- rather than something a single flight script should be expected to fully
paper over. Do not re-attempt the battery or `GzSimClock` hypotheses without
new evidence; both were tested and ruled out as documented above. Promising
unexplored directions: whether the loss is on the publish side (this
process → `MicroXRCEAgent`) or the PX4 side (`MicroXRCEAgent` →
`uxrce_dds_client`) was not isolated; CPU affinity / process priority
(M4 task 10) may reduce the underlying contention rather than PX4's
tolerance for it.

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
> for worker count ∈ {1, 2, 3, 4} at speed factors {1, 2, 4, 8}, plus
> per-worker RTF stdev. Pick the configuration with the best aggregate
> throughput at acceptable jitter, and budget M9 from that single number.

The old assumption "4 workers × 8× = 32× aggregate" is not supported by any
measurement and should not be planned against.

### Why one drone per world (D7)

**[source]** gz-sim advances one world on a single thread — there is no
multi-threaded per-model stepping in gz-sim 8. So a world with M vehicles
computes their physics one after another on one core, and its achievable speed
factor falls roughly as `1/M`. M1 measured a single drone nearly saturating that
thread at ~8.3×, which means at high speed factors one world realistically
carries one drone — packing more in would only divide the same throughput
across more vehicles, not add any.

**[source]** The coupling goes further than physics. PX4 SITL is built with
`ENABLE_LOCKSTEP_SCHEDULER yes` (`boards/px4/sitl/sitl.cmake:12`), and
`GZBridge::clockCallback` sets PX4's `CLOCK_MONOTONIC` from the world's `/clock`
topic on every tick (`GZBridge.cpp:331-345`). **Every drone in a world runs off
that one clock**, so a flight stack that stalls stalls its whole world, and a
`gz sim` crash loses every vehicle in that world, not one. One drone per world
avoids both.

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
