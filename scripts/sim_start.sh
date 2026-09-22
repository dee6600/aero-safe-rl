#!/usr/bin/env bash
# Start one fully isolated PX4 SITL worker (M1b).
#
# A worker is three processes that must agree on eight resources:
#
#   gz sim -s        own world, isolated by GZ_PARTITION
#   MicroXRCEAgent   own UDP port
#   px4 -i N         own ROS domain, DDS namespace, MAV_SYS_ID, model name
#
# All eight come from simulation/instance_spec.py. Nothing here recomputes any
# of them -- see CLAUDE.md §2 and docs/parallelism.md.
#
# Three things this script deliberately does differently from the naive
# approach, each because the naive version produced a silent bug:
#
#   1. We start the Gazebo server ourselves (PX4_GZ_STANDALONE=1). If PX4 starts
#      it, PX4 first probes for an already-running world and JOINS it -- so N
#      instances silently share one physics thread, one clock, and one
#      world-level speed factor. We also would not own its PID, so stopping one
#      worker would need a name-based pkill that kills its siblings too.
#   2. Readiness is a real ROS topic arriving on the right domain, not a log
#      grep. "Startup script returned successfully" does not mean the DDS link
#      is up.
#   3. Every process is started with setsid, so it survives the launching shell
#      and can be killed as a process group.
set -euo pipefail

INSTANCE=0
WORLD="default"
SPEED=1
POSE=""
MODEL="x500"
GUI=0
RUN_DIR="${AERO_RUN_DIR:-/tmp/aero-safe-rl-sim}"
# Generous by default: starting several workers at once contends for CPU during
# EKF convergence, and a readiness check that passes alone but times out under
# load is worse than no check. Overridable so tests can exercise the failure
# path without waiting 90s.
GZ_TIMEOUT="${AERO_GZ_TIMEOUT:-60}"
DDS_TIMEOUT="${AERO_DDS_TIMEOUT:-90}"

usage() {
	cat <<EOF
Usage: $(basename "$0") [options]

  -i, --instance N      PX4 instance id (default: 0). Each concurrent worker
                        needs a distinct id; everything else is derived from it.
  -w, --world NAME      Gazebo world sdf name (default: default)
  -s, --speed FACTOR    Simulation speed factor (default: 1). Applied per-world,
                        which is meaningful because each worker owns its world.
  -p, --pose "x,y,z,roll,pitch,yaw"
                        Spawn pose (default: world origin -- each worker has its
                        own world, so they need not be spread out)
  -m, --model NAME      Airframe model (default: x500)
  -g, --gui             Also start the Gazebo GUI (default: headless)
      --run-dir DIR     Where instance_<N>.json lives (default: $RUN_DIR)
  -h, --help            This help
EOF
}

while [ $# -gt 0 ]; do
	case "$1" in
	-i | --instance) INSTANCE="$2"; shift 2 ;;
	-w | --world) WORLD="$2"; shift 2 ;;
	-s | --speed) SPEED="$2"; shift 2 ;;
	-p | --pose) POSE="$2"; shift 2 ;;
	-m | --model) MODEL="$2"; shift 2 ;;
	-g | --gui) GUI=1; shift ;;
	--run-dir) RUN_DIR="$2"; shift 2 ;;
	-h | --help) usage; exit 0 ;;
	*) echo "Unknown option: $1" >&2; usage; exit 1 ;;
	esac
done

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/projects/PX4-Autopilot}"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
PX4_BIN="$PX4_BUILD/bin/px4"
LOG_DIR="$REPO_DIR/results/sim_logs"

export PATH="$HOME/.local/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"

# ---------------------------------------------------------------- preflight

die() { echo "ERROR: $*" >&2; exit 1; }

[ -x "$PX4_BIN" ] || die "$PX4_BIN not found. Build it: cd $PX4_DIR && make px4_sitl"
command -v MicroXRCEAgent >/dev/null 2>&1 || die "MicroXRCEAgent not on PATH ($HOME/.local/bin)"
command -v gz >/dev/null 2>&1 || die "gz not on PATH"
command -v python >/dev/null 2>&1 || die "python not on PATH -- run: source scripts/activate.sh"
# Readiness depends on ros2; a worker nobody can reach over ROS 2 is not ready
# by any definition this project cares about, so this is a hard requirement
# rather than a degraded fallback.
command -v ros2 >/dev/null 2>&1 || die "ros2 not on PATH -- run: source scripts/activate.sh"

# ------------------------------------------------------- identity from spec

SPEC_ARGS=(-i "$INSTANCE" -w "$WORLD" -m "$MODEL" -s "$SPEED")
[ -n "$POSE" ] && SPEC_ARGS+=(-p "$POSE")
[ "$GUI" -eq 1 ] && SPEC_ARGS+=(--gui)

if ! SPEC_SHELL="$(cd "$REPO_DIR" && python -m simulation.instance_spec "${SPEC_ARGS[@]}" --shell)"; then
	die "could not derive identity for instance $INSTANCE (see message above)"
fi
eval "$SPEC_SHELL"

mkdir -p "$LOG_DIR" "$RUN_DIR"
INSTANCE_FILE="$RUN_DIR/instance_${INSTANCE}.json"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"

if [ -f "$INSTANCE_FILE" ]; then
	if OLD_PID="$(python - "$INSTANCE_FILE" <<-'PY'
		import json, sys
		print(json.load(open(sys.argv[1]))["runtime"].get("pid_px4") or "")
	PY
	)" && [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
		die "instance $INSTANCE already running (px4 pid $OLD_PID). Stop it: scripts/sim_stop.sh -i $INSTANCE"
	fi
	echo "Note: stale $INSTANCE_FILE (no live process); replacing it."
	rm -f "$INSTANCE_FILE"
fi

echo "Starting worker: instance=$SPEC_INSTANCE world=$SPEC_WORLD partition=$SPEC_GZ_PARTITION"
echo "  domain=$SPEC_ROS_DOMAIN_ID ns=$SPEC_TOPIC_NS xrce=$SPEC_XRCE_PORT sys_id=$SPEC_MAV_SYS_ID speed=${SPEC_SPEED_FACTOR}x"

# --------------------------------------------------------- gazebo resources

# PX4's own env file supplies GZ_SIM_RESOURCE_PATH, GZ_SIM_SYSTEM_PLUGIN_PATH
# and GZ_SIM_SERVER_CONFIG_PATH. Without the server config the world comes up
# missing the systems PX4 expects.
[ -f "$PX4_BUILD/rootfs/gz_env.sh" ] || die "$PX4_BUILD/rootfs/gz_env.sh missing -- build PX4 first"
# gz_env.sh appends to these unconditionally ("export X=$X:...") and so trips
# `set -u` when they are unset, which is the normal case in a fresh shell.
# Seed them empty rather than relaxing `set -u` for the rest of the script.
: "${GZ_SIM_RESOURCE_PATH:=}"
: "${GZ_SIM_SYSTEM_PLUGIN_PATH:=}"
# shellcheck disable=SC1091
source "$PX4_BUILD/rootfs/gz_env.sh"
# Drop the empty leading entry the append above leaves behind; a bare ":" in a
# search path means "current directory", which is not what we want.
GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH#:}"
GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH#:}"
export GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH

# Our own models take precedence, so M6's x500_aero (with the rotor degradation
# plugin) is found without touching the PX4 tree. Wired now so M6 need not
# modify this launcher.
[ -d "$REPO_DIR/simulation/models" ] && export GZ_SIM_RESOURCE_PATH="$REPO_DIR/simulation/models:$GZ_SIM_RESOURCE_PATH"
[ -d "$REPO_DIR/simulation/gz_plugins/build" ] && export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_DIR/simulation/gz_plugins/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"

export GZ_PARTITION="$SPEC_GZ_PARTITION"
export GZ_IP=127.0.0.1

# ------------------------------------------------------------------ helpers

cleanup_on_failure() {
	local code=$?
	[ $code -eq 0 ] && return 0
	echo "Startup failed; cleaning up this worker's processes." >&2
	for pid in "${PID_PX4:-}" "${PID_AGENT:-}" "${PID_GUI:-}" "${PID_GZ:-}"; do
		[ -n "$pid" ] && kill -TERM -- "-$pid" 2>/dev/null || true
	done
	sleep 1
	for pid in "${PID_PX4:-}" "${PID_AGENT:-}" "${PID_GUI:-}" "${PID_GZ:-}"; do
		[ -n "$pid" ] && kill -KILL -- "-$pid" 2>/dev/null || true
	done
	rm -f "$INSTANCE_FILE"
	return $code
}
trap cleanup_on_failure EXIT

# Start a command detached in its own session, so it survives this shell and
# can later be killed as a process group.
start_detached() {
	local log="$1"; shift
	setsid "$@" >"$log" 2>&1 &
	echo $!
}

# --------------------------------------------------------------- 1. gazebo

LOG_GZ="$LOG_DIR/gz_world_${SPEC_WORLD_INDEX}_${STAMP}.log"
PID_GZ="$(start_detached "$LOG_GZ" gz sim --verbose=1 -r -s "$PX4_GZ_WORLDS/$SPEC_WORLD.sdf")"
ln -sf "$LOG_GZ" "$LOG_DIR/gz_world_${SPEC_WORLD_INDEX}_latest.log"
echo "  gz sim      pid $PID_GZ  (partition $GZ_PARTITION)"

deadline=$((SECONDS + GZ_TIMEOUT))
until timeout 3 gz service -i --service "/world/$SPEC_WORLD/scene/info" 2>/dev/null | grep -q "Service providers"; do
	kill -0 "$PID_GZ" 2>/dev/null || die "gz sim exited during startup. See $LOG_GZ"
	[ $SECONDS -lt $deadline ] || die "gz world '$SPEC_WORLD' not ready within ${GZ_TIMEOUT}s. See $LOG_GZ"
	sleep 0.5
done
echo "  gz world ready"

if [ "$GUI" -eq 1 ]; then
	# A remote/SSH shell has no DISPLAY of its own, but this machine has a real
	# logged-in graphical session. Target it explicitly rather than failing, so
	# `--gui` works when driving the machine over SSH.
	if [ -z "${DISPLAY:-}" ]; then
		export DISPLAY=":1"
		echo "  no DISPLAY set; using $DISPLAY (the machine's graphical session)"
	fi
	if [ -z "${XAUTHORITY:-}" ]; then
		for candidate in "/run/user/$(id -u)/gdm/Xauthority" "$HOME/.Xauthority"; do
			[ -f "$candidate" ] && { export XAUTHORITY="$candidate"; break; }
		done
	fi
	xdpyinfo >/dev/null 2>&1 || die "cannot open DISPLAY=$DISPLAY. Is a graphical session logged in? Check: who / loginctl list-sessions"

	LOG_GUI="$LOG_DIR/gz_gui_${SPEC_WORLD_INDEX}_${STAMP}.log"
	PID_GUI="$(start_detached "$LOG_GUI" gz sim -g)"
	echo "  gz gui      pid $PID_GUI  (DISPLAY=$DISPLAY, world $SPEC_WORLD_INDEX)"
fi

# ----------------------------------------------------------------- 2. agent

LOG_AGENT="$LOG_DIR/xrce_agent_${INSTANCE}_${STAMP}.log"
PID_AGENT="$(start_detached "$LOG_AGENT" MicroXRCEAgent udp4 -p "$SPEC_XRCE_PORT")"
ln -sf "$LOG_AGENT" "$LOG_DIR/xrce_agent_${INSTANCE}_latest.log"
echo "  xrce agent  pid $PID_AGENT  (udp $SPEC_XRCE_PORT)"

# ------------------------------------------------------------------- 3. px4

# shellcheck disable=SC1090
eval "$(cd "$REPO_DIR" && python -m simulation.instance_spec "${SPEC_ARGS[@]}" --env)"

# Project-local models (M6's x500_aero, etc.) need the launcher to spawn
# them itself, by absolute path, before px4 starts. px4-rc.gzsim's own spawn
# logic (~/projects/PX4-Autopilot, unmodified) hardcodes a path under PX4's
# OWN models directory and can never find one of ours -- confirmed live, not
# assumed; see simulation/instance_spec.py's _AUTOSTART_OVERRIDE_FOR_MODEL
# and gz_spawn_request() docstrings for the full story. PX4_GZ_MODEL_NAME,
# set by the --env eval above exactly when this is needed, then tells
# px4-rc.gzsim to attach to this already-spawned model instead of spawning
# its own -- so this block and that env var are always used together.
if [ -n "${PX4_GZ_MODEL_NAME:-}" ]; then
	MODEL_SDF="$REPO_DIR/simulation/models/$MODEL/model.sdf"
	[ -f "$MODEL_SDF" ] || die "model '$MODEL' needs custom spawn handling (PX4_GZ_MODEL_NAME=$PX4_GZ_MODEL_NAME set) but $MODEL_SDF does not exist"

	SPAWN_REQ="$(cd "$REPO_DIR" && python -m simulation.instance_spec "${SPEC_ARGS[@]}" --gz-spawn-request "$MODEL_SDF")"
	SPAWN_REPLY="$(timeout 10 gz service -s "/world/$SPEC_WORLD/create" \
		--reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
		--req "$SPAWN_REQ" 2>&1)"
	# gz service's own exit code only reflects whether the RPC round-tripped,
	# not whether entity creation actually succeeded -- that is the boolean
	# reply body, which must be checked explicitly ("confirm, don't assume",
	# the same principle M6's whole fault-injection design rests on).
	echo "$SPAWN_REPLY" | grep -q "data: true" ||
		die "failed to spawn model '$PX4_GZ_MODEL_NAME': $SPAWN_REPLY"
	echo "  model       spawned $PX4_GZ_MODEL_NAME (custom spawn path: $MODEL_SDF)"

	# px4-rc.gzsim's own spawn branch (the one this bypasses) also sets the
	# world's physics speed factor as part of spawning -- replicated here so
	# it is not silently lost for a worker started with -m other than the
	# default.
	timeout 10 gz service -s "/world/$SPEC_WORLD/set_physics" \
		--reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 5000 \
		--req "real_time_factor: $SPEED" >/dev/null 2>&1
fi

LOG_PX4="$LOG_DIR/px4_instance_${INSTANCE}_${STAMP}.log"
PID_PX4="$(start_detached "$LOG_PX4" "$PX4_BIN" -i "$INSTANCE" -d)"
ln -sf "$LOG_PX4" "$LOG_DIR/px4_instance_${INSTANCE}_latest.log"
echo "  px4         pid $PID_PX4"

# ------------------------------------------------------------- 4. readiness

# The real check: a PX4 message arriving over DDS, on this worker's domain, on
# this worker's namespace. PX4 publishes BEST_EFFORT, so the subscriber has to
# match or it will wait forever against a live publisher.
READY_TOPIC="/$SPEC_TOPIC_NS/fmu/out/vehicle_status_v1"
deadline=$((SECONDS + DDS_TIMEOUT))
until ROS_DOMAIN_ID="$SPEC_ROS_DOMAIN_ID" timeout 5 \
	ros2 topic echo --once --qos-reliability best_effort "$READY_TOPIC" >/dev/null 2>&1; do
	kill -0 "$PID_PX4" 2>/dev/null || die "px4 exited during startup. See $LOG_PX4"
	kill -0 "$PID_GZ" 2>/dev/null || die "gz sim died during px4 startup. See $LOG_GZ"
	# The agent is the DDS link itself. If it died (most often: its UDP port was
	# already taken) there is no point waiting out the full timeout.
	kill -0 "$PID_AGENT" 2>/dev/null || die "MicroXRCEAgent exited -- is udp port $SPEC_XRCE_PORT already in use? See $LOG_AGENT"
	[ $SECONDS -lt $deadline ] || die "no telemetry on $READY_TOPIC (domain $SPEC_ROS_DOMAIN_ID) within ${DDS_TIMEOUT}s. See $LOG_PX4"
	sleep 1
done
echo "  telemetry confirmed on $READY_TOPIC"

# This project drives PX4 purely over ROS 2/DDS and deliberately never provides
# a MAVLink GCS heartbeat. PX4 otherwise refuses to arm ("No connection to the
# GCS", governed by NAV_DLL_ACT). Every node here depends on arming without one.
#
# NOTE the argument order. PX4's shell client only accepts "--instance N" as
# argv[1] (platforms/posix/src/px4/common/main.cpp:154). Anywhere else it is
# silently ignored and the command goes to instance 0 -- so the obvious
# `px4-param set NAV_DLL_ACT 0 --instance 1` sets it on the WRONG vehicle and
# reports success. Confirmed by experiment.
"$PX4_BUILD/bin/px4-param" --instance "$INSTANCE" set NAV_DLL_ACT 0 >/dev/null 2>&1 || true

# Read it back rather than trusting the write: this parameter is the difference
# between a vehicle that arms and one that refuses with no obvious cause.
NAV_DLL_NOW="$("$PX4_BUILD/bin/px4-param" --instance "$INSTANCE" show NAV_DLL_ACT 2>/dev/null |
	awk '/NAV_DLL_ACT/ {print $NF}' | tail -1)"
if [ "$NAV_DLL_NOW" != "0" ]; then
	echo "  WARNING: NAV_DLL_ACT is '${NAV_DLL_NOW:-unknown}', expected 0 -- arming may be refused" >&2
fi

# ------------------------------------------------------- 5. handshake file

INSTANCE_FILE="$INSTANCE_FILE" \
PID_GZ="$PID_GZ" PID_AGENT="$PID_AGENT" PID_PX4="$PID_PX4" PID_GUI="${PID_GUI:-}" \
LOG_GZ="$LOG_GZ" LOG_AGENT="$LOG_AGENT" LOG_PX4="$LOG_PX4" \
SPEC_JSON="$(cd "$REPO_DIR" && python -m simulation.instance_spec "${SPEC_ARGS[@]}")" \
	python - <<'PY'
import json, os, datetime, pathlib, sys

sys.path.insert(0, os.environ.get("AERO_REPO", pathlib.Path(__file__).parent.as_posix()))
spec = json.loads(os.environ["SPEC_JSON"])
runtime = {
    "pid_gz": int(os.environ["PID_GZ"]),
    "pid_agent": int(os.environ["PID_AGENT"]),
    "pid_px4": int(os.environ["PID_PX4"]),
    "log_gz": os.environ["LOG_GZ"],
    "log_agent": os.environ["LOG_AGENT"],
    "log_px4": os.environ["LOG_PX4"],
    "started_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "extra": {},
}
if os.environ.get("PID_GUI"):
    runtime["extra"]["pid_gui"] = int(os.environ["PID_GUI"])

path = pathlib.Path(os.environ["INSTANCE_FILE"])
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(json.dumps({"spec": spec, "runtime": runtime}, indent=2, sort_keys=True) + "\n")
PY

trap - EXIT
echo "Instance $INSTANCE ready. Identity: $INSTANCE_FILE"
