#!/usr/bin/env bash
# Start one headless PX4 SITL + Gazebo Harmonic instance.
#
# PX4 v1.17.0's own gz-sim glue (ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim)
# already handles multi-instance correctly: if a Gazebo world is already
# running, a second `px4 -i 1` instance detects it (via `gz topic -l`) and
# spawns its own model into the SAME world/process instead of starting a
# second Gazebo server. Model name becomes "<model>_<instance>", and the
# per-instance PX4 working directory (rootfs/<instance>/) is created
# automatically by the px4 binary itself when `-i` is given without `-w`.
# We don't need to reimplement any of that here — just call the binary with
# the right env vars.
set -euo pipefail

INSTANCE=0
WORLD="default"
SPEED=1
POSE=""
MODEL="x500"
GUI=0

usage() {
	cat <<EOF
Usage: $(basename "$0") [options]

  -i, --instance N     PX4 instance id (default: 0). Each concurrent
                        instance needs a distinct id.
  -w, --world NAME      Gazebo world sdf name, from Tools/simulation/gz/worlds/
                        (default: default)
  -s, --speed FACTOR    PX4_SIM_SPEED_FACTOR (default: 1)
  -p, --pose "x,y,z,roll,pitch,yaw"
                        Spawn pose. Default: instances spread 3m apart along
                        x (instance 0 at x=0, instance 1 at x=3, ...).
  -m, --model NAME      Airframe model (default: x500)
  -g, --gui             Show the Gazebo GUI (default: headless, no GUI)
  -h, --help            This help
EOF
}

while [ $# -gt 0 ]; do
	case "$1" in
	-i | --instance)
		INSTANCE="$2"
		shift 2
		;;
	-w | --world)
		WORLD="$2"
		shift 2
		;;
	-s | --speed)
		SPEED="$2"
		shift 2
		;;
	-p | --pose)
		POSE="$2"
		shift 2
		;;
	-m | --model)
		MODEL="$2"
		shift 2
		;;
	-g | --gui)
		GUI=1
		shift
		;;
	-h | --help)
		usage
		exit 0
		;;
	*)
		echo "Unknown option: $1" >&2
		usage
		exit 1
		;;
	esac
done

PX4_DIR="$HOME/projects/PX4-Autopilot"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$REPO_DIR/results/sim_logs"
PID_DIR="/tmp/aero-safe-rl-sim"

export PATH="$HOME/.local/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"

if [ ! -x "$PX4_BIN" ]; then
	echo "ERROR: $PX4_BIN not found. Build it first: cd $PX4_DIR && make px4_sitl" >&2
	exit 1
fi
if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
	echo "ERROR: MicroXRCEAgent not found on PATH ($HOME/.local/bin)." >&2
	exit 1
fi

mkdir -p "$LOG_DIR" "$PID_DIR"

PID_FILE="$PID_DIR/px4_instance_${INSTANCE}.pid"
if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
	echo "ERROR: instance $INSTANCE already running (pid $(cat "$PID_FILE")). Stop it first with sim_stop.sh -i $INSTANCE" >&2
	exit 1
fi

# Each instance gets its own uXRCE-DDS port (8888+instance) and ROS_DOMAIN_ID
# (=instance), so this is ready for M8's multi-instance parallel training
# without rework -- not just single-instance M2 use.
XRCE_PORT=$((8888 + INSTANCE))
XRCE_PID_FILE="$PID_DIR/xrce_agent_${INSTANCE}.pid"
if [ -f "$XRCE_PID_FILE" ] && kill -0 "$(cat "$XRCE_PID_FILE")" 2>/dev/null; then
	echo "MicroXRCEAgent for instance $INSTANCE already running (pid $(cat "$XRCE_PID_FILE"), port $XRCE_PORT)"
else
	XRCE_LOG="$LOG_DIR/xrce_agent_${INSTANCE}_$(date -u +%Y%m%dT%H%M%SZ).log"
	MicroXRCEAgent udp4 -p "$XRCE_PORT" >"$XRCE_LOG" 2>&1 &
	echo $! >"$XRCE_PID_FILE"
	echo "MicroXRCEAgent started for instance $INSTANCE, pid $!, port $XRCE_PORT"
	sleep 1
fi

if [ -z "$POSE" ]; then
	OFFSET_X=$((INSTANCE * 3))
	POSE="${OFFSET_X},0,0,0,0,0"
fi

HEADLESS_VAL=1
[ "$GUI" -eq 1 ] && HEADLESS_VAL=""

LOG_FILE="$LOG_DIR/px4_instance_${INSTANCE}_$(date -u +%Y%m%dT%H%M%SZ).log"

echo "Starting instance $INSTANCE: model=gz_${MODEL} world=${WORLD} speed=${SPEED}x pose=${POSE} headless=$([ -n "$HEADLESS_VAL" ] && echo yes || echo no) xrce_port=${XRCE_PORT} ros_domain_id=${INSTANCE}"
echo "Log: $LOG_FILE"

env \
	PX4_SIM_MODEL="gz_${MODEL}" \
	PX4_GZ_WORLD="$WORLD" \
	PX4_SIM_SPEED_FACTOR="$SPEED" \
	PX4_GZ_MODEL_POSE="$POSE" \
	GZ_IP=127.0.0.1 \
	HEADLESS="$HEADLESS_VAL" \
	PX4_UXRCE_DDS_PORT="$XRCE_PORT" \
	ROS_DOMAIN_ID="$INSTANCE" \
	"$PX4_BIN" -i "$INSTANCE" -d \
	>"$LOG_FILE" 2>&1 &

PX4_PID=$!
echo "$PX4_PID" >"$PID_FILE"
ln -sf "$LOG_FILE" "$LOG_DIR/px4_instance_${INSTANCE}_latest.log"

echo "PX4 instance $INSTANCE started, pid $PX4_PID"

# Wait for the instance to actually report ready (gz_bridge started, EKF alive)
# rather than just returning immediately.
ATTEMPTS=60
while [ $ATTEMPTS -gt 0 ]; do
	if grep -q "Startup script returned successfully" "$LOG_FILE" 2>/dev/null; then
		# This project controls PX4 purely over ROS 2/DDS -- there is
		# deliberately never a MAVLink GCS heartbeat. By default PX4 refuses
		# to arm without one ("No connection to the GCS", governed by
		# NAV_DLL_ACT, see rcAndDataLinkCheck.cpp). Disable that requirement
		# here, once, for every instance -- every node in this project
		# depends on being able to arm without a GCS link.
		"$PX4_DIR/build/px4_sitl_default/bin/px4-param" set NAV_DLL_ACT 0 --instance "$INSTANCE" >/dev/null 2>&1
		echo "Instance $INSTANCE ready."
		exit 0
	fi
	if ! kill -0 "$PX4_PID" 2>/dev/null; then
		echo "ERROR: instance $INSTANCE exited during startup. Check $LOG_FILE" >&2
		exit 1
	fi
	ATTEMPTS=$((ATTEMPTS - 1))
	sleep 1
done

echo "WARNING: instance $INSTANCE did not confirm startup within 60s (may still be starting). Check $LOG_FILE" >&2
