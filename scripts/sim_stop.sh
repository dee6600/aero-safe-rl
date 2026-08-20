#!/usr/bin/env bash
# Cleanly stop PX4 SITL + Gazebo + MicroXRCEAgent instances started by
# sim_start.sh.
#
# Orphaned `px4`, `gz sim`, and `MicroXRCEAgent` processes are a known
# failure mode with this stack (a plain SIGTERM to px4 does not cascade to
# the Gazebo server it spawned, and the agent is a separate process
# entirely) -- this script always cleans up all three, and always sweeps for
# leftovers by process name in addition to the tracked PIDs, so a crashed or
# manually-started instance doesn't linger and quietly eat CPU.
set -uo pipefail

INSTANCE=""
ALL=0

usage() {
	cat <<EOF
Usage: $(basename "$0") [-i INSTANCE | --all]

  -i, --instance N   Stop only this instance
  -a, --all          Stop all tracked instances and any leftover
                      px4/gz sim processes (default if no -i given)
  -h, --help         This help
EOF
}

while [ $# -gt 0 ]; do
	case "$1" in
	-i | --instance)
		INSTANCE="$2"
		shift 2
		;;
	-a | --all)
		ALL=1
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

PID_DIR="/tmp/aero-safe-rl-sim"

stop_pid_file() {
	local pid_file="$1"
	local label="$2"
	if [ -f "$pid_file" ]; then
		local pid
		pid="$(cat "$pid_file")"
		if kill -0 "$pid" 2>/dev/null; then
			echo "Stopping $label (pid $pid)"
			kill "$pid" 2>/dev/null || true
			for _ in $(seq 1 10); do
				kill -0 "$pid" 2>/dev/null || break
				sleep 0.5
			done
			kill -9 "$pid" 2>/dev/null || true
		fi
		rm -f "$pid_file"
	fi
}

stop_instance() {
	local inst="$1"
	stop_pid_file "$PID_DIR/px4_instance_${inst}.pid" "instance $inst"
	stop_pid_file "$PID_DIR/xrce_agent_${inst}.pid" "MicroXRCEAgent for instance $inst"
	rm -f "/tmp/px4_lock-${inst}"
}

if [ -n "$INSTANCE" ]; then
	stop_instance "$INSTANCE"
else
	shopt -s nullglob
	for pid_file in "$PID_DIR"/px4_instance_*.pid; do
		inst="$(basename "$pid_file" .pid | sed 's/px4_instance_//')"
		stop_instance "$inst"
	done
	shopt -u nullglob
fi

if [ -z "$INSTANCE" ] || [ "$ALL" -eq 1 ]; then
	echo "Sweeping for leftover px4/gz sim/MicroXRCEAgent processes..."
	pkill -f "build/px4_sitl_default/bin/px4 -i" 2>/dev/null || true
	sleep 1
	pkill -9 -f "build/px4_sitl_default/bin/px4 -i" 2>/dev/null || true
	# Matches both the headless server ("gz sim --verbose=1 -r -s ...") and
	# the GUI client ("gz sim -g") -- a plain "--verbose" match misses the
	# GUI process and leaves it orphaned. Confirmed by testing.
	pkill -f "^gz sim " 2>/dev/null || true
	sleep 1
	pkill -9 -f "^gz sim " 2>/dev/null || true
	pkill -f "MicroXRCEAgent" 2>/dev/null || true
	sleep 1
	pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
	rm -f /tmp/px4_lock-*
fi

REMAINING=$(pgrep -f "build/px4_sitl_default/bin/px4 -i|^gz sim |MicroXRCEAgent" 2>/dev/null || true)
if [ -n "$REMAINING" ]; then
	echo "WARNING: processes still running after cleanup: $REMAINING" >&2
	exit 1
fi

echo "Clean."
