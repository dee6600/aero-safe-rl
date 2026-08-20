#!/usr/bin/env bash
# Visual check for M1b: start N isolated workers, each with its own Gazebo GUI
# window, and optionally fly them all at once.
#
# What you should see: N separate Gazebo windows, each containing exactly ONE
# drone. That is the whole point of M1b -- before it, N instances shared a
# single world and you would see N drones in one window, sharing one clock and
# one speed factor.
#
# Runs the simulators on this machine's real graphical session, so it works
# fine when invoked over SSH from another computer -- the windows appear on the
# Linux machine's own screen.
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKERS=2
SPEED=1
FLY=1

usage() {
	cat <<EOF
Usage: $(basename "$0") [-n N] [-s SPEED] [--no-fly]

  -n, --workers N    How many isolated workers/worlds (default: 2)
  -s, --speed FACTOR Speed factor (default: 1 -- real time, easiest to watch)
      --no-fly       Just bring the worlds up; do not fly anything
  -h, --help         This help

Leaves everything running so you can keep watching. Stop with:
  scripts/sim_stop.sh --all
EOF
}

while [ $# -gt 0 ]; do
	case "$1" in
	-n | --workers) WORKERS="$2"; shift 2 ;;
	-s | --speed) SPEED="$2"; shift 2 ;;
	--no-fly) FLY=0; shift ;;
	-h | --help) usage; exit 0 ;;
	*) echo "Unknown option: $1" >&2; usage; exit 1 ;;
	esac
done

if [ "$WORKERS" -gt 4 ]; then
	echo "Refusing to open more than 4 GUI windows -- each one costs real GPU." >&2
	exit 1
fi

# ------------------------------------------------------------------- start

LAST=$((WORKERS - 1))
for i in $(seq 0 $LAST); do
	echo "=== worker $i ==="
	if ! bash "$REPO_DIR/scripts/sim_start.sh" -i "$i" -s "$SPEED" --gui; then
		echo "ERROR: worker $i failed to start. Stopping what did start." >&2
		bash "$REPO_DIR/scripts/sim_stop.sh" --all
		exit 1
	fi
done

echo
echo "=== workers up ==="
bash "$REPO_DIR/scripts/sim_status.sh"

SERVERS=$(pgrep -af "^gz sim " | grep -c " -s ")
echo
if [ "$SERVERS" -eq "$WORKERS" ]; then
	echo "$SERVERS independent Gazebo servers for $WORKERS workers -- isolation confirmed."
else
	echo "WARNING: $SERVERS Gazebo servers for $WORKERS workers." >&2
	echo "         Expected one each. Partitions may not be applied." >&2
fi
echo "Look for $WORKERS Gazebo windows on the machine's screen, ONE drone in each."

# --------------------------------------------------------------------- fly

if [ "$FLY" -eq 1 ]; then
	echo
	echo "=== flying all $WORKERS drones at once (arm / takeoff / hover / land) ==="
	# shellcheck disable=SC1091
	source "$HOME/miniconda3/etc/profile.d/conda.sh"
	conda activate aero-safe-rl

	# Each worker talks over its own MAVLink port (14540+instance), so these are
	# genuinely independent flights rather than one flight seen N times.
	pids=()
	for i in $(seq 0 $LAST); do
		python "$REPO_DIR/scripts/fly_demo.py" --instance "$i" --speed "$SPEED" \
			>"/tmp/fly_demo_$i.log" 2>&1 &
		pids+=($!)
	done

	rc=0
	for idx in "${!pids[@]}"; do
		wait "${pids[$idx]}" || rc=1
		echo "--- worker $idx ---"
		sed 's/^/    /' "/tmp/fly_demo_$idx.log"
	done

	echo
	if [ $rc -eq 0 ]; then
		echo "All $WORKERS drones flew successfully, in $WORKERS separate worlds."
	else
		echo "At least one flight reported a failure -- see the output above." >&2
	fi
fi

echo
echo "Still running. When you are done:"
echo "  $REPO_DIR/scripts/sim_stop.sh --all"
