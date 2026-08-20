#!/usr/bin/env bash
# Start PX4 SITL + Gazebo with the GUI visible, fly a short arm/takeoff/
# hover/land sequence so you can watch it, and report whether everything
# actually worked. Leaves the simulation running afterward so you can keep
# watching or fly it manually; run sim_stop.sh when you're done.
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTANCE=0
SPEED=1

usage() {
	cat <<EOF
Usage: $(basename "$0") [-i INSTANCE] [-s SPEED]

  -i, --instance N   PX4 instance id (default: 0)
  -s, --speed FACTOR PX4_SIM_SPEED_FACTOR (default: 1 -- real time, easiest to watch)
  -h, --help         This help
EOF
}

while [ $# -gt 0 ]; do
	case "$1" in
	-i | --instance)
		INSTANCE="$2"
		shift 2
		;;
	-s | --speed)
		SPEED="$2"
		shift 2
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

# Find a usable X display. This machine has a real logged-in graphical
# session on :1 (seat0) even when this script runs from a remote/SSH shell
# that has no DISPLAY of its own -- target that session explicitly rather
# than relying on inherited environment variables.
if [ -z "${DISPLAY:-}" ]; then
	export DISPLAY=":1"
fi
if [ -z "${XAUTHORITY:-}" ]; then
	for candidate in "/run/user/$(id -u)/gdm/Xauthority" "$HOME/.Xauthority"; do
		if [ -f "$candidate" ]; then
			export XAUTHORITY="$candidate"
			break
		fi
	done
fi

echo "Using DISPLAY=$DISPLAY XAUTHORITY=${XAUTHORITY:-<none>}"
if ! xdpyinfo >/dev/null 2>&1; then
	echo "ERROR: cannot open display $DISPLAY (is a graphical session logged in on this machine?)." >&2
	echo "       Check 'who' / 'loginctl list-sessions' for the active seat's display number." >&2
	exit 1
fi
echo "Display OK."

echo ""
echo "=== Starting PX4 SITL + Gazebo (GUI) ==="
bash "$REPO_DIR/scripts/sim_start.sh" -i "$INSTANCE" -s "$SPEED" --gui
if [ $? -ne 0 ]; then
	echo "ERROR: sim_start.sh failed." >&2
	exit 1
fi

echo ""
echo "=== Flying a short arm / takeoff / hover / land check ==="
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate aero-safe-rl
python "$REPO_DIR/scripts/fly_demo.py" --instance "$INSTANCE" --speed "$SPEED"
RESULT=$?

echo ""
if [ $RESULT -eq 0 ]; then
	echo "Everything checked out. The Gazebo GUI is still open -- watch it, or fly"
	echo "it yourself. When you're done:"
	echo "  $REPO_DIR/scripts/sim_stop.sh"
else
	echo "One or more checks FAILED -- see output above. Simulation left running"
	echo "for inspection. Stop it with:"
	echo "  $REPO_DIR/scripts/sim_stop.sh"
fi

exit $RESULT
