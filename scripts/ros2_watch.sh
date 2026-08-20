#!/usr/bin/env bash
# M2 visual check: start PX4 SITL + Gazebo (GUI) + MicroXRCEAgent, then fly
# the aero_bridge test_flight node -- entirely over ROS 2, no MAVLink at
# all -- so you can watch it and confirm the ROS 2 bridge actually drives
# the vehicle. Leaves the simulation running afterward, same as
# sim_watch.sh; run sim_stop.sh when done.
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTANCE=0
SPEED=1

usage() {
	cat <<EOF
Usage: $(basename "$0") [-i INSTANCE] [-s SPEED]

  -i, --instance N   PX4 instance id (default: 0)
  -s, --speed FACTOR PX4_SIM_SPEED_FACTOR (default: 1)
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

# See sim_watch.sh for why this targets :1 explicitly.
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
	exit 1
fi
echo "Display OK."

echo ""
echo "=== Starting PX4 SITL + Gazebo (GUI) + MicroXRCEAgent ==="
bash "$REPO_DIR/scripts/sim_start.sh" -i "$INSTANCE" -s "$SPEED" --gui
if [ $? -ne 0 ]; then
	echo "ERROR: sim_start.sh failed." >&2
	exit 1
fi

echo ""
echo "=== Flying via ROS 2 only (aero_bridge test_flight -- no MAVLink) ==="
# ROS 2's own setup scripts reference unset variables internally (e.g.
# AMENT_TRACE_SETUP_FILES) -- `set -u` must be off while sourcing them, or
# the whole script aborts right here. Same fix as scripts/env_report.sh.
set +u
source /opt/ros/humble/setup.bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate aero-safe-rl
export ROS_DOMAIN_ID="$INSTANCE"
source "$REPO_DIR/ros2_ws/install/setup.bash"
set -u
ros2 run aero_bridge test_flight
RESULT=$?

echo ""
if [ $RESULT -eq 0 ]; then
	echo "Flight succeeded over ROS 2. The Gazebo GUI is still open. When done:"
	echo "  $REPO_DIR/scripts/sim_stop.sh"
else
	echo "Flight FAILED (exit $RESULT) -- see output above. Simulation left"
	echo "running for inspection. Stop it with:"
	echo "  $REPO_DIR/scripts/sim_stop.sh"
fi

exit $RESULT
