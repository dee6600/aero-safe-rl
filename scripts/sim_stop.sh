#!/usr/bin/env bash
# Stop PX4 SITL workers started by sim_start.sh (M1b).
#
# Stopping is PID-based, not name-based. Each worker's three PIDs are recorded
# in its instance_<N>.json, so `-i N` stops exactly that worker and cannot
# disturb its siblings -- a hard requirement for a supervisor that restarts
# individual failed workers during a multi-day training run.
#
# The old name-based `pkill -f "^gz sim "` sweep still exists, but only behind
# an explicit --sweep flag, because it cannot distinguish workers and will kill
# every simulator on the machine including ones this run does not own.
set -uo pipefail

INSTANCE=""
ALL=0
SWEEP=0
RUN_DIR="${AERO_RUN_DIR:-/tmp/aero-safe-rl-sim}"

usage() {
	cat <<EOF
Usage: $(basename "$0") [-i INSTANCE | --all] [--sweep]

  -i, --instance N   Stop only this worker, by its recorded PIDs
  -a, --all          Stop every worker tracked in the run dir (default when no
                     -i is given). Leftovers that this run does not own are
                     reported, not killed.
      --sweep        ALSO kill any px4 / gz sim / MicroXRCEAgent process on the
                     machine by name. Nuclear option -- it will kill simulators
                     belonging to other sessions. Use only when nothing else is
                     meant to be running.
      --run-dir DIR  Where instance_<N>.json files live (default: $RUN_DIR)
  -h, --help         This help
EOF
}

while [ $# -gt 0 ]; do
	case "$1" in
	-i | --instance) INSTANCE="$2"; shift 2 ;;
	-a | --all) ALL=1; shift ;;
	--sweep) SWEEP=1; shift ;;
	--run-dir) RUN_DIR="$2"; shift 2 ;;
	-h | --help) usage; exit 0 ;;
	*) echo "Unknown option: $1" >&2; usage; exit 1 ;;
	esac
done

# --------------------------------------------------------------- primitives

# Kill a process, taking its whole process group with it when it leads one.
# sim_start.sh uses setsid precisely so this works: a plain kill on the px4
# process does NOT cascade to anything it spawned (confirmed by testing).
stop_pid() {
	local pid="$1" label="$2"
	[ -z "$pid" ] || [ "$pid" = "null" ] && return 0
	if ! kill -0 "$pid" 2>/dev/null; then
		echo "  $label (pid $pid): already gone"
		return 0
	fi

	local pgid target
	pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
	if [ -n "$pgid" ] && [ "$pgid" = "$pid" ]; then
		target="-$pgid"   # negative pid == the whole process group
	else
		target="$pid"
	fi

	echo "  $label (pid $pid): stopping"
	kill -TERM -- "$target" 2>/dev/null || true
	for _ in $(seq 1 20); do
		kill -0 "$pid" 2>/dev/null || { echo "  $label: stopped"; return 0; }
		sleep 0.25
	done
	echo "  $label: did not exit on TERM, sending KILL" >&2
	kill -KILL -- "$target" 2>/dev/null || true
	sleep 0.5
	kill -0 "$pid" 2>/dev/null && { echo "  $label: STILL ALIVE (pid $pid)" >&2; return 1; }
	echo "  $label: stopped"
}

read_pids() {  # read_pids <instance_file> -> "px4 agent gz gui" on stdout
	python - "$1" <<-'PY'
		import json, sys
		rt = json.load(open(sys.argv[1])).get("runtime", {})
		print(rt.get("pid_px4") or "", rt.get("pid_agent") or "",
		      rt.get("pid_gz") or "", (rt.get("extra") or {}).get("pid_gui") or "")
	PY
}

stop_instance() {
	local file="$1"
	local inst
	inst="$(basename "$file" .json | sed 's/instance_//')"
	echo "Worker $inst ($file):"

	local pids px4 agent gz gui
	if ! pids="$(read_pids "$file" 2>/dev/null)"; then
		echo "  could not read $file; leaving it in place" >&2
		return 1
	fi
	read -r px4 agent gz gui <<<"$pids"

	local rc=0
	# Client before server: stop px4 first so it does not thrash against a
	# world that has already gone away.
	stop_pid "$px4" "px4" || rc=1
	stop_pid "$agent" "xrce agent" || rc=1
	[ -n "$gui" ] && { stop_pid "$gui" "gz gui" || rc=1; }
	stop_pid "$gz" "gz sim" || rc=1

	if [ $rc -eq 0 ]; then
		rm -f "$file"
	else
		echo "  keeping $file: something is still running" >&2
	fi
	return $rc
}

# ------------------------------------------------------------------- action

command -v python >/dev/null 2>&1 || { echo "ERROR: python not on PATH -- run: source scripts/activate.sh" >&2; exit 1; }

if [ -n "$INSTANCE" ] && [ "$ALL" -eq 1 ]; then
	echo "ERROR: -i and --all are mutually exclusive. Pick one." >&2
	exit 1
fi

RC=0
if [ -n "$INSTANCE" ]; then
	FILE="$RUN_DIR/instance_${INSTANCE}.json"
	[ -f "$FILE" ] || { echo "No instance file for worker $INSTANCE at $FILE" >&2; exit 1; }
	stop_instance "$FILE" || RC=1
else
	shopt -s nullglob
	FILES=("$RUN_DIR"/instance_*.json)
	shopt -u nullglob
	if [ ${#FILES[@]} -eq 0 ]; then
		echo "No tracked workers in $RUN_DIR"
	fi
	for f in "${FILES[@]}"; do
		stop_instance "$f" || RC=1
	done
fi

# ------------------------------------------------------------------- sweep

if [ "$SWEEP" -eq 1 ]; then
	echo "Sweeping ALL px4/gz sim/MicroXRCEAgent processes on this machine..."
	# "^gz sim " is anchored so it matches both the headless server and the GUI
	# client ("gz sim -g") without matching unrelated command lines. A plain
	# "--verbose" match misses the GUI and orphans it every time -- confirmed.
	for pattern in "build/px4_sitl_default/bin/px4 -i" "^gz sim " "MicroXRCEAgent"; do
		pkill -f "$pattern" 2>/dev/null || true
	done
	sleep 1
	for pattern in "build/px4_sitl_default/bin/px4 -i" "^gz sim " "MicroXRCEAgent"; do
		pkill -9 -f "$pattern" 2>/dev/null || true
	done
	rm -f /tmp/px4_lock-*
fi

# ------------------------------------------------------------- final report

LEFTOVER="$(pgrep -af "build/px4_sitl_default/bin/px4 -i|^gz sim |MicroXRCEAgent" 2>/dev/null | grep -v "sim_stop" || true)"
if [ -n "$LEFTOVER" ]; then
	if [ -n "$INSTANCE" ]; then
		# Expected: sibling workers are still running, and that is the point.
		echo "Note: other simulator processes are still running (siblings or untracked):"
	else
		echo "WARNING: simulator processes remain that this run does not own." >&2
		echo "         Use --sweep to kill everything on the machine, or stop them by hand." >&2
		RC=1
	fi
	echo "$LEFTOVER" | sed 's/^/  /' | cut -c1-120
else
	echo "Clean: no px4 / gz sim / MicroXRCEAgent processes remain."
fi

exit $RC
