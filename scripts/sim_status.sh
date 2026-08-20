#!/usr/bin/env bash
# Report every tracked SITL worker: identity, process liveness, and measured
# real-time factor (M1b).
#
# Reads the instance_<N>.json handshake files rather than guessing from process
# names, so what it prints is what sim_start.sh actually built.
set -uo pipefail

RUN_DIR="${AERO_RUN_DIR:-/tmp/aero-safe-rl-sim}"
SHOW_RTF=1

usage() {
	cat <<EOF
Usage: $(basename "$0") [--run-dir DIR] [--no-rtf]

  --run-dir DIR   Where instance_<N>.json files live (default: $RUN_DIR)
  --no-rtf        Skip the real-time factor probe (each one costs ~2s)
  -h, --help      This help
EOF
}

while [ $# -gt 0 ]; do
	case "$1" in
	--run-dir) RUN_DIR="$2"; shift 2 ;;
	--no-rtf) SHOW_RTF=0; shift ;;
	-h | --help) usage; exit 0 ;;
	*) echo "Unknown option: $1" >&2; usage; exit 1 ;;
	esac
done

command -v python >/dev/null 2>&1 || { echo "ERROR: python not on PATH -- run: source scripts/activate.sh" >&2; exit 1; }

shopt -s nullglob
FILES=("$RUN_DIR"/instance_*.json)
shopt -u nullglob

if [ ${#FILES[@]} -eq 0 ]; then
	echo "No tracked workers in $RUN_DIR"
	# Untracked simulator processes are worth surfacing: they are exactly the
	# orphans that quietly eat a core each.
	ORPHANS="$(pgrep -af "build/px4_sitl_default/bin/px4 -i|^gz sim |MicroXRCEAgent" 2>/dev/null | grep -v sim_status || true)"
	[ -n "$ORPHANS" ] && {
		echo "But these simulator processes are running untracked:"
		echo "$ORPHANS" | sed 's/^/  /' | cut -c1-120
		exit 1
	}
	exit 0
fi

alive() { kill -0 "$1" 2>/dev/null && echo "up" || echo "DEAD"; }

printf "%-4s %-10s %-8s %-6s %-6s %-8s %-22s %s\n" \
	INST PARTITION NS DOMAIN PORT SYSID "px4/agent/gz" RTF

RC=0
for f in "${FILES[@]}"; do
	# One python call per worker, emitting shell assignments -- cheaper and less
	# fragile than several greps over JSON.
	eval "$(python - "$f" <<-'PY'
		import json, sys
		d = json.load(open(sys.argv[1]))
		s, r = d["spec"], d.get("runtime", {})
		out = {
		    "I": s["instance"], "P": s["gz_partition"], "N": s["topic_ns"],
		    "D": s["ros_domain_id"], "T": s["xrce_port"], "S": s["mav_sys_id"],
		    "W": s["world"],
		    "PX4": r.get("pid_px4") or "", "AG": r.get("pid_agent") or "",
		    "GZ": r.get("pid_gz") or "",
		}
		for k, v in out.items():
		    print(f"F_{k}='{v}'")
	PY
	)"

	STATES="$(alive "${F_PX4:-0}")/$(alive "${F_AG:-0}")/$(alive "${F_GZ:-0}")"
	case "$STATES" in *DEAD*) RC=1 ;; esac

	RTF="-"
	if [ "$SHOW_RTF" -eq 1 ] && [ "$(alive "${F_GZ:-0}")" = "up" ]; then
		RTF="$(GZ_PARTITION="$F_P" timeout 4 gz topic -e -t "/world/$F_W/stats" -n 1 2>/dev/null |
			awk -F': ' '/real_time_factor/ {printf "%.2fx", $2; exit}')"
		[ -z "$RTF" ] && RTF="?"
	fi

	printf "%-4s %-10s %-8s %-6s %-6s %-8s %-22s %s\n" \
		"$F_I" "$F_P" "$F_N" "$F_D" "$F_T" "$F_S" "$STATES" "$RTF"
done

[ $RC -ne 0 ] && echo "One or more workers has a dead process -- see DEAD above." >&2
exit $RC
