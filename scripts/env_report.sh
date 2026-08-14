#!/usr/bin/env bash
# Prints the full toolchain version state as JSON.
# Run from anywhere; sources the conda env and ROS 2 setup itself.
set -o pipefail

CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"
ROS_SETUP="/opt/ros/humble/setup.bash"
PX4_DIR="$HOME/projects/PX4-Autopilot"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck disable=SC1090
[ -f "$CONDA_SH" ] && source "$CONDA_SH" && conda activate aero-safe-rl 2>/dev/null
# shellcheck disable=SC1091
[ -f "$ROS_SETUP" ] && source "$ROS_SETUP"
# shellcheck disable=SC1091
[ -f "$REPO_DIR/ros2_ws/install/setup.bash" ] && source "$REPO_DIR/ros2_ws/install/setup.bash"

export PATH="$HOME/.local/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"

json_str() { printf '%s' "$1" | python3 -c 'import json,sys; print(json.dumps(sys.stdin.read().strip()))'; }

OS_INFO=$(lsb_release -ds 2>/dev/null || echo "unknown")
KERNEL=$(uname -r)

GZ_VERSION=$(gz sim --versions 2>/dev/null || echo "not found")

PX4_TAG="not found"
PX4_BRANCH="not found"
PX4_COMMIT="not found"
if [ -d "$PX4_DIR/.git" ]; then
    PX4_TAG=$(git -C "$PX4_DIR" describe --tags 2>/dev/null || echo "unknown")
    PX4_BRANCH=$(git -C "$PX4_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
    PX4_COMMIT=$(git -C "$PX4_DIR" rev-parse HEAD 2>/dev/null || echo "unknown")
fi

PYTHON_VERSION=$(python3 --version 2>&1 || echo "not found")
CONDA_ENV="${CONDA_DEFAULT_ENV:-none}"

TORCH_INFO=$(python3 -c '
import json
try:
    import torch
    print(json.dumps({"version": torch.__version__, "cuda_available": torch.cuda.is_available(), "cuda_version": torch.version.cuda, "device": (torch.cuda.get_device_name(0) if torch.cuda.is_available() else None)}))
except Exception as e:
    print(json.dumps({"error": str(e)}))
' 2>/dev/null)

PKG_VERSIONS=$(python3 -c '
import json
pkgs = ["gymnasium", "stable_baselines3", "numpy", "scipy", "pandas", "yaml", "matplotlib", "tensorboard"]
out = {}
for p in pkgs:
    try:
        m = __import__(p)
        out[p] = getattr(m, "__version__", "unknown")
    except Exception as e:
        out[p] = None
print(json.dumps(out))
' 2>/dev/null)

NVIDIA_SMI=$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null || echo "not found")

ROS_DISTRO_VAL="${ROS_DISTRO:-not sourced}"
ROS2_PKG_COUNT=$(ros2 pkg list 2>/dev/null | wc -l)
PX4_MSGS_COUNT=$(ros2 interface list 2>/dev/null | grep -c px4_msgs)

if command -v MicroXRCEAgent >/dev/null 2>&1; then
    XRCE_AGENT="found at $(command -v MicroXRCEAgent)"
else
    XRCE_AGENT="not found"
fi

COLCON_VERSION=$(colcon version-check 2>/dev/null | head -1 || command -v colcon || echo "not found")

cat <<EOF
{
  "generated_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "os": $(json_str "$OS_INFO"),
  "kernel": $(json_str "$KERNEL"),
  "gazebo_version": $(json_str "$GZ_VERSION"),
  "px4": {
    "tag": $(json_str "$PX4_TAG"),
    "branch": $(json_str "$PX4_BRANCH"),
    "commit": $(json_str "$PX4_COMMIT")
  },
  "python_version": $(json_str "$PYTHON_VERSION"),
  "conda_env": $(json_str "${CONDA_ENV:-none}"),
  "torch": ${TORCH_INFO:-null},
  "packages": ${PKG_VERSIONS:-null},
  "nvidia_smi": $(json_str "$NVIDIA_SMI"),
  "ros_distro": $(json_str "$ROS_DISTRO_VAL"),
  "ros2_pkg_count": $ROS2_PKG_COUNT,
  "px4_msgs_interfaces_found": $PX4_MSGS_COUNT,
  "micro_xrce_agent": $(json_str "$XRCE_AGENT"),
  "colcon": $(json_str "$COLCON_VERSION")
}
EOF
