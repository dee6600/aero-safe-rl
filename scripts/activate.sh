#!/usr/bin/env bash
# Source this (do not execute it) to get a shell that can run everything in
# this project:
#
#   source scripts/activate.sh
#
# Conda supplies the interpreter (3.10.20) and the ML stack; ROS supplies rclpy
# from /opt/ros/humble on PYTHONPATH. Both are Python 3.10, so the C-extension
# ABI matches. Order matters -- see CLAUDE.md §0.
#
# Deliberately does nothing else. If a script needs more environment than this,
# that environment belongs in the script.

# shellcheck disable=SC1091
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate aero-safe-rl

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

_AERO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

_AERO_WS="$_AERO_ROOT/ros2_ws/install/setup.bash"
if [ -f "$_AERO_WS" ]; then
	# shellcheck disable=SC1090
	source "$_AERO_WS"
fi
unset _AERO_WS

# simulation/instance_spec.py is a plain (non-ROS) package at the repo root,
# imported by every aero_bridge node (M2 on) as the single source of instance
# identity. It is not installed by colcon, so it only becomes importable via
# PYTHONPATH -- prepended so it can never be shadowed by an installed package
# of the same name.
export PYTHONPATH="$_AERO_ROOT:${PYTHONPATH:-}"
unset _AERO_ROOT
