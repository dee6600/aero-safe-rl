#!/usr/bin/env bash
# Source this (do not execute it) to get a shell for the Isaac side of the
# project -- Isaac Sim 5.1 + Isaac Lab, Python 3.11 (CLAUDE.md §0):
#
#   source scripts/activate_isaac.sh
#
# The counterpart of scripts/activate.sh. The two environments never share a
# shell's Python path:
#   * ROS 2 Humble's Python 3.10 packages are removed from PYTHONPATH. This
#     machine's ~/.bashrc sources ROS, so every new shell starts with them,
#     and next to a 3.11 interpreter they fail with C-extension ABI errors
#     that look like a broken install.
#   * OMNI_KIT_ACCEPT_EULA=YES: Isaac Sim's first import otherwise asks for
#     licence acceptance interactively and hangs forever in a non-interactive
#     shell (found in M3b, docs/isaac_feasibility.md).

_AERO_NOUNSET=0
case "$-" in *u*) _AERO_NOUNSET=1 ;; esac
set +u

# shellcheck disable=SC1091
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate isaacsim

if [ -n "${PYTHONPATH:-}" ]; then
    PYTHONPATH=$(printf '%s' "$PYTHONPATH" | tr ':' '\n' | grep -v '^/opt/ros/' | paste -sd: -)
    if [ -z "$PYTHONPATH" ]; then unset PYTHONPATH; else export PYTHONPATH; fi
fi
export OMNI_KIT_ACCEPT_EULA=YES

[ "$_AERO_NOUNSET" = 1 ] && set -u
unset _AERO_NOUNSET
