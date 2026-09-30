#!/usr/bin/env bash
# =========================================================
# DRILLPULSE — Unified Environment Setup Script
# Sources ROS 2 Jazzy, activates Python virtual environment,
# and links patched diagnostic_updater ABI overlay.
# =========================================================

# 1. Source ROS 2 Jazzy base
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
else
    echo "ERROR: /opt/ros/jazzy/setup.bash not found!"
fi

# 2. Activate Python virtual environment
if [ -f "/home/loki/ros2_ws/.venv/bin/activate" ]; then
    source "/home/loki/ros2_ws/.venv/bin/activate"
fi

# 3. Prepend ABI compatibility overlay for robot_localization in Ubuntu 24.04
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_LIB="${SCRIPT_DIR}/diagnostic_updater_overlay/opt/ros/jazzy/lib"
if [ -d "${OVERLAY_LIB}" ]; then
    export LD_LIBRARY_PATH="${OVERLAY_LIB}:${LD_LIBRARY_PATH}"
fi

export DRILLPULSE_ROOT="${SCRIPT_DIR}"
