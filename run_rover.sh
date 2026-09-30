#!/usr/bin/env bash
# ============================================================
# DRILLPULSE — Onboard Rover Bringup Script
# Sourced environment, overlays, and launches full rover stack.
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/setup_environment.sh"

if [ -f "${SCRIPT_DIR}/rover_ws/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/rover_ws/install/setup.bash"
else
    echo "ERROR: rover_ws not built! Run ./build_all.sh first."
    exit 1
fi

echo "========================================================="
echo "Launching DrillPulse Onboard Rover Stack..."
echo "========================================================="
ros2 launch drillpulse_bringup rover.launch.py "$@"
