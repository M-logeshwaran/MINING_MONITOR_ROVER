#!/usr/bin/env bash
# ============================================================
# DRILLPULSE — Base Station Bringup Script
# Sources environment, overlays, and launches full station stack
# including Dual-Panel Web Dashboard and Transport Node.
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/setup_environment.sh"

if [ -f "${SCRIPT_DIR}/station_ws/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/station_ws/install/setup.bash"
else
    echo "ERROR: station_ws not built! Run ./build_all.sh first."
    exit 1
fi

echo "========================================================="
echo "Launching DrillPulse Base Station Stack..."
echo "Dashboard will be accessible at: http://localhost:8080"
echo "========================================================="
ros2 launch drillpulse_station station.launch.py "$@"
