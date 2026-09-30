#!/usr/bin/env bash
# ============================================================
# DRILLPULSE — Master Build Script
# Builds both rover_ws and station_ws cleanly from source.
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/setup_environment.sh"

echo "========================================================="
echo "Building Rover Workspace (rover_ws)..."
echo "========================================================="
cd "${SCRIPT_DIR}/rover_ws"
colcon build --symlink-install

echo "========================================================="
echo "Building Station Workspace (station_ws)..."
echo "========================================================="
cd "${SCRIPT_DIR}/station_ws"
colcon build --symlink-install

echo "========================================================="
echo "DrillPulse Build Completed Successfully!"
echo "========================================================="
