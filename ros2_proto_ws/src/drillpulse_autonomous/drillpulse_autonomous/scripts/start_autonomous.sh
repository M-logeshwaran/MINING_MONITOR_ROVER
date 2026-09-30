#!/bin/bash
# ==============================================================================
# DRILLPULSE AUTONOMOUS SYSTEM - START SCRIPT
# Usage:
#   bash scripts/start_autonomous.sh [free|point] [goal_x] [goal_y]
# Examples:
#   bash scripts/start_autonomous.sh free
#   bash scripts/start_autonomous.sh point 5 8
# ==============================================================================

MODE=${1:-free}
GOAL_X=${2:-0.0}
GOAL_Y=${3:-0.0}

echo -e "\e[1;34m[AUTO]\e[0m Starting DrillPulse Autonomous System in mode: ${MODE}"

if [ "$MODE" == "point" ]; then
    echo -e "\e[1;34m[AUTO]\e[0m Target Goal: X=${GOAL_X}, Y=${GOAL_Y}"
    ros2 launch drillpulse_autonomous autonomous.launch.py mode:=point goal_x:=${GOAL_X} goal_y:=${GOAL_Y}
else
    ros2 launch drillpulse_autonomous autonomous.launch.py mode:=free
fi
