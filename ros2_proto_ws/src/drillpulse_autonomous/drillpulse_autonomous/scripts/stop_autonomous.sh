#!/bin/bash
# ==============================================================================
# DRILLPULSE AUTONOMOUS SYSTEM - STOP SCRIPT
# Safely stops the autonomous node and sends emergency stop command to rover bridge
# ==============================================================================

echo -e "\e[1;31m[SAFETY]\e[0m Stopping DrillPulse Autonomous Package..."

# 1. Publish STOP command to Arduino bridge topic directly
ros2 topic pub --once /rover/command std_msgs/msg/String "{data: 'CMD,0,0'}" 2>/dev/null || true

# 2. Terminate running autonomous controller node
pkill -f autonomous_controller 2>/dev/null || true
pkill -f point_navigation 2>/dev/null || true

echo -e "\e[1;32m[AUTO]\e[0m DrillPulse Autonomous System stopped cleanly."
