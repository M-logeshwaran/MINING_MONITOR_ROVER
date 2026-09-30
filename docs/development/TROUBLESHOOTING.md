# DRILLPULSE — Troubleshooting & Diagnostic Guide

## 1. Environment & Build Issues
- diagnostic_updater ABI on Ubuntu 24.04: Source setup_environment.sh to link diagnostic_updater_overlay.
- Permission issues on clean build: Ensure setup.cfg uses /lib/<package_name>.

## 2. Hardware Connectivity Issues
- Missing /dev/ttyUSB*: Verify user dialout group (sudo usermod -aG dialout loki).
  Fallback nodes automatically activate SIMULATION mode with watermarks.
- Dashboard Disconnected: Verify UDP ports 14550/14551 are accessible.
