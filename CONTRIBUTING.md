# Contributing to DrillPulse

Thank you for your interest in contributing to the **DrillPulse** subterranean mine rover project!

## Development Workflow
1. **Target Environment**: Ubuntu 24.04 LTS with ROS 2 Jazzy Jalisco and Python 3.12.
2. **Branching**: Create feature branches from `main` (`feature/subsystem-name` or `bugfix/issue-description`).
3. **Workspace Isolation**:
   - `rover_ws`: Onboard compute nodes running on Raspberry Pi 5.
   - `station_ws`: Base station operator console and telemetry bridges.
   - `hardware/`: Embedded microcontroller firmware (Arduino / STM32).
   - `ros2_proto_ws`: Version-1 prototype reference (read-only reference; do not modify).

## Code Standards
- **Python**: PEP 8 compliance, clear docstrings, explicit ROS 2 parameter declarations, dynamic environment resolution via `DRILLPULSE_ROOT`.
- **C/C++**: C++17 for Arduino/host nodes, C11 for STM32 firmware. No dynamic memory allocation in embedded interrupt routines.
- **Topics & Interfaces**: All topic names and message formats must adhere to the canonical registry documented in `docs/development/TOPIC_CONTRACT.md`.

## Pre-Commit Verification
Before submitting a pull request, ensure all static checks and automated tests pass:
```bash
source setup_environment.sh
./build_all.sh
python3 scripts/validate_interfaces.py
./test_all.sh
```
