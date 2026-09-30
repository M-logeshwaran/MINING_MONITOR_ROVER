# DrillPulse Version 1 Prototype Reference

## Overview
The `ros2_proto_ws` directory contains the earlier physical proof-of-concept prototype developed for initial validation of subterranean mobility, basic camera streaming, and simple teleoperation.

## Packages
1. **`drillpulse_autonomous`**: Initial obstacle detection using OpenCV corridor edge detection and ultrasonic distance ranging. Point-to-point goal navigation without Nav2 stack.
2. **`drillpulse_control`**: Basic differential motor bridge logic communicating with an Arduino Uno / ESP32 over serial or Wi-Fi.
3. **`rover_dashboard`**: Early single-page web dashboard displaying camera stream and manual joystick buttons.

## Differences Between Version 1 and Current SIH Architecture
- **Kinematics**: Version 1 used simple 2-wheel/4-wheel differential assumptions. The current SIH architecture uses a 6-wheel rocker-bogie kinematic model with STM32 1040 PPR optical encoder feedback and dynamic wheel-slip detection.
- **Navigation**: Version 1 used direct reactive heading steering. The current architecture employs the full ROS 2 Nav2 stack (Navfn planner + DWB controller) with AMCL particle filter localization and SLAM Toolbox asynchronous mapping.
- **Communication**: Version 1 used a standard monolithic Wi-Fi connection. The current architecture deploys an asymmetrical dual-channel hybrid system (Sub-GHz LoRa mesh for <70B critical telemetry + Wi-Fi HaLow 802.11ah for video/maps) with a 4-tier priority ring buffer and 6-state connection quality state machine.
- **Safety & Recovery**: Version 1 had basic ultrasonic obstacle stops. The current architecture features DGMS-compliant multi-gas hazard supervision (CH4, CO, Temp), rollover detection at |roll| >= 50 deg, and dual 150mm linear actuator self-righting jacks with stroke/thermal interlocks.
