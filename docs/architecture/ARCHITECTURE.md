# DRILLPULSE — Software Architecture Specification

## 1. High-Level Architecture Overview

DrillPulse implements a dual-tier distributed robotics software architecture split across:
1. **Onboard Rover Computational Unit (rover_ws)**: Embedded Ubuntu computer executing hard real-time sensor processing, state estimation, safety watchdogs, roll-recovery FSM, Nav2 path planning, and frontier discovery.
2. **Base Station Terminal (station_ws)**: Surface control terminal hosting the ground LoRa transceiver gateway, dual-panel mission web console (Port 8080), and shift reporting engine.

## 2. Onboard Subsystem Breakdown

### 2.1 Sensor Drivers & Hardware Layer
- **drillpulse_hardware (esp32_hardware_bridge_node)**: Communicates with the onboard ESP32 via /dev/ttyUSB0 at 115200 baud. Ingests raw 6-wheel optical encoder pulses and translates teleoperation velocity targets (/cmd_vel) into differential PWM commands.
- **rplidar_ros (rplidar_node)**: Slamtec LiDAR node configured for 360 obstacle detection on /dev/ttyUSB1 at 115200 baud, publishing on /scan.
- **drillpulse_imu (sparkfun_imu_node)**: I2C/SPI driver for the 9-DOF ICM-20948 sensor on /dev/ttyUSB2. Publishes calibrated angular rates, linear acceleration, and orientation quaternions on /imu/data.

### 2.2 Odometry & State Estimation Layer
- **wheel_odometry_node (Mode 1)**: Converts 6-wheel RPM feedback into forward linear velocity and differential yaw rates using median aggregation across all active wheels. Publishes /odom_encoder.
- **imu_odometry_node (Mode 2)**: Integrates linear accelerometer data and gyro angular velocities using exponential velocity damping. Assigns monotonically expanding pose covariances to reflect open-loop drift. Publishes /odom_imu.
- **threshold_consistency_node**: Continuous cross-validation engine comparing encoder velocity vs IMU integration. Assigns one of 5 health states (CONSISTENT, ENCODER_DEGRADED, IMU_DEGRADED, SENSOR_DISAGREEMENT, SENSOR_TIMEOUT) and conditionally conditions the encoder covariance matrix.
- **odometry_manager_node & ekf_node (Mode 3)**: Coordinates EKF sensor fusion via robot_localization. Dynamically ensures exactly ONE node publishes the odom -> base_link transform.

### 2.3 Navigation & Exploration Layer
- **mission_manager_node**: High-level system supervisor executing job handshakes, waypoint queue dispatching, and mode isolation between EXPLORED and UNEXPLORED modes.
- **frontier_exploration_node**: Unexplored exploration engine running a Breadth-First Search (BFS) grid traverser over the SLAM costmap. Groups open frontier cells into clusters, computes true information gain and travel distance costs, and feeds exploration goals into Nav2.

### 2.4 Safety & Recovery Layer
- **safety_monitor_node**: Monitors hardware heartbeat, stalls, battery voltage drop (< 10.5V), gas hazard thresholds, and communications timeouts. Injects E-STOP upon hazard detection.
- **recovery_controller_node**: Manages dual linear recovery jacks. Responds to Key 6 (LEFT_RECOVERY), Key 7 (RIGHT_RECOVERY), and ESC (STOP_RECOVERY) with runtime bounding (3.0s maximum extension) and thermal cooldown gating.

## 3. Communication & Video Separation
- **Telemetry & Command Mesh (LoRa)**: Structured binary framing with 4-byte CRC32 running at 915 MHz (or simulated UDP ports 14550/14551). Payload size is strictly controlled under 220 bytes.
- **Visual Streaming (WiFi / HaLow)**: Independent RTSP/MJPEG video pipeline on port 8081. Never mixed with LoRa bandwidth. Clearly displays [SIMULATION] watermark overlay when running without physical camera hardware.
