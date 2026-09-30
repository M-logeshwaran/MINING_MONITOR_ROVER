# DrillPulse — Canonical ROS 2 Topic Contract

**Document:** `docs/TOPIC_CONTRACT.md`  
**Status:** Authoritative Specification  
**Architecture:** Ubuntu 24.04 LTS | ROS 2 Jazzy

---

## 1. Primary Principle

There is **ONE authoritative topic name** for every distinct capability in the DrillPulse ecosystem. Multiple conflicting aliases and fragmented topic namespaces are deprecated and eliminated.

---

## 2. Canonical Topic Registry

| Topic Name | Message Type | Canonical Publisher | Canonical Subscriber | Function & Role |
|---|---|---|---|---|
| **`/scan`** | `sensor_msgs/LaserScan` | `rplidar_ros` (`rplidar_node`) | SLAM Toolbox, Nav2 Costmaps | 2D LiDAR planar ranges |
| **`/imu/data`** | `sensor_msgs/Imu` | `drillpulse_imu` (`sparkfun_imu_node`) | `drillpulse_odometry`, `robot_localization` | Calibrated orientation, angular vel, linear accel |
| **`/odom_encoder`** | `nav_msgs/Odometry` | `drillpulse_odometry` (`wheel_odometry_node`) | `drillpulse_odometry` (evaluator), EKF | 6-wheel differential kinematics odometry |
| **`/odom_imu`** | `nav_msgs/Odometry` | `drillpulse_odometry` (`imu_odometry_node`) | `drillpulse_odometry` (evaluator) | Expanding covariance dead-reckoning |
| **`/odom`** | `nav_msgs/Odometry` | `robot_localization` (`ekf_node`) | Nav2, SLAM Toolbox, Dashboard | Fused dynamic state (`odom -> base_link`) |
| **`/odom/diagnostics`** | `drillpulse_msgs/OdometryDiagnostics` | `drillpulse_odometry` (`threshold_consistency_node`) | Dashboard, Report Gen, Mission FSM | 5-state consistency & slip monitor |
| **`/rover/telemetry`** | `drillpulse_msgs/RoverTelemetry` | `drillpulse_hardware` (`motor_telemetry_bridge`) | LoRa Transceiver, Dashboard, Report Gen | Full rover battery, motor & sensor state |
| **`/rover/link_status`** | `drillpulse_msgs/LinkStatus` | `drillpulse_transport` (`lora_transceiver_node`) | Dashboard, Mission Manager | LoRa mesh & link quality (RSSI, loss, latency) |
| **`/rover/mission_state`** | `drillpulse_msgs/MissionState` | `drillpulse_mission` (`mission_manager_node`) | LoRa Transceiver, Dashboard, Logger | Mission FSM state, active job, progress |
| **`/rover/hazard_event`** | `drillpulse_msgs/HazardEvent` | `drillpulse_environment` (`environment_monitor_node`) | LoRa Transceiver, Dashboard, Logger | Methane, CO, O2, rollover or trap alerts |
| **`/rover/safety_status`** | `drillpulse_msgs/SafetyStatus` | `drillpulse_safety` (`safety_monitor_node`) | Dashboard, LoRa Transceiver, Arbiter | Real-time safety, tilt hazard & E-stop states |
| **`/rover/navigation_status`** | `drillpulse_msgs/NavigationStatus` | `drillpulse_navigation` | Dashboard, LoRa Transceiver | Nav2 goal progress, frontiers, planner status |
| **`/rover/cmd_vel`** | `geometry_msgs/Twist` | Nav2 / Dashboard Manual Teleop | `drillpulse_hardware` (`motor_telemetry_bridge`) | Linear and angular velocity drive commands |
| **`/rover/return_to_start`** | `std_msgs/String` | Dashboard / Station Operator | `drillpulse_navigation` / `drillpulse_mission` | Command triggering autonomous return to start pose |
| **`/rover/target_pose`** | `geometry_msgs/PoseStamped` | Dashboard Interactive Map Click | Nav2 (`/goal_pose` bridge) | 2D navigation goal waypoint |
| **`/rover/recovery_command`** | `drillpulse_msgs/RecoveryCommand` | Dashboard (Keys 6, 7, ESC) / Station | `drillpulse_recovery` (`recovery_controller_node`) | Self-righting actuator commands |
| **`/rover/recovery_status`** | `drillpulse_msgs/RecoveryStatus` | `drillpulse_recovery` (`recovery_controller_node`) | Dashboard, Station LoRa Transceiver | Actuator extension state, roll, cooldown |
| **`/rover/emergency_stop`** | `std_msgs/Bool` | Operator E-Stop Button / ESC / Watchdog | `drillpulse_safety` (`safety_monitor_node`) | Master safety cutoff trigger |
| **`/rover/job_assignment`** | `drillpulse_msgs/JobAssignment` | Station Operator Dispatch | `drillpulse_mission` (`mission_manager_node`) | Operational mode assignment (`EXPLORED`, `UNEXPLORED`) |

---

## 3. Deprecated Topics & Redirections

To preserve backward compatibility during transition, deprecation bridges map:
- `/drillpulse/telemetry` → `/rover/telemetry`
- `/drillpulse/rover_telemetry` → `/rover/telemetry`
- `/mission/state` → `/rover/mission_state`
- `/hazards/events` → `/rover/hazard_event`
- `/drillpulse/hazard_event` → `/rover/hazard_event`
- `/recovery/command` → `/rover/recovery_command`
- `/recovery/status` → `/rover/recovery_status`
- `/drillpulse/safety/estop` → `/rover/emergency_stop`
- `/station/job_assignment` → `/rover/job_assignment`
