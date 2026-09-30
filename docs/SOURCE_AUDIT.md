# DRILLPULSE — Reference Source Code Audit

## 1. Audit Guidelines & Preservation Policy
Reference codebases:
- SOURCE A: /home/loki/ros2_ws (Original DrillPulse Rover Codebase)
- SOURCE B: /home/loki/ros2_lidar_ws (Working RPLIDAR and Navigation Prototype)

Both source archives were treated as strictly read-only references. No files were modified, moved, renamed, or deleted within the reference directories.

## 2. Reused & Adapted Component Registry
- rplidar_ros (Source B): Preserved working RPLiDAR A1/A2 driver and /scan contract.
- sparkfun_imu_node (Source A): Added canonical /imu/data topic and orientation validation.
- esp32_hardware_bridge (Source A): Adapted for 6-wheel pulse parsing and PWM teleop.
- wheel_odometry_node (Source A): Refactored with 6-wheel median filter and geometry.
- imu_odometry_node (Source A): Refactored with expanding covariance drift modeling.
- threshold_consistency_node: Formalized 5-state health machine and covariance inflation.
- frontier_exploration_node: Implemented 8-connected BFS clustering and info gain.
- recovery_controller_node: Mapped Keys 6 & 7 with runtime bounding and cooldown.
- lora_transceiver_node: Added CRC32 binary framing and UDP simulation.
- dashboard_node: Enforced zero-fake-data policy and mutual exclusion panel locking.
