# DRILLPULSE — Coordinate Frames & TF Hierarchy Specification

## 1. REP-105 Compliant Transform Hierarchy
map -> odom -> base_link -> laser, imu_link, camera_link, thermal_link

## 2. Transform Ownership Rules
1. map -> odom:
   - In Unexplored Mode: slam_toolbox owns and broadcasts map -> odom. AMCL is disabled.
   - In Explored Mode: amcl owns and broadcasts map -> odom. SLAM Toolbox is disabled.
2. odom -> base_link:
   - In Mode 3 Fused: ekf_node (robot_localization) is the sole dynamic publisher.
   - In manual fallback: odometry_manager_node coordinates TF publishing.
3. Static Sensor Offsets (base_link -> sensor):
   - base_link -> laser: [0.25m, 0.00m, 0.20m]
   - base_link -> imu_link: [0.00m, 0.00m, 0.15m]
   - base_link -> camera_link: [0.30m, 0.00m, 0.25m]
   - base_link -> thermal_link: [0.30m, 0.05m, 0.25m]
