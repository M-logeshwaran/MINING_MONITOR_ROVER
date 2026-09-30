# DRILLPULSE — Canonical Topic Interface Contract

## 1. Canonical ROS 2 Topics
- /scan (sensor_msgs/msg/LaserScan): 360 Planar laser range data from RPLiDAR.
- /imu/data (sensor_msgs/msg/Imu): Calibrated 9-DOF orientation, angular rates & acceleration.
- /odom_encoder (nav_msgs/msg/Odometry): Mode 1: 6-wheel median differential wheel odometry.
- /odom_imu (nav_msgs/msg/Odometry): Mode 2: IMU-only dead reckoning with expanding covariance.
- /odom (nav_msgs/msg/Odometry): Mode 3: Official system odometry (fused EKF state).
- /odom/diagnostics (drillpulse_msgs/msg/OdometryDiagnostics): 5-State consistency status.
- /rover/telemetry (drillpulse_msgs/msg/RoverTelemetry): Aggregate battery, gas, pose packet.
- /rover/mission_state (drillpulse_msgs/msg/MissionState): Current mission phase and active mode.
- /rover/job (drillpulse_msgs/msg/JobAssignment): Mode assignment request (EXPLORED, UNEXPLORED).
- /rover/job_ack (drillpulse_msgs/msg/JobAssignment): Mode handshake confirmation.
- /rover/mission_command (drillpulse_msgs/msg/MissionCommand): Waypoint and E-Stop commands.
- /rover/recovery_command (drillpulse_msgs/msg/RecoveryCommand): Recovery trigger (Keys 6, 7, ESC).
- /rover/actuator_cmd (drillpulse_msgs/msg/RecoveryCommand): Low-level push-rod extension signal.
- /rover/recovery_status (std_msgs/msg/String): Current FSM recovery state.
- /cmd_vel (geometry_msgs/msg/Twist): Linear and angular velocity drive commands.
- /goal_pose (geometry_msgs/msg/PoseStamped): Target navigation waypoint in map frame.
