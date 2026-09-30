# DRILLPULSE — Multi-Modal Odometry & Sensor Consistency Specification

## 1. The Three Odometry Modes
- Mode 1: 6-Wheel Median Differential Odometry (/odom_encoder)
  Computes track speeds using median of 3 active wheels per side:
  omega_left = median(w_L1, w_L2, w_L3), omega_right = median(w_R1, w_R2, w_R3).
  Physical parameters: Wheel diameter 0.110m, Track width 0.340m.
- Mode 2: IMU Dead Reckoning (/odom_imu)
  [IMU DEAD RECKONING — DRIFTING (NOT GROUND TRUTH)]
  Integrates linear acceleration with exponential damping (0.96) and acceleration deadband.
  Expanding covariance: sigma_x^2(t) = 0.50 + 0.05 * t.
- Mode 3: Fused EKF Odometry (/odom)
  robot_localization ekf_node fusing /odom_encoder/conditioned and /imu/data.

## 2. Sensor Consistency Monitoring (5-State Engine)
- CONSISTENT: Nominal agreement (|dv| <= 0.35 m/s, |dw| <= 0.50 rad/s).
- ENCODER_DEGRADED: Wheel slip detected (|dv| > 0.35 m/s). Covariance scaled 10x.
- IMU_DEGRADED: Angular divergence (|dw| > 0.50 rad/s). IMU covariance scaled 10x.
- SENSOR_DISAGREEMENT: Severe multi-axis divergence. Covariance scaled 50x.
- SENSOR_TIMEOUT: Watchdog trigger after 1.0s without updates.
