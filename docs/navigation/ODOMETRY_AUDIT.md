# DRILLPULSE — ODOMETRY & SENSOR FUSION AUDIT

## 1. Overview
The DrillPulse odometry architecture integrates 6-wheel rocker-bogie encoder feedback, SparkFun ICM-20948 IMU telemetry, and dynamic threshold consistency monitoring to provide a drift-bounded, fail-safe pose estimate.

---

## 2. Wheel Odometry (`wheel_odometry_node.py`)

* **Suspension & Drivetrain:** 6-wheel rocker-bogie differential drive.
* **Physical Constants:**
  - Wheel Diameter $D = 0.22\text{ m}$ ($r = 0.11\text{ m}$, Circumference $C = \pi D = 0.69115\text{ m}$).
  - Effective Track Width $L = 0.90\text{ m}$.
* **Multi-Wheel Median Filtering:**
  - Left side RPM: $\text{median}(LF, LM, LB)$
  - Right side RPM: $\text{median}(RF, RM, RB)$
  - Rejects single-wheel slippage or terrain loss-of-contact.
* **Kinematics & Exact Arc Integration:**
  $$\begin{aligned}
  v_{\text{left}} &= \frac{\text{RPM}_{\text{left}}}{60} \cdot \pi D, \quad v_{\text{right}} = \frac{\text{RPM}_{\text{right}}}{60} \cdot \pi D \\
  v_{\text{linear}} &= \frac{v_{\text{right}} + v_{\text{left}}}{2}, \quad \omega_z = \frac{v_{\text{right}} - v_{\text{left}}}{L}
  \end{aligned}$$
* **Covariance Assignment:**
  - Velocity: $\sigma_{vx}^2 = 0.02$, $\sigma_{vy}^2 = 10^6$ (non-holonomic constraint), $\sigma_{\omega z}^2 = 0.05$.
  - Pose: $\sigma_x^2 = 0.05$, $\sigma_y^2 = 0.05$, $\sigma_{\theta}^2 = 0.10$.
* **TF Ownership:** `publish_tf: false` by default; prevents conflict with EKF.

---

## 3. IMU Dead Reckoning (`imu_odometry_node.py`)

* **Sensor:** SparkFun ICM-20948 (9-DOF IMU @ 50 Hz).
* **Bias Calibration:** Empirical stationary biases subtracted from angular rates:
  - $\text{Bias}_x = -0.00310\text{ rad/s}$, $\text{Bias}_y = 0.00864\text{ rad/s}$, $\text{Bias}_z = -0.00388\text{ rad/s}$.
* **Drift Damping:** Exponential decay damping ($\alpha = 0.96$) bounds double-integration velocity runaway.
* **Unbounded Uncertainty Modeling:**
  - Pose covariance grows over time:
    $$\sigma_{x}^2(t) = \sigma_{x,0}^2 \cdot (1.0 + 0.1 \cdot \min(100, t))$$
  - Distinctly flags the IMU-only state as drift-prone dead reckoning.
* **TF Ownership:** `publish_tf: false` by default.

---

## 4. Robot Localization EKF Filter (`ekf.yaml`)

* **Package:** `robot_localization` (`ekf_node`)
* **Frequency:** 30.0 Hz
* **Two-Dimensional Mode:** `two_d_mode: true` (restricts pose to $x, y, \text{yaw}$).
* **Coordinate Frames:** `odom_frame: odom`, `base_link_frame: base_link`, `world_frame: odom`.
* **Authoritative Role:** SINGLE authoritative publisher of dynamic `odom -> base_link` transform.
* **Sensors Fused:**
  1. `odom0` (`/odom_encoder`): Fuses linear velocity $v_x$ and yaw rate $\omega_z$.
  2. `imu0` (`/sparkfun/imu/data`): Fuses rotational angular velocity $\omega_z$.

---

## 5. Threshold Consistency Checker (`threshold_consistency_node.py`)

Cross-validates kinematic wheel odometry against inertial IMU telemetry:

| Consistency State | Trigger Condition | System Action |
| :--- | :--- | :--- |
| `CONSISTENT` | $\Delta v < 0.35\text{ m/s}$ and $\Delta \omega_z < 0.30\text{ rad/s}$ | Nominal full EKF fusion |
| `ENCODER_DEGRADED` | $\Delta v > 0.35\text{ m/s}$ (wheel slip detected) | Encoder confidence penalized (0.7x) |
| `IMU_DEGRADED` | $\Delta \omega_z > 0.30\text{ rad/s}$ (gyroscope drift) | IMU confidence penalized (0.7x) |
| `SENSOR_DISAGREEMENT`| Both $\Delta v$ and $\Delta \omega_z$ exceed $1.8\times$ thresholds | Fallback gating engaged |
| `SENSOR_TIMEOUT` | No data from encoder or IMU for $> 0.50\text{ s}$ | Watchdog triggers failsafe halt |
