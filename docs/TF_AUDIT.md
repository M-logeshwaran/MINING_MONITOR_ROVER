# DRILLPULSE — TF TRANSFORM HIERARCHY & OWNERSHIP AUDIT

## 1. Overview
This document specifies the exact coordinate frame hierarchy, dynamic transform owners, and static transform broadcasters in the DrillPulse autonomous system.

---

## 2. Complete Coordinate Frame Tree

```
map
 |
 | (Dynamic TF — Owned by AMCL in EXPLORED mode OR SLAM Toolbox in UNEXPLORED mode)
 v
odom
 |
 | (Dynamic TF — Owned SOLELY by robot_localization EKF filter)
 v
base_link
 |
 +----> laser               (Static TF: x=0.25, y=0.00, z=0.20)
 |
 +----> sparkfun_imu_link   (Static TF: x=0.00, y=0.00, z=0.15)
 |
 +----> imu_link            (Static TF: x=0.00, y=0.00, z=0.15 - Standard alias)
 |
 +----> camera_link         (Static TF: x=0.30, y=0.00, z=0.25)
 |
 +----> thermal_link        (Static TF: x=0.30, y=0.05, z=0.25)
```

---

## 3. Strict Single Dynamic Ownership Rules

| Parent Frame | Child Frame | Authoritative Publisher | Inactive Publishers (Strictly Prohibited) |
| :--- | :--- | :--- | :--- |
| `map` | `odom` | **Mode A:** `nav2_amcl`<br>**Mode B:** `slam_toolbox` | `amcl` and `slam_toolbox` must NEVER run concurrently. |
| `odom` | `base_link` | `robot_localization` (`ekf_filter_node`) | `wheel_odometry_node` (`publish_tf=False`)<br>`imu_odometry_node` (`publish_tf=False`)<br>`odometry_manager_node` (TF disabled in FUSED mode) |

---

## 4. Static Transform Broadcasters (`rover.launch.py`)

All static transforms are broadcast at startup via `tf2_ros/static_transform_publisher`:

1. `base_to_laser_broadcaster`:
   - Translation: $[0.25, 0.00, 0.20]\text{ m}$, Rotation: $[0, 0, 0, 1]$
2. `base_to_sparkfun_imu_broadcaster`:
   - Translation: $[0.00, 0.00, 0.15]\text{ m}$, Rotation: $[0, 0, 0, 1]$
3. `base_to_imu_link_broadcaster`:
   - Translation: $[0.00, 0.00, 0.15]\text{ m}$, Rotation: $[0, 0, 0, 1]$
4. `base_to_camera_broadcaster`:
   - Translation: $[0.30, 0.00, 0.25]\text{ m}$, Rotation: $[0, 0, 0, 1]$
5. `base_to_thermal_broadcaster`:
   - Translation: $[0.30, 0.05, 0.25]\text{ m}$, Rotation: $[0, 0, 0, 1]$

---

## 5. Verification Against Duplicate Publishers
1. Verified via `test_tf_tree.py` and `test_navigation_integration.py` (Test 2).
2. Confirmed that no node simultaneously broadcasts `odom -> base_link` with different timestamps or transform values.
