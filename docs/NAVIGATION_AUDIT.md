# DRILLPULSE — NAVIGATION & SLAM SUBSYSTEM AUDIT

## 1. Overview
This document audits the autonomous navigation stack, path planners, costmap configurations, SLAM Toolbox online mapping, AMCL localization, and return-to-start behaviors for the DrillPulse 6-wheel rocker-bogie rover.

---

## 2. Nav2 Architecture & Server Nodes

| Server Node | Package | Implementation Plugin | Purpose | Parameters / Constraints |
| :--- | :--- | :--- | :--- | :--- |
| `controller_server` | `nav2_controller` | `dwb_core::DWBLocalPlanner` | Local reactive trajectory tracking | Min vx: -0.20 m/s, Max vx: 0.45 m/s, Max vtheta: 1.0 rad/s |
| `planner_server` | `nav2_planner` | `nav2_navfn_planner::NavfnPlanner` | Global route planning | A* enabled (`use_astar: true`), `allow_unknown: true`, tolerance: 0.5m |
| `behavior_server` | `nav2_behaviors` | `Spin`, `BackUp`, `Wait` | Recovery behaviors on obstacle trap | Global frame: `odom`, robot frame: `base_link` |
| `bt_navigator` | `nav2_bt_navigator` | Nav2 Behavior Tree Engine | Coordinates execution flow | Target topic: `/goal_pose`, feedback on navigation progress |
| `lifecycle_manager` | `nav2_lifecycle_manager` | Lifecycle coordinator | State transitions for navigation nodes | Manages `controller`, `planner`, `behaviors`, `bt_navigator` |

---

## 3. Costmap Configuration & Rocker-Bogie Footprint

* **Platform Geometry:**
  - Length: 0.85 m, Width (track width): 0.90 m, Wheel diameter: 0.22 m
  - Effective Robot Radius: `0.45 m`
* **Local Costmap:**
  - Global Frame: `odom`, Robot Frame: `base_link`
  - Dimensions: 4.0m x 4.0m rolling window, resolution 0.05m
  - Observation Source: `/scan` (`sensor_msgs/LaserScan`, max obstacle height 2.0m)
  - Inflation Layer: Radius 0.55m, cost scaling factor 3.0
* **Global Costmap:**
  - Global Frame: `map`, Robot Frame: `base_link`
  - Layers: `static_layer` (from map_server or SLAM), `obstacle_layer` (from `/scan`), `inflation_layer`
  - Unknown space tracking: `track_unknown_space: true`

---

## 4. Mode Separation: AMCL vs SLAM Toolbox

```
+-----------------------------------------------------------------------------------+
|                                  OPERATIONAL MODES                                |
|                                                                                   |
|   MODE A: EXPLORED MINE AREA                       MODE B: UNEXPLORED MINE AREA   |
|   - Pre-surveyed map loaded from disk              - No prior map exists          |
|   - Node: nav2_map_server (publishes /map)         - Node: slam_toolbox           |
|   - Node: nav2_amcl (publishes map -> odom)        - Mode: async online mapping   |
|   - Authority: AMCL owns map -> odom               - Authority: SLAM owns map->odom|
|   - SLAM Toolbox: STRICTLY DISABLED                - AMCL / Map Server: DISABLED  |
+-----------------------------------------------------------------------------------+
```

Mutual exclusivity is enforced programmatically in `amcl.launch.py`, `slam.launch.py`, and `mission_manager_node.py`. The two localization providers never run concurrently.

---

## 5. Return-to-Start Navigation
* **Mission Start Origin Locking:** On mission initiation (first valid odometry read or mode assignment), `mission_manager_node` locks $(x_0, y_0, \theta_0)$.
* **Trigger Conditions:**
  1. Operator commands `RETURN_HOME` via web dashboard button or shortcut.
  2. Battery drops below 10.5V or SoC drops below 15% (`RETURNING_LOW_BATTERY`).
  3. Continuous communication loss exceeding 10.0 seconds (`RETURNING_COMM_LOSS`).
* **Execution:** Dispatches Nav2 `PoseStamped` goal targeting $(x_0, y_0, \theta_0)$ in the `map` coordinate frame. When arrival distance is $\le 0.35\text{ m}$, state transitions to `RETURNED_TO_START` / `AT_START_SAFE`.

---

## 6. Frontier Exploration Algorithm
* **Occupancy Grid Processing:** Evaluates cells in the `/map` topic (`0` = free, `100` = obstacle, `-1` = unknown).
* **Candidate Extraction:** Detects boundary cells with values $0 \le \text{grid}[r, c] \le 20$ adjacent to unknown cells ($-1$).
* **Gating Filters:**
  - Rejects cells within 2 cells of known obstacles ($> 50$).
  - Distance window: $1.0\text{ m} \le d \le 25.0\text{ m}$ from rover position.
  - Blacklist check: Disallows candidates that failed to reach target within 15 seconds.
  - Low-battery interlock: Blocks exploration when battery is critical.
* **Goal Orientation:** Target yaw faces radially outward toward the unexplored boundary.
