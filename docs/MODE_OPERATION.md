# DRILLPULSE — OPERATIONAL MODES ARCHITECTURE

## 1. Overview
The DrillPulse rover operates in two mutually exclusive autonomous mission modes:
- **MODE A — EXPLORED MINE AREA** (Known pre-surveyed map)
- **MODE B — UNEXPLORED MINE AREA** (Unknown environment / active exploration)

---

## 2. Mode Comparison Matrix

| Architectural Dimension | MODE A — EXPLORED | MODE B — UNEXPLORED |
| :--- | :--- | :--- |
| **Map Source** | Pre-built map file (`maps/rover_map.yaml`) | Generated in real-time from LiDAR scans |
| **Map Publisher** | `nav2_map_server` | `slam_toolbox` |
| **Localization Method** | `nav2_amcl` (Adaptive Monte Carlo Localization) | `slam_toolbox` scan-matching & graph optimization |
| **Transform `map -> odom`** | Broadcasted by `amcl` | Broadcasted by `slam_toolbox` |
| **SLAM Status** | **STRICTLY DISABLED** | **ACTIVE** (`async_slam_toolbox_node`) |
| **AMCL Status** | **ACTIVE** | **STRICTLY DISABLED** |
| **Goal Generation** | Operator waypoint dispatch via dashboard | Autonomous frontier exploration node |
| **Dashboard Display** | Explored Panel active/green; Unexplored disabled | Unexplored Panel active/green; Explored disabled |
| **Safety Returns** | Autonomous return to mission origin | Autonomous return to mission origin |

---

## 3. Mode A: Explored Operational Workflow

1. Operator clicks `ACTIVATE EXPLORED PATROL` on ground station dashboard.
2. Dashboard emits `/rover/job_assignment` with `assigned_job: EXPLORED`.
3. `mission_manager_node` locks current rover pose $(x_0, y_0, \theta_0)$ as mission start origin.
4. `nav2_map_server` and `nav2_amcl` initialize. `slam_toolbox` remains disabled.
5. Rover receives particles and localizes against the pre-built map.
6. Operator clicks target coordinate on the web dashboard canvas.
7. Canvas coordinate converts to world coordinates $(x_w, y_w)$ and dispatches to `/goal_pose`.
8. Nav2 `planner_server` (A* global planner) and `controller_server` (DWB local planner) guide rover safely.
9. Continuous obstacle checking via `/scan` updates the local costmap in real-time.
10. Operator or system can command `RETURN_HOME` at any moment.

---

## 4. Mode B: Unexplored Operational Workflow

1. Operator clicks `ACTIVATE UNEXPLORED EXPLORATION` on dashboard.
2. Dashboard emits `/rover/job_assignment` with `assigned_job: UNEXPLORED`.
3. `mission_manager_node` locks $(x_0, y_0, \theta_0)$ as mission origin.
4. `slam_toolbox` initializes in async mapping mode. AMCL remains disabled.
5. LiDAR scans are matched into an incremental 2D occupancy grid published on `/map`.
6. `frontier_exploration_node` inspects `/map`, detects free/unknown boundary cells, applies obstacle clearance and distance filters, and selects the optimal frontier.
7. Dispatched frontier goal is routed through Nav2 `planner_server` and `controller_server`.
8. Upon reaching target or encountering boundary limits, the next frontier cluster is processed.
9. If no reachable frontiers remain, `frontier_exploration_node` reports `EXPLORATION_COMPLETE`.
10. Critical battery ($< 10.5\text{V}$) or link silence ($> 10\text{s}$) halts exploration and executes safe return.
