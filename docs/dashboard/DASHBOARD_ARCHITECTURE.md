# DRILLPULSE — OPERATOR DASHBOARD ARCHITECTURE
**Step 4: Dual-Panel Mode Locking, Zero-Fake-Data Enforcement, Interactive Canvas & Controls**
**Date:** 2026-09-29
**Platform:** ROS 2 Jazzy, FastAPI / WebSocket, HTML5 Canvas, Vanilla JS

---

## 1. Dashboard Architecture Overview

The DrillPulse Ground Station dashboard provides an operations console for underground search-and-rescue and mine surveying. It connects to the ROS 2 station environment via `dashboard_node.py` which exposes WebSocket telemetry and REST command endpoints to the browser UI (`web/index.html`, `web/app.js`, `web/style.css`).

```
+---------------------------------------------------------------------------------+
|                                OPERATOR CONSOLE                                 |
+---------------------------------------------------------------------------------+
| [HEADER]: MODE: [EXPLORED]  | LINK: [GOOD -65dBm] | BAT: [84% 72m] | [RETURN TO START] |
+-------------------------------------------------------+-------------------------+
|                PRIMARY INTERACTIVE MAP                |   TELEMETRY & HAZARD    |
|                                                       | - CH4: 0.05% | CO: 4ppm |
| - Mode A: Explored (Static Map + AMCL Pose)           | - CO2: 450ppm| Dust: 12 |
| - Mode B: Unexplored (Dynamic SLAM Grid + Frontier)   | - Temp: 24C  | Hum: 68% |
| - Interactive Canvas Click:                           +-------------------------+
|   Converts (px, py) -> Metric (gx, gy) -> Dispatch    |  DECOUPLED VIDEO FEEDS  |
| - Live Target Reticle (Red Crosshair)                 | - Optical: [STREAMING]  |
| - Robot Footprint & Heading Arrow                     | - Thermal: [STREAMING]  |
+-------------------------------------------------------+-------------------------+
| [FOOTER / CONTROLS]:                                                            |
| - Teleoperation: W/A/S/D or Arrow Keys (0.2 m/s, 0.4 rad/s)                     |
| - Linear Recovery: [Key 6] 45 deg Swivel | [Key 7] Actuator Extension            |
| - Safety: [ESC] EMERGENCY STOP | CMD STATUS: [WAYPOINT: ACCEPTED]               |
+---------------------------------------------------------------------------------+
```

---

## 2. Dual-Panel Mode Locking

To prevent operator error and ensure pipeline clarity:
- **Explored Panel (Mode A):** Active when navigating known underground tunnels.
  - Displays the high-resolution pre-surveyed metric map.
  - Localized with AMCL particle cloud and Nav2 global costmap.
  - Dispatches goals directly to `/goal_pose`.
- **Unexplored Panel (Mode B):** Active in uncharted collapsed zones or new headings.
  - Displays live occupancy grid generated online by `slam_toolbox`.
  - Highlights frontier exploration cluster boundaries and active exploration centroids.
- **Mutex Mode Locking:** The UI locks controls to the active mode. Switching modes requires an explicit confirmation modal and issues `/rover/request_mode` (`EXPLORED` or `UNEXPLORED`). Navigation tools for the inactive mode are deactivated.

---

## 3. Zero-Fake-Data Enforcement

Underground life-safety protocols strictly prohibit displaying misleading placeholders (such as showing 100% battery or 0 ppm gas when disconnected):
1. **Disconnected / Uninitialized State:**
   - All numerical displays show `--` or `NO DATA`.
   - Badges display `[DISCONNECTED]` with red styling.
2. **Stale Data Watchdog:**
   - If no telemetry packet arrives within **3.5 seconds**, the dashboard automatically sets `telemetry_stale = true`.
   - A high-visibility alert banner (`STALE TELEMETRY — SENSORS FROZEN`) overlays the HUD.
   - Gas indicators are flagged with warning icons rather than displaying stale nominal readings.
3. **Decoupled Video Status:**
   - If optical or thermal video frames stop arriving for > 2.5 seconds, the respective video viewport renders `RGB: OFFLINE` or `THERMAL: OFFLINE`.
   - The low-bandwidth LoRa telemetry HUD continues operating smoothly without interruption.

---

## 4. Interactive Canvas Click-to-Coordinate Dispatch

Operators can dispatch autonomous navigation goals directly by interacting with the map canvas:
1. Operator clicks on the HTML5 map canvas.
2. The UI transforms canvas client coordinates to internal image pixel coordinates `(px, py)`.
3. Metric world transformation:
   $$x_{\text{world}} = x_{\text{origin}} + (px \times \text{resolution})$$
   $$y_{\text{world}} = y_{\text{origin}} + ((\text{height} - py) \times \text{resolution})$$
4. The calculated $(x, y)$ values automatically populate target coordinate input fields.
5. A red targeting reticle is rendered at the target location.
6. Pressing **Dispatch Goal** sends `POST /api/waypoint` with JSON `{x: gx, y: gy, yaw: 0.0}`, which publishes a `geometry_msgs/msg/PoseStamped` to Nav2.

---

## 5. Command Lifecycle & Explicit ACK Tracking

Commands dispatched to the rover follow an end-to-end lifecycle state machine:
```
[SENT]  --->  [RECEIVED]  --->  [ACCEPTED]  --->  [EXECUTING]  --->  [COMPLETED]
                                      \
                                       +------->  [REJECTED] / [TIMEOUT]
```
- The dashboard displays the live command lifecycle status in the status bar (e.g. `CMD: WAYPOINT: ACCEPTED`).
- If an ACK is not received within 3.0 seconds, the status transitions to `TIMEOUT` and alerts the operator.

---

## 6. Keyboard Hotkeys & Operator Controls

| Key / Binding | Action | Target ROS Topic / Service |
|---|---|---|
| **W / Up Arrow** | Forward Velocity ($+0.2\text{ m/s}$) | `/rover/cmd_vel` |
| **S / Down Arrow** | Reverse Velocity ($-0.2\text{ m/s}$) | `/rover/cmd_vel` |
| **A / Left Arrow** | Turn Left ($+0.4\text{ rad/s}$) | `/rover/cmd_vel` |
| **D / Right Arrow** | Turn Right ($-0.4\text{ rad/s}$) | `/rover/cmd_vel` |
| **Key 6** | Swivel Recovery ($45^\circ$) | `/rover/trigger_recovery` (`action: "SWIVEL_45"`) |
| **Key 7** | Actuator Extension Recovery | `/rover/trigger_recovery` (`action: "EXTEND_ACTUATORS"`) |
| **ESC** | Immediate Emergency Stop | `/rover/emergency_stop` |
| **Return to Start Button** | Return to $(x_0, y_0, \theta_0)$ | `/rover/return_home` |
