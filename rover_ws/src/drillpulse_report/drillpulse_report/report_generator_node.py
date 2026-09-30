#!/usr/bin/env python3
# =========================================================
# FILE: report_generator_node.py
# PURPOSE: Logs rover mission telemetry, aggregates hazard events,
#          and compiles comprehensive mine safety reports in
#          HTML, JSON, and CSV format for onboard SD card storage
#          and workstation display.
# INPUT TOPICS:
#   - /drillpulse/telemetry (drillpulse_msgs/RoverTelemetry)
#   - /drillpulse/hazard_event (drillpulse_msgs/HazardEvent)
#   - /mission/state (drillpulse_msgs/MissionState)
#   - /odom/diagnostics (drillpulse_msgs/OdometryDiagnostics)
#   - /recovery/status (std_msgs/String)
#   - /drillpulse/generate_report (std_msgs/String)
# OUTPUT TOPICS:
#   - /drillpulse/report_status (std_msgs/String)
# CONTROL FLOW:
#   Subscribes to telemetry, hazard events, and state changes.
#   Maintains an in-memory chronological event journal and metrics buffer.
#   Appends raw records to SD-card JSONL log. Upon mission end or manual
#   request, compiles a self-contained HTML audit report and JSON summary.
# TF OWNERSHIP: None
# =========================================================

# =========================================================
# USER CONFIGURATION
# =========================================================
LOG_DIR = "/home/loki/SIH_REPO_FINAL/logs"
REPORT_DIR = "/home/loki/SIH_REPO_FINAL/reports"
AUTO_GENERATE_ON_COMPLETE = True
MAX_BUFFER_EVENTS = 5000
REPORT_TITLE = "DrillPulse Underground Mine Rover Mission & Safety Audit Report"
# =========================================================

import os
import json
import csv
import time
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from drillpulse_msgs.msg import RoverTelemetry, HazardEvent, MissionState, OdometryDiagnostics

class ReportGeneratorNode(Node):
    """Generates comprehensive mission and safety incident reports."""

    def __init__(self):
        super().__init__('report_generator_node')

        self.declare_parameter('log_dir', LOG_DIR)
        self.declare_parameter('report_dir', REPORT_DIR)
        self.declare_parameter('auto_generate', AUTO_GENERATE_ON_COMPLETE)

        self.log_dir = self.get_parameter('log_dir').value
        self.report_dir = self.get_parameter('report_dir').value
        self.auto_generate = self.get_parameter('auto_generate').value

        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.report_dir, exist_ok=True)

        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.sd_log_path = os.path.join(self.log_dir, f"sd_mission_{self.session_id}.jsonl")

        self.telemetry_history = []
        self.hazards = []
        self.state_history = []
        self.recovery_events = []
        self.latest_telemetry = None
        self.latest_state = "IDLE"
        self.start_time = time.time()

        # Subscribers
        self.sub_telem = self.create_subscription(
            RoverTelemetry, '/drillpulse/telemetry', self._on_telemetry, 10
        )
        self.sub_hazard = self.create_subscription(
            HazardEvent, '/drillpulse/hazard_event', self._on_hazard, 10
        )
        self.sub_state = self.create_subscription(
            MissionState, '/mission/state', self._on_mission_state, 10
        )
        self.sub_diag = self.create_subscription(
            OdometryDiagnostics, '/odom/diagnostics', self._on_diagnostics, 10
        )
        self.sub_recovery = self.create_subscription(
            String, '/recovery/status', self._on_recovery, 10
        )
        self.sub_trigger = self.create_subscription(
            String, '/drillpulse/generate_report', self._on_generate_trigger, 10
        )

        # Publisher
        self.pub_status = self.create_publisher(String, '/drillpulse/report_status', 10)

        self.get_logger().info(f"ReportGeneratorNode initialized. SD Log: {self.sd_log_path}")

    def _write_sd_log(self, record_type: str, data: dict):
        record = {
            "timestamp": time.time(),
            "iso_time": datetime.now().isoformat(),
            "type": record_type,
            "data": data
        }
        try:
            with open(self.sd_log_path, 'a') as f:
                f.write(json.dumps(record) + "\n")
        except Exception as e:
            self.get_logger().warn(f"Failed to write SD log: {e}")

    def _on_telemetry(self, msg: RoverTelemetry):
        self.latest_telemetry = {
            "battery_pct": round(msg.battery_percentage, 1),
            "temp_c": round(msg.ambient_temperature, 1),
            "humidity_pct": round(msg.ambient_humidity, 1),
            "ch4_ppm": round(msg.ch4_concentration, 2),
            "co_ppm": round(msg.co_concentration, 2),
            "o2_pct": round(msg.o2_concentration, 2),
            "dust_mg_m3": round(msg.dust_concentration, 2),
            "speed_mps": round(msg.linear_speed, 2),
            "pitch_deg": round(msg.pitch_angle, 1),
            "roll_deg": round(msg.roll_angle, 1),
            "pose_x": round(msg.pose_x, 3),
            "pose_y": round(msg.pose_y, 3),
            "lora_rssi": round(msg.lora_rssi, 1),
            "active_mode": msg.active_mode
        }
        # Keep manageable sample size in memory
        if len(self.telemetry_history) < MAX_BUFFER_EVENTS:
            self.telemetry_history.append((time.time(), self.latest_telemetry))
        self._write_sd_log("TELEMETRY", self.latest_telemetry)

    def _on_hazard(self, msg: HazardEvent):
        h = {
            "time": datetime.now().strftime("%H:%M:%S"),
            "hazard_type": msg.hazard_type,
            "severity": msg.severity,
            "description": msg.description,
            "value": round(msg.measured_value, 2),
            "threshold": round(msg.threshold_value, 2),
            "x": round(msg.location_x, 2),
            "y": round(msg.location_y, 2)
        }
        self.hazards.append(h)
        self._write_sd_log("HAZARD", h)
        self.get_logger().warn(f"Report logged HAZARD: {h['hazard_type']} ({h['severity']})")

    def _on_mission_state(self, msg: MissionState):
        prev = self.latest_state
        self.latest_state = msg.current_state
        event = {
            "time": datetime.now().strftime("%H:%M:%S"),
            "previous_state": prev,
            "current_state": msg.current_state,
            "active_job": msg.active_job,
            "frontiers_count": msg.discovered_frontiers_count
        }
        self.state_history.append(event)
        self._write_sd_log("STATE_TRANSITION", event)

        if self.auto_generate and msg.current_state in ["MISSION_COMPLETED", "ABORTED"]:
            self.generate_reports()

    def _on_diagnostics(self, msg: OdometryDiagnostics):
        diag = {
            "state": msg.fusion_state,
            "source": msg.selected_source,
            "vel_err": round(msg.velocity_error, 3),
            "yaw_err": round(msg.yaw_error, 3)
        }
        self._write_sd_log("ODOM_DIAG", diag)

    def _on_recovery(self, msg: String):
        r = {
            "time": datetime.now().strftime("%H:%M:%S"),
            "status": msg.data
        }
        self.recovery_events.append(r)
        self._write_sd_log("RECOVERY", r)

    def _on_generate_trigger(self, msg: String):
        self.get_logger().info(f"Manual report generation triggered: {msg.data}")
        self.generate_reports()

    def generate_reports(self):
        duration = round(time.time() - self.start_time, 1)
        json_path = os.path.join(self.report_dir, f"report_{self.session_id}.json")
        html_path = os.path.join(self.report_dir, f"report_{self.session_id}.html")

        # Compute summary metrics
        gas_ch4_max = max([t[1]["ch4_ppm"] for t in self.telemetry_history], default=0.0)
        gas_co_max = max([t[1]["co_ppm"] for t in self.telemetry_history], default=0.0)
        o2_min = min([t[1]["o2_pct"] for t in self.telemetry_history], default=20.9)
        temp_max = max([t[1]["temp_c"] for t in self.telemetry_history], default=0.0)

        report_data = {
            "title": REPORT_TITLE,
            "session_id": self.session_id,
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "duration_seconds": duration,
            "final_state": self.latest_state,
            "metrics": {
                "max_ch4_ppm": gas_ch4_max,
                "max_co_ppm": gas_co_max,
                "min_o2_pct": o2_min,
                "max_temperature_c": temp_max,
                "total_hazards_logged": len(self.hazards),
                "total_recoveries_triggered": len(self.recovery_events),
                "sd_log_file": self.sd_log_path
            },
            "hazards": self.hazards,
            "state_history": self.state_history,
            "recovery_history": self.recovery_events
        }

        # Write JSON
        try:
            with open(json_path, 'w') as f:
                json.dump(report_data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to write JSON report: {e}")

        # Write HTML
        try:
            hazard_rows = "".join([
                f"<tr class='{h['severity'].lower()}'><td>{h['time']}</td><td><strong>{h['hazard_type']}</strong></td><td><span class='badge {h['severity'].lower()}'>{h['severity']}</span></td><td>{h['description']}</td><td>{h['value']} (Limit: {h['threshold']})</td><td>({h['x']}, {h['y']})</td></tr>"
                for h in self.hazards
            ]) or "<tr><td colspan='6' class='empty-row'>No critical hazards detected during mission.</td></tr>"

            state_rows = "".join([
                f"<tr><td>{s['time']}</td><td>{s['previous_state']} &rarr; <strong>{s['current_state']}</strong></td><td>{s['active_job']}</td><td>{s['frontiers_count']}</td></tr>"
                for s in self.state_history
            ]) or "<tr><td colspan='4' class='empty-row'>No transitions recorded.</td></tr>"

            recovery_rows = "".join([
                f"<tr><td>{r['time']}</td><td>{r['status']}</td></tr>"
                for r in self.recovery_events
            ]) or "<tr><td colspan='2' class='empty-row'>No recovery triggers invoked.</td></tr>"

            html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>{REPORT_TITLE}</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #0b0f17; color: #e1e7ec; margin: 0; padding: 24px; }}
  .container {{ max-width: 1100px; margin: 0 auto; background: #131a26; border-radius: 10px; border: 1px solid #243247; padding: 32px; box-shadow: 0 8px 24px rgba(0,0,0,0.5); }}
  h1 {{ margin-top: 0; color: #38bdf8; font-size: 26px; border-bottom: 2px solid #243247; padding-bottom: 12px; }}
  h2 {{ color: #94a3b8; font-size: 18px; margin-top: 28px; border-bottom: 1px solid #1e293b; padding-bottom: 8px; }}
  .meta-grid {{ display: grid; grid-template-columns: repeat(4, 1fr); gap: 16px; margin-bottom: 24px; }}
  .meta-card {{ background: #1a2333; border: 1px solid #2d3c54; padding: 14px; border-radius: 6px; }}
  .meta-label {{ font-size: 11px; text-transform: uppercase; color: #64748b; letter-spacing: 0.05em; }}
  .meta-value {{ font-size: 20px; font-weight: bold; color: #f8fafc; margin-top: 4px; }}
  table {{ width: 100%; border-collapse: collapse; margin-top: 12px; font-size: 13px; }}
  th, td {{ padding: 10px 12px; text-align: left; border-bottom: 1px solid #1e293b; }}
  th {{ background: #162030; color: #94a3b8; font-weight: 600; text-transform: uppercase; font-size: 11px; }}
  tr:hover {{ background: #192436; }}
  .badge {{ padding: 3px 8px; border-radius: 4px; font-weight: bold; font-size: 11px; }}
  .badge.critical {{ background: #ef444422; color: #f87171; border: 1px solid #ef4444; }}
  .badge.warning {{ background: #f59e0b22; color: #fbbf24; border: 1px solid #f59e0b; }}
  .badge.info {{ background: #06b6d422; color: #38bdf8; border: 1px solid #06b6d4; }}
  .empty-row {{ text-align: center; color: #64748b; font-style: italic; padding: 16px; }}
  .footer {{ margin-top: 36px; padding-top: 16px; border-top: 1px solid #243247; font-size: 12px; color: #64748b; display: flex; justify-content: space-between; }}
</style>
</head>
<body>
<div class="container">
  <h1>{REPORT_TITLE}</h1>
  <div class="meta-grid">
    <div class="meta-card"><div class="meta-label">Session ID</div><div class="meta-value" style="font-size:16px;">{self.session_id}</div></div>
    <div class="meta-card"><div class="meta-label">Mission Duration</div><div class="meta-value">{duration} s</div></div>
    <div class="meta-card"><div class="meta-label">Final State</div><div class="meta-value">{self.latest_state}</div></div>
    <div class="meta-card"><div class="meta-label">Total Hazards</div><div class="meta-value" style="color:{'#f87171' if len(self.hazards) > 0 else '#4ade80'}">{len(self.hazards)}</div></div>
  </div>

  <div class="meta-grid">
    <div class="meta-card"><div class="meta-label">Max CH4 (Methane)</div><div class="meta-value">{gas_ch4_max} ppm</div></div>
    <div class="meta-card"><div class="meta-label">Max CO (Carbon Monoxide)</div><div class="meta-value">{gas_co_max} ppm</div></div>
    <div class="meta-card"><div class="meta-label">Min O2 Level</div><div class="meta-value">{o2_min} %</div></div>
    <div class="meta-card"><div class="meta-label">Max Temp</div><div class="meta-value">{temp_max} °C</div></div>
  </div>

  <h2>Logged Environmental & Safety Hazards</h2>
  <table>
    <thead><tr><th>Timestamp</th><th>Hazard Type</th><th>Severity</th><th>Description</th><th>Observed Value</th><th>Coordinates</th></tr></thead>
    <tbody>{hazard_rows}</tbody>
  </table>

  <h2>Mission State Transitions</h2>
  <table>
    <thead><tr><th>Timestamp</th><th>Transition</th><th>Active Job</th><th>Frontiers</th></tr></thead>
    <tbody>{state_rows}</tbody>
  </table>

  <h2>Self-Righting & Actuator Recoveries</h2>
  <table>
    <thead><tr><th>Timestamp</th><th>Event / Action Details</th></tr></thead>
    <tbody>{recovery_rows}</tbody>
  </table>

  <div class="footer">
    <span>DrillPulse Autonomous Mine Safety Architecture</span>
    <span>Persistent SD Log: {self.sd_log_path}</span>
  </div>
</div>
</body>
</html>
"""
            with open(html_path, 'w') as f:
                f.write(html_content)
            self.get_logger().info(f"Successfully generated HTML report: {html_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write HTML report: {e}")

        # Publish notification
        status_msg = String()
        status_msg.data = f"REPORT_GENERATED:{html_path}"
        self.pub_status.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReportGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.generate_reports()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
