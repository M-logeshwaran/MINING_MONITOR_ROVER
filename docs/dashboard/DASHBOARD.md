# DRILLPULSE — Dual-Panel Dashboard Specification

## 1. Web Architecture
- Host Port: 8080 (http://localhost:8080).
- Node: drillpulse_dashboard/dashboard_node.py.
- WebSocket server on /ws for 20 Hz low-latency telemetry streaming.

## 2. Zero-Fake-Data Policy & Disconnected Behavior
- NO FAKE DATA: Disconnected states display "--" or "NO DATA". Never mock 100% battery or 0 ppm gas.
- Explored vs Unexplored panels are mutually exclusive.
- Panel activation requires rover ACK (job_ack: JOB_ACCEPTED).
- Keyboard controls: W/A/S/D teleop, Spacebar E-Stop, Key 6 Left Recovery, Key 7 Right Recovery, ESC Stop Recovery.
