# DRILLPULSE — Video Transport Abstraction (WiFi / HaLow)

## 1. Bandwidth Domain Separation
- Telemetry & Commands: 915 MHz LoRa Mesh (~9.6 kbps, high penetration).
- Video Transport: 2.4/5GHz WiFi or 900MHz WiFi-HaLow (1.5-15 Mbps, port 8081).

## 2. Streamer Node Architecture (drillpulse_camera)
- Stream endpoint: http://<rover-ip>:8081/video_feed.
- Real Camera: V4L2 capture on /dev/video0 or /dev/video2 with HUD telemetry overlay.
- Simulation Fallback: 640x480 test pattern with prominent watermark:
  [SIMULATION VIDEO STREAM — HARDWARE BENCH NOT DETECTED].
