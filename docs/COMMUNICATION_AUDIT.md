# DRILLPULSE — COMMUNICATION ARCHITECTURE & AUDIT
**Step 4: Dual-Channel Communication, Protocol V2 Framing & Link Security**
**Date:** 2026-09-29
**Platform:** ROS 2 Jazzy, Python 3.12, Linux UDP / Serial Transport

---

## 1. Executive Summary & Communication Architecture Overview

DrillPulse implements a decoupled, fail-safe dual-channel communication architecture designed specifically for underground mine environments:

```
+-----------------------------------------------------------------------------------+
|                               ROVER ONBOARD SYSTEM                                |
|                                                                                   |
|  +-------------------------------------+   +------------------------------------+ |
|  |       Low-Bandwidth Subsystem       |   |      High-Bandwidth Subsystem      | |
|  |  (LoRa / SX1262 / 868-915 MHz)      |   |        (Wi-Fi HaLow / 802.11ah)    | |
|  |                                     |   |                                    | |
|  | - Protocol V2 Binary/JSON Framing   |   | - Optical Camera (RGB Compressed)  | |
|  | - CRC32 Checksum Validation         |   | - FLIR Lepton Thermal (Compressed) | |
|  | - Sequence & Anti-Replay Rejection  |   | - H.264 / MJPEG Multi-Cast Stream  | |
|  | - Priority Message Content Filter   |   | - Independent QoS (Best Effort)   | |
|  | - Command Lifecycle & Explicit ACKs |   | - Independent Transport Port       | |
|  +-------------------------------------+   +------------------------------------+ |
+-----------------------------------------------------------------------------------+
          |                                                   |
          |  Channel A: Sub-1 GHz LoRa                        |  Channel B: Wi-Fi HaLow
          |  MTU: 220 bytes, 0.5 - 10 Hz                      |  MTU: 1500 bytes, 15-30 FPS
          v                                                   v
+-----------------------------------------------------------------------------------+
|                             BASE STATION / OPERATOR                               |
|                                                                                   |
|  +-------------------------------------+   +------------------------------------+ |
|  |      LoRa Transceiver Node (Base)   |   |       Video Stream Receiver        | |
|  |                                     |   |                                    | |
|  | - Packet Framing & CRC32 Verify     |   | - Decoupled Image Deserializer     | |
|  | - Sequence Stale Rejection          |   | - Frame Drop / Timeout Detector    | |
|  | - Telemetry Deserializer & Pub      |   | - Telemetry Isolation Guarantee    | |
|  +-------------------------------------+   +------------------------------------+ |
|                    \                                   /                          |
|                     v                                 v                           |
|  +-----------------------------------------------------------------------------+  |
|  |                        OPERATOR WEB DASHBOARD                               |  |
|  | - Dual-Panel Mode Locking (Explored vs Unexplored)                          |  |
|  | - Zero-Fake-Data Enforcement (-- / Stale Banner)                            |  |
|  | - Interactive Canvas Click-to-Coordinate Dispatch                           |  |
|  | - Teleoperation (WASD), Recovery Triggers (6, 7), E-Stop (ESC)              |  |
|  +-----------------------------------------------------------------------------+  |
+-----------------------------------------------------------------------------------+
```

---

## 2. Low-Bandwidth LoRa Channel & Protocol V2 Framing

### 2.1 Packet Structure & Wire Format
The low-bandwidth transport operates with a 220-byte payload MTU limit to fit within standard LoRa spreading factor envelopes (e.g. SF7–SF10 at 125 kHz bandwidth).

Every Protocol V2 packet conforms to:
```
[Header: 0xAA 0x55] [Length: 1B] [Seq: 2B BE] [Timestamp: 4B BE] [Priority: 1B] [Payload: N bytes] [CRC32: 4B BE]
```

- **Header (2 bytes):** Magic bytes `0xAA 0x55` for frame synchronization and stream alignment.
- **Length (1 byte):** Payload byte count `N` (1 to 220 bytes).
- **Sequence Number (2 bytes, Big Endian):** Incremental sequence counter $[0, 65535]$ with rollover handling.
- **Timestamp (4 bytes, Big Endian):** Unix timestamp integer (seconds) when packet was emitted.
- **Priority Level (1 byte):**
  - `0x01` — `CRITICAL` (E-Stop, gas hazard alarm, rollover warning, ACK/NACK).
  - `0x02` — `HIGH` (Navigation goal, return-to-start, recovery command).
  - `0x03` — `NORMAL` (Standard state telemetry, battery, basic posture).
  - `0x04` — `LOW` (Ambient temperature, dust, relative humidity).
- **Payload (`N` bytes):** Compressed JSON UTF-8 payload.
- **CRC32 Checksum (4 bytes, Big Endian):** Standard IEEE 802.3 CRC-32 computed over `[Length + Seq + Timestamp + Priority + Payload]`.

### 2.2 Replay Protection, Stale Packet Drop & Error Detection
1. **CRC32 Integrity:** If the calculated CRC32 does not match the trailing 4 bytes, the packet is discarded immediately, incrementing `crc_error_count`.
2. **Stale Packet Window:** Packets with timestamps older than 5.0 seconds compared to the receiver's local clock are rejected to prevent replay or severely buffered stale execution.
3. **Sequence Tracking:** The receiver tracks `last_rx_seq`. If an incoming packet's sequence number is $\le$ `last_rx_seq` (accounting for 16-bit wrap-around modulo 65536) and within a 50-sequence duplicate window, it is dropped as duplicate.

---

## 3. High-Bandwidth Wi-Fi HaLow & Video Decoupling

Underground operations require thermal sensing (to locate trapped miners or hotspots) and optical RGB cameras (for structural navigation). However, video streams are inherently prone to RF multipath fading and tunnel attenuation.

### 3.1 Network Isolation Guarantees
- Video frames (`sensor_msgs/msg/CompressedImage`) are transmitted on an isolated network path (separate port / HaLow interface) and NEVER share the LoRa socket or serial buffer.
- `video_streamer_node.py` encodes optical and FLIR thermal feeds independently.
- **Zero-Coupling Invariant:** A complete loss of the video stream (0 FPS, UDP packet loss, or camera hardware crash) has **zero impact** on low-bandwidth LoRa telemetry, E-Stop propagation, or recovery triggering.
- The dashboard monitors video frame arrival independently: if no video frame arrives within 2.5 seconds, the UI renders `RGB: OFFLINE` and `THERMAL: OFFLINE` without marking rover telemetry as disconnected.

---

## 4. Port & Transport Configuration

| Channel | Interface | Default Port / Device | Max MTU | Purpose |
|---|---|---|---|---|
| **LoRa (Rover $\rightarrow$ Station)** | UDP Loopback / Serial | Rover TX: 9001 $\rightarrow$ Station RX: 9002 | 220 Bytes | Telemetry, alerts, ACKs |
| **LoRa (Station $\rightarrow$ Rover)** | UDP Loopback / Serial | Station TX: 9002 $\rightarrow$ Rover RX: 9001 | 220 Bytes | Commands, teleop, waypoints |
| **Wi-Fi HaLow (Optical)** | UDP / TCP Stream | Port 5000 / 8080 | 1500 Bytes | Optical RGB compressed feed |
| **Wi-Fi HaLow (Thermal)** | UDP / TCP Stream | Port 5001 / 8081 | 1500 Bytes | FLIR thermal grayscale feed |

*Note: For testing and simulation on Ubuntu 24.04 without physical radio peripherals, socket option `SO_REUSEADDR` is applied to permit rapid socket rebirth.*
