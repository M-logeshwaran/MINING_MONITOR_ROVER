# DRILLPULSE — LoRa Mesh Protocol & Binary Packet Framing

## 1. Physical Layer & Framing
- Frequency: 915 MHz (Semtech SX1262 LoRa Transceiver).
- Serial Port: /dev/ttyUSB3 at 115200 baud.
- Maximum Payload: 220 bytes.
- Fallback: Local UDP loopback sockets (14550/14551) tagged [TRANSPORT: UDP_SIMULATION].

## 2. Binary Packet Structure
[Sync: 0xAA 0x55 (2B)] [Type (1B)] [Sequence (2B)] [Length (1B)] [Payload (0-210B)] [CRC32 (4B)]
CRC32 covers Type, Sequence, Length, and Payload. Corrupt frames are dropped.
