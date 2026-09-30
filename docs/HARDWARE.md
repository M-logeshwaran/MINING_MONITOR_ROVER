# DRILLPULSE — Hardware Integration & Pinout Specification

## 1. Electrical & Actuation Topology
- Power: 4S LiFePO4 battery pack (14.8V 10000mAh) with dual buck converters (5V 10A, 12V 20A).
- Drive: 6x 12V DC high-torque gear motors driven via Cytron MD30C motor drivers.
- Sensors: Slamtec RPLiDAR A2M8 (/dev/ttyUSB1), SparkFun ICM-20948 IMU (/dev/ttyUSB2), Waveshare SX1262 LoRa (/dev/ttyUSB3).

## 2. Recovery Push-Rod Actuators (Left & Right)
- Actuators: Heavy-duty 12V linear jack-screws (stroke 150mm, thrust 750N).
- Control: ESP32 GPIO 25 (Left), GPIO 26 (Right).
- Interlocks: Max 3.0s runtime, 1.0s thermal cooldown, auto cutoff when roll <= 15 deg.
