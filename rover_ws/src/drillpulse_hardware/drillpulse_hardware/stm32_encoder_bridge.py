#!/usr/bin/env python3
# ============================================================
# DRILLPULSE
#
# FILE:
#     stm32_encoder_bridge.py
#
# PURPOSE:
#     Interfaces with the onboard STM32 microcontroller to acquire
#     wheel encoder feedback for all 6 wheels. Signs the encoder
#     magnitudes based on active commanded motion.
#
# INPUT:
#     STM32 Serial stream (@ /dev/ttyACM1 or configured port)
#     /rover/command_rpm or /cmd_vel sign reference
#
# OUTPUT:
#     /encoder (std_msgs/Float32MultiArray)
#     Contract: [timestamp_ms, LF_RPM, RF_RPM, LM_RPM, RM_RPM, LB_RPM, RB_RPM]
#
# TF OWNER:
#     None
#
# USER PARAMETERS:
#     See USER CONFIGURATION section below.
# ============================================================

# =========================================================
# USER CONFIGURATION
# =========================================================
DEFAULT_ROVER_PORT = '/dev/ttyACM1'
DEFAULT_ROVER_BAUD = 115200
DEFAULT_SIMULATION_MODE = False
DEFAULT_PUBLISH_RATE_HZ = 50.0

# 6-wheel geometry references (meters)
WHEEL_DIAMETER = 0.22
TRACK_WIDTH = 0.90
# =========================================================

import math
import re
import sys
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist

try:
    import serial
except ImportError:
    serial = None


class STM32EncoderBridge(Node):
    """Bridges STM32 6-wheel encoder telemetry into ROS 2."""

    def __init__(self):
        super().__init__('stm32_encoder_bridge')

        self.declare_parameter('rover_port', DEFAULT_ROVER_PORT)
        self.declare_parameter('rover_baud', DEFAULT_ROVER_BAUD)
        self.declare_parameter('simulation_mode', DEFAULT_SIMULATION_MODE)
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE_HZ)

        self.rover_port = str(self.get_parameter('rover_port').value)
        self.rover_baud = int(self.get_parameter('rover_baud').value)
        self.simulation_mode = bool(self.get_parameter('simulation_mode').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.encoder_pub = self.create_publisher(Float32MultiArray, '/encoder', 20)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Commanded velocity tracking to sign raw unsigned encoder counts
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.cmd_lock = threading.Lock()

        # Serial connection
        self.serial_conn = None
        self.running = True

        if not self.simulation_mode:
            if serial is None:
                self.get_logger().error('pyserial not available. Enabling simulation mode.')
                self.simulation_mode = True
            else:
                try:
                    self.serial_conn = serial.Serial(
                        self.rover_port,
                        self.rover_baud,
                        timeout=0.05
                    )
                    self.get_logger().info(f'Connected to STM32 encoder bridge: {self.rover_port} @ {self.rover_baud}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to open STM32 serial {self.rover_port}: {e}. Enabling simulation.')
                    self.simulation_mode = True

        self.timer_period = 1.0 / max(1.0, publish_rate)
        self.timer = self.create_timer(self.timer_period, self.update_encoders)
        self.get_logger().info(f'STM32 Encoder Bridge ready (Simulation: {self.simulation_mode})')

    def cmd_vel_callback(self, msg: Twist):
        with self.cmd_lock:
            self.cmd_linear_x = msg.linear.x
            self.cmd_angular_z = msg.angular.z

    def update_encoders(self):
        if self.simulation_mode or self.serial_conn is None:
            self._publish_simulated_encoder()
        else:
            self._read_hardware_serial()

    def _read_hardware_serial(self):
        try:
            while self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # Expected formats:
                # 1) "ENC,ts,lf,rf,lm,rm,lb,rb"
                # 2) "E,lf,rf,lm,rm,lb,rb"
                parts = line.split(',')
                if parts[0] in ('ENC', 'E') and len(parts) >= 7:
                    try:
                        ts = float(parts[1]) if parts[0] == 'ENC' and len(parts) == 8 else time.time() * 1000.0
                        vals = [float(x) for x in parts[-6:]]
                        self._publish_signed_encoder(ts, vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])
                    except ValueError:
                        continue
        except Exception as e:
            self.get_logger().warn(f'STM32 read error: {e}')

    def _publish_simulated_encoder(self):
        with self.cmd_lock:
            vx = self.cmd_linear_x
            wz = self.cmd_angular_z

        # Differential drive kinematic prediction:
        # v_left = vx - (wz * track_width / 2)
        # v_right = vx + (wz * track_width / 2)
        # RPM = (v / (pi * diameter)) * 60
        wheel_circ = math.pi * WHEEL_DIAMETER
        v_l = vx - (wz * TRACK_WIDTH * 0.5)
        v_r = vx + (wz * TRACK_WIDTH * 0.5)

        rpm_l = (v_l / wheel_circ) * 60.0
        rpm_r = (v_r / wheel_circ) * 60.0

        ts = time.time() * 1000.0
        self._publish_signed_encoder(ts, rpm_l, rpm_r, rpm_l, rpm_r, rpm_l, rpm_r)

    def _publish_signed_encoder(self, ts_ms, lf, rf, lm, rm, lb, rb):
        with self.cmd_lock:
            vx = self.cmd_linear_x
            wz = self.cmd_angular_z

        # If incoming hardware reports absolute magnitude, apply signs from motion command
        if lf >= 0.0 and rf >= 0.0 and lm >= 0.0 and rm >= 0.0 and lb >= 0.0 and rb >= 0.0:
            if vx < -0.01:
                lf, lm, lb = -lf, -lm, -lb
                rf, rm, rb = -rf, -rm, -rb
            elif wz > 0.05:  # Turn left: left backward, right forward
                lf, lm, lb = -abs(lf), -abs(lm), -abs(lb)
                rf, rm, rb = abs(rf), abs(rm), abs(rb)
            elif wz < -0.05: # Turn right: left forward, right backward
                lf, lm, lb = abs(lf), abs(lm), abs(lb)
                rf, rm, rb = -abs(rf), -abs(rm), -abs(rb)

        msg = Float32MultiArray()
        msg.data = [
            float(ts_ms),
            float(lf), float(rf),
            float(lm), float(rm),
            float(lb), float(rb)
        ]
        self.encoder_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = STM32EncoderBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
