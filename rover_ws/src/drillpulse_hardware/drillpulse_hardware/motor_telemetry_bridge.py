#!/usr/bin/env python3
# ============================================================
# DRILLPULSE
#
# FILE:
#     motor_telemetry_bridge.py
#
# PURPOSE:
#     Translates high-level /cmd_vel into the DrillPulse motor
#     microcontroller protocol ('CMD,X,Y,SPEED\n'). Receives environmental
#     telemetry strings ('S,TEMP,HUMIDITY,MQ4_ANALOG,...') and publishes
#     structured ROS 2 telemetry topics.
#
# INPUT:
#     /cmd_vel (geometry_msgs/Twist)
#     Motor controller serial (@ /dev/rfcomm0 or USB @ /dev/ttyUSB1)
#
# OUTPUT:
#     /drillpulse/temperature (std_msgs/Float32)
#     /drillpulse/humidity (std_msgs/Float32)
#     /drillpulse/gas (std_msgs/Float32)
#     /drillpulse/gas_status (std_msgs/String)
#     /drillpulse/left_distance (std_msgs/Float32)
#     /drillpulse/right_distance (std_msgs/Float32)
#     /drillpulse/rover_telemetry (drillpulse_msgs/RoverTelemetry)
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
DEFAULT_PORT = '/dev/rfcomm0'
DEFAULT_BAUD = 9600
DEFAULT_SIMULATION_MODE = False
MAX_LINEAR_SPEED = 0.50     # m/s
MAX_ANGULAR_SPEED = 1.00    # rad/s
GAS_ALERT_THRESHOLD_PPM = 1000.0
# =========================================================

import re
import sys
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from drillpulse_msgs.msg import RoverTelemetry

try:
    import serial
except ImportError:
    serial = None


class MotorTelemetryBridge(Node):
    """Bridges /cmd_vel to motor hardware and telemetry back to ROS 2."""

    def __init__(self):
        super().__init__('motor_telemetry_bridge')

        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud', DEFAULT_BAUD)
        self.declare_parameter('simulation_mode', DEFAULT_SIMULATION_MODE)
        self.declare_parameter('max_linear_speed', MAX_LINEAR_SPEED)
        self.declare_parameter('max_angular_speed', MAX_ANGULAR_SPEED)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.simulation_mode = bool(self.get_parameter('simulation_mode').value)
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)

        # Publishers
        self.temp_pub = self.create_publisher(Float32, '/drillpulse/temperature', 10)
        self.hum_pub = self.create_publisher(Float32, '/drillpulse/humidity', 10)
        self.gas_pub = self.create_publisher(Float32, '/drillpulse/gas', 10)
        self.gas_status_pub = self.create_publisher(String, '/drillpulse/gas_status', 10)
        self.left_dist_pub = self.create_publisher(Float32, '/drillpulse/left_distance', 10)
        self.right_dist_pub = self.create_publisher(Float32, '/drillpulse/right_distance', 10)
        self.telem_pub = self.create_publisher(RoverTelemetry, '/drillpulse/rover_telemetry', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.current_temp = 24.5
        self.current_hum = 55.0
        self.current_gas = 120.0
        self.current_ldist = 150.0
        self.current_rdist = 150.0
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0

        self.serial_conn = None
        if not self.simulation_mode:
            if serial is None:
                self.simulation_mode = True
            else:
                try:
                    self.serial_conn = serial.Serial(self.port, self.baud, timeout=0.05)
                    self.get_logger().info(f'Connected to Motor Bridge on {self.port} @ {self.baud}')
                except Exception as e:
                    self.get_logger().warn(f'Could not open {self.port}: {e}. Enabling simulation fallback.')
                    self.simulation_mode = True

        self.timer = self.create_timer(0.05, self.update_loop)
        self.get_logger().info(f'Motor Telemetry Bridge active (Simulation: {self.simulation_mode})')

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z

        # Format DrillPulse Arduino protocol: CMD,X,Y,SPEED\n
        x = int(max(-100, min(100, (msg.angular.z / self.max_ang) * 100.0)))
        y = int(max(-100, min(100, (msg.linear.x / self.max_lin) * 100.0)))
        speed = max(abs(x), abs(y))

        cmd_str = f"CMD,{x},{y},{speed}\n"
        if self.serial_conn and not self.simulation_mode:
            try:
                self.serial_conn.write(cmd_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f'Failed to send motor command: {e}')

    def update_loop(self):
        if self.simulation_mode or self.serial_conn is None:
            self._update_simulated_telemetry()
        else:
            self._read_hardware_telemetry()

        self._publish_telemetry_msgs()

    def _read_hardware_telemetry(self):
        try:
            while self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # Expected: S,TEMP,HUMIDITY,MQ4_ANALOG,MQ4_DIGITAL,LEFT_DISTANCE,RIGHT_DISTANCE
                parts = line.split(',')
                if parts[0] == 'S' and len(parts) >= 7:
                    try:
                        self.current_temp = float(parts[1])
                        self.current_hum = float(parts[2])
                        self.current_gas = float(parts[3])
                        self.current_ldist = float(parts[5])
                        self.current_rdist = float(parts[6])
                    except ValueError:
                        continue
        except Exception as e:
            self.get_logger().warn(f'Telemetry read error: {e}')

    def _update_simulated_telemetry(self):
        # Baseline environmental simulation with nominal variation
        self.current_temp = 25.2
        self.current_hum = 58.5
        self.current_gas = 135.0
        self.current_ldist = 180.0
        self.current_rdist = 180.0

    def _publish_telemetry_msgs(self):
        t_msg = Float32()
        t_msg.data = float(self.current_temp)
        self.temp_pub.publish(t_msg)

        h_msg = Float32()
        h_msg.data = float(self.current_hum)
        self.hum_pub.publish(h_msg)

        g_msg = Float32()
        g_msg.data = float(self.current_gas)
        self.gas_pub.publish(g_msg)

        gas_status = "CRITICAL" if self.current_gas >= GAS_ALERT_THRESHOLD_PPM else "NORMAL"
        s_msg = String()
        s_msg.data = gas_status
        self.gas_status_pub.publish(s_msg)

        ld_msg = Float32()
        ld_msg.data = float(self.current_ldist)
        self.left_dist_pub.publish(ld_msg)

        rd_msg = Float32()
        rd_msg.data = float(self.current_rdist)
        self.right_dist_pub.publish(rd_msg)

        # Unified telemetry packet
        telem = RoverTelemetry()
        telem.header.stamp = self.get_clock().now().to_msg()
        telem.header.frame_id = 'base_link'
        telem.battery_percentage = 94.0
        telem.battery_voltage = 25.2
        telem.current_draw = 3.5
        telem.temperature = float(self.current_temp)
        telem.humidity = float(self.current_hum)
        telem.gas_ppm = float(self.current_gas)
        telem.gas_status = gas_status
        telem.linear_velocity = float(self.cmd_vx)
        telem.angular_velocity = float(self.cmd_wz)
        telem.current_state = "OPERATIONAL"
        self.telem_pub.publish(telem)


def main(args=None):
    rclpy.init(args=args)
    node = MotorTelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
