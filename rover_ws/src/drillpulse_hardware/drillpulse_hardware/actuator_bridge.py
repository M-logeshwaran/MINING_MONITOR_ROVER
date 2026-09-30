#!/usr/bin/env python3
# ============================================================
# DRILLPULSE
#
# FILE:
#     actuator_bridge.py
#
# PURPOSE:
#     Interfaces with the linear actuator driver board for physical
#     self-righting / rollover recovery. Drives Left and Right
#     telescoping pistons through extension, retraction, and hold states.
#
# INPUT:
#     /rover/recovery_cmd (drillpulse_msgs/RecoveryCommand)
#
# OUTPUT:
#     Hardware actuator PWM/relay pins or serial driver
#     /rover/actuator_state (std_msgs/String)
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
DEFAULT_PORT = '/dev/ttyACM2'
DEFAULT_BAUD = 115200
DEFAULT_SIMULATION_MODE = True
MAX_STROKE_MM = 200.0
EXTENSION_SPEED_MM_S = 25.0
SAFETY_TIMEOUT_SEC = 10.0
# =========================================================

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from drillpulse_msgs.msg import RecoveryCommand

try:
    import serial
except ImportError:
    serial = None


class ActuatorBridge(Node):
    """Low-level driver for Left and Right self-righting linear actuators."""

    def __init__(self):
        super().__init__('actuator_bridge')

        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud', DEFAULT_BAUD)
        self.declare_parameter('simulation_mode', DEFAULT_SIMULATION_MODE)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.simulation_mode = bool(self.get_parameter('simulation_mode').value)

        self.cmd_sub = self.create_subscription(
            RecoveryCommand,
            '/rover/recovery_cmd',
            self.recovery_callback,
            10
        )
        self.state_pub = self.create_publisher(String, '/rover/actuator_state', 10)

        self.current_state = 'IDLE'
        self.left_extension = 0.0
        self.right_extension = 0.0
        self.serial_conn = None

        if not self.simulation_mode:
            if serial is not None:
                try:
                    self.serial_conn = serial.Serial(self.port, self.baud, timeout=0.05)
                    self.get_logger().info(f'Actuator hardware connected on {self.port}')
                except Exception as e:
                    self.get_logger().warn(f'Could not open {self.port}: {e}. Simulation enabled.')
                    self.simulation_mode = True

        self.timer = self.create_timer(0.1, self.publish_state)
        self.get_logger().info(f'Actuator Bridge active (Simulation: {self.simulation_mode})')

    def recovery_callback(self, msg: RecoveryCommand):
        cmd = msg.command_type.upper()
        self.get_logger().info(f'Received Actuator Recovery Command: {cmd} (target ext: {msg.actuator_extension})')

        if cmd == 'LEFT_RECOVERY':
            self.current_state = 'EXTENDING_LEFT'
            self._send_hardware_cmd('L_EXT')
        elif cmd == 'RIGHT_RECOVERY':
            self.current_state = 'EXTENDING_RIGHT'
            self._send_hardware_cmd('R_EXT')
        elif cmd == 'STOP_RECOVERY':
            self.current_state = 'STOPPED'
            self._send_hardware_cmd('STOP_ALL')
        elif cmd == 'RESET':
            self.current_state = 'RETRACTING'
            self._send_hardware_cmd('RETRACT_ALL')

    def _send_hardware_cmd(self, code: str):
        if self.serial_conn and not self.simulation_mode:
            try:
                self.serial_conn.write(f'{code}\n'.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f'Actuator serial write failed: {e}')

    def publish_state(self):
        msg = String()
        msg.data = f'STATE:{self.current_state},LEFT:{self.left_extension:.1f},RIGHT:{self.right_extension:.1f}'
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
