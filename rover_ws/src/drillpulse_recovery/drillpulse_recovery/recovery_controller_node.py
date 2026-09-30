#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Recovery Controller Node
# ============================================================
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from drillpulse_msgs.msg import RecoveryCommand

class RecoveryControllerNode(Node):
    def __init__(self):
        super().__init__('recovery_controller_node')
        self.declare_parameter('rollover_threshold_deg', 50.0)
        self.rollover_deg = float(self.get_parameter('rollover_threshold_deg').value)

        self.state = 'IDLE'
        self.active_side = None
        self.state_start_time = 0.0
        self.current_roll = 0.0

        self.status_pub = self.create_publisher(String, '/rover/recovery_status', 10)
        self.actuator_pub = self.create_publisher(RecoveryCommand, '/rover/actuator_cmd', 10)

        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.imu_compat = self.create_subscription(Imu, '/sparkfun/imu/data', self.imu_callback, 10)
        self.cmd_sub = self.create_subscription(RecoveryCommand, '/rover/recovery_cmd', self.command_callback, 10)
        self.cmd_sub_long = self.create_subscription(RecoveryCommand, '/rover/recovery_command', self.command_callback, 10)

        self.timer = self.create_timer(0.05, self.fsm_cycle)
        self.get_logger().info('Recovery Controller Node initialized.')

    def imu_callback(self, msg: Imu):
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        norm = math.sqrt(ay * ay + az * az)
        if norm > 0.1:
            self.current_roll = math.atan2(ay, az) * 180.0 / math.pi

    def command_callback(self, msg: RecoveryCommand):
        cmd = msg.command_type.upper()
        if cmd in ('LEFT_RECOVERY', 'KEY_6'):
            self._start('LEFT')
        elif cmd in ('RIGHT_RECOVERY', 'KEY_7'):
            self._start('RIGHT')
        elif cmd in ('STOP_RECOVERY', 'ABORT'):
            self.state = 'COOLDOWN'
            self.state_start_time = time.time()
            self.active_side = None
            cmd_reset = RecoveryCommand()
            cmd_reset.header.stamp = self.get_clock().now().to_msg()
            cmd_reset.command_type = 'RESET'
            self.actuator_pub.publish(cmd_reset)

    def _start(self, side: str):
        if self.state == 'IDLE':
            self.active_side = side
            self.state = 'EXTENDING'
            self.state_start_time = time.time()
            self.get_logger().info(f'Starting {side} recovery sequence.')
            cmd = RecoveryCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.command_type = f'{self.active_side}_RECOVERY'
            self.actuator_pub.publish(cmd)

    def fsm_cycle(self):
        now = time.time()
        elapsed = now - self.state_start_time

        if self.state == 'IDLE':
            if abs(self.current_roll) > self.rollover_deg:
                self._start('LEFT' if self.current_roll < 0 else 'RIGHT')
        elif self.state == 'SETTLE':
            if elapsed >= 0.5:
                self.state = 'EXTENDING'
                self.state_start_time = now
                cmd = RecoveryCommand()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.command_type = f'{self.active_side}_RECOVERY'
                self.actuator_pub.publish(cmd)
        elif self.state == 'EXTENDING':
            if abs(self.current_roll) <= 15.0 or elapsed >= 3.0:
                self.state = 'RETRACTING'
                self.state_start_time = now
                cmd = RecoveryCommand()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.command_type = 'RESET'
                self.actuator_pub.publish(cmd)
        elif self.state == 'RETRACTING':
            if elapsed >= 2.0:
                self.state = 'IDLE'
                self.active_side = None
        elif self.state == 'COOLDOWN':
            if elapsed >= 1.0:
                self.state = 'IDLE'
                self.active_side = None

        msg = String()
        msg.data = f'STATE:{self.state},SIDE:{self.active_side},ROLL:{self.current_roll:.1f}'
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RecoveryControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
