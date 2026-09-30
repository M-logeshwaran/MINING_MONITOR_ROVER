#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Safety Monitor Node
# ============================================================
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from drillpulse_msgs.msg import LinkStatus, OdometryDiagnostics

class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')
        self.declare_parameter('lora_timeout', 5.0)
        self.declare_parameter('rollover_threshold_deg', 45.0)
        self.declare_parameter('link_loss_policy', 'STOP')

        self.lora_timeout = float(self.get_parameter('lora_timeout').value)
        self.rollover_deg = float(self.get_parameter('rollover_threshold_deg').value)
        self.link_loss_policy = str(self.get_parameter('link_loss_policy').value).upper()

        self.current_state = 'NORMAL'
        self.last_lora_time = time.time()
        self.latest_imu = None
        self.latest_diagnostics = None
        self.emergency_stop_latched = False

        self.state_pub = self.create_publisher(String, '/rover/safety_state', 10)
        self.estop_pub = self.create_publisher(Bool, '/rover/emergency_stop', 10)

        self.imu_sub = self.create_subscription(Imu, '/sparkfun/imu/data', self.imu_callback, 10)
        self.link_sub = self.create_subscription(LinkStatus, '/transport/lora/link_status', self.link_callback, 10)
        self.diag_sub = self.create_subscription(OdometryDiagnostics, '/odom/diagnostics', self.diag_callback, 10)
        self.estop_sub = self.create_subscription(Bool, '/rover/set_emergency_stop', self.estop_cmd_callback, 10)

        self.timer = self.create_timer(0.1, self.evaluate_safety)
        self.get_logger().info('Safety Monitor initialized.')

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg

    def link_callback(self, msg: LinkStatus):
        if msg.connected:
            self.last_lora_time = time.time()

    def diag_callback(self, msg: OdometryDiagnostics):
        self.latest_diagnostics = msg

    def estop_cmd_callback(self, msg: Bool):
        self.emergency_stop_latched = msg.data

    def evaluate_safety(self):
        now = time.time()
        prev = self.current_state

        if self.emergency_stop_latched:
            self.current_state = 'EMERGENCY_STOP'
        elif self._check_rollover():
            self.current_state = 'ROLLOVER'
        elif self.latest_diagnostics and self.latest_diagnostics.fusion_state in ('SENSOR_TIMEOUT', 'SENSOR_DISAGREEMENT'):
            self.current_state = 'SENSOR_FAULT'
        elif (now - self.last_lora_time) > self.lora_timeout:
            self.current_state = 'RETURN_HOME' if self.link_loss_policy == 'RETURN_HOME' else 'DEGRADED_LINK'
        else:
            self.current_state = 'NORMAL'

        s_msg = String()
        s_msg.data = self.current_state
        self.state_pub.publish(s_msg)

        b_msg = Bool()
        b_msg.data = (self.current_state in ('EMERGENCY_STOP', 'ROLLOVER', 'SENSOR_FAULT'))
        self.estop_pub.publish(b_msg)

    def _check_rollover(self):
        if self.latest_imu is None:
            return False
        ax = self.latest_imu.linear_acceleration.x
        ay = self.latest_imu.linear_acceleration.y
        az = self.latest_imu.linear_acceleration.z
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 0.1:
            return False
        roll = math.atan2(ay, az) * 180.0 / math.pi
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi
        return abs(roll) > self.rollover_deg or abs(pitch) > self.rollover_deg

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
