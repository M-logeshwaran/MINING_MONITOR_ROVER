#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Environment & Hazard Monitor Node
# ============================================================
import json
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from drillpulse_msgs.msg import EnvironmentState, HazardEvent, MissionState

class EnvironmentMonitorNode(Node):
    def __init__(self):
        super().__init__('environment_monitor_node')
        self.declare_parameter('methane_warning', 500.0)
        self.declare_parameter('methane_danger', 1000.0)
        self.declare_parameter('report_dir', '/home/loki/SIH_REPO_FINAL/reports')

        self.m_warn = float(self.get_parameter('methane_warning').value)
        self.m_danger = float(self.get_parameter('methane_danger').value)
        self.report_dir = str(self.get_parameter('report_dir').value)
        os.makedirs(self.report_dir, exist_ok=True)

        self.current_temp = 24.0
        self.current_hum = 55.0
        self.current_ch4 = 120.0
        self.current_co = 12.0
        self.rover_x = 0.0
        self.rover_y = 0.0
        self.mission_id = 'MISSION_ALPHA_01'
        self.last_hazard_time = 0.0

        self.env_pub = self.create_publisher(EnvironmentState, '/environment/state', 10)
        self.hazard_pub = self.create_publisher(HazardEvent, '/hazards/events', 10)

        self.gas_sub = self.create_subscription(Float32, '/drillpulse/gas', lambda m: setattr(self, 'current_ch4', m.data), 10)
        self.temp_sub = self.create_subscription(Float32, '/drillpulse/temperature', lambda m: setattr(self, 'current_temp', m.data), 10)
        self.hum_sub = self.create_subscription(Float32, '/drillpulse/humidity', lambda m: setattr(self, 'current_hum', m.data), 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.state_sub = self.create_subscription(MissionState, '/rover/mission_state', lambda m: setattr(self, 'mission_id', m.mission_id), 10)

        self.timer = self.create_timer(1.0, self.monitor_cycle)
        self.get_logger().info('Environment & Hazard Monitor initialized.')

    def on_odom(self, msg: Odometry):
        self.rover_x = msg.pose.pose.position.x
        self.rover_y = msg.pose.pose.position.y

    def monitor_cycle(self):
        now = self.get_clock().now()
        risk = 'NORMAL'
        alarm = False
        if self.current_ch4 >= self.m_danger or self.current_temp >= 45.0:
            risk = 'CRITICAL'
            alarm = True
        elif self.current_ch4 >= self.m_warn or self.current_co >= 50.0:
            risk = 'WARNING'

        env = EnvironmentState()
        env.header.stamp = now.to_msg()
        env.temperature = float(self.current_temp)
        env.humidity = float(self.current_hum)
        env.methane_ppm = float(self.current_ch4)
        env.carbon_monoxide_ppm = float(self.current_co)
        env.oxygen_percentage = 20.8
        env.air_quality_index = 45.0
        env.risk_level = risk
        env.gas_alarm = alarm
        env.rover_x = float(self.rover_x)
        env.rover_y = float(self.rover_y)
        self.env_pub.publish(env)

        t_now = time.time()
        if alarm and (t_now - self.last_hazard_time > 10.0):
            self.last_hazard_time = t_now
            h = HazardEvent()
            h.header.stamp = now.to_msg()
            h.event_id = f"HAZ_{int(t_now)}"
            h.event_type = "GAS_ALERT" if self.current_ch4 >= self.m_danger else "FIRE"
            h.confidence = 0.95
            h.x = float(self.rover_x)
            h.y = float(self.rover_y)
            h.description = f"Critical {h.event_type}: CH4={self.current_ch4:.1f}ppm, Temp={self.current_temp:.1f}C"
            h.severity = "CRITICAL"
            self.hazard_pub.publish(h)

def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
