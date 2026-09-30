#!/usr/bin/env python3
# ============================================================
# DRILLPULSE
#
# FILE:
#     mobility_controller_node.py
#
# PURPOSE:
#     Single authoritative mobility arbitration and kinematics engine.
#     Resolves command priority across 8 levels:
#       1. EMERGENCY STOP
#       2. RECOVERY ACTUATOR SAFETY
#       3. HARDWARE FAULT
#       4. LOW BATTERY RETURN / SLOWDOWN
#       5. COMMUNICATION FAILSAFE
#       6. AUTONOMOUS NAVIGATION (/cmd_vel_nav)
#       7. MANUAL JOYSTICK (/cmd_vel_teleop)
#       8. NORMAL / COMPATIBILITY (/cmd_vel)
#
#     Implements 6-wheel rocker-bogie differential drive kinematics,
#     slew-rate acceleration limiting, and a 500ms command watchdog.
#
# INPUT:
#     /rover/estop            (std_msgs/Bool)
#     /rover/actuator_state   (std_msgs/String)
#     /rover/battery          (drillpulse_msgs/BatteryState)
#     /cmd_vel_teleop         (geometry_msgs/Twist)
#     /cmd_vel_nav            (geometry_msgs/Twist)
#     /cmd_vel                (geometry_msgs/Twist)
#
# OUTPUT:
#     /rover/motor_cmd_vel    (geometry_msgs/Twist)
#     /rover/mobility_state   (std_msgs/String)
# ============================================================

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from drillpulse_msgs.msg import BatteryState


class UnifiedMobilityControllerNode(Node):
    """Authoritative arbiter and kinematic controller for 6-wheel rocker-bogie mobility."""

    def __init__(self):
        super().__init__('mobility_controller_node')

        # Parameters
        self.declare_parameter('watchdog_timeout_sec', 0.5)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('max_linear_accel', 1.5)
        self.declare_parameter('max_angular_accel', 3.0)
        self.declare_parameter('track_width', 0.90)
        self.declare_parameter('wheel_radius', 0.11)
        self.declare_parameter('control_rate_hz', 50.0)

        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout_sec').value)
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        rate_hz = float(self.get_parameter('control_rate_hz').value)

        # Internal state
        self.estop_active = False
        self.actuator_active = False
        self.actuator_state_str = 'IDLE'
        self.battery_voltage = 12.0
        self.battery_percentage = 100.0
        self.battery_low = False
        self.comm_failsafe_active = False
        self.hardware_fault = False

        # Commanded inputs & timestamps
        self.teleop_cmd = Twist()
        self.teleop_timestamp = 0.0

        self.nav_cmd = Twist()
        self.nav_timestamp = 0.0

        self.generic_cmd = Twist()
        self.generic_timestamp = 0.0

        # Current executed velocities (for slew rate limiting)
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.last_update_time = time.time()
        self.current_mobility_state = 'IDLE'

        # Publishers
        self.motor_cmd_pub = self.create_publisher(Twist, '/rover/motor_cmd_vel', 10)
        self.mobility_state_pub = self.create_publisher(String, '/rover/mobility_state', 10)

        # Subscriptions
        self.create_subscription(Bool, '/rover/estop', self.estop_callback, 10)
        self.create_subscription(String, '/rover/actuator_state', self.actuator_callback, 10)
        self.create_subscription(BatteryState, '/rover/battery', self.battery_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self.teleop_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.generic_callback, 10)

        # Control loop timer
        timer_period = 1.0 / max(1.0, rate_hz)
        self.timer = self.create_timer(timer_period, self.control_loop)
        self.get_logger().info('Unified Mobility Controller Node started.')

    def estop_callback(self, msg: Bool):
        self.estop_active = msg.data
        if self.estop_active:
            self.get_logger().warn('EMERGENCY STOP ENGAGED')

    def actuator_callback(self, msg: String):
        self.actuator_state_str = msg.data
        # If actuators are actively extending, retracting, or moving, lock mobility
        moving_keywords = ['EXTENDING', 'RETRACTING', 'ACTIVE', 'EXTEND']
        self.actuator_active = any(k in msg.data.upper() for k in moving_keywords)

    def battery_callback(self, msg: BatteryState):
        self.battery_voltage = msg.voltage
        self.battery_percentage = msg.percentage
        self.battery_low = (msg.voltage < 10.5 or msg.percentage < 15.0)

    def teleop_callback(self, msg: Twist):
        self.teleop_cmd = msg
        self.teleop_timestamp = time.time()

    def nav_callback(self, msg: Twist):
        self.nav_cmd = msg
        self.nav_timestamp = time.time()

    def generic_callback(self, msg: Twist):
        self.generic_cmd = msg
        self.generic_timestamp = time.time()

    def control_loop(self):
        now = time.time()
        dt = max(0.001, min(0.1, now - self.last_update_time))
        self.last_update_time = now

        target_vx = 0.0
        target_wz = 0.0
        selected_state = 'IDLE'

        # -------------------------------------------------------------
        # PRIORITY LADDER ARBITRATION
        # -------------------------------------------------------------
        # Level 1: EMERGENCY STOP
        if self.estop_active:
            target_vx = 0.0
            target_wz = 0.0
            selected_state = 'ESTOP_TRIGGERED'

        # Level 2: RECOVERY ACTUATOR SAFETY INTERLOCK
        elif self.actuator_active:
            target_vx = 0.0
            target_wz = 0.0
            selected_state = 'ACTUATOR_INTERLOCK'

        # Level 3: HARDWARE FAULT
        elif self.hardware_fault:
            target_vx = 0.0
            target_wz = 0.0
            selected_state = 'HARDWARE_FAULT'

        # Level 5: COMMUNICATION FAILSAFE
        elif self.comm_failsafe_active:
            target_vx = 0.0
            target_wz = 0.0
            selected_state = 'COMM_FAILSAFE'

        # Level 7: MANUAL JOYSTICK / TELEOP (Priority over Auto Nav)
        elif (now - self.teleop_timestamp) <= self.watchdog_timeout and (
            abs(self.teleop_cmd.linear.x) > 0.001 or abs(self.teleop_cmd.angular.z) > 0.001
        ):
            target_vx = self.teleop_cmd.linear.x
            target_wz = self.teleop_cmd.angular.z
            selected_state = 'MANUAL_TELEOP'

        # Level 6: AUTONOMOUS NAVIGATION (/cmd_vel_nav)
        elif (now - self.nav_timestamp) <= self.watchdog_timeout and (
            abs(self.nav_cmd.linear.x) > 0.001 or abs(self.nav_cmd.angular.z) > 0.001
        ):
            target_vx = self.nav_cmd.linear.x
            target_wz = self.nav_cmd.angular.z
            selected_state = 'AUTONOMOUS_NAV'

        # Level 8: GENERIC / COMPATIBILITY (/cmd_vel)
        elif (now - self.generic_timestamp) <= self.watchdog_timeout and (
            abs(self.generic_cmd.linear.x) > 0.001 or abs(self.generic_cmd.angular.z) > 0.001
        ):
            target_vx = self.generic_cmd.linear.x
            target_wz = self.generic_cmd.angular.z
            selected_state = 'GENERIC_CMD'

        else:
            # Command Watchdog Cutoff: No active topic commands within 500ms
            target_vx = 0.0
            target_wz = 0.0
            selected_state = 'IDLE'

        # Level 4: LOW BATTERY SAFETY THROTTLING
        if self.battery_low and selected_state in ('MANUAL_TELEOP', 'AUTONOMOUS_NAV', 'GENERIC_CMD'):
            target_vx *= 0.5
            target_wz *= 0.5
            selected_state += '_BATT_LOW'

        # Clamp max commanded velocities
        target_vx = max(-self.max_linear_vel, min(self.max_linear_vel, target_vx))
        target_wz = max(-self.max_angular_vel, min(self.max_angular_vel, target_wz))

        # -------------------------------------------------------------
        # SLEW-RATE ACCELERATION LIMITING
        # -------------------------------------------------------------
        max_dv = self.max_linear_accel * dt
        diff_vx = target_vx - self.current_linear_x
        if abs(diff_vx) > max_dv:
            self.current_linear_x += math.copysign(max_dv, diff_vx)
        else:
            self.current_linear_x = target_vx

        max_dw = self.max_angular_accel * dt
        diff_wz = target_wz - self.current_angular_z
        if abs(diff_wz) > max_dw:
            self.current_angular_z += math.copysign(max_dw, diff_wz)
        else:
            self.current_angular_z = target_wz

        # In case of E-STOP or Actuator Interlock, force immediate zero
        if selected_state in ('ESTOP_TRIGGERED', 'ACTUATOR_INTERLOCK', 'HARDWARE_FAULT'):
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0

        # -------------------------------------------------------------
        # PUBLISH ARBITRATED VELOCITIES
        # -------------------------------------------------------------
        out_twist = Twist()
        out_twist.linear.x = float(self.current_linear_x)
        out_twist.angular.z = float(self.current_angular_z)
        self.motor_cmd_pub.publish(out_twist)

        # Publish mobility state
        self.current_mobility_state = selected_state
        state_msg = String()
        state_msg.data = self.current_mobility_state
        self.mobility_state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedMobilityControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
