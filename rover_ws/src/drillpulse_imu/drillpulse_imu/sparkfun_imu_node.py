#!/usr/bin/env python3
# ============================================================
# DRILLPULSE
#
# FILE:
#     sparkfun_imu_node.py
#
# PURPOSE:
#     Acquires 9-DOF data from SparkFun ICM-20948 IMU over serial.
#     Publishes calibrated angular velocity, linear acceleration,
#     and orientation estimates.
#
# INPUT:
#     Hardware Serial stream (ICM-20948 @ /dev/ttyACM0)
#
# OUTPUT:
#     /sparkfun/imu/data (sensor_msgs/Imu)
#
# TF OWNER:
#     None (Frame: sparkfun_imu_link defined by static transform)
#
# USER PARAMETERS:
#     See USER CONFIGURATION section below.
# ============================================================

# =========================================================
# USER CONFIGURATION
# =========================================================
DEFAULT_SERIAL_PORT = '/dev/ttyACM0'
DEFAULT_SERIAL_BAUD = 115200
DEFAULT_FRAME_ID = 'sparkfun_imu_link'
DEFAULT_SIMULATION_MODE = False
SAMPLE_RATE_HZ = 50.0

# Stationary Gyroscope Biases (rad/s) from 60s empirical calibration
GYRO_BIAS_X = -0.00310054
GYRO_BIAS_Y = 0.00864302
GYRO_BIAS_Z = -0.00387895

# Empirical stationary variances
GYRO_COV_X = 0.00030832
GYRO_COV_Y = 0.00055044
GYRO_COV_Z = 0.00067196
ACCEL_COV_X = 0.1
ACCEL_COV_Y = 0.1
ACCEL_COV_Z = 0.1
# =========================================================

import math
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import serial
except ImportError:
    serial = None


class SparkFunIMUNode(Node):
    """Acquires and publishes ICM-20948 IMU data with bias correction."""

    def __init__(self):
        super().__init__('sparkfun_imu_node')

        # Declare ROS Parameters
        self.declare_parameter('port', DEFAULT_SERIAL_PORT)
        self.declare_parameter('baudrate', DEFAULT_SERIAL_BAUD)
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)
        self.declare_parameter('simulation_mode', DEFAULT_SIMULATION_MODE)
        self.declare_parameter('sample_rate', SAMPLE_RATE_HZ)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.simulation_mode = bool(self.get_parameter('simulation_mode').value)
        sample_rate = float(self.get_parameter('sample_rate').value)

        self.serial_port = None
        self.sim_yaw = 0.0
        self.last_sim_time = time.time()

        # Connect to Hardware Serial
        if not self.simulation_mode:
            if serial is None:
                self.get_logger().error('pyserial module not installed. Falling back to simulation mode.')
                self.simulation_mode = True
            else:
                try:
                    self.serial_port = serial.Serial(
                        port=self.port,
                        baudrate=self.baudrate,
                        timeout=0.05
                    )
                    self.get_logger().info(f'Connected to SparkFun IMU on {self.port} @ {self.baudrate}')
                except Exception as e:
                    self.get_logger().warn(f'Could not open IMU serial {self.port}: {e}. Enabling simulation fallback.')
                    self.simulation_mode = True

        self.imu_publisher = self.create_publisher(Imu, '/sparkfun/imu/data', 50)
        timer_period = 1.0 / max(1.0, sample_rate)
        self.timer = self.create_timer(timer_period, self.process_imu)
        self.get_logger().info(f'SparkFun IMU Node active (Simulation: {self.simulation_mode})')

    def process_imu(self):
        if self.simulation_mode or self.serial_port is None:
            self._publish_simulated_imu()
        else:
            self._read_hardware_imu()

    def _read_hardware_imu(self):
        try:
            while self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if not line or line == 'ICM20948_OK':
                    continue

                parts = line.split(',')
                if len(parts) < 6:
                    continue

                try:
                    ax = float(parts[0])
                    ay = float(parts[1])
                    az = float(parts[2])
                    gx = float(parts[3])
                    gy = float(parts[4])
                    gz = float(parts[5])
                except ValueError:
                    continue

                self._build_and_publish_msg(ax, ay, az, gx, gy, gz)
        except Exception as e:
            self.get_logger().warn(f'IMU read error: {e}')

    def _publish_simulated_imu(self):
        now = time.time()
        dt = now - self.last_sim_time
        self.last_sim_time = now

        # Stationary gravity vector with tiny white noise
        ax = 0.0
        ay = 0.0
        az = 9.80665

        gx = 0.0
        gy = 0.0
        gz = 0.0

        self._build_and_publish_msg(ax, ay, az, gx, gy, gz)

    def _build_and_publish_msg(self, ax, ay, az, gx, gy, gz):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Linear acceleration
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        msg.linear_acceleration_covariance = [
            ACCEL_COV_X, 0.0, 0.0,
            0.0, ACCEL_COV_Y, 0.0,
            0.0, 0.0, ACCEL_COV_Z
        ]

        # Angular velocity with empirical bias calibration applied
        msg.angular_velocity.x = float(gx - GYRO_BIAS_X)
        msg.angular_velocity.y = float(gy - GYRO_BIAS_Y)
        msg.angular_velocity.z = float(gz - GYRO_BIAS_Z)
        msg.angular_velocity_covariance = [
            GYRO_COV_X, 0.0, 0.0,
            0.0, GYRO_COV_Y, 0.0,
            0.0, 0.0, GYRO_COV_Z
        ]

        # Set orientation unknown (-1 in index 0) since raw 6-DOF lacks absolute reference
        msg.orientation_covariance[0] = -1.0

        self.imu_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SparkFunIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
