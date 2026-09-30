#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Onboard Rover Master Launch File
# Platform: Ubuntu 24.04 LTS + ROS 2 Jazzy
# Initializes Hardware Bridges, Sensors, TF Tree, and Odometry Stack
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('drillpulse_bringup')
    odom_share = get_package_share_directory('drillpulse_odometry')

    ekf_config_path = os.path.join(odom_share, 'config', 'ekf.yaml')
    rover_odom_config_path = os.path.join(odom_share, 'config', 'rover_odom.yaml')

    overlay_lib = '/home/loki/SIH_REPO_FINAL/rover_ws/diagnostic_updater_overlay/opt/ros/jazzy/lib'
    current_ld = os.environ.get('LD_LIBRARY_PATH', '')
    ld_library_path = f"{overlay_lib}:{current_ld}" if os.path.exists(overlay_lib) else current_ld

    # Launch Configurations
    start_lidar_arg = DeclareLaunchArgument('start_lidar', default_value='true', description='Start RPLIDAR node')
    start_imu_arg = DeclareLaunchArgument('start_imu', default_value='true', description='Start SparkFun IMU node')
    start_encoders_arg = DeclareLaunchArgument('start_encoders', default_value='true', description='Start STM32 encoder bridge')
    start_motors_arg = DeclareLaunchArgument('start_motors', default_value='true', description='Start motor & telemetry bridge')
    simulation_mode_arg = DeclareLaunchArgument('simulation_mode', default_value='false', description='Simulation/Test fallback mode')
    odometry_mode_arg = DeclareLaunchArgument('odometry_mode', default_value='FUSED', description='Odometry Mode: FUSED, ENCODER_ONLY, IMU_ONLY')

    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0', description='LiDAR USB serial port')
    lidar_baud_arg = DeclareLaunchArgument('lidar_baudrate', default_value='115200', description='LiDAR baudrate')
    imu_port_arg = DeclareLaunchArgument('imu_port', default_value='/dev/ttyACM0', description='IMU USB serial port')
    rover_serial_arg = DeclareLaunchArgument('rover_serial_port', default_value='/dev/ttyACM1', description='STM32 serial port')

    # ============================================================
    # 1. STATIC TRANSFORM BROADCASTERS
    # Frame hierarchy: base_link -> [laser, sparkfun_imu_link, camera_link, thermal_link]
    # ============================================================
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_broadcaster',
        arguments=['--x', '0.25', '--y', '0.0', '--z', '0.20', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        output='screen'
    )

    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_broadcaster',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.15', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'sparkfun_imu_link'],
        output='screen'
    )

    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_broadcaster',
        arguments=['--x', '0.30', '--y', '0.0', '--z', '0.25', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
        output='screen'
    )

    tf_thermal = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_thermal_broadcaster',
        arguments=['--x', '0.30', '--y', '0.05', '--z', '0.25', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'thermal_link'],
        output='screen'
    )

    # ============================================================
    # 2. SENSOR & HARDWARE BRIDGES
    # ============================================================
    # RPLIDAR A1/A2 Driver
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_lidar')),
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }]
    )

    # SparkFun ICM-20948 IMU
    imu_node = Node(
        package='drillpulse_imu',
        executable='sparkfun_imu_node',
        name='sparkfun_imu_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_imu')),
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baudrate': 115200,
            'frame_id': 'sparkfun_imu_link',
            'simulation_mode': LaunchConfiguration('simulation_mode')
        }]
    )

    # STM32 Encoder Bridge
    encoder_node = Node(
        package='drillpulse_hardware',
        executable='stm32_encoder_bridge',
        name='stm32_encoder_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_encoders')),
        parameters=[{
            'rover_port': LaunchConfiguration('rover_serial_port'),
            'rover_baud': 115200,
            'simulation_mode': LaunchConfiguration('simulation_mode')
        }]
    )

    # Motor & Telemetry Bridge
    motor_node = Node(
        package='drillpulse_hardware',
        executable='motor_telemetry_bridge',
        name='motor_telemetry_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_motors')),
        parameters=[{
            'simulation_mode': LaunchConfiguration('simulation_mode')
        }]
    )

    # Actuator Hardware Bridge
    actuator_node = Node(
        package='drillpulse_hardware',
        executable='actuator_bridge',
        name='actuator_bridge',
        output='screen',
        parameters=[{
            'simulation_mode': LaunchConfiguration('simulation_mode')
        }]
    )

    # ============================================================
    # 3. ODOMETRY & CONSISTENCY STACK
    # ============================================================
    # Mode 1: Wheel Odometry
    wheel_odom_node = Node(
        package='drillpulse_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen',
        parameters=[rover_odom_config_path]
    )

    # Mode 2: IMU Odometry
    imu_odom_node = Node(
        package='drillpulse_odometry',
        executable='imu_odometry_node',
        name='imu_odometry_node',
        output='screen',
        parameters=[{'publish_tf': False}]
    )

    # Threshold Consistency Checker
    consistency_node = Node(
        package='drillpulse_odometry',
        executable='threshold_consistency_node',
        name='threshold_consistency_node',
        output='screen'
    )

    # Mode 3: robot_localization EKF (fuses /odom_encoder & /sparkfun/imu/data -> /odom & dynamic TF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        additional_env={'LD_LIBRARY_PATH': ld_library_path},
        remappings=[
            ('odometry/filtered', '/odom')
        ]
    )

    # Odometry Mode Manager
    manager_node = Node(
        package='drillpulse_odometry',
        executable='odometry_manager_node',
        name='odometry_manager_node',
        output='screen',
        parameters=[{'mode': LaunchConfiguration('odometry_mode')}]
    )

    return LaunchDescription([
        start_lidar_arg,
        start_imu_arg,
        start_encoders_arg,
        start_motors_arg,
        simulation_mode_arg,
        odometry_mode_arg,
        lidar_port_arg,
        lidar_baud_arg,
        imu_port_arg,
        rover_serial_arg,

        tf_laser,
        tf_imu,
        tf_camera,
        tf_thermal,

        rplidar_node,
        imu_node,
        encoder_node,
        motor_node,
        actuator_node,

        wheel_odom_node,
        imu_odom_node,
        consistency_node,
        ekf_node,
        manager_node
    ])
