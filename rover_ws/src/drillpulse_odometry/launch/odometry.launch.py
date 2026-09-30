#!/usr/bin/env python3
# ============================================================
# DRILLPULSE — Integrated Odometry Launch File
# Starts Wheel Odom, IMU Odom, Threshold Diagnostics, and EKF Fusion
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drillpulse_odometry')
    default_rover_config = os.path.join(pkg_share, 'config', 'rover_odom.yaml')
    default_ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    rover_config_arg = DeclareLaunchArgument(
        'rover_params_file',
        default_value=default_rover_config,
        description='Path to rover_odom.yaml'
    )
    ekf_config_arg = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=default_ekf_config,
        description='Path to ekf.yaml'
    )

    # 1. Wheel Odometry Node (Mode 1)
    wheel_odom_node = Node(
        package='drillpulse_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen',
        parameters=[LaunchConfiguration('rover_params_file')]
    )

    # 2. IMU Odometry Node (Mode 2)
    imu_odom_node = Node(
        package='drillpulse_odometry',
        executable='imu_odometry_node',
        name='imu_odometry_node',
        output='screen',
        parameters=[{'publish_tf': False}]
    )

    # 3. Threshold Consistency & Sensor Health Checker
    consistency_node = Node(
        package='drillpulse_odometry',
        executable='threshold_consistency_node',
        name='threshold_consistency_node',
        output='screen'
    )

    overlay_lib = '/home/loki/SIH_REPO_FINAL/rover_ws/diagnostic_updater_overlay/opt/ros/jazzy/lib'
    current_ld = os.environ.get('LD_LIBRARY_PATH', '')
    ld_library_path = f"{overlay_lib}:{current_ld}" if os.path.exists(overlay_lib) else current_ld

    # 4. robot_localization EKF (Mode 3: Fused Odometry + Dynamic TF odom -> base_link)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_params_file')],
        additional_env={'LD_LIBRARY_PATH': ld_library_path},
        remappings=[
            ('odometry/filtered', '/odom')
        ]
    )

    # 5. Odometry Mode Manager
    manager_node = Node(
        package='drillpulse_odometry',
        executable='odometry_manager_node',
        name='odometry_manager_node',
        output='screen',
        parameters=[{'mode': 'FUSED'}]
    )

    return LaunchDescription([
        rover_config_arg,
        ekf_config_arg,
        wheel_odom_node,
        imu_odom_node,
        consistency_node,
        ekf_node,
        manager_node
    ])
