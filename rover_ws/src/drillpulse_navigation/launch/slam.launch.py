#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('drillpulse_navigation')
    default_config = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_config)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    return LaunchDescription([params_file_arg, slam_node])
