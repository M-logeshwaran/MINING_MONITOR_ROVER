#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('drillpulse_navigation')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params)

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[('cmd_vel', '/cmd_vel')]
    )
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
        }]
    )

    return LaunchDescription([params_file_arg, controller, planner, behaviors, bt, lifecycle])
