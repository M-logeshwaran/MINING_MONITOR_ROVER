#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('drillpulse_navigation')
    default_nav_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map = '/home/loki/SIH_REPO_FINAL/maps/rover_map.yaml'

    map_arg = DeclareLaunchArgument('map', default_value=default_map)
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_nav_params)

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {'yaml_filename': LaunchConfiguration('map')}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'amcl']}]
    )

    return LaunchDescription([map_arg, params_file_arg, map_server_node, amcl_node, lifecycle_manager_node])
