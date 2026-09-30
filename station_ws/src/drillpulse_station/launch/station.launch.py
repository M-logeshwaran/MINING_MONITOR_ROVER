#!/usr/bin/env python3
# =========================================================
# FILE: station.launch.py
# PURPOSE: Primary operator workstation launch file. Brings up the
#          station LoRa transceiver, real-time dual-panel dashboard,
#          and report aggregation system.
# INPUT ARGUMENTS:
#   - web_port: Dashboard HTTP/WebSocket port (default: 8080)
#   - transport_mode: 'sim' (UDP) or 'serial' (hardware UART)
#   - lora_port: Station local UDP/serial port (default: 9002)
#   - rover_ip: Rover IP / hostname (default: 127.0.0.1)
#   - rover_port: Rover UDP port (default: 9001)
# TF OWNERSHIP: None
# =========================================================

# =========================================================
# USER CONFIGURATION
# =========================================================
DEFAULT_WEB_PORT = 8080
DEFAULT_TRANSPORT_MODE = 'sim'
DEFAULT_LORA_PORT = 9002
DEFAULT_ROVER_PORT = 9001
DEFAULT_ROVER_IP = '127.0.0.1'
# =========================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    web_port_arg = DeclareLaunchArgument('web_port', default_value=str(DEFAULT_WEB_PORT))
    transport_mode_arg = DeclareLaunchArgument('transport_mode', default_value=DEFAULT_TRANSPORT_MODE)
    lora_port_arg = DeclareLaunchArgument('lora_port', default_value=str(DEFAULT_LORA_PORT))
    rover_ip_arg = DeclareLaunchArgument('rover_ip', default_value=DEFAULT_ROVER_IP)
    rover_port_arg = DeclareLaunchArgument('rover_port', default_value=str(DEFAULT_ROVER_PORT))

    dashboard_node = Node(
        package='drillpulse_dashboard',
        executable='dashboard_node',
        name='dashboard_node',
        output='screen',
        parameters=[{
            'web_port': LaunchConfiguration('web_port'),
            'map_yaml': '/home/loki/SIH_REPO_FINAL/maps/rover_map.yaml',
            'report_dir': '/home/loki/SIH_REPO_FINAL/reports',
        }]
    )

    station_transport_node = Node(
        package='drillpulse_transport',
        executable='lora_transceiver_node',
        name='station_lora_transceiver',
        output='screen',
        parameters=[{
            'role': 'station',
            'mode': LaunchConfiguration('transport_mode'),
            'local_udp_port': LaunchConfiguration('lora_port'),
            'remote_udp_port': LaunchConfiguration('rover_port'),
            'remote_ip': LaunchConfiguration('rover_ip'),
        }]
    )

    report_node = Node(
        package='drillpulse_report',
        executable='report_generator_node',
        name='station_report_generator',
        output='screen',
        parameters=[{
            'log_dir': '/home/loki/SIH_REPO_FINAL/logs',
            'report_dir': '/home/loki/SIH_REPO_FINAL/reports',
            'auto_generate': True,
        }]
    )

    return LaunchDescription([
        web_port_arg,
        transport_mode_arg,
        lora_port_arg,
        rover_ip_arg,
        rover_port_arg,
        LogInfo(msg="=== Starting DrillPulse Operator Workstation ==="),
        dashboard_node,
        station_transport_node,
        report_node,
    ])
