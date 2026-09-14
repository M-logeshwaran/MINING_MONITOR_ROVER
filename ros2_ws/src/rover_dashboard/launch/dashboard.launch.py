from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rover_dashboard",
                executable="thermal",
                name="thermal_node",
                output="screen",
            ),
            Node(
                package="rover_dashboard",
                executable="dashboard",
                name="dashboard_node",
                output="screen",
            ),
        ]
    )
