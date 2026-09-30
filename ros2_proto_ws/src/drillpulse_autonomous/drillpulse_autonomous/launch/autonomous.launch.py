import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('drillpulse_autonomous')
    default_config_path = os.path.join(pkg_share, 'config', 'autonomous.yaml')
    nav_points_path = os.path.join(pkg_share, 'config', 'navigation_points.yaml')

    mode = LaunchConfiguration('mode').perform(context)
    goal_x = float(LaunchConfiguration('goal_x').perform(context))
    goal_y = float(LaunchConfiguration('goal_y').perform(context))
    simulation_str = LaunchConfiguration('simulation').perform(context)

    simulation_val = (simulation_str.lower() == 'true')

    node = Node(
        package='drillpulse_autonomous',
        executable='autonomous_controller',
        name='drillpulse_autonomous_node',
        output='screen',
        parameters=[
            default_config_path,
            {
                'mode': mode,
                'goal_x': goal_x,
                'goal_y': goal_y,
                'simulation': simulation_val,
                'navigation_points_file': nav_points_path
            }
        ]
    )

    return [node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='free',
            description='Autonomous Mode: "free" (Free Roam) or "point" (Point to Point)'
        ),
        DeclareLaunchArgument(
            'goal_x',
            default_value='0.0',
            description='Destination X coordinate for point-to-point mode'
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='0.0',
            description='Destination Y coordinate for point-to-point mode'
        ),
        DeclareLaunchArgument(
            'simulation',
            default_value='false',
            description='Enable simulation mode using webcam / synthetic sensors'
        ),
        OpaqueFunction(function=launch_setup)
    ])
