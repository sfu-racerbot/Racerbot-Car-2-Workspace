import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'pure_pursuit.yaml'
    )
    # Read the yaml at launch-description-generation time (same trick
    # particle_filter's localize_launch.py already uses) so the
    # `waypoints_file` launch argument's default is whatever the config
    # file says, rather than an empty string. That way, running the
    # launch file with no arguments does exactly what editing the yaml
    # directly would do -- and you can still override it from the
    # command line to swap racelines without touching the yaml:
    #   ros2 launch pure_pursuit pure_pursuit_launch.py \
    #       waypoints_file:=/path/to/other_track_profiled.csv
    config_dict = yaml.safe_load(open(config_path, 'r'))
    default_waypoints_file = config_dict['pure_pursuit_node']['ros__parameters'].get('waypoints_file', '')

    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints_file,
        description='Profiled (x,y,speed) waypoints CSV to race.'
    )

    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[
            config_path,
            {'waypoints_file': LaunchConfiguration('waypoints_file')},
        ],
    )

    return LaunchDescription([waypoints_arg, pure_pursuit_node])
