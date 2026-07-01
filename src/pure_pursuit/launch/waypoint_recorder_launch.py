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
        'waypoint_recorder.yaml'
    )
    config_dict = yaml.safe_load(open(config_path, 'r'))
    default_output_file = config_dict['waypoint_recorder_node']['ros__parameters'].get('output_file', '')

    output_arg = DeclareLaunchArgument(
        'output_file',
        default_value=default_output_file,
        description='Where to write the recorded (x,y) waypoints CSV, e.g.: '
                     'ros2 launch pure_pursuit waypoint_recorder_launch.py '
                     'output_file:=/home/racerbotcar-2/racerbot-ws/src/pure_pursuit/waypoints/my_track_raw.csv'
    )

    recorder_node = Node(
        package='pure_pursuit',
        executable='waypoint_recorder_node',
        name='waypoint_recorder_node',
        output='screen',
        parameters=[
            config_path,
            {'output_file': LaunchConfiguration('output_file')},
        ],
    )

    return LaunchDescription([output_arg, recorder_node])
