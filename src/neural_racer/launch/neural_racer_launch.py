"""Launch neural_racer_node. Shadow mode unless shadow:=false.

    ros2 launch neural_racer neural_racer_launch.py model_hash:=<hash>
    ros2 launch neural_racer neural_racer_launch.py model_hash:=<hash> shadow:=false max_speed:=1.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('neural_racer'), 'config', 'neural_racer.yaml')
    args = [
        DeclareLaunchArgument('model_hash', description='approved model hash (required)'),
        DeclareLaunchArgument('shadow', default_value='true'),
        DeclareLaunchArgument('max_speed', default_value='1.0'),
        DeclareLaunchArgument('mode', default_value='reactive'),
        DeclareLaunchArgument('drive_topic', default_value='/drive'),
    ]
    node = Node(
        package='neural_racer', executable='neural_racer_node', name='neural_racer_node',
        output='screen',
        parameters=[config, {
            'model_hash': LaunchConfiguration('model_hash'),
            'shadow': ParameterValue(LaunchConfiguration('shadow'), value_type=bool),
            'max_speed': ParameterValue(LaunchConfiguration('max_speed'), value_type=float),
            'mode': LaunchConfiguration('mode'),
            'drive_topic': LaunchConfiguration('drive_topic'),
        }])
    return LaunchDescription(args + [node])
