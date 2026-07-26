import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('odom_calibration'),
        'config',
        'odom_calibration.yaml',
    )
    return LaunchDescription([
        Node(
            package='odom_calibration',
            executable='calibration_node',
            name='odom_calibration_node',
            output='screen',
            parameters=[config_path],
        ),
    ])
