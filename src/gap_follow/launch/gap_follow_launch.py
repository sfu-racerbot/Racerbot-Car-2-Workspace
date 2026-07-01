import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gap_follow_config = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'gap_follow.yaml'
    )

    gap_follow_node = Node(
        package='gap_follow',
        executable='gap_follow_node',
        name='gap_follow_node',
        output='screen',
        parameters=[gap_follow_config],
    )

    return LaunchDescription([gap_follow_node])
