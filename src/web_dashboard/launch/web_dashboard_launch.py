import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('web_dashboard'),
        'config',
        'web_dashboard.yaml'
    )

    dashboard_node = Node(
        package='web_dashboard',
        executable='dashboard_node',
        name='web_dashboard_node',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([dashboard_node])
