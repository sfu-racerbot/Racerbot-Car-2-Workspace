import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('usb_cam_stream'),
        'config',
        'usb_cam_stream.yaml'
    )

    camera_stream_node = Node(
        package='usb_cam_stream',
        executable='camera_stream_node',
        name='usb_cam_stream_node',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([camera_stream_node])
