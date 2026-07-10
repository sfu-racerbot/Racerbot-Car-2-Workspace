import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """MJPEG stream of the RealSense D435i's color feed, on port 9090 --
    the port web_dashboard's camera panel auto-connects to. Same
    camera_stream_node as usb_cam_stream_launch.py, but sourced from the
    /camera/camera/color/image_raw topic instead of a V4L2 device (the
    RealSense's device is held exclusively by realsense2_camera_node --
    see docs/realsense-camera.md). Needs
    `ros2 launch racerbot_launch realsense_camera_launch.py` running to
    have frames to serve.
    """
    config_path = os.path.join(
        get_package_share_directory('usb_cam_stream'),
        'config',
        'realsense_stream.yaml'
    )

    camera_stream_node = Node(
        package='usb_cam_stream',
        executable='camera_stream_node',
        name='usb_cam_stream_node',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([camera_stream_node])
