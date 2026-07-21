import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Everything the web dashboard can show, minus anything that can move
    the car: LiDAR + RealSense camera + the camera->MJPEG bridge + the
    dashboard itself, all in one launch, for testing the dashboard/sensors
    without bringing up the driving stack (VESC/joy/ackermann_mux from
    f1tenth_stack's bringup_launch.py).

    None of these nodes touch /drive, /ackermann_cmd, or
    /commands/motor|servo/* -- support/tooling per
    docs/adding-your-own-code.md, same category as web_dashboard/
    usb_cam_stream/realsense_camera_launch.py already are -- so there's no
    LB-deadman check and no bringup ordering to worry about here, unlike
    the driving-code procedures in docs/operations.md.

    For actual driving/racing, don't use this file -- launch
    bringup_launch.py + a control layer as usual (see docs/operations.md),
    and web_dashboard_launch.py on its own if you also want the dashboard
    up alongside them.

        ros2 launch racerbot_launch dashboard_test_launch.py

    Then open http://<car-ip>:8080/.
    """
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'sensors.yaml')

    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[sensors_config],
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser'],
    )

    realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('racerbot_launch'), 'launch',
                'realsense_camera_launch.py')
        )
    )
    realsense_stream_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('usb_cam_stream'), 'launch',
                'realsense_stream_launch.py')
        )
    )
    web_dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('web_dashboard'), 'launch',
                'web_dashboard_launch.py')
        )
    )

    return LaunchDescription([
        urg_node,
        static_tf_node,
        realsense_camera_launch,
        realsense_stream_launch,
        web_dashboard_launch,
    ])
