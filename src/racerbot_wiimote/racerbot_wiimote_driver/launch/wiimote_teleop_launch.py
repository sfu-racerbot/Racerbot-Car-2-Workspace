"""Drive the car with a Wiimote: a control layer, like teleop_launch.py.

    terminal 1: ros2 launch f1tenth_stack bringup_launch.py
    terminal 2: ros2 launch racerbot_wiimote_driver wiimote_teleop_launch.py

Hold "2" (the deadman), hold A to drive forward or B to reverse, tilt to
steer. Let go of "2" and the car is commanded to stop.

Run this INSTEAD of teleop_launch.py, never beside it: both publish /teleop,
and the F710 profile's idle zeros would fight the Wiimote's commands.

The Wiimote publishes on /wiimote/joy, not /joy, so it can never act as the
LB deadman for gap_follow or pure_pursuit. See docs/wiimote.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('racerbot_wiimote_driver')
    return LaunchDescription([
        DeclareLaunchArgument(
            'driver_config',
            default_value=os.path.join(share, 'config', 'wiimote_driver.yaml')),
        DeclareLaunchArgument(
            'teleop_config',
            default_value=os.path.join(share, 'config', 'wiimote_teleop.yaml')),
        Node(
            package='racerbot_wiimote_driver',
            executable='wiimote_driver_node',
            name='wiimote_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('driver_config')],
        ),
        Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='wiimote_teleop',
            output='screen',
            parameters=[LaunchConfiguration('teleop_config')],
            remappings=[('joy', '/wiimote/joy')],
        ),
    ])
