"""The Wiimote driver on its own: Joy on /wiimote/joy, nothing that drives.

    ros2 launch racerbot_wiimote_driver wiimote_driver_launch.py
    ros2 launch racerbot_wiimote_driver wiimote_driver_launch.py raw_mode:=true

Settings are in config/wiimote_driver.yaml. To drive the car with the
Wiimote, use wiimote_teleop_launch.py instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('racerbot_wiimote_driver'), 'config',
        'wiimote_driver.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config,
                              description='Driver parameter file.'),
        DeclareLaunchArgument(
            'raw_mode', default_value='false',
            description='true: publish no Joy and leave every LED to /wiimote/led.'),
        Node(
            package='racerbot_wiimote_driver',
            executable='wiimote_driver_node',
            name='wiimote_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('config'),
                {'raw_mode': ParameterValue(LaunchConfiguration('raw_mode'), value_type=bool)},
            ],
        ),
    ])
