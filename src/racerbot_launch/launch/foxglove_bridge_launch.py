"""foxglove_bridge for Lichtblick, with this car's settings.

    ros2 launch racerbot_launch foxglove_bridge_launch.py

Support/tooling, not a control layer: start it alongside anything, in its
own terminal, like the web dashboard. It listens on 127.0.0.1:8765 only
and is reached from off the car through the Cloudflare Tunnel.

It is NOT read-only the way the dashboard is. Clients can set parameters
and call services on every node, and can publish to /initialpose -- and
to nothing else, because a raw /drive publish would skip every driving
node's LB deadman. Read docs/foxglove-bridge.md before changing
config/foxglove_bridge.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('racerbot_launch'), 'config', 'foxglove_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='foxglove_bridge parameter file. The default keeps client '
                        'publishing limited to /initialpose -- see '
                        'docs/foxglove-bridge.md before pointing this elsewhere.'),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
