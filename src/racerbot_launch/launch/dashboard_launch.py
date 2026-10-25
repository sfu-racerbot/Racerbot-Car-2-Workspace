"""The web dashboard's car-side server, with car 2's settings.

    ros2 launch racerbot_launch dashboard_launch.py

Exactly `ros2 launch web_dashboard web_dashboard_launch.py` with
car_config:=config/web_dashboard_rb2.yaml from this package: the package
(in the src/web_dashboards submodule) ships generic defaults, and this
car's site, geometry, tunable nodes, stoppable processes and map folders
live in that YAML. Open it through https://dashboard.sfuracerbot.ca.

Support/tooling, not a control layer: publishes to no topic, so it is safe
to start alongside anything, at any time. On car 2 it runs at boot as
racerbot-dashboard.service (systemd/ in this package), so don't also start it
by hand there: the second copy dies on the busy port 8080. See
docs/web-dashboard.md#the-dashboards-boot-service.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    car_config = os.path.join(
        get_package_share_directory('racerbot_launch'), 'config', 'web_dashboard_rb2.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('web_dashboard'), 'launch',
                'web_dashboard_launch.py')),
            launch_arguments={'car_config': car_config}.items(),
        ),
    ])
