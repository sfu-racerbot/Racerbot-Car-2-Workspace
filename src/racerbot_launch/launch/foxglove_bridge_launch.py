"""TEMPORARY forwarder: foxglove_bridge moved into the web_dashboard package.

    ros2 launch web_dashboard foxglove_bridge_launch.py      # the real one

The bridge's config, launch file and safety test now live in
src/web_dashboards/car/ros/web_dashboard (sfu-racerbot/web-dashboards).
This file only includes that launch, unchanged, so the foxglove-bridge
systemd unit installed on this car before the move -- whose wrapper,
tools/systemd/foxglove-bridge.sh, runs
`ros2 launch racerbot_launch foxglove_bridge_launch.py` -- keeps working
across a restart or reboot until the unit is reinstalled from
src/web_dashboards/car/systemd/ (docs/foxglove-bridge.md).

REMOVE this file, and tools/systemd/, in the commit after that reinstall.

Client publishing stays OFF: the included launch loads web_dashboard's
config/foxglove_bridge.yaml, which has no clientPublish capability (on
foxglove_bridge 3.5.0 client_topic_whitelist is not enforced, so a
browser /drive would skip every driving node's LB deadman).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('web_dashboard'), 'launch',
            'foxglove_bridge_launch.py'))),
    ])
