"""Bringup's automatic run recorder: records every run of an algorithm.

Included by every f1tenth_stack bringup (bringup_launch.py and its two
alternates) unless `record:=false` is passed to it. Not meant to be started
on its own, though it can be, next to any bringup that lacks it:

    ros2 launch race_diagnostics auto_record_launch.py

It records only while some node publishes /drive (or another trigger topic
in config/auto_record.yaml), never beside another recorder, into
~/racerbot-ws/data/racerbot_runs/<time>/. Subscribe-free and publish-free: it reads the
ROS graph and nothing else, so it starts nothing that can move the car.
docs/run-diagnostics.md#automatic-recording
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('race_diagnostics'), 'config', 'auto_record.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'record_output_directory', default_value='~/racerbot-ws/data/racerbot_runs',
            description='Where automatic recordings go, one timestamped '
                        'subdirectory per run.'),
        Node(
            package='race_diagnostics',
            executable='auto_record',
            name='auto_record',
            output='screen',
            parameters=[config, {
                'output_directory': LaunchConfiguration('record_output_directory')}],
            # On bringup's Ctrl+C the node closes the bag of a recording in
            # progress (up to its stop_timeout_sec, 15 s); launch must not
            # escalate to SIGKILL before that.
            sigterm_timeout='20',
        ),
    ])
