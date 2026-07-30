"""Record one run: probe + rosbag, into a single timestamped directory.

Run this in its own terminal ALONGSIDE the driving stack, exactly like the
web dashboard -- it is a third layer on top of the two-tier launch pattern
(see docs/architecture.md), subscribe-only, and starts nothing that can
move the car.

    ros2 launch race_diagnostics record_run.py

It prints the run directory it created. The one thing it cannot capture by
itself is the driving terminal's own stdout, because that belongs to
another process -- so start the driving stack with `| tee <dir>/launch.log`
as printed. See docs/run-diagnostics.md.
"""

import os
from time import strftime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Everything needed to reconstruct a run offline. /scan dominates the size
# (~10x everything else combined) but is also the only way to answer "what
# did the car actually see" after the fact, which is precisely the question
# that could not be answered about the 2026-07-27 collision.
DEFAULT_TOPICS = [
    '/scan', '/odom', '/slam_pose', '/drive', '/ackermann_cmd',
    '/teleop', '/tf', '/tf_static', '/map', '/joy',
    '/auto_map/drive', '/auto_race/drive',
]


def _setup(context, *args, **kwargs):
    parent = os.path.expanduser(
        LaunchConfiguration('output_directory').perform(context))
    run_dir = os.path.join(parent, strftime('%Y%m%d-%H%M%S'))
    os.makedirs(run_dir, exist_ok=True)

    topics = LaunchConfiguration('topics').perform(context).split()
    config = os.path.join(
        get_package_share_directory('race_diagnostics'),
        'config', 'race_diagnostics.yaml')

    actions = [
        LogInfo(msg=f'\n{"=" * 72}\nRecording run to: {run_dir}\n'
                    f'Start the driving stack in another terminal WITH tee, so its\n'
                    f'terminal output is captured too:\n\n'
                    f'  ros2 launch racerbot_launch auto_map_race_launch.py \\\n'
                    f'    2>&1 | tee {run_dir}/launch.log\n\n'
                    f'Afterwards:\n'
                    f'  ros2 run race_diagnostics summarize_run {run_dir}\n{"=" * 72}'),
        Node(
            package='race_diagnostics',
            executable='race_diag_node',
            name='race_diag_node',
            output='screen',
            parameters=[config, {'output_directory': run_dir}],
        ),
    ]

    if LaunchConfiguration('record_bag').perform(context).lower() in ('true', '1'):
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(run_dir, 'bag')] + topics,
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_directory', default_value='~/.ros/racerbot_runs',
            description='Parent directory; one timestamped subdirectory per run.'),
        DeclareLaunchArgument(
            'record_bag', default_value='true',
            description='Record a rosbag. Set false to capture only the probe stream.'),
        DeclareLaunchArgument(
            'topics', default_value=' '.join(DEFAULT_TOPICS),
            description='Space-separated topics to bag. Drop /scan to shrink ~10x.'),
        OpaqueFunction(function=_setup),
    ])
