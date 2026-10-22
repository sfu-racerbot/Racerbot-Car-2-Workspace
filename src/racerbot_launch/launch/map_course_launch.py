"""Map a course with gap_follow, save everything, and stop -- no racing.

Normally started through the wrapper, which also prints the race commands
once everything has shut down:

    ros2 run racerbot_launch map_course

It never starts the hardware: ``bringup_launch.py`` must already be running
in its own terminal (``sim_bringup_launch.py`` in simulation), the same
two-layer pattern as every other control layer here (docs/architecture.md).
Starting a second copy of the drivers beside an existing bringup makes the
two fight over the VESC serial port and the LiDAR's single connection.

What runs: slam_toolbox mapping, gap_follow driving (its commands go to auto_map_race_node, never
straight to /drive), the run recorder, and auto_map_race_node in
finish_after_mapping mode. The supervisor forwards gap_follow's commands
only while LB is held, counts laps from SLAM's pose, and after ``laps``
(default 3) closed laps saves the map, pose graph and racing line to
``~/.ros/racerbot_auto/<timestamp>/``, writes commands.txt there, stops the
car, and exits. Its exit shuts this whole launch down.

Differences from auto_map_race_launch.py: no pure_pursuit, no particle
filter, no racing handover, and every requested lap is driven (that launch
races as soon as lap 2 is clean).

The LB deadman applies throughout: gap_follow_node checks it and so does
auto_map_race_node. Releasing LB stops the car; holding it again resumes
mapping where it left off.
"""

import os

from ament_index_python.packages import get_package_share_directory
from gap_follow.speed_overrides import mapping_speed_overrides
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'laps', default_value='3',
            description='Closed laps to drive before saving. The first is '
                        'discovery; the last is kept as the racing line. '
                        'Up to two more are driven if the last lap fails the '
                        'quality check.'),
        DeclareLaunchArgument(
            'mapping_max_speed', default_value='',
            description='Optional speed cap while mapping (m/s). Empty: '
                        'gap_follow.yaml governs. Use 1.5 on a course nobody '
                        'has driven yet.'),
        DeclareLaunchArgument(
            'mapping_min_speed', default_value='',
            description='Optional speed floor. Empty: gap_follow.yaml governs.'),
        DeclareLaunchArgument(
            'output_directory', default_value='~/.ros/racerbot_auto',
            description='Parent directory of the run directory.'),
        DeclareLaunchArgument(
            'diagnostics', default_value='true',
            description='Run race_diagnostics alongside (subscribe-only).'),
        DeclareLaunchArgument(
            'record_bag', default_value='false',
            description='Bag the run as well (large: /scan dominates).'),
        DeclareLaunchArgument(
            'supervisor_config',
            default_value=os.path.join(
                get_package_share_directory('pure_pursuit'), 'config',
                'auto_map_race.yaml'),
            description='auto_map_race_node parameter file (racing-line '
                        'profile limits live here).'),
    ]

    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('race_diagnostics'), 'launch', 'record_run.py')),
        # Its own folder, given explicitly: launch arguments are shared with
        # included files, and this launch's output_directory (the map run
        # folder) would otherwise become the recorder's too.
        launch_arguments={'record_bag': LaunchConfiguration('record_bag'),
                          'output_directory': '~/.ros/racerbot_runs'}.items(),
        condition=IfCondition(LaunchConfiguration('diagnostics')),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('racerbot_launch'), 'launch', 'slam_launch.py')),
    )
    gap_config = os.path.join(
        get_package_share_directory('gap_follow'), 'config', 'gap_follow.yaml')

    def nodes(context):
        # Same speed-override rule as auto_map_race_launch.py: a cap moves
        # corner_speed/corner_speed_wide with it, or gap_follow refuses to
        # start (gap_follow/speed_overrides.py).
        speeds = mapping_speed_overrides(
            gap_config,
            LaunchConfiguration('mapping_max_speed').perform(context),
            LaunchConfiguration('mapping_min_speed').perform(context))
        laps = int(LaunchConfiguration('laps').perform(context))
        if laps < 2:
            raise RuntimeError(
                f'laps:={laps}: at least 2 are needed -- the first lap only '
                'discovers the course and is never used as the racing line.')
        gap_follow = Node(
            package='gap_follow', executable='gap_follow_node', name='gap_follow_node',
            output='screen',
            parameters=[gap_config, dict({'drive_topic': '/auto_map/drive'}, **speeds)],
        )
        supervisor = Node(
            package='pure_pursuit', executable='auto_map_race_node',
            name='auto_map_race_node', output='screen',
            parameters=[LaunchConfiguration('supervisor_config'), {
                'mapping_laps': laps,
                'max_mapping_laps': laps + 2,
                'finish_after_mapping': True,
                'output_directory': LaunchConfiguration('output_directory'),
            }],
        )
        # The supervisor exiting -- finished, crashed or Ctrl+C -- ends the
        # run: nothing else here should keep the car's stack up afterwards.
        shutdown = RegisterEventHandler(OnProcessExit(
            target_action=supervisor,
            on_exit=[EmitEvent(event=Shutdown(reason='mapping run finished'))]))
        return [gap_follow, supervisor, shutdown]

    return LaunchDescription(args + [diagnostics, slam, OpaqueFunction(function=nodes)])
