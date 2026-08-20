"""One-command autonomous course discovery followed by pure-pursuit racing."""

import os

from ament_index_python.packages import get_package_share_directory
from gap_follow.speed_overrides import mapping_speed_overrides
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    include_bringup_arg = DeclareLaunchArgument(
        'include_bringup', default_value='true',
        description=(
            'Start VESC, LiDAR, joystick, and command mux too. '
            'Set false if already running.'))
    mapping_max_speed_arg = DeclareLaunchArgument(
        'mapping_max_speed', default_value='',
        description=(
            'Optional speed cap while discovering and recording the course. '
            'Empty (the default) means no cap: gap_follow.yaml governs, and '
            'the sensed curvature/clearance caps do the limiting. Set a '
            'number for a cautious first look at a course nobody trusts yet '
            '-- corner_speed and corner_speed_wide scale with it.'))
    mapping_min_speed_arg = DeclareLaunchArgument(
        'mapping_min_speed', default_value='',
        description='Optional speed floor. Empty means gap_follow.yaml governs.')
    mapping_laps_arg = DeclareLaunchArgument(
        'mapping_laps', default_value='2',
        description='Default 2: discover/close SLAM loop, then record one settled raceline lap.')
    output_directory_arg = DeclareLaunchArgument(
        'output_directory', default_value='~/.ros/racerbot_auto',
        description='Parent directory for generated map, pose graph, and raceline files.')
    diagnostics_arg = DeclareLaunchArgument(
        'diagnostics', default_value='true',
        description=(
            'Run race_diagnostics alongside, so the run leaves a machine-'
            'readable record (pose lag, watchdog stops, pipeline health) '
            'instead of only terminal scrollback. Subscribe-only and cheap; '
            'the rosbag, which is not cheap, stays off unless record_bag is '
            'set. Read the result with `ros2 run race_diagnostics '
            'summarize_run <dir>`.'))
    record_bag_arg = DeclareLaunchArgument(
        'record_bag', default_value='false',
        description=(
            'Bag the run as well. Off by default: /scan alone is about ten '
            'times everything else combined, and a 126m mapping lap is a '
            'long time to be writing it.'))
    supervisor_config_arg = DeclareLaunchArgument(
        'supervisor_config',
        default_value=os.path.join(
            get_package_share_directory('pure_pursuit'), 'config', 'auto_map_race.yaml'),
        description=(
            'Parameter file for auto_map_race_node. Point it at a copy to race a '
            'tight course more slowly (profile_max_speed / profile_max_lateral_accel) '
            'without editing the packaged config.'))

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('f1tenth_stack'), 'launch', 'bringup_launch.py')),
        condition=IfCondition(LaunchConfiguration('include_bringup')),
    )
    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('race_diagnostics'),
            'launch', 'record_run.py')),
        launch_arguments={'record_bag': LaunchConfiguration('record_bag')}.items(),
        condition=IfCondition(LaunchConfiguration('diagnostics')),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('racerbot_launch'), 'launch', 'slam_launch.py')),
    )

    gap_config = os.path.join(
        get_package_share_directory('gap_follow'), 'config', 'gap_follow.yaml')
    pure_config = os.path.join(
        get_package_share_directory('pure_pursuit'), 'config', 'pure_pursuit.yaml')

    def mapping_controller(context):
        # Empty speed arguments mean "no cap" and produce no overrides at
        # all -- gap_follow maps at its own tuned speeds. The 2026-08-19
        # run is why that is the default: a forced max_speed of 1.0 was
        # the binding limit on 81% of that run's driving ticks, with no
        # sensed cap ever below it.
        #
        # When a cap *is* asked for, it cannot be applied alone:
        # corner_speed and corner_speed_wide are defined relative to
        # max_speed and the node rejects the set at startup if lowering
        # max_speed leaves them above it. That killed gap_follow_node on
        # the same day and, with nothing publishing /auto_map/drive, left
        # the whole one-command run parked in pure_pursuit's
        # 'waiting_for_profile'. See gap_follow/speed_overrides.py.
        speeds = mapping_speed_overrides(
            gap_config,
            LaunchConfiguration('mapping_max_speed').perform(context),
            LaunchConfiguration('mapping_min_speed').perform(context))
        return [Node(
            package='gap_follow',
            executable='gap_follow_node',
            name='gap_follow_node',
            output='screen',
            parameters=[gap_config, dict(
                {'drive_topic': '/auto_map/drive'}, **speeds)],
        )]

    racing_controller = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[pure_config, {
            'waypoints_file': '',
            'wait_for_waypoints': True,
            'pose_topic': '/slam_pose',
            'drive_topic': '/auto_race/drive',
            'opponent_detection_mode': 'map',
        }],
    )
    supervisor = Node(
        package='pure_pursuit',
        executable='auto_map_race_node',
        name='auto_map_race_node',
        output='screen',
        parameters=[LaunchConfiguration('supervisor_config'), {
            'mapping_laps': LaunchConfiguration('mapping_laps'),
            'output_directory': LaunchConfiguration('output_directory'),
        }],
    )

    return LaunchDescription([
        include_bringup_arg,
        mapping_max_speed_arg,
        mapping_min_speed_arg,
        mapping_laps_arg,
        output_directory_arg,
        supervisor_config_arg,
        diagnostics_arg,
        record_bag_arg,
        bringup,
        diagnostics,
        slam,
        OpaqueFunction(function=mapping_controller),
        racing_controller,
        supervisor,
    ])
