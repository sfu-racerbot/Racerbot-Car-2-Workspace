"""One-command autonomous course discovery followed by pure-pursuit racing."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
        'mapping_max_speed', default_value='1.0',
        description='Cautious speed cap while discovering and recording the course.')
    mapping_min_speed_arg = DeclareLaunchArgument(
        'mapping_min_speed', default_value='0.4')
    mapping_laps_arg = DeclareLaunchArgument(
        'mapping_laps', default_value='2',
        description='Default 2: discover/close SLAM loop, then record one settled raceline lap.')
    output_directory_arg = DeclareLaunchArgument(
        'output_directory', default_value='~/.ros/racerbot_auto',
        description='Parent directory for generated map, pose graph, and raceline files.')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('f1tenth_stack'), 'launch', 'bringup_launch.py')),
        condition=IfCondition(LaunchConfiguration('include_bringup')),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('racerbot_launch'), 'launch', 'slam_launch.py')),
    )

    gap_config = os.path.join(
        get_package_share_directory('gap_follow'), 'config', 'gap_follow.yaml')
    pure_config = os.path.join(
        get_package_share_directory('pure_pursuit'), 'config', 'pure_pursuit.yaml')
    supervisor_config = os.path.join(
        get_package_share_directory('pure_pursuit'), 'config', 'auto_map_race.yaml')

    mapping_controller = Node(
        package='gap_follow',
        executable='gap_follow_node',
        name='gap_follow_node',
        output='screen',
        parameters=[gap_config, {
            'drive_topic': '/auto_map/drive',
            'max_speed': LaunchConfiguration('mapping_max_speed'),
            'min_speed': LaunchConfiguration('mapping_min_speed'),
        }],
    )
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
        parameters=[supervisor_config, {
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
        bringup,
        slam,
        mapping_controller,
        racing_controller,
        supervisor,
    ])
