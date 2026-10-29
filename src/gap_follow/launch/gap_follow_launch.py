import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from gap_follow.launch_speed import speed_cap_overrides, yaml_parameters


def generate_launch_description():
    gap_follow_config = os.path.join(
        get_package_share_directory('gap_follow'),
        'config',
        'gap_follow.yaml'
    )

    opponent_arg = DeclareLaunchArgument(
        'opponent_awareness', default_value='false',
        description='true when other cars share the track: track moving '
                    'cars, follow them, pass only with room on a straight '
                    '(docs/gap-follow.md#several-cars-at-once). Simulator-'
                    'validated only.')
    max_speed_arg = DeclareLaunchArgument(
        'max_speed', default_value='',
        description='Top speed for this run (m/s), up to 5.0: e.g. 1.0 for '
                    'a first floor test, 5.0 for a fast straight. Empty: '
                    'gap_follow.yaml\'s max_speed. Refused above 5.0. Below '
                    'the YAML value, min_speed, corner_speed and '
                    'corner_speed_wide are lowered with it; above it, only '
                    'max_speed rises.')

    def node(context):
        enabled = LaunchConfiguration('opponent_awareness').perform(context).lower()
        if enabled not in ('true', 'false'):
            raise RuntimeError(
                f"opponent_awareness:={enabled!r}: use true or false")
        speed = speed_cap_overrides(
            yaml_parameters(gap_follow_config),
            LaunchConfiguration('max_speed').perform(context))
        return [Node(
            package='gap_follow',
            executable='gap_follow_node',
            name='gap_follow_node',
            output='screen',
            parameters=[gap_follow_config,
                        {'enable_opponent_awareness': enabled == 'true'},
                        speed],
        )]

    return LaunchDescription([opponent_arg, max_speed_arg, OpaqueFunction(function=node)])
