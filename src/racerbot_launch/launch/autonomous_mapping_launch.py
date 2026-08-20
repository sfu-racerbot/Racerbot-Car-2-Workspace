import os

from ament_index_python.packages import get_package_share_directory
from gap_follow.speed_overrides import mapping_speed_overrides
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Autonomous mapping: slam_toolbox + gap_follow together, so the car
    builds the map by driving *itself* around the track reactively,
    instead of a human steering it by hand. See
    docs/operations.md#building-a-map-autonomously-no-steering-required
    and docs/racing-autonomy.md for the full explanation of why this
    needs zero new algorithm code -- gap_follow already drives with no
    map at all, and slam_toolbox already builds a map from whatever
    /scan + /odom it sees go by, regardless of what's doing the driving.

    Still requires a human holding LB the *entire* time -- gap_follow's
    mandatory deadman check (docs/architecture.md) means nobody needs to
    touch the steering stick, but the car will not move at all unless LB
    is held, and letting go stops it immediately. That's a workspace
    safety policy, not a suggestion, and it applies here exactly as it
    does to every other autonomy node.

    gap_follow drives at its own tuned speeds (config/gap_follow.yaml) by
    default. This used to force a 1.0m/s cap on every run, and the
    2026-08-19 run measured what that cost: 154 of 191 driving ticks were
    commanded at exactly that cap, so it -- not the sensed curvature or
    clearance limits -- was what the car was obeying 81% of the time.

    For a genuinely unfamiliar track, cap it explicitly:

        ros2 launch racerbot_launch autonomous_mapping_launch.py \\
            mapping_max_speed:=1.5 mapping_min_speed:=0.6

    A cap also scales the parameters defined relative to max_speed
    (corner_speed, corner_speed_wide), because the node validates their
    ordering against max_speed at startup and will not run at all if only
    max_speed moves.
    """
    max_speed_arg = DeclareLaunchArgument(
        'mapping_max_speed', default_value='',
        description="Optional cap, empty by default -- gap_follow.yaml's own tuned "
                    "speeds govern. Set a number for a cautious first lap around a "
                    "track nobody trusts yet; the coupled corner caps scale with it."
    )
    min_speed_arg = DeclareLaunchArgument(
        'mapping_min_speed', default_value='',
        description='See mapping_max_speed.'
    )

    gap_follow_config = os.path.join(
        get_package_share_directory('gap_follow'), 'config', 'gap_follow.yaml')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('racerbot_launch'), 'launch', 'slam_launch.py')
        )
    )

    # The same node gap_follow_launch.py itself starts, with the same
    # config file as its base -- just with a slower speed layered on top
    # for this specific "first look at the track" scenario. Everything
    # else (safety bubble, emergency stop, the mandatory deadman check)
    # comes from gap_follow.yaml completely unchanged.
    # corner_speed and corner_speed_wide are defined relative to max_speed
    # and checked against it at startup, so lowering max_speed alone is not
    # a valid parameter set -- the node exits instead of driving. Every
    # coupled cap comes down by the same factor here; see
    # gap_follow/speed_overrides.py.
    def gap_follow_node(context):
        speeds = mapping_speed_overrides(
            gap_follow_config,
            LaunchConfiguration('mapping_max_speed').perform(context),
            LaunchConfiguration('mapping_min_speed').perform(context))
        return [Node(
            package='gap_follow',
            executable='gap_follow_node',
            name='gap_follow_node',
            output='screen',
            parameters=[gap_follow_config, speeds],
        )]

    return LaunchDescription([
        max_speed_arg, min_speed_arg, slam_launch,
        OpaqueFunction(function=gap_follow_node),
    ])
