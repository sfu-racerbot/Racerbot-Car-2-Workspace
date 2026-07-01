import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    gap_follow's own speed parameters (config/gap_follow.yaml) are
    deliberately overridden slower here by default -- this is the car's
    *first* look at a track it may never have driven before, and mapping
    accuracy benefits from a slow, steady lap more than a fast one does.
    Override mapping_max_speed/mapping_min_speed on the command line once
    you trust the track and the car's behavior on it, e.g.:

        ros2 launch racerbot_launch autonomous_mapping_launch.py \\
            mapping_max_speed:=1.5 mapping_min_speed:=0.6
    """
    max_speed_arg = DeclareLaunchArgument(
        'mapping_max_speed', default_value='1.0',
        description="Deliberately slower than gap_follow.yaml's own default -- a first "
                    "autonomous lap around a possibly-unfamiliar track should be cautious."
    )
    min_speed_arg = DeclareLaunchArgument(
        'mapping_min_speed', default_value='0.4',
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
    gap_follow_node = Node(
        package='gap_follow',
        executable='gap_follow_node',
        name='gap_follow_node',
        output='screen',
        parameters=[
            gap_follow_config,
            {
                'max_speed': LaunchConfiguration('mapping_max_speed'),
                'min_speed': LaunchConfiguration('mapping_min_speed'),
            },
        ],
    )

    return LaunchDescription([max_speed_arg, min_speed_arg, slam_launch, gap_follow_node])
