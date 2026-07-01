import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Race-day launch: localization (particle filter against a saved map)
    plus the pure-pursuit race controller, together.

    Run this *after* f1tenth_stack's bringup_launch.py is already up
    (VESC/LiDAR/mux) -- see docs/operations.md and
    docs/racing-autonomy.md for the full procedure, including giving the
    particle filter its "2D Pose Estimate" seed in RViz before the car
    will go anywhere, and stopping joy_node/joy_teleop so /drive actually
    reaches the VESC (docs/architecture.md's safety model).

        ros2 launch racerbot_launch race_launch.py \
            waypoints_file:=/path/to/track_profiled.csv
    """
    # Read pure_pursuit's own config at generation time (same pattern
    # particle_filter's localize_launch.py and pure_pursuit_launch.py
    # both already use) so that *not* passing waypoints_file here falls
    # through to whatever pure_pursuit's own config/pure_pursuit.yaml
    # says, instead of silently overriding it with an empty string.
    pp_config_path = os.path.join(
        get_package_share_directory('pure_pursuit'), 'config', 'pure_pursuit.yaml')
    pp_config = yaml.safe_load(open(pp_config_path, 'r'))
    default_waypoints_file = pp_config['pure_pursuit_node']['ros__parameters'].get('waypoints_file', '')

    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints_file,
        description='Profiled (x,y,speed) waypoints CSV for this track.'
    )

    localize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('particle_filter'), 'launch', 'localize_launch.py')
        )
    )

    pure_pursuit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pure_pursuit'), 'launch', 'pure_pursuit_launch.py')
        ),
        launch_arguments={'waypoints_file': LaunchConfiguration('waypoints_file')}.items(),
    )

    return LaunchDescription([waypoints_arg, localize_launch, pure_pursuit_launch])
