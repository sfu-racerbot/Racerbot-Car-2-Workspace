"""The real auto_map_race composition, driven by the simulator.

Deliberately *includes* racerbot_launch's `auto_map_race_launch.py` with
`include_bringup:=false` rather than reproducing its contents. A copy
would drift, and then this would be validating a launch file nobody
runs. The only substitution is the hardware layer underneath.

    ros2 launch racerbot_sim sim_auto_map_race_launch.py
    ros2 launch racerbot_sim sim_auto_map_race_launch.py \\
        track:=indoor_wide opponents:="12,1.0,0.0;24,0.8,0.0"

Add `dashboard:=true` for the browser view on http://localhost:8080/.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('racerbot_sim')

    arguments = [
        DeclareLaunchArgument('track', default_value='indoor_oval'),
        DeclareLaunchArgument('opponents', default_value=''),
        DeclareLaunchArgument('seed', default_value='12345'),
        DeclareLaunchArgument('hold_deadman', default_value='true'),
        DeclareLaunchArgument('release_after_sec', default_value='-1.0'),
        DeclareLaunchArgument('odom_speed_scale', default_value='1.0'),
        DeclareLaunchArgument('mapping_max_speed', default_value='1.0'),
        DeclareLaunchArgument('mapping_min_speed', default_value='0.4'),
        DeclareLaunchArgument('mapping_laps', default_value='2'),
        DeclareLaunchArgument(
            'output_directory', default_value='~/.ros/racerbot_sim/auto'),
        DeclareLaunchArgument(
            'supervisor_config', default_value=os.path.join(
                get_package_share_directory('pure_pursuit'),
                'config', 'auto_map_race.yaml'),
            description='Passed straight through, so a sweep can vary the racing profile.'),
        DeclareLaunchArgument('dashboard', default_value='false'),
    ]

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'sim_bringup_launch.py')),
        launch_arguments={
            'track': LaunchConfiguration('track'),
            'opponents': LaunchConfiguration('opponents'),
            'seed': LaunchConfiguration('seed'),
            'hold_deadman': LaunchConfiguration('hold_deadman'),
            'release_after_sec': LaunchConfiguration('release_after_sec'),
            'odom_speed_scale': LaunchConfiguration('odom_speed_scale'),
        }.items(),
    )
    auto_map_race = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('racerbot_launch'),
            'launch', 'auto_map_race_launch.py')),
        launch_arguments={
            'include_bringup': 'false',
            'mapping_max_speed': LaunchConfiguration('mapping_max_speed'),
            'mapping_min_speed': LaunchConfiguration('mapping_min_speed'),
            'mapping_laps': LaunchConfiguration('mapping_laps'),
            'output_directory': LaunchConfiguration('output_directory'),
            'supervisor_config': LaunchConfiguration('supervisor_config'),
        }.items(),
    )
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('web_dashboard'),
            'launch', 'web_dashboard_launch.py')),
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    return LaunchDescription(arguments + [sim_bringup, auto_map_race, dashboard])
