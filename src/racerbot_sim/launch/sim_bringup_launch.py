"""Simulated stand-in for f1tenth_stack/bringup_launch.py.

Same shape as the real foundation layer -- hardware in, arbitration, no
control layer of its own -- with the two hardware halves swapped for the
simulator:

    bringup_launch.py                    sim_bringup_launch.py
    -----------------                    ---------------------
    joy_node (physical F710)             sim_joy_node (synthetic LB)
    urg_node (Hokuyo)                 \
    vesc_driver + ackermann_to_vesc    >  gym_bridge_node
    vesc_to_odom                      /
    ackermann_mux                        ackermann_mux         (identical)
    static base_link->laser              static base_link->laser (identical)

The mux and the static transform are literally the same nodes with the
same config file, so `/teleop` still outranks `/drive` in simulation for
the same reason it does on the car.

Then put exactly one control layer on top, in another terminal, exactly
as on the car -- or use sim_auto_map_race_launch.py, which stacks the
real automatic composition on this.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'mux.yaml')

    arguments = [
        DeclareLaunchArgument(
            'track', default_value='indoor_oval',
            description=(
                'Generated closed-loop layout: indoor_oval, indoor_tight, '
                'indoor_wide, asb_10000.')),
        DeclareLaunchArgument(
            'opponents', default_value='',
            description=(
                'Semicolon-separated opponent list, each "offset_m,speed_mps,lateral_m". '
                'Empty means solo. Speed 0 parks that car on the line.')),
        DeclareLaunchArgument(
            'seed', default_value='12345'),
        DeclareLaunchArgument(
            'hold_deadman', default_value='true',
            description='Whether the synthetic controller holds LB. False proves the car stays stopped.'),
        DeclareLaunchArgument(
            'release_after_sec', default_value='-1.0',
            description='Release the synthetic LB this many seconds in (-1 = never).'),
        DeclareLaunchArgument(
            'odom_speed_scale', default_value='1.0',
            description='Odometry scale error, as vesc_to_odom would have. 1.0 is a perfect wheel-speed estimate.'),
    ]

    gym_bridge = Node(
        package='racerbot_sim',
        executable='gym_bridge_node',
        name='gym_bridge_node',
        output='screen',
        parameters=[{
            'track': LaunchConfiguration('track'),
            'seed': LaunchConfiguration('seed'),
            'odom_speed_scale': LaunchConfiguration('odom_speed_scale'),
            # Forced to str: an empty argument would otherwise YAML-parse to
            # None, and "3,1.0,0" would parse to a list.
            'opponents': ParameterValue(
                LaunchConfiguration('opponents'), value_type=str),
        }],
    )
    sim_joy = Node(
        package='racerbot_sim',
        executable='sim_joy_node',
        name='sim_joy_node',
        output='screen',
        parameters=[{
            'hold_deadman': LaunchConfiguration('hold_deadman'),
            'release_after_sec': LaunchConfiguration('release_after_sec'),
        }],
    )
    ackermann_mux = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[mux_config],
        # Copied verbatim from bringup_launch.py, dead remap included: the
        # mux advertises "ackermann_cmd", never "ackermann_cmd_out", so this
        # renames nothing. Kept identical anyway -- the value of this launch
        # file is that the arbitration here is bit-for-bit the car's.
        remappings=[('ackermann_cmd_out', 'ackermann_drive')],
    )
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.33', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser'],
    )

    return LaunchDescription(arguments + [
        gym_bridge, sim_joy, ackermann_mux, static_tf])
