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
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def _validate_controlled_cars(context):
    value = LaunchConfiguration('controlled_cars').perform(context)
    if value not in ('1', '2'):
        raise RuntimeError('controlled_cars must be 1 or 2 in sim_bringup_launch.py')
    return []


def _mux(mux_config, namespace='', condition=None):
    root = f'/{namespace}' if namespace else ''
    return GroupAction([
        *(SetRemap(src=root + suffix,
                   dst=[LaunchConfiguration('drive_topic_prefix'), root + suffix])
          for suffix in ('/drive', '/ackermann_cmd')),
        Node(package='ackermann_mux', executable='ackermann_mux',
             name='ackermann_mux', namespace=namespace,
             parameters=[mux_config]),
    ], condition=condition)


def generate_launch_description():
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'mux.yaml')
    # YAML's root node key targets /ackermann_mux, not /car2/ackermann_mux.
    # Pass the same parameter values directly so launch scopes them to car 2.
    with open(mux_config, encoding='utf-8') as handle:
        car2_mux_config = yaml.safe_load(handle)['ackermann_mux']['ros__parameters']

    arguments = [
        DeclareLaunchArgument('publish_imu', default_value='true'),
        DeclareLaunchArgument('imu_fault', default_value='none'),
        DeclareLaunchArgument('car2_imu_fault', default_value='none'),
        DeclareLaunchArgument('drive_topic_prefix', default_value='',
                              description='Optional drive-chain topic prefix for isolated tests.'),
        DeclareLaunchArgument(
            'track', default_value='indoor_oval',
            description=(
                'Generated closed-loop layout: indoor_oval, indoor_tight, '
                'indoor_wide, asb_10000.')),
        DeclareLaunchArgument(
            'track_directory', default_value='~/.ros/racerbot_sim/tracks',
            description='Directory containing generated or imported simulator tracks.'),
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
            'car2_hold_deadman', default_value='true',
            description='Whether car 2 synthetic controller holds LB.'),
        DeclareLaunchArgument(
            'release_after_sec', default_value='-1.0',
            description='Release the synthetic LB this many seconds in (-1 = never).'),
        DeclareLaunchArgument(
            'odom_speed_scale', default_value='1.0',
            description='Odometry scale error, as vesc_to_odom would have. 1.0 is a perfect wheel-speed estimate.'),
        DeclareLaunchArgument(
            'odom_yaw_rate_scale', default_value='1.0',
            description='Dead-reckoned yaw-rate scale error from a miscalibrated steering model.'),
        DeclareLaunchArgument(
            'controlled_cars', default_value='1',
            description='Number of ROS-controlled cars: 1 or 2.'),
        DeclareLaunchArgument(
            'controlled_start_offsets_m', default_value='',
            description='Comma-separated starting offsets in metres, one per controlled car; empty uses defaults.'),
    ]

    gym_bridge = Node(
        package='racerbot_sim',
        executable='gym_bridge_node',
        name='gym_bridge_node',
        output='screen',
        parameters=[{
            'publish_imu': ParameterValue(LaunchConfiguration('publish_imu'), value_type=bool),
            'imu_fault': ParameterValue(LaunchConfiguration('imu_fault'), value_type=str),
            'car2_imu_fault': ParameterValue(LaunchConfiguration('car2_imu_fault'), value_type=str),
            'track': LaunchConfiguration('track'),
            'track_directory': LaunchConfiguration('track_directory'),
            'seed': LaunchConfiguration('seed'),
            'odom_speed_scale': LaunchConfiguration('odom_speed_scale'),
            'odom_yaw_rate_scale': LaunchConfiguration('odom_yaw_rate_scale'),
            'controlled_cars': ParameterValue(LaunchConfiguration('controlled_cars'), value_type=int),
            'controlled_start_offsets_m': ParameterValue(
                LaunchConfiguration('controlled_start_offsets_m'), value_type=str),
            # Forced to str: an empty argument would otherwise YAML-parse to
            # None, and "3,1.0,0" would parse to a list.
            'opponents': ParameterValue(
                LaunchConfiguration('opponents'), value_type=str),
        }],
    )
    gym_bridge = GroupAction([
        *(SetRemap(src=topic, dst=[LaunchConfiguration('drive_topic_prefix'), topic])
          for topic in ('/ackermann_cmd', '/car2/ackermann_cmd')),
        gym_bridge,
    ])
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
    ackermann_mux = _mux(mux_config)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        # Same measured 0.26 m mount as the real bringup_launch.py.
        arguments=['0.26', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser'],
    )

    second_car = IfCondition(PythonExpression(["'", LaunchConfiguration('controlled_cars'), "' == '2'"]))
    car2_joy = Node(
        package='racerbot_sim', executable='sim_joy_node', name='sim_joy_node',
        namespace='car2', output='screen', condition=second_car,
        parameters=[{'joy_topic': '/car2/joy',
                      'hold_deadman': LaunchConfiguration('car2_hold_deadman'),
                      'release_after_sec': LaunchConfiguration('release_after_sec')}])
    car2_mux = _mux(car2_mux_config, namespace='car2', condition=second_car)
    car2_tf = Node(package='tf2_ros', executable='static_transform_publisher',
                   name='static_baselink_to_laser', namespace='car2', condition=second_car,
                   arguments=['0.26', '0.0', '0.11', '0.0', '0.0', '0.0',
                              'car2/base_link', 'car2/laser'])

    return LaunchDescription(arguments + [OpaqueFunction(function=_validate_controlled_cars),
        gym_bridge, sim_joy, ackermann_mux, static_tf, car2_joy, car2_mux, car2_tf])
