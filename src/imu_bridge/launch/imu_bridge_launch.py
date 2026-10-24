"""Calibrated IMU topics. Support code: cannot move the car.

Run on top of bringup (it reads vesc_driver_node's /sensors/imu/raw and
vesc_to_odom_node's /odom), in its own terminal:

    ros2 launch imu_bridge imu_bridge_launch.py
    ros2 launch imu_bridge imu_bridge_launch.py bno_port:=/dev/ttyACM1

Keep the car still for the first startup_bias_s seconds (3 s by default).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

OVERRIDES = ('bno_port', 'primary', 'racer_imu_root')


def _node(context, *args, **kwargs):
    config = os.path.join(
        get_package_share_directory('imu_bridge'), 'config', 'imu_bridge.yaml')
    # Only arguments actually given override the YAML, and they come AFTER
    # the params file in the list -- later entries win, so they take effect.
    overrides = {}
    for name in OVERRIDES:
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    return [Node(
        package='imu_bridge',
        executable='imu_bridge_node',
        name='imu_bridge_node',
        output='screen',
        parameters=[config, overrides],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bno_port', default_value='',
            description="BNO085 microcontroller port, e.g. /dev/ttyACM1. "
                        "Empty = the YAML's value."),
        DeclareLaunchArgument(
            'primary', default_value='',
            description="auto | vesc | bno: which IMU /imu/data carries. "
                        "Empty = the YAML's value."),
        DeclareLaunchArgument(
            'racer_imu_root', default_value='',
            description="Where imu-calibration is cloned. Empty = the YAML's value."),
        OpaqueFunction(function=_node),
    ])
