"""robot_localization EKF in shadow mode: publishes /odometry/filtered, no TF.

Fuses /imu_bridge/wheel_odom (vx) and /imu/data (yaw rate), so start
imu_bridge_launch.py first. Nothing drives off its output; it exists to be
recorded and compared against SLAM / the particle filter. See
config/ekf_shadow.yaml for why it must not publish odom -> base_link.

    ros2 launch imu_bridge imu_ekf_shadow_launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('imu_bridge'), 'config', 'ekf_shadow.yaml')
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config],
        ),
    ])
