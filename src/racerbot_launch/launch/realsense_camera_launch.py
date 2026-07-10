import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """RealSense D435i: color + depth + IMU, low-res/low-fps so it doesn't
    compete with the LiDAR/SLAM/localization/pure_pursuit stack already
    running on this Jetson. Pure sensor publisher -- no /drive, no
    /ackermann_cmd, no /commands/motor|servo/* -- so this is support/tooling
    code per docs/adding-your-own-code.md, same category as web_dashboard/
    usb_cam_stream: no LB-deadman check, safe to run alongside anything else
    at any time. See docs/realsense-camera.md.

    Wraps realsense2_camera's own rs_launch.py the same way race_launch.py
    wraps particle_filter's/pure_pursuit's own launch files -- this file
    owns only this car's tuning (as launch arguments) and the
    base_link->camera_link static transform, not the driver itself.

    Pointcloud is deliberately off (not needed yet, costs CPU).
    """
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'rgb_camera.color_profile': '424,240,15',
            'depth_module.depth_profile': '424,240,15',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',   # linear_interpolation -- one combined /camera/camera/imu topic
            'pointcloud.enable': 'false',
            'base_frame_id': 'camera_link',
        }.items(),
    )

    # Mount offset from base_link -- PLACEHOLDER, not yet measured. Same
    # "placeholder, documented follow-up" treatment as web_dashboard.yaml's
    # laser_offset_x/y. Update once the physical mount position is measured
    # (see docs/hardware-reference.md).
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
    )

    return LaunchDescription([realsense_launch, camera_tf])
