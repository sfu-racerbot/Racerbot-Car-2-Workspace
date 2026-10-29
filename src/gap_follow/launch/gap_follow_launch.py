import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from gap_follow.launch_speed import speed_cap_overrides, yaml_parameters


def _flag(name, value):
    """A launch boolean, read the way racerbot_launch's map_course_launch.py reads its own."""
    text = str(value).strip().lower()
    if text in ('true', '1'):
        return True
    if text in ('false', '0'):
        return False
    raise RuntimeError(f"{name}:={value!r}: expected 'true' or 'false'")


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
                    'gap_follow.yaml\'s max_speed (5.0). Refused above 5.0. Below '
                    'the YAML value, min_speed, corner_speed and '
                    'corner_speed_wide are lowered with it; above it, only '
                    'max_speed rises.')

    steering_damping_arg = DeclareLaunchArgument(
        'steering_damping', default_value='false',
        description='true: steer against the car\'s own rotation, using the gyro yaw '
                    'rate in /imu/data. Starts imu_bridge\'s vehicle_state_launch.py '
                    '(the IMU stack). Car 2: false until its BNO085 is mounted and '
                    'calibrated (docs/car1-vs-car2.md#imu); car 1 ships true. '
                    'Simulator-validated only, not yet run on the floor '
                    '(docs/gap-follow.md#steering-damping-gyro).')
    adaptive_grip_arg = DeclareLaunchArgument(
        'adaptive_grip', default_value='false',
        description='true: raise or cut the lateral limit behind the corner speed '
                    'caps from the gyro\'s grip ratio, starting at max_lateral_accel. '
                    'Starts imu_bridge\'s vehicle_state_launch.py (the IMU stack), '
                    'once, even when steering_damping:=true as well. Ships off; '
                    'simulator-validated only, not yet run on the floor '
                    '(docs/gap-follow.md#adaptive-grip-imu).')

    def node(context):
        enabled = LaunchConfiguration('opponent_awareness').perform(context).lower()
        if enabled not in ('true', 'false'):
            raise RuntimeError(
                f"opponent_awareness:={enabled!r}: use true or false")
        damping = _flag('steering_damping',
                        LaunchConfiguration('steering_damping').perform(context))
        grip = _flag('adaptive_grip',
                     LaunchConfiguration('adaptive_grip').perform(context))
        speed = speed_cap_overrides(
            yaml_parameters(gap_follow_config),
            LaunchConfiguration('max_speed').perform(context))
        actions = [Node(
            package='gap_follow',
            executable='gap_follow_node',
            name='gap_follow_node',
            output='screen',
            parameters=[gap_follow_config,
                        {'enable_opponent_awareness': enabled == 'true',
                         'enable_steering_damping': damping,
                         'enable_adaptive_grip': grip},
                        speed],
        )]
        if damping or grip:
            # The IMU stack that publishes /imu/data and /imu/status. The same
            # include map_course_launch.py makes, so do not run both at once.
            # Either gyro feature needs it, and both together still start one.
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('imu_bridge'), 'launch',
                             'vehicle_state_launch.py'))))
        return actions

    return LaunchDescription([opponent_arg, max_speed_arg, steering_damping_arg,
                              adaptive_grip_arg, OpaqueFunction(function=node)])
