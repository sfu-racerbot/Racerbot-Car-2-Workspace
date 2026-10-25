"""Map a course with gap_follow, save everything, and stop -- no racing.

Normally started through the wrapper, which also prints the race commands
once everything has shut down:

    ros2 run racerbot_launch map_course

It never starts the hardware: ``bringup_launch.py`` must already be running
in its own terminal (``sim_bringup_launch.py`` in simulation), the same
two-layer pattern as every other control layer here (docs/architecture.md).
Starting a second copy of the drivers beside an existing bringup makes the
two fight over the VESC serial port and the LiDAR's single connection.

What runs: slam_toolbox mapping, gap_follow driving (its commands go to auto_map_race_node, never
straight to /drive), the run recorder, and auto_map_race_node in
finish_after_mapping mode. The supervisor forwards gap_follow's commands
only while LB is held, counts laps from SLAM's pose, and after ``laps``
(default 3) closed laps saves the map, pose graph and racing line to
``~/.ros/racerbot_auto/<timestamp>/``, writes commands.txt there, stops the
car, and exits. Its exit shuts this whole launch down.

Differences from auto_map_race_launch.py: no pure_pursuit, no particle
filter, no racing handover, and every requested lap is driven (that launch
races as soon as lap 2 is clean).

The LB deadman applies throughout: gap_follow_node checks it and so does
auto_map_race_node. Releasing LB stops the car; holding it again resumes
mapping where it left off.

IMU (``imu:=true``; **default false on car 2**, which has no BNO085 yet and
no ~/imu-calibration checkout, without which imu_bridge_node refuses to
start -- docs/car1-vs-car2.md; car 1 defaults to true): imu_bridge's
vehicle_state_launch.py also runs -- the BNO085 bridge, imu_bridge and
vehicle_state_node, which
publish /odom/fused (wheel speed fused with the BNO's accelerometer) and
/steering/measured (the angle the car is really turning at). Support code,
nothing there can move the car. Keep the car still for the first 3 s.

``fused_speed:=true`` (default false) gives gap_follow /odom/fused in place
of /odom, for its TTC braking, acceleration ramp and stale-odometry stop. It
is never more than vehicle_state's max_divergence (0.3 m/s) from the wheel
speed, and if vehicle_state dies /odom/fused goes silent and gap_follow stops
the car as it would for a silent /odom. SLAM, the odom -> base_link
transform and auto_map_race_node stay on /odom either way. Compare runs with
``record_bag:=true`` (the default): the bag then holds /odom, /odom/fused and
/steering/measured side by side. In the simulator pass ``imu:=false``
(the map_course wrapper does it for you): there is no IMU there.

``gyro_odom:=true`` (default false) gives slam_toolbox gyro odometry: its
odom_frame becomes odom_imu, and vehicle_state publishes odom_imu -> odom
so that odom_imu -> base_link is vesc_to_odom's odometry with the gyro's
heading. vesc_to_odom keeps odom -> base_link, so nothing else changes:
gap_follow drives on /scan as before, the lap counter still counts turning
in `odom`. Only the map and the map-frame pose differ. /odom/imu is
recorded either way, so every run can be replayed through SLAM both ways.
"""

import os

from ament_index_python.packages import get_package_share_directory
from gap_follow.speed_overrides import mapping_speed_overrides
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FUSED_ODOM = '/odom/fused'   # vehicle_state.yaml's fused_odom_topic
IMU_ODOM_FRAME = 'odom_imu'  # vehicle_state.yaml's imu_odom_frame


def _flag(name, value):
    """A launch boolean, the way IfCondition reads one."""
    text = str(value).strip().lower()
    if text in ('true', '1'):
        return True
    if text in ('false', '0'):
        return False
    raise RuntimeError(f"{name}:={value!r}: expected 'true' or 'false'")


def odom_overrides(imu, fused_speed):
    """gap_follow's odom_topic for this run: /odom/fused only when asked for,
    and only with the IMU stack running to publish it."""
    if not _flag('fused_speed', fused_speed):
        return {}
    if not _flag('imu', imu):
        raise RuntimeError(
            'fused_speed:=true needs imu:=true: /odom/fused comes from the IMU '
            "stack, and without it gap_follow would see no odometry at all.")
    return {'odom_topic': FUSED_ODOM}


def gyro_odom_settings(imu, gyro_odom):
    """(slam_toolbox odom_frame, vehicle_state publish_odom_tf) for this run.

    Always set together: the transform is only right with SLAM on odom_imu,
    and SLAM on odom_imu has no odometry without it."""
    if not _flag('gyro_odom', gyro_odom):
        return 'odom', 'false'
    if not _flag('imu', imu):
        raise RuntimeError(
            'gyro_odom:=true needs imu:=true: odom_imu comes from the IMU stack, '
            'and without it slam_toolbox would have no odometry at all.')
    return IMU_ODOM_FRAME, 'true'


def _wiring(context):
    frame, publish_tf = gyro_odom_settings(
        LaunchConfiguration('imu').perform(context),
        LaunchConfiguration('gyro_odom').perform(context))
    # Launch configurations are shared with included files: slam_launch.py
    # reads slam_odom_frame, vehicle_state_launch.py publish_odom_tf.
    return [SetLaunchConfiguration('slam_odom_frame', frame),
            SetLaunchConfiguration('publish_odom_tf', publish_tf)]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'laps', default_value='3',
            description='Closed laps to drive before saving. The first is '
                        'discovery; the last is kept as the racing line. '
                        'Up to two more are driven if the last lap fails the '
                        'quality check.'),
        DeclareLaunchArgument(
            'mapping_max_speed', default_value='',
            description='Optional speed cap while mapping (m/s). Empty: '
                        'gap_follow.yaml governs. Use 1.5 on a course nobody '
                        'has driven yet.'),
        DeclareLaunchArgument(
            'mapping_min_speed', default_value='',
            description='Optional speed floor. Empty: gap_follow.yaml governs.'),
        DeclareLaunchArgument(
            'output_directory', default_value='~/.ros/racerbot_auto',
            description='Parent directory of the run directory.'),
        DeclareLaunchArgument(
            'diagnostics', default_value='true',
            description='Run race_diagnostics alongside (subscribe-only).'),
        DeclareLaunchArgument(
            'record_bag', default_value='true',
            description='Bag the run as well (a few hundred MB per run: /scan '
                        'dominates). Needed to replay the run through SLAM offline.'),
        DeclareLaunchArgument(
            # Car 2: false until its BNO085 is mounted and ~/imu-calibration
            # is cloned (docs/car1-vs-car2.md#imu). Car 1 ships true.
            'imu', default_value='false',
            description='Run the IMU stack (BNO085 bridge, imu_bridge, '
                        'vehicle_state): records /odom/fused and '
                        '/steering/measured. Off by default on car 2 (no '
                        'BNO085 or imu-calibration yet); false in the simulator.'),
        DeclareLaunchArgument(
            'gyro_odom', default_value='false',
            description='true: slam_toolbox maps on gyro odometry (odom_imu) instead of '
                        "vesc_to_odom's steering-model odometry. Needs imu:=true."),
        DeclareLaunchArgument(
            'fused_speed', default_value='false',
            description='true: gap_follow reads /odom/fused (IMU-fused speed) '
                        'instead of /odom. Needs imu:=true.'),
        DeclareLaunchArgument(
            'supervisor_config',
            default_value=os.path.join(
                get_package_share_directory('pure_pursuit'), 'config',
                'auto_map_race.yaml'),
            description='auto_map_race_node parameter file (racing-line '
                        'profile limits live here).'),
    ]

    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('race_diagnostics'), 'launch', 'record_run.py')),
        # Its own folder, given explicitly: launch arguments are shared with
        # included files, and this launch's output_directory (the map run
        # folder) would otherwise become the recorder's too.
        launch_arguments={'record_bag': LaunchConfiguration('record_bag'),
                          'output_directory': '~/.ros/racerbot_runs'}.items(),
        condition=IfCondition(LaunchConfiguration('diagnostics')),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('racerbot_launch'), 'launch', 'slam_launch.py')),
    )
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('imu_bridge'), 'launch', 'vehicle_state_launch.py')),
        condition=IfCondition(LaunchConfiguration('imu')),
    )
    gap_config = os.path.join(
        get_package_share_directory('gap_follow'), 'config', 'gap_follow.yaml')

    def nodes(context):
        # Same speed-override rule as auto_map_race_launch.py: a cap moves
        # corner_speed/corner_speed_wide with it, or gap_follow refuses to
        # start (gap_follow/speed_overrides.py).
        speeds = mapping_speed_overrides(
            gap_config,
            LaunchConfiguration('mapping_max_speed').perform(context),
            LaunchConfiguration('mapping_min_speed').perform(context))
        odom = odom_overrides(LaunchConfiguration('imu').perform(context),
                              LaunchConfiguration('fused_speed').perform(context))
        laps = int(LaunchConfiguration('laps').perform(context))
        if laps < 2:
            raise RuntimeError(
                f'laps:={laps}: at least 2 are needed -- the first lap only '
                'discovers the course and is never used as the racing line.')
        gap_follow = Node(
            package='gap_follow', executable='gap_follow_node', name='gap_follow_node',
            output='screen',
            parameters=[gap_config,
                        dict({'drive_topic': '/auto_map/drive'}, **speeds, **odom)],
        )
        supervisor = Node(
            package='pure_pursuit', executable='auto_map_race_node',
            name='auto_map_race_node', output='screen',
            parameters=[LaunchConfiguration('supervisor_config'), {
                'mapping_laps': laps,
                'max_mapping_laps': laps + 2,
                'finish_after_mapping': True,
                'output_directory': LaunchConfiguration('output_directory'),
            }],
        )
        # The supervisor exiting -- finished, crashed or Ctrl+C -- ends the
        # run: nothing else here should keep the car's stack up afterwards.
        shutdown = RegisterEventHandler(OnProcessExit(
            target_action=supervisor,
            on_exit=[EmitEvent(event=Shutdown(reason='mapping run finished'))]))
        return [gap_follow, supervisor, shutdown]

    return LaunchDescription(args + [OpaqueFunction(function=_wiring), diagnostics, slam, imu,
                                     OpaqueFunction(function=nodes)])
