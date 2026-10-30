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
``~/racerbot-ws/data/racerbot_auto/<timestamp>/``, writes commands.txt there, stops the
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
/steering/measured side by side. In the simulator the map_course wrapper
adds ``imu_hardware:=false``: vehicle_state_node uses the simulated IMU,
without starting the real sensor readers. Direct launches default to
``imu_hardware:=auto``, detecting the running simulator; true/false select
real/simulated inputs explicitly.

``gyro_odom:=true`` (default false) gives slam_toolbox gyro odometry: its
odom_frame becomes odom_imu, and vehicle_state publishes odom_imu -> odom
so that odom_imu -> base_link is vesc_to_odom's odometry with the gyro's
heading. vesc_to_odom keeps odom -> base_link, so nothing else changes:
gap_follow drives on /scan as before, the lap counter still counts turning
in `odom`. Only the map and the map-frame pose differ. SLAM also loads
slam_gyro.yaml, a tighter loop-closure search: the wider one in
slam_tracking.yaml let a loop closure land on a neighbouring stretch of
track. /odom/imu is recorded either way, so every run can be replayed
through SLAM both ways.
"""

import os

from ament_index_python.packages import get_package_share_directory
from gap_follow.speed_overrides import mapping_speed_overrides
from gap_follow.launch_imu import select_imu_hardware
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FUSED_ODOM = '/odom/fused'   # vehicle_state.yaml's fused_odom_topic
IMU_ODOM_FRAME = 'odom_imu'  # vehicle_state.yaml's imu_odom_frame
SLAM_GYRO_PROFILE = 'slam_gyro.yaml'  # racerbot_launch/config, layered over slam_tracking.yaml


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


def slam_profile_for(gyro_odom):
    """The slam_toolbox profile this run layers over slam_tracking.yaml, or None.

    Only gyro odometry gets one. slam_tracking.yaml's 8 m loop-closure search
    was set for steering odometry, which drifts about 4 m a lap and needs it;
    with the gyro, that same search can close onto a neighbouring stretch of
    track.
    """
    if not _flag('gyro_odom', gyro_odom):
        return None
    return os.path.join(
        get_package_share_directory('racerbot_launch'), 'config', SLAM_GYRO_PROFILE)


def _wiring(context, imu_probe=None):
    imu = _flag('imu', LaunchConfiguration('imu').perform(context))
    all_imu = _flag('imu_all', LaunchConfiguration('imu_all').perform(context))
    lap_turns = _flag('imu_lap_turns', LaunchConfiguration('imu_lap_turns').perform(context))
    gyro_odom = LaunchConfiguration('gyro_odom').perform(context)
    gyro = _flag('gyro_odom', gyro_odom)
    fused = _flag('fused_speed', LaunchConfiguration('fused_speed').perform(context))
    if not imu:
        for name, enabled in (('imu_all', all_imu), ('imu_lap_turns', lap_turns),
                              ('gyro_odom', gyro), ('fused_speed', fused)):
            if enabled:
                raise RuntimeError(f'{name}:=true needs imu:=true; pass imu:=true or {name}:=false')
    if lap_turns and not gyro:
        raise RuntimeError('imu_lap_turns:=true needs gyro_odom:=true; pass gyro_odom:=true '
                           'or imu_lap_turns:=false')
    frame, publish_tf = gyro_odom_settings(
        LaunchConfiguration('imu').perform(context), gyro_odom)
    actions = [SetLaunchConfiguration('slam_odom_frame', frame),
               SetLaunchConfiguration('publish_odom_tf', publish_tf)]
    if _flag('imu', LaunchConfiguration('imu').perform(context)):
        hardware, _ = select_imu_hardware(
            LaunchConfiguration('imu_hardware').perform(context), probe=imu_probe)
        actions.append(SetLaunchConfiguration('imu_selected_hardware', hardware))
    profile = slam_profile_for(gyro_odom)
    if profile is not None:
        actions.append(SetLaunchConfiguration('slam_profile_file', profile))
    # Launch configurations are shared with included files: slam_launch.py
    # reads slam_odom_frame and slam_profile_file, vehicle_state_launch.py
    # publish_odom_tf.
    return actions


def generate_launch_description(imu_probe=None):
    args = [
        DeclareLaunchArgument(
            'opponent_awareness', default_value='false',
            description='Mapping is one car; opponent awareness is for racing. '
                        'Set true to enable it while mapping.'),
        DeclareLaunchArgument(
            'laps', default_value='3',
            description='Closed laps to drive before saving. The first is '
                        'discovery; the last is kept as the racing line. '
                        'Up to two more are driven if the last lap fails the '
                        'quality check.'),
        # Gyro SLAM (slam_gyro.yaml) was validated by replay only up to about
        # 3.3 m/s. Raise this default only after a faster recorded gyro run has
        # a clean slam_report.
        DeclareLaunchArgument(
            'mapping_max_speed', default_value='3.5',
            description='Mapping speed cap in m/s (default 3.5). Gyro SLAM was '
                        'replay-validated only to about 3.3 m/s; raise the default '
                        'only after a faster recorded gyro run has a clean slam_report. '
                        'Use 1.5 on a new course; empty uses gap_follow.yaml.'),
        DeclareLaunchArgument(
            'mapping_min_speed', default_value='',
            description='Optional speed floor. Empty: gap_follow.yaml governs.'),
        DeclareLaunchArgument(
            'output_directory', default_value='~/racerbot-ws/data/racerbot_auto',
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
                        '/steering/measured, including from simulated IMUs. Off '
                        'by default on car 2 (BNO085 not calibrated yet).'),
        DeclareLaunchArgument(
            'imu_hardware', default_value='auto',
            description='IMU source: auto uses simulated topics when gym_bridge_node is running, '
                        'otherwise real hardware. true/false explicitly select real/simulated inputs.'),
        DeclareLaunchArgument(
            'imu_all', default_value='false',
            description='Enable gyro_odom, fused_speed and imu_lap_turns unless explicitly set.'),
        DeclareLaunchArgument(
            'gyro_odom', default_value=LaunchConfiguration('imu_all', default='false'),
            description='true: slam_toolbox maps on gyro odometry (odom_imu) instead of '
                        "vesc_to_odom's steering-model odometry, and loads slam_gyro.yaml. "
                        'Needs imu:=true.'),
        DeclareLaunchArgument(
            'fused_speed', default_value=LaunchConfiguration('imu_all', default='false'),
            description='true: gap_follow reads /odom/fused (IMU-fused speed) '
                        'instead of /odom. Needs imu:=true.'),
        DeclareLaunchArgument(
            'imu_lap_turns', default_value=LaunchConfiguration('imu_all', default='false'),
            description='true: count lap turns in odom_imu. Needs gyro_odom:=true.'),
        DeclareLaunchArgument(
            'supervisor_config',
            default_value=os.path.join(
                get_package_share_directory('pure_pursuit'), 'config',
                'auto_map_race.yaml'),
            description='auto_map_race_node parameter file (racing-line '
                        'profile limits live here).'),
    ]

    diagnostics_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('race_diagnostics'), 'launch', 'record_run.py')),
        # Its own folder, given explicitly: launch arguments are shared with
        # included files, and this launch's output_directory (the map run
        # folder) would otherwise become the recorder's too.
        launch_arguments={'record_bag': LaunchConfiguration('record_bag'),
                          'output_directory': '~/racerbot-ws/data/racerbot_runs'}.items(),
        condition=IfCondition(LaunchConfiguration('diagnostics')),
    )
    diagnostics = GroupAction(actions=[diagnostics_include], scoped=True)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('racerbot_launch'), 'launch', 'slam_launch.py')),
    )
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('imu_bridge'), 'launch', 'vehicle_state_launch.py')),
        launch_arguments={'hardware': LaunchConfiguration('imu_selected_hardware')}.items(),
        condition=IfCondition(LaunchConfiguration('imu')),
    )
    gap_config = os.path.join(
        get_package_share_directory('gap_follow'), 'config', 'gap_follow.yaml')

    def nodes(context):
        enabled = LaunchConfiguration('opponent_awareness').perform(context).lower()
        if enabled not in ('true', 'false'):
            raise RuntimeError(f"opponent_awareness:={enabled!r}: use true or false")
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
            parameters=[gap_config, {'enable_opponent_awareness': enabled == 'true'},
                        dict({'drive_topic': '/auto_map/drive'}, **speeds, **odom)],
        )
        lap_frame = ({'odom_frame': IMU_ODOM_FRAME}
                     if _flag('imu_lap_turns', LaunchConfiguration('imu_lap_turns').perform(context))
                     else {})
        supervisor = Node(
            package='pure_pursuit', executable='auto_map_race_node',
            name='auto_map_race_node', output='screen',
            parameters=[LaunchConfiguration('supervisor_config'), {
                'mapping_laps': laps,
                'max_mapping_laps': laps + 2,
                'finish_after_mapping': True,
                'output_directory': LaunchConfiguration('output_directory'),
                **lap_frame,
            }],
        )
        # The supervisor exiting -- finished, crashed or Ctrl+C -- ends the
        # run: nothing else here should keep the car's stack up afterwards.
        shutdown = RegisterEventHandler(OnProcessExit(
            target_action=supervisor,
            on_exit=[EmitEvent(event=Shutdown(reason='mapping run finished'))]))
        return [gap_follow, supervisor, shutdown]

    return LaunchDescription(args + [OpaqueFunction(function=_wiring, kwargs={'imu_probe': imu_probe}),
                                     diagnostics, slam, imu,
                                     OpaqueFunction(function=nodes)])
