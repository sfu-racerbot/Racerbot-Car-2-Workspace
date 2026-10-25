"""Record one run: probe + rosbag, into a single timestamped directory.

Run this in its own terminal ALONGSIDE the driving stack, exactly like the
web dashboard -- it is a third layer on top of the two-tier launch pattern
(see docs/architecture.md), subscribe-only, and starts nothing that can
move the car.

    ros2 launch race_diagnostics record_run.py

It prints the run directory it created. The one thing it cannot capture by
itself is the driving terminal's own stdout, because that belongs to
another process -- so start the driving stack with `| tee <dir>/launch.log`
as printed. See docs/run-diagnostics.md.
"""

import os
from time import strftime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from race_diagnostics.run_metadata import (
    find_repo_root, imu_calibration_files, write_run_metadata)

# Everything needed to reconstruct a run offline. /scan dominates the size
# (~10x everything else combined) but is also the only way to answer "what
# did the car actually see" after the fact, which is precisely the question
# that could not be answered about the 2026-07-27 collision.
#
# /parameter_events is every parameter change on every node, whoever made
# it: the web dashboard's live tuning, Lichtblick through foxglove_bridge,
# or `ros2 param set` in a terminal. Without it a bag can show the car
# driving differently halfway through a run with no record of why. It is a
# few small messages per change, so it costs nothing next to /scan.
DEFAULT_TOPICS = [
    '/scan', '/odom', '/slam_pose', '/drive', '/ackermann_cmd',
    '/teleop', '/tf', '/tf_static', '/map', '/joy',
    '/auto_map/drive', '/auto_race/drive',
    '/parameter_events',
    # The VESC's IMU (gyro + accelerometer). Nothing drives on it yet; bagged
    # so recorded runs can show what it would do for odometry and SLAM
    # (2026-10-23: odometry's steering-based turn rate was off by 0.44 rad/s
    # RMS on the 2026-10-19 run).
    '/sensors/imu/raw',
    # With imu_bridge (src/imu_bridge/README.md) each run is also an IMU
    # dataset: calibrated IMU next to the raw one above, so a calibration can
    # be redone offline; the VESC state and servo command that explain wheel
    # speed and yaw; the shadow EKF next to the particle filter's pose it is
    # judged against. All small next to /scan; unpublished topics record nothing.
    '/sensors/core', '/sensors/servo_position_command',
    '/imu/data', '/imu/vesc/data', '/imu/bno/raw', '/imu/bno/data',
    '/imu_bridge/wheel_odom', '/odometry/filtered', '/pf/viz/inferred_pose',
    # vehicle_state_node (imu_bridge): IMU-fused speed next to /odom's wheel
    # speed, and the steering angle the car really turned at next to the
    # command on /drive -- the two comparisons a fused_speed:=true run is
    # judged on (map_course_launch.py).
    '/odom/fused', '/steering/measured',
    # Gyro odometry (imu_bridge vehicle_state_node), recorded on every run
    # whether SLAM uses it or not (map_course gyro_odom:=), so each bag can be
    # replayed through slam_toolbox both ways. SLAM's own outputs are what a
    # replay is judged against: its pose with covariance, its pose graph
    # (loop closures show here), and the map's metadata.
    '/odom/imu', '/pose', '/slam_toolbox/graph_visualization', '/map_metadata',
    # Every node's log lines, timestamped on the same clock as the data --
    # "why did it stop" and "what was it doing" in one file -- and the
    # driving nodes' own account of what they were trying to do.
    '/rosout', '/diagnostics', '/drive_intent', '/initialpose',
]


def _setup(context, *args, **kwargs):
    parent = os.path.expanduser(
        LaunchConfiguration('output_directory').perform(context))
    run_dir = os.path.join(parent, strftime('%Y%m%d-%H%M%S'))
    os.makedirs(run_dir, exist_ok=True)

    # What the run was made with -- code, calibration, parameters -- written
    # into <run>/metadata/ (race_diagnostics/run_metadata.py). Never fatal.
    racer_imu_root = os.path.expanduser(
        LaunchConfiguration('racer_imu_root', default='~/imu-calibration').perform(context))
    try:
        meta = write_run_metadata(
            run_dir,
            {'racerbot-ws': find_repo_root(os.path.realpath(__file__)),
             'imu-calibration': racer_imu_root},
            imu_calibration_files(racer_imu_root))
    except Exception as e:  # noqa: BLE001 -- a metadata failure must not stop a run
        meta = os.path.join(run_dir, 'metadata')
        os.makedirs(meta, exist_ok=True)
        with open(os.path.join(meta, 'ERROR.txt'), 'w') as handle:
            handle.write(f'metadata not written: {e!r}\n')

    topics = LaunchConfiguration('topics').perform(context).split()
    config = os.path.join(
        get_package_share_directory('race_diagnostics'),
        'config', 'race_diagnostics.yaml')

    actions = [
        LogInfo(msg=f'\n{"=" * 72}\nRecording run to: {run_dir}\n'
                    f'Start the driving stack in another terminal, ideally WITH tee\n'
                    f'so its terminal output is captured directly:\n\n'
                    f'  ros2 launch racerbot_launch auto_map_race_launch.py \\\n'
                    f'    2>&1 | tee {run_dir}/launch.log\n\n'
                    f'`| tee` is optional, not required: if you forget it, '
                    f'summarize_run\n'
                    f'reconstructs an equivalent log from each node\'s own '
                    f'~/.ros/log/\n'
                    f'file instead -- see docs/run-diagnostics.md.\n\n'
                    f'Afterwards:\n'
                    f'  ros2 run race_diagnostics summarize_run {run_dir}\n{"=" * 72}'),
        Node(
            package='race_diagnostics',
            executable='race_diag_node',
            name='race_diag_node',
            output='screen',
            parameters=[config, {'output_directory': run_dir}],
        ),
        Node(
            package='race_diagnostics',
            executable='snapshot_params',
            name='snapshot_params',
            output='screen',
            arguments=['--output', os.path.join(str(meta), 'params')],
        ),
    ]

    if LaunchConfiguration('record_bag').perform(context).lower() in ('true', '1'):
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(run_dir, 'bag')] + topics,
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_directory', default_value='~/.ros/racerbot_runs',
            description='Parent directory; one timestamped subdirectory per run.'),
        DeclareLaunchArgument(
            'record_bag', default_value='true',
            description='Record a rosbag. Set false to capture only the probe stream.'),
        DeclareLaunchArgument(
            'topics', default_value=' '.join(DEFAULT_TOPICS),
            description='Space-separated topics to bag. Drop /scan to shrink ~10x.'),
        OpaqueFunction(function=_setup),
    ])
