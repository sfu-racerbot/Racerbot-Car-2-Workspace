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
import sys
from pathlib import Path
import threading
from time import strftime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from race_diagnostics.auto_record import recorder_node_names
from race_diagnostics.recording import bag_record_command, DEFAULT_TOPICS
from race_diagnostics.run_metadata import (
    find_repo_root, imu_calibration_files, team_repos, write_packages,
    write_run_metadata, write_submodules)
import yaml


# `ros2 launch` itself can hang at interpreter exit, in threading._shutdown, with
# all its children gone (3 of 14 test runs, 2026-10-31, after auto_record was
# killed and nothing was left to stop it). Longer than launch's own escalation
# for a stuck child, SIGINT -> 5 s -> SIGTERM -> 10 s -> SIGKILL, so it never
# orphans one.
LAUNCH_EXIT_AFTER_SEC = 18.0


def _exit_even_if_launch_hangs(event, context):
    timer = threading.Timer(LAUNCH_EXIT_AFTER_SEC, os._exit, args=(0,))
    timer.daemon = True
    timer.start()


def _setup(context, *args, **kwargs):
    run_dir = LaunchConfiguration('run_directory').perform(context).strip()
    if run_dir:
        run_dir = os.path.expanduser(run_dir)
    else:
        parent = os.path.expanduser(
            LaunchConfiguration('output_directory').perform(context))
        run_dir = os.path.join(parent, strftime('%Y%m%d-%H%M%S'))
    os.makedirs(run_dir, exist_ok=True)
    # Bringup's auto_record passes a prefix so it can tell the nodes of its
    # own recording from someone else's recorder (race_diagnostics/
    # auto_record.py). Empty for a recording started by hand.
    prefix = LaunchConfiguration('name_prefix').perform(context).strip()
    diag_name, snapshot_name, bag_name = recorder_node_names(
        prefix)

    # What the run was made with -- code, calibration, parameters -- written
    # into <run>/metadata/ (race_diagnostics/run_metadata.py). Never fatal.
    racer_imu_root = os.path.expanduser(
        LaunchConfiguration('racer_imu_root', default='~/imu-calibration').perform(context))
    try:
        repos = {'racerbot-ws': find_repo_root(os.path.realpath(__file__)),
                 'imu-calibration': racer_imu_root}
        # Team A's (and B's) checked-out code, so a run of their algorithm
        # records exactly which commit and edits ran.
        repos.update(team_repos(repos['racerbot-ws']))
        meta = write_run_metadata(
            run_dir, repos, imu_calibration_files(racer_imu_root))
    except Exception as e:  # noqa: BLE001 -- a metadata failure must not stop a run
        repos = {}
        meta = os.path.join(run_dir, 'metadata')
        os.makedirs(meta, exist_ok=True)
        with open(os.path.join(meta, 'ERROR.txt'), 'w') as handle:
            handle.write(f'metadata not written: {e!r}\n')

    # The installed apt package versions (slam_toolbox's release -- apt,
    # so git state cannot say which one ran) and each submodule's
    # checked-out commit next to what the workspace records for it
    # (team A's pin moves often). Two separate tries: one failing must
    # not skip the other, and neither may stop a run.
    meta = Path(meta)
    try:
        write_packages(meta)
    except Exception as e:  # noqa: BLE001 -- must not stop a run
        try:
            with open(meta / 'ERROR.txt', 'a') as handle:
                handle.write(f'metadata packages.txt not written: {e!r}\n')
        except OSError as log_error:
            print(f'metadata packages.txt not written: {e!r}; error log: {log_error}', file=sys.stderr)
    try:
        write_submodules(meta, repos.get('racerbot-ws'))
    except Exception as e:  # noqa: BLE001 -- must not stop a run
        try:
            with open(meta / 'ERROR.txt', 'a') as handle:
                handle.write(f'metadata submodules.txt not written: {e!r}\n')
        except OSError as log_error:
            print(f'metadata submodules.txt not written: {e!r}; error log: {log_error}', file=sys.stderr)

    topics = LaunchConfiguration('topics').perform(context).split()
    config = os.path.join(
        get_package_share_directory('race_diagnostics'),
        'config', 'race_diagnostics.yaml')
    # Passed as values rather than as the file: the file is keyed by the
    # name race_diag_node, and would not reach a prefixed node.
    with open(config) as handle:
        diag_params = yaml.safe_load(handle)['race_diag_node']['ros__parameters']

    if LaunchConfiguration('run_directory').perform(context).strip():
        banner = LogInfo(msg=f'Recording run to: {run_dir}')
    else:
        banner = LogInfo(msg=f'\n{"=" * 72}\nRecording run to: {run_dir}\n'
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
            f'  ros2 run race_diagnostics summarize_run {run_dir}\n{"=" * 72}')

    actions = [
        banner,
        Node(
            package='race_diagnostics',
            executable='race_diag_node',
            name=diag_name,
            output='screen',
            parameters=[diag_params, {'output_directory': run_dir}],
        ),
        Node(
            package='race_diagnostics',
            executable='snapshot_params',
            name=snapshot_name,
            output='screen',
            arguments=['--output', os.path.join(str(meta), 'params')],
        ),
    ]

    if LaunchConfiguration('record_bag').perform(context).lower() in ('true', '1'):
        actions.append(ExecuteProcess(
            cmd=bag_record_command(os.path.join(run_dir, 'bag'), bag_name, topics),
            output='screen',
        ))
    if LaunchConfiguration('system_log', default='true').perform(context).lower() in ('true', '1'):
        # Not `ros2 run`: launch signals only the process it started, and that
        # wrapper swallows SIGINT, expecting a terminal to have sent it to its
        # child too. Every recording then waited out launch's 5 s escalation.
        actions.append(ExecuteProcess(
            cmd=[sys.executable, '-m', 'race_diagnostics.system_log',
                 '--output', os.path.join(run_dir, 'system')],
            name=f'{prefix}system_log',
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        RegisterEventHandler(OnShutdown(on_shutdown=_exit_even_if_launch_hangs)),
        DeclareLaunchArgument(
            'output_directory', default_value='~/racerbot-ws/data/racerbot_runs',
            description='Parent directory; one timestamped subdirectory per run.'),
        DeclareLaunchArgument(
            'run_directory', default_value='',
            description='Record into exactly this directory (bringup\'s '
                        'auto_record uses it). Empty: a new timestamped '
                        'subdirectory of output_directory.'),
        DeclareLaunchArgument(
            'name_prefix', default_value='',
            description='Prefix for this recording\'s node names. auto_record '
                        'passes one to tell its own recording from anyone '
                        'else\'s. Leave empty.'),
        DeclareLaunchArgument(
            'record_bag', default_value='true',
            description='Record a rosbag. Set false to capture only the probe stream.'),
        DeclareLaunchArgument(
            'system_log', default_value='true',
            description='Record per-process system resource usage.'),
        DeclareLaunchArgument(
            'topics', default_value=' '.join(DEFAULT_TOPICS),
            description='Space-separated topics to bag. Drop /scan to shrink ~10x.'),
        OpaqueFunction(function=_setup),
    ])
