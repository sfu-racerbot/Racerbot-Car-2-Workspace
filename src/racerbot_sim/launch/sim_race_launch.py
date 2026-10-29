"""Launch two real controllers, the simulator, referee, and recorder."""

from __future__ import annotations

import os
import importlib.util
import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

from racerbot_sim import tracks


def _params(path):
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)['gap_follow_node']['ros__parameters']


def _setup(context):
    opponent = LaunchConfiguration('opponent').perform(context)
    if opponent not in ('ours', 'team_a'):
        raise RuntimeError(f"unknown opponent:={opponent!r}; expected ours or team_a")
    track = LaunchConfiguration('track').perform(context)
    if track in tracks.LAYOUTS:
        track_directory = '~/.ros/racerbot_sim/tracks'
    else:
        here = Path(__file__).resolve()
        try:
            from race_diagnostics.run_metadata import find_repo_root
            root = find_repo_root(here)
        except ImportError:
            root = None
        candidate = root / 'tools' / 'f1tenth_sim' / 'maps' if root else None
        if candidate is None or not (candidate / f'{track}_map.yaml').is_file() or not (candidate / f'{track}_centerline.csv').is_file():
            raise RuntimeError(f"track '{track}' was not found in {candidate or '<no repository root>'}")
        track_directory = str(candidate)
    sim_share = get_package_share_directory('racerbot_sim')
    gap_share = get_package_share_directory('gap_follow')
    default_yaml = os.path.join(gap_share, 'config', 'gap_follow.yaml')
    car1_yaml = LaunchConfiguration('car1_params').perform(context) or default_yaml
    offsets = ('-3.0,0.0' if LaunchConfiguration('swap_starts').perform(context).lower() == 'true'
               else '0.0,-3.0')
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'sim_bringup_launch.py')),
        launch_arguments={
            'track': track, 'track_directory': track_directory,
            'seed': LaunchConfiguration('seed'), 'controlled_cars': '2',
            'controlled_start_offsets_m': offsets,
            'hold_deadman': LaunchConfiguration('car1_hold_deadman'),
            'car2_hold_deadman': LaunchConfiguration('car2_hold_deadman'),
            'drive_topic_prefix': LaunchConfiguration('drive_topic_prefix'),
        }.items())
    car1 = Node(package='gap_follow', executable='gap_follow_node', name='gap_follow_node',
                parameters=[car1_yaml, {'drive_topic': [LaunchConfiguration('drive_topic_prefix'), '/drive']}],
                output='screen')
    if opponent == 'ours':
        car2_yaml = LaunchConfiguration('car2_params').perform(context) or default_yaml
        car2 = _params(car2_yaml)
        car2.update(scan_topic='/car2/scan', odom_topic='/car2/odom',
                    drive_topic=LaunchConfiguration('drive_topic_prefix').perform(context) + '/car2/drive',
                    joy_topic='/car2/joy', intent_topic='/car2/drive_intent',
                    imu_topic='/car2/imu/data', imu_status_topic='/car2/imu/status')
        car2_node = Node(package='gap_follow', executable='gap_follow_node', name='gap_follow_node',
                         namespace='car2', parameters=[car2], output='screen')
    else:
        car2 = {'controller': 'disparity_extender', 'safety': 'safety_node',
                'parameters': 'upstream launch defaults'}
        includes = []
        for package, launch_file in (('disparity_extender', 'disparity_extender_launch.py'),
                                     ('safety_node', 'launch_safety.py')):
            try:
                share = get_package_share_directory(package)
            except PackageNotFoundError as exc:
                raise RuntimeError(f'opponent:=team_a requires installed package {package}') from exc
            includes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, 'launch', launch_file))))
        car2_node = GroupAction([
            PushRosNamespace('car2'),
            *(SetRemap(src=src, dst=dst) for src, dst in (
                ('/joy', '/car2/joy'), ('/scan', '/car2/scan'),
                ('/ego_racecar/odom', '/car2/odom'), ('/safety/brake', '/car2/safety/brake'),
                ('/drive', LaunchConfiguration('drive_topic_prefix').perform(context) + '/car2/drive'),
                ('/car2/drive', LaunchConfiguration('drive_topic_prefix').perform(context) + '/car2/drive'))),
            *includes,
        ])
    referee = Node(package='racerbot_sim', executable='race_referee_node', name='race_referee_node',
                   parameters=[{'track': track, 'track_directory': track_directory,
                                 'laps': ParameterValue(LaunchConfiguration('laps'), value_type=float),
                                 'controlled_cars': 2,
                                 'reverse': LaunchConfiguration('reverse'),
                                 'timeout_s': ParameterValue(LaunchConfiguration('timeout_s'), value_type=float),
                                 'control_parameters_json': ParameterValue(
                                     json.dumps({'car1': _params(car1_yaml), 'car2': car2}), value_type=str),
                                 'output_dir': LaunchConfiguration('output_dir')}], output='screen')
    actions = [bringup, car1, car2_node, referee]
    if LaunchConfiguration('record').perform(context).lower() == 'true':
        record_path = os.path.join(get_package_share_directory('race_diagnostics'),
                                   'launch', 'record_run.py')
        spec = importlib.util.spec_from_file_location('race_record_run', record_path)
        record_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(record_module)
        extra_topics = ('/car2/scan /car2/odom /car2/drive /car2/ackermann_cmd /car2/joy '
                  '/car2/drive_intent /car2/teleop /car2/safety/brake /sim/ground_truth_pose '
                   '/sim/car2/ground_truth_pose /sim/status /sim/race_result')
        topics = ' '.join(dict.fromkeys(record_module.DEFAULT_TOPICS + extra_topics.split()))
        recorder = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(record_path),
            launch_arguments={'topics': topics, 'run_directory': LaunchConfiguration('record_dir')}.items())
        actions.append(recorder)
    actions.append(RegisterEventHandler(OnProcessExit(target_action=referee, on_exit=[Shutdown()])))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('drive_topic_prefix', default_value=''),
        DeclareLaunchArgument('track', default_value='indoor_oval'),
        DeclareLaunchArgument('opponent', default_value='ours'),
        DeclareLaunchArgument('car1_params', default_value=''),
        DeclareLaunchArgument('car2_params', default_value=''),
        DeclareLaunchArgument('laps', default_value='3.0'),
        DeclareLaunchArgument('swap_starts', default_value='false'),
        DeclareLaunchArgument('seed', default_value='12345'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('record_dir', default_value=''),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('reverse', default_value='false'),
        DeclareLaunchArgument('car1_hold_deadman', default_value='true'),
        DeclareLaunchArgument('car2_hold_deadman', default_value='true'),
        DeclareLaunchArgument('timeout_s', default_value='300.0'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
