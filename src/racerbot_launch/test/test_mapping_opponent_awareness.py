"""Mapping launch parameter wiring; run with racerbot_launch's pytest suite.

No processes or ROS graph are started. Oracle: mapping is one car and the
2026-10-28 decision in CLAUDE.md's gap_follow package row specifies off by
default, with an explicit true override.
"""
import importlib.machinery
import importlib.util
import os
from pathlib import Path
import stat

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCHES = ('map_course_launch.py', 'auto_map_race_launch.py',
            'autonomous_mapping_launch.py')


def _load(path):
    loader = importlib.machinery.SourceFileLoader(path.stem, str(path))
    spec = importlib.util.spec_from_loader(path.stem, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def _node(filename, value=None):
    module = _load(PACKAGE / 'launch' / filename)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = ''.join(
                part.perform(context) for part in action.default_value or [])
    if value is not None:
        context.launch_configurations['opponent_awareness'] = value
    nodes = []
    for action in description.entities:
        if isinstance(action, OpaqueFunction):
            nodes.extend(a for a in action.execute(context) if isinstance(a, Node)
                         and a.node_package == 'gap_follow')
    assert len(nodes) == 1
    return nodes[0], context


def _parameters(node, context):
    result = {}
    files = []
    for entry in node._Node__parameters:
        if isinstance(entry, ParameterFile):
            path = Path(''.join(part.perform(context) for part in entry.param_file))
            files.append(path.name)
            result.update(yaml.safe_load(path.read_text())['gap_follow_node']['ros__parameters'])
        elif isinstance(entry, dict):
            for key, value in entry.items():
                name = ''.join(part.perform(context) for part in key)
                if isinstance(value, tuple):
                    value = yaml.safe_load(''.join(part.perform(context) for part in value))
                result[name] = value
    return result, files


@pytest.mark.parametrize('filename', LAUNCHES)
def test_default_is_off_after_yaml(filename):
    node, context = _node(filename)
    params, files = _parameters(node, context)
    # oracle: spec CLAUDE.md: mapping defaults off despite racing YAML.
    assert files == ['gap_follow.yaml']
    assert params['enable_opponent_awareness'] is False


@pytest.mark.parametrize('filename', LAUNCHES)
def test_true_enables_awareness(filename):
    node, context = _node(filename, 'true')
    params, _ = _parameters(node, context)
    # oracle: spec CLAUDE.md: true enables awareness during mapping.
    assert params['enable_opponent_awareness'] is True


@pytest.mark.parametrize('filename', LAUNCHES)
def test_invalid_value_is_refused(filename):
    # oracle: launch boolean contract: only true/false, with the bad value named.
    with pytest.raises(RuntimeError, match='opponent_awareness.*maybe'):
        _node(filename, 'maybe')


def test_map_course_forwards_opponent_argument(tmp_path, monkeypatch):
    wrapper = _load(PACKAGE / 'scripts' / 'map_course')
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    ros2 = bin_dir / 'ros2'
    args = tmp_path / 'args'
    ros2.write_text(f'#!/bin/sh\nprintf "%s\\n" "$@" > "{args}"\n')
    ros2.chmod(ros2.stat().st_mode | stat.S_IXUSR)
    monkeypatch.setenv('PATH', f'{bin_dir}{os.pathsep}{os.environ["PATH"]}')
    monkeypatch.setattr(wrapper, '_running', lambda name: name == wrapper.SIM_DRIVER)
    monkeypatch.setattr(wrapper, 'LOG_DIRECTORY', str(tmp_path / 'logs'))
    assert wrapper.main(['opponent_awareness:=true']) == 0
    # oracle: scripts/map_course contract: name:=value reaches ros2 launch.
    assert args.read_text().splitlines() == [
        'launch', 'racerbot_launch', 'map_course_launch.py',
        'opponent_awareness:=true', 'imu:=false']
