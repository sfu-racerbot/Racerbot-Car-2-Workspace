"""
Every f1tenth_stack bringup starts the automatic run recorder, on by
default, off with record:=false, and never fails to start because of it.

Spec: docs/run-diagnostics.md#automatic-recording and docs/git-setup.md
("Addition 6"). The bringup files are vendored (f1tenth_system), so an
upstream sync or a port from car 2 silently drops this; that is what the
first group of tests catches. They read the launch files with ast, no ROS:

    python3 -m pytest src/race_diagnostics/test/test_bringup_auto_record.py -v

The last two build the real launch description and evaluate its condition,
so they need ROS sourced and the workspace built (skipped otherwise).
"""
import ast
import os
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[2]
STACK = SRC / 'f1tenth_system' / 'f1tenth_stack' / 'launch'
BRINGUPS = [STACK / 'bringup_launch.py', STACK / 'no_lidar_bringup_launch.py',
            STACK / 'sick_bringup_launch.py']
PKG = Path(__file__).resolve().parents[1]


def _calls(tree, name):
    return [c for c in ast.walk(tree)
            if isinstance(c, ast.Call) and getattr(c.func, 'id', None) == name]


def _kw(call, name):
    return next((k.value for k in call.keywords if k.arg == name), None)


@pytest.mark.parametrize('path', BRINGUPS, ids=lambda p: p.name)
def test_bringup_declares_record_on_by_default(path):
    tree = ast.parse(path.read_text())
    record = [c for c in _calls(tree, 'DeclareLaunchArgument')
              if c.args and isinstance(c.args[0], ast.Constant) and c.args[0].value == 'record']
    assert len(record) == 1, f'{path.name}: no record launch argument'
    assert ast.literal_eval(_kw(record[0], 'default_value')) == 'true'


@pytest.mark.parametrize('path', BRINGUPS, ids=lambda p: p.name)
def test_bringup_includes_auto_record_only_when_record_is_true(path):
    source = path.read_text()
    tree = ast.parse(source)
    includes = _calls(tree, 'IncludeLaunchDescription')
    assert len(includes) == 1, f'{path.name}: expected one include, the recorder'
    condition = _kw(includes[0], 'condition')
    assert ast.unparse(condition) == "IfCondition(LaunchConfiguration('record'))"
    assert "'launch', 'auto_record_launch.py')" in source
    assert "get_package_share_directory('race_diagnostics')" in source


@pytest.mark.parametrize('path', BRINGUPS, ids=lambda p: p.name)
def test_a_missing_recorder_package_cannot_stop_bringup(path):
    """The share-directory lookup sits in a try whose PackageNotFoundError
    handler logs and carries on."""
    tree = ast.parse(path.read_text())
    guarded = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Try):
            continue
        looks_up = any(getattr(c.func, 'id', None) == 'get_package_share_directory'
                       and c.args and isinstance(c.args[0], ast.Constant)
                       and c.args[0].value == 'race_diagnostics'
                       for c in ast.walk(ast.Module(body=node.body, type_ignores=[]))
                       if isinstance(c, ast.Call))
        handles = [h for h in node.handlers
                   if getattr(h.type, 'id', None) == 'PackageNotFoundError']
        if looks_up and handles:
            assert not any(isinstance(n, ast.Raise) for h in handles for n in ast.walk(h))
            guarded.append(node)
    assert len(guarded) == 1, f'{path.name}: recorder lookup is not guarded'


def test_the_included_launch_file_and_executable_are_installed():
    setup = (PKG / 'setup.py').read_text()
    assert "'launch/auto_record_launch.py'" in setup
    assert "'config/auto_record.yaml'" in setup
    assert "'auto_record = race_diagnostics.auto_record_node:main'" in setup
    launch = (PKG / 'launch' / 'auto_record_launch.py').read_text()
    assert "executable='auto_record'" in launch


def _bringup_include(record):
    launch = pytest.importorskip('launch', reason='needs ROS sourced')
    pytest.importorskip('launch_ros', reason='needs ROS sourced')
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    try:
        get_package_share_directory('race_diagnostics')
    except PackageNotFoundError:
        pytest.skip('workspace not built/sourced (source install/setup.bash)')
    import importlib.util
    spec = importlib.util.spec_from_file_location('bringup', BRINGUPS[0])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()
    context = launch.LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, launch.actions.DeclareLaunchArgument):
            context.launch_configurations[entity.name] = (
                record if entity.name == 'record' else '')
    includes = [e for e in ld.entities
                if isinstance(e, launch.actions.IncludeLaunchDescription)]
    assert len(includes) == 1
    return includes[0], context


def test_real_bringup_starts_the_recorder_by_default():
    include, context = _bringup_include('true')
    assert include.condition.evaluate(context) is True
    included = include.launch_description_source.get_launch_description(context)
    location = include.launch_description_source.location
    assert location.endswith(os.path.join('race_diagnostics', 'launch', 'auto_record_launch.py'))
    nodes = [(getattr(e, 'node_package', None), getattr(e, 'node_executable', None))
             for e in included.entities]
    assert ('race_diagnostics', 'auto_record') in nodes


def test_real_bringup_record_false_leaves_the_recorder_out():
    include, context = _bringup_include('false')
    assert include.condition.evaluate(context) is False
