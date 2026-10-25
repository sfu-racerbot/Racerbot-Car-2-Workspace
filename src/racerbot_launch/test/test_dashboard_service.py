"""systemd/racerbot-dashboard.service: car 2's web dashboard at boot.

A unit that starts fine but runs the wrong thing fails silently: a bare
`web_dashboard_launch.py` allows no site (the dashboard shows CAR OFFLINE),
and a different ROS domain shows an empty map. So each check compares the
unit with something independent it has to agree with:

  * what it runs      -> dashboard_launch.py, the documented car-2 entry
                         point (CLAUDE.md, docs/web-dashboard.md), which
                         must exist in this package
  * which ROS graph   -> the submodule's foxglove-bridge.service and its
                         wrapper, already proven on the car's graph: same
                         user, no ROS_DOMAIN_ID / RMW_IMPLEMENTATION
  * install.sh        -> fills every @PLACEHOLDER@ the unit uses
  * dashboard_test_launch.py -> must not start a second dashboard by default
                         (it would die on the busy port 8080)

The first three need no ROS:

    python3 -m pytest src/racerbot_launch/test/test_dashboard_service.py -v

The last needs ROS sourced and is skipped without it (reason below). Also
run by `colcon test --packages-select racerbot_launch`.
"""
import importlib.util
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_SRC = os.path.dirname(_PKG)
_UNIT = os.path.join(_PKG, 'systemd', 'racerbot-dashboard.service')
_INSTALL = os.path.join(_PKG, 'systemd', 'install.sh')
_BRIDGE_DIR = os.path.join(_SRC, 'web_dashboards', 'car', 'systemd')


def _read(path):
    with open(path) as f:
        return f.read()


def _directives(text):
    """[(key, value)] for every non-comment `Key=value` line, in order
    (systemd allows repeated keys such as Environment=)."""
    out = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith(('#', ';', '[')) or '=' not in line:
            continue
        key, value = line.split('=', 1)
        out.append((key.strip(), value.strip()))
    return out


def _values(text, key):
    return [v for k, v in _directives(text) if k == key]


def _env(text):
    env = {}
    for value in _values(text, 'Environment'):
        name, _, val = value.partition('=')
        env[name] = val
    return env


def test_unit_runs_car2_dashboard_launch():
    (exec_start,) = _values(_read(_UNIT), 'ExecStart')
    # The documented car-2 entry point, and it exists in this package.
    assert 'exec ros2 launch racerbot_launch dashboard_launch.py' in exec_start
    assert os.path.isfile(os.path.join(_PKG, 'launch', 'dashboard_launch.py'))
    # Both setups sourced, the workspace one from the unit's own RACERBOT_WS
    # ($$ is systemd's escape for a literal $).
    assert 'source /opt/ros/jazzy/setup.bash' in exec_start
    assert 'source "$${RACERBOT_WS}/install/setup.bash"' in exec_start


def test_unit_joins_the_same_graph_as_the_proven_bridge_unit():
    unit = _read(_UNIT)
    bridge = _read(os.path.join(_BRIDGE_DIR, 'foxglove-bridge.service'))
    for key in ('User', 'Group'):
        assert _values(unit, key) == _values(bridge, key) == ['@USER@']
    assert _env(unit)['HOME'] == _env(bridge)['HOME'] == '/home/@USER@'
    # The bridge leaves the domain and RMW unset (its wrapper says why); a
    # dashboard that set either would sit on a different graph.
    wrapper = _read(os.path.join(_BRIDGE_DIR, 'foxglove-bridge.sh'))
    for name in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION'):
        assert name not in _env(bridge)
        assert not re.search(rf'^\s*(export\s+)?{name}=', wrapper, re.M)
        assert name not in _env(unit)
        assert name not in _values(unit, 'ExecStart')[0]


def test_unit_restarts_whatever_way_it_exits_and_stops_like_ctrl_c():
    # The ask (2026-10-24): the dashboard "always runs" -- a clean exit of
    # ros2 launch must bring it back too, which on-failure would not.
    unit = _read(_UNIT)
    assert _values(unit, 'Restart') == ['always']
    assert _values(unit, 'KillSignal') == ['SIGINT']
    assert _values(unit, 'WantedBy') == ['multi-user.target']


def test_install_fills_every_placeholder_the_unit_uses():
    unit = _read(_UNIT)
    install = _read(_INSTALL)
    used = set(re.findall(r'@[A-Z_]+@', ''.join(
        f'{k}={v}\n' for k, v in _directives(unit))))
    filled = set(re.findall(r's\|(@[A-Z_]+@)\|', install))
    assert used, 'the unit should be a template'
    assert used <= filled, f'install.sh leaves {sorted(used - filled)} unfilled'
    assert '/etc/systemd/system/racerbot-dashboard.service' in install


def _ros():
    try:
        import launch  # noqa: F401
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('racerbot_launch')
        get_package_share_directory('f1tenth_stack')
        return True
    except Exception:  # noqa: BLE001 -- any import/lookup failure means "not sourced"
        return False


@pytest.mark.skipif(
    not _ros(),
    reason='needs ROS sourced and the workspace built (launch + ament index); '
           'run after `source install/setup.bash` -- see CLAUDE.md "Build, test, run"')
@pytest.mark.parametrize('override, expected', [(None, 0), ('false', 0), ('true', 1)])
def test_dashboard_test_launch_starts_no_second_dashboard_by_default(override, expected):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

    path = os.path.join(_PKG, 'launch', 'dashboard_test_launch.py')
    spec = importlib.util.spec_from_file_location('dashboard_test_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()

    ctx = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = ''.join(s.perform(ctx) for s in entity.default_value or [])
            ctx.launch_configurations[entity.name] = (
                override if override is not None and entity.name == 'dashboard' else default)

    def location(include):
        # Unperformed until launch runs it; `.location` is only a repr then.
        subs = include.launch_description_source._LaunchDescriptionSource__location
        return ''.join(s.perform(ctx) for s in subs)

    dashboards = [
        e for e in ld.entities
        if isinstance(e, IncludeLaunchDescription)
        and location(e).endswith('/dashboard_launch.py')]
    assert len(dashboards) == 1
    started = [e for e in dashboards
               if e.condition is None or e.condition.evaluate(ctx)]
    assert len(started) == expected
