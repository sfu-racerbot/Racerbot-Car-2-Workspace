"""
Every bringup wires ackermann_mux's output straight into ackermann_to_vesc.

The drive path is /teleop or /drive -> ackermann_mux -> ackermann_cmd ->
ackermann_to_vesc -> /commands/motor|servo/*. Both hops are named in C++,
not in config, so a remap in a launch file is the only way to rewire them,
and a remap whose old name no node uses does nothing at all, silently. Until
2026-10-24 every bringup remapped `ackermann_cmd_out -> ackermann_drive`, a
name the mux never advertises; the drive path worked only because neither
end used a remapped name (docs/known-issues.md history).

Oracle: the topic names in the C++ sources (ackermann_mux.cpp's
create_publisher, ackermann_to_vesc.cpp's create_subscription) and the mux's
input topics in mux.yaml, applied to the remappings each launch file passes.
The launch files are read with ast, so no ROS is needed:

    python3 -m pytest src/racerbot_launch/test/ -v
"""
import ast
import os
import re

import pytest
import yaml

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SYSTEM = os.path.join(_SRC, 'f1tenth_system')
_LAUNCHES = [
    os.path.join(_SYSTEM, 'f1tenth_stack', 'launch', 'bringup_launch.py'),
    os.path.join(_SYSTEM, 'f1tenth_stack', 'launch', 'no_lidar_bringup_launch.py'),
    os.path.join(_SYSTEM, 'f1tenth_stack', 'launch', 'sick_bringup_launch.py'),
    os.path.join(_SRC, 'racerbot_sim', 'launch', 'sim_bringup_launch.py'),
]


def _cpp_topics(path, call):
    """String-literal topic names passed to `call` (create_publisher/...)."""
    with open(path, encoding='utf-8') as handle:
        source = handle.read()
    return set(re.findall(call + r'\s*<[^>]*>\s*\(\s*"([^"]+)"', source))


def mux_output():
    names = _cpp_topics(
        os.path.join(_SYSTEM, 'ackermann_mux', 'src', 'ackermann_mux.cpp'),
        'create_publisher')
    names.discard('diagnostics')
    assert len(names) == 1, names
    return names.pop()


def vesc_input():
    names = _cpp_topics(
        os.path.join(_SYSTEM, 'vesc', 'vesc_ackermann', 'src', 'ackermann_to_vesc.cpp'),
        'create_subscription')
    assert len(names) == 1, names
    return names.pop()


def mux_names():
    """Every topic name ackermann_mux uses: its output and its inputs."""
    with open(os.path.join(_SYSTEM, 'f1tenth_stack', 'config', 'mux.yaml'),
              encoding='utf-8') as handle:
        params = yaml.safe_load(handle)['ackermann_mux']['ros__parameters']
    inputs = {entry['topic'] for group in ('topics', 'locks')
              for entry in (params.get(group) or {}).values()}
    return inputs | {mux_output()}


def node_remappings(path, package, executable=None):
    """The literal remappings of each Node(package=..., executable=...)."""
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    found = []
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and getattr(call.func, 'id', None) == 'Node'):
            continue
        kwargs = {kw.arg: kw.value for kw in call.keywords}
        if executable is not None and ast.literal_eval(kwargs['executable']) != executable:
            continue
        if 'package' in kwargs and ast.literal_eval(kwargs['package']) == package:
            if 'remappings' in kwargs:
                found.append(dict(ast.literal_eval(kwargs['remappings'])))
            else:
                found.append({})
    return found


def remapped(name, remappings):
    return remappings.get(name, name)


@pytest.mark.parametrize('path', _LAUNCHES, ids=os.path.basename)
def test_every_mux_remap_renames_a_topic_the_mux_uses(path):
    (remappings,) = node_remappings(path, 'ackermann_mux')
    dead = sorted(set(remappings) - mux_names())
    assert dead == [], f'{os.path.basename(path)} remaps names the mux never uses: {dead}'


@pytest.mark.parametrize('path', _LAUNCHES[:3], ids=os.path.basename)
def test_mux_output_reaches_the_vesc_converter(path):
    (mux,) = node_remappings(path, 'ackermann_mux')
    (converter,) = node_remappings(path, 'vesc_ackermann', 'ackermann_to_vesc_node')
    assert remapped(mux_output(), mux) == remapped(vesc_input(), converter)


def _declared_default(path, name):
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    for call in ast.walk(tree):
        if (isinstance(call, ast.Call) and getattr(call.func, 'attr', None) == 'declare_parameter'
                and ast.literal_eval(call.args[0]) == name):
            return ast.literal_eval(call.args[1])
    raise AssertionError(f'{name} is not declared in {path}')


def test_sim_mux_output_reaches_the_gym_bridge():
    # gym_bridge_node stands where the VESC chain does; sim_bringup_launch.py
    # passes it no drive_topic, so its declared default is what it hears.
    path = _LAUNCHES[3]
    (mux,) = node_remappings(path, 'ackermann_mux')
    with open(path, encoding='utf-8') as handle:
        assert "'drive_topic'" not in handle.read()
    listens = _declared_default(
        os.path.join(_SRC, 'racerbot_sim', 'racerbot_sim', 'gym_bridge_node.py'), 'drive_topic')
    assert listens == '/' + remapped(mux_output(), mux)


# The checks themselves, on known cases, so they are not only seen passing.
def test_the_oracle_reads_the_names_the_drive_path_uses():
    assert mux_output() == 'ackermann_cmd'
    assert vesc_input() == 'ackermann_cmd'
    assert {'drive', 'teleop'} <= mux_names()


def test_a_remap_of_the_real_output_would_break_the_path():
    assert remapped(mux_output(), {'ackermann_cmd': 'ackermann_drive'}) != vesc_input()
