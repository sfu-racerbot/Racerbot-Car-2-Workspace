"""
config/vehicle_state.yaml against the code that reads it. No ROS needed:

    python3 -m pytest src/imu_bridge/test/ -v

Oracles: EstimatorConfig's defaults (the node declares every parameter with
them, so a YAML that drifts from them is a second, silent set of numbers),
and car 1's wheelbase as written in f1tenth_stack's vesc.yaml.
"""
from dataclasses import fields
import os
from pathlib import Path
import sys

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from imu_bridge.vehicle_state import (  # noqa: E402
    EstimatorConfig, check_frames, check_output_topics)

YAML = HERE.parent / 'config' / 'vehicle_state.yaml'
VESC_YAML = (HERE.parents[1] / 'f1tenth_system' / 'f1tenth_stack' / 'config' / 'vesc.yaml')
NODE = HERE.parent / 'imu_bridge' / 'vehicle_state_node.py'
TABLES = {'x_offset': ('bno_x_offset', 'vesc_x_offset'),
          'gyro_weight': ('bno_gyro_weight', 'vesc_gyro_weight')}


def _params():
    return yaml.safe_load(YAML.read_text())['vehicle_state_node']['ros__parameters']


def test_every_estimator_knob_is_in_the_yaml_with_the_default_value():
    # oracle: invariant -- one set of numbers, not two.
    params = _params()
    defaults = EstimatorConfig()
    for f in fields(EstimatorConfig):
        if f.name in TABLES:
            table = dict(getattr(defaults, f.name))
            for source, key in zip(('bno', 'vesc'), TABLES[f.name]):
                assert params[key] == table[source], key
        else:
            value = params[f.name]
            value = tuple(value) if isinstance(value, list) else value   # YAML list = code tuple
            assert value == getattr(defaults, f.name), f.name


def test_float_knobs_are_written_as_floats():
    # ROS rejects an int for a parameter declared double (the node declares
    # them from EstimatorConfig's float defaults): `slip_gate: 4` would stop
    # the node at startup.
    params = _params()
    for key, value in params.items():
        if (key.endswith('_topic') or key.endswith('_frame') or key == 'accel_source'
                or isinstance(value, (bool, list))):
            continue
        assert isinstance(value, float), f'{key}: {value!r} is not a float'


def test_yaml_builds_a_valid_estimator_and_safe_topics():
    params = _params()
    kwargs = {}
    for f in fields(EstimatorConfig):
        if f.name in TABLES:
            kwargs[f.name] = tuple(zip(('bno', 'vesc'), (params[k] for k in TABLES[f.name])))
        else:
            kwargs[f.name] = params[f.name]
    EstimatorConfig(**kwargs)
    check_output_topics(params['odom_topic'], params['fused_odom_topic'],
                        params['steering_topic'], params['imu_odom_topic'])
    check_frames(params['imu_odom_frame'], params['odom_frame'], params['base_frame'])
    # Off unless asked for: the transform is wrong while SLAM is on `odom`.
    assert params['publish_odom_tf'] is False


def test_wheelbase_is_car_1s():
    # oracle: spec -- vesc.yaml's wheelbase (docs/hardware-reference.md
    # "Physical dimensions used in config": change them together).
    vesc = yaml.safe_load(VESC_YAML.read_text())
    wheelbase = vesc['vesc_to_odom_node']['ros__parameters']['wheelbase']
    assert _params()['wheelbase'] == wheelbase
    # The BNO sits on the front axle: one wheelbase ahead of base_link.
    assert _params()['bno_x_offset'] == wheelbase


def test_node_publishes_only_odometry_joint_state_and_its_one_transform():
    # oracle: spec -- CLAUDE.md safety model. Static check of the node's
    # source: exactly three publishers, none a drive message type, and the
    # only transform it can send is imu_odom_frame -> odom_frame.
    code = NODE.read_text()
    assert code.count('create_publisher(') == 3
    assert 'create_publisher(Odometry, fused_topic' in code
    assert 'create_publisher(JointState, steering_topic' in code
    assert 'create_publisher(Odometry, imu_odom_topic' in code
    assert code.count('sendTransform(') == 1
    assert 'tf.header.frame_id = self.imu_odom_frame' in code
    assert 'tf.child_frame_id = self.odom_frame' in code
    assert 'TransformBroadcaster(self) if self.publish_tf else None' in code
    # (The docstring names /drive and /commands/* to say it never uses them,
    # so the message types are what is checked, not the topic names.)
    for forbidden in ('AckermannDrive', 'ackermann_msgs', 'std_msgs'):
        assert forbidden not in code, forbidden
    assert 'check_output_topics(odom_topic, fused_topic, steering_topic,' in code
    assert 'check_frames(self.imu_odom_frame, self.odom_frame, self.base_frame)' in code
    # A second parent for odom on /tf stops the transform for good.
    assert 'if self.tf_broadcaster is not None and self.tf_blocked_by is None:' in code


def _bridge_pattern():
    import ast
    tree = ast.parse((HERE.parent / 'launch' / 'vehicle_state_launch.py').read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'BRIDGE_PATTERN' for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError('vehicle_state_launch.py no longer defines BRIDGE_PATTERN')


def test_bridge_detection_matches_a_running_bridge_not_an_open_file():
    # oracle: spec -- vehicle_state_launch.py: reuse a bridge that owns the
    # sensor; only a Python process running the script owns it. Python's re
    # stands in for pgrep's POSIX ERE (this pattern uses only the common subset).
    import re
    pattern = re.compile(_bridge_pattern())
    running = [
        'python3 -u tools/bno_i2c_bridge.py --pty /tmp/bno085 --no-dyncal',  # by hand, 2026-10-23
        'python3 -u /home/racerbotcar-1/imu-calibration/tools/bno_i2c_bridge.py --pty /tmp/bno085 --no-dyncal',
        '/usr/bin/python3.12 ./bno_i2c_bridge.py --print',
    ]
    not_running = [
        'vim tools/bno_i2c_bridge.py',
        'less /home/racerbotcar-1/imu-calibration/tools/bno_i2c_bridge.py',
        'grep -n pty tools/bno_i2c_bridge.py',
        'python3 tools/calibrate_imu.py --bno /tmp/bno085',
    ]
    for cmd in running:
        assert pattern.search(cmd), cmd
    for cmd in not_running:
        assert not pattern.search(cmd), cmd
