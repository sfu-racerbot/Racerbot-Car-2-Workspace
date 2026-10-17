"""Writing vesc.yaml: only the asked-for values change, in the right block.

Runs against a copy of the real, committed vesc.yaml so its comments and
layout are what the editor has to survive. The filesystem is used for real
(pytest tmp_path), not mocked.
"""

import math
import os
from pathlib import Path
import re
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import vesc_yaml  # noqa: E402

REAL_VESC_YAML = (Path(__file__).resolve().parents[2]
                  / 'f1tenth_system' / 'f1tenth_stack' / 'config' / 'vesc.yaml')


@pytest.fixture
def original():
    return REAL_VESC_YAML.read_text(encoding='utf-8')


def _params(text, block):
    return yaml.safe_load(text)[block]['ros__parameters']


def _comments(text):
    return [line for line in text.splitlines() if line.lstrip().startswith('#')]


def test_steering_offset_changes_only_in_the_shared_block(original):
    new = vesc_yaml.update_text(original, {'steering_angle_to_servo_offset': 0.5712})
    assert _params(new, '/**')['steering_angle_to_servo_offset'] == 0.5712
    vesc_yaml.verify(original, new, {'steering_angle_to_servo_offset': 0.5712})
    assert _comments(new) == _comments(original)
    # Exactly one line differs.
    changed = [(a, b) for a, b in zip(original.splitlines(), new.splitlines()) if a != b]
    assert changed == [('    steering_angle_to_servo_offset: 0.5904',
                        '    steering_angle_to_servo_offset: 0.5712')]


def test_odom_gain_never_touches_the_motor_command_gain(original):
    # The shared /** speed gain (+4614) sets the motor command; only the
    # vesc_to_odom_node override may take a calibrated odometry gain.
    new = vesc_yaml.update_text(original, {'speed_to_erpm_gain': -4500.25})
    assert _params(new, 'vesc_to_odom_node')['speed_to_erpm_gain'] == -4500.25
    assert _params(new, '/**')['speed_to_erpm_gain'] == 4614.0
    vesc_yaml.verify(original, new, {'speed_to_erpm_gain': -4500.25})


def test_odom_offset_is_added_to_the_odom_block_not_the_shared_one(original):
    assert 'speed_to_erpm_offset' not in _params(original, 'vesc_to_odom_node')
    new = vesc_yaml.update_text(original, {'speed_to_erpm_offset': 12.5})
    assert _params(new, 'vesc_to_odom_node')['speed_to_erpm_offset'] == 12.5
    assert _params(new, '/**')['speed_to_erpm_offset'] == 0.0
    vesc_yaml.verify(original, new, {'speed_to_erpm_offset': 12.5})
    lines = new.splitlines()
    gain = next(i for i, line in enumerate(lines)
                if line.strip().startswith('speed_to_erpm_gain: -4447'))
    assert lines[gain + 1] == '    speed_to_erpm_offset: 12.5'


def test_all_four_values_at_once(original):
    updates = {'steering_angle_to_servo_gain': -1.18,
               'steering_angle_to_servo_offset': 0.6,
               'speed_to_erpm_gain': -4400.0,
               'speed_to_erpm_offset': -3.0}
    new = vesc_yaml.update_text(original, updates)
    vesc_yaml.verify(original, new, updates)
    assert vesc_yaml.read_values(new) == updates


def test_read_values_reports_what_each_node_uses(original):
    values = vesc_yaml.read_values(original)
    assert values['speed_to_erpm_gain'] == -4447.983786       # odom override
    assert values['speed_to_erpm_offset'] == 0.0               # inherited from /**
    assert values['steering_angle_to_servo_offset'] == 0.5904


@pytest.mark.parametrize('updates,match', [
    ({'brake_max': 1.0}, 'not a calibration value'),   # servo_max became writable 2026-10-16
    ({'speed_max': 1.0}, 'not a calibration value'),
    ({'steering_angle_to_servo_gain': float('nan')}, 'finite number'),
    ({'steering_angle_to_servo_gain': float('inf')}, 'finite number'),
    ({'steering_angle_to_servo_gain': True}, 'finite number'),
    ({'steering_angle_to_servo_gain': '1.0'}, 'finite number'),
])
def test_refuses_anything_but_a_finite_calibration_value(original, updates, match):
    with pytest.raises(ValueError, match=match):
        vesc_yaml.update_text(original, updates)


def test_refuses_a_file_without_the_block():
    text = '/**:\n  ros__parameters:\n    steering_angle_to_servo_gain: -1.0\n'
    with pytest.raises(ValueError, match='no vesc_to_odom_node block'):
        vesc_yaml.update_text(text, {'speed_to_erpm_gain': -4000.0})


def test_refuses_a_missing_key_it_may_not_add():
    text = '/**:\n  ros__parameters:\n    speed_to_erpm_gain: 4614.0\n'
    with pytest.raises(ValueError, match='will not guess'):
        vesc_yaml.update_text(text, {'steering_angle_to_servo_offset': 0.5})


def test_refuses_a_duplicated_key():
    text = ('/**:\n  ros__parameters:\n    steering_angle_to_servo_offset: 0.5\n'
            '    steering_angle_to_servo_offset: 0.6\n')
    with pytest.raises(ValueError, match='more than once'):
        vesc_yaml.update_text(text, {'steering_angle_to_servo_offset': 0.55})


def test_verify_catches_any_other_change(original):
    tampered = vesc_yaml.update_text(original, {'steering_angle_to_servo_offset': 0.6})
    # Tamper with whatever servo_max the real file holds today, and prove the
    # tamper landed, so a recalibration cannot turn this into a no-op edit.
    tampered, count = re.subn(r'^(\s*servo_max:\s*)\S+', r'\g<1>0.123', tampered,
                              flags=re.MULTILINE)
    assert count == 1
    assert _params(tampered, '/**')['servo_max'] != _params(original, '/**')['servo_max']
    with pytest.raises(ValueError, match='changed other values in the /\\*\\* block'):
        vesc_yaml.verify(original, tampered, {'steering_angle_to_servo_offset': 0.6})


def test_verify_catches_a_value_that_did_not_land(original):
    with pytest.raises(ValueError, match='did not come out as 0.6'):
        vesc_yaml.verify(original, original, {'steering_angle_to_servo_offset': 0.6})


def test_comment_after_a_value_is_kept():
    text = '/**:\n  ros__parameters:\n    steering_angle_to_servo_offset: 0.5  # trim\n'
    new = vesc_yaml.update_text(text, {'steering_angle_to_servo_offset': 0.55})
    assert new == '/**:\n  ros__parameters:\n    steering_angle_to_servo_offset: 0.55  # trim\n'


@pytest.mark.parametrize('value,text', [
    (0.5, '0.5'), (1.0, '1.0'), (-4447.983786, '-4447.983786'),
    (0.12345678, '0.123457'), (0.0, '0.0'), (3, '3.0'),
])
def test_numbers_are_written_as_yaml_floats(value, text):
    assert vesc_yaml._format(value) == text
    assert isinstance(yaml.safe_load(f'x: {text}')['x'], float)


def test_apply_backs_up_first_then_writes(tmp_path, original):
    target = tmp_path / 'vesc.yaml'
    target.write_text(original, encoding='utf-8')
    result = vesc_yaml.apply(target, {'steering_angle_to_servo_offset': 0.5712},
                             tmp_path / 'backups', label='test run!')
    backup = Path(result['backup'])
    assert backup.parent == tmp_path / 'backups'
    assert backup.name.endswith('.test_run_.bak')
    assert backup.read_text(encoding='utf-8') == original
    assert _params(target.read_text(), '/**')['steering_angle_to_servo_offset'] == 0.5712
    assert result['before'] == {'steering_angle_to_servo_offset': 0.5904}
    assert result['after'] == {'steering_angle_to_servo_offset': 0.5712}
    assert not list(tmp_path.glob('.vesc.yaml.*')), 'temporary file left behind'


def test_apply_writes_nothing_when_the_edit_is_refused(tmp_path, original):
    target = tmp_path / 'vesc.yaml'
    target.write_text(original, encoding='utf-8')
    with pytest.raises(ValueError):
        vesc_yaml.apply(target, {'brake_max': 1.0}, tmp_path / 'backups')
    assert target.read_text(encoding='utf-8') == original
    assert not (tmp_path / 'backups').exists()


def test_apply_refuses_a_missing_file(tmp_path):
    with pytest.raises(ValueError, match='not found'):
        vesc_yaml.apply(tmp_path / 'nope.yaml', {'steering_angle_to_servo_offset': 0.5},
                        tmp_path / 'backups')


def test_restore_puts_the_backup_back(tmp_path, original):
    target = tmp_path / 'vesc.yaml'
    target.write_text(original, encoding='utf-8')
    result = vesc_yaml.apply(target, {'steering_angle_to_servo_gain': -1.1},
                             tmp_path / 'backups')
    assert target.read_text(encoding='utf-8') != original
    vesc_yaml.restore(target, result['backup'])
    assert target.read_text(encoding='utf-8') == original


def test_restore_refuses_a_backup_that_does_not_parse(tmp_path, original):
    target = tmp_path / 'vesc.yaml'
    target.write_text(original, encoding='utf-8')
    broken = tmp_path / 'broken.bak'
    broken.write_text('/**: [unclosed\n', encoding='utf-8')
    with pytest.raises(yaml.YAMLError):
        vesc_yaml.restore(target, broken)
    assert target.read_text(encoding='utf-8') == original


def test_restore_refuses_a_vanished_backup(tmp_path):
    with pytest.raises(ValueError, match='no longer exists'):
        vesc_yaml.restore(tmp_path / 'vesc.yaml', tmp_path / 'gone.bak')


def test_real_file_round_trips_unchanged_values(original):
    # Writing the value that is already there must reproduce the file.
    current = vesc_yaml.read_values(original)
    new = vesc_yaml.update_text(original, {
        'steering_angle_to_servo_gain': current['steering_angle_to_servo_gain'],
        'steering_angle_to_servo_offset': current['steering_angle_to_servo_offset'],
        'speed_to_erpm_gain': current['speed_to_erpm_gain'],
    })
    assert new == original
    # Oracle: the gain as written on its own line of the file, read with a plain
    # regex rather than YAML, so it is independent of read_values and of
    # whatever the car is calibrated to today.
    written = re.findall(r'^\s+steering_angle_to_servo_gain:\s*(\S+)', original,
                         flags=re.MULTILINE)
    assert len(written) == 1
    assert math.isclose(current['steering_angle_to_servo_gain'], float(written[0]),
                        abs_tol=0.0)
    # And a value nobody calibrates to, so a reader that returns today's
    # number without parsing it fails.
    edited = re.sub(r'^(\s+steering_angle_to_servo_gain:\s*)\S+', r'\g<1>-1.234567',
                    original, flags=re.MULTILINE)
    assert math.isclose(vesc_yaml.read_values(edited)['steering_angle_to_servo_gain'],
                        -1.234567, abs_tol=0.0)
