"""Writing joy_teleop.yaml: only the full-stick steering scale may change.

Runs against a copy of the real, committed joy_teleop.yaml, so its comments
and its vendored axis fix (docs/git-setup.md) are what the editor has to
survive. The filesystem is used for real (pytest tmp_path), not mocked.
"""

import os
from pathlib import Path
import re
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import teleop_yaml, vesc_yaml  # noqa: E402

REAL = (Path(__file__).resolve().parents[2]
        / 'f1tenth_system' / 'f1tenth_stack' / 'config' / 'joy_teleop.yaml')


@pytest.fixture
def original():
    return REAL.read_text(encoding='utf-8')


def _profiles(text):
    return yaml.safe_load(text)['joy_teleop']['ros__parameters']


def _scale_written(text):
    """Oracle: the scale line under drive-steering_angle in human_control,
    found by reading the text top to bottom, independent of the module."""
    in_human = in_steer = False
    for line in text.splitlines():
        if re.match(r'^ {4}\S', line):
            in_human = line.strip() == 'human_control:'
        if in_human and line.strip() == 'drive-steering_angle:':
            in_steer = True
            continue
        if in_steer and line.strip().startswith('scale:'):
            return float(line.split(':', 1)[1].split('#')[0])
    raise AssertionError('no steering scale line found')


def test_reads_the_scale_the_file_sets(original):
    assert teleop_yaml.read_scale(original) == _scale_written(original)


def test_writes_only_the_human_control_steering_scale(original):
    new = teleop_yaml.update_text(original, 0.4)
    teleop_yaml.verify(original, new, 0.4)
    assert _scale_written(new) == 0.4
    before, after = _profiles(original), _profiles(new)
    after['human_control']['axis_mappings']['drive-steering_angle']['scale'] = \
        before['human_control']['axis_mappings']['drive-steering_angle']['scale']
    assert after == before   # nothing else, in any profile
    changed = [(a, b) for a, b in zip(original.splitlines(), new.splitlines()) if a != b]
    assert len(changed) == 1 and 'scale:' in changed[0][0]
    assert [l for l in new.splitlines() if l.lstrip().startswith('#')] == \
        [l for l in original.splitlines() if l.lstrip().startswith('#')]


def test_the_default_profile_scale_is_never_touched(original):
    default_before = _profiles(original)['default']
    new = teleop_yaml.update_text(original, 0.3)
    assert _profiles(new)['default'] == default_before


@pytest.mark.parametrize('value', [0.0, -0.4, float('nan'), float('inf'), True, '0.4'])
def test_refuses_a_value_that_is_not_a_positive_number(original, value):
    with pytest.raises(ValueError, match='positive finite number'):
        teleop_yaml.update_text(original, value)


def test_refuses_a_file_without_the_mapping():
    text = 'joy_teleop:\n  ros__parameters:\n    human_control:\n      type: topic\n'
    with pytest.raises(ValueError, match='will not guess'):
        teleop_yaml.update_text(text, 0.4)


def test_refuses_two_scale_lines(original):
    # Duplicate the human_control steering scale line itself.
    line = f'          scale: {_scale_written(original)!r}\n'
    assert original.count(line) == 1, 'the fixture edit must land on one line'
    doubled = original.replace(line, line + '          scale: 0.1\n')
    with pytest.raises(ValueError, match='2 scale lines'):
        teleop_yaml.update_text(doubled, 0.4)


def test_verify_catches_any_other_change(original):
    new = teleop_yaml.update_text(original, 0.4)
    tampered = new.replace('deadman_buttons: [4]', 'deadman_buttons: [5]', 1)
    assert tampered != new
    with pytest.raises(ValueError, match='changed other values'):
        teleop_yaml.verify(original, tampered, 0.4)


def test_verify_catches_a_value_that_did_not_land(original):
    with pytest.raises(ValueError, match='did not come out as 0.4'):
        teleop_yaml.verify(original, original, 0.4)


def test_apply_backs_up_then_writes_and_restore_puts_it_back(tmp_path, original):
    target = tmp_path / 'joy_teleop.yaml'
    target.write_text(original, encoding='utf-8')
    result = teleop_yaml.apply(target, 0.4, tmp_path / 'backups')
    assert _scale_written(target.read_text(encoding='utf-8')) == 0.4
    assert Path(result['backup']).read_text(encoding='utf-8') == original
    assert result['before'] == {'steering_stick_scale': _scale_written(original)}
    assert result['after'] == {'steering_stick_scale': 0.4}
    vesc_yaml.restore(target, result['backup'])
    assert target.read_text(encoding='utf-8') == original


def test_apply_writes_nothing_when_refused(tmp_path, original):
    target = tmp_path / 'joy_teleop.yaml'
    target.write_text(original, encoding='utf-8')
    with pytest.raises(ValueError):
        teleop_yaml.apply(target, -1.0, tmp_path / 'backups')
    assert target.read_text(encoding='utf-8') == original
    assert not (tmp_path / 'backups').exists()


# --- servo limits in vesc.yaml -----------------------------------------------

VESC = REAL.parent / 'vesc.yaml'


def test_servo_max_is_written_into_the_shared_block_only():
    text = VESC.read_text(encoding='utf-8')
    new = vesc_yaml.update_text(text, {'servo_max': 0.9})
    vesc_yaml.verify(text, new, {'servo_max': 0.9})
    assert yaml.safe_load(new)['/**']['ros__parameters']['servo_max'] == 0.9
    assert vesc_yaml.read_values(new, ('servo_max', 'servo_min'))['servo_max'] == 0.9


def test_reading_calibration_values_does_not_return_the_limits():
    values = vesc_yaml.read_values(VESC.read_text(encoding='utf-8'))
    assert set(values) == set(vesc_yaml.CALIBRATION_NAMES)
