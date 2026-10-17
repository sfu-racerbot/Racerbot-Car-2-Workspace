"""Write the full-stick steering angle into joy_teleop.yaml, backup first.

The one value this wizard may change here is the manual driving profile's
steering scale, ``human_control.axis_mappings.drive-steering_angle.scale``:
the F710 axis reads +/-1, so it is the angle full stick asks for, in rad.
Like vesc.yaml the file carries hand-written comments (and a vendored local
fix, the axis number; docs/git-setup.md), so it is edited on its one line,
then parsed and compared with the original: only that value may differ, or
nothing is written.

No ROS imports: tested against a copy of the real file.
"""

from __future__ import annotations

from datetime import datetime, timezone
import copy
import math
import os
from pathlib import Path
import re
import shutil

import yaml

from odom_calibration.vesc_yaml import _atomic_write, _format

PROFILE = 'human_control'
MAPPING = 'drive-steering_angle'
NAME = 'steering_stick_scale'


def _indent(line):
    return len(line) - len(line.lstrip(' '))


def _child_range(lines, start, key):
    """(index, end) of ``key:`` nested under the line at ``start``."""
    parent = _indent(lines[start])
    for index in range(start + 1, len(lines)):
        stripped = lines[index].strip()
        if not stripped or stripped.startswith('#'):
            continue
        if _indent(lines[index]) <= parent:
            break
        if stripped.split(':', 1)[0].strip().strip('\'"') == key:
            end = index + 1
            while end < len(lines):
                text = lines[end].strip()
                if text and not text.startswith('#') and \
                        _indent(lines[end]) <= _indent(lines[index]):
                    break
                end += 1
            return index, end
    return None


def _scale_line(lines):
    top = next((i for i, line in enumerate(lines)
                if line.rstrip('\n') == 'joy_teleop:'), None)
    if top is None:
        raise ValueError('joy_teleop.yaml has no joy_teleop block.')
    path = [top]
    for key in ('ros__parameters', PROFILE, 'axis_mappings', MAPPING):
        found = _child_range(lines, path[-1], key)
        if found is None:
            raise ValueError(f'joy_teleop.yaml has no {key} where the steering '
                             'scale belongs; the wizard will not guess.')
        path.append(found[0])
    start, end = _child_range(lines, path[-2], MAPPING)
    pattern = re.compile(r'^(?P<indent>\s+)scale:(?P<space>\s*)'
                         r'(?P<value>[^\s#]+)(?P<rest>\s*(#.*)?)$')
    found = [i for i in range(start + 1, end) if pattern.match(lines[i].rstrip('\n'))]
    if len(found) != 1:
        raise ValueError(f'{MAPPING} has {len(found)} scale lines, expected one.')
    return found[0], pattern


def read_scale(text):
    """The full-stick steering angle (rad) the file sets."""
    document = yaml.safe_load(text)
    value = (document['joy_teleop']['ros__parameters'][PROFILE]
             ['axis_mappings'][MAPPING]['scale'])
    if not isinstance(value, (int, float)) or isinstance(value, bool) \
            or not math.isfinite(float(value)):
        raise ValueError('The steering scale in joy_teleop.yaml is not a number.')
    return float(value)


def update_text(text, value):
    """Return joy_teleop.yaml text with the steering scale set to ``value``."""
    if not isinstance(value, (int, float)) or isinstance(value, bool) \
            or not math.isfinite(float(value)) or float(value) <= 0.0:
        raise ValueError('The steering scale must be a positive finite number.')
    lines = text.splitlines(keepends=True)
    index, pattern = _scale_line(lines)
    match = pattern.match(lines[index].rstrip('\n'))
    newline = '\n' if lines[index].endswith('\n') else ''
    lines[index] = (f"{match.group('indent')}scale:{match.group('space') or ' '}"
                    f"{_format(value)}{match.group('rest')}{newline}")
    return ''.join(lines)


def verify(original_text, new_text, value):
    """Raise unless the only difference is the steering scale, now ``value``."""
    before = yaml.safe_load(original_text)
    after = yaml.safe_load(new_text)
    if abs(read_scale(new_text) - float(value)) > 5e-7:
        raise ValueError(f'The steering scale did not come out as {value}.')
    expected = copy.deepcopy(before)
    (expected['joy_teleop']['ros__parameters'][PROFILE]['axis_mappings']
     [MAPPING]['scale']) = (after['joy_teleop']['ros__parameters'][PROFILE]
                            ['axis_mappings'][MAPPING]['scale'])
    if expected != after:
        raise ValueError('The edit changed other values in joy_teleop.yaml.')


def apply(path, value, backup_directory, label='calibration'):
    """Back up joy_teleop.yaml, write the scale, verify, report what happened."""
    path = Path(os.path.expanduser(str(path))).resolve()
    if not path.is_file():
        raise ValueError(f'joy_teleop.yaml not found at {path}.')
    original = path.read_text(encoding='utf-8')
    new_text = update_text(original, value)
    verify(original, new_text, value)
    backup_directory = Path(os.path.expanduser(str(backup_directory)))
    backup_directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    safe_label = re.sub(r'[^A-Za-z0-9_.-]', '_', label)[:40]
    backup = backup_directory / f'joy_teleop.yaml.{stamp}.{safe_label}.bak'
    shutil.copy2(path, backup)
    _atomic_write(path, new_text)
    return {
        'path': str(path),
        'backup': str(backup),
        'before': {NAME: read_scale(original)},
        'after': {NAME: float(value)},
    }
