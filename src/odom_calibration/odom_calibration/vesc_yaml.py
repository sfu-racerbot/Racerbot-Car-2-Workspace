"""Write accepted calibration values into vesc.yaml, with a backup first.

vesc.yaml carries hand-written comments that record why each value is what
it is (see docs/git-setup.md), so it is edited line by line rather than
re-dumped through a YAML library. Only the four calibration values can be
written, each into the one block that owns it:

- steering gain and offset live in the shared ``/**`` block, read by
  ackermann_to_vesc_node (the servo command) and vesc_to_odom_node alike;
- the odometry speed gain and offset live in the ``vesc_to_odom_node``
  override, never in ``/**``, whose +4614 speed gain sets the motor command.

After editing, the result is parsed and compared with the original: exactly
the requested values may differ, or nothing is written.

No ROS imports: tested against a copy of the real file.
"""

from __future__ import annotations

from datetime import datetime, timezone
import math
import os
from pathlib import Path
import re
import shutil
import tempfile

import yaml


TARGET_BLOCKS = {
    'steering_angle_to_servo_gain': '/**',
    'steering_angle_to_servo_offset': '/**',
    'speed_to_erpm_gain': 'vesc_to_odom_node',
    'speed_to_erpm_offset': 'vesc_to_odom_node',
}
# A missing key may be added after this sibling in its block. Only the odom
# offset qualifies: it is inherited from /** until a calibration sets it.
INSERT_AFTER = {'speed_to_erpm_offset': 'speed_to_erpm_gain'}

_TOP_LEVEL = re.compile(r'^(?P<name>[^\s#][^:]*):\s*(#.*)?$')


def _format(value):
    text = f'{float(value):.6f}'.rstrip('0')
    return text + '0' if text.endswith('.') else text


def _block_ranges(lines):
    """Top-level block name -> (first line, end line) in the text."""
    starts = []
    for index, line in enumerate(lines):
        match = _TOP_LEVEL.match(line)
        if match:
            starts.append((match.group('name').strip().strip('\'"'), index))
    ranges = {}
    for position, (name, start) in enumerate(starts):
        end = starts[position + 1][1] if position + 1 < len(starts) else len(lines)
        ranges[name] = (start, end)
    return ranges


def update_text(text, updates):
    """Return vesc.yaml text with ``updates`` written into their blocks."""
    for name, value in updates.items():
        if name not in TARGET_BLOCKS:
            raise ValueError(f'{name} is not a calibration value this wizard may write.')
        if not isinstance(value, (int, float)) or isinstance(value, bool) \
                or not math.isfinite(float(value)):
            raise ValueError(f'{name} must be a finite number.')
    lines = text.splitlines(keepends=True)
    for name, value in updates.items():
        block = TARGET_BLOCKS[name]
        ranges = _block_ranges(lines)
        if block not in ranges:
            raise ValueError(f'vesc.yaml has no {block} block.')
        start, end = ranges[block]
        pattern = re.compile(
            rf'^(?P<indent>\s+){re.escape(name)}:(?P<space>\s*)'
            rf'(?P<value>[^\s#]+)(?P<rest>\s*(#.*)?)$')
        found = [i for i in range(start, end) if pattern.match(lines[i].rstrip('\n'))]
        if len(found) > 1:
            raise ValueError(f'{name} appears more than once in the {block} block.')
        if found:
            index = found[0]
            match = pattern.match(lines[index].rstrip('\n'))
            newline = '\n' if lines[index].endswith('\n') else ''
            lines[index] = (f"{match.group('indent')}{name}:{match.group('space') or ' '}"
                            f"{_format(value)}{match.group('rest')}{newline}")
            continue
        sibling = INSERT_AFTER.get(name)
        anchor = None
        if sibling:
            sibling_pattern = re.compile(rf'^(?P<indent>\s+){re.escape(sibling)}:')
            anchor = next((i for i in range(start, end)
                           if sibling_pattern.match(lines[i])), None)
        if anchor is None:
            raise ValueError(f'{name} is not in the {block} block, and the wizard '
                             'will not guess where to add it.')
        indent = sibling_pattern.match(lines[anchor]).group('indent')
        lines.insert(anchor + 1, f'{indent}{name}: {_format(value)}\n')
    return ''.join(lines)


def _parameters(document, block):
    return ((document or {}).get(block) or {}).get('ros__parameters') or {}


def verify(original_text, new_text, updates):
    """Raise unless new_text differs from original_text by exactly ``updates``."""
    before = yaml.safe_load(original_text)
    after = yaml.safe_load(new_text)
    if set(before) != set(after):
        raise ValueError('The edit changed which blocks vesc.yaml contains.')
    for block in before:
        old = dict(_parameters(before, block))
        new = dict(_parameters(after, block))
        for name, value in updates.items():
            if TARGET_BLOCKS[name] != block:
                continue
            if not isinstance(new.get(name), float) \
                    or abs(new[name] - float(value)) > 5e-7:
                raise ValueError(f'{name} did not come out as {value} in {block}.')
            old.pop(name, None)
            new.pop(name, None)
        if old != new:
            raise ValueError(f'The edit changed other values in the {block} block.')


def read_values(text):
    """The current calibration values, from the blocks the car reads them in."""
    document = yaml.safe_load(text)
    shared = _parameters(document, '/**')
    odom = _parameters(document, 'vesc_to_odom_node')
    values = {}
    for name, block in TARGET_BLOCKS.items():
        source = odom if block == 'vesc_to_odom_node' else shared
        value = source.get(name, shared.get(name))
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            values[name] = float(value)
    return values


def _atomic_write(path, text):
    descriptor, temporary = tempfile.mkstemp(dir=path.parent, prefix=f'.{path.name}.')
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        shutil.copymode(path, temporary)
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def apply(path, updates, backup_directory, label='calibration'):
    """Back up vesc.yaml, write ``updates``, verify, and return what happened.

    The backup goes to ``backup_directory`` (outside the git working tree by
    default), named with a UTC timestamp, before anything is written.
    """
    path = Path(os.path.expanduser(str(path))).resolve()
    if not path.is_file():
        raise ValueError(f'vesc.yaml not found at {path}.')
    original = path.read_text(encoding='utf-8')
    new_text = update_text(original, updates)
    verify(original, new_text, updates)
    backup_directory = Path(os.path.expanduser(str(backup_directory)))
    backup_directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    safe_label = re.sub(r'[^A-Za-z0-9_.-]', '_', label)[:40]
    backup = backup_directory / f'vesc.yaml.{stamp}.{safe_label}.bak'
    shutil.copy2(path, backup)
    _atomic_write(path, new_text)
    return {
        'path': str(path),
        'backup': str(backup),
        'before': {name: value for name, value in read_values(original).items()
                   if name in updates},
        'after': {name: float(value) for name, value in updates.items()},
    }


def restore(path, backup):
    """Put a backup taken by ``apply`` back in place."""
    path = Path(os.path.expanduser(str(path))).resolve()
    backup = Path(backup)
    if not backup.is_file():
        raise ValueError(f'Backup {backup} no longer exists.')
    text = backup.read_text(encoding='utf-8')
    yaml.safe_load(text)  # never restore something that does not parse
    _atomic_write(path, text)
    return {'path': str(path), 'restored_from': str(backup)}
