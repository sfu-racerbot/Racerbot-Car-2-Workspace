"""Decisions behind the node's write actions, kept free of ROS for testing.

calibration_node.py only wires these to HTTP and rclpy: what may be written
to vesc.yaml and joy_teleop.yaml, and how the session follows values changed
on the car.
"""

from __future__ import annotations

import math

from odom_calibration import calibration_math
from odom_calibration import steering_chain
from odom_calibration import teleop_yaml
from odom_calibration.calibration_math import _finite
from odom_calibration.vesc_yaml import TARGET_BLOCKS

WRITABLE = (*TARGET_BLOCKS, teleop_yaml.NAME)


def report_updates(report, names):
    """Values from the report that may be written; ValueError otherwise."""
    if not report:
        raise ValueError('Generate the report first.')
    if not isinstance(names, list) or not names:
        raise ValueError('Choose at least one value to apply.')
    updates = {}
    for name in names:
        if name not in WRITABLE:
            raise ValueError(f'{name!r} cannot be applied.')
        blocked = (report.get('blocked_suggestions') or {}).get(name)
        if blocked:
            raise ValueError(f'{name} is blocked: {blocked}.')
        not_ready = (report.get('not_ready_to_apply') or {}).get(name)
        if not_ready:
            raise ValueError(f'{name} is not ready to apply: {not_ready}.')
        value = (report.get('parameter_suggestions') or {}).get(name)
        if not _finite(value):
            raise ValueError(f'There is no suggestion for {name}.')
        updates[name] = float(value)
    return updates


def manual_updates(values, current, servo_limits):
    """Values typed on the page, if they may be written; ValueError otherwise.

    The person has chosen these numbers, so confidence does not apply, but
    the hard checks do: a value that cannot work on this car (wrong-sign
    gain, an offset outside the servo range, a servo limit past the other,
    full stick past 0.70 rad) is refused exactly as a suggestion would be.
    """
    if not isinstance(values, dict) or not values:
        raise ValueError('Type at least one value to write.')
    for name in values:
        if name not in WRITABLE:
            raise ValueError(f'{name!r} cannot be written.')
    low = values.get('servo_min', servo_limits[0] if servo_limits else None)
    high = values.get('servo_max', servo_limits[1] if servo_limits else None)
    limits = (float(low), float(high)) if _finite(low) and _finite(high) else None
    updates = {}
    for name, value in values.items():
        if isinstance(value, bool) or not _finite(value):
            raise ValueError(f'{name} must be a finite number.')
        blocked, _ = calibration_math.vet_suggestion(
            name, float(value), current.get(name), 'good', limits)
        if blocked:
            raise ValueError(f'{name} {float(value):g} is refused: {blocked}.')
        updates[name] = float(value)
    offset = updates.get('steering_angle_to_servo_offset',
                         current.get('steering_angle_to_servo_offset'))
    if limits and _finite(offset) and not limits[0] < float(offset) < limits[1]:
        raise ValueError(
            f'servo limits {limits[0]:g}-{limits[1]:g} would leave the steering '
            f'offset {float(offset):g} outside them: one side could not steer.')
    return updates


# The largest servo-limit step one press may take. The linkage near full
# lock is where a servo strains, so steps stay small.
MAX_LIMIT_STEP = 0.10
# Full stick raised for a limit step goes this far past the new limit (rad),
# so the clamp, not the stick, decides where the servo stops.
STICK_REACH_MARGIN_RAD = 0.01


def limit_step_updates(current, servo_limits, stick_scale, side, step, raise_stick):
    """The next servo-limit step for one side, and full stick to reach it.

    ``side`` is 'left' or 'right'. Which limit that is follows the gain's
    sign (servo = gain * angle + offset, left positive). Returns the updates
    and a description; ValueError when the step cannot be taken.
    """
    gain = current.get('steering_angle_to_servo_gain')
    offset = current.get('steering_angle_to_servo_offset')
    if not (_finite(gain) and gain != 0.0 and _finite(offset)):
        raise ValueError('The steering gain and offset are not known yet.')
    if side not in ('left', 'right'):
        raise ValueError("side must be 'left' or 'right'.")
    if isinstance(step, bool) or not _finite(step) or not 0.005 <= float(step) <= MAX_LIMIT_STEP:
        raise ValueError(f'The step must be between 0.005 and {MAX_LIMIT_STEP:g}.')
    if not servo_limits or not all(_finite(v) for v in servo_limits):
        raise ValueError('The servo limits are not known yet.')
    towards_max = (gain > 0.0) == (side == 'left')
    name = 'servo_max' if towards_max else 'servo_min'
    now = float(servo_limits[1] if towards_max else servo_limits[0])
    value = round(min(1.0, now + float(step)) if towards_max
                  else max(0.0, now - float(step)), 4)
    if abs(value - now) < 1e-9:
        raise ValueError(f'{name} is already {now:g}, the end of the 0-1 servo signal.')
    updates = {name: value}
    needed = abs(value - float(offset)) / abs(float(gain))
    if raise_stick and not (_finite(stick_scale) and float(stick_scale) >= needed):
        updates[teleop_yaml.NAME] = math.ceil(
            (needed + STICK_REACH_MARGIN_RAD) * 100.0) / 100.0
    return updates


def restore_plan(changes, everything=False):
    """Which backups an undo puts back, and which records it closes.

    The latest write is undone as a whole (both files of one write). With
    ``everything``, each file goes back to its backup from before the
    session's first write still open, so the car returns to where the
    session started.
    """
    open_changes = [c for c in changes if not c.get('restored')]
    if not open_changes:
        raise ValueError('Nothing applied in this session is left to undo.')
    if not everything:
        last = open_changes[-1].get('apply_id')
        batch = [c for c in open_changes
                 if last is not None and c.get('apply_id') == last] or open_changes[-1:]
        return batch, batch
    first_per_file = {}
    for change in open_changes:
        first_per_file.setdefault(change['path'], change)
    return list(first_per_file.values()), open_changes


def split_updates(updates):
    """(vesc.yaml updates, joy_teleop.yaml steering scale or None)."""
    vesc = {name: value for name, value in updates.items() if name in TARGET_BLOCKS}
    return vesc, updates.get(teleop_yaml.NAME)


def centre_updates(session, servo_limits):
    """The drive-straight offset, if it may be written; ValueError otherwise."""
    centre = steering_chain.centre_offset_suggestion(
        session, calibration_math.drift_steering_angle)
    if centre is None:
        raise ValueError('Record and accept a drive-straight run first.')
    if centre.get('blocked'):
        raise ValueError(f"The offset is blocked: {centre['blocked']}.")
    value = centre.get('suggested_offset')
    blocked, _ = calibration_math.vet_suggestion(
        'steering_angle_to_servo_offset', value,
        session.get('current_parameters', {}).get('steering_angle_to_servo_offset'),
        'good', tuple(servo_limits))
    if blocked:
        raise ValueError(f'The offset is blocked: {blocked}.')
    return {'steering_angle_to_servo_offset': float(value)}


def sync_session_parameters(session, live_parameters, servo_limits,
                            stick_scale=None):
    """Make the session's current values match the car; return what changed.

    Only names the session already tracks are updated, so a node that is
    not running (and whose fallback was never replaced) changes nothing.
    ``stick_scale`` is joy_teleop's full-stick steering angle, when known.
    """
    changed = []
    current = session.setdefault('current_parameters', {})
    for name, value in live_parameters.items():
        if name in current and _finite(value) \
                and abs(float(current[name]) - float(value)) > 1e-9:
            changed.append(f'{name} {current[name]} -> {value}')
            current[name] = float(value)
    vehicle = session.setdefault('vehicle', {})
    for name, value in zip(('servo_min', 'servo_max'), servo_limits):
        if not _finite(value):
            continue
        old = vehicle.get(name)
        if not _finite(old) or abs(float(old) - float(value)) > 1e-9:
            changed.append(f'{name} {old} -> {value}')
            vehicle[name] = float(value)
    if _finite(stick_scale):
        old = vehicle.get(teleop_yaml.NAME)
        if not _finite(old) or abs(float(old) - float(stick_scale)) > 1e-9:
            changed.append(f'{teleop_yaml.NAME} {old} -> {stick_scale}')
            vehicle[teleop_yaml.NAME] = float(stick_scale)
    if changed:
        vehicle['wheelbase_m'] = current.get('wheelbase')
        # The report compared against the old values; it no longer applies.
        session['report'] = None
    return changed


def pending_restart(session, live_parameters):
    """Names last written that the running car does not use yet.

    ``live_parameters`` holds every value the car reports, including
    servo_min/servo_max and joy_teleop's steering_stick_scale. One write can
    touch two files; all of its records count.
    """
    changes = [c for c in session.get('applied_changes', [])
               if not c.get('restored')]
    if not changes:
        return []
    last = changes[-1].get('apply_id')
    batch = [c for c in changes if last is not None and c.get('apply_id') == last] \
        or changes[-1:]
    written = {}
    for change in batch:
        written.update(change.get('after', {}))
    return [
        name for name, value in written.items()
        if not _finite(live_parameters.get(name))
        or abs(float(live_parameters[name]) - float(value)) > 1e-6
    ]
