"""Decisions behind the node's write actions, kept free of ROS for testing.

calibration_node.py only wires these to HTTP and rclpy: what may be written
to vesc.yaml and joy_teleop.yaml, and how the session follows values changed
on the car.
"""

from __future__ import annotations

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
