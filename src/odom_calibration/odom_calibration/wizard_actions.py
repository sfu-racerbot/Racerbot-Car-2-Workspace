"""Decisions behind the node's write actions, kept free of ROS for testing.

calibration_node.py only wires these to HTTP and rclpy: what may be written
to vesc.yaml, and how the session follows values changed on the car.
"""

from __future__ import annotations

from odom_calibration import calibration_math
from odom_calibration import steering_chain
from odom_calibration.calibration_math import _finite
from odom_calibration.vesc_yaml import TARGET_BLOCKS


def report_updates(report, names):
    """Values from the report that may be written; ValueError otherwise."""
    if not report:
        raise ValueError('Generate the report first.')
    if not isinstance(names, list) or not names:
        raise ValueError('Choose at least one value to apply.')
    updates = {}
    for name in names:
        if name not in TARGET_BLOCKS:
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


def sync_session_parameters(session, live_parameters, servo_limits):
    """Make the session's current values match the car; return what changed.

    Only names the session already tracks are updated, so a node that is
    not running (and whose fallback was never replaced) changes nothing.
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
    if changed:
        vehicle['wheelbase_m'] = current.get('wheelbase')
        # The report compared against the old values; it no longer applies.
        session['report'] = None
    return changed


def pending_restart(session, live_parameters):
    """Names written to vesc.yaml that the running car does not use yet."""
    changes = [c for c in session.get('applied_changes', [])
               if not c.get('restored')]
    if not changes:
        return []
    return [
        name for name, value in changes[-1].get('after', {}).items()
        if not _finite(live_parameters.get(name))
        or abs(float(live_parameters[name]) - float(value)) > 1e-6
    ]
