"""Steering checks that need no extra driving: the centre and the clamp.

A steering command passes through these software layers before the wheels
move:

    remote stick  ->  joy_teleop scale  ->  /ackermann_cmd angle
      ->  servo = gain * angle + offset  ->  vesc_driver clamp
      (servo_min / servo_max)  ->  servo, horn, linkage, wheels

On 2026-10-10 the clamp cut full right to 12.3 deg while full left reached
19.5 deg, with nothing on the car saying so. ``clamp_check`` works that out
from the settings alone: it needs no capture, so it is always shown. The
wheels-up "steering range" capture that used to do this with a phone angle
app was removed on 2026-10-16; measured circles at several servo_max values
(steering_curve.py) answer the hardware half better.

``centre_offset_suggestion`` turns the latest drive-straight run into the
servo value where the car drives straight.

Pure Python: no ROS, so every rule here is tested with synthetic sessions.
"""

from __future__ import annotations

import math

from odom_calibration.calibration_math import _finite
from odom_calibration.steering_curve import symmetric_reach  # noqa: F401


# The largest centre-offset change one drift run may suggest. More than
# this is a line-up mistake far more often than a real trim error.
MAX_CENTRE_STEP = 0.10
# The car drives straight enough once a drift run implies less than this
# (rad). 0.0025 rad is about 9 cm sideways over 5 m with this wheelbase.
CENTRE_TOLERANCE_RAD = 0.0025


def _fmt_deg(radians):
    return f'{math.degrees(radians):.1f}°' if _finite(radians) else 'n/a'


def clamp_check(parameters, limits, stick_scale=None, planner_max=None):
    """Where the servo clamp cuts steering short, from the settings alone.

    Returns the angle (rad) each side can command before servo_min/servo_max
    clips it, and a finding for each setting that asks for more: full stick
    (``stick_scale``, joy_teleop.yaml) and the driving nodes' limit
    (``planner_max``, their max_steering_angle).
    """
    gain = (parameters or {}).get('steering_angle_to_servo_gain')
    offset = (parameters or {}).get('steering_angle_to_servo_offset')
    result = {'left_reach_rad': None, 'right_reach_rad': None, 'findings': [],
              'stick_scale_rad': stick_scale if _finite(stick_scale) else None,
              'planner_max_rad': planner_max if _finite(planner_max) else None}
    if not (_finite(gain) and gain != 0.0 and _finite(offset) and limits
            and all(_finite(v) for v in limits)):
        return result
    low, high = float(limits[0]), float(limits[1])
    if not low < offset < high:
        result['findings'].append(
            f'The steering offset {offset:.4f} is outside servo_min {low:.3f} / '
            f'servo_max {high:.3f}: one side cannot steer at all.')
        return result
    ends = sorted([(low - offset) / gain, (high - offset) / gain])
    result['right_reach_rad'], result['left_reach_rad'] = -ends[0], ends[1]
    limit_of = {'left': 'servo_max' if gain > 0.0 else 'servo_min',
                'right': 'servo_min' if gain > 0.0 else 'servo_max'}
    for label, asked in (('Full stick (joy_teleop.yaml steering scale)', stick_scale),
                         ('The driving nodes\' max_steering_angle', planner_max)):
        if not _finite(asked):
            continue
        for side in ('left', 'right'):
            reach = result[f'{side}_reach_rad']
            if asked > reach + 1e-6:
                result['findings'].append(
                    f'{label} asks for {_fmt_deg(asked)}, but {limit_of[side]} '
                    f'cuts the {side} side at {_fmt_deg(reach)}. Commands past '
                    'that are clipped without any warning, so the car turns '
                    f'wider {side} than it was told to.')
    return result


def centre_offset_suggestion(session, drift_angle_fn):
    """Offset that would make the latest drift run straight.

    The servo value measured during a drift run produced the measured drift
    angle, so on the line servo = gain * angle + offset the straight-ahead
    servo is servo - gain * angle. Uses the gain that was live for that run.
    """
    drifts = [
        trial for trial in session.get('trials', [])
        if trial.get('accepted') and trial.get('kind') == 'steering_drift'
    ]
    if not drifts:
        return None
    trial = drifts[-1]
    parameters = trial.get('parameters_at_capture') or session.get(
        'current_parameters', {})
    gain = parameters.get('steering_angle_to_servo_gain')
    current = session.get('current_parameters', {}).get(
        'steering_angle_to_servo_offset')
    wheelbase = parameters.get('wheelbase', 0.36)
    summary = trial.get('summary', {})
    stats = summary.get('servo_active') or summary.get('servo') or {}
    servo = stats.get('median')
    result = {
        'trial_id': trial.get('id'),
        'current_offset': current,
        'suggested_offset': None,
        'drift_angle_rad': None,
        'straight': False,
        'blocked': None,
        'stale': False,
        'run_count': len(drifts),
    }
    # A run recorded under an older offset says nothing about the current
    # one: after writing a new offset, the next step is another run.
    recorded = parameters.get('steering_angle_to_servo_offset')
    if _finite(recorded) and _finite(current) and abs(recorded - current) > 1e-9:
        result['stale'] = True
        result['blocked'] = (
            f'the latest drive-straight run was recorded with offset '
            f'{recorded:.4f}, but the car now uses {current:.4f}; record '
            'another drive-straight run to check the new offset')
        return result
    if (stats.get('count') or 0) and stats['count'] < 10 and 'servo_active' in summary:
        result['blocked'] = 'too few servo samples while driving; re-record'
        return result
    if not (_finite(gain) and gain != 0.0 and _finite(servo)):
        result['blocked'] = 'no servo value or gain for this run'
        return result
    try:
        angle = drift_angle_fn(
            wheelbase, trial.get('measured_forward_m'), trial.get('measured_lateral_m'))
    except ValueError as exc:
        result['blocked'] = str(exc)
        return result
    suggested = servo - gain * angle
    result['drift_angle_rad'] = angle
    result['suggested_offset'] = suggested
    result['straight'] = abs(angle) <= CENTRE_TOLERANCE_RAD
    if _finite(current) and abs(suggested - current) > MAX_CENTRE_STEP:
        result['blocked'] = (
            f'it would move the offset by {abs(suggested - current):.3f}, more '
            f'than {MAX_CENTRE_STEP:.2f} in one step; re-check the line-up and '
            'the sideways measurement, then repeat the run')
    return result
