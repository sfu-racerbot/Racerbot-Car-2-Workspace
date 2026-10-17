"""What a set of measured circles says about the steering linkage.

Each accepted circle is one point: the servo value the car sent (after the
servo_min/servo_max clamp) and the wheel angle the tape measure implies. A
session can hold several circles per side at different settings: the same
full stick under a raised servo_max reaches further. This module turns those
points into:

- a per-side curve, servo travel from the straight-ahead servo against wheel
  angle, with repeats at one setting merged;
- where the curve levels off: a servo step that adds less than a measuring
  error's worth of wheel angle means the linkage is at, or near, its stop;
- the servo limit that keeps the servo off that stop, the angle both sides
  really reach, and a full-stick steering angle inside it;
- a check of each circle: the angle the gain *believed* it commanded against
  the angle measured, which is how a new calibration is confirmed.

The procedure is the one used on car 2 on 2026-10-16
(docs/hardware-reference.md). Pure Python, no ROS.
"""

from __future__ import annotations

import math


def _finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


# Circles whose servo values are this close were driven at the same setting.
SERVO_GROUP_TOLERANCE = 0.01
# Steps smaller than this in servo travel are too short to judge a slope by.
MIN_STEP_SERVO = 0.02
# A step that adds less wheel angle than this (degrees) adds nothing a tape
# measure can see: one circle's +/-5 cm on a ~1.3 m diameter is ~0.6 deg.
STOP_GAIN_DEG = 1.0
# A servo value this close to a limit is on that limit.
LIMIT_TOLERANCE = 0.002
# Full stick is set this far inside the angle both sides reach (degrees),
# then rounded down to a whole degree.
STICK_MARGIN_DEG = 0.5
# A circle whose measured angle is within this of what the gain commanded
# (degrees) confirms the calibration.
CHECK_TOLERANCE_DEG = 1.0
# Repeats at one setting whose radii differ by more than this disagree.
REPEAT_SPREAD_FRACTION = 0.10


def commanded_angle(servo, gain, offset):
    """The wheel angle (rad) a gain/offset pair believes a servo value gives."""
    if not (_finite(servo) and _finite(gain) and _finite(offset)) or gain == 0.0:
        return None
    return (float(servo) - float(offset)) / float(gain)


def group_side(items, centre):
    """One side's circles merged by setting, ordered by servo travel.

    ``items`` are dicts with ``servo``, ``angle_rad`` (signed) and
    ``radius_m``. Returns points with the mean servo, travel from ``centre``,
    mean unsigned angle in degrees, and the radius spread of any repeats.
    """
    groups = []
    for item in sorted(items, key=lambda i: abs(i['servo'] - centre)):
        for group in groups:
            if abs(group['servos'][0] - item['servo']) <= SERVO_GROUP_TOLERANCE:
                group['servos'].append(item['servo'])
                group['angles'].append(abs(item['angle_rad']))
                group['radii'].append(item['radius_m'])
                group['trial_ids'].append(item.get('trial_id'))
                break
        else:
            groups.append({'servos': [item['servo']],
                           'angles': [abs(item['angle_rad'])],
                           'radii': [item['radius_m']],
                           'trial_ids': [item.get('trial_id')]})
    points = []
    for group in groups:
        servo = sum(group['servos']) / len(group['servos'])
        radii = group['radii']
        mean_radius = sum(radii) / len(radii)
        points.append({
            'servo': servo,
            'travel': abs(servo - centre),
            'angle_deg': math.degrees(sum(group['angles']) / len(group['angles'])),
            'count': len(radii),
            'radius_spread': ((max(radii) - min(radii)) / mean_radius
                              if len(radii) > 1 and mean_radius > 0.0 else 0.0),
            'trial_ids': group['trial_ids'],
        })
    points.sort(key=lambda point: point['travel'])
    return points


def steps_and_stop(points):
    """Angle added by each servo step outward, and the first flat one.

    Returns (steps, stop). ``stop`` is the inner end of the first step of at
    least MIN_STEP_SERVO travel that adds less than STOP_GAIN_DEG: the servo
    value past which more servo stops turning the wheels.
    """
    steps, stop = [], None
    for inner, outer in zip(points, points[1:]):
        span = outer['travel'] - inner['travel']
        added = outer['angle_deg'] - inner['angle_deg']
        step = {'from_servo': inner['servo'], 'to_servo': outer['servo'],
                'added_deg': added, 'servo_span': span,
                'flat': span >= MIN_STEP_SERVO and added < STOP_GAIN_DEG}
        steps.append(step)
        if step['flat'] and stop is None:
            stop = {'servo': inner['servo'], 'beyond_servo': outer['servo'],
                    'added_deg': added}
    return steps, stop


def _within(servo, limits):
    return (limits is None
            or limits[0] - LIMIT_TOLERANCE <= servo <= limits[1] + LIMIT_TOLERANCE)


def side_curve(items, centre, limit, limits):
    """Everything one side's circles say. ``limit`` is that side's servo limit."""
    points = group_side(items, centre)
    steps, stop = steps_and_stop(points)
    usable = [p for p in points if _within(p['servo'], limits)
              and (stop is None or p['travel'] <= abs(stop['servo'] - centre)
                   + LIMIT_TOLERANCE)]
    reach = max(usable, key=lambda p: p['angle_deg']) if usable else None
    outermost = points[-1] if points else None
    return {
        'points': points,
        'steps': steps,
        'stop': stop,
        'reach_deg': reach['angle_deg'] if reach else None,
        'reach_servo': reach['servo'] if reach else None,
        'still_gaining_at_limit': bool(
            outermost and stop is None and _finite(limit)
            and abs(outermost['servo'] - limit) <= LIMIT_TOLERANCE
            and len(points) >= 2),
    }


def servo_limit_suggestion(curve, centre, current_limit):
    """The servo limit that keeps this side off its stop, or None.

    Only ever a value a circle was driven at, so it never moves the servo
    somewhere nobody has checked.
    """
    stop = curve.get('stop')
    if not stop or not _finite(current_limit):
        return None
    value = stop['servo']
    if not 0.0 <= value <= 1.0 or abs(value - current_limit) <= LIMIT_TOLERANCE:
        return None
    if (value - centre) * (current_limit - centre) <= 0.0:
        return None  # the stop is on the wrong side of the centre
    return value


def symmetric_reach(gain, offset, limits):
    """Largest angle (rad) both sides can command inside the servo limits."""
    if not (_finite(gain) and gain != 0.0 and _finite(offset) and limits):
        return None
    ends = [(limit - offset) / gain for limit in limits]
    left, right = max(ends), min(ends)
    if not left > 0.0 > right:
        return 0.0
    return min(left, -right)


def stick_scale_suggestion(left_reach_deg, right_reach_deg, gain, offset, limits):
    """Full-stick steering (rad): inside what both sides reach, never clipped.

    Rounded down to a whole degree after STICK_MARGIN_DEG, and lowered if the
    servo limits would clip it under ``gain``/``offset``.
    """
    if not (_finite(left_reach_deg) and _finite(right_reach_deg)):
        return None
    degrees = math.floor(min(left_reach_deg, right_reach_deg) - STICK_MARGIN_DEG)
    clip = symmetric_reach(gain, offset, limits)
    if clip is not None:
        degrees = min(degrees, math.floor(math.degrees(clip)))
    if degrees <= 0:
        return None
    return math.radians(degrees)


def check_circle(measured_rad, servo, gain, offset):
    """Measured against commanded, for one circle; None without the numbers."""
    commanded = commanded_angle(servo, gain, offset)
    if commanded is None or not _finite(measured_rad):
        return None
    error = math.degrees(abs(measured_rad)) - math.degrees(abs(commanded))
    return {
        'commanded_deg': math.degrees(abs(commanded)),
        'measured_deg': math.degrees(abs(measured_rad)),
        'error_deg': error,
        'passed': abs(error) <= CHECK_TOLERANCE_DEG,
    }


def expected_inner_diameter(angle_rad, wheelbase, track):
    """Inner rear-tire diameter (m) a correct calibration gives at an angle."""
    if not (_finite(angle_rad) and _finite(wheelbase) and _finite(track)):
        return None
    if angle_rad == 0.0 or wheelbase <= 0.0 or track <= 0.0:
        return None
    diameter = 2.0 * wheelbase / math.tan(abs(angle_rad)) - track
    return diameter if diameter > 0.0 else None
