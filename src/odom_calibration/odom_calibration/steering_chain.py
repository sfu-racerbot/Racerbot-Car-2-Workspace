"""Is the steering limited by software or by hardware?

A steering command passes through five places before the wheels move, and
any of them can make full left and full right unequal:

    remote stick  ->  joy_teleop scale  ->  /ackermann_cmd angle
      ->  servo = gain * angle + offset  ->  vesc_driver clamp
      (servo_min / servo_max)  ->  servo, horn, linkage, wheels

The first four are software and are all visible on ROS topics. The last is
hardware and is only visible with a phone or protractor on the wheel. The
steering range check records the software layers while the driver holds full
left and full right with the wheels off the ground; the operator then types
the measured wheel angles. Comparing layer by layer names the first place
the two sides stop matching.

Pure Python: no ROS, so every rule here is tested with synthetic captures.
"""

from __future__ import annotations

import math

from odom_calibration.calibration_math import (
    _Lookup,
    _finite,
    median,
)


# Steering commands smaller than this are "centred" (rad, about 1.1 deg).
SIDE_THRESHOLD_RAD = 0.02
# Within a side, samples at least this fraction of the side's largest
# command count as "holding the stick fully over".
PLATEAU_FRACTION = 0.9
# A stick that cannot reach this far is short of full travel.
FULL_STICK_AXIS = 0.95
# Fewer plateau samples than this per side is not a held stick.
MIN_PLATEAU_SAMPLES = 5
# Servo values within this of each other are equal (servo units). Matches
# calibration_math.SERVO_LIMIT_TOLERANCE.
SERVO_TOLERANCE = 0.002
# A measured servo command further than this from gain * angle + offset means
# the wizard's parameters are not the ones commanding the servo.
SERVO_MODEL_TOLERANCE = 0.01
# Left/right commands differing by more than this fraction are asymmetric.
COMMAND_SYMMETRY_FRACTION = 0.03
# Commanded or measured angles differing by more than these are asymmetric.
# 1 degree is about what a phone angle app can resolve on a tyre wall.
COMMANDED_SYMMETRY_DEG = 1.0
MEASURED_SYMMETRY_DEG = 1.5
# The wheel/command ratios of the two sides agreeing within this fraction
# means the wheels follow the command: the software alone explains the gap.
RATIO_AGREEMENT_FRACTION = 0.10
# A wheel that turns this much less than commanded on one side is stopped
# by something mechanical before the servo gets there.
SHORTFALL_FRACTION = 0.80
# The largest centre-offset change one drift run may suggest. More than
# this is a line-up mistake far more often than a real trim error.
MAX_CENTRE_STEP = 0.10
# The car drives straight enough once a drift run implies less than this
# (rad). 0.0025 rad is about 9 cm sideways over 5 m with this wheelbase.
CENTRE_TOLERANCE_RAD = 0.0025


def _deg(radians):
    return math.degrees(radians) if _finite(radians) else None


def _fmt_deg(radians):
    return f'{math.degrees(radians):.1f} deg' if _finite(radians) else 'n/a'


def sweep_summary(capture: dict):
    """Per-side medians of every software layer while the stick is held over.

    A side's "plateau" is the LB-held stretch where the commanded angle is
    within PLATEAU_FRACTION of that side's largest command, so slow stick
    movements on the way out and back are left out.
    """
    lb = _Lookup(capture.get('joy', []), 'lb_held')
    drive = [
        sample for sample in capture.get('drive', [])
        if _finite(sample.get('t')) and _finite(sample.get('steering'))
        and lb.at(float(sample['t'])) is True
    ]
    peak = {
        'left': max((s['steering'] for s in drive if s['steering'] > SIDE_THRESHOLD_RAD),
                    default=None),
        'right': min((s['steering'] for s in drive if s['steering'] < -SIDE_THRESHOLD_RAD),
                     default=None),
    }

    def side_of(steering):
        for side, extreme in peak.items():
            if extreme is not None and steering * extreme > 0.0 \
                    and abs(steering) >= PLATEAU_FRACTION * abs(extreme):
                return side
        return None

    drive_side = _Lookup(
        [{'t': s['t'], 'side': side_of(s['steering'])} for s in drive], 'side')

    def plateau_values(topic, field):
        values = {'left': [], 'right': []}
        for sample in capture.get(topic, []):
            if not _finite(sample.get('t')) or not _finite(sample.get(field)):
                continue
            if lb.at(float(sample['t'])) is not True:
                continue
            side = drive_side.at(float(sample['t']))
            if side in values:
                values[side].append(float(sample[field]))
        return values

    command = {'left': [], 'right': []}
    for sample in drive:
        side = side_of(sample['steering'])
        if side:
            command[side].append(float(sample['steering']))
    servo_command = plateau_values('servo_cmd', 'value')
    servo_clamped = plateau_values('servo', 'value')
    axis = plateau_values('joy', 'steer_axis')
    axes = [
        float(sample['steer_axis']) for sample in capture.get('joy', [])
        if _finite(sample.get('steer_axis'))
    ]

    sides = {}
    for side in ('left', 'right'):
        sides[side] = {
            'samples': len(command[side]),
            'command_rad': median(command[side]),
            'servo_command': median(servo_command[side]),
            'servo_clamped': median(servo_clamped[side]),
            'joy_axis': median(axis[side]),
        }
    warnings = []
    for side in ('left', 'right'):
        if sides[side]['samples'] < MIN_PLATEAU_SAMPLES:
            warnings.append(
                f'No steady full-{side} hold was recorded. Hold LB and push '
                f'the stick fully {side} for about 3 seconds.')
    if not capture.get('servo_cmd'):
        warnings.append(
            'No /commands/servo/position messages; the pre-clamp servo value '
            'is computed from the parameters instead of measured.')
    return {
        'sides': sides,
        'joy_axis_max': max(axes, default=None),
        'joy_axis_min': min(axes, default=None),
        'warnings': warnings,
    }


def _measured_wheel_angle(lock):
    """Mean of the front-wheel angles typed for one lock, degrees -> rad."""
    values = [
        abs(float(lock[name])) for name in ('left_wheel_deg', 'right_wheel_deg')
        if _finite((lock or {}).get(name))
    ]
    return math.radians(sum(values) / len(values)) if values else None


def validate_sweep_measurement(payload: dict):
    """Operator-entered phone readings; every field is optional."""
    fields = {}
    for lock in ('left_lock', 'right_lock'):
        entered = payload.get(lock) or {}
        if not isinstance(entered, dict):
            raise ValueError(f'{lock} must be an object.')
        clean = {}
        for name in ('left_wheel_deg', 'right_wheel_deg'):
            value = entered.get(name)
            if value is None or value == '':
                continue
            if not _finite(value):
                raise ValueError('Wheel angles must be numbers in degrees.')
            if not 0.0 <= abs(float(value)) <= 60.0:
                raise ValueError(
                    'Wheel angles must be between 0 and 60 degrees; measure '
                    'from straight ahead, not from the phone\'s own zero.')
            clean[name] = abs(float(value))
        fields[lock] = clean
    for flag in ('buzz_left', 'buzz_right'):
        value = payload.get(flag, False)
        if not isinstance(value, bool):
            raise ValueError(f'{flag} must be true or false.')
        fields[flag] = value
    return fields


def symmetric_reach(gain, offset, limits):
    """Largest angle (rad) both sides can command inside the servo limits."""
    if not (_finite(gain) and gain != 0.0 and _finite(offset) and limits):
        return None
    ends = [(limit - offset) / gain for limit in limits]
    left, right = max(ends), min(ends)
    if not left > 0.0 > right:
        return 0.0
    return min(left, -right)


def diagnose(sweep, measurements, parameters, limits,
             planner_max_rad=None, circle_angles=None):
    """Name the first layer where full left and full right stop matching."""
    gain = parameters.get('steering_angle_to_servo_gain')
    offset = parameters.get('steering_angle_to_servo_offset')
    sides = (sweep or {}).get('sides', {})
    left, right = sides.get('left', {}), sides.get('right', {})
    layers, recommendations, software_causes = [], [], []
    have_holds = all(
        (side.get('samples') or 0) >= MIN_PLATEAU_SAMPLES for side in (left, right))
    if not have_holds or not (_finite(gain) and gain != 0.0 and _finite(offset)):
        return {
            'verdict': 'insufficient',
            'summary': 'Record the steering range check: wheels off the '
                       'ground, LB held, full left then full right.',
            'layers': [], 'recommendations': [], 'software_causes': [],
        }

    # 1. Remote stick travel.
    axis_left, axis_right = sweep.get('joy_axis_max'), sweep.get('joy_axis_min')
    if _finite(axis_left) and _finite(axis_right):
        short = [name for name, value in (('left', axis_left), ('right', -axis_right))
                 if value < FULL_STICK_AXIS]
        layers.append({
            'id': 'stick', 'label': 'Remote stick travel',
            'left_text': f'{axis_left:+.2f}', 'right_text': f'{axis_right:+.2f}',
            'status': 'short' if short else 'ok',
            'finding': (f"The stick only reaches {', '.join(short)} partway "
                        '(full is 1.00). Controller hardware or mode.'
                        if short else ''),
        })
        if short:
            software_causes.append('stick')
            recommendations.append(
                'Check the F710: MODE light off, switch on X (XInput). Then '
                'confirm with `ros2 topic echo /joy` that axis 3 reaches '
                '+1.00 and -1.00.')
    else:
        layers.append({'id': 'stick', 'label': 'Remote stick travel',
                       'status': 'missing', 'finding': 'No /joy axis values recorded.'})

    # 2. Command angle published on /ackermann_cmd.
    cmd_left = left.get('command_rad')
    cmd_right = right.get('command_rad')
    mean_cmd = (abs(cmd_left) + abs(cmd_right)) / 2.0
    cmd_asym = abs(abs(cmd_left) - abs(cmd_right)) / mean_cmd > COMMAND_SYMMETRY_FRACTION
    layers.append({
        'id': 'command', 'label': 'Steering command (/ackermann_cmd)',
        'left_text': _fmt_deg(cmd_left), 'right_text': _fmt_deg(abs(cmd_right)),
        'status': 'asymmetric' if cmd_asym else 'ok',
        'finding': ('The two sides are scaled differently before the servo '
                    'conversion: check joy_teleop.yaml and any node remapped '
                    'onto /teleop.' if cmd_asym else
                    f'Full stick asks for {_fmt_deg(mean_cmd)} each way '
                    '(joy_teleop.yaml steering scale).'),
    })
    if cmd_asym:
        software_causes.append('command')

    # 3. Servo command before the clamp.
    expected = {name: gain * angle + offset
                for name, angle in (('left', cmd_left), ('right', cmd_right))}
    measured = {'left': left.get('servo_command'), 'right': right.get('servo_command')}
    pre = {name: measured[name] if _finite(measured[name]) else expected[name]
           for name in expected}
    mismatch = [name for name in expected if _finite(measured[name])
                and abs(measured[name] - expected[name]) > SERVO_MODEL_TOLERANCE]
    layers.append({
        'id': 'servo_command', 'label': 'Servo command before the clamp',
        'left_text': f"{pre['left']:.4f}", 'right_text': f"{pre['right']:.4f}",
        'status': 'mismatch' if mismatch else 'ok',
        'finding': (f"The servo command on the {', '.join(mismatch)} side is "
                    'not gain * angle + offset with the values this wizard '
                    'read: ackermann_to_vesc_node is running different '
                    'parameters. Restart bringup after editing vesc.yaml.'
                    if mismatch else
                    'Measured on /commands/servo/position.'
                    if all(_finite(v) for v in measured.values())
                    else 'Computed from gain and offset (no pre-clamp topic).'),
    })
    if mismatch:
        software_causes.append('servo_model')

    # 4. vesc_driver clamp.
    clamped = {}
    for name, side in (('left', left), ('right', right)):
        value = side.get('servo_clamped')
        if not _finite(value) and limits:
            value = min(max(pre[name], limits[0]), limits[1])
        clamped[name] = value
    clipped = [name for name in clamped if _finite(clamped[name])
               and abs(pre[name] - clamped[name]) > SERVO_TOLERANCE]
    clamp_findings = []
    for name in clipped:
        bound = 'servo_max' if pre[name] > clamped[name] else 'servo_min'
        lost = abs((pre[name] - clamped[name]) / gain)
        clamp_findings.append(
            f'{name} is cut from {pre[name]:.3f} to {clamped[name]:.3f} by '
            f'{bound}, losing {_fmt_deg(lost)}')
    layers.append({
        'id': 'clamp', 'label': 'Servo clamp (servo_min / servo_max)',
        'left_text': f"{clamped['left']:.4f}" if _finite(clamped['left']) else 'n/a',
        'right_text': f"{clamped['right']:.4f}" if _finite(clamped['right']) else 'n/a',
        'status': 'limited' if clipped else 'ok',
        'finding': ('; '.join(clamp_findings) + '. This is a software limit.'
                    if clipped else 'Neither side reaches a servo limit.'),
    })
    if clipped:
        software_causes.append('clamp')

    # 5. Angle the software actually commands, after the clamp.
    commanded = {name: abs((clamped[name] - offset) / gain)
                 if _finite(clamped[name]) else None for name in clamped}
    commanded_asym = (
        all(_finite(v) for v in commanded.values())
        and abs(_deg(commanded['left']) - _deg(commanded['right'])) > COMMANDED_SYMMETRY_DEG)
    layers.append({
        'id': 'commanded', 'label': 'Steering the software commands',
        'left_text': _fmt_deg(commanded['left']),
        'right_text': _fmt_deg(commanded['right']),
        'status': 'asymmetric' if commanded_asym else 'ok',
        'finding': ('Software alone makes the two sides unequal.'
                    if commanded_asym else 'Software commands both sides equally.'),
    })

    # 6. Wheels, measured by the operator.
    measurements = measurements or {}
    physical = {
        'left': _measured_wheel_angle(measurements.get('left_lock')),
        'right': _measured_wheel_angle(measurements.get('right_lock')),
    }
    have_physical = all(_finite(v) for v in physical.values())
    physical_asym = (have_physical and abs(
        _deg(physical['left']) - _deg(physical['right'])) > MEASURED_SYMMETRY_DEG)
    shortfall = []
    if have_physical:
        for name in ('left', 'right'):
            if commanded[name] and physical[name] < SHORTFALL_FRACTION * commanded[name]:
                shortfall.append(name)
    buzz = [name for name in ('left', 'right') if measurements.get(f'buzz_{name}')]
    layers.append({
        'id': 'wheels', 'label': 'Wheels, measured with a phone',
        'left_text': _fmt_deg(physical['left']), 'right_text': _fmt_deg(physical['right']),
        'status': 'missing' if not have_physical
        else 'asymmetric' if physical_asym else 'ok',
        'finding': ('Not measured, so hardware cannot be judged yet.'
                    if not have_physical else
                    f"The wheels turn much less than commanded on the "
                    f"{', '.join(shortfall)} side: something stops them first."
                    if shortfall else ''),
    })
    if circle_angles and all(_finite(circle_angles.get(n)) for n in ('left', 'right')):
        layers.append({
            'id': 'circles', 'label': 'Full lock from the driven circles',
            'left_text': _fmt_deg(abs(circle_angles['left'])),
            'right_text': _fmt_deg(abs(circle_angles['right'])),
            'status': 'info',
            'finding': 'Measured under load; tyre slip makes these a little smaller.',
        })

    software_asym = (commanded_asym or 'stick' in software_causes
                     or 'command' in software_causes)
    if not have_physical:
        verdict = 'software' if software_asym else 'unknown'
    elif not physical_asym and not software_asym:
        verdict = 'symmetric'
    elif physical_asym and not software_asym:
        verdict = 'hardware'
    elif not physical_asym:
        verdict = 'mixed'
    else:
        ratio_left = physical['left'] / commanded['left']
        ratio_right = physical['right'] / commanded['right']
        agree = abs(ratio_left - ratio_right) / max(ratio_left, ratio_right) \
            <= RATIO_AGREEMENT_FRACTION
        verdict = 'software' if agree else 'both'

    summaries = {
        'insufficient': '',
        'symmetric': 'Both sides command and turn the same. Nothing to fix here.',
        'software': ('The difference comes from software settings. '
                     + ('The wheels follow the command, so fixing the '
                        'settings fixes the car.' if have_physical else
                        'Measure the wheels with a phone to rule out hardware too.')),
        'hardware': ('Software commands both sides equally, but the wheels do '
                     'not turn equally: the cause is mechanical (servo horn, '
                     'tie rods, bellcrank or a physical stop).'),
        'both': ('Software commands the sides unequally, and the wheels are '
                 'unequal by a different amount: both software and hardware '
                 'contribute. Fix the software limit first, then re-measure.'),
        'mixed': ('Software commands the sides unequally but the wheels turn '
                  'equally, so the linkage is stopping the larger side early. '
                  'Check for a physical stop or servo strain on that side.'),
        'unknown': ('Software commands both sides equally. Measure the wheels '
                    'with a phone to check the hardware.'),
    }

    reach = symmetric_reach(gain, offset, limits) if limits else None
    if 'clamp' in software_causes and limits:
        midpoint = (limits[0] + limits[1]) / 2.0
        needed = sorted(pre.values())
        recommendations.append(
            f'The steering offset is {offset:.4f}, {abs(offset - midpoint):.3f} '
            f'from the middle ({midpoint:.2f}) of servo_min {limits[0]:.2f} / '
            f'servo_max {limits[1]:.2f}, so one side runs out of servo range '
            'first. Pick one fix: (a) re-centre the servo horn on its splines '
            'so the car drives straight with an offset near '
            f'{midpoint:.2f}, then recalibrate; (b) widen the limits to '
            f'{needed[0]:.3f}-{needed[1]:.3f}, but only after checking by hand, '
            'wheels off the ground, that the linkage still moves and the servo '
            'does not strain there; or (c) cap every steering command at '
            f'{_fmt_deg(reach)} (joy_teleop scale and each driving node\'s '
            f'max_steering_angle at most {reach:.3f} rad).')
    if (_finite(planner_max_rad) and reach is not None and planner_max_rad > reach + 1e-6):
        recommendations.append(
            f'Driving nodes are configured for up to {_fmt_deg(planner_max_rad)} '
            f'({planner_max_rad:.3f} rad) but both sides only reach '
            f'{_fmt_deg(reach)}. Commands past that are clipped on one side '
            'without any warning, so the car turns wider than planned.')
    if verdict in ('hardware', 'both', 'mixed') or shortfall or buzz:
        recommendations.append(
            'Mechanical checklist: set the offset back to the servo midpoint; '
            'pull the horn and refit it as close to 90 degrees as the splines '
            'allow; make both tie rods the same length (calipers); check the '
            'bellcrank sits centred and nothing (bulkhead, driveshaft tunnel, '
            'chassis rib) touches it early on one side; check the servo saver '
            'does not flex more one way.')
    if buzz:
        recommendations.append(
            f"The servo strained at full {' and '.join(buzz)}: the linkage hits "
            'a stop before the servo reaches its command. Reduce servo travel '
            'on that side instead of forcing it, or the servo will overheat.')
    if verdict == 'hardware':
        recommendations.append(
            'To split servo from linkage: remove the horn and check the servo '
            'spline turns equally at full left and full right. If it does not, '
            'the servo or the VESC servo output is at fault (VESC Tool, App '
            'Settings, General: Enable Servo Output), not the linkage.')

    return {
        'verdict': verdict,
        'summary': summaries[verdict],
        'layers': layers,
        'recommendations': recommendations,
        'software_causes': software_causes,
        'symmetric_reach_rad': reach,
        'commanded_rad': commanded,
        'measured_rad': physical,
    }


def latest_sweep(session):
    trials = [
        trial for trial in session.get('trials', [])
        if trial.get('accepted') and trial.get('kind') == 'steering_sweep'
    ]
    return trials[-1] if trials else None


def diagnose_session(session, steering_report=None):
    trial = latest_sweep(session)
    if trial is None:
        return diagnose(None, None, {}, None)
    parameters = trial.get('parameters_at_capture') or session.get(
        'current_parameters', {})
    vehicle = session.get('vehicle', {}) or {}
    limits = trial.get('servo_limits_at_capture') or (
        [vehicle['servo_min'], vehicle['servo_max']]
        if _finite(vehicle.get('servo_min')) and _finite(vehicle.get('servo_max'))
        else None)
    circles = None
    if steering_report:
        sides = steering_report.get('sides', {})
        circles = {name: (sides.get(name) or {}).get('full_lock_angle_rad')
                   for name in ('left', 'right')}
    result = diagnose(
        trial.get('summary', {}).get('sweep'),
        trial.get('measurements'),
        parameters,
        limits,
        vehicle.get('planner_max_steering_rad'),
        circles,
    )
    # The check describes the settings it was recorded with. After a
    # vesc.yaml change it is history, not the car as it is now.
    current = session.get('current_parameters', {})
    changed = [
        name for name in ('steering_angle_to_servo_gain',
                          'steering_angle_to_servo_offset')
        if _finite(parameters.get(name)) and _finite(current.get(name))
        and abs(parameters[name] - current[name]) > 1e-9
    ]
    live_limits = [vehicle.get('servo_min'), vehicle.get('servo_max')]
    if limits and all(_finite(v) for v in live_limits) and any(
            abs(a - b) > 1e-9 for a, b in zip(limits, live_limits)):
        changed.append('servo limits')
    result['stale'] = bool(changed)
    if changed and result['verdict'] != 'insufficient':
        result['summary'] = (
            f"Recorded before {', '.join(changed)} changed; record the "
            'steering range check again to see the car as it is now. '
            + result['summary'])
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
