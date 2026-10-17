"""Multi-setting circle calibration: the 2026-10-16 procedure on car 2.

Recorded measurement used as an oracle throughout: car 2's steering
calibration of 2026-10-16 (docs/hardware-reference.md, table "Steering
calibration, 2026-10-16"). Inner rear-tire diameters, tape-measured, at the
servo value each full-stick circle was driven at. Every angle below is
recomputed here from those diameters with the bicycle model
(angle = atan(wheelbase / ((inner + track) / 2))), not read from the code.

    python3 -m pytest src/odom_calibration/test/test_steering_curve.py -v
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import calibration_math, steering_curve  # noqa: E402

WHEELBASE = 0.36     # m, measured 2026-08-24
TRACK = 0.27         # m, rear tire centre to centre, from the 2026-10-10 circles
CENTRE = 0.5904      # servo where car 2 drives straight (drive-straight run)

# (side, servo, inner diameter m), 2026-10-16.
RIGHT_SWEEP = [('right', 0.85, 1.81), ('right', 0.90, 1.40),
               ('right', 0.95, 1.23), ('right', 1.00, 1.20)]
LEFT_CIRCLES = [('left', 0.1778, 1.02), ('left', 0.318, 1.60), ('left', 0.1744, 1.02)]


def angle_from_inner(inner):
    return math.atan(WHEELBASE / ((inner + TRACK) / 2.0))


def _item(side, servo, inner):
    angle = angle_from_inner(inner)
    return {'servo': servo, 'angle_rad': angle if side == 'left' else -angle,
            'radius_m': (inner + TRACK) / 2.0, 'trial_id': f'{side}-{servo}'}


# --- inner-tire circles ------------------------------------------------------

def test_inner_tire_radius_is_inner_plus_track_over_two():
    trial = {'measurement_method': 'inner_tire', 'measured_inner_diameter_m': 1.33}
    radius, track = calibration_math.circle_radius(trial, 0.27)
    assert radius == pytest.approx((1.33 + 0.27) / 2.0, abs=1e-12)   # m
    assert track is None   # no outer circle, so no free track measurement


@pytest.mark.parametrize('track', [None, 0.0, -0.27, float('nan')])
def test_inner_tire_without_a_usable_track_is_refused(track):
    trial = {'measurement_method': 'inner_tire', 'measured_inner_diameter_m': 1.33}
    with pytest.raises(ValueError, match='rear track is not set'):
        calibration_math.circle_radius(trial, track)


@pytest.mark.parametrize('inner', [0.0, -1.0, float('nan'), float('inf'), None])
def test_inner_tire_rejects_a_bad_diameter(inner):
    trial = {'measurement_method': 'inner_tire', 'measured_inner_diameter_m': inner}
    with pytest.raises(ValueError, match='Inner tire diameter must be positive'):
        calibration_math.circle_radius(trial, 0.27)


def test_validate_inner_tire_keeps_only_the_inner_diameter():
    fields = calibration_math.validate_steering_measurement('steering_right', {
        'measurement_method': 'inner_tire', 'full_lock': True,
        'measured_inner_diameter_m': 1.4, 'measured_outer_diameter_m': 9.9})
    assert fields == {'measurement_method': 'inner_tire', 'full_lock': True,
                      'measured_inner_diameter_m': 1.4}


def test_validate_inner_tire_needs_the_diameter():
    with pytest.raises(ValueError, match='Inner tire circle diameter must be a positive'):
        calibration_math.validate_steering_measurement(
            'steering_left', {'measurement_method': 'inner_tire', 'full_lock': True})


# --- grouping and the stop ---------------------------------------------------

def test_circles_at_one_setting_merge_and_others_do_not():
    items = [_item(*c) for c in LEFT_CIRCLES]
    points = steering_curve.group_side(items, CENTRE)
    # 0.1778 and 0.1744 are 0.0034 apart (one setting); 0.318 is another.
    assert [p['count'] for p in points] == [1, 2]
    assert points[1]['servo'] == pytest.approx((0.1778 + 0.1744) / 2.0, abs=1e-12)
    assert points[0]['travel'] < points[1]['travel']


@pytest.mark.parametrize('gap,merged', [(0.0099, True), (0.0101, False)])
def test_group_tolerance_boundary(gap, merged):
    items = [_item('right', 0.90, 1.40), _item('right', 0.90 + gap, 1.40)]
    points = steering_curve.group_side(items, CENTRE)
    assert (len(points) == 1) is merged


def test_the_right_side_levels_off_after_servo_0_95():
    curve = steering_curve.side_curve(
        [_item(*c) for c in RIGHT_SWEEP], CENTRE, 1.00, (0.15, 1.00))
    added = [s['added_deg'] for s in curve['steps']]
    expected = [math.degrees(angle_from_inner(b) - angle_from_inner(a))
                for (_, _, a), (_, _, b) in zip(RIGHT_SWEEP, RIGHT_SWEEP[1:])]
    assert added == pytest.approx(expected, abs=1e-9)        # deg
    assert expected[-1] < steering_curve.STOP_GAIN_DEG < expected[-2]
    assert curve['stop']['servo'] == pytest.approx(0.95, abs=1e-12)
    assert curve['stop']['beyond_servo'] == pytest.approx(1.00, abs=1e-12)
    # Reach stops at the stop, not at the further 1.00 circle.
    assert curve['reach_deg'] == pytest.approx(
        math.degrees(angle_from_inner(1.23)), abs=1e-9)
    assert curve['still_gaining_at_limit'] is False


def _two_points(added_deg, span):
    # Travel from 0 so the span is exact in floating point.
    inner = {'servo': CENTRE, 'travel': 0.0, 'angle_deg': 23.0}
    outer = {'servo': CENTRE + span, 'travel': span, 'angle_deg': 23.0 + added_deg}
    return [inner, outer]


@pytest.mark.parametrize('added,flat', [
    (1.0, False),       # exactly STOP_GAIN_DEG: still gaining
    (0.99, True),
    (0.0, True),
    (-0.5, True),       # turned less with more servo: measuring noise at a stop
])
def test_stop_threshold_boundary(added, flat):
    _, stop = steering_curve.steps_and_stop(_two_points(added, 0.05))
    assert (stop is not None) is flat


@pytest.mark.parametrize('span,judged', [(0.0199, False), (0.02, True)])
def test_a_step_too_short_is_not_judged(span, judged):
    steps, stop = steering_curve.steps_and_stop(_two_points(0.0, span))
    assert steps[0]['flat'] is judged
    assert (stop is not None) is judged


def test_still_gaining_at_the_limit_is_flagged():
    # Trial 1 of 2026-10-16: servo_max 0.90, the right side still turning.
    curve = steering_curve.side_curve(
        [_item(*c) for c in RIGHT_SWEEP[:2]], CENTRE, 0.90, (0.15, 0.90))
    assert curve['stop'] is None
    assert curve['still_gaining_at_limit'] is True


def test_gaining_but_short_of_the_limit_is_not_flagged():
    # Circles up to 0.90 while servo_max is already 0.95: nothing was driven
    # at the limit, so nothing says the limit is where it still gains.
    curve = steering_curve.side_curve(
        [_item(*c) for c in RIGHT_SWEEP[:2]], CENTRE, 0.95, (0.15, 0.95))
    assert curve['still_gaining_at_limit'] is False


def test_one_circle_is_not_a_trend():
    curve = steering_curve.side_curve(
        [_item(*RIGHT_SWEEP[0])], CENTRE, 0.85, (0.15, 0.85))
    assert curve['still_gaining_at_limit'] is False
    assert curve['stop'] is None


def test_empty_side_has_no_reach():
    curve = steering_curve.side_curve([], CENTRE, 0.95, (0.15, 0.95))
    assert curve['points'] == [] and curve['reach_deg'] is None


# --- suggestions -------------------------------------------------------------

def test_servo_max_suggestion_is_the_last_circle_before_the_stop():
    curve = steering_curve.side_curve(
        [_item(*c) for c in RIGHT_SWEEP], CENTRE, 1.00, (0.15, 1.00))
    assert steering_curve.servo_limit_suggestion(curve, CENTRE, 1.00) == \
        pytest.approx(0.95, abs=1e-12)


def test_no_servo_limit_change_when_already_at_the_stop():
    curve = steering_curve.side_curve(
        [_item(*c) for c in RIGHT_SWEEP], CENTRE, 0.95, (0.15, 0.95))
    assert steering_curve.servo_limit_suggestion(curve, CENTRE, 0.95) is None


def test_no_servo_limit_suggestion_without_a_stop():
    curve = steering_curve.side_curve(
        [_item(*c) for c in RIGHT_SWEEP[:2]], CENTRE, 0.90, (0.15, 0.90))
    assert steering_curve.servo_limit_suggestion(curve, CENTRE, 0.90) is None


def test_a_stop_on_the_other_side_of_centre_is_not_a_limit():
    curve = {'stop': {'servo': 0.40, 'beyond_servo': 0.35, 'added_deg': 0.1}}
    assert steering_curve.servo_limit_suggestion(curve, CENTRE, 0.95) is None


def test_stick_scale_is_inside_what_both_sides_reach():
    left = math.degrees(angle_from_inner(1.02))    # 29.2 deg
    right = math.degrees(angle_from_inner(1.23))   # 25.6 deg
    scale = steering_curve.stick_scale_suggestion(left, right, -0.78, CENTRE, (0.15, 0.95))
    # floor(min(29.2, 25.6) - 0.5) = 25 deg; -0.78 / 0.5904 / 0.95 clips at 26.4 deg.
    assert scale == pytest.approx(math.radians(25.0), abs=1e-12)


@pytest.mark.parametrize('reach,degrees', [(25.5, 25), (25.49, 24), (1.4, None)])
def test_stick_scale_margin_boundary(reach, degrees):
    scale = steering_curve.stick_scale_suggestion(reach, 40.0, -0.78, CENTRE, None)
    assert scale == (None if degrees is None else pytest.approx(math.radians(degrees), abs=1e-12))


def test_stick_scale_is_lowered_when_the_servo_limits_would_clip_it():
    # The 2026-10-10 settings: -1.2135 / 0.5904 / 0.85 clip the right at 12.25 deg.
    scale = steering_curve.stick_scale_suggestion(29.2, 25.6, -1.2135, CENTRE, (0.15, 0.85))
    clip = math.degrees((0.85 - CENTRE) / 1.2135)
    assert scale == pytest.approx(math.radians(math.floor(clip)), abs=1e-12)


def test_stick_scale_needs_both_sides():
    assert steering_curve.stick_scale_suggestion(29.2, None, -0.78, CENTRE, None) is None


# --- the check circle --------------------------------------------------------

def test_the_2026_10_16_check_circle_passes():
    # Full stick 25 deg under -0.78 / 0.5904 sent servo 0.9305 right; measured 1.33 m.
    servo = CENTRE + 0.78 * math.radians(25.0)
    result = steering_curve.check_circle(-angle_from_inner(1.33), servo, -0.78, CENTRE)
    assert result['commanded_deg'] == pytest.approx(25.0, abs=1e-9)
    assert result['error_deg'] == pytest.approx(
        math.degrees(angle_from_inner(1.33)) - 25.0, abs=1e-9)
    assert result['passed'] is True


@pytest.mark.parametrize('error,passed', [(1.0, True), (-1.0, True), (1.01, False), (-1.01, False)])
def test_check_tolerance_boundary(error, passed):
    measured = math.radians(25.0 + error)
    servo = CENTRE - 0.78 * math.radians(25.0)
    assert steering_curve.check_circle(measured, servo, -0.78, CENTRE)['passed'] is passed


def test_the_old_gain_fails_the_check():
    # 2026-10-10: gain -1.2135 believed servo 0.1778 was 19.5 deg; it was 29.2.
    result = steering_curve.check_circle(angle_from_inner(1.02), 0.1778, -1.2135, CENTRE)
    assert result['commanded_deg'] == pytest.approx(
        math.degrees((CENTRE - 0.1778) / 1.2135), abs=1e-9)
    assert result['passed'] is False


@pytest.mark.parametrize('gain', [0.0, float('nan'), None])
def test_check_without_a_gain_is_none(gain):
    assert steering_curve.check_circle(0.4, 0.3, gain, CENTRE) is None


def test_expected_inner_diameter_inverts_the_measurement():
    angle = angle_from_inner(1.33)
    assert steering_curve.expected_inner_diameter(angle, WHEELBASE, TRACK) == \
        pytest.approx(1.33, abs=1e-12)   # m
    assert steering_curve.expected_inner_diameter(-angle, WHEELBASE, TRACK) == \
        pytest.approx(1.33, abs=1e-12)


@pytest.mark.parametrize('angle', [0.0, float('nan'), 1.5])
def test_expected_inner_diameter_edges(angle):
    # Straight has no circle; at 1.5 rad the axle circle is inside the track.
    assert steering_curve.expected_inner_diameter(angle, WHEELBASE, TRACK) is None


# --- the whole 2026-10-16 session through the report ------------------------

def _trial(identifier, kind, servo, inner, gain, limits):
    return {
        'id': identifier, 'kind': kind, 'accepted': True,
        'summary': {'servo_active': {'median': servo, 'p05': servo, 'p95': servo,
                                     'count': 100}},
        'measurement_method': 'inner_tire', 'full_lock': True,
        'measured_inner_diameter_m': inner,
        'parameters_at_capture': {'steering_angle_to_servo_gain': gain,
                                  'steering_angle_to_servo_offset': CENTRE},
        'servo_limits_at_capture': list(limits),
    }


def _session_2026_10_16(current_max=1.00):
    trials = [{
        'id': 'drift', 'kind': 'steering_drift', 'accepted': True,
        'summary': {'servo_active': {'median': CENTRE, 'p05': CENTRE, 'p95': CENTRE,
                                     'count': 100}},
        'measured_forward_m': 5.0, 'measured_lateral_m': 0.0,
    }]
    trials += [
        _trial('l-old', 'steering_left', 0.1778, 1.02, -1.2135, (0.15, 0.85)),
        _trial('r-085', 'steering_right', 0.85, 1.81, -1.2135, (0.15, 0.85)),
        _trial('l-080', 'steering_left', 0.318, 1.60, -0.80, (0.15, 0.85)),
        _trial('r-090', 'steering_right', 0.90, 1.40, -0.80, (0.15, 0.90)),
        _trial('l-052', 'steering_left', 0.1744, 1.02, -0.80, (0.15, 0.90)),
        _trial('r-095', 'steering_right', 0.95, 1.23, -0.80, (0.15, 0.95)),
        _trial('r-100', 'steering_right', 1.00, 1.20, -0.80, (0.15, 1.00)),
    ]
    return {
        'mode': 'steering',
        'current_parameters': {'steering_angle_to_servo_gain': -0.80,
                               'steering_angle_to_servo_offset': CENTRE,
                               'wheelbase': WHEELBASE},
        'vehicle': {'servo_min': 0.15, 'servo_max': current_max, 'rear_track_m': TRACK},
        'trials': trials,
    }


def test_the_line_goes_through_a_drift_run_that_curved():
    # Synthetic linear linkage, gain -0.8 / offset 0.58. The drift run curved
    # 0.02 rad left, so its servo is not the offset: the fit must recover the
    # offset from it exactly, not take the drift servo as the centre.
    gain, offset = -0.8, 0.58
    drift_angle = 0.02
    radius = WHEELBASE / math.tan(drift_angle)
    forward, lateral = radius * math.sin(5.0 / radius), radius * (1 - math.cos(5.0 / radius))
    session = {
        'mode': 'steering',
        'current_parameters': {'steering_angle_to_servo_gain': -0.78,
                               'steering_angle_to_servo_offset': 0.59, 'wheelbase': WHEELBASE},
        'vehicle': {'servo_min': 0.0, 'servo_max': 1.0, 'rear_track_m': TRACK},
        'trials': [{'id': 'drift', 'kind': 'steering_drift', 'accepted': True,
                    'summary': {'servo_active': {'median': gain * drift_angle + offset,
                                                 'count': 100}},
                    'measured_forward_m': forward, 'measured_lateral_m': lateral}],
    }
    for kind, angle in (('steering_left', 0.42), ('steering_right', -0.40)):
        inner = 2.0 * WHEELBASE / math.tan(abs(angle)) - TRACK
        session['trials'].append(_trial(kind, kind, gain * angle + offset, inner,
                                        -0.78, (0.0, 1.0)))
    report = calibration_math.steering_calibration(session)
    assert report['suggested_steering_angle_to_servo_gain'] == pytest.approx(gain, abs=1e-9)
    assert report['suggested_steering_angle_to_servo_offset'] == pytest.approx(offset, abs=1e-9)


def test_the_session_fits_through_the_drive_straight_point_without_the_stopped_circle():
    report = calibration_math.steering_calibration(_session_2026_10_16())
    # Oracle: closed-form least squares through (0, CENTRE) over every circle
    # except the one past the stop (servo 1.00).
    fitted = [(s, i, side) for side, s, i in LEFT_CIRCLES + RIGHT_SWEEP[:3]]
    xs = [angle_from_inner(i) * (1 if side == 'left' else -1) for _, i, side in fitted]
    ys = [s - CENTRE for s, _, _ in fitted]
    gain = sum(x * y for x, y in zip(xs, ys)) / sum(x * x for x in xs)
    assert report['suggested_steering_angle_to_servo_gain'] == pytest.approx(gain, abs=1e-12)
    assert report['suggested_steering_angle_to_servo_offset'] == pytest.approx(CENTRE, abs=1e-12)
    assert report['centre_source'] == 'drive_straight'
    beyond = [i['trial_id'] for i in report['trial_results'] if i.get('beyond_stop')]
    assert beyond == ['r-100']


def test_the_session_suggests_servo_max_0_95_and_a_25_degree_stick():
    report = calibration_math.steering_calibration(_session_2026_10_16())
    assert report['limit_suggestions']['servo_max'] == pytest.approx(0.95, abs=1e-12)
    assert 'servo_min' not in report['limit_suggestions']
    assert report['limit_suggestions']['steering_stick_scale'] == \
        pytest.approx(math.radians(25.0), abs=1e-12)
    assert any('right steering levels off' in w for w in report['warnings'])


def test_left_runs_towards_servo_min_with_a_negative_gain():
    report = calibration_math.steering_calibration(_session_2026_10_16())
    assert report['curves']['left']['limit_name'] == 'servo_min'
    assert report['curves']['right']['limit_name'] == 'servo_max'


def test_the_old_gain_circles_fail_their_check():
    report = calibration_math.steering_calibration(_session_2026_10_16())
    checks = {i['trial_id']: i['check'] for i in report['trial_results'] if i.get('check')}
    assert checks['l-old']['passed'] is False     # 19.5 commanded, 29.2 measured
    assert report['calibration_confirmed'] is False


def test_check_circles_under_the_new_values_confirm_the_calibration():
    session = _session_2026_10_16(current_max=0.95)
    servo_r = CENTRE + 0.78 * math.radians(25.0)
    servo_l = CENTRE - 0.78 * math.radians(25.0)
    session['trials'] += [
        _trial('l-check', 'steering_left', servo_l, 1.33, -0.78, (0.15, 0.95)),
        _trial('r-check', 'steering_right', servo_r, 1.33, -0.78, (0.15, 0.95)),
    ]
    report = calibration_math.steering_calibration(session)
    assert report['sides']['left']['check']['passed'] is True
    assert report['sides']['right']['check']['passed'] is True
    assert report['calibration_confirmed'] is True


def test_a_report_reaches_the_new_suggestions_through_build_report():
    session = _session_2026_10_16()
    report = calibration_math.build_report(session)
    assert report['parameter_suggestions']['servo_max'] == pytest.approx(0.95, abs=1e-12)
    assert 'steering_stick_scale' in report['parameter_suggestions']
