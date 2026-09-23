import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import calibration_math  # noqa: E402

# Measured 2026-08-24 (docs/hardware-reference.md). Any positive value works.
WHEELBASE = 0.36
# A synthetic "true" servo linkage, deliberately different from the
# configured -1.2135 / 0.5304 so a fit that echoes the baseline fails.
TRUE_GAIN = -1.15
TRUE_OFFSET = 0.52


def _arc_endpoint(steering_angle, arc_length):
    """Oracle: where a kinematic-bicycle rear axle ends up after driving
    arc_length along a constant-steering arc starting on the x axis,
    heading +x. Independent of the R = (s^2 + d^2) / (2d) inversion under
    test: it goes forward from the angle, not backward from the point."""
    radius = WHEELBASE / math.tan(steering_angle)
    theta = arc_length / radius
    return radius * math.sin(theta), radius * (1.0 - math.cos(theta))


def _servo(angle):
    return TRUE_GAIN * angle + TRUE_OFFSET


def _trial(identifier, kind, servo, **fields):
    trial = {
        'id': identifier,
        'kind': kind,
        'accepted': True,
        'summary': {
            'servo': {'median': servo, 'p05': servo, 'p95': servo},
            'odom_yaw_rad': 0.0,
        },
    }
    trial.update(fields)
    return trial


def _session(trials, mode='steering', limits=(0.15, 0.85)):
    vehicle = {}
    if limits:
        vehicle = {'servo_min': limits[0], 'servo_max': limits[1]}
    return {
        'session_id': 'test',
        'mode': mode,
        'current_parameters': {
            'speed_to_erpm_gain': -4614.0,
            'speed_to_erpm_offset': 0.0,
            'steering_angle_to_servo_gain': -1.2135,
            'steering_angle_to_servo_offset': 0.5304,
            'wheelbase': WHEELBASE,
        },
        'vehicle': vehicle,
        'trials': trials,
    }


def _tire_diameters(radius, track):
    """Oracle: inner/outer rear-tire circle diameters for an axle-centre
    radius and a rear track (contact centre to contact centre)."""
    return 2.0 * (radius - track / 2.0), 2.0 * (radius + track / 2.0)


def _full_car_trials(left_angle=0.30, right_angle=-0.28, drift_angle=0.02,
                     left_gain=TRUE_GAIN, right_gain=TRUE_GAIN, track=0.25):
    forward, lateral = _arc_endpoint(drift_angle, 5.0)
    left_radius = WHEELBASE / math.tan(left_angle)
    right_radius = WHEELBASE / math.tan(-right_angle)
    inner, outer = _tire_diameters(left_radius, track)
    return [
        _trial('drift', 'steering_drift', _servo(drift_angle),
               measured_forward_m=forward, measured_lateral_m=lateral),
        _trial('left', 'steering_left',
               left_gain * left_angle + TRUE_OFFSET,
               measurement_method='rear_tires', full_lock=True,
               measured_inner_diameter_m=inner,
               measured_outer_diameter_m=outer),
        _trial('right', 'steering_right',
               right_gain * right_angle + TRUE_OFFSET,
               measurement_method='axle_centre', full_lock=True,
               measured_diameter_m=2.0 * right_radius),
    ]


# --- drift_steering_angle ---------------------------------------------------

@pytest.mark.parametrize('angle', [0.005, 0.02, 0.1, -0.02, -0.1])
def test_drift_angle_recovers_the_arc_that_produced_the_point(angle):
    forward, lateral = _arc_endpoint(angle, 5.0)
    result = calibration_math.drift_steering_angle(WHEELBASE, forward, lateral)
    # Oracle: forward arc simulation above. 1e-9 rad is float noise.
    assert result == pytest.approx(angle, abs=1e-9)


def test_drift_with_no_sideways_offset_is_exactly_straight():
    # Invariant: a straight run means zero curvature.
    assert calibration_math.drift_steering_angle(WHEELBASE, 5.0, 0.0) == 0.0


def test_drift_sign_follows_sideways_offset():
    # Invariant: left positive, right negative (REP 103, as vesc_to_odom).
    left = calibration_math.drift_steering_angle(WHEELBASE, 5.0, 0.2)
    right = calibration_math.drift_steering_angle(WHEELBASE, 5.0, -0.2)
    assert left > 0.0
    assert right == pytest.approx(-left, abs=1e-12)  # rad


@pytest.mark.parametrize('forward,lateral,match', [
    (0.0, 0.0, 'forward distance must be positive'),
    (-5.0, 0.1, 'forward distance must be positive'),
    (5.0, 5.0, 'smaller than the forward distance'),
    (5.0, -5.5, 'smaller than the forward distance'),
    (float('nan'), 0.1, 'finite'),
    (5.0, float('inf'), 'finite'),
])
def test_drift_rejects_bad_measurements(forward, lateral, match):
    with pytest.raises(ValueError, match=match):
        calibration_math.drift_steering_angle(WHEELBASE, forward, lateral)


def test_drift_rejects_nonpositive_wheelbase():
    with pytest.raises(ValueError, match='wheelbase must be positive'):
        calibration_math.drift_steering_angle(0.0, 5.0, 0.1)


# --- circle_radius -----------------------------------------------------------

def test_axle_centre_radius_is_half_the_diameter():
    radius, track = calibration_math.circle_radius(
        {'measurement_method': 'axle_centre', 'measured_diameter_m': 3.1})
    assert radius == pytest.approx(1.55, abs=1e-12)  # m, closed form D/2
    assert track is None


def test_missing_method_defaults_to_axle_centre_for_old_sessions():
    radius, _ = calibration_math.circle_radius({'measured_diameter_m': 2.0})
    assert radius == pytest.approx(1.0, abs=1e-12)  # m


@pytest.mark.parametrize('radius,track', [(1.2, 0.25), (2.5, 0.22), (0.9, 0.3)])
def test_rear_tire_circles_recover_axle_radius_and_track(radius, track):
    inner, outer = _tire_diameters(radius, track)
    got_radius, got_track = calibration_math.circle_radius({
        'measurement_method': 'rear_tires',
        'measured_inner_diameter_m': inner,
        'measured_outer_diameter_m': outer,
    })
    # Oracle: _tire_diameters construction.
    assert got_radius == pytest.approx(radius, abs=1e-12)  # m
    assert got_track == pytest.approx(track, abs=1e-12)  # m


@pytest.mark.parametrize('trial,match', [
    ({'measurement_method': 'rear_tires', 'measured_inner_diameter_m': 3.0,
      'measured_outer_diameter_m': 2.5}, 'outer tire circle must be larger'),
    ({'measurement_method': 'rear_tires', 'measured_inner_diameter_m': 2.5,
      'measured_outer_diameter_m': 2.5}, 'outer tire circle must be larger'),
    ({'measurement_method': 'rear_tires', 'measured_inner_diameter_m': 0.0,
      'measured_outer_diameter_m': 2.5}, 'must be positive'),
    ({'measurement_method': 'axle_centre', 'measured_diameter_m': 0.0},
     'must be positive'),
    ({'measurement_method': 'axle_centre'}, 'must be positive'),
    ({'measurement_method': 'bumper', 'measured_diameter_m': 2.0},
     'Unknown circle measurement method'),
])
def test_circle_radius_rejects_bad_measurements(trial, match):
    with pytest.raises(ValueError, match=match):
        calibration_math.circle_radius(trial)


# --- validate_steering_measurement ------------------------------------------

def test_validate_drift_keeps_signed_offset():
    fields = calibration_math.validate_steering_measurement(
        'steering_drift', {'measured_forward_m': 5.0, 'measured_lateral_m': -0.3})
    assert fields == {'measured_forward_m': 5.0, 'measured_lateral_m': -0.3}


def test_validate_circle_returns_method_and_full_lock():
    fields = calibration_math.validate_steering_measurement('steering_left', {
        'measurement_method': 'rear_tires', 'full_lock': True,
        'measured_inner_diameter_m': 2.1, 'measured_outer_diameter_m': 2.6,
        'unrelated': 'ignored'})
    assert fields == {
        'measurement_method': 'rear_tires', 'full_lock': True,
        'measured_inner_diameter_m': 2.1, 'measured_outer_diameter_m': 2.6}


def test_validate_centre_needs_no_measurement():
    assert calibration_math.validate_steering_measurement(
        'steering_center', {}) == {}


@pytest.mark.parametrize('kind,payload,match', [
    ('steering_drift', {'measured_forward_m': 5.0}, 'Sideways offset'),
    ('steering_drift', {'measured_forward_m': '5', 'measured_lateral_m': 0.1},
     'Forward distance'),
    ('steering_drift', {'measured_forward_m': 5.0, 'measured_lateral_m': 6.0},
     'smaller than the forward distance'),
    ('steering_drift', {'measured_forward_m': 5.0,
                        'measured_lateral_m': float('nan')}, 'Sideways offset'),
    ('steering_left', {'measurement_method': 'axle_centre'},
     'Rear-axle circle diameter'),
    ('steering_right', {'measurement_method': 'axle_centre', 'full_lock': 'yes',
                        'measured_diameter_m': 2.0}, 'full_lock'),
    ('steering_right', {'measurement_method': 'rear_tires',
                        'measured_inner_diameter_m': 2.6,
                        'measured_outer_diameter_m': 2.1},
     'outer tire circle must be larger'),
    ('steering_left', {'measurement_method': 'tape', 'measured_diameter_m': 2.0},
     'Unknown circle measurement method'),
    ('movement', {}, 'Unknown steering capture kind'),
])
def test_validate_rejects_operator_mistakes(kind, payload, match):
    with pytest.raises(ValueError, match=match):
        calibration_math.validate_steering_measurement(kind, payload)


# --- steering_calibration ----------------------------------------------------

def test_fit_recovers_true_linkage_from_drift_and_both_full_locks():
    report = calibration_math.steering_calibration(_session(_full_car_trials()))
    # Oracle: trials were generated from TRUE_GAIN / TRUE_OFFSET.
    assert report['suggested_steering_angle_to_servo_gain'] == \
        pytest.approx(TRUE_GAIN, abs=1e-9)  # servo units per rad
    assert report['suggested_steering_angle_to_servo_offset'] == \
        pytest.approx(TRUE_OFFSET, abs=1e-9)  # servo units
    assert report['status'] == 'good'
    assert report['usable_point_count'] == 3
    for item in report['trial_results']:
        assert item['fit_residual_servo'] == pytest.approx(0.0, abs=1e-9)


def test_full_lock_sides_report_radius_and_angle():
    report = calibration_math.steering_calibration(_session(_full_car_trials()))
    left, right = report['sides']['left'], report['sides']['right']
    # Oracle: R = wheelbase / tan(angle) for the generating angles.
    assert left['full_lock_radius_m'] == pytest.approx(
        WHEELBASE / math.tan(0.30), abs=1e-9)  # m
    assert left['full_lock_angle_rad'] == pytest.approx(0.30, abs=1e-9)
    assert right['full_lock_radius_m'] == pytest.approx(
        WHEELBASE / math.tan(0.28), abs=1e-9)  # m
    assert right['full_lock_angle_rad'] == pytest.approx(-0.28, abs=1e-9)
    assert left['full_lock_count'] == right['full_lock_count'] == 1
    assert report['rear_track_m'] == pytest.approx(0.25, abs=1e-12)  # m


def test_servo_limits_give_max_commandable_angles():
    report = calibration_math.steering_calibration(_session(_full_car_trials()))
    reach = report['max_commandable_steering']
    # Oracle: invert servo = g*angle + o at each limit; g < 0, so servo_min
    # is the left end.
    left = (0.15 - TRUE_OFFSET) / TRUE_GAIN
    right = (0.85 - TRUE_OFFSET) / TRUE_GAIN
    assert reach['left_rad'] == pytest.approx(left, abs=1e-9)  # rad
    assert reach['right_rad'] == pytest.approx(right, abs=1e-9)  # rad
    assert reach['left_min_radius_m'] == pytest.approx(
        WHEELBASE / math.tan(left), abs=1e-9)  # m
    assert reach['right_min_radius_m'] == pytest.approx(
        WHEELBASE / math.tan(-right), abs=1e-9)  # m


def test_no_servo_limits_means_no_reach_claim():
    report = calibration_math.steering_calibration(
        _session(_full_car_trials(), limits=None))
    assert report['max_commandable_steering'] is None
    assert report['servo_limits'] is None


def test_full_lock_on_servo_limit_is_flagged():
    trials = _full_car_trials()
    trials[1]['summary']['servo'] = {'median': 0.1505, 'p05': 0.1505, 'p95': 0.1505}
    report = calibration_math.steering_calibration(_session(trials))
    left = next(i for i in report['trial_results'] if i['kind'] == 'steering_left')
    right = next(i for i in report['trial_results'] if i['kind'] == 'steering_right')
    # Spec: within 0.002 of servo_min (0.15) counts as clamped.
    assert left['at_servo_limit'] is True
    assert right['at_servo_limit'] is False
    assert report['sides']['left']['full_lock_at_servo_limit'] is True


def test_asymmetric_linkage_is_warned():
    report = calibration_math.steering_calibration(_session(
        _full_car_trials(left_gain=-1.3, right_gain=-1.0)))
    # Oracle: each side's gain is fitted through the drift point and that
    # side's circle only; asymmetry = |gl - gr| / mean(|gl|, |gr|).
    left_gain = report['sides']['left']['gain']
    right_gain = report['sides']['right']['gain']
    assert report['side_gain_asymmetry'] == pytest.approx(
        abs(left_gain - right_gain) / ((abs(left_gain) + abs(right_gain)) / 2.0),
        abs=1e-12)
    assert report['side_gain_asymmetry'] > 0.10
    assert any('Left and right steering gains differ' in w
               for w in report['warnings'])


def test_symmetric_linkage_is_not_warned():
    report = calibration_math.steering_calibration(_session(_full_car_trials()))
    assert report['side_gain_asymmetry'] == pytest.approx(0.0, abs=1e-9)
    assert not any('differ' in w for w in report['warnings'])


def test_inconsistent_rear_track_is_warned():
    trials = _full_car_trials(track=0.25)
    extra_radius = WHEELBASE / math.tan(0.15)
    inner, outer = _tire_diameters(extra_radius, 0.29)
    trials.append(_trial('left2', 'steering_left', _servo(0.15),
                         measurement_method='rear_tires', full_lock=False,
                         measured_inner_diameter_m=inner,
                         measured_outer_diameter_m=outer))
    report = calibration_math.steering_calibration(_session(trials))
    # Spec: tracks 0.25 and 0.29 m differ by 4 cm > 3 cm threshold.
    assert any('Rear track widths' in w for w in report['warnings'])


def test_consistent_rear_track_is_not_warned():
    trials = _full_car_trials(track=0.25)
    extra_radius = WHEELBASE / math.tan(0.15)
    inner, outer = _tire_diameters(extra_radius, 0.26)
    trials.append(_trial('left2', 'steering_left', _servo(0.15),
                         measurement_method='rear_tires', full_lock=False,
                         measured_inner_diameter_m=inner,
                         measured_outer_diameter_m=outer))
    report = calibration_math.steering_calibration(_session(trials))
    assert not any('Rear track widths' in w for w in report['warnings'])


def test_centre_capture_that_echoes_the_offset_is_explained():
    trials = [_trial('centre', 'steering_center', 0.5304)] + _full_car_trials()[1:]
    report = calibration_math.steering_calibration(_session(trials))
    assert any('only reads the current setting back' in w
               for w in report['warnings'])


def test_missing_drift_test_lowers_confidence_in_offset():
    report = calibration_math.steering_calibration(
        _session(_full_car_trials()[1:]))
    assert report['status'] == 'low'
    assert any('No straight-line drift test' in w for w in report['warnings'])


def test_bad_stored_measurement_is_unusable_not_fitted():
    trials = _full_car_trials()
    trials[1]['measured_outer_diameter_m'] = 1.0  # smaller than inner
    report = calibration_math.steering_calibration(_session(trials))
    left = next(i for i in report['trial_results'] if i['kind'] == 'steering_left')
    assert left['usable'] is False
    assert report['usable_point_count'] == 2


# --- reports -----------------------------------------------------------------

def test_steering_only_report_has_no_movement():
    report = calibration_math.build_report(_session(_full_car_trials()))
    assert report['movement'] is None
    assert set(report['parameter_suggestions']) == {
        'steering_angle_to_servo_gain', 'steering_angle_to_servo_offset'}
    assert report['overall_status'] == 'ready'
    markdown = calibration_math.report_markdown(report)
    assert '## Steering calibration' in markdown
    assert '## Movement calibration' not in markdown
    assert 'Full lock left' in markdown
    assert 'Servo limits allow at most' in markdown


def test_combined_report_still_has_both_parts():
    report = calibration_math.build_report(
        _session(_full_car_trials(), mode='movement_steering'))
    assert report['movement'] is not None
    assert report['steering'] is not None
    assert 'speed_to_erpm_gain' in report['parameter_suggestions']
    assert 'steering_angle_to_servo_gain' in report['parameter_suggestions']
