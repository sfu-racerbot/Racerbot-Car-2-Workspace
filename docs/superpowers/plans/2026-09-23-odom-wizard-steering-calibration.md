# Steering calibration + SFU Racerbot redesign for the odom wizard — Implementation Plan

> **For agentic workers:** executed with opencode-driven development (opencode v2 CLI driven by hand, one git worktree). Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the odom wizard's thin steering step into a real steering calibration (straight-line drift test for the true centre, left/right circle tests with two measuring methods, full-lock and per-side analysis, servo-limit reach, a steering-only mode) and redesign its Web UI in the sfuracerbot.ca "Constructor in daylight" identity.

**Architecture:** All new math stays in the ROS-free `calibration_math.py` / `session_store.py` so it is tested without `rclpy`. `calibration_node.py` only wires those helpers to the HTTP API and adds a live capture-progress readout. The browser UI (`web/index.html`, `web/style.css`, `web/wizard.js`) is rewritten as plain HTML/CSS/JS served by the existing Tornado static handler, with the team's fonts vendored so it works at a track with no internet.

**Tech Stack:** Python 3.12, ROS 2 Jazzy (`rclpy`, Tornado), pytest, vanilla JS (no framework, no build step), CSS custom properties.

**Spec:** this document (design decisions below are the spec). Background: `src/odom_calibration/README.md`, `docs/odom-calibration.md`, and the website design system in `sfu-racerbot/racerbot-website` `static/css/main.css`.

## Design decisions (the spec)

1. **Why the old steering step is weak.** It fits `servo = gain·angle + offset` from one left circle, one right circle, and a "centred wheels" capture. With the stick neutral, the recorded servo value *is* `gain·0 + configured_offset`, so the centre capture only reads the current offset back — it measures nothing. And `/sensors/servo_position_command` is published by `vesc_driver` **after** clamping to `servo_min 0.15` / `servo_max 0.85` (`src/f1tenth_system/vesc/vesc_driver/src/vesc_driver.cpp:331-340`), while full stick asks for ±0.34 rad (`joy_teleop.yaml`), which clamps on both sides with today's values — so full-lock circles measure the car's *real* tightest turn at the servo limit, which the report should say.
2. **Straight-line drift test (new capture kind `steering_drift`).** Put the rear-axle centre on a taped lane line, pointing along it; drive ~5 m forward with the steering stick released; stop; measure the forward distance along the line `s` and the sideways offset `d` of the rear-axle centre (left positive). The constant-curvature path tangent to the line through both points has `R = (s² + d²) / (2d)`, so the true steering angle at that servo value is `atan(wheelbase / R)`. This is the real centre measurement.
3. **Circle tests, two ways to measure.** `axle_centre`: the diameter of the circle traced by the rear-axle centre (as today). `rear_tires` (new): diameters of the circles traced by the inner and outer rear tires; the axle centre is midway, so `R = (inner + outer) / 4` exactly, and `(outer − inner) / 2` is the rear track — a free consistency check across trials. Each circle is flagged `full_lock` (stick held fully over) or not.
4. **Report additions.** Per side: full-lock rear-axle radius and angle, whether the full-lock servo value sits on a servo limit; left-only and right-only gains and their asymmetry (warn above 10%); median rear track (warn if trials disagree by more than 3 cm); the maximum steering angle the servo limits allow with the suggested values, and the matching minimum turning radius per side. A "centred wheels" capture that equals the configured offset (within 0.002) gets a warning explaining it carries no information. The suggestion is still one straight line, because `vesc_to_odom`/`ackermann_to_vesc` only support a linear map.
5. **Modes.** `movement` (speed only), `movement_steering`, and new `steering` (steering only: preflight → steering → report). Stationary/movement captures are refused in steering-only sessions; steering captures are refused in speed-only sessions.
6. **Servo limits.** New node parameters `servo_min: 0.15`, `servo_max: 0.85` (must match `vesc.yaml`), stored in the session's `vehicle` block; the math reads them from there.
7. **Live progress.** While a capture runs, the snapshot carries `capture_progress = {odom_distance_m, odom_yaw_rad}` integrated incrementally, so the circle screen can show "0.8 turns so far". Odom yaw is itself computed from the servo command and the *current* steering values, so the UI labels it an estimate.
8. **Redesign.** Themed on sfuracerbot.ca ("Constructor in daylight"): white paper + navy `#0c1e3a` + blue `#1d5f9e` + light blue `#7fb2e5` (on navy only), Saira Condensed uppercase headings, Public Sans body, IBM Plex Mono for live numbers only, navy top bar with a checker strip on its bottom edge, sheared-corner buttons, race-plate number badges, the 3:2:1 sector rule, a timing-tower step list. One memorable element: a live top-down steering diagram plus a servo-vs-angle fit chart on the steering and report screens. De-AI the copy: plain sentence-case instructions, no slogans, no puns, no emoji, no "→" in buttons, no middle-dot meta strings, no uppercase eyebrow above every heading.

## Global Constraints

- Canadian English in all user-facing text (tire, centre, metre, colour, labelled, travelled). Identifiers use the same spelling: `rear_tires`, `axle_centre`.
- `odom_calibration` stays read-only: it must create **no ROS publishers**. Do not add any.
- New logic goes in `calibration_math.py` or `session_store.py` and must import without `rclpy`. Tests import only those modules.
- Test verification command, run from the repo root: `source /opt/ros/jazzy/setup.bash && python3 -m pytest src/odom_calibration/test -q` — it must collect every test (`grep -c '^def test' src/odom_calibration/test/*.py` counts the functions) with zero collection errors. Never use `--continue-on-collection-errors`.
- **Hard rule 1 (repo `TEST_QUALITY_STANDARDS.md`):** every new test must be seen to fail against a broken implementation before it is done: stub the unit (`return <expected constant>`, `raise NotImplementedError`, or the listed mutation), run only that test, see it fail, revert, see it pass. Record each mutation and its result in your RESULT block.
- **Hard rule 2:** never weaken, skip, or delete an existing test to make the suite pass. If an existing test fails, fix the code, or stop and report it.
- Every expected value in a test traces to a closed form recomputed in the test, an invariant, or a cited spec — say which in a comment. `pytest.approx` always has an explicit `abs=` with a unit comment.
- Match the surrounding code style (4-space Python, single quotes, existing docstring density). No new third-party dependencies (Python or JS). No CDN or Google Fonts links: the UI must work offline.
- Do not touch files outside `src/odom_calibration/` and `docs/odom-calibration.md`.

## Review Focus

- A steering-only session (`mode: 'steering'`) must produce a report and Markdown with **no** movement section and no crash (`report['movement'] is None`). Covered by Task 1 `test_steering_only_report_has_no_movement`.
- A drift run with zero sideways offset must give exactly 0 rad, and a leftward/rightward offset must give a positive/negative angle — a sign slip here flips the offset correction. Covered by Task 1 drift tests.
- Operator typos: outer tire circle smaller than inner, sideways offset ≥ forward distance, missing values, strings, `NaN` — must be rejected at accept time with a clear message, and never produce a fit point. Covered by Task 1 validation tests.
- Old saved sessions (created before this change) with `steering_center` trials and no `measurement_method` field must still compute (default `axle_centre`). Covered by the existing test `test_left_right_and_center_fit_steering_parameters`, which must still pass unchanged.
- The UI on a phone at the track: the live panel, the record/stop button and the accept form must be usable at 375 px wide with no horizontal scrolling. Checked in Task 3's visual verification.

---

### Task 1: Steering math — drift test, tire-circle method, per-side analysis, steering-only reports

**Files:**
- Modify: `src/odom_calibration/odom_calibration/calibration_math.py` — replace everything from `def steering_calibration(session: dict):` up to (not including) `def _format_number(value, digits=4):` with the code block in Step 3; then replace the body of `report_markdown` as in Step 4.
- Create: `src/odom_calibration/test/test_steering_math.py`

**Interfaces:**
- Produces (used by Tasks 2–3):
  - `STEERING_KINDS = ('steering_center', 'steering_drift', 'steering_left', 'steering_right')`
  - `CIRCLE_METHODS = ('axle_centre', 'rear_tires')`
  - `drift_steering_angle(wheelbase: float, forward_m: float, lateral_m: float) -> float` (rad, left positive; raises `ValueError`)
  - `circle_radius(trial: dict) -> tuple[float, float | None]` (rear-axle radius m, rear track m or None; raises `ValueError`)
  - `validate_steering_measurement(kind: str, payload: dict) -> dict` (fields to store on the trial; raises `ValueError`)
  - `steering_calibration(session) -> dict` gains keys `servo_limits`, `max_commandable_steering` (`{left_rad, right_rad, left_min_radius_m, right_min_radius_m}` or None), `sides` (`{'left'|'right': {circle_count, full_lock_count, full_lock_radius_m, full_lock_angle_rad, full_lock_at_servo_limit, gain}}`), `side_gain_asymmetry`, `rear_track_m`; each usable `trial_results` item gains `fit_residual_servo`; circle items gain `measurement_method`, `full_lock`, `rear_axle_radius_m`, optional `rear_track_m`, optional `at_servo_limit`.
  - `MODES_WITH_MOVEMENT = ('movement', 'movement_steering')`, `MODES_WITH_STEERING = ('movement_steering', 'steering')`
  - `build_report(session)`: `movement` is `None` in `steering` mode; `parameter_suggestions` holds only the keys for the parts computed.
- Trial fields read from the session (written by Task 2's accept handler): drift → `measured_forward_m`, `measured_lateral_m`; circle → `measurement_method`, `full_lock`, and either `measured_diameter_m` or `measured_inner_diameter_m` + `measured_outer_diameter_m`. Servo limits are read from `session['vehicle']['servo_min']` / `['servo_max']`.

- [ ] **Step 1: Write the failing tests** in `src/odom_calibration/test/test_steering_math.py`:

```python
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
```

- [ ] **Step 2: Run the new tests to verify they fail**

Run: `source /opt/ros/jazzy/setup.bash && python3 -m pytest src/odom_calibration/test/test_steering_math.py -q`
Expected: FAIL (AttributeError: module has no attribute `drift_steering_angle`, etc.).

- [ ] **Step 3: Replace the steering section of `calibration_math.py`.** Delete everything from the line `def steering_calibration(session: dict):` down to, but not including, `def _format_number(value, digits=4):`, and insert exactly this code in its place (it includes the new `steering_calibration` and `build_report`):

```python
STEERING_KINDS = (
    'steering_center',
    'steering_drift',
    'steering_left',
    'steering_right',
)
CIRCLE_METHODS = ('axle_centre', 'rear_tires')
# A servo reading this close to servo_min/servo_max is treated as clamped.
SERVO_LIMIT_TOLERANCE = 0.002
# A "centred wheels" capture whose servo value is this close to the configured
# offset carries no information: with the stick neutral the servo command is
# computed *from* that offset, so the capture just reads it back.
CENTRE_ECHO_TOLERANCE = 0.002
# Left/right gains that differ by more than this fraction mean one straight
# line does not describe the servo linkage well.
ASYMMETRY_WARNING_FRACTION = 0.10
# Rear-tire track widths from different circles that disagree by more than
# this suggest a marking or measuring error.
TRACK_SPREAD_WARNING_M = 0.03


def drift_steering_angle(wheelbase, forward_m, lateral_m):
    """Steering angle from a straight-line drift test.

    The rear-axle centre starts on a lane line, heading along it. After the
    run it sits ``forward_m`` along the line and ``lateral_m`` off it (left
    positive). A constant-curvature path tangent to the line through both
    points has radius R = (s^2 + d^2) / (2 d); the kinematic bicycle model
    then gives steering = atan(wheelbase / R).
    """
    for name, value in (
            ('wheelbase', wheelbase),
            ('forward distance', forward_m),
            ('lateral offset', lateral_m)):
        if not _finite(value):
            raise ValueError(f'{name} must be a finite number')
    if wheelbase <= 0.0:
        raise ValueError('wheelbase must be positive')
    if forward_m <= 0.0:
        raise ValueError('forward distance must be positive')
    if abs(lateral_m) >= forward_m:
        raise ValueError(
            'lateral offset must be smaller than the forward distance; '
            'the drift test is for a nearly straight run')
    curvature = 2.0 * lateral_m / (forward_m ** 2 + lateral_m ** 2)
    return math.atan(wheelbase * curvature)


def circle_radius(trial: dict):
    """Rear-axle-centre radius and (optionally) rear track for a circle trial.

    ``rear_tires`` uses the circles traced by the inner and outer rear tires.
    The axle centre sits midway between them, so its radius is the mean of
    the two radii exactly, with no track-width constant needed. The
    difference is the rear track, which becomes a free consistency check.
    """
    method = trial.get('measurement_method', 'axle_centre')
    if method == 'rear_tires':
        inner = trial.get('measured_inner_diameter_m')
        outer = trial.get('measured_outer_diameter_m')
        if not _finite(inner) or not _finite(outer) or inner <= 0.0:
            raise ValueError('Inner and outer tire diameters must be positive.')
        if outer <= inner:
            raise ValueError(
                'The outer tire circle must be larger than the inner one.')
        return (float(inner) + float(outer)) / 4.0, \
            (float(outer) - float(inner)) / 2.0
    if method != 'axle_centre':
        raise ValueError(f'Unknown circle measurement method: {method!r}.')
    diameter = trial.get('measured_diameter_m')
    if not _finite(diameter) or float(diameter) <= 0.0:
        raise ValueError('Measured diameter must be positive.')
    return float(diameter) / 2.0, None


def _positive_number(payload, name, label):
    value = payload.get(name)
    if not _finite(value) or float(value) <= 0.0:
        raise ValueError(f'{label} must be a positive number.')
    return float(value)


def validate_steering_measurement(kind: str, payload: dict):
    """Validate operator-entered measurements; return the fields to store."""
    if kind == 'steering_center':
        return {}
    if kind == 'steering_drift':
        forward = _positive_number(
            payload, 'measured_forward_m', 'Forward distance along the line')
        lateral = payload.get('measured_lateral_m')
        if not _finite(lateral):
            raise ValueError('Sideways offset must be a number (0 if none).')
        lateral = float(lateral)
        if abs(lateral) >= forward:
            raise ValueError(
                'Sideways offset must be smaller than the forward distance.')
        return {'measured_forward_m': forward, 'measured_lateral_m': lateral}
    if kind not in ('steering_left', 'steering_right'):
        raise ValueError(f'Unknown steering capture kind: {kind!r}.')
    full_lock = payload.get('full_lock', False)
    if not isinstance(full_lock, bool):
        raise ValueError('full_lock must be true or false.')
    method = payload.get('measurement_method', 'axle_centre')
    fields = {'measurement_method': method, 'full_lock': full_lock}
    if method == 'axle_centre':
        fields['measured_diameter_m'] = _positive_number(
            payload, 'measured_diameter_m', 'Rear-axle circle diameter')
    elif method == 'rear_tires':
        fields['measured_inner_diameter_m'] = _positive_number(
            payload, 'measured_inner_diameter_m', 'Inner tire circle diameter')
        fields['measured_outer_diameter_m'] = _positive_number(
            payload, 'measured_outer_diameter_m', 'Outer tire circle diameter')
    else:
        raise ValueError(f'Unknown circle measurement method: {method!r}.')
    circle_radius(fields)  # cross-field checks (outer > inner)
    return fields


def _servo_limits(session: dict):
    vehicle = session.get('vehicle', {}) or {}
    low, high = vehicle.get('servo_min'), vehicle.get('servo_max')
    if _finite(low) and _finite(high) and float(low) < float(high):
        return float(low), float(high)
    return None


def _side_gain(points):
    fit = _linear_fit(points)
    return fit[0] if fit else None


def steering_calibration(session: dict):
    params = session.get('current_parameters', {})
    wheelbase = float(params.get('wheelbase', 0.36))
    current_gain = float(params.get('steering_angle_to_servo_gain', -1.2135))
    current_offset = float(params.get('steering_angle_to_servo_offset', 0.5304))
    limits = _servo_limits(session)
    trials = [
        trial for trial in session.get('trials', [])
        if trial.get('accepted') and trial.get('kind') in STEERING_KINDS
    ]
    warnings = []
    points = []
    trial_results = []
    directions = set()
    track_widths = []

    for index, trial in enumerate(trials, start=1):
        kind = trial.get('kind')
        summary = trial.get('summary', {})
        servo = summary.get('servo', {}).get('median')
        item = {
            'trial_id': trial.get('id'),
            'trial_number': index,
            'kind': kind,
            'median_servo': servo,
            'usable': False,
            'warnings': [],
        }
        if not _finite(servo):
            item['warnings'].append('No finite servo command was captured.')
            trial_results.append(item)
            continue
        servo_p05 = summary.get('servo', {}).get('p05')
        servo_p95 = summary.get('servo', {}).get('p95')
        if _finite(servo_p05) and _finite(servo_p95):
            servo_span = float(servo_p95) - float(servo_p05)
            item['servo_span'] = servo_span
            if servo_span > 0.05:
                item['warnings'].append(
                    'Servo command varied substantially; hold steering steadier.'
                )
        if limits is not None:
            item['at_servo_limit'] = (
                abs(float(servo) - limits[0]) <= SERVO_LIMIT_TOLERANCE
                or abs(float(servo) - limits[1]) <= SERVO_LIMIT_TOLERANCE
            )

        if kind == 'steering_center':
            actual_angle = 0.0
            if abs(float(servo) - current_offset) <= CENTRE_ECHO_TOLERANCE:
                item['warnings'].append(
                    'Servo value matches the configured offset, so this capture '
                    'only reads the current setting back. Use a straight-line '
                    'drift test to measure the true centre.'
                )
        elif kind == 'steering_drift':
            forward = trial.get('measured_forward_m')
            lateral = trial.get('measured_lateral_m')
            item['measured_forward_m'] = forward
            item['measured_lateral_m'] = lateral
            try:
                actual_angle = drift_steering_angle(wheelbase, forward, lateral)
            except ValueError as exc:
                item['warnings'].append(f'Drift measurement unusable: {exc}.')
                trial_results.append(item)
                continue
        else:
            try:
                radius, track = circle_radius(trial)
            except ValueError as exc:
                item['warnings'].append(str(exc))
                trial_results.append(item)
                continue
            item['measurement_method'] = trial.get(
                'measurement_method', 'axle_centre')
            item['full_lock'] = bool(trial.get('full_lock', False))
            item['rear_axle_radius_m'] = radius
            if track is not None:
                item['rear_track_m'] = track
                track_widths.append(track)
            magnitude = math.atan(wheelbase / radius)
            actual_angle = magnitude if kind == 'steering_left' else -magnitude
            directions.add(kind)
            yaw = summary.get('odom_yaw_rad')
            item['odom_yaw_rad'] = yaw
            if _finite(yaw) and abs(float(yaw)) > 0.2:
                expected_sign = 1.0 if kind == 'steering_left' else -1.0
                if math.copysign(1.0, float(yaw)) != expected_sign:
                    item['warnings'].append(
                        'Odometry yaw sign disagrees with the confirmed turn direction.'
                    )
                item['odom_turns'] = float(yaw) / (2.0 * math.pi)

        item['actual_steering_angle_rad'] = actual_angle
        item['usable'] = True
        points.append((actual_angle, float(servo)))
        trial_results.append(item)

    fit = _linear_fit(points)
    has_centre_region = any(
        item['usable'] and item['kind'] in ('steering_center', 'steering_drift')
        for item in trial_results
    )
    max_commandable = None
    if fit is None:
        suggested_gain = None
        suggested_offset = None
        rmse = None
        status = 'insufficient'
        warnings.append(
            'At least two distinct usable steering measurements are required.'
        )
    else:
        suggested_gain, suggested_offset, rmse = fit
        for item in trial_results:
            if item['usable']:
                item['fit_residual_servo'] = item['median_servo'] - (
                    suggested_gain * item['actual_steering_angle_rad']
                    + suggested_offset)
        if abs(suggested_gain) < 0.1:
            status = 'invalid'
            warnings.append(
                'Fitted steering gain is implausibly close to zero.'
            )
        elif directions == {'steering_left', 'steering_right'} and len(points) >= 3:
            status = 'good' if rmse <= 0.02 else 'low'
        else:
            status = 'low'
        if directions != {'steering_left', 'steering_right'}:
            warnings.append('Record both a left and a right circle.')
        if not has_centre_region:
            warnings.append(
                'No straight-line drift test was recorded; the steering '
                'offset rests on the circles alone and is less certain.'
            )
        if suggested_gain * current_gain < 0.0:
            warnings.append(
                'Suggested steering gain reverses the configured sign; '
                'verify left/right labels before applying it.'
            )
        if rmse > 0.02:
            warnings.append(
                f"Steering fit residual is {rmse:.3f} servo units; repeat unstable trials."
            )
        if limits is not None and status != 'invalid':
            ends = [(limit - suggested_offset) / suggested_gain for limit in limits]
            left, right = max(ends), min(ends)
            if left > 0.0 > right:
                max_commandable = {
                    'left_rad': left,
                    'right_rad': right,
                    'left_min_radius_m': wheelbase / math.tan(left),
                    'right_min_radius_m': wheelbase / math.tan(-right),
                }

    centre_points = [
        (item['actual_steering_angle_rad'], item['median_servo'])
        for item in trial_results
        if item['usable'] and item['kind'] in ('steering_center', 'steering_drift')
    ]
    sides = {}
    for side, kind in (('left', 'steering_left'), ('right', 'steering_right')):
        circles = [
            item for item in trial_results
            if item['usable'] and item['kind'] == kind
        ]
        full = [item for item in circles if item.get('full_lock')]
        full_radius = median(item['rear_axle_radius_m'] for item in full)
        sides[side] = {
            'circle_count': len(circles),
            'full_lock_count': len(full),
            'full_lock_radius_m': full_radius,
            'full_lock_angle_rad': (
                math.atan(wheelbase / full_radius) * (1.0 if side == 'left' else -1.0)
                if full_radius else None
            ),
            'full_lock_at_servo_limit': (
                all(item.get('at_servo_limit') for item in full)
                if full and limits is not None else None
            ),
            'gain': _side_gain(centre_points + [
                (item['actual_steering_angle_rad'], item['median_servo'])
                for item in circles
            ]) if centre_points and circles else None,
        }
    left_gain, right_gain = sides['left']['gain'], sides['right']['gain']
    asymmetry = None
    if _finite(left_gain) and _finite(right_gain):
        mean_magnitude = (abs(left_gain) + abs(right_gain)) / 2.0
        if mean_magnitude > EPSILON:
            asymmetry = abs(left_gain - right_gain) / mean_magnitude
            if asymmetry > ASYMMETRY_WARNING_FRACTION:
                warnings.append(
                    f'Left and right steering gains differ by '
                    f'{asymmetry * 100.0:.0f}%; one straight line does not '
                    'describe this linkage well. The suggestion is a compromise.'
                )

    track = None
    if track_widths:
        track = median(track_widths)
        if max(track_widths) - min(track_widths) > TRACK_SPREAD_WARNING_M:
            warnings.append(
                'Rear track widths from different circles disagree by more '
                f'than {TRACK_SPREAD_WARNING_M * 100:.0f} cm; re-measure the '
                'tire circles.'
            )

    for item in trial_results:
        if item.get('kind') == 'steering_center' and item['warnings']:
            warnings.extend(
                w for w in item['warnings'] if 'configured offset' in w)
            break

    return {
        'status': status,
        'wheelbase_m': wheelbase,
        'usable_point_count': len(points),
        'suggested_steering_angle_to_servo_gain': suggested_gain,
        'current_steering_angle_to_servo_gain': current_gain,
        'suggested_steering_angle_to_servo_offset': suggested_offset,
        'current_steering_angle_to_servo_offset': current_offset,
        'fit_rmse_servo': rmse,
        'servo_limits': list(limits) if limits else None,
        'max_commandable_steering': max_commandable,
        'sides': sides,
        'side_gain_asymmetry': asymmetry,
        'rear_track_m': track,
        'trial_results': trial_results,
        'warnings': warnings,
    }


MODES_WITH_MOVEMENT = ('movement', 'movement_steering')
MODES_WITH_STEERING = ('movement_steering', 'steering')


def build_report(session: dict):
    mode = session.get('mode')
    movement = (
        movement_calibration(session)
        if mode in MODES_WITH_MOVEMENT or mode is None
        else None
    )
    steering = (
        steering_calibration(session)
        if mode in MODES_WITH_STEERING
        else None
    )
    suggestions = {}
    if movement:
        suggestions.update({
            'speed_to_erpm_gain': movement['suggested_speed_to_erpm_gain'],
            'speed_to_erpm_offset': movement['suggested_speed_to_erpm_offset'],
        })
    if steering:
        suggestions.update({
            'steering_angle_to_servo_gain':
                steering['suggested_steering_angle_to_servo_gain'],
            'steering_angle_to_servo_offset':
                steering['suggested_steering_angle_to_servo_offset'],
        })
    statuses = [part['status'] for part in (movement, steering) if part]
    overall = (
        'ready'
        if statuses and all(status in ('good', 'high') for status in statuses)
        else 'review'
    )
    notes = []
    if movement:
        notes.append(
            'Speed gain and offset suggestions apply to vesc_to_odom_node only; '
            'do not copy a negative odometry gain into the shared motor-command '
            'configuration.')
    if steering:
        notes.append(
            'Steering gain and offset apply to the shared steering conversion '
            'in vesc.yaml.')
    notes.append(
        'Review suggestions before editing YAML. Re-test wheels off the '
        'ground, then at low speed while holding LB.')
    return {
        'schema_version': 1,
        'session_id': session.get('session_id'),
        'created_at': session.get('created_at'),
        'generated_at': session.get('updated_at'),
        'mode': mode,
        'overall_status': overall,
        'vehicle': session.get('vehicle', {}),
        'current_parameters': session.get('current_parameters', {}),
        'accepted_trials': [
            trial for trial in session.get('trials', [])
            if trial.get('accepted')
        ],
        'movement': movement,
        'steering': steering,
        'parameter_suggestions': suggestions,
        'safety_note': ' '.join(notes),
    }
```

- [ ] **Step 4: Rewrite `report_markdown`** so a missing movement section is skipped and steering lists per-side results. Replace everything in `report_markdown` from the line `    lines.extend(['```', '', '## Movement calibration', ''])` up to (not including) the `'## Safety'` block with:

```python
    lines.append('```')
    movement = report.get('movement')
    if movement:
        lines.extend(['', '## Movement calibration', ''])
        lines.extend([
            f"- Confidence: **{movement.get('status')}**",
            f"- Usable trials: {movement.get('usable_trial_count', 0)}",
            f"- Gain: {_format_number(movement.get('current_speed_to_erpm_gain'))}"
            f" → {_format_number(movement.get('suggested_speed_to_erpm_gain'))}",
            f"- Offset: {_format_number(movement.get('current_speed_to_erpm_offset'))}"
            f" → {_format_number(movement.get('suggested_speed_to_erpm_offset'))}",
        ])
        for warning in movement.get('warnings', []):
            lines.append(f"- Warning: {warning}")
    steering = report.get('steering')
    if steering:
        lines.extend(['', '## Steering calibration', ''])
        lines.extend([
            f"- Confidence: **{steering.get('status')}**",
            f"- Usable points: {steering.get('usable_point_count', 0)}",
            f"- Gain: "
            f"{_format_number(steering.get('current_steering_angle_to_servo_gain'))}"
            f" → "
            f"{_format_number(steering.get('suggested_steering_angle_to_servo_gain'))}",
            f"- Offset: "
            f"{_format_number(steering.get('current_steering_angle_to_servo_offset'))}"
            f" → "
            f"{_format_number(steering.get('suggested_steering_angle_to_servo_offset'))}",
        ])
        for side, data in (steering.get('sides') or {}).items():
            if data.get('full_lock_radius_m'):
                lines.append(
                    f"- Full lock {side}: rear-axle radius "
                    f"{_format_number(data['full_lock_radius_m'], 3)} m, "
                    f"{_format_number(math.degrees(abs(data['full_lock_angle_rad'])), 1)} degrees"
                )
        reach = steering.get('max_commandable_steering')
        if reach:
            lines.append(
                f"- Servo limits allow at most "
                f"{_format_number(reach['left_rad'], 3)} rad left and "
                f"{_format_number(-reach['right_rad'], 3)} rad right "
                "with the suggested values."
            )
        for warning in steering.get('warnings', []):
            lines.append(f"- Warning: {warning}")
```

- [ ] **Step 5: Run the whole package suite**

Run: `source /opt/ros/jazzy/setup.bash && python3 -m pytest src/odom_calibration/test -q`
Expected: all pass, including every pre-existing test unchanged.

- [ ] **Step 6: Prove the tests can fail (Hard rule 1).** Apply each mutation below to `calibration_math.py` one at a time, run `python3 -m pytest src/odom_calibration/test/test_steering_math.py -q`, confirm at least one test fails, then revert. Record each result.
  1. In `drift_steering_angle`, change `2.0 * lateral_m` to `lateral_m`.
  2. In `drift_steering_angle`, `return 0.0`.
  3. In `circle_radius`, change `/ 4.0` to `/ 2.0`.
  4. In `steering_calibration`, flip the circle sign: `actual_angle = -magnitude if kind == 'steering_left' else magnitude`.
  5. In `steering_calibration`, change `ASYMMETRY_WARNING_FRACTION` to `1.0`.
  6. In `steering_calibration`, swap `left, right = max(ends), min(ends)` to `left, right = min(ends), max(ends)`.
  7. In `validate_steering_measurement`, delete the `abs(lateral) >= forward` check.
  8. In `build_report`, compute movement unconditionally.
  Then confirm `git diff` shows none of the mutations remain.

- [ ] **Step 7: Commit**

```bash
git add src/odom_calibration/odom_calibration/calibration_math.py src/odom_calibration/test/test_steering_math.py
git commit -m "odom_calibration: steering drift test, tire-circle method, per-side analysis"
```

---

### Task 2: Session modes, capture gating, live progress, and node wiring

**Files:**
- Modify: `src/odom_calibration/odom_calibration/session_store.py`
- Modify: `src/odom_calibration/odom_calibration/calibration_math.py` (add `RunningIntegral` after `integrate_samples`)
- Modify: `src/odom_calibration/odom_calibration/calibration_node.py`
- Modify: `src/odom_calibration/config/odom_calibration.yaml`
- Create: `src/odom_calibration/test/test_modes_and_progress.py`

**Interfaces:**
- Consumes (Task 1): `calibration_math.validate_steering_measurement(kind, payload) -> dict`, `calibration_math.STEERING_KINDS`.
- Produces (Task 3 reads these over HTTP/WebSocket):
  - `session_store.VALID_MODES = ('movement', 'movement_steering', 'steering')`
  - `session_store.MODE_STAGES = {'movement': ('preflight', 'stationary', 'movement', 'report'), 'movement_steering': ('preflight', 'stationary', 'movement', 'steering', 'report'), 'steering': ('preflight', 'steering', 'report')}`
  - `session_store.CAPTURE_KINDS = ('stationary', 'movement', 'steering_center', 'steering_drift', 'steering_left', 'steering_right')`
  - `session_store.stage_allowed(mode: str, stage: str) -> bool`
  - `session_store.capture_allowed(mode: str, kind: str) -> bool` — `stationary` needs stage `stationary` in the mode, `movement` needs `movement`, any `steering_*` kind needs `steering`; unknown mode or kind → False.
  - `calibration_math.RunningIntegral(max_gap_sec=0.5)` with `.add(t, value) -> None` and `.total` (float); same rules as `integrate_samples` for in-order data: skips non-finite samples, skips non-increasing timestamps, does not integrate across a gap larger than `max_gap_sec`.
  - Snapshot key `capture_progress`: `None` when no capture runs, else `{'kind': str, 'odom_distance_m': float, 'odom_yaw_rad': float}`.
  - Snapshot key `session.vehicle.servo_min` / `servo_max` (floats) on new sessions.
  - Accept payload fields for steering kinds exactly as `validate_steering_measurement` reads them: drift `measured_forward_m`, `measured_lateral_m` (signed, left positive); circles `measurement_method` (`axle_centre`|`rear_tires`), `full_lock` (bool), `measured_diameter_m` or `measured_inner_diameter_m` + `measured_outer_diameter_m`.

- [ ] **Step 1: Write the failing tests** in `src/odom_calibration/test/test_modes_and_progress.py`:

```python
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from odom_calibration import calibration_math  # noqa: E402
from odom_calibration import session_store  # noqa: E402

PARAMETERS = {
    'speed_to_erpm_gain': -4614.0,
    'speed_to_erpm_offset': 0.0,
    'steering_angle_to_servo_gain': -1.2135,
    'steering_angle_to_servo_offset': 0.5304,
    'wheelbase': 0.36,
}


def test_steering_only_session_can_be_created():
    session = session_store.new_session('steering', PARAMETERS)
    assert session['mode'] == 'steering'
    assert session['stage'] == 'preflight'


def test_unknown_mode_is_rejected():
    with pytest.raises(ValueError, match='mode must be one of'):
        session_store.new_session('drift', PARAMETERS)


@pytest.mark.parametrize('mode,stage,allowed', [
    ('steering', 'preflight', True),
    ('steering', 'steering', True),
    ('steering', 'report', True),
    ('steering', 'stationary', False),
    ('steering', 'movement', False),
    ('movement', 'steering', False),
    ('movement', 'movement', True),
    ('movement_steering', 'steering', True),
    ('movement_steering', 'stationary', True),
    ('unknown', 'preflight', False),
    ('steering', 'nonsense', False),
])
def test_stage_allowed_follows_mode_table(mode, stage, allowed):
    # Spec: MODE_STAGES in the plan's Task 2 interfaces.
    assert session_store.stage_allowed(mode, stage) is allowed


@pytest.mark.parametrize('mode,kind,allowed', [
    ('steering', 'steering_drift', True),
    ('steering', 'steering_left', True),
    ('steering', 'steering_center', True),
    ('steering', 'stationary', False),
    ('steering', 'movement', False),
    ('movement', 'steering_drift', False),
    ('movement', 'movement', True),
    ('movement', 'stationary', True),
    ('movement_steering', 'steering_right', True),
    ('movement_steering', 'movement', True),
    ('movement_steering', 'teleport', False),
    ('unknown', 'movement', False),
])
def test_capture_allowed_follows_mode_table(mode, kind, allowed):
    assert session_store.capture_allowed(mode, kind) is allowed


def test_every_mode_starts_at_preflight_and_ends_at_report():
    # Invariant the UI's step list relies on.
    for mode in session_store.VALID_MODES:
        stages = session_store.MODE_STAGES[mode]
        assert stages[0] == 'preflight'
        assert stages[-1] == 'report'


def _samples():
    # 10 Hz, value 2.0, with a NaN, a repeated timestamp, and a 1 s gap.
    samples = [{'t': 0.1 * i, 'v': 2.0} for i in range(11)]
    samples.insert(3, {'t': 0.25, 'v': float('nan')})
    samples.insert(6, {'t': 0.4, 'v': 2.0})  # duplicate timestamp
    samples += [{'t': 2.0 + 0.1 * i, 'v': 2.0} for i in range(6)]
    return samples


def test_running_integral_matches_batch_integrator():
    running = calibration_math.RunningIntegral(max_gap_sec=0.5)
    for sample in _samples():
        running.add(sample['t'], sample['v'])
    batch = calibration_math.integrate_samples(_samples(), 'v', max_gap_sec=0.5)
    # Oracle: the existing batch integrator (independent code path).
    assert running.total == pytest.approx(batch['integral'], abs=1e-12)


def test_running_integral_closed_form():
    running = calibration_math.RunningIntegral(max_gap_sec=0.5)
    for i in range(11):
        running.add(0.1 * i, 2.0)
    # Closed form: constant 2.0 for 1.0 s integrates to 2.0.
    assert running.total == pytest.approx(2.0, abs=1e-12)


def test_running_integral_skips_large_gap():
    running = calibration_math.RunningIntegral(max_gap_sec=0.5)
    running.add(0.0, 1.0)
    running.add(0.1, 1.0)
    running.add(5.0, 1.0)  # 4.9 s gap: not integrated
    running.add(5.1, 1.0)
    # Closed form: two 0.1 s intervals of value 1.0.
    assert running.total == pytest.approx(0.2, abs=1e-12)


def test_running_integral_ignores_non_finite_and_backwards_time():
    running = calibration_math.RunningIntegral(max_gap_sec=0.5)
    running.add(0.0, 1.0)
    running.add(0.1, float('inf'))
    running.add(0.05, 1.0)  # earlier than the last accepted sample
    running.add(0.2, 1.0)
    # Only 0.0 -> 0.2 at value 1.0 counts.
    assert running.total == pytest.approx(0.2, abs=1e-12)


def test_running_integral_starts_at_zero():
    assert calibration_math.RunningIntegral().total == 0.0


def test_running_integral_rejects_bad_gap():
    with pytest.raises(ValueError, match='max_gap_sec'):
        calibration_math.RunningIntegral(max_gap_sec=0.0)
```

- [ ] **Step 2: Run to verify they fail**

Run: `source /opt/ros/jazzy/setup.bash && python3 -m pytest src/odom_calibration/test/test_modes_and_progress.py -q`
Expected: FAIL (no `stage_allowed`, no `RunningIntegral`, `'steering'` mode rejected).

- [ ] **Step 3: `session_store.py`.** Replace `VALID_MODES` and add below `VALID_STAGES`:

```python
VALID_MODES = ('movement', 'movement_steering', 'steering')
```

```python
MODE_STAGES = {
    'movement': ('preflight', 'stationary', 'movement', 'report'),
    'movement_steering': (
        'preflight', 'stationary', 'movement', 'steering', 'report'),
    'steering': ('preflight', 'steering', 'report'),
}
CAPTURE_KINDS = (
    'stationary',
    'movement',
    'steering_center',
    'steering_drift',
    'steering_left',
    'steering_right',
)


def stage_allowed(mode, stage):
    return stage in MODE_STAGES.get(mode, ())


def capture_allowed(mode, kind):
    if kind not in CAPTURE_KINDS:
        return False
    stage = 'steering' if kind.startswith('steering_') else kind
    return stage_allowed(mode, stage)
```

- [ ] **Step 4: `calibration_math.py`.** Add directly after `integrate_samples`:

```python
class RunningIntegral:
    """Incremental twin of ``integrate_samples`` for live progress readouts.

    Accepts samples in arrival order and applies the same rules: non-finite
    values and non-increasing timestamps are ignored, and a gap longer than
    ``max_gap_sec`` is not integrated.
    """

    def __init__(self, max_gap_sec: float = 0.5):
        if not _finite(max_gap_sec) or max_gap_sec <= 0.0:
            raise ValueError('max_gap_sec must be finite and positive')
        self.max_gap_sec = float(max_gap_sec)
        self.total = 0.0
        self._last = None

    def add(self, t, value):
        if not _finite(t) or not _finite(value):
            return
        t, value = float(t), float(value)
        if self._last is not None:
            last_t, last_value = self._last
            dt = t - last_t
            if dt <= 0.0:
                return
            if dt <= self.max_gap_sec:
                self.total += 0.5 * (last_value + value) * dt
        self._last = (t, value)
```

- [ ] **Step 5: Run the new tests to verify they pass**

Run: `source /opt/ros/jazzy/setup.bash && python3 -m pytest src/odom_calibration/test -q`
Expected: all pass.

- [ ] **Step 6: Wire the node (`calibration_node.py`).** Make exactly these changes:
  1. Remove the module-level `CAPTURE_KINDS` tuple and import `CAPTURE_KINDS`, `capture_allowed`, `stage_allowed` from `odom_calibration.session_store` alongside the existing imports.
  2. Declare two parameters after `wheelbase`: `self.declare_parameter('servo_min', 0.15)` and `self.declare_parameter('servo_max', 0.85)`; read them into `self.servo_min` / `self.servo_max` as floats; if not finite or `servo_min >= servo_max`, raise `ValueError('servo_min must be below servo_max')`.
  3. In `CaptureRecorder.__init__` add `self.progress = {'odom_distance_m': calibration_math.RunningIntegral(), 'odom_yaw_rad': calibration_math.RunningIntegral()}`. In `odom_callback`, inside the existing `if self.active_recorder:` block, also call `self.active_recorder.progress['odom_distance_m'].add(now, speed)` and `self.active_recorder.progress['odom_yaw_rad'].add(now, angular_z)`.
  4. In `snapshot()`, add `'capture_progress'`: `None` when `self.active_recorder` is falsy, else `{'kind': self.active_recorder.kind, 'odom_distance_m': <progress distance total>, 'odom_yaw_rad': <progress yaw total>}`.
  5. In `new_session`: add `'servo_min': self.servo_min, 'servo_max': self.servo_max` to the `vehicle` dict, and wrap `new_session(mode, parameters, vehicle)` so a `ValueError` becomes `WizardError(str(exc))`.
  6. In `set_stage`: replace the `stage == 'steering' and mode != 'movement_steering'` check with `if not stage_allowed(self.session.get('mode'), stage): raise WizardError(f'Stage {stage!r} is not part of this session.')` (keep the `VALID_STAGES` check before it).
  7. In `_start_capture`: keep the `kind not in CAPTURE_KINDS` check; replace the `kind.startswith('steering_') and mode != 'movement_steering'` check with `if not capture_allowed(self.session.get('mode'), kind): raise WizardError('That test is not part of this session.')`.
  8. In `_accept_capture`: replace the `elif kind in ('steering_left', 'steering_right'):` block with `elif kind in calibration_math.STEERING_KINDS:` that calls `fields = calibration_math.validate_steering_measurement(kind, payload)` inside `try/except ValueError as exc: raise WizardError(str(exc)) from exc`, then `trial.update(fields)`.
  9. Do not add any publisher. The node must stay read-only.

- [ ] **Step 7: Config.** In `src/odom_calibration/config/odom_calibration.yaml`, after `wheelbase: 0.36`, add:

```yaml
    # Servo clamp applied by vesc_driver. Must match vesc.yaml servo_min /
    # servo_max; the report uses them to show the steering the car can reach.
    servo_min: 0.15
    servo_max: 0.85
```

- [ ] **Step 8: Verify the node still imports and builds.**

Run (repo root):
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select odom_calibration
source install/setup.bash
python3 -c "import odom_calibration.calibration_node as n; print('ok', n.CAPTURE_KINDS)"
grep -n "create_publisher" src/odom_calibration/odom_calibration/calibration_node.py || echo "no publishers"
python3 -m pytest src/odom_calibration/test -q
```
Expected: `ok (...)` listing six kinds, `no publishers`, all tests pass.

- [ ] **Step 9: Prove the new tests can fail.** One at a time, then revert: (a) `stage_allowed` → `return True`; (b) `capture_allowed` → map `steering_*` to `'movement'`; (c) `RunningIntegral.add` → integrate across gaps (drop the `dt <= self.max_gap_sec` check); (d) `RunningIntegral.add` → drop the `dt <= 0.0` return. Each must fail at least one test in `test_modes_and_progress.py`. Record results.

- [ ] **Step 10: Commit**

```bash
git add src/odom_calibration/odom_calibration/session_store.py src/odom_calibration/odom_calibration/calibration_math.py src/odom_calibration/odom_calibration/calibration_node.py src/odom_calibration/config/odom_calibration.yaml src/odom_calibration/test/test_modes_and_progress.py
git commit -m "odom_calibration: steering-only mode, capture gating, live capture progress"
```

---

### Task 3: Web UI redesign + steering screens

**Files:**
- Rewrite: `src/odom_calibration/web/index.html`, `src/odom_calibration/web/style.css`, `src/odom_calibration/web/wizard.js`
- Already present (copied in, untracked — commit them): `src/odom_calibration/web/fonts/saira-condensed-700.woff2`, `saira-condensed-800.woff2`, `public-sans.woff2` (variable, 100–900), `ibm-plex-mono-500.woff2`, and `src/odom_calibration/web/icon.png` (team logo, 128 px).
- Modify: `src/odom_calibration/setup.py` — install `web/icon.png` into `share/odom_calibration/web` and the four fonts into `share/odom_calibration/web/fonts`.

**Interfaces:**
- Consumes (server, unchanged API): `GET /api/state` and WebSocket `/ws` snapshots `{session, telemetry, health, live_parameters, live_parameter_status, capture_duration_sec, capture_progress, read_only, report_directory}`; `POST /api/action` with `{action, ...}` for `new_session {mode, current_parameters, replace_existing?}`, `set_stage {stage}`, `start_capture {kind}`, `stop_capture`, `accept_capture {confirmed, notes, ...fields}`, `discard_capture`, `delete_trial {trial_id}`, `generate_report`; downloads `/api/report/md`, `/api/report/json`. Keep the current `wizard.js` networking code (`api`, `fetchState`, `connectSocket`, `updateSnapshot` and its `renderKey` re-render guard) — rewrite the rendering.
- Stage lists per mode come from Task 2: movement → preflight, stationary, movement, report; movement_steering → preflight, stationary, movement, steering, report; steering → preflight, steering, report. Mirror that table in JS as `MODE_STAGES`.
- Accept fields per kind: movement `direction`, `measured_distance_m`; drift `measured_forward_m` (positive), `measured_lateral_m` (signed: the UI asks for a positive magnitude plus a Left/Right choice and sends `+m` for left, `-m` for right, `0` if the operator ticks "ended on the line"); circles `measurement_method`, `full_lock` (checkbox), and `measured_diameter_m` or `measured_inner_diameter_m` + `measured_outer_diameter_m`.
- Report JSON fields to show (Task 1): `overall_status`, `parameter_suggestions`, `current_parameters`, `movement` (may be null), `steering` with `status`, `fit_rmse_servo`, `usable_point_count`, `trial_results[] {kind, median_servo, actual_steering_angle_rad, fit_residual_servo, full_lock, at_servo_limit, rear_axle_radius_m}`, `sides.left|right {full_lock_radius_m, full_lock_angle_rad, full_lock_at_servo_limit}`, `max_commandable_steering`, `rear_track_m`, `warnings`, and `safety_note`.

**Design system (copy these tokens exactly — from sfuracerbot.ca `static/css/main.css`):**

```css
@font-face { font-family: 'Saira Condensed'; font-weight: 700; font-display: swap; src: url('/fonts/saira-condensed-700.woff2') format('woff2'); }
@font-face { font-family: 'Saira Condensed'; font-weight: 800; font-display: swap; src: url('/fonts/saira-condensed-800.woff2') format('woff2'); }
@font-face { font-family: 'Public Sans'; font-weight: 100 900; font-display: swap; src: url('/fonts/public-sans.woff2') format('woff2'); }
@font-face { font-family: 'IBM Plex Mono'; font-weight: 500; font-display: swap; src: url('/fonts/ibm-plex-mono-500.woff2') format('woff2'); }

:root {
  color-scheme: light dark;
  --paper: #ffffff; --wash: #f3f5f9; --wash-2: #e9edf4;
  --navy: #0c1e3a; --navy-2: #13294b; --navy-line: rgba(154,183,220,.18);
  --ink: #16233d; --muted: #4c5a76; --faint: #7a89a6; --heading: var(--navy);
  --snow: #f2f6fb; --mist: #aebfd9;
  --blue: #1d5f9e; --blue-deep: #164a7c; --blue-hi: #7fb2e5; --link-ink: var(--blue);
  --line: #dbe1ec; --line-2: #c8d1e0; --checker: #ffffff;
  --good: #1e7a46; --warn: #b54708; --bad: #b42318;
  --font-display: 'Saira Condensed', 'Arial Narrow', sans-serif;
  --font-body: 'Public Sans', system-ui, sans-serif;
  --font-mono: 'IBM Plex Mono', ui-monospace, monospace;
}
@media (prefers-color-scheme: dark) { :root {
  --paper: #0b1626; --wash: #101f36; --wash-2: #17273f;
  --navy: #16294c; --navy-2: #1e355c; --navy-line: rgba(154,183,220,.16);
  --ink: #dde5f2; --muted: #9fb0cc; --faint: #71829f; --heading: #f3f7fc;
  --blue: #3f7fc4; --blue-deep: #2f6690; --link-ink: var(--blue-hi);
  --line: rgba(154,183,220,.18); --line-2: rgba(154,183,220,.28);
  --good: #5cc28a; --warn: #f0a35e; --bad: #f28b82;
} }
```

Components to reproduce from the site (same geometry): headings `h1,h2,h3` in `--font-display`, weight 700, `text-transform: uppercase`, `line-height: .98`; body `--font-body`, 1rem/1.6. Primary button: `--font-display` 700, uppercase, `letter-spacing .05em`, padding `.8rem 1.9rem`, background `--blue` (dark mode `--blue-deep`), white text, `clip-path: polygon(0 0, 100% 0, 100% 100%, 11px 100%)`. Ghost button: same shape, `1px solid` border at `color-mix(in srgb, currentColor 45%, transparent)`. Danger button: same shape, background `--bad`, white text. Checker strip: `height:10px; background: repeating-conic-gradient(var(--checker) 0 25%, transparent 0 50%) 0 0/20px 20px`. Race plate (trial numbers, step numbers): `--font-mono` 600-ish (use 500) .75rem, `border:1px solid var(--blue)`, colour `--link-ink`, `clip-path: polygon(0 0,100% 0,100% 100%,8px 100%)`, padding `.15rem .6rem`. Sector rule under each page title: a 3-column grid `3fr 2fr 1fr`, `gap:6px`, `height:3px`, spans coloured `--navy`, `--blue`, `--blue-hi`. Focus: `:focus-visible { outline: 2px solid var(--blue); outline-offset: 3px; }`. Minimum touch target 44 px.

**Layout:**

```
desktop (≥ 1100 px)
┌───────────────────────────────────────────────────────────────────────────┐
│ navy bar: [logo] SFU RACERBOT  Calibration   · status ·  [New session]     │
│ ▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚ checker strip ▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚ │
├──────────────┬───────────────────────────────────────┬────────────────────┤
│ timing tower │ Step 3 of 4 (the only eyebrow)         │ LIVE CAR            │
│ 1 Preflight ✓│ STEERING                               │ LB  [RELEASED]      │
│ 2 Stationary │ ███████ ████ ██  (sector rule)          │ speed   +0.00 m/s   │
│ 3 Steering ◀ │ one-sentence plain lead                │ servo    0.5304     │
│ 4 Report     │ [steering diagram + test picker]        │ steer   +0.0°  est. │
│              │ numbered instructions                   │ raw ERPM   +0       │
│ note: saved  │ [Record] button                         │ ─ topics ─          │
│ on the car   │ accepted trials table                   │ ■ Odometry 50 Hz    │
└──────────────┴───────────────────────────────────────┴────────────────────┘
mobile (< 760 px): navy bar; a compact sticky live strip (LB state, speed, servo)
under it; step list becomes a horizontal scroller of race plates; main column;
full live panel and topic health at the bottom. No horizontal page scroll at
375 px.
```

Main column max-width 72ch-ish (`max-width: 46rem`), left-aligned. Rail 15rem, live panel 18rem. Sections are separated by `1px solid var(--line)` rules and spacing, **not** by a grid of identical rounded cards with shadows. No border-radius anywhere except the round status dots. No gradients except the checker strip. No drop shadows.

**The one bold element — steering diagram (`renderSteeringDiagram()`, inline SVG, ~320×220):** a top-down car in navy line work: body outline, rear axle line, rear wheels, front axle, and two front wheels rotated by the live implied steering angle `(telemetry.servo - offset) / gain` using the session's `current_parameters` (clamped to ±0.5 rad for drawing), plus a dashed blue arc showing the rear-axle path that angle produces (radius `wheelbase / tan(angle)`, straight line when |angle| < 0.002). Under it, the readout "Implied steering +12.3° left (from current settings)". On the steering screen during a circle capture show `capture_progress.odom_yaw_rad / (2π)` as "0.82 turns (odometry estimate)" and distance.

**Fit chart (`renderFitChart(steering)`, inline SVG):** x = steering angle (deg), y = servo value; points for each usable trial (drift = navy square, left = blue circle, right = blue triangle, centre = hollow square), fitted line in `--blue`, dashed horizontal lines at servo limits labelled "servo limit". Axis labels in body font; tick numbers in mono. Show on the steering screen once ≥ 2 accepted steering trials exist, computed client-side with the same least-squares formula as `_linear_fit` in `calibration_math.py`, and on the report from `report.steering.trial_results` (which carries `actual_steering_angle_rad`). Client-side angle for display: drift `atan(L·2d/(s²+d²))`, circles `atan(L/R)` with the circle's R (axle `D/2`, tires `(inner+outer)/4`), sign by side. Label the chart caption plainly: "Each point is one test. The line is the suggested setting."

**Screens and copy (use this wording; sentence case; Canadian spelling):**

- *Top bar:* logo `/icon.png` 36 px, wordmark "SFU Racerbot" (Saira 800 uppercase, "Racerbot" in `--blue-hi` like the site), then "Calibration" in `--mist`. Right side: connection state text ("Connected" / "Reconnecting…"), and a small line "Read-only: drive with the remote and hold LB". "New session" ghost button when a session exists.
- *Setup (no session):* title "Calibration". Lead: "Pick what to calibrate, check the current values, and start. You drive the car with the remote; this page only records." Three choices as painted-corner boxes (the site's `.slot`: 2 px navy corner marks, not full borders), selectable with radio semantics: "Speed" — "Stationary offset and tape-measured distance runs. About 15 minutes."; "Speed and steering" — "Everything in Speed, then the steering tests. About 30 minutes."; "Steering" — "A straight-line drift run and left and right circles. About 15 minutes." Then "Current values" with the five parameter inputs (labels: "Odometry ERPM gain (signed)", "Odometry ERPM offset", "Steering to servo gain", "Steering to servo offset", "Wheelbase (m)") and a note from `live_parameter_status`. Button: "Start calibration".
- *Preflight:* title "Preflight". Lead: "Start the normal bringup and keep the car still with the remote on." Topic checks as a timing-tower list (status square, topic name in mono, rate and age, "required"/"optional"). Numbered list: 1 "Put the car on level ground with clear space around it." 2 "Turn on the remote and start bringup. Leave the sticks centred." 3 "If odometry shows missing, hold LB for a moment with the sticks centred." Button "Continue" (disabled until odometry is fresh; show "Waiting for odometry" next to it).
- *Stationary:* title "Stationary baseline". Lead: "Measures the raw motor reading when nothing moves." Steps: "Release LB and leave the sticks centred." / "Press Record, don't touch the car for 5 seconds, then press Stop." Buttons "Record" and, once one is accepted, "Continue".
- *Movement:* title "Distance runs". Lead: "Drive a measured straight line; the wizard compares it with what the car counted." Steps: "Tape out a straight 5 to 10 m lane." / "Mark the rear-axle centre at the start." / "Press Record before moving. Hold LB and drive smoothly to the end mark." / "Stop, release LB, press Stop, then measure." Progress: race plates "Run 1", "Run 2", "Run 3" (third labelled "recommended"). Note: "Two forward and one reverse is best."
- *Steering:* title "Steering". Lead: "Three kinds of test: a straight run finds true centre, and circles to each side find how far the wheels really turn." Test picker (segmented control, three buttons, `aria-pressed`): "Straight-line drift", "Left circle", "Right circle". Instructions per test:
  - Drift: 1 "Tape a straight line at least 6 m long." 2 "Put the rear-axle centre on the line with the car pointing along it." 3 "Press Record. Hold LB and drive slowly about 5 m forward without touching the steering stick." 4 "Stop, release LB, press Stop." 5 "Measure how far along the line the rear-axle centre is, and how far it ended up to the left or right of the line."
  - Circle (left/right): 1 "Mark the ground under the rear-axle centre." 2 "Press Record. Hold LB, hold the steering stick fully [left/right], and drive one slow full circle back to your mark." 3 "Stop, release LB, press Stop." 4 "Measure the circle. Either measure the diameter traced by the rear-axle centre, or measure the inner and outer rear-tire circles; the wizard takes the midpoint." Tip: "A second circle with the stick half over shows whether the steering is linear."
  Progress plates: "Drift" (needed), "Left full lock" (needed), "Right full lock" (needed). Button "Record [test name]". Show the steering diagram and, when available, the fit chart. "Build report" appears once left and right full-lock circles exist.
- *Recording (any stage):* big timer (Saira 800, ~4rem) with a pulsing red dot (respect `prefers-reduced-motion`), the capture's name, live distance and (for circles) turns so far. One sentence: "Stop the car and release LB before pressing Stop." Button "Stop" (danger).
- *Review before accepting:* title "Check this run". Summary as a definition list (duration, odom distance, raw ERPM integral, odom yaw, median servo) with numbers in mono. Warnings list (amber left rule) or "No problems found in the recording." Inputs per kind (see Interfaces). Drift inputs: "Forward distance along the line (m)", "Sideways offset at the end (m)", radio "Left of the line / Right of the line", checkbox "Ended exactly on the line". Circle inputs: radio "Rear-axle centre" / "Inner and outer rear tires"; fields "Diameter (m)" or "Inner tire circle diameter (m)" + "Outer tire circle diameter (m)"; checkbox "Stick was held fully over (full lock)", checked by default. Notes textarea. Confirmation checkbox: "These measurements are for this run." Buttons "Accept run" and "Discard".
- *Accepted runs table:* columns #, Test, Measurement, Servo, Odom; race-plate number; "Remove" ghost button (confirm dialog).
- *Report:* title "Report". Status plate "Ready" (green) or "Needs review" (amber). The `safety_note` as the lead. Parameter table: Parameter (mono), Now, Suggested, Change (signed difference); rows only for keys present in `parameter_suggestions`; missing value shows "Not enough data". Movement section only if `report.movement`. Steering section: fit chart, per-side table (Full-lock radius m, Full-lock angle deg, On servo limit yes/no), "Servo limits allow up to X° left and Y° right" (from `max_commandable_steering`), rear track, residual, warnings. "Suggested YAML" code block. Buttons "Download Markdown", "Download JSON", "Recalculate", "Add another run" (goes back to the last test stage of the mode).
- *Toasts:* bottom-left, navy background, white text; errors with a `--bad` left bar. Toast copy says what happened: "Run accepted and saved on the car.", "Run discarded.", "Report saved on the car.", "Session started."

**Rules:** no emoji; no `→` or `↗` glyphs in buttons or links; no "·"-joined meta strings; no all-caps labels except the site's heading style and the single step eyebrow; no marketing lines. Escape every server-provided string with the existing `escapeHtml`. Keep all behaviour in `wizard.js` (no inline `<script>` blocks other than loading it); inline `onclick` handlers are fine as today. `index.html` gets `<link rel="icon" href="/icon.png">` and `<meta name="theme-color" content="#0c1e3a">`.

- [ ] **Step 1:** Update `setup.py` data_files (add a separate `('share/' + package_name + '/web/fonts', [...four woff2 files...])` entry and add `'web/icon.png'` to the web entry).
- [ ] **Step 2:** Write `index.html` (shell: top bar, checker strip, `#step-nav` rail, `#wizard-view` main, `#live-panel` aside, mobile live strip `#live-strip`, `#toast-region`).
- [ ] **Step 3:** Write `style.css` with the tokens and components above, plus the responsive rules.
- [ ] **Step 4:** Rewrite `wizard.js` rendering for every screen above, the `MODE_STAGES` table, the steering diagram, the fit chart, and the per-kind accept forms. Keep the networking code.
- [ ] **Step 5: Verify.**

```bash
node --check src/odom_calibration/web/wizard.js
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select odom_calibration
ls install/odom_calibration/share/odom_calibration/web/fonts install/odom_calibration/share/odom_calibration/web/icon.png
grep -n "googleapis\|gstatic\|cdn" src/odom_calibration/web/* || echo "offline-safe"
grep -n "→\|·" src/odom_calibration/web/wizard.js src/odom_calibration/web/index.html || echo "no arrow or middot glyphs"
python3 -m pytest src/odom_calibration/test -q
```
Expected: no syntax error, four fonts and icon installed, "offline-safe", "no arrow or middot glyphs", tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/odom_calibration/setup.py src/odom_calibration/web
git commit -m "odom_calibration: redesign wizard UI in the SFU Racerbot style, add steering screens"
```

---

### Task 4: Documentation

**Files:**
- Modify: `src/odom_calibration/README.md`
- Modify: `docs/odom-calibration.md`

- [ ] **Step 1: `src/odom_calibration/README.md`.** Update "Choose a mode" (three modes), replace "### 5. Optional steering" with a "### 5. Steering" section describing: why the old centred-wheels capture only reads the configured offset back; the straight-line drift test with its formula `R = (s² + d²) / (2d)`, `steering_angle = atan(wheelbase / R)` and a sentence on why heading alignment at the start matters (1° of misalignment is ~9 cm of sideways error over 5 m); the circle test with both measuring methods and why the tire-midpoint method needs no track-width constant; full-lock circles and the servo clamp (servo_min 0.15 / servo_max 0.85, published after clamping by `vesc_driver`, so full lock measures the real tightest turn); per-side gains and the asymmetry warning; the "servo limits allow" line and what to do with it (compare against `max_steering_angle` in the driving nodes' configs, do not raise it past what the car reaches). Add `servo_min`/`servo_max` to the documented parameters and mention the live "turns so far" readout is an odometry estimate. Keep existing sections that are still true.
- [ ] **Step 2: `docs/odom-calibration.md`.** Update the header block so "You'll be able to" reads "calibrate VESC speed odometry and the steering conversion with a tape measure, using a browser wizard", and add one short paragraph telling a newcomer which mode to pick (Steering only when the car pulls to one side or its turns don't match; Speed only when distances read wrong). Keep it short; point to the package README for the procedure.
- [ ] **Step 3: Verify** every command and path mentioned exists (`grep -n` for each), and Canadian spelling (`grep -n -i "tyre\|color\b\|center\b" ...` should only hit code identifiers like `steering_center`).
- [ ] **Step 4: Commit**

```bash
git add src/odom_calibration/README.md docs/odom-calibration.md
git commit -m "docs: odom wizard steering calibration and steering-only mode"
```
