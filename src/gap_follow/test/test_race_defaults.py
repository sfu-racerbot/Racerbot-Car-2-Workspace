"""The race defaults of 2026-10-27, checked against the table that justifies them.

Spec: docs/gap-follow.md#race-defaults-2026-10-27 -- the simulator table and the
list of what changed. The values are read from the YAML the car loads
(config/gap_follow.yaml). Pure YAML, no rclpy, so it runs unsourced:

    python3 -m pytest src/gap_follow/test/test_race_defaults.py -v

It also runs in the sourced gap_follow suite, unchanged.
"""
import os

import pytest
import yaml

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config',
                      'gap_follow.yaml')
with open(CONFIG, encoding='utf-8') as handle:
    P = yaml.safe_load(handle)['gap_follow_node']['ros__parameters']

OLD_TOP_SPEED = 3.5  # m/s, the top speed before the 2026-10-27 race defaults
OLD_SPEEDS = {'min_speed': 1.0, 'corner_speed': 1.5, 'corner_speed_wide': 1.9,
              'narrow_corridor_speed': 2.5}  # m/s, the same before


def test_top_speed_is_the_launch_ceiling():
    # oracle: spec -- the table's max_speed row (3.5 -> 5.0), and the launch's
    # ceiling (gap_follow/launch_speed.py MAX_SPEED_CEILING, requested 2026-10-26).
    assert P['max_speed'] == pytest.approx(5.0, abs=1e-9)  # m/s


@pytest.mark.parametrize('key', sorted(OLD_SPEEDS))
def test_speeds_tied_to_the_top_speed_scale_with_it(key):
    # oracle: closed form -- the old value x 5.0/3.5, shipped to 1 mm.
    assert P[key] == pytest.approx(OLD_SPEEDS[key] * 5.0 / OLD_TOP_SPEED, abs=5e-4)  # m/s


def test_the_race_switches_are_set_as_the_table_says():
    # oracle: spec -- the table in docs/gap-follow.md#race-defaults-2026-10-27,
    # except damping: car 2 ships it OFF until its BNO085 is mounted and
    # calibrated (docs/car1-vs-car2.md#imu; car 1 ships it on). The gain is
    # the table's, ready for that flip.
    assert P['enable_steering_damping'] is False
    assert P['steering_damping_gain'] == pytest.approx(0.10, abs=1e-9)  # s
    assert P['enable_corner_setup'] is False
    assert P['max_acceleration'] == pytest.approx(5.0, abs=1e-9)  # m/s^2
    # Ships off: the simulator put it slower on car 1's course.
    assert P['enable_adaptive_grip'] is False


def test_the_safety_values_the_race_defaults_left_alone():
    # oracle: spec -- docs/gap-follow.md#race-defaults-2026-10-27 lists none of
    # these as changed. The deadman is the workspace policy in
    # docs/architecture.md, not a tuning knob.
    assert P['enable_deadman'] is True
    assert P['enable_ttc'] is True
    assert P['ttc_threshold_sec'] == pytest.approx(0.30, abs=1e-9)  # s
    assert P['max_braking_decel'] == pytest.approx(3.0, abs=1e-9)  # m/s^2
    assert P['emergency_stop_clearance'] == pytest.approx(0.0, abs=1e-9)  # m
    assert P['forward_stop_clearance'] == pytest.approx(0.20, abs=1e-9)  # m
    # Car 2's limit (docs/car1-vs-car2.md, from its 2026-10-16 circles; car 1: 0.33).
    assert P['max_steering_angle'] == pytest.approx(0.349, abs=1e-9)  # rad


def test_the_course_defaults_of_2026_10_30():
    # oracle: spec -- docs/gap-follow.md#speed-on-the-real-course-2026-10-30.
    assert P['aim_depth_bias'] == pytest.approx(0.0, abs=1e-9)
    assert P['aim_depth_bias_fallback'] == pytest.approx(1.0, abs=1e-9)
    assert P['max_lateral_accel'] == pytest.approx(2.5, abs=1e-9)  # m/s^2
    assert P['swept_clearance_max_length'] == pytest.approx(5.0, abs=1e-9)  # m
    assert P['opponent_follow_floor'] == pytest.approx(0.5, abs=1e-9)  # m/s


def test_the_braking_lookahead_covers_a_measured_stop_from_top_speed():
    # oracle: recorded measurement, run 20261030-175124: LB released at 5.41 m/s on
    # a straight, 3.39 m by the wall ahead to 0.29 m/s. A shorter lookahead than the
    # real stop means the braking ceiling cannot act in time at top speed.
    assert P['max_speed'] <= 5.41
    assert P['swept_clearance_max_length'] >= 3.39 + P['forward_stop_clearance']
