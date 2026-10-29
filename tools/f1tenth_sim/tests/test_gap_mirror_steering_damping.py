"""gap_mirror's steering damping term: the simulator's copy of gap_follow's law
(gap_follow.gap_logic.steering_damping, docs/gap-follow.md#steering-damping-gyro).

The parity test (src/gap_follow/test/test_gap_mirror_parity.py) is the check
against the real node. This file checks the mirror on its own, unsourced:

- with the flag off, a yaw rate changes nothing;
- with the flag on and no healthy IMU (yaw_rate=None), nothing is added, as on
  the car when /imu/status is not 'ok';
- with the flag on, the command moves by exactly -gain x yaw on the same scan,
  and the term is added before the clip, so a large term cannot exceed the
  steering limit;
- a gap_follow.yaml from before the feature loads with it off (LEGACY_VALUES).

Unsourced:

    python3 -m pytest tools/f1tenth_sim/tests/test_gap_mirror_steering_damping.py -v -p no:cacheprovider
"""
import os
import sys

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'src', 'gap_follow'))
import gap_mirror  # noqa: E402

GAIN = 0.07   # the test gain; the shipped value is in gap_follow.yaml


class _Lidar:
    """The car's LiDAR sweep: 1081 beams over 270 degrees, from -135 degrees."""
    angle_min = -2.35619449
    angle_increment = 4.71238898 / 1080
    range_min = 0.023


ANGLES = _Lidar.angle_min + np.arange(1081) * _Lidar.angle_increment


def _corridor(left_wall, right_wall, far_end=30.0, max_range=10.0):
    """Ranges from a corridor whose walls are left_wall and right_wall metres to each side."""
    s, c = np.sin(ANGLES), np.cos(ANGLES)
    side = np.where(s > 0, left_wall / np.maximum(s, 1e-9), right_wall / np.maximum(-s, 1e-9))
    ahead = np.where(c > 0, far_end / np.maximum(c, 1e-9), np.inf)
    return np.clip(np.minimum(side, ahead), 0.05, max_range)


# A gap to the left, so the undamped command steers left. Built once.
SCAN = _corridor(left_wall=1.4, right_wall=0.6)


def _tuned(params):
    """The test overrides: the arc guard is off, so only the term under test
    can move the command, and the slew limit is out of the way."""
    params = dict(params)
    params['enable_arc_guard'] = False
    params['max_steering_rate'] = 100.0
    return params


def _command(params, yaw_rate=None):
    ctrl = gap_mirror.GapController(params, _Lidar())
    steering, _speed = ctrl.command(SCAN, 1.0, 0, yaw_rate=yaw_rate)
    assert not ctrl.stopped, 'the test scene must drive'
    return steering, ctrl.telemetry


def _off():
    # Pinned off explicitly: the shipped YAML turns steering damping on since
    # 2026-10-27, and "off" has to mean off for the comparisons that use it.
    return _tuned(dict(gap_mirror.node_params(), enable_steering_damping=False))


def _on(gain=GAIN):
    return _tuned(dict(gap_mirror.node_params(), enable_steering_damping=True,
                       steering_damping_gain=gain))


def test_flag_off_ignores_the_yaw_rate():
    assert _command(_off(), yaw_rate=1.0)[0] == _command(_off(), yaw_rate=None)[0]


def test_flag_on_without_a_healthy_imu_adds_nothing():
    assert _command(_on(), yaw_rate=None)[0] == _command(_off(), yaw_rate=None)[0]


def test_flag_on_moves_the_command_by_minus_gain_times_yaw():
    # oracle: closed form -- on the same scan, the difference is -GAIN x yaw.
    undamped, _ = _command(_off())
    damped, tel = _command(_on(), yaw_rate=1.0)
    assert damped - undamped == pytest.approx(-GAIN * 1.0, abs=1e-9)  # rad
    assert tel['steering_damping'] == pytest.approx(-GAIN * 1.0, abs=1e-12)  # rad


def test_a_right_yaw_rate_steers_left_by_the_same_amount():
    # oracle: invariant -- the mirror image of a left yaw rate is an equal and
    # opposite term.
    undamped, _ = _command(_off())
    damped, _ = _command(_on(), yaw_rate=-1.0)
    assert damped - undamped == pytest.approx(GAIN * 1.0, abs=1e-9)  # rad


def test_the_damped_request_is_clipped_like_any_other_request():
    # The term joins the request before the clip (the requested_steering line in
    # gap_mirror.command). oracle: the configured limit. Added after the clip, a
    # -50 rad/s yaw (term +3.5 rad) would command far past the limit.
    params = _on()
    steering, _ = _command(params, yaw_rate=-50.0)
    assert steering == pytest.approx(params['max_steering_angle'], abs=1e-9)  # rad


def test_a_yaml_from_before_the_feature_loads_with_it_off(tmp_path):
    # oracle: LEGACY_VALUES -- the values that reproduce the behaviour from
    # before the parameters existed, i.e. the feature off.
    text = yaml.safe_load(gap_mirror.NODE_CONFIG.read_text())
    for name in ('enable_steering_damping', 'steering_damping_gain'):
        text['gap_follow_node']['ros__parameters'].pop(name)
    path = tmp_path / 'old_gap_follow.yaml'
    path.write_text(yaml.safe_dump(text))
    old = _tuned(gap_mirror.node_params(path, fill_legacy=True))
    assert old['enable_steering_damping'] is False
    assert _command(old, yaw_rate=1.0)[0] == _command(_off(), yaw_rate=None)[0]
