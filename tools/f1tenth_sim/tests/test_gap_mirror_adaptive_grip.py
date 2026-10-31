"""gap_mirror's adaptive grip: the simulator's copy of gap_follow's lateral limit
(gap_follow.gap_logic.AdaptiveGrip and adaptive_grip_step,
docs/gap-follow.md#adaptive-grip-imu).

The parity test (src/gap_follow/test/test_gap_mirror_parity.py) is the check
against the real node. This file checks the mirror on its own, unsourced:

- with the flag off, a yaw rate changes nothing and the limit is max_lateral_accel;
- with the flag on and no healthy gyro (yaw_rate=None), the limit is the base and
  the command is the flag-off command;
- with the flag on, on a turn the steering asks for, the limit rises by the
  closed form min(ceiling, base + rise_per_s * t), t the sum of the command
  intervals the mirror uses;
- a gyro that drops out after a rise restarts the limit at the base;
- a gap_follow.yaml from before the feature loads with it off (LEGACY_VALUES).

Unsourced:

    python3 -m pytest tools/f1tenth_sim/tests/test_gap_mirror_adaptive_grip.py -v -p no:cacheprovider
"""
import math
import os
import sys

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'src', 'gap_follow'))
import gap_mirror  # noqa: E402

SPEED = 2.0               # m/s, forced every tick: the scene is driven at a steady speed
WHEELBASE = 0.36          # gap_follow.yaml's
ADAPTIVE_KEYS = (
    'enable_adaptive_grip', 'adaptive_grip_ceiling', 'adaptive_grip_rise_per_s',
    'adaptive_grip_cut_factor', 'adaptive_grip_hold_ratio',
    'adaptive_grip_slip_ratio', 'adaptive_grip_judge_fraction')


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


SCAN = _corridor(left_wall=1.4, right_wall=0.6)


def _params(**overrides):
    """The shipped YAML with the adaptive grip ON, the arc guard off, and the
    slew and the speed schedule out of the way, so the steering is the same on
    every tick of the scene and the yaw rate the test feeds is the only thing
    that changes. Steering damping is pinned off: it ships on since 2026-10-27,
    and these tests are about the grip. Tests that want the feature off say so."""
    params = dict(gap_mirror.node_params(), enable_arc_guard=False,
                  max_steering_rate=100.0, steering_gain_speed_ref=0.0,
                  enable_adaptive_grip=True, enable_steering_damping=False,
                  # A car mid-course, not a start: the launch boost would hold
                  # it straight, so the rack would never ask for a turn.
                  enable_launch_boost=False)
    params.update(overrides)
    return params


SCAN_RIGHT = _corridor(left_wall=0.6, right_wall=1.4)   # the mirror image: steers right


def _run(params, ticks, yaw_of_tick, scan_of_tick=None):
    """Tick one GapController `ticks` times at SPEED on SCAN, or on
    scan_of_tick(k) for tick k. yaw_of_tick(k) returns the yaw rate for tick k,
    or None for no healthy gyro. Returns the steerings, the commanded speeds and
    the lateral limits, one per tick."""
    ctrl = gap_mirror.GapController(params, _Lidar())
    steerings, speeds, limits = [], [], []
    for step in range(ticks):
        scan = SCAN if scan_of_tick is None else scan_of_tick(step)
        steering, speed = ctrl.command(scan, SPEED, step, yaw_rate=yaw_of_tick(step))
        assert not ctrl.stopped, f'tick {step}: the test scene must drive'
        steerings.append(steering)
        speeds.append(speed)
        limits.append(ctrl.telemetry['lateral_limit'])
    return steerings, speeds, limits


def _rack_steering(scan=SCAN):
    """The steering this scene asks for, with the grip off: it does not depend on
    the lateral limit, and it is the same on every tick."""
    steerings, _speeds, _limits = _run(_params(enable_adaptive_grip=False), 1,
                                       lambda k: None, lambda k: scan)
    return steerings[0]


def _judged_start(intervals, lag):
    """The first tick the mirror can judge: the first whose window's edge, a lag
    before it on the control clock, reaches back to the first sample. The clock is
    the running sum of the command intervals, as gap_logic.SteeringLag keeps it."""
    clock, times = 0.0, []
    for dt in intervals:
        clock += dt
        times.append(clock)
    return next(k for k, t in enumerate(times) if t - lag >= times[0] - 1e-9)


def _turn_yaw(steering):
    """The yaw rate a car makes on the turn `steering` asks for: no slip."""
    return SPEED * math.tan(steering) / WHEELBASE


def test_with_the_flag_off_a_yaw_rate_changes_nothing():
    params = _params(enable_adaptive_grip=False)
    none_steer, _speed, _limits = _run(params, 5, lambda k: None)
    yaw_steer, _speed, yaw_limits = _run(params, 5, lambda k: 1.0)
    assert yaw_steer == none_steer
    assert yaw_limits == [params['max_lateral_accel']] * 5


def test_with_the_grip_on_and_no_healthy_gyro_the_command_is_the_flag_off_command():
    # oracle: spec -- with the gyro not trusted the limit is max_lateral_accel,
    # so the command is the one the node publishes with the feature off.
    off_steer, off_speed, _l = _run(_params(enable_adaptive_grip=False), 5, lambda k: None)
    on_steer, on_speed, on_limits = _run(_params(), 5, lambda k: None)
    assert on_steer == off_steer
    assert on_speed == off_speed
    assert on_limits == [_params()['max_lateral_accel']] * 5


def test_a_turn_the_steering_asks_for_raises_the_limit_by_the_closed_form():
    # oracle: closed form. On the turn the rack asks for, every judged tick has
    # grip ratio 1.0 and a demand above 0.8 of the limit, so the limit is
    # min(ceiling, base + rise_per_s * t), t the sum of the command intervals
    # from the first judged tick on. Nothing is judged until a lag of steering is
    # recorded. The mirror's intervals: command_slew_max_dt on the first tick (no
    # command before it), then CONTROL_DT each tick.
    params = _params()
    yaw = _turn_yaw(_rack_steering())
    ticks = 40
    _steer, _speed, limits = _run(params, ticks, lambda k: yaw)
    intervals = [params['command_slew_max_dt']] + [gap_mirror.CONTROL_DT] * (ticks - 1)
    base, rise = params['max_lateral_accel'], params['adaptive_grip_rise_per_s']
    first = _judged_start(intervals, params['adaptive_grip_steering_lag_sec'])
    expected, t = [base] * first, 0.0
    for k in range(first, ticks):
        t += intervals[k]
        expected.append(min(params['adaptive_grip_ceiling'], base + rise * t))
    assert limits == pytest.approx(expected, abs=1e-9)  # m/s^2


def test_a_steering_jump_holds_the_limit_for_a_lag_and_then_judging_resumes():
    # oracle: closed form on the control clock. The scene switches to the mirror
    # image on tick 20, so the rack's steering changes sign. The yaw is the
    # kinematic yaw of the new steering. Until the window's edge is a lag past the
    # switch, the steering a lag ago is the old one, far more than 0.05 rad away:
    # no judgement, so the limit holds exactly. Judging resumes on the first tick
    # whose edge is at the switch.
    params = _params()
    left, right = _rack_steering(SCAN), _rack_steering(SCAN_RIGHT)
    assert abs(right - left) > 0.05, (left, right)      # rad: well over the threshold
    switch, ticks = 20, 32
    _s, _sp, limits = _run(
        params, ticks,
        lambda k: _turn_yaw(left if k < switch else right),
        lambda k: SCAN if k < switch else SCAN_RIGHT)
    intervals = [params['command_slew_max_dt']] + [gap_mirror.CONTROL_DT] * (ticks - 1)
    clock = np.cumsum(intervals)
    lag = params['adaptive_grip_steering_lag_sec']
    reopen = next(k for k in range(switch + 1, ticks)
                  if clock[k] - clock[switch] >= lag - 1e-9)
    assert limits[switch - 1] > params['max_lateral_accel'] + 0.05   # m/s^2: risen
    for k in range(switch, reopen):
        assert limits[k] == pytest.approx(limits[switch - 1], abs=1e-12)  # m/s^2: held
    assert limits[reopen] > limits[reopen - 1]          # m/s^2: judging resumed


def test_a_gyro_that_drops_out_restarts_the_limit_at_the_base():
    # oracle: closed form. Twenty judged ticks raise the limit; tick 20 has no
    # healthy gyro, so the limit is the base; tick 21 rises from the base by
    # rise_per_s times its interval, CONTROL_DT, so the filter restarted too.
    params = _params()
    yaw = _turn_yaw(_rack_steering())
    _steer, _speed, limits = _run(params, 22, lambda k: None if k == 20 else yaw)
    base = params['max_lateral_accel']
    assert limits[19] > base + 0.05  # m/s^2: risen before the dropout
    assert limits[20] == pytest.approx(base, abs=1e-12)  # m/s^2: reset
    assert limits[21] == pytest.approx(base + params['adaptive_grip_rise_per_s']
                                       * gap_mirror.CONTROL_DT, abs=1e-9)  # m/s^2


def test_a_gyro_that_drops_out_restarts_the_low_pass_as_well_as_the_limit():
    # oracle: closed form. Ten slip ticks leave the ratio filter near 0.5; a
    # dropout, then one full-grip tick. The filter restarts, so that tick is
    # judged on its own ratio (1.0) and rises by rise_per_s times CONTROL_DT.
    # Had the filter kept its slip value, the tick would read as a slip.
    params = _params()
    yaw = _turn_yaw(_rack_steering())

    def yaw_of_tick(k):
        if k < 10:
            return 0.5 * yaw
        return None if k == 10 else yaw

    _steer, _speed, limits = _run(params, 12, yaw_of_tick)
    base = params['max_lateral_accel']
    assert limits[10] == pytest.approx(base, abs=1e-12)  # m/s^2: the dropout
    assert limits[11] == pytest.approx(
        base + params['adaptive_grip_rise_per_s'] * gap_mirror.CONTROL_DT, abs=1e-9)


def test_a_yaml_from_before_the_feature_loads_with_it_off(tmp_path):
    # oracle: LEGACY_VALUES -- the values that reproduce the behaviour from
    # before the parameters existed, i.e. the feature off.
    text = yaml.safe_load(gap_mirror.NODE_CONFIG.read_text())
    for name in ADAPTIVE_KEYS:
        text['gap_follow_node']['ros__parameters'].pop(name)
    path = tmp_path / 'old_gap_follow.yaml'
    path.write_text(yaml.safe_dump(text))
    old = gap_mirror.node_params(path, fill_legacy=True)
    assert old['enable_adaptive_grip'] is False
    off_steer, off_speed, _l = _run(_params(enable_adaptive_grip=False), 5, lambda k: 1.0)
    # Damping and the launch boost are pinned off on this side too, as _params
    # pins them: the legacy file keeps the current YAML's keys for both.
    old_steer, old_speed, _l = _run(dict(old, enable_arc_guard=False, max_steering_rate=100.0,
                                         steering_gain_speed_ref=0.0,
                                         enable_steering_damping=False,
                                         enable_launch_boost=False),
                                    5, lambda k: 1.0)
    assert old_steer == off_steer
    assert old_speed == off_speed
