"""R1 conservative safety inputs; run with ROS-sourced package pytest."""
import pytest
from rclpy.duration import Duration

from test_gap_follow_fused_gate_node import FUSED, _boundary_command
from test_gap_follow_traction_node import _node, _odom, _tick
from test_gap_follow_steering_damping_node import (
    _capture, _feed_imu, _feed_odom, _hold_deadman, _scan)


def test_fused_below_wheels_keeps_the_feature_off_published_stop():
    # oracle: invariant R1, same wheel speed and history must retain the stop.
    assert _boundary_command('ok') == _boundary_command('ok', fused=False)
    assert _boundary_command('ok')[0] == 0.0


def _higher_boundary(fused):
    with _node(False, FUSED if fused else ()) as n:
        commands = _capture(n)
        _feed_imu(n, yaw_rate=0.0)
        _feed_odom(n, 2.2 if fused else 2.0)
        _odom(n.wheel_odom_callback, 2.0)
        _hold_deadman(n)
        n.last_commanded_speed = 1.5
        n.last_command_time = n.get_clock().now() - Duration(seconds=0.025)
        # oracle: closed form: TTC(2.2) < threshold < TTC(2.0).
        clearance = 2.1 * n.ttc_threshold_sec
        front = (n.wheelbase + n.car_length) / 2 - n.laser_offset_x
        ranges = [8.0] * 541
        ranges[270] = front + clearance
        ranges[0] = n.car_width / 2 + clearance * 0.75
        n.scan_callback(_scan(ranges))
        return commands[-1].drive.speed, n.last_decision_state


def test_fused_above_wheels_brakes_at_the_higher_speed_boundary():
    # oracle: closed form above; the higher estimate must brake earlier.
    assert _higher_boundary(True) == (0.0, 'ttc_brake')
    assert _higher_boundary(False)[0] > 0.0


def test_fresh_fused_stream_cannot_mask_stale_wheel_odometry():
    with _node(False, FUSED) as n:
        commands = _capture(n)
        _feed_imu(n, yaw_rate=0.0)
        _feed_odom(n, 1.8)
        _odom(n.wheel_odom_callback, 2.0)
        n.last_wheel_time -= Duration(seconds=n.odom_timeout_sec)
        _hold_deadman(n)
        n.last_commanded_speed = 1.5
        _tick(n)
        # oracle: spec R1: /odom freshness remains mandatory at the boundary.
        assert commands[-1].drive.speed == 0.0
        assert n.last_decision_state == 'odometry_stale'


@pytest.mark.parametrize('wheel,fused', [(2.0, 1.8), (2.0, 2.2),
                                          (-2.0, 2.2), (0.0, 2.2)])
def test_safety_speed_retains_wheel_direction_and_larger_magnitude(wheel, fused):
    import math
    with _node(False, FUSED) as n:
        _feed_imu(n, yaw_rate=0.0)
        _feed_odom(n, fused)
        _odom(n.wheel_odom_callback, wheel)
        # oracle: closed form R1, including standstill and reverse sign.
        assert n._safety_speed() == math.copysign(max(abs(wheel), abs(fused)), wheel)


def _independent_ttc_command(wheel, fused, use_fused=True, mirror=False):
    with _node(False, FUSED if use_fused else ()) as n:
        commands = _capture(n)
        _feed_imu(n, yaw_rate=0.0)
        _feed_odom(n, fused if use_fused else wheel)
        _odom(n.wheel_odom_callback, wheel)
        _hold_deadman(n)
        recent_command, dt = 2.0, 0.025
        n.last_commanded_speed = recent_command
        n.last_command_time = n.get_clock().now() - Duration(seconds=dt)
        # oracle: closed form from shipped YAML. At speed 2, TTC is
        # (2 - fallback_threshold)*T/2 < T; at speed .15 it is > T.
        # For the above-threshold cases: 1.8 < 1.9 < 2.0 m/s.
        clearance = (recent_command - n.ttc_command_fallback_max_odom_speed) * n.ttc_threshold_sec
        front = (n.wheelbase + n.car_length) / 2 - n.laser_offset_x
        ranges = [8.0] * 541
        ranges[270] = front + clearance
        # Non-directional geometry prevents an escape from masking the brake.
        ranges[0] = n.car_width / 2 + clearance * 0.75
        assert clearance * 0.75 > n.emergency_stop_clearance
        assert dt < n.ttc_command_speed_timeout_sec
        scan = _scan(ranges)
        if mirror:
            from pathlib import Path
            import sys
            sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools/f1tenth_sim'))
            from gap_mirror import GapController, node_params

            controller = GapController(node_params(), scan)
            controller.last_commanded_speed = recent_command
            controller.last_command_time = 0
            steering, speed = controller.command(
                scan.ranges, wheel, 1, yaw_rate=0.0,
                fused_speed=fused if use_fused else None)
            return speed, steering, controller.stop_reason
        n.scan_callback(scan)
        return (commands[-1].drive.speed, commands[-1].drive.steering_angle,
                n.last_decision_state)


@pytest.mark.parametrize('wheel,fused', [(0.05, 0.15), (0.0, 0.15),
                                       (0.1, 0.15), (-0.05, -0.15)])
def test_wheel_command_fallback_stop_survives_fused_threshold_crossing(wheel, fused):
    on = _independent_ttc_command(wheel, fused)
    off = _independent_ttc_command(wheel, fused, use_fused=False)
    # oracle: invariant R1 round 2: preserve the raw-wheel effective TTC stop,
    # including zero, reverse magnitude and equality at the fallback threshold.
    assert on == off == (0.0, 0.0, 'ttc_brake')


def test_fused_command_fallback_also_brakes_when_wheels_cross_the_threshold():
    # oracle: closed form above: the fused .05 sample independently uses the
    # recent 2 m/s command, even though raw wheels at .15 alone do not brake.
    assert _independent_ttc_command(0.15, 0.05) == (0.0, 0.0, 'ttc_brake')
    assert _independent_ttc_command(0.15, 0.05, use_fused=False)[0] > 0.0


@pytest.mark.parametrize('wheel,fused', [(2.0, 1.8), (1.8, 2.0)])
def test_above_fallback_threshold_the_larger_speed_still_governs(wheel, fused):
    on = _independent_ttc_command(wheel, fused)
    larger_raw = _independent_ttc_command(max(wheel, fused), fused, use_fused=False)
    # oracle: closed form: no fallback for either source; max measurement
    # gives TTC=.285 s < shipped .30 s, whereas 1.8 gives .3167 s.
    assert on == larger_raw == (0.0, 0.0, 'ttc_brake')
    assert _independent_ttc_command(min(wheel, fused), fused, use_fused=False)[0] > 0.0


@pytest.mark.parametrize('wheel,fused', [(0.05, 0.15), (0.15, 0.05),
                                       (2.0, 1.8), (1.8, 2.0)])
def test_mirror_combines_independent_ttc_results_like_the_node(wheel, fused):
    # oracle: invariant: real node published stop and mirror command agree;
    # neither receives a mocked TTC computation or a second scan pipeline.
    assert _independent_ttc_command(wheel, fused, mirror=True) == (
        _independent_ttc_command(wheel, fused)) == (0.0, 0.0, 'ttc_brake')
