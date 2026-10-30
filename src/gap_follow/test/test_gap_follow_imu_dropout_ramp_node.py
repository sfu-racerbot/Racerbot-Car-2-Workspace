"""I4/I5 dropout uses actual command history; ROS-sourced package pytest.

Both cases rejected an in-memory mutation that published the never-capped
controller's command on dropout (2026-10-29), rather than the seeded oracle.
"""
import pytest
from nav_msgs.msg import Odometry
from rclpy.duration import Duration

from test_gap_follow_impact_node import _node, _joy, _imu, _tick, _command
from test_gap_follow_steering_damping_node import _capture, _feed_odom, _scan
from test_gap_follow_understeer_node import _measured
from test_gap_follow_corners_node import _corridor, _raycast


def _run(feature, mode, seed=None):
    enabled = mode == 'dropout'
    with _node(False, ['-p', f'enable_imu_{feature}:={str(enabled).lower()}']) as n:
        commands = _capture(n)
        scan = _raycast(_corridor(1.4, .6)) if feature == 'understeer' else _scan()
        _joy(n)
        _imu(n)
        _feed_odom(n, 2.0)
        _measured(n, .02)
        if feature == 'understeer' and enabled:
            for _ in range(40):
                n._understeer_cap(.152, .025)
            assert n.understeer_check.engaged
        fused = Odometry()
        fused.twist.twist.linear.x = 1.7
        n.fused_odom_callback(fused)
        n.steering_basis = .152 if feature == 'understeer' else 0.0
        n.last_commanded_speed = 3.0
        previous = n.last_commanded_speed
        _tick(n, scan=scan, speed=2.0)
        first = _command(commands[-1])
        # The paired feature-off seeded node reproduces the actual post-cap
        # state; it has not lived the never-capped command history.
        if mode == 'seeded':
            n.last_commanded_speed, n.steering_basis = seed
        if enabled:
            assert (n.understeer_check.engaged if feature == 'understeer'
                    else n.traction_hold.holding)
            if feature == 'understeer':
                assert first[1] == 2.0 * n.imu_understeer_speed_factor
        _imu(n)
        n.last_imu_time -= Duration(seconds=1)
        _tick(n, 1, scan=scan, speed=2.0)
        return first, _command(commands[-1]), previous


@pytest.mark.parametrize('feature', ['understeer', 'traction'])
def test_dropout_ramps_from_actual_state_not_never_capped_history(feature):
    first, dropout, previous = _run(feature, 'dropout')
    _, oracle, _ = _run(feature, 'seeded', seed=(first[1], first[0]))
    uncapped_first, never_capped, _ = _run(feature, 'never-capped')
    # oracle: feature-off law seeded with identical last command, rack, odom,
    # scan and dt. Dropout removes the feature, not the history it produced.
    assert previous > 2.0
    assert uncapped_first[1] > 2.0
    assert dropout == oracle
    assert dropout[1] < never_capped[1]
    assert dropout[1] <= max(first[1], 2.0) + 5.0 * .025
