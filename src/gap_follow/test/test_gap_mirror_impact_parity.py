"""Independent impact histories in node/mirror; ROS-sourced package pytest.

Mutation evidence (2026-10-29): a mirror that never latches fails enabled
parity; a mirror that always stops fails feature-off parity.
"""
import numpy as np
import pytest

from test_gap_follow_impact_node import _node, _capture, _joy, _imu, _tick
from test_gap_follow_steering_damping_node import _scan
from test_gap_mirror_parity import gap_mirror, _Lidar


@pytest.mark.parametrize('enabled', [True, False])
def test_impact_mirror_spike_hold_release_press_parity(enabled):
    with _node(enabled) as n:
        commands = _capture(n)
        scan = _scan()
        params = gap_mirror.node_params()
        params['enable_imu_impact_stop'] = enabled
        params['enable_launch_boost'] = False  # mid-race, as _node's node
        mirror = gap_mirror.GapController(params, _Lidar(scan))
        mirror.last_command_time = -1
        speeds = []
        for tick in range(206):
            pressed = tick != 203
            h = n.imu_impact_threshold + 1 if tick == 1 else 0.0
            _joy(n, pressed)
            _imu(n, h)
            _tick(n, tick)
            steering, speed = mirror.command(np.asarray(scan.ranges), 1.0, tick,
                                              yaw_rate=0.0, horizontal_accel=h,
                                              lb_pressed=pressed)
            # oracle: integration invariant: independent state, identical inputs.
            assert commands[-1].drive.speed == pytest.approx(speed, abs=1e-9)  # m/s
            assert commands[-1].drive.steering_angle == pytest.approx(steering, abs=1e-9)  # rad
            speeds.append(speed)
        # oracle: spec I6: spike stops until release (deadman) then press.
        assert speeds[0] > 0
        if enabled:
            assert speeds[1:204] == [0.0] * 203
        else:
            assert all(s > 0 for s in speeds[:203])
            assert speeds[203] == 0
        assert speeds[204] > 0
