"""Node/mirror histories with independent state; ROS-sourced pytest runner."""
import numpy as np
import pytest

from test_gap_follow_traction_node import _node, _inputs, _tick
from test_gap_follow_steering_damping_node import _capture, _hold_deadman, _scan
from test_gap_mirror_parity import gap_mirror, _Lidar


@pytest.mark.parametrize('enabled', [False, True])
def test_traction_mirror_matches_hold_expiry_rearm_and_dropout(enabled):
    with _node(enabled) as n:
        commands = _capture(n)
        scan = _scan()
        params = gap_mirror.node_params()
        params['enable_imu_traction'] = enabled
        mirror = gap_mirror.GapController(params, _Lidar(scan))
        # Both histories start one control period before tick zero.
        mirror.last_command_time = -1
        held = []
        for tick in range(100):
            # First ramp has subthreshold slip. Two sustained slip stretches
            # are separated by a low-slip rearm tick; last tick loses the IMU.
            fused = 1.8 if tick < 5 else 2.0 if tick == 60 else 1.7
            _inputs(n, fused=fused, failure='fault' if tick == 99 else None)
            _hold_deadman(n)
            _tick(n, tick)
            steering, speed = mirror.command(np.asarray(scan.ranges), 2.0, tick,
                                              yaw_rate=None if tick == 99 else 0.0,
                                              fused_speed=fused)
            # oracle: integration invariant: same inputs, independent histories.
            assert commands[-1].drive.speed == pytest.approx(speed, abs=1e-9)  # m/s
            assert commands[-1].drive.steering_angle == pytest.approx(steering, abs=1e-9)  # rad
            held.append(mirror.telemetry.get('imu_traction_holding', False))
        # oracle: scenario: real engagement, max-hold release, lockout, rearm.
        assert any(held[5:24]) == enabled
        assert not any(held[24:61])
        assert any(held[61:80]) == enabled
        assert not held[99]
