"""Independent controller histories through slip, release and IMU dropout."""
import numpy as np
import pytest
import rclpy
from rclpy.duration import Duration

from test_gap_follow_understeer_node import _make, _inputs, _measured, DT
from test_gap_follow_steering_damping_node import _capture, _hold_deadman
from test_gap_follow_corners_node import _corridor, _raycast
from test_gap_mirror_parity import gap_mirror, _Lidar


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('hold_sec', [0.0, 2.0])
def test_understeer_mirror_matches_each_command_with_independent_state(enabled, hold_sec):
    node = _make(enabled, ['-p', f'speed_hold_sec:={hold_sec}'])
    try:
        published = _capture(node)
        params = gap_mirror.node_params()
        params.update(enable_imu_understeer=enabled, command_slew_max_dt=DT,
                      speed_hold_sec=hold_sec)
        scan = _raycast(_corridor(1.4, 0.6))
        mirror = gap_mirror.GapController(params, _Lidar(scan))
        engaged = []
        for tick in range(150):
            _hold_deadman(node)
            _inputs(node, 'stale-imu' if tick >= 140 else None)
            # A sensor responding to the previous physical command. Both
            # controllers receive this same input; neither's state is copied.
            command = published[-1].drive.steering_angle if published else 0.0
            measured = (0.5 if tick < 70 else 1.0) * command
            _measured(node, measured)
            node.last_command_time = node.get_clock().now() - Duration(seconds=DT)
            node.scan_callback(scan)
            steering, speed = mirror.command(
                np.asarray(scan.ranges), 2.0, tick,
                yaw_rate=0.0 if tick < 140 else None, measured_steering=measured)
            # oracle: integration invariant; no state is copied between ticks.
            assert published[-1].drive.speed == pytest.approx(speed, abs=1e-6)  # m/s
            assert published[-1].drive.steering_angle == pytest.approx(steering, abs=1e-6)  # rad
            engaged.append(mirror.telemetry.get('imu_understeer_engaged', False))
        # oracle: scenario check: slip engages, healthy turn releases, flag off
        # cannot engage despite exactly the same low measured-turn inputs.
        assert any(engaged[30:70]) == enabled
        assert not any(engaged[120:])
        if not enabled:
            assert all(p.drive.speed > 2.0 * params['imu_understeer_speed_factor']
                       for p in published[30:70])
    finally:
        node.destroy_node()
        rclpy.shutdown()
