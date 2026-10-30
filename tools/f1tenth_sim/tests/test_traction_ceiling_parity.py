"""R2 mirror parity without starting a simulator; package/tools pytest."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gap_mirror import GapController, node_params


def _mirror(enabled, fused):
    class Lidar:
        angle_min = -np.pi / 2
        angle_increment = np.pi / 540
        range_min = 0.023

    controller = GapController(dict(node_params(), enable_imu_traction=enabled), Lidar())
    controller.last_commanded_speed = 1.5
    controller.last_command_time = 0
    return controller.command(np.full(541, 8.0), 2.0, 1,
                              yaw_rate=0.0, fused_speed=fused)[1]


@pytest.mark.parametrize('fused', [1.7, 1.8, 2.2])
def test_mirror_traction_only_lowers(fused):
    # oracle: invariant R2, same scan and state with only traction switched.
    assert _mirror(True, fused) <= _mirror(False, fused)


def test_review_case_mirror_matches_node_and_closed_form_ramp():
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/gap_follow/test'))
    from test_gap_follow_traction_bounds import _bounded_command

    on, _ = _bounded_command(True, 2.2)
    # oracle: closed form: max(last=1.5,wheel=2) + a*dt, in m/s.
    assert _mirror(True, 2.2) == _mirror(False, 2.2) == on == 2.0 + 5.0 * 0.025


def test_mirror_names_the_traction_off_ceiling_when_it_binds():
    class Lidar:
        angle_min = -np.pi / 2
        angle_increment = np.pi / 540
        range_min = 0.023

    def binding(enabled):
        controller = GapController(dict(node_params(), enable_imu_traction=enabled), Lidar())
        controller.last_commanded_speed = 1.5
        controller.last_command_time = 0
        controller.command(np.full(541, 8.0), 2.0, 1, yaw_rate=0.0, fused_speed=2.2)
        return controller.binding_cap

    # oracle: closed form, as the node test: 2.125 m/s (wheel ramp) is below
    # traction's own 2.325 m/s (fused ramp); with traction off nothing binds
    # there but the ordinary acceleration ramp.
    assert binding(True) == 'traction_off'
    assert binding(False) == 'accel'
