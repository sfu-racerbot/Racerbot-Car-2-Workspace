"""gap_mirror's per-tick telemetry, which tools/run_analysis/weave.py reads.

The parity test (src/gap_follow/test/test_gap_mirror_parity.py) compares commands,
not telemetry, so the telemetry's own contract is pinned here:

- requested_steering is the steering the tick returns, after the slew limit, on
  driving ticks and on stop ticks alike;
- target_angle is present on a driving tick and absent on a stop tick, which is
  how weave.target_jump_share tells the two apart.

Unsourced:

    python3 -m pytest tools/f1tenth_sim/tests/test_gap_mirror_telemetry.py -v -p no:cacheprovider
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'src', 'gap_follow'))
import gap_mirror  # noqa: E402


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


def test_requested_steering_is_the_steering_the_tick_returns_after_the_slew():
    # oracle: the invariant weave relies on. The rack starts hard right (-0.3 rad)
    # and the corridor's gap is to the left, so the slew limit holds the command
    # away from the desired value. That makes the test discriminating: a
    # requested_steering that copied desired_steering would fail the equality.
    ctrl = gap_mirror.GapController(gap_mirror.node_params(), _Lidar())
    ctrl.steering_basis = -0.3
    steering, _speed = ctrl.command(_corridor(left_wall=1.4, right_wall=0.6), 1.0, 0)
    tel = ctrl.telemetry
    assert not ctrl.stopped
    assert abs(tel['desired_steering'] - steering) > 0.01   # the slew is active here
    assert tel['requested_steering'] == steering            # exact: the value returned
    assert math.isfinite(tel['target_angle'])               # a driving tick has an aim point


def test_stop_ticks_report_the_held_steering_and_no_target():
    # oracle: a stop returns the held steering with zero speed (gap_mirror._stop).
    # With no returns at all there is no safe gap, so the tick stops. Its telemetry
    # reports the held steering as requested_steering, and carries no target_angle,
    # which is how weave.target_jump_share skips it.
    ctrl = gap_mirror.GapController(gap_mirror.node_params(), _Lidar())
    ctrl.steering_basis = 0.15
    steering, speed = ctrl.command(np.full(1081, np.nan), 1.0, 0)
    assert ctrl.stopped and ctrl.stop_reason == 'no_safe_gap'
    assert speed == 0.0
    assert steering == 0.15
    assert ctrl.telemetry['requested_steering'] == 0.15
    assert 'target_angle' not in ctrl.telemetry
