"""Tests that the run_validation harness actually routes steering through the
pluggable law (spec 01), not a hidden copy of pure pursuit.

Pure Python where possible; the FidelityPlant pieces need the gym checkout
(.sim/). Run unsourced:

    python3 -m pytest tools/f1tenth_sim/tests/ -v
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import law_compare  # noqa: E402  (the verdict rule; see test_law_compare.py)


def test_law_param_parsing_rejects_unknown_keys():
    import run_validation as rv
    with pytest.raises(SystemExit):
        rv._parse_law_params(['not_a_real_param.gain=1.0'])


def test_law_param_parsing_accepts_dotted_law_keys():
    import run_validation as rv
    params = rv._parse_law_params(['stanley.gain=1.2', 'lqr.q_lateral=3.0',
                                   'mpc.horizon_steps=25'])
    assert params['stanley.gain'] == pytest.approx(1.2)
    assert params['lqr.q_lateral'] == pytest.approx(3.0)
    assert params['mpc.horizon_steps'] == pytest.approx(25.0)


def test_path_follower_uses_the_given_law():
    """A PathFollower built for stanley must produce Stanley's output, not
    pure pursuit's, on the same state. Compared against a directly-built
    Stanley law (the independent computation)."""
    import run_validation as rv
    from pure_pursuit import racing_math, tracking_laws

    # A gentle closed loop.
    n = 80
    ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
    xy = np.column_stack([3.0 * np.cos(ang), 3.0 * np.sin(ang)])
    plan = rv.PathPlan(xy=xy,
                       seg_len=racing_math.compute_segment_lengths(xy, closed=True),
                       speed=np.full(n, 1.5))
    follower = rv.PathFollower(plan, law_name='stanley',
                               law_params={'stanley.gain': 1.6,
                                           'stanley.softening': 0.4})
    # A state slightly off the loop, moving.
    state = np.array([3.2, 0.1, 0.0, 1.5, math.pi / 2.0])
    steering, _, nearest, _, _ = follower.command(state, previous_steering=0.0)
    direct = tracking_laws.make_law('stanley', rv.PathFollower.VEHICLE_LIMITS,
                                    {'stanley.gain': 1.6, 'stanley.softening': 0.4})
    expected = direct.steer(tracking_laws.TrackingInput(
        x=3.2, y=0.1, yaw=math.pi / 2.0, speed=1.5, nearest_idx=nearest,
        previous_steering=0.0), follower.path_context)
    assert steering == pytest.approx(expected.steering, abs=1e-9)


def test_path_follower_law_failure_commands_zero():
    import run_validation as rv
    from pure_pursuit import racing_math, tracking_laws

    n = 40
    ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
    xy = np.column_stack([3.0 * np.cos(ang), 3.0 * np.sin(ang)])
    plan = rv.PathPlan(xy=xy,
                       seg_len=racing_math.compute_segment_lengths(xy, closed=True),
                       speed=np.full(n, 1.5))

    class ExplodingLaw:
        name = 'pure_pursuit'

        def steer(self, inp, path):
            raise tracking_laws.TrackingLawError('boom')

        def reset(self):
            pass

    follower = rv.PathFollower(plan, law=ExplodingLaw())
    steering, speed, _, _, _ = follower.command(
        np.array([3.2, 0.1, 0.0, 1.5, math.pi / 2.0]))
    assert steering == 0.0
    assert speed == 0.0
    assert follower.law_fail_steps == 1
