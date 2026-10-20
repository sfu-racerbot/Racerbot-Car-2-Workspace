"""Gym's own collision response must not move a car (sim_fidelity/plant.py,
FidelityPlant docstring).

When gym's time-to-collision check fires it zeroes ``state[3:]`` -- speed,
steering, yaw, yaw rate, slip -- so the car is rotated in place to heading
0. On 2026-10-20 that turned two gap_follow cars round on indoor_wide and
they lapped backwards. The fidelity profiles make their own contact
verdicts and switch that response off. Needs the gym checkout (.sim/); run
unsourced:

    python3 -m pytest tools/f1tenth_sim/tests/test_gym_collision_response.py -v
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import eval_gap_multi  # noqa: E402  (bootstraps gym)


def _heading_error(yaw, target):
    return abs((yaw - target + math.pi) % (2.0 * math.pi) - math.pi)


def test_closing_on_a_parked_car_keeps_the_heading():
    # Two cars on indoor_wide's bottom straight, both facing -x (heading pi,
    # so a zeroed yaw is unmistakable). The rear one drives at the parked
    # one. Oracle: the invariant that nothing but the steering may change a
    # car's heading -- with zero steering commanded it must stay pi until
    # our own real-chassis check reports contact.
    plant = eval_gap_multi.make_plant('indoor_wide', 12345, 2)
    try:
        poses = np.array([[1.4, -4.5, math.pi], [0.6, -4.5, math.pi]])
        plant.reset(poses)
        closed_in = False
        for _ in range(200):
            plant.step([(0.0, 1.5), (0.0, 0.0)])
            rear, front = plant.truth(0), plant.truth(1)
            gap = math.hypot(rear[0] - front[0], rear[1] - front[1])
            if plant._chassis_contact():
                break
            closed_in |= gap < 0.58          # padded envelopes (car_length) overlap
            # 0.05 rad: the steering servo and grip model wobble far less
            # than this; a zeroed yaw is off by pi.
            assert _heading_error(float(rear[4]), math.pi) < 0.05, f'rear car turned at gap {gap:.2f} m'
            assert _heading_error(float(front[4]), math.pi) < 0.05
        assert closed_in, 'precondition: the cars must get inside gym\'s padded envelope'
    finally:
        plant.close()
