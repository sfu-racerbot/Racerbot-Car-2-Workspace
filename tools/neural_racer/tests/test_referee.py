"""The referee: races on the calibrated CPU simulator (FidelityPlant, 'car' profile).

Runner: PYTHONPATH=<torch> python3 -m pytest tools/neural_racer/tests/test_referee.py -v
(needs the pinned gym from tools/f1tenth_sim/setup.sh; ~1 minute on the Jetson).

Oracles: FidelityPlant's own chassis contact geometry (never gym's flag), and
ground-truth progress along the track's own line.
"""
import math

import conftest  # noqa: F401
import pytest

from neural_racer_train import fidelity, referee

pytestmark = pytest.mark.skipif(not fidelity.gym_available(), reason='needs the pinned gym: tools/f1tenth_sim/setup.sh')


class Straight:
    """Drives dead straight at a fixed speed: on Spielberg it meets a barrier."""
    def __init__(self, speed):
        self.speed = speed

    def command(self, agent_obs, truth, step):
        return 0.0, self.speed


def test_real_gap_follow_makes_progress_without_contact():
    r = referee.race('Spielberg', {'gap': referee.gap_factory()}, seed=0, laps=1, timeout_s=25)
    assert r.progress[0] > 10.0 and r.contact == [False]


def test_wall_contact_is_flagged_and_at_fault():
    """Measured in tools/f1tenth_sim/README.md: straight at 4 m/s from the
    Spielberg start meets the first barrier at ~35.5 m."""
    r = referee.race('Spielberg', {'straight': lambda plant: Straight(4.0)}, seed=0, laps=1, timeout_s=20)
    assert r.contact == [True] and r.at_fault == [True]
    assert 20.0 < r.progress[0] < 45.0


def test_stopped_car_is_never_in_contact():
    r = referee.race('Spielberg', {'still': lambda plant: Straight(0.0)}, seed=0, laps=1, timeout_s=3)
    assert r.contact == [False] and abs(r.progress[0]) < 0.2


def test_order_by_progress_when_nobody_finishes():
    r = referee.race('Spielberg', {'slow': lambda plant: Straight(0.0), 'gap': referee.gap_factory()},
                     seed=0, laps=1, timeout_s=10)
    # 'gap' starts 1 m behind on the grid and overtakes a parked car only if it
    # gets past; either way the order must follow measured progress
    assert r.order == sorted(r.names, key=lambda n: -r.progress[r.names.index(n)])


def test_grid_overlap_refused(monkeypatch):
    monkeypatch.setattr(referee, 'GRID_STAGGER', 0.1)
    with pytest.raises(RuntimeError, match='overlap at the grid start'):
        referee.race('Spielberg', {'a': lambda p: Straight(0.0), 'b': lambda p: Straight(0.0)}, seed=0, timeout_s=1)
