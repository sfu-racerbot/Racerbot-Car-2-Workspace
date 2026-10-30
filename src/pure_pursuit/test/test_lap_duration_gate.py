"""Duration floor must fit the measured short course and physical turn rate."""
import math
from pathlib import Path

import yaml

from pure_pursuit.auto_map_race_node import AutoMapRaceNode


def test_duration_gate_fits_short_course_and_fastest_turn():
    src = Path(__file__).resolve().parents[2]
    params = yaml.safe_load((src / 'pure_pursuit/config/auto_map_race.yaml').read_text())[
        'auto_map_race_node']['ros__parameters']
    speed = yaml.safe_load((src / 'gap_follow/config/gap_follow.yaml').read_text())[
        'gap_follow_node']['ros__parameters']['max_speed']
    duration = params['minimum_lap_duration_sec']
    # oracle: spec docs/hardware-reference.md: autonomy radius is 1.05 m.
    # Recorded run 20261026-194913: shortest mapped course is 28.7 m.
    fastest_turn_sec = params['minimum_lap_turn_deg'] / math.degrees(speed / 1.05)
    assert duration >= 0
    assert fastest_turn_sec < duration < 28.7 / speed


def test_declared_duration_default_matches_yaml():
    import ast
    import inspect

    params = yaml.safe_load((Path(__file__).resolve().parents[1]
                             / 'config/auto_map_race.yaml').read_text())[
        'auto_map_race_node']['ros__parameters']
    tree = ast.parse(inspect.getsource(AutoMapRaceNode))
    declarations = [call for call in ast.walk(tree)
                    if isinstance(call, ast.Call)
                    and isinstance(call.func, ast.Attribute)
                    and call.func.attr == 'declare_parameter'
                    and call.args and isinstance(call.args[0], ast.Constant)
                    and call.args[0].value == 'minimum_lap_duration_sec']
    # oracle: invariant, launching without YAML must use the same floor.
    assert [ast.literal_eval(call.args[1]) for call in declarations] == [
        params['minimum_lap_duration_sec']]


# --- The gate scales with the course once a lap has closed. ---------------
# A car circles a 4 m-radius course (25.1 m) at a fixed period per lap. A lap
# closes on its first return to the start (about one period) unless the
# elapsed-time gate refuses it, in which case it closes one circuit later.
# The oracle is which circuit closed it: < 1.5 periods = first, else second.
import pytest  # noqa: E402

from pure_pursuit.auto_map_race_node import LapRecorder  # noqa: E402

RADIUS_M = 4.0
DT_SEC = 0.02


def _recorder(floor=4.0, fraction=0.5):
    return LapRecorder(spacing=0.15, min_distance=1.0, departure_distance=2.0,
                       closure_distance=0.5, closure_heading_rad=math.radians(30.0),
                       min_duration_sec=floor, min_turn_rad=math.radians(300.0),
                       min_duration_fraction=fraction)


def _circuits_per_lap(periods, floor=4.0, fraction=0.5):
    """For each lap period, how many circuits the recorder needed to close."""
    recorder = _recorder(floor, fraction)
    t, theta, circuits = 0.0, 0.0, []
    for period in periods:
        omega = 2.0 * math.pi / period
        lap_start = t
        while True:
            t += DT_SEC
            theta += omega * DT_SEC
            yaw = math.atan2(math.sin(theta + math.pi / 2), math.cos(theta + math.pi / 2))
            if recorder.update(RADIUS_M * math.cos(theta), RADIUS_M * math.sin(theta),
                               yaw, t, odom_yaw=yaw):
                circuits.append(1 if t - lap_start < 1.5 * period else 2)
                recorder.reset()
                break
            assert t - lap_start < 3.0 * period, 'lap never closed'
    return circuits


def test_first_lap_uses_the_fixed_floor():
    # oracle: closed form, no lap has closed, so the gate is the 4.0 s floor:
    # a 3.0 s circuit is refused once, a 5.0 s circuit closes first time.
    assert _circuits_per_lap([3.0]) == [2]
    assert _circuits_per_lap([5.0]) == [1]
    assert _recorder().required_duration_sec == 4.0


def test_later_laps_need_half_the_previous_lap():
    # oracle: closed form, lap 1 takes ~9.8 s, so lap 2 needs >= ~4.9 s:
    # 0.55 x 10 s closes on its first return, 0.45 x 10 s (above the 4 s
    # floor, so only the fraction can refuse it) needs a second circuit.
    assert _circuits_per_lap([10.0, 5.5]) == [1, 1]
    assert _circuits_per_lap([10.0, 4.5]) == [1, 2]


def test_the_floor_governs_when_half_the_previous_lap_is_shorter():
    # oracle: closed form, half of ~5.9 s is below 4 s, so the floor rules:
    # a 4.5 s circuit closes first time, a 3.5 s one (above half the
    # previous lap, below the floor) only the floor can refuse.
    assert _circuits_per_lap([6.0, 4.5]) == [1, 1]
    assert _circuits_per_lap([6.0, 3.5]) == [1, 2]


def test_each_lap_is_judged_against_the_one_just_before():
    # oracle: closed form, lap 2 takes ~19.8 s, so lap 3 needs ~9.9 s and a
    # 9 s circuit is refused; judged against lap 1 (~9.8 s) it would pass.
    assert _circuits_per_lap([10.0, 20.0, 9.0]) == [1, 1, 2]


def test_fraction_zero_is_the_floor_alone():
    # oracle: spec, the parameter comment: 0 = the floor alone.
    assert _circuits_per_lap([10.0, 4.5], fraction=0.0) == [1, 1]


def test_previous_lap_survives_reset():
    # oracle: invariant, reset() starts the next lap; the course stays known.
    recorder = _recorder()
    recorder.previous_lap_duration_sec = 12.0
    recorder.reset()
    assert recorder.required_duration_sec == 6.0


@pytest.mark.parametrize('fraction', [-0.1, 1.5, math.nan, math.inf])
def test_invalid_fraction_refused_at_startup(fraction):
    # oracle: spec, the parameter comment: a fraction in 0..1.
    with pytest.raises(ValueError, match='min_duration_fraction'):
        _recorder(fraction=fraction)


def test_declared_fraction_default_matches_yaml():
    import ast
    import inspect

    params = yaml.safe_load((Path(__file__).resolve().parents[1]
                             / 'config/auto_map_race.yaml').read_text())[
        'auto_map_race_node']['ros__parameters']
    tree = ast.parse(inspect.getsource(AutoMapRaceNode))
    declarations = [call for call in ast.walk(tree)
                    if isinstance(call, ast.Call)
                    and isinstance(call.func, ast.Attribute)
                    and call.func.attr == 'declare_parameter'
                    and call.args and isinstance(call.args[0], ast.Constant)
                    and call.args[0].value == 'minimum_lap_duration_fraction']
    # oracle: invariant, launching without YAML must use the same fraction.
    assert [ast.literal_eval(call.args[1]) for call in declarations] == [
        params['minimum_lap_duration_fraction']]
