"""Coverage for the F1TENTH Gym bridge that does not need the Gym itself.

The pieces here -- the scripted opponent and the dead-reckoned odometry --
are pure geometry and integration, so they are testable without a physics
step. The Gym-backed parts are exercised by
`tools/racerbot_sim/run_auto_map_validation.py`.

    python3 -m pytest src/racerbot_sim/test/test_sim_bridge.py -v
"""

import math
import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_sim import sim_bridge, tracks  # noqa: E402


@pytest.fixture
def loop():
    return tracks.rounded_rectangle_centerline(half_x=5.0, half_y=3.2, radius=1.8)


# --- scripted opponents ---------------------------------------------------

def test_opponent_starts_where_it_was_asked_to(loop):
    follower = sim_bridge.CenterlineFollower(
        loop, sim_bridge.OpponentPlan(start_offset_m=0.0, speed=1.0))
    pose = follower.start_pose()
    assert pose[0] == pytest.approx(0.0, abs=0.05)
    assert pose[1] == pytest.approx(-3.2, abs=0.05)
    assert pose[2] == pytest.approx(0.0, abs=0.05)


def test_opponent_start_offset_moves_it_along_the_loop(loop):
    near = sim_bridge.CenterlineFollower(
        loop, sim_bridge.OpponentPlan(start_offset_m=0.0)).start_pose()
    far = sim_bridge.CenterlineFollower(
        loop, sim_bridge.OpponentPlan(start_offset_m=8.0)).start_pose()
    assert np.hypot(*(far[:2] - near[:2])) > 3.0


def test_a_parked_opponent_commands_nothing(loop):
    """Speed 0 is a static obstacle on the line -- the case that used to
    end a race outright, because pure pursuit's hard stop had no way out."""
    follower = sim_bridge.CenterlineFollower(
        loop, sim_bridge.OpponentPlan(start_offset_m=4.0, speed=0.0))
    assert follower.command(0.0, -3.2, 0.0) == (0.0, 0.0)


def test_opponent_steers_toward_the_loop_from_beside_it(loop):
    follower = sim_bridge.CenterlineFollower(
        loop, sim_bridge.OpponentPlan(start_offset_m=0.0, speed=1.0))
    # Sitting 0.4m to the right of the bottom straight, facing along it.
    steering, speed = follower.command(0.0, -3.6, 0.0)
    assert speed == pytest.approx(1.0)
    assert steering > 0.0, 'the loop is to the left, so steer left'


def test_opponent_steering_stays_inside_a_plausible_rack(loop):
    follower = sim_bridge.CenterlineFollower(
        loop, sim_bridge.OpponentPlan(start_offset_m=0.0, speed=1.0))
    for x, y, yaw in [(0.0, -3.2, math.pi / 2), (3.0, 3.0, -1.0),
                      (-5.0, 0.0, 2.5), (0.0, 0.0, 0.0)]:
        steering, _ = follower.command(x, y, yaw)
        assert abs(steering) <= 0.4


def test_lateral_offset_moves_the_opponents_line_sideways(loop):
    centred = sim_bridge.CenterlineFollower(
        loop, sim_bridge.OpponentPlan(start_offset_m=0.0)).start_pose()
    offset = sim_bridge.CenterlineFollower(
        loop, sim_bridge.OpponentPlan(start_offset_m=0.0,
                                      lateral_offset_m=0.4)).start_pose()
    assert offset[1] - centred[1] == pytest.approx(0.4, abs=0.05)


# --- dead-reckoned odometry ----------------------------------------------

def _odometry(**overrides):
    config = sim_bridge.SimConfig(track_path='', centerline=np.zeros((4, 2)),
                                  odom_speed_noise_std=0.0, **overrides)
    return sim_bridge.DeadReckonedOdometry(0.0, 0.0, 0.0, config,
                                           np.random.default_rng(0))


def test_odometry_integrates_a_straight_run():
    odom = _odometry()
    for _ in range(40):
        odom.update(1.0, 0.0, 0.025)
    assert odom.x == pytest.approx(1.0, abs=1e-6)
    assert odom.y == pytest.approx(0.0, abs=1e-9)
    assert odom.yaw == pytest.approx(0.0, abs=1e-9)


def test_odometry_turns_from_the_commanded_steering_angle():
    """vesc_to_odom runs with use_servo_cmd_to_calc_angular_velocity, so the
    yaw rate comes from the servo command and the wheel speed -- never from
    a gyro. The drift SLAM has to absorb is that shape, and a bridge that
    published ground truth here would hide every mapping problem."""
    odom = _odometry()
    for _ in range(40):
        odom.update(1.0, 0.2, 0.025)
    expected = math.tan(0.2) / sim_bridge.WHEELBASE   # rad/s at 1 m/s
    assert odom.yaw == pytest.approx(expected, rel=0.02)


def test_odometry_scale_error_shows_up_as_distance_error():
    odom = _odometry(odom_speed_scale=1.1)
    for _ in range(40):
        odom.update(1.0, 0.0, 0.025)
    assert odom.x == pytest.approx(1.1, abs=1e-6)


def test_odometry_reports_what_it_integrated():
    odom = _odometry()
    odom.update(2.0, 0.1, 0.025)
    assert odom.speed == pytest.approx(2.0)
    assert odom.yaw_rate == pytest.approx(2.0 * math.tan(0.1) / sim_bridge.WHEELBASE)


def test_odometry_yaw_stays_wrapped():
    odom = _odometry()
    for _ in range(400):
        odom.update(2.0, 0.26, 0.025)
    assert -math.pi <= odom.yaw <= math.pi


# --- geometry the bridge shares with the car ------------------------------

def test_lidar_geometry_matches_the_real_sensor():
    """Hokuyo UST-10LX: 1081 beams at 0.25deg over 270deg
    (docs/hardware-reference.md). The older controller-math harness uses
    819; docs/sim-fidelity-audit.md measured the correction as free."""
    assert sim_bridge.LIDAR_BEAMS == 1081
    assert math.degrees(2.0 * sim_bridge.LIDAR_HALF_FOV) == pytest.approx(270.0)
    increment = 2.0 * sim_bridge.LIDAR_HALF_FOV / (sim_bridge.LIDAR_BEAMS - 1)
    assert math.degrees(increment) == pytest.approx(0.25, abs=0.001)


def _car_params(package, node):
    """The vehicle geometry a real driving node is actually launched with."""
    path = os.path.join(
        os.path.dirname(__file__), '..', '..', package, 'config', f'{package}.yaml')
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)[node]['ros__parameters']


@pytest.mark.parametrize('package,node,fields', [
    ('gap_follow', 'gap_follow_node',
     ('wheelbase', 'car_width', 'car_length', 'laser_offset_x')),
    # pure_pursuit.yaml declares laser_offset_x under the same name but in a
    # different section; car_length/car_width/wheelbase are all there too.
    ('pure_pursuit', 'pure_pursuit_node',
     ('wheelbase', 'car_width', 'car_length', 'laser_offset_x')),
])
def test_vehicle_geometry_matches_the_car_configs(package, node, fields):
    """The simulator must fly the car the real code is aimed at.

    Read out of the YAML each node is launched with rather than restated
    here. The previous version of this test carried its own copy of the
    four numbers, so "the simulator agrees with the car" held only until
    someone edited one file and not the other -- which is exactly what
    re-measuring the car does.
    """
    params = _car_params(package, node)
    sim = {
        'wheelbase': sim_bridge.WHEELBASE,
        'car_width': sim_bridge.CAR_WIDTH,
        'car_length': sim_bridge.CAR_LENGTH,
        'laser_offset_x': sim_bridge.LIDAR_OFFSET_X,
    }
    missing = [f for f in fields if f not in params]
    assert not missing, f'{package}.yaml no longer declares {missing}'
    for field in fields:
        assert sim[field] == params[field], (
            f'{package}.yaml {field}={params[field]} but the simulator uses '
            f'{sim[field]}')
