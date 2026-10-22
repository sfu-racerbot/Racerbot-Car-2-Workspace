"""seed_start_pose: the particle filter's start seed for a saved-map race.

Needs ROS sourced and an isolated domain (CLAUDE.md):

    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/pure_pursuit/test/test_seed_start_pose.py -v

Oracle for the pose: the filter's particles are LiDAR poses, so a car whose
rear axle (base_link) is at (x, y, yaw) is seeded at the LiDAR, laser_offset_x
ahead along yaw -- closed form below.
"""
import math
import os
import sys
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pure_pursuit.seed_start_pose import SeedStartPose, seed_message_fields  # noqa: E402


@pytest.mark.parametrize('x,y,yaw', [(0.0, 0.0, 0.0), (1.0, -2.0, math.pi / 2),
                                     (0.5, 0.5, -2.5)])
def test_seed_is_the_lidar_pose_of_the_given_base_link_pose(x, y, yaw):
    lx, ly, qz, qw = seed_message_fields(x, y, yaw, 0.26, 0.0)
    assert lx == pytest.approx(x + 0.26 * math.cos(yaw), abs=1e-12)
    assert ly == pytest.approx(y + 0.26 * math.sin(yaw), abs=1e-12)
    assert 2.0 * math.atan2(qz, qw) == pytest.approx(yaw, abs=1e-12)


def test_origin_seed_is_ahead_of_the_rear_axle():
    assert seed_message_fields(0.0, 0.0, 0.0, 0.26, 0.0) == pytest.approx(
        (0.26, 0.0, 0.0, 1.0), abs=1e-12)


def _spin_until(nodes, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.02)
    return predicate()


def _pose(x, y):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


ARGS = ['--ros-args', '-p', 'repeats:=3', '-p', 'period_sec:=0.05',
        '-p', 'initialpose_topic:=/test_only/initialpose',
        '-p', 'pose_topic:=/test_only/pf_pose', '-p', 'confirm_radius:=1.0']


def test_seeds_exactly_repeats_times_then_waits_for_the_filter_to_take_it():
    rclpy.init(args=ARGS)
    seeder = SeedStartPose()
    helper = rclpy.create_node('fake_particle_filter')
    got = []
    pose_pub = helper.create_publisher(PoseStamped, '/test_only/pf_pose', 10)
    try:
        # Nobody listening yet: nothing is sent.
        _spin_until([seeder], lambda: False, timeout=0.3)
        assert seeder.remaining == 3 and not seeder.done
        # A pose right on the seed BEFORE seeding is the filter's own guess,
        # not confirmation.
        pose_pub.publish(_pose(0.26, 0.0))
        _spin_until([seeder, helper], lambda: False, timeout=0.2)
        assert not seeder.done
        helper.create_subscription(
            PoseWithCovarianceStamped, '/test_only/initialpose', got.append, 10)
        assert _spin_until([seeder, helper], lambda: seeder.seeded)
        _spin_until([helper], lambda: False, timeout=0.3)
        assert len(got) == 3
        for msg in got:
            assert msg.header.frame_id == 'map'
            assert msg.pose.pose.position.x == pytest.approx(0.26, abs=1e-12)
            assert msg.pose.pose.position.y == pytest.approx(0.0, abs=1e-12)
        # Seeded but the filter reports somewhere else (1.01 m off): not done.
        pose_pub.publish(_pose(0.26 + 1.01, 0.0))
        _spin_until([seeder, helper], lambda: False, timeout=0.3)
        assert not seeder.done
        # Within confirm_radius (1.0 m): done, and a success.
        pose_pub.publish(_pose(0.26 + 0.99, 0.0))
        assert _spin_until([seeder, helper], lambda: seeder.done)
        assert seeder.succeeded
    finally:
        helper.destroy_node()
        seeder.destroy_node()
        rclpy.shutdown()


def test_gives_up_as_a_failure_when_no_filter_ever_listens():
    rclpy.init(args=['--ros-args', '-p', 'give_up_sec:=0.2', '-p', 'period_sec:=0.05',
                     '-p', 'initialpose_topic:=/test_only/initialpose_nobody'])
    seeder = SeedStartPose()
    try:
        assert _spin_until([seeder], lambda: seeder.done, timeout=3.0)
        assert seeder.remaining == 3       # never sent anything
        assert not seeder.succeeded
    finally:
        seeder.destroy_node()
        rclpy.shutdown()


def test_gives_up_as_a_failure_when_the_filter_never_takes_the_seed():
    rclpy.init(args=ARGS + ['-p', 'give_up_sec:=1.0'])
    seeder = SeedStartPose()
    helper = rclpy.create_node('fake_particle_filter')
    helper.create_subscription(PoseWithCovarianceStamped, '/test_only/initialpose',
                               lambda _m: None, 10)
    pose_pub = helper.create_publisher(PoseStamped, '/test_only/pf_pose', 10)
    try:
        assert _spin_until([seeder, helper], lambda: seeder.seeded)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not seeder.done:
            pose_pub.publish(_pose(5.0, 5.0))     # always far from the seed
            _spin_until([seeder, helper], lambda: False, timeout=0.05)
        assert seeder.done and not seeder.succeeded
    finally:
        helper.destroy_node()
        seeder.destroy_node()
        rclpy.shutdown()


def test_process_exit_code_is_what_race_launch_gates_on():
    # race_launch.py starts pure_pursuit only on exit code 0: a seeder that
    # gives up must exit non-zero.
    import subprocess
    done = subprocess.run(
        [sys.executable, '-c', 'from pure_pursuit.seed_start_pose import main; main()',
         '--ros-args', '-p', 'give_up_sec:=0.3', '-p', 'period_sec:=0.05',
         '-p', 'initialpose_topic:=/test_only/initialpose_exit'],
        capture_output=True, text=True, timeout=30,
        cwd=os.path.join(os.path.dirname(__file__), '..'))
    assert done.returncode == 1, done.stderr
