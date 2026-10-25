"""imu_report: gyros vs odometry vs LiDAR. The analyses run without ROS:

    python3 -m pytest tools/run_analysis/tests -q

Oracles: synthetic runs with the answer built in -- a known steering lag
and turning gain (car 2's measured 0.15 s and 0.95, docs/run-analysis.md),
a known odometry heading over-read -- and closed forms for each statistic.
The bag round trip at the end needs ROS sourced (rosbag2_py) and is skipped,
saying so, without it.
"""
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from run_analysis import actuation, imu_report  # noqa: E402

L = 0.36


def _odom(t, v, wz=None, yaw=None):
    """runlog odom rows (t, vx, wz, x, y, qz, qw), integrated from v and yaw."""
    t = np.asarray(t, float)
    v = np.broadcast_to(np.asarray(v, float), t.shape)
    wz = np.zeros_like(t) if wz is None else np.asarray(wz, float)
    yaw = np.zeros_like(t) if yaw is None else np.asarray(yaw, float)
    dt = np.diff(t, prepend=t[0])
    x = np.cumsum(v * np.cos(yaw) * dt)
    y = np.cumsum(v * np.sin(yaw) * dt)
    return np.column_stack([t, v, wz, x, y, np.sin(yaw / 2), np.cos(yaw / 2)])


# ---------------------------------------------------------------- turn_rate_table

def test_turn_rate_table_scaled_source():
    # oracle: closed form -- other = g * ref: RMS = |g - 1| rms(ref),
    # mean = (g - 1) mean(ref), least-squares gain = g.
    t = np.arange(0, 10, 0.004)
    ref = 0.5 + np.sin(t)
    g = 0.95
    rows = imu_report.turn_rate_table(t, ref, {'x': (t, g * ref)}, _odom(t, 1.0))
    name, n, rms, mean, gain = rows[0]
    assert n == len(t)
    assert rms == pytest.approx(abs(g - 1) * np.sqrt(np.mean(ref ** 2)), abs=1e-9)  # rad/s
    assert mean == pytest.approx((g - 1) * np.mean(ref), abs=1e-9)  # rad/s
    assert gain == pytest.approx(g, abs=1e-9)


def test_turn_rate_table_ignores_stopped_samples_and_non_overlap():
    t = np.arange(0, 10, 0.01)
    speed = np.where((t > 4) & (t < 6), 0.0, 1.0)      # stopped for 2 s
    odom = _odom(t, speed)
    rows = imu_report.turn_rate_table(t, np.ones_like(t), {
        'same': (t, np.ones_like(t)),
        'later': (t + 100, np.ones_like(t)),
        'empty': (np.array([]), np.array([]))}, odom)
    by = {r[0]: r for r in rows}
    # oracle: closed form -- 2 s of 10 at 100 Hz excluded (boundary samples
    # at exactly 4 and 6 are moving).
    assert by['same'][1] == int(np.sum(speed >= imu_report.MIN_SPEED))
    assert by['same'][2] == pytest.approx(0.0, abs=1e-12)
    assert by['later'][1] == 0 and math.isnan(by['later'][2])
    assert by['empty'][1] == 0


# ---------------------------------------------------------------- steering fit

def test_gyro_finds_the_built_in_steering_lag_and_turning_gain():
    # oracle: constructed -- the car turns at 0.95 * v tan(cmd(t - 0.15)) / L,
    # measured by a perfect gyro. The fit must recover 0.15 s (on its 25 ms
    # grid) and 0.95.
    lag, gain, v = 0.15, 0.95, 1.2
    t = np.arange(0, 30, 0.004)
    cmd_t = np.arange(0, 30, 0.025)
    cmd = 0.25 * np.sin(2 * np.pi * 0.3 * cmd_t) + 0.1 * np.sin(2 * np.pi * 0.7 * cmd_t)
    w = gain * v * np.tan(np.interp(t - lag, cmd_t, cmd)) / L
    drive = np.column_stack([cmd_t, cmd_t, cmd, np.full_like(cmd_t, v)])
    odom = _odom(cmd_t, v)
    rep = actuation.analyze(imu_report.gyro_motion(t, w, odom), drive, odom, 1.0, 29.0,
                            min_speed=imu_report.FIT_MIN_SPEED)
    assert rep.steering_lag_s == pytest.approx(lag, abs=1e-9)  # s
    assert rep.curvature_gain == pytest.approx(gain, abs=0.01)


def test_gyro_motion_carries_odom_speed_and_gyro_rate():
    t = np.array([0.5, 1.5])
    odom = _odom(np.array([0.0, 1.0, 2.0]), np.array([0.0, 1.0, 2.0]))
    m = imu_report.gyro_motion(t, np.array([0.1, -0.2]), odom)
    # oracle: closed form -- linear interpolation of odom speed 0 -> 1 -> 2.
    assert m.v == pytest.approx([0.5, 1.5], abs=1e-12)
    assert list(m.w) == [0.1, -0.2]


# ---------------------------------------------------------------- heading corrections

def test_heading_correction_of_an_over_reading_odometry():
    # oracle: closed form -- truth (SLAM) turns at r; /odom says 1.1 r (it
    # over-reads by 10%); /odom/imu says r. SLAM's correction on /odom grows
    # to -0.1 r T, on /odom/imu stays 0. r T > pi, so unwrapping matters.
    r, v, T = 0.5, 1.0, 20.0
    t = np.arange(0, T + 1e-9, 0.02)
    truth = r * t
    wrap = lambda a: np.arctan2(np.sin(a), np.cos(a))  # noqa: E731
    slam = np.column_stack([t, np.zeros_like(t), np.zeros_like(t), wrap(truth)])
    odom = _odom(t, v, yaw=wrap(1.1 * truth))
    rows = {r_[0]: r_ for r_ in imu_report.heading_corrections(
        slam, {'/odom': (t, wrap(1.1 * truth)), '/odom/imu': (t, wrap(truth))}, odom)}
    name, final, peak, per100 = rows['/odom']
    assert final == pytest.approx(-0.1 * r * T, abs=1e-6)  # rad
    assert peak == pytest.approx(0.1 * r * T, abs=1e-6)  # rad
    # per 100 m of /odom travel: v T metres
    assert per100 == pytest.approx(0.1 * r * T / (v * T) * 100, rel=1e-3)
    assert rows['/odom/imu'][1] == pytest.approx(0.0, abs=1e-9)
    assert rows['/odom/imu'][2] == pytest.approx(0.0, abs=1e-9)


def test_heading_correction_needs_a_reference_and_overlap():
    t = np.arange(0, 5, 0.1)
    odom = _odom(t, 1.0)
    assert imu_report.heading_corrections(np.zeros((1, 4)), {'/odom': (t, t)}, odom) == []
    slam = np.column_stack([t + 100, t * 0, t * 0, t * 0])
    assert imu_report.heading_corrections(slam, {'/odom': (t, t)}, odom) == []


def test_odom_yaw_from_planar_quaternion():
    yaw = np.array([0.0, 0.5, -2.0, 3.0])
    odom = _odom(np.arange(4.0), 1.0, yaw=yaw)
    # oracle: closed form -- q = (0, 0, sin(y/2), cos(y/2))
    assert imu_report.odom_yaw(odom) == pytest.approx(yaw, abs=1e-12)


# ---------------------------------------------------------------- report

def _fake_run(t, v=1.2, rate=0.4):
    odom = _odom(t, v, wz=np.full_like(t, rate * 1.05), yaw=rate * 1.05 * t)
    cmd_t = t[::5]
    drive = np.column_stack([cmd_t, cmd_t, 0.2 * np.sin(cmd_t), np.full_like(cmd_t, v)])
    return SimpleNamespace(odom=odom, drive=drive)


def test_report_without_a_gyro_says_so():
    run = _fake_run(np.arange(0, 5, 0.02))
    lines = imu_report.build_report(run, {'bno': np.zeros((0, 2)), 'vesc': np.zeros((0, 2))})
    assert lines == ['no gyro in this bag (/imu/bno/data, /imu/vesc/data): was the IMU stack up?']


def test_report_sections_and_numbers():
    t = np.arange(0, 20, 0.02)
    run = _fake_run(t)
    gyro = np.column_stack([t, np.full_like(t, 0.4)])
    oi_yaw = 0.4 * t
    imu = {'bno': gyro, 'vesc': gyro.copy(),
           'odom_imu': np.column_stack([t, t, t, oi_yaw]),
           'slam': np.column_stack([t, t, t, 0.4 * t]), 'slam_pose': np.zeros((0, 4))}
    text = '\n'.join(imu_report.build_report(run, imu))
    assert 'Turn rate vs the bno gyro' in text
    # odom says 1.05 x 0.4 = 0.42: mean +0.020, gain 1.050
    assert 'odom (steering model)' in text and 'mean +0.020' in text and 'gain 1.050' in text
    assert 'Steering response measured by the bno gyro' in text
    assert "Heading correction SLAM (slam)" in text
    # /odom over-reads 5% of 0.4 rad/s for 20 s: SLAM takes 0.4 rad = 22.9 deg back
    assert '/odom      end  -22.' in text


@pytest.fixture
def rosbag2_py():
    return pytest.importorskip(
        'rosbag2_py', reason='needs ROS sourced (docs/run-analysis.md); runs sourced')


def test_read_imu_bag_round_trip(tmp_path, rosbag2_py):
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu

    uri = tmp_path / 'bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = {'/imu/bno/data': 'sensor_msgs/msg/Imu', '/odom/imu': 'nav_msgs/msg/Odometry',
              '/pose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
              '/slam_pose': 'geometry_msgs/msg/PoseStamped'}
    for i, (name, type_) in enumerate(topics.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=i, name=name, type=type_, serialization_format='cdr'))
    imu = Imu()
    imu.angular_velocity.z = 0.37
    writer.write('/imu/bno/data', serialize_message(imu), 1_000_000_000)
    odom = Odometry()
    odom.pose.pose.position.x = 1.5
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(0.25), math.cos(0.25)
    writer.write('/odom/imu', serialize_message(odom), 2_000_000_000)
    pose = PoseWithCovarianceStamped()
    pose.pose.pose.position.y = -2.0
    pose.pose.pose.orientation.w = 1.0
    writer.write('/pose', serialize_message(pose), 3_000_000_000)
    sp = PoseStamped()
    sp.pose.position.x = 4.0
    sp.pose.orientation.z, sp.pose.orientation.w = math.sin(-0.5), math.cos(-0.5)
    writer.write('/slam_pose', serialize_message(sp), 4_000_000_000)
    del writer

    data = imu_report.read_imu_bag(uri)
    assert data['bno'].tolist() == [[1.0, 0.37]]
    assert data['vesc'].shape == (0, 2)
    assert data['odom_imu'][0] == pytest.approx([2.0, 1.5, 0.0, 0.5], abs=1e-9)
    assert data['slam'][0] == pytest.approx([3.0, 0.0, -2.0, 0.0], abs=1e-9)
    assert data['slam_pose'][0] == pytest.approx([4.0, 4.0, 0.0, -1.0], abs=1e-9)


def test_heading_correction_ignores_where_each_frame_started():
    # The odom frame starts wherever the car was powered up, the map frame
    # where SLAM started: a constant offset between them is not drift.
    # oracle: invariant -- offset 1.0 rad, identical turning: correction 0.
    t = np.arange(0, 10, 0.02)
    slam = np.column_stack([t, t * 0, t * 0, 0.3 * t])
    odom = _odom(t, 1.0, yaw=0.3 * t + 1.0)
    (name, final, peak, _), = imu_report.heading_corrections(
        slam, {'/odom': (t, 0.3 * t + 1.0)}, odom)
    assert final == pytest.approx(0.0, abs=1e-9) and peak == pytest.approx(0.0, abs=1e-9)


def test_a_parked_run_says_there_was_not_enough_driving():
    # The 2026-10-24 stationary check: real bag, car parked -- the report
    # must say so, not print NaN gains.
    t = np.arange(0, 30, 0.02)
    run = SimpleNamespace(odom=_odom(t, 0.0),
                          drive=np.column_stack([t, t, t * 0, t * 0]))
    imu = {'bno': np.column_stack([t, t * 0]), 'vesc': np.zeros((0, 2)),
           'odom_imu': np.zeros((0, 4)), 'slam': np.zeros((0, 4)), 'slam_pose': np.zeros((0, 4))}
    text = '\n'.join(imu_report.build_report(run, imu))
    assert 'only 0 gyro samples above 0.5 m/s' in text
    assert 'nan' not in text.lower()
