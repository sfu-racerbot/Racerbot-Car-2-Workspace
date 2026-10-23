"""Calibrated IMU topics for the car, from the VESC's IMU and an optional BNO085.

Support code: publishes only sensor_msgs/Imu and a copy of /odom with a real
twist covariance. Never /drive, /ackermann_cmd or /commands/*, so it cannot
move the car and needs no LB deadman. It never opens the VESC serial port --
vesc_driver_node owns that; the VESC IMU arrives over /sensors/imu/raw.

All the math is in convert.py (no rclpy), tested in test/test_convert.py.
"""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu

from imu_bridge.convert import (
    ORIENTATION_UNKNOWN,
    StartupBias,
    covariance_diag,
    load_calibration,
    load_mount,
    monotonic_to_ros_ns,
    resolve_primary,
    vesc_raw_to_si,
    wheel_twist_covariance,
)

RATE_LOG_PERIOD_S = 5.0
# A wheel speed older than this is "unknown", which StartupBias treats as moving.
ODOM_STALE_S = 0.5
BNO_DRAIN_PERIOD_S = 0.005
# Publishers are reliable: they match both reliable subscribers (ekf_node)
# and best-effort ones (rosbag, sensor-data QoS).
PUB_QOS = 10


def _finite3(v):
    return v.shape == (3,) and bool(np.all(np.isfinite(v)))


class _Stream:
    """One IMU's calibration, startup bias and message counter."""

    def __init__(self, name, calibration, startup_bias):
        self.name = name
        self.cal = calibration
        self.bias = startup_bias
        self.count = 0
        self.dropped = 0
        self.bias_logged = startup_bias.done


class ImuBridgeNode(Node):
    def __init__(self):
        super().__init__('imu_bridge_node')

        self.declare_parameter('racer_imu_root', '~/imu-calibration')
        self.declare_parameter('car_yaml', '')
        self.declare_parameter('vesc_topic', '/sensors/imu/raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('bno_port', '')
        self.declare_parameter('bno_baud', 921600)
        self.declare_parameter('primary', 'auto')
        self.declare_parameter('gyro_sigma', 0.0017)
        self.declare_parameter('accel_sigma', 0.03)
        self.declare_parameter('wheel_speed_sigma', 0.08)
        self.declare_parameter('startup_bias_s', 3.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.racer_imu_root = os.path.expanduser(p('racer_imu_root'))
        car_yaml = os.path.expanduser(p('car_yaml')) or os.path.join(
            self.racer_imu_root, 'config', 'car.yaml')
        self.frame_id = p('frame_id')
        bno_port = p('bno_port')
        self.primary = resolve_primary(p('primary'), bno_port)
        self.gyro_cov = covariance_diag(p('gyro_sigma'))
        self.accel_cov = covariance_diag(p('accel_sigma'))
        self.twist_cov = wheel_twist_covariance(p('wheel_speed_sigma'))
        bias_s = float(p('startup_bias_s'))

        if not os.path.isfile(car_yaml):
            raise FileNotFoundError(
                f'{car_yaml} not found. imu_bridge needs the imu-calibration repo for the '
                'IMU mount rotation (the VESC is mounted rotated ~85 deg, so identity would '
                'swap x and y): clone github.com/sfu-racerbot/imu-calibration to '
                f'{self.racer_imu_root}, or set racer_imu_root / car_yaml.')

        self.vesc = self._make_stream('vesc', car_yaml, bias_s)
        self.vesc_all_zero = 0

        self.speed = None
        self.speed_t = -math.inf
        self.odom_count = 0

        self.pub_vesc = self.create_publisher(Imu, '/imu/vesc/data', PUB_QOS)
        self.pub_primary = self.create_publisher(Imu, '/imu/data', PUB_QOS)
        self.pub_wheel = self.create_publisher(Odometry, '/imu_bridge/wheel_odom', PUB_QOS)

        self.create_subscription(Imu, p('vesc_topic'), self._on_vesc,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, p('odom_topic'), self._on_odom,
                                 qos_profile_sensor_data)

        self.bno = None
        self.bno_driver = None
        if bno_port:
            self.bno = self._make_stream('bno', car_yaml, bias_s)
            self.bno_driver = self._start_bno(bno_port, int(p('bno_baud')))
            self.pub_bno_raw = self.create_publisher(Imu, '/imu/bno/raw', PUB_QOS)
            self.pub_bno = self.create_publisher(Imu, '/imu/bno/data', PUB_QOS)
            self.create_timer(BNO_DRAIN_PERIOD_S, self._drain_bno)

        self.create_timer(RATE_LOG_PERIOD_S, self._log_rates)
        self.get_logger().info(
            f'imu_bridge up: primary={self.primary}, frame_id={self.frame_id}, '
            f'startup bias {bias_s:.1f} s (keep the car still), BNO '
            f'{"on " + bno_port if bno_port else "off"}')

    # ------------------------------------------------------------ setup

    def _make_stream(self, name, car_yaml, bias_s):
        rpy_deg, calib_path = load_mount(car_yaml, name)
        cal = load_calibration(calib_path, rpy_deg)
        self.get_logger().info(
            f'{name}: mount rpy_deg={rpy_deg}, calibration: {cal.source}')
        if calib_path is not None and cal.source != str(calib_path):
            self.get_logger().warn(f'{name}: {calib_path} not found, running uncalibrated')
        return _Stream(name, cal, StartupBias(seconds=bias_s))

    def _start_bno(self, port, baud):
        if self.racer_imu_root not in sys.path:
            sys.path.insert(0, self.racer_imu_root)
        from racer_imu.bno_driver import BnoDriver
        driver = BnoDriver(port, name='bno', baud=baud)
        driver.start()
        return driver

    # ------------------------------------------------------------ callbacks

    def _on_odom(self, msg):
        self.odom_count += 1
        self.speed = msg.twist.twist.linear.x
        self.speed_t = time.monotonic()
        msg.twist.covariance = self.twist_cov
        self.pub_wheel.publish(msg)

    def _current_speed(self):
        if time.monotonic() - self.speed_t > ODOM_STALE_S:
            return None
        return self.speed

    def _on_vesc(self, msg):
        acc_g = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        gyro_dps = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                             msg.angular_velocity.z])
        if not (np.any(acc_g) or np.any(gyro_dps)):
            # Measured 2026-08-19: an IMU disabled in VESC Tool answers the
            # poll with all zeros -- no gravity, which no real reading lacks.
            self.vesc_all_zero += 1
        acc, gyro = vesc_raw_to_si(acc_g, gyro_dps)
        out = self._calibrate(self.vesc, acc, gyro)
        if out is None:
            return
        data = self._imu_msg(msg.header.stamp, self.frame_id, *out)
        self.pub_vesc.publish(data)
        if self.primary == 'vesc':
            self.pub_primary.publish(data)

    def _drain_bno(self):
        samples = self.bno_driver.drain()
        if not samples:
            return
        mono_now = time.monotonic()
        ros_now_ns = self.get_clock().now().nanoseconds
        for s in samples:
            stamp = Time(nanoseconds=monotonic_to_ros_ns(s.t, mono_now, ros_now_ns)).to_msg()
            raw = self._imu_msg(stamp, 'bno_sensor', s.acc, s.gyro)
            if np.all(np.isfinite(s.quat)):
                raw.orientation.w, raw.orientation.x, raw.orientation.y, raw.orientation.z = (
                    float(q) for q in s.quat)
                raw.orientation_covariance = [0.0] * 9   # present, covariance unknown
            self.pub_bno_raw.publish(raw)
            out = self._calibrate(self.bno, np.asarray(s.acc, float), np.asarray(s.gyro, float))
            if out is None:
                continue
            data = self._imu_msg(stamp, self.frame_id, *out)
            self.pub_bno.publish(data)
            if self.primary == 'bno':
                self.pub_primary.publish(data)

    # ------------------------------------------------------------ helpers

    def _calibrate(self, stream, acc, gyro):
        """Calibrated car-axis (acc, gyro) with the startup bias removed, or None."""
        if not (_finite3(acc) and _finite3(gyro)):
            stream.dropped += 1
            return None
        acc_car, gyro_car = stream.cal.apply(acc, gyro)
        stream.bias.update(time.monotonic(), gyro_car, self._current_speed())
        if stream.bias.done and not stream.bias_logged:
            stream.bias_logged = True
            r = stream.bias.residual()
            self.get_logger().info(
                f'{stream.name}: startup gyro bias removed '
                f'[{r[0]:+.5f}, {r[1]:+.5f}, {r[2]:+.5f}] rad/s '
                f'([{math.degrees(r[0]):+.3f}, {math.degrees(r[1]):+.3f}, '
                f'{math.degrees(r[2]):+.3f}] deg/s)')
        stream.count += 1
        return acc_car, gyro_car - stream.bias.residual()

    def _imu_msg(self, stamp, frame_id, acc, gyro):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.orientation_covariance = list(ORIENTATION_UNKNOWN)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            float(v) for v in gyro)
        msg.angular_velocity_covariance = self.gyro_cov
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
            float(v) for v in acc)
        msg.linear_acceleration_covariance = self.accel_cov
        return msg

    def _log_rates(self):
        log = self.get_logger()
        period = RATE_LOG_PERIOD_S
        parts = [f'vesc {self.vesc.count / period:.1f} Hz',
                 f'odom {self.odom_count / period:.1f} Hz']
        if self.bno is not None:
            parts.append(f'bno {self.bno.count / period:.1f} Hz')
        log.info('rates: ' + ', '.join(parts))

        if self.vesc.count == 0:
            log.warn('no VESC IMU data in the last 5 s: is bringup running, and is '
                     'the IMU enabled in VESC Tool? (ros2 topic hz /sensors/imu/raw)')
        elif self.vesc_all_zero == self.vesc.count:
            log.warn('every VESC IMU sample in the last 5 s was all zeros: the IMU is '
                     'off or not fitted (VESC Tool -> App Settings -> IMU, sample rate > 0). '
                     'Outputs are meaningless until it is on.')
        self.vesc_all_zero = 0
        if self.odom_count == 0:
            log.warn('no /odom in the last 5 s: wheel_odom is silent and the startup '
                     'bias cannot finish (it treats unknown speed as moving)')
        if self.bno is not None and self.bno.count == 0:
            err = self.bno_driver.last_error or 'no error reported'
            log.warn(f'no BNO085 data in the last 5 s ({err})')
        for stream in (self.vesc, self.bno):
            if stream is not None and stream.dropped:
                log.warn(f'{stream.name}: dropped {stream.dropped} non-finite samples')
                stream.dropped = 0
            if stream is not None:
                stream.count = 0
        self.odom_count = 0

    def destroy_node(self):
        if self.bno_driver is not None:
            self.bno_driver.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuBridgeNode()   # a missing car.yaml or bad calibration raises here, loudly
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
