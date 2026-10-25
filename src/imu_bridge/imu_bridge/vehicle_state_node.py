"""Fused speed, measured steering angle, and gyro odometry.

Support code: publishes only nav_msgs/Odometry, sensor_msgs/JointState and,
when asked, one transform (imu_odom_frame -> odom). Never /drive,
/ackermann_cmd or /commands/*, so it cannot move the car and needs no LB
deadman; vehicle_state.check_output_topics refuses any remap that would
point an output at a drive topic or back onto /odom.

Inputs: /odom (vesc_to_odom_node's wheel speed), vesc_to_odom_node's
odom -> base_link transform on /tf, and imu_bridge_node's calibrated,
car-axis /imu/bno/data and /imu/vesc/data. Start imu_bridge first
(vehicle_state_launch.py starts both).

/odom/fused is /odom with twist.linear.x (and its variance) replaced; pose,
yaw rate, frames and stamp are /odom's own. A consumer switches to it with
odom_topic:=/odom/fused and nothing else. It is published once per /odom
message, so if /odom stops, so does /odom/fused -- a consumer's stale-odometry
check fires exactly as it would on /odom.

/steering/measured is one joint named "steering": position = the effective
steering angle [rad], positive left, NaN when it cannot be measured (too slow,
no gyro, or the two gyros disagree).

/odom/imu is vesc_to_odom's odometry re-integrated with a gyro's heading,
in frame odom_imu, once per vesc_to_odom transform. With publish_odom_tf it
also publishes odom_imu -> odom, so that odom_imu -> base_link is the gyro
pose and slam_toolbox can run on it (odom_frame: odom_imu) while
vesc_to_odom keeps owning odom -> base_link. If anything else gives `odom`
a parent (slam_toolbox left on `odom`, a particle filter), the node stops
publishing that transform for good and says so: two parents would make the
tree flip.

All the math is in vehicle_state.py (no rclpy), tested in
test/test_vehicle_state.py and test/test_gyro_odometry.py.
"""

from __future__ import annotations

from dataclasses import fields
import math
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from imu_bridge.vehicle_state import (
    MODE_CLAMPED,
    MODE_FUSED,
    MODE_INVALID,
    MODE_SLIP,
    MODE_WHEEL_ONLY,
    MODEL,
    EstimatorConfig,
    GyroOdometry,
    VehicleStateEstimator,
    check_frames,
    check_output_topics,
    foreign_parent,
)

LOG_PERIOD_S = 5.0
# Reliable publishers: they match reliable subscribers (gap_follow's /odom
# subscription is) and best-effort ones (rosbag, sensor-data QoS).
PUB_QOS = 10
# /tf as tf2 itself subscribes to it.
TF_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
# EstimatorConfig's two per-IMU tables, flattened into scalar ROS parameters.
TABLE_PARAMS = {
    'x_offset': ('bno_x_offset', 'vesc_x_offset'),
    'gyro_weight': ('bno_gyro_weight', 'vesc_gyro_weight'),
}
# EstimatorConfig fields that are lists of names, not numbers.
NAME_LISTS = ('heading_sources',)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp_s(stamp):
    return Time.from_msg(stamp).nanoseconds * 1e-9


class VehicleStateNode(Node):
    def __init__(self):
        super().__init__('vehicle_state_node')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('fused_odom_topic', '/odom/fused')
        self.declare_parameter('steering_topic', '/steering/measured')
        self.declare_parameter('imu_odom_topic', '/odom/imu')
        self.declare_parameter('bno_topic', '/imu/bno/data')
        self.declare_parameter('vesc_topic', '/imu/vesc/data')
        self.declare_parameter('steering_rate_hz', 50.0)
        self.declare_parameter('imu_odom_frame', 'odom_imu')
        self.declare_parameter('publish_odom_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # Every estimator knob, defaulting to EstimatorConfig's own defaults
        # (config/vehicle_state.yaml repeats them, with the reasoning).
        defaults = EstimatorConfig()
        for f in fields(EstimatorConfig):
            if f.name in TABLE_PARAMS:
                table = dict(getattr(defaults, f.name))
                for source, param in zip(('bno', 'vesc'), TABLE_PARAMS[f.name]):
                    self.declare_parameter(param, float(table[source]))
            elif f.name in NAME_LISTS:
                self.declare_parameter(f.name, list(getattr(defaults, f.name)))
            else:
                self.declare_parameter(f.name, getattr(defaults, f.name))

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        odom_topic = p('odom_topic')
        fused_topic = p('fused_odom_topic')
        steering_topic = p('steering_topic')
        imu_odom_topic = p('imu_odom_topic')
        check_output_topics(odom_topic, fused_topic, steering_topic,
                            imu_odom_topic)  # raises, loudly
        self.imu_odom_frame = p('imu_odom_frame')
        self.odom_frame = p('odom_frame')
        self.base_frame = p('base_frame')
        check_frames(self.imu_odom_frame, self.odom_frame, self.base_frame)
        self.publish_tf = bool(p('publish_odom_tf'))
        rate = float(p('steering_rate_hz'))
        if not (math.isfinite(rate) and rate > 0.0):
            raise ValueError(f'steering_rate_hz must be finite and > 0, got {rate}')

        kwargs = {}
        for f in fields(EstimatorConfig):
            if f.name in TABLE_PARAMS:
                kwargs[f.name] = tuple(
                    (source, float(p(param)))
                    for source, param in zip(('bno', 'vesc'), TABLE_PARAMS[f.name]))
            elif f.name in NAME_LISTS:
                kwargs[f.name] = tuple(str(v) for v in p(f.name))
            elif f.name == 'accel_source':
                kwargs[f.name] = str(p(f.name))
            else:
                kwargs[f.name] = float(p(f.name))
        self.cfg = EstimatorConfig(**kwargs)   # raises on a bad value, loudly
        self.est = VehicleStateEstimator(self.cfg)
        self.odo = GyroOdometry(self.est)

        self.pub_odom = self.create_publisher(Odometry, fused_topic, PUB_QOS)
        self.pub_steering = self.create_publisher(JointState, steering_topic, PUB_QOS)
        self.pub_imu_odom = self.create_publisher(Odometry, imu_odom_topic, PUB_QOS)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.tf_blocked_by = None    # the other parent of odom, once seen
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, p('bno_topic'), lambda m: self._on_imu('bno', m),
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, p('vesc_topic'), lambda m: self._on_imu('vesc', m),
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self._on_tf, TF_QOS)
        self.create_timer(1.0 / rate, self._publish_steering)
        self.create_timer(LOG_PERIOD_S, self._log)

        self.imu_count = {'bno': 0, 'vesc': 0}
        self.modes = dict.fromkeys(
            (MODE_FUSED, MODE_CLAMPED, MODE_SLIP, MODE_WHEEL_ONLY, MODE_INVALID), 0)
        self.heading_sources = dict.fromkeys(('bno', 'vesc', MODEL), 0)
        self.steering_valid = 0
        self.steering_total = 0
        self.get_logger().info(
            f'vehicle_state up: {odom_topic} -> {fused_topic}, {steering_topic} at '
            f'{rate:.0f} Hz, {imu_odom_topic} in {self.imu_odom_frame}; accel from '
            f'{self.cfg.accel_source}, heading from {list(self.cfg.heading_sources)}, '
            f'gyro weights {dict(self.cfg.gyro_weight)}, wheelbase {self.cfg.wheelbase} m; '
            + (f'publishing {self.imu_odom_frame} -> {self.odom_frame}'
               if self.publish_tf else 'no transform (publish_odom_tf false)'))

    # ------------------------------------------------------------ callbacks

    def _on_imu(self, source, msg):
        self.imu_count[source] += 1
        self.est.on_imu(source, time.monotonic(), _stamp_s(msg.header.stamp),
                        msg.linear_acceleration.x, msg.linear_acceleration.z,
                        msg.angular_velocity.z)

    def _on_odom(self, msg):
        e = self.est.on_wheel(time.monotonic(), msg.twist.twist.linear.x)
        self.modes[e.mode] += 1
        msg.twist.twist.linear.x = float(e.speed)
        cov = list(msg.twist.covariance)
        cov[0] = float(e.variance)
        msg.twist.covariance = cov
        self.pub_odom.publish(msg)

    def _on_tf(self, msg):
        if self.publish_tf and self.tf_blocked_by is None:
            other = foreign_parent(
                [(t.header.frame_id, t.child_frame_id) for t in msg.transforms],
                self.imu_odom_frame, self.odom_frame)
            if other is not None:
                self.tf_blocked_by = other
                self.get_logger().error(
                    f"'{other}' -> '{self.odom_frame}' is on /tf as well: "
                    f"'{self.odom_frame}' cannot have two parents, so this node has "
                    f"STOPPED publishing {self.imu_odom_frame} -> {self.odom_frame} for "
                    f"good. Point slam_toolbox at odom_frame: {self.imu_odom_frame} "
                    f"(map_course gyro_odom:=true), or run without publish_odom_tf.")
        for t in msg.transforms:
            if t.header.frame_id == self.odom_frame and t.child_frame_id == self.base_frame:
                self._on_wheel_pose(t)

    def _on_wheel_pose(self, t):
        q = t.transform.rotation
        step = self.odo.on_wheel_pose(time.monotonic(), _stamp_s(t.header.stamp),
                                      t.transform.translation.x,
                                      t.transform.translation.y, _yaw(q))
        if step is None:
            return
        self.heading_sources[step.source] += 1
        x, y, yaw = step.pose
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = self.imu_odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = float(step.speed)
        odom.twist.twist.angular.z = float(step.yaw_rate)
        self.pub_imu_odom.publish(odom)
        if self.tf_broadcaster is not None and self.tf_blocked_by is None:
            cx, cy, cyaw = step.correction
            tf = TransformStamped()
            tf.header.stamp = t.header.stamp      # vesc_to_odom's own stamp: the
            tf.header.frame_id = self.imu_odom_frame  # chain composes at that instant
            tf.child_frame_id = self.odom_frame
            tf.transform.translation.x = float(cx)
            tf.transform.translation.y = float(cy)
            tf.transform.rotation.z = math.sin(cyaw / 2.0)
            tf.transform.rotation.w = math.cos(cyaw / 2.0)
            self.tf_broadcaster.sendTransform(tf)

    def _publish_steering(self):
        angle = self.est.steering(time.monotonic())
        self.steering_total += 1
        self.steering_valid += math.isfinite(angle)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['steering']
        msg.position = [float(angle)]
        self.pub_steering.publish(msg)

    # ------------------------------------------------------------ logs

    def _log(self):
        log = self.get_logger()
        now = time.monotonic()
        speeds = ', '.join(f'{m} {n}' for m, n in self.modes.items() if n)
        headings = ', '.join(f'{m} {n}' for m, n in self.heading_sources.items() if n)
        log.info(
            f'rates: bno {self.imu_count["bno"] / LOG_PERIOD_S:.0f} Hz, vesc '
            f'{self.imu_count["vesc"] / LOG_PERIOD_S:.0f} Hz | speed: {speeds or "no /odom"}'
            f' | accel bias {self.est.filter.b:+.3f} m/s^2 | steering valid '
            f'{self.steering_valid}/{self.steering_total} | heading: '
            f'{headings or "no odom -> base_link transform"}')

        if sum(self.modes.values()) == 0:
            log.warn('no /odom in the last 5 s: /odom/fused is silent (is bringup up?)')
        elif (self.cfg.accel_source != 'none'
              and self.modes[MODE_WHEEL_ONLY] == sum(self.modes.values())):
            log.warn(f'no usable {self.cfg.accel_source} accelerometer: /odom/fused is '
                     'exactly the wheel speed')
        if self.modes[MODE_SLIP]:
            log.warn(f'wheels rejected as slipping on {self.modes[MODE_SLIP]} of '
                     f'{sum(self.modes.values())} /odom messages')
        steps = sum(self.heading_sources.values())
        if steps == 0:
            log.warn(f'no {self.odom_frame} -> {self.base_frame} transform on /tf in the '
                     f'last 5 s: /odom/imu is silent (vesc_to_odom_node publish_tf?)')
        elif (self.heading_sources[MODEL] == steps
              and self.cfg.heading_sources != ('none',)):
            log.warn('every /odom/imu step used the steering model: no usable gyro, so '
                     '/odom/imu is exactly /odom')
        if self.tf_blocked_by is not None:
            log.error(f'still NOT publishing {self.imu_odom_frame} -> {self.odom_frame}: '
                      f"'{self.tf_blocked_by}' is its parent")
        for name, s in self.est.src.items():
            if self.imu_count[name] and not s.mount_ok:
                log.warn(f'{name}: averaged z acceleration {s.gravity_z:+.2f} m/s^2, not '
                         '~+9.8: mounted wrong (upside down, or wrong rpy_deg in '
                         'car.yaml)? Its gyro and accelerometer are not used.')
            elif self.imu_count[name] == 0 and dict(self.cfg.gyro_weight)[name] > 0.0:
                log.warn(f'{name}: no samples in the last 5 s')
        if self.est.gyros_disagree(now):
            log.warn(f'gyros disagree: bno - vesc yaw rate averages '
                     f'{self.est.gyro_difference:+.3f} rad/s; /steering/measured is NaN '
                     'and /odom/imu uses the steering model until they agree or one '
                     'drops out')

        self.imu_count = dict.fromkeys(self.imu_count, 0)
        self.modes = dict.fromkeys(self.modes, 0)
        self.heading_sources = dict.fromkeys(self.heading_sources, 0)
        self.steering_valid = self.steering_total = 0


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VehicleStateNode()   # a bad parameter or topic raises here, loudly
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
