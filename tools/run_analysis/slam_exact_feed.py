#!/usr/bin/env python3
"""Force recorded scan admission, or run the all-scans timing experiment.

Forced mode sends the first planned scan, three sacrificial scans immediately
before the second planned scan, then only the remaining planned scans. In
2.8.5 src/slam_toolbox_common.cpp:795, callback counts 2-4 are rejected for startup
stabilisation. Sacrifices must produce no pose. Admission overrides open the
throttle/time/travel gates: each planned scan already passed these on the car
relative to the same previous processed pose, so identical inputs pass again.
Scan matching and solver settings are preserved. Forced mode publishes the
recorded /tf message payloads in recorded order, dropping ONLY transforms
whose parent is map (including the map entries of mixed messages). Its
dynamic chain is derived from the recorded tree and configured odom/base
frames. Each pair gets its own at/before and at/after interpolation bracket;
no composed odometry interpolation or odom_tf_relay is used in this mode.

All-scans mode retains the recorded admission parameters, sends every scan
and keeps the odometry relay, preserving the round-1 timing experiment.

In all-scans mode, send odometry through the first sample at or after each
scan. In both modes, advance /clock to the scan stamp, never to its future
bracket. Only all-scans unplanned inputs may lack complete brackets, leaving
their fate to SLAM's TF filter; forced inputs and planned all-scans inputs
without brackets refuse before publication.

slam_toolbox 2.8.5 has an unbounded processing FIFO (slam_toolbox_sync.hpp:54;
slam_toolbox_sync.cpp:47-53,118-120), but its earlier TF message filter is
bounded by scan_queue_size (slam_toolbox_common.cpp:472-475). For bracketed
scans, wait for the map->odom TF header that proves laserCallback received
this scan (sync.cpp:100; common.cpp:507). This is a receipt barrier, not a
pose wait: rejected scans still update scan_header. It prevents the feeder
from overrunning the sensor-data subscription or TF filter. Boundary scans
cannot have callback receipts and are sent with a wall-clock discovery yield.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from slam_replay import (ExactPoseAccounting, bag_messages, exact_feed_steps,
                         forced_scan_plan, prepare_recorded_tf, stamp_ns)


class FeedTimeout(RuntimeError):
    """A wall-clock wait expired; pose-checkpoint timeouts may be nonfatal."""


class ExactFeed(Node):
    def __init__(self, odom_topic, params, exact_mode='forced'):
        super().__init__('slam_exact_feed')
        self.params = params
        self.exact_mode = exact_mode
        self.transform_timeout_ns = round(params.get('transform_timeout', .5) * 1_000_000_000)
        self.map_frame = params.get('map_frame', 'map')
        self.odom_frame = params['odom_frame']
        self.boundary_yield_s = max(.02, params.get('transform_publish_period', .05))
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.static_pub = self.create_publisher(
            TFMessage, '/tf_static', QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.recorded_tf_pub = self.create_publisher(
            TFMessage, '/tf', QoSProfile(history=HistoryPolicy.KEEP_ALL, depth=1)
        ) if exact_mode == 'forced' else None
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 100) if exact_mode == 'all-scans' else None
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', QoSProfile(history=HistoryPolicy.KEEP_ALL, depth=1))
        self.accounting = ExactPoseAccounting([])
        self.tf_stamps = set()
        self.callback_stamps = set()
        self.lookahead_samples = 0
        self.completed_scans = 0
        self.scans_sent = 0
        self.unbracketed_scans = 0
        self.sacrificial_stamps = set()
        self.tf_seen = set()
        self.tf_messages_sent = 0
        self.tf_info = {}
        self.create_subscription(PoseWithCovarianceStamped, '/pose',
                                 self.on_pose, 100)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)

    def on_pose(self, msg):
        stamp = stamp_ns(msg)
        self.accounting.observe(stamp)
        if stamp in self.sacrificial_stamps:
            raise RuntimeError(f'sacrificial scan unexpectedly produced /pose at {stamp}')

    def on_tf(self, msg):
        for transform in msg.transforms:
            self.tf_seen.add((transform.header.frame_id, transform.child_frame_id, stamp_ns(transform)))
            if transform.child_frame_id == 'base_link':
                self.tf_stamps.add(stamp_ns(transform))
            if (transform.header.frame_id == self.map_frame
                    and transform.child_frame_id == self.odom_frame):
                self.callback_stamps.add(stamp_ns(transform) - self.transform_timeout_ns)

    def wait(self, predicate, description, timeout=10.0, tick=None):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise FeedTimeout(f'timeout waiting for {description}')
            if tick is not None:
                tick()
            rclpy.spin_once(self, timeout_sec=.02)

    def clock(self, stamp):
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(stamp, 1_000_000_000)
        self.clock_pub.publish(msg)

    def feed(self, bag, odom_topic):
        scans, poses, odoms, statics, tf_messages = [], [], [], [], []
        source_topic = '/tf' if self.exact_mode == 'forced' else odom_topic
        for topic, msg in bag_messages(bag, ['/scan', '/pose', '/tf_static', source_topic]):
            if topic == '/scan':
                scans.append(msg)
            elif topic == '/pose':
                poses.append(stamp_ns(msg))
            elif topic == odom_topic:
                odoms.append(msg)
            elif topic == '/tf':
                tf_messages.append(msg)
            else:
                statics.extend(msg.transforms)
        if self.params.get('restamp_tf', False) or self.params.get('transform_publish_period', .05) <= 0:
            raise RuntimeError('scan receipt barrier requires restamp_tf=false and transform_publish_period>0')
        self.accounting = ExactPoseAccounting(poses)
        odoms.sort(key=stamp_ns)
        odom_stamps = [stamp_ns(msg) for msg in odoms]
        if self.exact_mode == 'forced':
            forced = forced_scan_plan([stamp_ns(msg) for msg in scans], poses)
            tf_messages, ends, self.tf_info = prepare_recorded_tf(
                tf_messages, statics, self.odom_frame, self.params.get('base_frame', 'base_footprint'),
                [stamp for stamp, _ in forced])
            by_stamp = {stamp_ns(msg): msg for msg in scans}
            scans = [by_stamp[stamp] for stamp, _ in forced]
            steps = [(stamp, planned, end, True) for (stamp, planned), end in zip(forced, ends)]
        else:
            steps = exact_feed_steps(odom_stamps, [stamp_ns(msg) for msg in scans], poses)
        self.wait(lambda: self.scan_pub.get_subscription_count() > 0
                  and ((self.recorded_tf_pub is not None
                        and self.recorded_tf_pub.get_subscription_count() > 1)
                       or (self.odom_pub is not None and self.odom_pub.get_subscription_count() > 0))
                  and self.clock_pub.get_subscription_count() > 0,
                  'SLAM, transform and clock subscribers')
        self.static_pub.publish(TFMessage(transforms=statics))
        if scans:
            # rosbag2 --use-sim-time defers topic discovery until its first clock.
            # Match its TF subscription as well as its pose subscription before
            # sending the first volatile TF message, so the replay bag retains it.
            def recorder_ready():
                pose_ready = any(info.node_name.startswith('rosbag2_recorder')
                                 for info in self.get_subscriptions_info_by_topic('/pose'))
                tf_ready = self.recorded_tf_pub is None or (
                    self.recorded_tf_pub.get_subscription_count() >= 3
                    and any(info.node_name.startswith('rosbag2_recorder')
                            for info in self.get_subscriptions_info_by_topic('/tf')))
                return pose_ready and tf_ready

            self.wait(recorder_ready, 'recorder subscriptions',
                      tick=lambda: self.clock(stamp_ns(scans[0])))
        index = 0
        for scan, (stamp, planned, end, bracketed) in zip(scans, steps):
            while index < end:
                if self.exact_mode == 'forced':
                    transforms = tf_messages[index]
                    self.recorded_tf_pub.publish(TFMessage(transforms=transforms))
                    keys = [(tf.header.frame_id, tf.child_frame_id, stamp_ns(tf)) for tf in transforms]
                    # Spin for receipt instead of blocking this executor in
                    # wait_for_all_acked while SLAM's TF timer keeps publishing.
                    self.wait(lambda: all(key in self.tf_seen for key in keys),
                              f'recorded TF receipt {keys} at scan {stamp}')
                    self.tf_messages_sent += 1
                    self.lookahead_samples += sum(stamp_ns(tf) > stamp for tf in transforms)
                else:
                    odom = odoms[index]
                    self.odom_pub.publish(odom)
                    self.wait(lambda: stamp_ns(odom) in self.tf_stamps,
                              f'relay TF at {stamp_ns(odom)}')
                    self.lookahead_samples += stamp_ns(odom) > stamp
                index += 1
            self.clock(stamp)
            if self.exact_mode == 'forced' and not planned:
                self.sacrificial_stamps.add(stamp)
            self.scan_pub.publish(scan)
            self.scans_sent += 1
            receipt_timed_out = False
            if bracketed:
                try:
                    self.wait(lambda: stamp in self.callback_stamps, f'SLAM scan callback at {stamp}')
                except FeedTimeout:
                    if not planned:
                        raise
                    receipt_timed_out = True
            else:
                self.unbracketed_scans += 1
                deadline = time.monotonic() + self.boundary_yield_s
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=.005)
            if planned:
                if not receipt_timed_out:
                    try:
                        self.wait(lambda: stamp in self.accounting.observed, f'/pose at scan stamp {stamp}')
                    except FeedTimeout:
                        pass  # checkpoint below records the timeout and enforces the consecutive limit
                if self.accounting.checkpoint(stamp):
                    self.completed_scans += 1
                    print(f'answered {stamp}', flush=True)
                else:
                    print(f'missing planned pose {stamp}; continuing', flush=True)
            elif self.exact_mode == 'forced':
                # Continue checking all later pose callbacks too, so a delayed
                # sacrificial pose is fatal rather than silently counted extra.
                deadline = time.monotonic() + .05
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=.01)
        # Capture final unplanned poses while the synchronous FIFO drains.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
        print(f'finished {self.scans_sent} scans, {self.completed_scans} planned poses answered', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--odom-topic', required=True)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--slam-params', type=Path, required=True)
    parser.add_argument('--exact-mode', choices=('forced', 'all-scans'), default='forced')
    args = parser.parse_args()
    rclpy.init()
    params = yaml.safe_load(args.slam_params.read_text())['slam_toolbox']['ros__parameters']
    node = ExactFeed(args.odom_topic, params, args.exact_mode)
    try:
        node.feed(args.bag, args.odom_topic)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        args.result.write_text(json.dumps({'lookahead_samples': node.lookahead_samples,
                                          'unbracketed_scans': node.unbracketed_scans,
                                          'scans_sent': node.scans_sent,
                                          'scan_callbacks_confirmed': len(node.callback_stamps),
                                          'completed_scans': node.completed_scans,
                                          'sacrificial_scan_stamps': sorted(node.sacrificial_stamps),
                                          'tf_messages_sent': node.tf_messages_sent,
                                          **node.tf_info,
                                          **node.accounting.summary()}) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
