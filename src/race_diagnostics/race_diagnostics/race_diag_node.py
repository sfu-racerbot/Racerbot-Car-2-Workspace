"""
race_diag_node.py

A read-only witness to one run of the driving stack.

This node ONLY subscribes. It never publishes to any topic, so it cannot
influence how the car drives and is not subject to the LB deadman policy
that governs driving code (see docs/adding-your-own-code.md -- this is
support tooling, the same category as web_dashboard). It is safe to leave
running alongside anything, including a race.

It answers the questions that the launch terminal alone could not during
the 2026-07-27 debugging session:

  * Is each link of the pipeline alive?
        /scan -> slam_toolbox -> /map + map->base_link TF -> /slam_pose
              -> auto_map_race_node -> /drive
    A dead link upstream makes every node downstream merely "waiting",
    which reads identically to "working but idle" in their own logs.

  * How stale is the pose *really*? Localization staleness is invisible
    from a topic's message rate: auto_map_race_node republishes SLAM's
    transform at a fixed 40Hz whatever its age, so a frozen transform
    arrives just as punctually as a live one. Only the header stamp
    reveals it. This is what put the car into a wall, and it is also what
    makes the web dashboard's scan sit off the map (overlay error is
    roughly speed x pose lag).

  * Is the pose advancing at all while odometry says the car is moving?

Output goes two places at once:
  * stdout, human-readable, deliberately sparse (state changes plus a
    periodic summary that repeats only when something actually changed).
  * an events.jsonl file, one JSON object per line, for offline analysis
    by a person or an agent long after the run. See docs/run-diagnostics.md.
"""

import json
import math
import os
import time
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Joy, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class RaceDiagNode(Node):
    """Watch every link of the driving pipeline and record what happened."""

    def __init__(self):
        super().__init__('race_diag_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('pose_topic', '/slam_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('summary_period_sec', 20.0)
        self.declare_parameter('idle_period_sec', 600.0)
        self.declare_parameter('stale_sec', 3.0)
        self.declare_parameter('pose_lag_alert_sec', 0.30)
        self.declare_parameter('pose_still_travel_m', 0.02)
        self.declare_parameter('output_directory', '')

        def value(name):
            return self.get_parameter(name).value

        self.summary_period_sec = float(value('summary_period_sec'))
        self.idle_period_sec = float(value('idle_period_sec'))
        self.stale_sec = float(value('stale_sec'))
        self.pose_lag_alert_sec = float(value('pose_lag_alert_sec'))
        self.pose_still_travel_m = float(value('pose_still_travel_m'))
        self.map_frame = str(value('map_frame'))
        self.base_frame = str(value('base_frame'))

        self.counts = {'scan': 0, 'map': 0, 'pose': 0, 'odom': 0, 'drive': 0, 'joy': 0}
        self.last = {key: None for key in self.counts}
        self.seen = set()
        self.map_info = None
        self.speed = 0.0
        self.cmd_speed = 0.0
        self.pose_xy = None
        self.pose_lag = None
        self.pose_lag_max = 0.0
        self.deadman_held = False
        self.tf_ok = False
        self._pose_lag_alerted = False
        self._pose_still_since = None
        self._last_summary = 0.0
        self._last_signature = None

        self._events_file = None
        directory = str(value('output_directory'))
        if directory:
            path = Path(os.path.expanduser(directory))
            path.mkdir(parents=True, exist_ok=True)
            # Line-buffered so a hard kill (or a car that has to be
            # switched off in a hurry) still leaves a complete file.
            self._events_file = open(path / 'events.jsonl', 'a', buffering=1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            LaserScan, str(value('scan_topic')), self._scan, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, str(value('map_topic')), self._map, latched)
        self.create_subscription(
            PoseStamped, str(value('pose_topic')), self._pose, 10)
        self.create_subscription(
            Odometry, str(value('odom_topic')), self._odom, 10)
        self.create_subscription(
            AckermannDriveStamped, str(value('drive_topic')), self._drive, 10)
        self.create_subscription(Joy, str(value('joy_topic')), self._joy, 10)
        self.create_timer(1.0, self._tick)

        self._emit('armed', 'probe armed (subscribe-only); waiting for the stack')

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------
    def _emit(self, category: str, message: str, **fields):
        stamp = time.time()
        print(f'[{time.strftime("%H:%M:%S")}] {category.upper()}: {message}', flush=True)
        if self._events_file is not None:
            record = {'t': stamp, 'category': category, 'message': message}
            record.update(fields)
            self._events_file.write(json.dumps(record) + '\n')

    def _mark(self, key: str, detail: str = ''):
        self.counts[key] += 1
        self.last[key] = time.monotonic()
        if key not in self.seen:
            self.seen.add(key)
            self._emit('first_message', f'first {key}: {detail}', topic=key)

    def _fresh(self, key: str) -> bool:
        return (self.last[key] is not None
                and time.monotonic() - self.last[key] < self.stale_sec)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _scan(self, msg):
        self._mark('scan', f'{len(msg.ranges)} beams, frame_id={msg.header.frame_id}')

    def _map(self, msg):
        info = msg.info
        total = info.width * info.height
        known = occupied = 0
        if total:
            known = total - msg.data.count(-1)
            occupied = sum(1 for cell in msg.data if cell >= 65)
        previous = self.map_info
        self.map_info = (info.width, info.height, info.resolution,
                         100.0 * known / total if total else 0.0, occupied)
        self._mark('map', f'{info.width}x{info.height} @ {info.resolution:.3f}m/cell, '
                          f'{occupied} occupied cells')
        if previous is not None and occupied and previous[4] == 0:
            self._emit('map_content',
                       f'map now has {occupied} occupied cells (was empty)',
                       occupied=occupied)

    def _pose(self, msg):
        previous = self.pose_xy
        self.pose_xy = (msg.pose.position.x, msg.pose.position.y)

        # Lag from the pose's own stamp. The single most valuable number
        # this probe produces: it is invisible in message rate, it is what
        # the pure_pursuit pose_stale/pose_frozen watchdogs act on, and at
        # speed v it is also the dashboard's scan-vs-map overlay error
        # (roughly v x lag).
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp > 0.0:
            lag = self.get_clock().now().nanoseconds / 1e9 - stamp
            self.pose_lag = lag
            if lag > self.pose_lag_max:
                self.pose_lag_max = lag
            if lag > self.pose_lag_alert_sec and not self._pose_lag_alerted:
                self._pose_lag_alerted = True
                self._emit('pose_lag_high',
                           f'/slam_pose stamp is {lag:.2f}s behind now; localization '
                           'is not keeping up (dashboard overlay error ~= speed x lag)',
                           lag_sec=round(lag, 3))

        moved = previous is None or math.dist(previous, self.pose_xy) >= self.pose_still_travel_m
        if moved:
            self._pose_still_since = None
        elif self._pose_still_since is None:
            self._pose_still_since = time.monotonic()
        self._mark('pose', f'map-frame ({self.pose_xy[0]:+.2f}, {self.pose_xy[1]:+.2f})')

    def _odom(self, msg):
        self.speed = msg.twist.twist.linear.x
        self._mark('odom', f'linear.x={self.speed:+.2f}m/s')

    def _drive(self, msg):
        self.cmd_speed = msg.drive.speed
        self._mark('drive', f'speed={self.cmd_speed:.2f}m/s')

    def _joy(self, msg):
        held = len(msg.buttons) > 4 and bool(msg.buttons[4])
        if held != self.deadman_held:
            self.deadman_held = held
            self._emit('deadman', 'LB held' if held else 'LB released', held=held)
        self._mark('joy', f'{len(msg.buttons)} buttons')

    # ------------------------------------------------------------------
    # Periodic
    # ------------------------------------------------------------------
    def _tick(self):
        now = time.monotonic()
        try:
            self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            if not self.tf_ok:
                self.tf_ok = True
                self._emit('tf_up',
                           f'{self.map_frame}->{self.base_frame} available; '
                           'SLAM is localizing')
        except TransformException as exc:
            if self.tf_ok:
                self.tf_ok = False
                self._emit('tf_lost',
                           f'{self.map_frame}->{self.base_frame} lost: {str(exc)[:120]}')

        frozen = (self._pose_still_since is not None
                  and abs(self.speed) > 0.2
                  and now - self._pose_still_since > 0.5)

        period = self.summary_period_sec if self.seen else self.idle_period_sec
        if now - self._last_summary < period:
            return
        # Repeat a summary only when the picture actually changed, so a
        # parked car or a dead node cannot bury the events that matter.
        signature = (self._fresh('scan'), self._fresh('odom'), self._fresh('drive'),
                     self._fresh('pose'), self.tf_ok, frozen,
                     None if self.map_info is None else self.map_info[4],
                     abs(self.speed) > 0.05, round(self.cmd_speed, 1))
        if signature == self._last_signature and now - self._last_summary < self.idle_period_sec:
            return
        self._last_signature = signature
        self._last_summary = now

        def state(key):
            if self.last[key] is None:
                return 'NEVER'
            age = now - self.last[key]
            return 'ok' if age < self.stale_sec else f'STALE {age:.0f}s'

        if self.map_info is None:
            map_text = 'map=NEVER'
        else:
            w, h, res, pct, occ = self.map_info
            map_text = f'map={w}x{h}@{res:.3f} {pct:.0f}%known {occ}occ'
        lag_text = '' if self.pose_lag is None else \
            f' lag={self.pose_lag:.2f}s(max {self.pose_lag_max:.2f})'
        self._emit(
            'status',
            f'scan={state("scan")} | {map_text} | '
            f'{self.map_frame}->{self.base_frame}={"OK" if self.tf_ok else "MISSING"} | '
            f'pose={state("pose")}{lag_text}{" FROZEN" if frozen else ""} | '
            f'odom={state("odom")} {self.speed:+.2f}m/s | '
            f'drive={state("drive")} cmd={self.cmd_speed:.2f}m/s | '
            f'LB={"held" if self.deadman_held else "released"}',
            pose_lag_sec=None if self.pose_lag is None else round(self.pose_lag, 3),
            pose_lag_max_sec=round(self.pose_lag_max, 3),
            tf_ok=self.tf_ok, speed=round(self.speed, 2),
            cmd_speed=round(self.cmd_speed, 2), pose_frozen=frozen,
            occupied_cells=None if self.map_info is None else self.map_info[4])

    def close(self):
        if self._events_file is not None:
            self._emit('closing', f'worst pose lag this run: {self.pose_lag_max:.2f}s')
            self._events_file.close()
            self._events_file = None


def main(args=None):
    rclpy.init(args=args)
    node = RaceDiagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
