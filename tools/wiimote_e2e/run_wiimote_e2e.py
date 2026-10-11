#!/usr/bin/env python3
"""End-to-end checks of the Wiimote control layer, without a Wiimote.

1. The real wiimote_driver_node starts with no Wiimote connected: it must
   not crash, must say how to connect one, and must publish "nothing held"
   on /wiimote/joy (never on /joy).
2. The real joy_teleop with config/wiimote_teleop.yaml, fed the Joy messages
   the driver produces, must put the right commands on /teleop: drive only
   while "2" is held, an active zero when it is not.

Run from a sourced, built workspace, isolated from the car's graph:

    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 tools/wiimote_e2e/run_wiimote_e2e.py
"""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / 'src' / 'racerbot_wiimote' / 'racerbot_wiimote_driver' / 'config'


class Probe(Node):
    def __init__(self):
        super().__init__('wiimote_e2e_probe')
        self.lock = threading.Lock()
        self.wiimote_joy = []
        self.plain_joy = []
        self.teleop = []
        self.create_subscription(Joy, '/wiimote/joy', lambda m: self.keep(self.wiimote_joy, m), 50)
        self.create_subscription(Joy, '/joy', lambda m: self.keep(self.plain_joy, m), 50)
        self.create_subscription(AckermannDriveStamped, '/teleop',
                                 lambda m: self.keep(self.teleop, m), 50)
        self.fake_joy = self.create_publisher(Joy, '/wiimote/joy', 10)

    def keep(self, store, msg):
        with self.lock:
            store.append(msg)

    def clear(self):
        with self.lock:
            self.wiimote_joy.clear()
            self.plain_joy.clear()
            self.teleop.clear()


def joy(two=False, a=False, b=False, tilt=0.0):
    """What build_joy() publishes for these inputs (default layout)."""
    msg = Joy()
    msg.axes = [0.0, 0.0]
    msg.buttons = [0] * 5
    if two:
        msg.buttons[4] = 1
        msg.buttons[1] = int(a)
        msg.buttons[2] = int(b)
        msg.axes[1] = 1.0 if a and not b else -1.0 if b and not a else 0.0
        msg.axes[0] = tilt
    return msg


def main():
    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  -- ' + detail) if detail else ''}",
              flush=True)

    tmp = Path(tempfile.mkdtemp(prefix='wiimote_e2e_'))
    rclpy.init()
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    print('\n[driver] no Wiimote connected')
    log = open(tmp / 'driver.log', 'w')
    driver = subprocess.Popen(
        ['ros2', 'launch', 'racerbot_wiimote_driver', 'wiimote_driver_launch.py'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(6)
    alive = driver.poll() is None
    with probe.lock:
        frames = list(probe.wiimote_joy)
        plain = len(probe.plain_joy)
    os.killpg(driver.pid, signal.SIGINT)
    driver.wait(timeout=15)
    text = (tmp / 'driver.log').read_text()
    check('driver keeps running without a Wiimote', alive, text[-300:] if not alive else '')
    check('driver tells the operator how to connect one',
          'No Wiimote connected. Press 1+2' in text)
    check('driver publishes "nothing held" on /wiimote/joy', len(frames) > 50 and all(
        not any(f.buttons) and not any(f.axes) for f in frames), f'{len(frames)} messages')
    check('driver publishes nothing on /joy (cannot act as the F710 deadman)', plain == 0)
    check('driver exits cleanly on Ctrl+C', driver.returncode == 0, str(driver.returncode))

    print('\n[teleop] real joy_teleop with wiimote_teleop.yaml')
    log = open(tmp / 'teleop.log', 'w')
    teleop = subprocess.Popen(
        ['ros2', 'run', 'joy_teleop', 'joy_teleop', '--ros-args',
         '--params-file', str(CONFIG / 'wiimote_teleop.yaml'),
         '-r', '__node:=wiimote_teleop', '-r', 'joy:=/wiimote/joy'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(4)

    def send(msg, seconds=0.6):
        probe.clear()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg.header.stamp = probe.get_clock().now().to_msg()
            probe.fake_joy.publish(msg)
            time.sleep(0.02)
        with probe.lock:
            return list(probe.teleop)

    cases = [
        ('"2" + A + tilt left drives forward and steers left', joy(True, a=True, tilt=0.5),
         1.0, 0.17),
        ('"2" + B reverses', joy(True, b=True), -1.0, 0.0),
        ('"2" + A + B is no throttle', joy(True, a=True, b=True), 0.0, 0.0),
        ('"2" alone with tilt right steers only', joy(True, tilt=-1.0), 0.0, -0.34),
        ('nothing held commands an active stop', joy(), 0.0, 0.0),
    ]
    for name, msg, speed, steer in cases:
        out = send(msg)
        last = out[-5:] if out else []
        ok = len(out) >= 5 and all(abs(m.drive.speed - speed) < 1e-6
                                   and abs(m.drive.steering_angle - steer) < 1e-6 for m in last)
        check(name, ok, f'{len(out)} msgs, last speed '
              f'{last[-1].drive.speed if last else None}, steer '
              f'{last[-1].drive.steering_angle if last else None}')
    # Why build_joy() zeroes everything without "2": an unmasked "A held,
    # 2 released" message activates no profile at all, so joy_teleop would
    # go silent and leave the last drive command to the mux timeout.
    unmasked = joy()
    unmasked.buttons[1] = 1
    unmasked.axes[1] = 1.0
    out = send(unmasked)
    check('an unmasked "A without 2" would publish nothing (why the driver masks it)',
          len(out) == 0, f'{len(out)} msgs')
    os.killpg(teleop.pid, signal.SIGINT)
    teleop.wait(timeout=15)

    executor.shutdown(timeout_sec=2.0)
    spinner.join(timeout=2.0)
    probe.destroy_node()
    rclpy.shutdown()
    failed = results.count(False)
    print(f'\n{len(results) - failed} passed, {failed} failed. Logs: {tmp}')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
