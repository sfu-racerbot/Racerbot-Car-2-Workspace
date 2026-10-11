#!/usr/bin/env python3
"""End-to-end test of the odometry/steering calibration wizard.

Runs the car's real software chain and fakes only the hardware:

    scripted F710 (/joy) -> joy_teleop -> ackermann_mux -> /ackermann_cmd
      -> ackermann_to_vesc_node -> /commands/servo/position, /commands/motor/speed
      -> FAKE vesc_driver_node (clips the servo exactly like vesc_driver,
         simulates a car whose true linkage and wheel scale are NOT the
         configured ones, publishes /sensors/core and the clipped servo)
      -> vesc_to_odom_node -> /odom
    odom_calibration_node (its real launch file) watches all of it.

The harness then operates the wizard exactly as a person would, over its
HTTP API, and "tape-measures" each run from the simulated car's true path.
It checks that the wizard recovers the simulated car's true values, that
the hardware/software diagnosis names the right cause, that writing to
vesc.yaml (a temporary copy) and restarting bringup are noticed, and that
the page, its WebSocket and its API all answer.

Run from a sourced, built workspace (never on the car's ROS graph):

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 tools/odom_calibration_e2e/run_e2e.py

It exits non-zero if any check fails. Nothing here publishes /drive; the
only drive commands are the scripted remote's /teleop, which reach the
FAKE VESC. Do not run it with the real driver stack up.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
import yaml

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / 'src' / 'f1tenth_system' / 'f1tenth_stack' / 'config'
WIZARD_CONFIG = REPO / 'src' / 'odom_calibration' / 'config' / 'odom_calibration.yaml'

# The simulated car. Deliberately different from vesc.yaml so a wizard that
# echoes the configured values fails.
TRUE_SPEED_GAIN = 4500.0          # ERPM per m/s the wheels really turn at
TRUE_STEER_GAIN = -1.10           # servo units per rad of real wheel angle
TRUE_STEER_OFFSET = 0.575         # servo value where the wheels point straight
WHEELBASE = 0.36
REAR_TRACK = 0.25
PHONE_ERROR_DEG = 0.0


class FakeCar(Node):
    """vesc_driver_node stand-in plus the F710 remote and the car's motion."""

    def __init__(self, servo_limits):
        super().__init__('vesc_driver_node')
        # Read live by the wizard, exactly like the real driver's CommandLimit.
        self.declare_parameter('servo_min', servo_limits[0])
        self.declare_parameter('servo_max', servo_limits[1])
        self.lock = threading.Lock()
        self.erpm = 0.0
        self.servo = None
        self.x = self.y = self.th = 0.0
        self.stop_left = None      # mechanical stop (rad), None = none
        self.stop_right = None
        self.joy_axes = [0.0] * 8
        self.joy_buttons = [0] * 12
        self.wheel_angle = 0.0
        self.core_pub = self.create_publisher(VescStateStamped, 'sensors/core', 10)
        self.servo_pub = self.create_publisher(Float64, 'sensors/servo_position_command', 10)
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.create_subscription(Float64, 'commands/servo/position', self.on_servo, 10)
        self.create_subscription(Float64, 'commands/motor/speed', self.on_erpm, 10)
        self.create_timer(0.005, self.step)
        self.create_timer(0.02, self.publish_state)
        self.create_timer(0.02, self.publish_joy)
        self.last = time.monotonic()

    def limits(self):
        return (self.get_parameter('servo_min').value, self.get_parameter('servo_max').value)

    def on_servo(self, msg):
        low, high = self.limits()
        clipped = min(max(msg.data, low), high)
        with self.lock:
            self.servo = clipped
        out = Float64()
        out.data = clipped
        self.servo_pub.publish(out)

    def on_erpm(self, msg):
        with self.lock:
            self.erpm = msg.data

    def true_wheel_angle(self, servo):
        angle = (servo - TRUE_STEER_OFFSET) / TRUE_STEER_GAIN
        if self.stop_left is not None:
            angle = min(angle, self.stop_left)
        if self.stop_right is not None:
            angle = max(angle, self.stop_right)
        return angle

    def step(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        with self.lock:
            if self.servo is None:
                return
            speed = self.erpm / TRUE_SPEED_GAIN
            angle = self.true_wheel_angle(self.servo)
            self.wheel_angle = angle
            self.x += speed * math.cos(self.th) * dt
            self.y += speed * math.sin(self.th) * dt
            self.th += speed * math.tan(angle) / WHEELBASE * dt

    def publish_state(self):
        msg = VescStateStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            # This car's VESC reports ERPM with the commanded sign (vesc.yaml).
            msg.state.speed = self.erpm
        self.core_pub.publish(msg)

    def publish_joy(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.axes = list(self.joy_axes)
            msg.buttons = list(self.joy_buttons)
        self.joy_pub.publish(msg)

    # --- operator ---------------------------------------------------------

    def remote(self, lb, throttle=0.0, steer=0.0):
        with self.lock:
            self.joy_buttons[4] = 1 if lb else 0
            self.joy_axes[1] = throttle
            self.joy_axes[3] = steer

    def reset_pose(self):
        with self.lock:
            self.x = self.y = self.th = 0.0

    def pose(self):
        with self.lock:
            return self.x, self.y, self.th

    def tires(self):
        x, y, th = self.pose()
        half = REAR_TRACK / 2.0
        left = (x - half * math.sin(th), y + half * math.cos(th))
        right = (x + half * math.sin(th), y - half * math.cos(th))
        return left, right


class Checks:
    def __init__(self):
        self.results = []

    def __call__(self, name, ok, detail=''):
        self.results.append((name, bool(ok), detail))
        print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  -- ' + detail) if detail else ''}",
              flush=True)
        return ok

    def failed(self):
        return [r for r in self.results if not r[1]]


class Wizard:
    def __init__(self, port):
        self.base = f'http://127.0.0.1:{port}'

    def state(self):
        with urllib.request.urlopen(self.base + '/api/state', timeout=5) as response:
            return json.loads(response.read())

    def raw(self, path, body=None, content_type='application/json'):
        request = urllib.request.Request(
            self.base + path, data=body,
            headers={'Content-Type': content_type} if body is not None else {})
        try:
            with urllib.request.urlopen(request, timeout=10) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()

    def act(self, action, expect_ok=True, **payload):
        status, body = self.raw('/api/action', json.dumps({'action': action, **payload}).encode())
        result = json.loads(body)
        if expect_ok and not result.get('ok'):
            raise RuntimeError(f'{action} failed ({status}): {result.get("error")}')
        return result


def wait_for(predicate, timeout, interval=0.2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            value = predicate()
            if value:
                return value
        except Exception:  # noqa: BLE001 - polling a service that is starting
            pass
        time.sleep(interval)
    return None


class Stack:
    """Real car nodes as child processes, restartable like bringup."""

    def __init__(self, vesc_yaml, wizard_yaml, log_dir):
        self.vesc_yaml = vesc_yaml
        self.wizard_yaml = wizard_yaml
        self.log_dir = log_dir
        self.procs = {}

    def start(self, name, command):
        log = open(self.log_dir / f'{name}.log', 'a')
        self.procs[name] = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def bringup(self):
        vesc = ['--ros-args', '--params-file', str(self.vesc_yaml)]
        self.start('ackermann_to_vesc', ['ros2', 'run', 'vesc_ackermann', 'ackermann_to_vesc_node',
                                         *vesc, '-r', '__node:=ackermann_to_vesc_node'])
        self.start('vesc_to_odom', ['ros2', 'run', 'vesc_ackermann', 'vesc_to_odom_node',
                                    *vesc, '-r', '__node:=vesc_to_odom_node'])

    def all(self):
        self.bringup()
        self.start('ackermann_mux', ['ros2', 'run', 'ackermann_mux', 'ackermann_mux', '--ros-args',
                                     '--params-file', str(CONFIG / 'mux.yaml'),
                                     '-r', '__node:=ackermann_mux'])
        self.start('joy_teleop', ['ros2', 'run', 'joy_teleop', 'joy_teleop', '--ros-args',
                                  '--params-file', str(CONFIG / 'joy_teleop.yaml'),
                                  '-r', '__node:=joy_teleop'])
        self.start('wizard', ['ros2', 'launch', 'odom_calibration', 'odom_calibration_launch.py',
                              f'config:={self.wizard_yaml}'])

    def stop(self, *names):
        for name in names or list(self.procs):
            proc = self.procs.pop(name, None)
            if proc and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)

    def restart_bringup(self):
        self.stop('ackermann_to_vesc', 'vesc_to_odom')
        self.bringup()


def record(wizard, car, kind, drive):
    """Start a capture, run the scripted operator, stop it, return the pending run."""
    # Like an operator, wait for the topic list to go green first (after a
    # bringup restart vesc_to_odom_node needs a moment before /odom flows).
    topic = 'servo' if kind == 'steering_sweep' else 'odom'
    if not wait_for(lambda: wizard.state()['health'][topic]['status'] == 'good', 30):
        raise RuntimeError(f'{topic} never became healthy before {kind}')
    wizard.act('start_capture', kind=kind)
    car.remote(False)
    time.sleep(1.5)          # the operator walks back to the car: idle time
    drive()
    car.remote(False)
    time.sleep(1.5)          # and walks to the laptop again
    wizard.act('stop_capture')
    return wizard.state()['session']['pending_capture']


def drive_until(car, throttle, steer, done, timeout=30.0):
    car.remote(True, throttle, steer)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not done():
        time.sleep(0.01)
    car.remote(True, 0.0, steer)
    time.sleep(0.4)          # coast to a stop with LB still held


def straight_run(car, distance):
    car.reset_pose()
    drive_until(car, 0.2, 0.0, lambda: math.hypot(*car.pose()[:2]) >= distance)
    x, y, _ = car.pose()
    return x, y


def half_circle(wizard, car, side):
    steer = 1.0 if side == 'left' else -1.0
    car.reset_pose()
    start_left, start_right = car.tires()
    pending = record(wizard, car, f'steering_{side}', lambda: drive_until(
        car, 0.15, steer, lambda: abs(car.pose()[2]) >= math.pi, timeout=40.0))
    end_left, end_right = car.tires()
    inner_start, inner_end = (start_left, end_left) if side == 'left' else (start_right, end_right)
    outer_start, outer_end = (start_right, end_right) if side == 'left' else (start_left, end_left)
    inner = math.dist(inner_start, inner_end)
    outer = math.dist(outer_start, outer_end)
    return pending, inner, outer, car.pose()[2]


def sweep(wizard, car):
    angles = {}

    def operate():
        for side, steer in (('left', 1.0), ('right', -1.0)):
            car.remote(True, 0.0, 0.0)
            time.sleep(1.0)
            car.remote(True, 0.0, steer)
            time.sleep(3.0)
            angles[side] = abs(math.degrees(car.wheel_angle))
        car.remote(True, 0.0, 0.0)
        time.sleep(1.0)
    pending = record(wizard, car, 'steering_sweep', operate)
    return pending, angles


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--port', type=int, default=8095)
    parser.add_argument('--keep', action='store_true', help='keep the temp directory')
    parser.add_argument('--browser', metavar='CHROMIUM',
                        help='also click through a session in this Chromium '
                             '(needs `pip install playwright`)')
    args = parser.parse_args()

    tmp = Path(tempfile.mkdtemp(prefix='odom_cal_e2e_'))
    vesc_yaml = tmp / 'vesc.yaml'
    shutil.copy(CONFIG / 'vesc.yaml', vesc_yaml)
    original_vesc = vesc_yaml.read_text()
    wizard_cfg = yaml.safe_load(WIZARD_CONFIG.read_text())
    params = wizard_cfg['odom_calibration_node']['ros__parameters']
    params.update({'port': args.port, 'vesc_yaml_path': str(vesc_yaml),
                   'report_directory': str(tmp / 'reports')})
    wizard_yaml = tmp / 'odom_calibration.yaml'
    wizard_yaml.write_text(yaml.safe_dump(wizard_cfg))
    shared = yaml.safe_load(original_vesc)['/**']['ros__parameters']

    check = Checks()
    rclpy.init()
    car = FakeCar((shared['servo_min'], shared['servo_max']))
    executor = MultiThreadedExecutor()
    executor.add_node(car)
    threading.Thread(target=executor.spin, daemon=True).start()
    stack = Stack(vesc_yaml, wizard_yaml, tmp)
    stack.all()
    wizard = Wizard(args.port)
    try:
        run(check, wizard, car, stack, vesc_yaml, original_vesc, shared, tmp)
        if args.browser:
            # Start the page test from the car's real, off-centre offset.
            vesc_yaml.write_text(original_vesc)
            stack.restart_bringup()
            wait_for(lambda: abs(wizard.state()['live_parameters'][
                'steering_angle_to_servo_offset'] - shared['steering_angle_to_servo_offset']) < 1e-9, 30)
            browser_session(check, wizard, car, stack, vesc_yaml, args.browser, tmp)
    except Exception as exc:  # noqa: BLE001 - report, then fail the run
        check('scenario ran to the end', False, repr(exc))
    finally:
        stack.stop()
        executor.shutdown()
        car.destroy_node()
        rclpy.shutdown()
    failed = check.failed()
    print(f'\n{len(check.results) - len(failed)} passed, {len(failed)} failed. Logs: {tmp}')
    if not args.keep and not failed:
        shutil.rmtree(tmp, ignore_errors=True)
    sys.exit(1 if failed else 0)


def run(check, wizard, car, stack, vesc_yaml, original_vesc, shared, tmp):
    print('\n[connect] wizard, topics and live parameters')
    snap = wait_for(lambda: (lambda s: s if all(
        v == 'read' for v in s['live_parameter_sources'].values()) else None)(wizard.state()), 60)
    check('wizard reads live values from all three nodes', snap,
          json.dumps(snap and snap['live_parameter_sources']))
    check('live steering offset is the one in vesc.yaml',
          snap and abs(snap['live_parameters']['steering_angle_to_servo_offset']
                       - shared['steering_angle_to_servo_offset']) < 1e-9)
    check('live servo limits come from vesc_driver_node',
          snap and snap['live_servo_limits'] == [shared['servo_min'], shared['servo_max']])
    car.remote(True)
    time.sleep(1.0)
    car.remote(False)
    health = wait_for(lambda: (lambda h: h if all(
        h[name]['status'] == 'good' for name in h) else None)(wizard.state()['health']), 20)
    check('every topic is healthy (odom, VESC, servo before/after clamp, drive, joy)', health,
          json.dumps({k: v['status'] for k, v in (health or wizard.state()['health']).items()}))
    status, body = wizard.raw('/')
    check('page is served', status == 200)
    status, body = wizard.raw('/wizard.js')
    check('wizard.js is served', status == 200 and b'steering_sweep' in body)
    status, body = wizard.raw('/api/action', b'{"action":"generate_report"}', 'text/plain')
    check('a non-JSON (cross-site style) request is refused', status == 415, str(status))
    check('the WebSocket streams snapshots', websocket_snapshot(wizard.base))

    print('\n[steering 1] range check against a car with a mechanical stop on the left')
    wizard.act('new_session', mode='steering', replace_existing=True)
    car.stop_left = 0.30
    pending, angles = sweep(wizard, car)
    sides = pending['summary']['sweep']['sides']
    check('range check saw a full-left and a full-right hold',
          sides['left']['samples'] >= 5 and sides['right']['samples'] >= 5)
    wizard.act('accept_capture', confirmed=True,
               left_lock={'left_wheel_deg': angles['left'], 'right_wheel_deg': angles['left']},
               right_lock={'left_wheel_deg': angles['right'], 'right_wheel_deg': angles['right']})
    chain = wizard.state()['steering_chain']
    layers = {layer['id']: layer for layer in chain['layers']}
    gain, offset = shared['steering_angle_to_servo_gain'], shared['steering_angle_to_servo_offset']
    right_reach = math.degrees((shared['servo_max'] - offset) / gain)
    check('servo_max clamp found on the right',
          layers['clamp']['status'] == 'limited' and 'right is cut' in layers['clamp']['finding'],
          layers['clamp']['finding'])
    check('commanded right angle matches the clamp closed form',
          layers['commanded']['right_text'] == f'{abs(right_reach):.1f} deg',
          f"{layers['commanded']['right_text']} vs {abs(right_reach):.1f}")
    check('verdict: software and hardware both contribute', chain['verdict'] == 'both',
          f"{chain['verdict']}: {chain['summary']}")

    print('\n[steering 2] full calibration of a car with no mechanical stop')
    car.stop_left = None
    wizard.act('new_session', mode='steering', replace_existing=True)
    pending, angles = sweep(wizard, car)
    wizard.act('accept_capture', confirmed=True,
               left_lock={'left_wheel_deg': angles['left']},
               right_lock={'left_wheel_deg': angles['right']})
    chain = wizard.state()['steering_chain']
    check('without a stop the diagnosis is software (the clamp)',
          chain['verdict'] == 'software', chain['summary'])

    drift_angles = []
    for attempt in (1, 2):
        result = {}
        pending = record(wizard, car, 'steering_drift',
                         lambda: result.update(zip('xy', straight_run(car, 5.0))))
        wizard.act('accept_capture', confirmed=True,
                   measured_forward_m=result['x'], measured_lateral_m=result['y'])
        centre = wizard.state()['centre_offset']
        drift_angles.append(abs(centre['drift_angle_rad']))
        print(f"    drift {attempt}: {result['y'] * 100:+.1f} cm over {result['x']:.2f} m, "
              f"suggested offset {centre['suggested_offset']:.4f}")
        if attempt == 1:
            check('drift run used only LB-held, moving samples',
                  pending['summary']['servo_active']['count'] >= 10
                  and pending['summary']['servo']['count']
                  > pending['summary']['servo_active']['count'])
            check('suggested centre offset is near the true straight-ahead servo',
                  abs(centre['suggested_offset'] - TRUE_STEER_OFFSET) < 0.005,
                  f"{centre['suggested_offset']:.4f} vs true {TRUE_STEER_OFFSET}")
            applied = wizard.act('apply_parameters', confirmed=True, source='centre')['applied']
            written = yaml.safe_load(vesc_yaml.read_text())['/**']['ros__parameters']
            check('offset written to vesc.yaml, comments kept',
                  abs(written['steering_angle_to_servo_offset'] - centre['suggested_offset']) < 1e-6
                  and vesc_yaml.read_text().count('#') == original_vesc.count('#'))
            check('backup holds the previous vesc.yaml',
                  Path(applied['backup']).read_text() == original_vesc)
            check('page asks for a bringup restart',
                  wizard.state()['pending_restart'] == ['steering_angle_to_servo_offset'])
            stack.restart_bringup()
            synced = wait_for(lambda: not wizard.state()['pending_restart'], 30)
            check('restarted bringup is noticed and the session follows it', synced and abs(
                wizard.state()['session']['current_parameters']['steering_angle_to_servo_offset']
                - centre['suggested_offset']) < 1e-6)
            car.remote(True)
            time.sleep(0.5)
    check('second drift run is straighter after the offset change',
          drift_angles[1] < drift_angles[0] / 3,
          f'{math.degrees(drift_angles[0]):.3f} -> {math.degrees(drift_angles[1]):.3f} deg')

    for side in ('left', 'right'):
        pending, inner, outer, heading = half_circle(wizard, car, side)
        print(f'    {side} circle: inner {inner:.3f} m, outer {outer:.3f} m, '
              f'heading {math.degrees(heading):.0f} deg')
        wizard.act('accept_capture', confirmed=True, measurement_method='rear_tires',
                   full_lock=True, measured_inner_diameter_m=inner,
                   measured_outer_diameter_m=outer)
    wizard.act('generate_report')
    report = wizard.state()['session']['report']
    steering = report['steering']
    print(f"    fitted gain {steering['suggested_steering_angle_to_servo_gain']:.4f}, "
          f"offset {steering['suggested_steering_angle_to_servo_offset']:.4f}, "
          f"status {steering['status']}, rmse {steering['fit_rmse_servo']}")
    check('fitted steering gain recovers the true linkage (within 3%)',
          abs(steering['suggested_steering_angle_to_servo_gain'] / TRUE_STEER_GAIN - 1) < 0.03)
    check('fitted steering offset recovers the true centre (within 0.005)',
          abs(steering['suggested_steering_angle_to_servo_offset'] - TRUE_STEER_OFFSET) < 0.005)
    check('nothing is blocked and the steering status is good',
          not report['blocked_suggestions'] and steering['status'] == 'good',
          json.dumps(report['blocked_suggestions']) + ' ' + json.dumps(steering['warnings']))
    check('the circles are not flagged as short arcs or bad radii', not any(
        'turned only' in w or 'no steered circle' in w
        for item in steering['trial_results'] for w in item['warnings']))
    status, body = wizard.raw('/api/report/md')
    check('Markdown report downloads with the diagnosis', status == 200
          and b'Steering range: hardware or software?' in body)
    before = vesc_yaml.read_text()
    names = [n for n in ('steering_angle_to_servo_gain', 'steering_angle_to_servo_offset')
             if n not in report['not_ready_to_apply']]
    wizard.act('apply_parameters', confirmed=True, source='report', names=names)
    written = yaml.safe_load(vesc_yaml.read_text())['/**']['ros__parameters']
    check('report values written to vesc.yaml', all(
        abs(written[n] - report['parameter_suggestions'][n]) < 1e-6 for n in names), str(names))
    wizard.act('restore_backup', confirmed=True)
    check('undo restores the file exactly', vesc_yaml.read_text() == before)
    stack.restart_bringup()
    wait_for(lambda: not wizard.state()['pending_restart'], 30)

    print('\n[wheels] stationary baseline and three tape runs')
    wizard.act('new_session', mode='movement', replace_existing=True)
    record(wizard, car, 'stationary', lambda: time.sleep(5.0))
    wizard.act('accept_capture', confirmed=True)
    for direction, throttle in (('forward', 0.2), ('forward', 0.2), ('reverse', -0.2)):
        car.reset_pose()
        record(wizard, car, 'movement', lambda: drive_until(
            car, throttle, 0.0, lambda: abs(car.pose()[0]) >= 5.0))
        wizard.act('accept_capture', confirmed=True, direction=direction,
                   measured_distance_m=abs(car.pose()[0]))
    wizard.act('generate_report')
    report = wizard.state()['session']['report']
    movement = report['movement']
    print(f"    gain {movement['suggested_speed_to_erpm_gain']:.1f}, status {movement['status']}")
    check('wheel gain recovers the true scale with the odometry sign (within 1%)',
          abs(movement['suggested_speed_to_erpm_gain'] / -TRUE_SPEED_GAIN - 1) < 0.01)
    check('wheel calibration confidence is high', movement['status'] == 'high',
          json.dumps(movement['warnings']))
    wizard.act('apply_parameters', confirmed=True, source='report',
               names=['speed_to_erpm_gain', 'speed_to_erpm_offset'])
    document = yaml.safe_load(vesc_yaml.read_text())
    check('odometry gain written only to vesc_to_odom_node',
          abs(document['vesc_to_odom_node']['ros__parameters']['speed_to_erpm_gain']
              - movement['suggested_speed_to_erpm_gain']) < 1e-6
          and document['/**']['ros__parameters']['speed_to_erpm_gain'] == 4614.0)
    stack.restart_bringup()
    synced = wait_for(lambda: not wizard.state()['pending_restart'], 30)
    check('vesc_to_odom_node runs the new gain after restart', synced)


def browser_session(check, wizard, car, stack, vesc_yaml, chromium, tmp):
    """Operate the real page in Chromium: every click a person makes."""
    from playwright.sync_api import sync_playwright

    print('\n[browser] steering session driven through the page')
    errors = []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(executable_path=chromium)
        page = browser.new_page(viewport={'width': 1280, 'height': 900})
        page.on('pageerror', lambda exc: errors.append(f'pageerror: {exc}'))
        page.on('console', lambda msg: msg.type == 'error' and errors.append(msg.text))
        page.on('dialog', lambda dialog: dialog.accept())
        view = page.locator('#wizard-view')

        def click(text):
            page.get_by_role('button', name=text, exact=True).first.click()

        page.goto(wizard.base + '/')
        page.wait_for_selector('#wizard-view h1')
        click('Change calibration')
        page.check('input[name="calibration-mode"][value="steering"]')
        click('Start calibration')
        page.wait_for_selector('text=Check the car')
        check('page: new steering session starts on the preflight check',
              'Check the car' in view.text_content())
        car.remote(True)
        time.sleep(0.5)
        car.remote(False)
        click('Start step 2: Steering range')
        page.wait_for_selector('#wizard-view h1:has-text("Steering range")')

        click('Record')
        page.wait_for_selector('text=Recording: Steering range')
        car.remote(True, 0.0, -1.0)
        time.sleep(3.0)
        live = page.inner_text('#chain-servo')
        check('page: live chain shows the clipped servo during the hold', live == '0.8500', live)
        car.remote(True, 0.0, 1.0)
        time.sleep(3.0)
        car.remote(False)
        click('Stop')
        page.wait_for_selector('#sweep-ll-left')
        page.fill('#sweep-ll-left', f'{abs(math.degrees(car.true_wheel_angle(0.1778))):.1f}')
        page.fill('#sweep-rl-left', f'{abs(math.degrees(car.true_wheel_angle(0.85))):.1f}')
        page.check('#trial-confirmed')
        click('Accept run')
        page.wait_for_selector('text=Cause:')
        card = view.text_content()
        check('page: diagnosis card names the clamp', 'Cause: Software' in card
              and 'right is cut from 1.003 to 0.850 by servo_max' in card, card[:300])

        click('Next: Drive straight')
        page.wait_for_selector('#wizard-view h1:has-text("Drive straight")')
        click('Record')
        page.wait_for_selector('text=Recording: Drive straight')
        x, y = straight_run(car, 5.0)
        car.remote(False)
        time.sleep(1.0)
        click('Stop')
        page.wait_for_selector('#drift-forward')
        page.fill('#drift-forward', f'{x:.3f}')
        page.fill('#drift-side-mag', f'{abs(y):.3f}')
        page.check(f'input[name="drift-side"][value="{"left" if y > 0 else "right"}"]')
        page.check('#trial-confirmed')
        click('Accept run')
        page.wait_for_selector('text=Centre offset')
        before = vesc_yaml.read_text()
        click('Write this offset to vesc.yaml')
        page.wait_for_selector('text=Restart bringup.')
        check('page: writing the offset changes vesc.yaml and asks for a restart',
              vesc_yaml.read_text() != before)
        stack.restart_bringup()
        car.remote(True)
        cleared = wait_for(lambda: page.locator('text=Restart bringup.').count() == 0, 40)
        car.remote(False)
        check('page: restart banner clears once bringup runs the new value', cleared)
        page.screenshot(path=str(tmp / 'browser_final.png'), full_page=True)
        browser.close()
    check('page: no JavaScript errors', not errors, '; '.join(errors[:3]))


def websocket_snapshot(base):
    import asyncio
    from tornado.websocket import websocket_connect

    async def fetch():
        connection = await websocket_connect(base.replace('http', 'ws') + '/ws')
        message = await asyncio.wait_for(connection.read_message(), 5)
        connection.close()
        return json.loads(message)
    try:
        return asyncio.run(fetch()).get('type') == 'snapshot'
    except Exception:  # noqa: BLE001
        return False


if __name__ == '__main__':
    main()
