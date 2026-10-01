"""
Tests for checks.py -- the verdicts.

Pure Python, no rclpy:  python3 -m pytest src/racerbot_doctor/test/ -v

Oracles:
  * thresholds are racerbot_doctor/car.py's constants, each of which cites
    its source there (vesc_driver.cpp's 50 Hz timer, sensors.yaml, the
    team's 3S battery decision of 2026-10-01). Boundary tests sit exactly
    on and one step past each threshold.
  * the ladder rule and the "bringup down is INFO, partly down is FAIL"
    rule are in docs/superpowers/plans/2026-10-01-racerbot-doctor.md.
Most tests start from a fully healthy car (every layer PASS -- itself
asserted) and break exactly one thing, then assert which layer catches it
and that nothing above it is evaluated. That is the "plugged in vs. driver
loaded vs. data flowing" distinction this package exists for.
"""
import math
import os
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_doctor import car  # noqa: E402
from racerbot_doctor.checks import (  # noqa: E402
    battery_layer, check_bringup, check_camera, check_environment,
    check_joystick, check_lidar, check_vesc, run_all)
from racerbot_doctor.snapshot import (  # noqa: E402
    LiveSample, NetIface, Process, Snapshot, Status, TopicStats, UsbDevice)

P, W, F, I, S = Status.PASS, Status.WARN, Status.FAIL, Status.INFO, Status.SKIP

PIDS = {car.EXE_VESC_DRIVER: 100, car.EXE_URG: 101, car.EXE_JOY: 102,
        car.EXE_MUX: 103, car.EXE_ACK_TO_VESC: 104, car.EXE_VESC_TO_ODOM: 105,
        car.EXE_REALSENSE: 106}


def healthy_snap():
    return Snapshot(
        usb=[UsbDevice('1-2.3', *car.VESC_USB, drivers=['cdc_acm'],
                       ttys=['ttyACM0'], speed_mbps=12),
             UsbDevice('1-2.4', car.JOY_VID, car.JOY_XINPUT_PID,
                       drivers=['xpad'], joysticks=['js0'], speed_mbps=12),
             UsbDevice('2-1.1', *car.CAMERA_USB, drivers=['uvcvideo', 'usbhid'],
                       videos=['video0', 'video1'], speed_mbps=5000)],
        processes=[Process(pid, [f'/ws/lib/x/{exe}', '--ros-args'])
                   for exe, pid in PIDS.items()],
        ifaces={car.LIDAR_IFACE: NetIface(car.LIDAR_IFACE, True,
                                          [car.LIDAR_HOST_IP])},
        vesc_symlink='/dev/ttyACM0', vesc_symlink_rw=True,
        tty_holders={'/dev/ttyACM0': [PIDS[car.EXE_VESC_DRIVER]]},
        js_readable={'js0': True}, lidar_ping=True, lidar_tcp=None,
        env={'ROS_DISTRO': 'jazzy', 'ROS_DOMAIN_ID': '0',
             'AMENT_PREFIX_PATH': '/ws/install/vesc_driver:/ws/install/'
             'ackermann_mux:/ws/install/f1tenth_stack:/opt/ros/jazzy'})


def _ts(hz, last=None, window=3.0):
    return TopicStats(count=int(round(hz * window)), window_s=window, last=last)


def core_msg(fault=0, volts=11.6):
    return NS(state=NS(fault_code=fault, voltage_input=volts))


def scan_msg(ranges):
    return NS(ranges=list(ranges))


def healthy_live():
    return LiveSample(
        topics={'/sensors/core': _ts(50, core_msg()),
                '/scan': _ts(40, scan_msg([1.0] * 1081)),
                '/joy': _ts(20),
                '/odom': _ts(50),
                car.CAMERA_TOPIC: _ts(15)},
        teleop_publishers=0, laser_tf=car.LASER_TF)


def statuses(report):
    return [l.status for l in report.layers]


def layer(report, name_part):
    (hit,) = [l for l in report.layers if name_part in l.name]
    return hit


def first_bad(report):
    """First layer that is not PASS or SKIP. A FAIL/INFO always comes
    before the SKIPs it causes, so skipping SKIP loses nothing."""
    for l in report.layers:
        if l.status not in (P, S):
            return l
    return None


def without(snap, exe):
    snap.processes = [p for p in snap.processes if not p.runs(exe)]
    return snap


# ---------------------------------------------------------------- baseline

@pytest.mark.parametrize('check', [check_vesc, check_lidar, check_joystick,
                                   check_camera, check_bringup,
                                   check_environment])
def test_healthy_car_is_all_pass(check):
    report = check(healthy_snap(), healthy_live())
    assert report.layers, 'a check with no layers proves nothing'
    bad = [(l.name, l.status, l.detail) for l in report.layers
           if l.status is not P and 'not probed' not in l.detail]
    assert bad == []


# -------------------------------------------------------------------- VESC

def test_vesc_unplugged_fails_at_usb_and_blocks_everything_above():
    snap = healthy_snap()
    snap.usb = [d for d in snap.usb if (d.vid, d.pid) != car.VESC_USB]
    r = check_vesc(snap, healthy_live())
    assert r.layers[0].status is F
    assert 'not plugged in' in r.layers[0].detail
    assert set(statuses(r)[1:]) == {S}


def test_vesc_no_kernel_driver_is_not_reported_as_unplugged():
    snap = healthy_snap()
    snap.usb[0].drivers, snap.usb[0].ttys = [], []
    r = check_vesc(snap, healthy_live())
    assert statuses(r)[0] is P
    assert r.layers[1].status is F
    assert 'cdc_acm' in r.layers[1].detail
    assert set(statuses(r)[2:]) == {S}


def test_vesc_tty_from_the_wrong_kernel_driver_fails():
    snap = healthy_snap()
    snap.usb[0].drivers, snap.usb[0].ttys = ['option'], ['ttyUSB0']
    r = check_vesc(snap, healthy_live())
    assert r.layers[1].status is F and 'cdc_acm' in r.layers[1].detail


def test_vesc_symlink_missing():
    snap = healthy_snap()
    snap.vesc_symlink = None
    bad = first_bad(check_vesc(snap, healthy_live()))
    assert bad.status is F and '/dev/sensors/vesc' in bad.detail
    assert '99-vesc.rules' in bad.fix


def test_vesc_symlink_points_elsewhere():
    snap = healthy_snap()
    snap.vesc_symlink = '/dev/ttyACM1'
    bad = first_bad(check_vesc(snap, healthy_live()))
    assert bad.status is F
    assert 'ttyACM1' in bad.detail and 'ttyACM0' in bad.detail


def test_vesc_symlink_not_rw_points_at_dialout():
    snap = healthy_snap()
    snap.vesc_symlink_rw = False
    bad = first_bad(check_vesc(snap, healthy_live()))
    assert bad.status is F and 'dialout' in bad.fix


def test_vesc_tty_held_by_another_process():
    snap = healthy_snap()
    snap.processes.append(Process(900, ['/usr/bin/screen', '/dev/ttyACM0']))
    snap.tty_holders = {'/dev/ttyACM0': [100, 900]}
    bad = first_bad(check_vesc(snap, healthy_live()))
    assert bad.status is F and '900' in bad.detail and 'screen' in bad.detail


def test_vesc_tty_holder_matched_by_basename_across_dev_roots():
    snap = healthy_snap()
    snap.tty_holders = {'/tmp/fake/dev/ttyACM0': [900]}
    snap.processes.append(Process(900, ['minicom']))
    assert first_bad(check_vesc(snap, healthy_live())).status is F


def test_vesc_driver_not_running_with_bringup_down_is_info():
    snap = healthy_snap()
    for exe in car.BRINGUP_EXES:
        without(snap, exe)
    snap.tty_holders = {}
    r = check_vesc(snap, None)
    bad = first_bad(r)
    assert bad.status is I and 'bringup' in bad.detail
    # the hardware layers below it still passed
    assert statuses(r)[:4] == [P, P, P, P]


def test_vesc_driver_missing_while_rest_of_bringup_runs_is_fail():
    snap = without(healthy_snap(), car.EXE_VESC_DRIVER)
    snap.tty_holders = {}
    bad = first_bad(check_vesc(snap, healthy_live()))
    assert bad.status is F
    assert 'crashed' in bad.detail or 'failed to start' in bad.detail


def test_vesc_usb_present_no_telemetry_blames_battery():
    live = healthy_live()
    live.topics['/sensors/core'] = TopicStats(0, 3.0)
    bad = first_bad(check_vesc(healthy_snap(), live))
    assert bad.status is F and 'battery' in bad.detail.lower()


def test_vesc_telemetry_topic_absent_from_sample_is_treated_as_silent():
    live = healthy_live()
    del live.topics['/sensors/core']
    assert first_bad(check_vesc(healthy_snap(), live)).status is F


def test_vesc_core_rate_boundary():
    live = healthy_live()
    live.topics['/sensors/core'] = TopicStats(75, 3.0, core_msg())  # 25.0 Hz
    assert first_bad(check_vesc(healthy_snap(), live)) is None
    live.topics['/sensors/core'] = TopicStats(74, 3.0, core_msg())  # 24.67
    bad = first_bad(check_vesc(healthy_snap(), live))
    assert bad.status is F and '24.7' in bad.detail


def test_vesc_fault_code_named():
    live = healthy_live()
    live.topics['/sensors/core'] = _ts(50, core_msg(fault=2))
    bad = first_bad(check_vesc(healthy_snap(), live))
    assert bad.status is F and 'UNDER_VOLTAGE' in bad.detail


def test_vesc_unknown_fault_code_still_fails():
    live = healthy_live()
    live.topics['/sensors/core'] = _ts(50, core_msg(fault=42))
    bad = first_bad(check_vesc(healthy_snap(), live))
    assert bad.status is F and '42' in bad.detail


def test_vesc_live_stage_not_run_skips_data_layers_without_failing():
    r = check_vesc(healthy_snap(), None)
    assert F not in statuses(r)
    assert statuses(r)[-3:] == [S, S, S]


@pytest.mark.parametrize('volts, want', [
    (0.0, F), (4.99, F), (5.0, F), (10.19, F),     # absent / critically low
    (10.2, W), (10.79, W),                         # low
    (10.8, P), (11.6, P), (12.6, P), (12.75, P),   # healthy 3S
    (12.76, W), (16.8, W),                         # above 3S full: 4S fitted?
    (float('nan'), F), (float('inf'), F), (-1.0, F),
])
def test_battery_thresholds(volts, want):
    assert battery_layer(volts).status is want


def test_battery_absent_message_distinct_from_low():
    assert 'battery' in battery_layer(0.3).detail.lower()
    assert 'unplugged' in battery_layer(0.3).detail.lower() or \
        'off' in battery_layer(0.3).detail.lower()
    assert 'unplugged' not in battery_layer(10.0).detail.lower()


def test_battery_absent_boundary_is_strictly_below_5v():
    # car.BATTERY_ABSENT_V: below 5.0 V the VESC is running on USB power
    # alone; at 5.0 V it is a (dead-flat) battery, which is a different fix.
    assert 'no battery voltage' in battery_layer(4.99).detail
    assert 'no battery voltage' not in battery_layer(5.0).detail
    assert 'critically low' in battery_layer(5.0).detail


def test_battery_reports_volts_and_per_cell():
    d = battery_layer(11.4).detail
    assert '11.40 V' in d and '3.80 V/cell' in d


# ------------------------------------------------------------------- LiDAR

def test_lidar_no_carrier_blames_cable_or_power():
    snap = healthy_snap()
    snap.ifaces[car.LIDAR_IFACE].carrier = False
    r = check_lidar(snap, healthy_live())
    assert r.layers[0].status is F and 'cable' in r.layers[0].detail
    assert set(statuses(r)[1:]) == {S}


def test_lidar_iface_missing():
    snap = healthy_snap()
    snap.ifaces = {}
    assert check_lidar(snap, healthy_live()).layers[0].status is F


def test_lidar_wrong_host_ip_suggests_nmcli():
    snap = healthy_snap()
    snap.ifaces[car.LIDAR_IFACE].ipv4 = ['169.254.3.3/16']
    bad = first_bad(check_lidar(snap, healthy_live()))
    assert bad.status is F and 'nmcli' in bad.fix and '169.254.3.3' in bad.detail


def test_lidar_no_ping():
    snap = healthy_snap()
    snap.lidar_ping = False
    bad = first_bad(check_lidar(snap, healthy_live()))
    assert bad.status is F and car.LIDAR_IP in bad.detail


def test_lidar_ping_unavailable_warns_and_continues():
    snap = healthy_snap()
    snap.lidar_ping = None
    r = check_lidar(snap, healthy_live())
    assert first_bad(r).status is W
    assert F not in statuses(r)


def test_lidar_tcp_refused_with_driver_down():
    snap = without(healthy_snap(), car.EXE_URG)
    snap.lidar_tcp = False
    bad = first_bad(check_lidar(snap, None))
    assert bad.status is F and str(car.LIDAR_PORT) in bad.detail


def test_lidar_tcp_not_probed_while_urg_node_runs_is_not_a_failure():
    snap = healthy_snap()          # urg running, lidar_tcp None
    r = check_lidar(snap, healthy_live())
    tcp = layer(r, 'TCP')
    assert tcp.status is S and 'urg_node' in tcp.detail
    assert r.layers[-1].status is P   # and the layers above still ran


def test_lidar_urg_crashed_while_bringup_up():
    snap = without(healthy_snap(), car.EXE_URG)
    snap.lidar_tcp = True
    bad = first_bad(check_lidar(snap, healthy_live()))
    assert bad.status is F and 'urg_node' in bad.detail


def test_lidar_scan_rate_boundary():
    live = healthy_live()
    live.topics['/scan'] = TopicStats(90, 3.0, scan_msg([1.0] * 10))  # 30 Hz
    assert first_bad(check_lidar(healthy_snap(), live)) is None
    live.topics['/scan'] = TopicStats(89, 3.0, scan_msg([1.0] * 10))
    assert first_bad(check_lidar(healthy_snap(), live)).status is F


@pytest.mark.parametrize('ranges, want', [
    ([], F),
    ([math.inf] * 100, F),
    ([math.nan] * 100, F),
    ([0.0] * 100, F),                      # all-zero: no returns at all
    ([1.0] * 49 + [math.inf] * 51, W),     # 49 % finite
    ([1.0] * 50 + [math.inf] * 50, P),     # exactly 50 %
    ([2.0], P),
])
def test_lidar_scan_content(ranges, want):
    live = healthy_live()
    live.topics['/scan'] = _ts(40, scan_msg(ranges))
    assert check_lidar(healthy_snap(), live).layers[-1].status is want


# ---------------------------------------------------------------- joystick

def test_joystick_receiver_missing():
    snap = healthy_snap()
    snap.usb = [d for d in snap.usb if d.vid != car.JOY_VID]
    r = check_joystick(snap, healthy_live())
    assert r.layers[0].status is F and 'not plugged in' in r.layers[0].detail


def test_joystick_directinput_mode_says_flip_the_switch():
    snap = healthy_snap()
    pad = [d for d in snap.usb if d.vid == car.JOY_VID][0]
    pad.pid, pad.drivers, pad.joysticks = car.JOY_DIRECTINPUT_PID, [], []
    r = check_joystick(snap, healthy_live())
    assert r.layers[0].status is F
    assert 'DirectInput' in r.layers[0].detail
    assert 'X' in r.layers[0].fix and 'switch' in r.layers[0].fix


def test_joystick_no_js_node():
    snap = healthy_snap()
    [d for d in snap.usb if d.vid == car.JOY_VID][0].joysticks = []
    bad = first_bad(check_joystick(snap, healthy_live()))
    assert bad.status is F and 'js' in bad.detail


def test_joystick_not_readable_points_at_input_group():
    snap = healthy_snap()
    snap.js_readable = {'js0': False}
    bad = first_bad(check_joystick(snap, healthy_live()))
    assert bad.status is F and 'input' in bad.fix


def test_joy_rate_boundary():
    live = healthy_live()
    live.topics['/joy'] = TopicStats(15, 3.0)        # 5.0 Hz
    assert first_bad(check_joystick(healthy_snap(), live)) is None
    live.topics['/joy'] = TopicStats(14, 3.0)
    assert first_bad(check_joystick(healthy_snap(), live)).status is F


# ------------------------------------------------------------------ camera

def test_camera_missing_is_warn_not_fail():
    snap = healthy_snap()
    snap.usb = [d for d in snap.usb if (d.vid, d.pid) != car.CAMERA_USB]
    r = check_camera(snap, healthy_live())
    assert r.layers[0].status is W and 'not plugged in' in r.layers[0].detail
    assert set(statuses(r)[1:]) == {S}       # still blocks the layers above
    assert r.worst() is W


def test_camera_on_usb2_warns_and_continues():
    snap = healthy_snap()
    [d for d in snap.usb if (d.vid, d.pid) == car.CAMERA_USB][0].speed_mbps = 480
    r = check_camera(snap, healthy_live())
    assert layer(r, 'USB speed').status is W
    assert 'USB 2' in layer(r, 'USB speed').detail
    assert r.layers[-1].status is P


def test_camera_speed_boundary():
    snap = healthy_snap()
    cam = [d for d in snap.usb if (d.vid, d.pid) == car.CAMERA_USB][0]
    cam.speed_mbps = 5000
    assert layer(check_camera(snap, None), 'USB speed').status is P
    cam.speed_mbps = 4999
    assert layer(check_camera(snap, None), 'USB speed').status is W


def test_camera_no_uvc_is_warn():
    snap = healthy_snap()
    cam = [d for d in snap.usb if (d.vid, d.pid) == car.CAMERA_USB][0]
    cam.drivers, cam.videos = ['usbhid'], []
    bad = first_bad(check_camera(snap, healthy_live()))
    assert bad.status is W and 'uvcvideo' in bad.detail


def test_camera_not_launched_is_info_even_with_bringup_up():
    snap = without(healthy_snap(), car.EXE_REALSENSE)
    bad = first_bad(check_camera(snap, healthy_live()))
    assert bad.status is I and 'realsense_camera_launch.py' in bad.fix


def test_camera_launched_but_no_frames_is_warn():
    live = healthy_live()
    live.topics[car.CAMERA_TOPIC] = TopicStats(0, 3.0)
    r = check_camera(healthy_snap(), live)
    assert r.layers[-1].status is W and r.worst() is W


# ----------------------------------------------------------------- bringup

def test_bringup_down_is_single_info():
    snap = healthy_snap()
    for exe in car.BRINGUP_EXES:
        without(snap, exe)
    r = check_bringup(snap, None)
    assert statuses(r) == [I]
    assert 'bringup_launch.py' in r.layers[0].fix


def test_bringup_mux_missing_fails():
    r = check_bringup(without(healthy_snap(), car.EXE_MUX), healthy_live())
    assert layer(r, car.EXE_MUX).status is F


def test_bringup_odom_rate_and_gating():
    live = healthy_live()
    live.topics['/odom'] = TopicStats(74, 3.0)
    assert layer(check_bringup(healthy_snap(), live), '/odom').status is F
    r = check_bringup(without(healthy_snap(), car.EXE_VESC_TO_ODOM), live)
    assert layer(r, car.EXE_VESC_TO_ODOM).status is F
    assert layer(r, '/odom').status is S


def test_bringup_odom_silent_because_vesc_silent_points_at_the_vesc():
    # vesc_to_odom_node derives /odom from /sensors/core (bringup_launch.py),
    # so a silent /odom with a silent /sensors/core is the VESC's problem.
    live = healthy_live()
    live.topics['/odom'] = TopicStats(0, 3.0)
    live.topics['/sensors/core'] = TopicStats(0, 3.0)
    o = layer(check_bringup(healthy_snap(), live), '/odom')
    assert o.status is F and 'fix the VESC first' in o.detail
    live.topics['/sensors/core'] = _ts(50, core_msg())
    o = layer(check_bringup(healthy_snap(), live), '/odom')
    assert o.status is F and 'VESC' not in o.detail


@pytest.mark.parametrize('tf, want', [
    (car.LASER_TF, P),
    ((0.2609, 0.0, 0.11), P),            # 0.9 mm
    ((0.2611, 0.0, 0.11), F),            # 1.1 mm
    ((0.33, 0.0, 0.11), F),              # the pre-2026-08-24 offset
    ((0.26, 0.0, 0.0), F),
    (None, F),
])
def test_bringup_laser_tf(tf, want):
    live = healthy_live()
    live.laser_tf = tf
    assert layer(check_bringup(healthy_snap(), live), 'laser').status is want


def test_bringup_teleop_publisher_is_reported():
    live = healthy_live()
    live.teleop_publishers = 1
    t = layer(check_bringup(healthy_snap(), live), 'teleop')
    assert t.status is I and '/drive' in t.detail


# ------------------------------------------------------------- environment

def test_env_not_sourced():
    snap = healthy_snap()
    snap.env = {}
    r = check_environment(snap, None)
    assert layer(r, 'ROS sourced').status is F
    assert layer(r, 'workspace').status is F


def test_env_wrong_distro():
    snap = healthy_snap()
    snap.env['ROS_DISTRO'] = 'humble'
    assert layer(check_environment(snap, None), 'ROS sourced').status is F


def test_env_workspace_missing_one_package():
    snap = healthy_snap()
    snap.env['AMENT_PREFIX_PATH'] = '/ws/install/vesc_driver:/opt/ros/jazzy'
    w = layer(check_environment(snap, None), 'workspace')
    assert w.status is F and 'ackermann_mux' in w.detail


def test_env_test_domain_warns():
    snap = healthy_snap()
    snap.env['ROS_DOMAIN_ID'] = car.TEST_DOMAIN_ID
    assert layer(check_environment(snap, None), 'domain').status is W


def test_env_sim_beside_real_drivers_fails():
    snap = healthy_snap()
    snap.processes.append(Process(700, ['/usr/bin/python3',
                                        '/ws/lib/racerbot_sim/gym_bridge_node']))
    assert layer(check_environment(snap, None), 'sim').status is F


def test_env_sim_alone_warns():
    snap = healthy_snap()
    for exe in car.BRINGUP_EXES:
        without(snap, exe)
    snap.processes.append(Process(700, ['sim_joy_node']))
    assert layer(check_environment(snap, None), 'sim').status is W


# ----------------------------------------------------------------- run_all

def test_run_all_covers_every_device_in_order():
    names = [r.device for r in run_all(healthy_snap(), healthy_live())]
    assert names == ['Environment', 'VESC', 'LiDAR', 'Joystick',
                     'Camera', 'Bringup']
