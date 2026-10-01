"""Verdicts: Snapshot (+ LiveSample, or None if the live stage didn't run)
-> one DeviceReport per device. Pure functions, no I/O, no rclpy.

Each device is a ladder checked bottom-up -- plugged in, kernel driver,
permissions, ROS driver process, data rate, data content -- so the report
names the *first* broken layer instead of a vague "VESC broken".
"""
import math
import os

from . import car
from .snapshot import DeviceReport, Layer, Status, ladder

P, W, F, I, S = Status.PASS, Status.WARN, Status.FAIL, Status.INFO, Status.SKIP

BRINGUP_FIX = 'ros2 launch f1tenth_stack bringup_launch.py'
NO_LIVE = 'live stage not run (ROS not sourced, or no driver running)'

# vesc_msgs/msg/VescState.msg
FAULT_NAMES = {1: 'OVER_VOLTAGE', 2: 'UNDER_VOLTAGE', 3: 'DRV8302',
               4: 'ABS_OVER_CURRENT', 5: 'OVER_TEMP_FET', 6: 'OVER_TEMP_MOTOR'}


# ----------------------------------------------------------------- helpers

def _usb(snap, vid, pid=None):
    return [d for d in snap.usb if d.vid == vid and (pid is None or d.pid == pid)]


def _pids(snap, exe):
    return [p.pid for p in snap.processes if p.runs(exe)]


def bringup_up(snap):
    return any(_pids(snap, exe) for exe in car.BRINGUP_EXES)


def _process_layer(snap, exe, *, optional_fix=None):
    """Is a ROS driver process running?

    Not running is INFO when bringup as a whole is down (nothing is wrong,
    nothing has been started), FAIL when the rest of bringup is up (it was
    started and died -- "driver did not load"). optional_fix marks a driver
    that is not part of bringup at all: not running is always INFO.
    """
    name = f'ROS driver {exe}'
    pids = _pids(snap, exe)
    if pids:
        return Layer(name, P, f'running (pid {", ".join(map(str, pids))})')
    if optional_fix:
        return Layer(name, I, 'not launched (optional)', optional_fix)
    if not bringup_up(snap):
        return Layer(name, I, 'not running -- bringup is not up, so the '
                     'layers above were not checked', BRINGUP_FIX)
    return Layer(name, F, f'bringup is running but {exe} is not -- it '
                 'crashed or failed to start',
                 'read the bringup terminal for its error; restart bringup')


def _stats(live, topic):
    return live.topics.get(topic) if live is not None else None


def _rate_layer(live, topic, min_hz, *, silent_detail=None, silent_fix=''):
    name = f'{topic} rate'
    if live is None:
        return Layer(name, S, NO_LIVE)
    st = _stats(live, topic)
    if st is None or st.count == 0:
        window = st.window_s if st is not None else 0
        return Layer(name, F, silent_detail or
                     f'no messages in {window:.1f} s', silent_fix)
    if st.hz < min_hz:
        return Layer(name, F, f'{st.hz:.1f} Hz, expected >= {min_hz:g} Hz',
                     'driver is struggling: check its terminal, CPU load '
                     '(`top`), and the cable')
    return Layer(name, P, f'{st.hz:.1f} Hz')


def _last(live, topic):
    st = _stats(live, topic)
    return st.last if st is not None else None


# -------------------------------------------------------------------- VESC

def battery_layer(volts):
    name = 'battery (3S)'
    try:
        v = float(volts)
    except (TypeError, ValueError):
        v = math.nan
    if not math.isfinite(v) or v < 0:
        return Layer(name, F, f'invalid voltage reading {volts!r}')
    cell = v / car.BATTERY_CELLS
    reading = f'{v:.2f} V ({cell:.2f} V/cell)'
    if v < car.BATTERY_ABSENT_V:
        return Layer(name, F, f'{reading}: no battery voltage -- the VESC is '
                     'alive on USB only; battery unplugged or switched off',
                     'connect and switch on the battery')
    if v < car.BATTERY_FAIL_V:
        return Layer(name, F, f'{reading}: critically low (< '
                     f'{car.BATTERY_FAIL_V:.1f} V) -- stop and charge now',
                     'swap or charge the battery before driving')
    if v < car.BATTERY_WARN_V:
        return Layer(name, W, f'{reading}: low (< {car.BATTERY_WARN_V:.1f} V)',
                     'charge soon; voltage sags further under load')
    if v > car.BATTERY_HIGH_V:
        return Layer(name, W, f'{reading}: above a full 3S charge '
                     f'({car.BATTERY_HIGH_V:.2f} V) -- is a 4S pack fitted?')
    return Layer(name, P, f'{reading} at rest')


def check_vesc(snap, live):
    devs = _usb(snap, *car.VESC_USB)
    dev = devs[0] if devs else None
    vid_pid = ':'.join(car.VESC_USB)

    def usb():
        if dev is None:
            return Layer('USB device', F, f'not plugged in (no {vid_pid} on USB)',
                         'plug in the VESC USB cable; `lsusb` should list '
                         f'{vid_pid}')
        return Layer('USB device', P, f'{vid_pid} at {dev.sysname}')

    def kernel():
        if car.VESC_KERNEL_DRIVER not in dev.drivers or not dev.ttys:
            bound = ', '.join(dev.drivers) or 'none'
            return Layer('kernel driver', F, 'USB is present but '
                         f'{car.VESC_KERNEL_DRIVER} has not created a serial '
                         f'port (bound: {bound})',
                         're-plug the cable; `sudo dmesg | tail` for USB '
                         'errors; `sudo modprobe cdc_acm`')
        return Layer('kernel driver', P,
                     f'{car.VESC_KERNEL_DRIVER} -> /dev/{dev.ttys[0]}')

    def symlink():
        link = f'/dev/{car.VESC_SYMLINK}'
        if snap.vesc_symlink is None:
            return Layer(link, F, f'{link} missing although /dev/{dev.ttys[0]} '
                         'exists', 'udev rule /etc/udev/rules.d/99-vesc.rules: '
                         '`sudo udevadm control --reload && sudo udevadm '
                         'trigger`, then re-plug')
        target = os.path.basename(snap.vesc_symlink)
        if target not in dev.ttys:
            return Layer(link, F, f'{link} points at {target}, but the VESC is '
                         f'{", ".join(dev.ttys)}', 're-plug the VESC so udev '
                         'rewrites the link; check 99-vesc.rules')
        if not snap.vesc_symlink_rw:
            return Layer(link, F, f'{link} is not readable+writable by you',
                         '`sudo usermod -aG dialout $USER`, then log out and in')
        return Layer(link, P, f'-> {target}, read/write OK')

    def holders():
        name = 'serial port free'
        ttys = set(dev.ttys)
        pids = sorted({pid for path, ps in snap.tty_holders.items()
                       if os.path.basename(path) in ttys for pid in ps})
        driver_pids = set(_pids(snap, car.EXE_VESC_DRIVER))
        others = [pid for pid in pids if pid not in driver_pids]
        if others:
            by_pid = {p.pid: p for p in snap.processes}
            who = ', '.join(
                f'{pid} ({os.path.basename(by_pid[pid].argv[0])})'
                if pid in by_pid else str(pid) for pid in others)
            return Layer(name, F, f'serial port held by another process: '
                         f'pid {who}', 'close it (screen/minicom/VESC Tool?) '
                         'or `kill` that pid')
        if pids:
            return Layer(name, P, 'held only by vesc_driver_node')
        return Layer(name, P, 'no other process has it open')

    def telemetry():
        return _rate_layer(
            live, '/sensors/core', car.VESC_CORE_MIN_HZ,
            silent_detail='vesc_driver_node is running but the VESC sends no '
            'telemetry -- USB is up, the VESC is not answering. Most likely '
            'the battery is unplugged or switched off',
            silent_fix='switch on the battery, then restart bringup')

    def fault():
        if live is None:
            return Layer('fault code', S, NO_LIVE)
        code = _last(live, '/sensors/core').state.fault_code
        if code == 0:
            return Layer('fault code', P, 'FAULT_CODE_NONE')
        return Layer('fault code', F, f'VESC reports fault {code} '
                     f'({FAULT_NAMES.get(code, "unknown")})',
                     'power-cycle the VESC; check the battery and motor wiring')

    def battery():
        if live is None:
            return Layer('battery (3S)', S, NO_LIVE)
        return battery_layer(_last(live, '/sensors/core').state.voltage_input)

    return ladder('VESC', [
        ('USB device', usb), ('kernel driver', kernel),
        (f'/dev/{car.VESC_SYMLINK}', symlink), ('serial port free', holders),
        (f'ROS driver {car.EXE_VESC_DRIVER}',
         lambda: _process_layer(snap, car.EXE_VESC_DRIVER)),
        ('/sensors/core rate', telemetry), ('fault code', fault),
        ('battery (3S)', battery)])


# ------------------------------------------------------------------- LiDAR

def check_lidar(snap, live):
    iface = snap.ifaces.get(car.LIDAR_IFACE)
    urg_running = bool(_pids(snap, car.EXE_URG))

    def link():
        name = 'Ethernet link'
        if iface is None or iface.carrier is None:
            return Layer(name, F, f'{car.LIDAR_IFACE} is missing or down',
                         f'`nmcli device connect {car.LIDAR_IFACE}`')
        if not iface.carrier:
            return Layer(name, F, f'no link on {car.LIDAR_IFACE}: Ethernet '
                         'cable unplugged, or the LiDAR has no power',
                         'check the cable at both ends and the LiDAR power '
                         '(battery on?)')
        return Layer(name, P, f'{car.LIDAR_IFACE} carrier up')

    def host_ip():
        name = 'host IP'
        if car.LIDAR_HOST_IP not in iface.ipv4:
            have = ', '.join(iface.ipv4) or 'no IPv4 address'
            return Layer(name, F, f'{car.LIDAR_IFACE} has {have}, needs '
                         f'{car.LIDAR_HOST_IP}', '`nmcli connection up hokuyo`')
        return Layer(name, P, car.LIDAR_HOST_IP)

    def ping():
        name = 'ping'
        if snap.lidar_ping is None:
            return Layer(name, W, 'could not run `ping`; reachability unknown')
        if not snap.lidar_ping:
            return Layer(name, F, f'{car.LIDAR_IP} does not answer: LiDAR '
                         'still booting (~10 s after power), or its IP was '
                         'changed', 'wait and retry; see docs/hardware-reference.md')
        return Layer(name, P, f'{car.LIDAR_IP} answers')

    def tcp():
        name = f'TCP {car.LIDAR_PORT}'
        if urg_running:
            return Layer(name, S, 'not probed: urg_node owns the sensor')
        if snap.lidar_tcp is None:
            return Layer(name, S, 'not probed')
        if not snap.lidar_tcp:
            return Layer(name, F, f'{car.LIDAR_IP}:{car.LIDAR_PORT} refused '
                         'the connection -- another client may hold the '
                         'sensor', 'stop any other urg_node/UrgBenri; '
                         'power-cycle the LiDAR')
        return Layer(name, P, 'accepts connections')

    def content():
        name = '/scan content'
        if live is None:
            return Layer(name, S, NO_LIVE)
        ranges = list(_last(live, '/scan').ranges)
        if not ranges:
            return Layer(name, F, 'scans have no beams')
        valid = sum(1 for r in ranges if math.isfinite(r) and r > 0)
        frac = valid / len(ranges)
        detail = f'{len(ranges)} beams, {frac:.0%} with a return'
        if valid == 0:
            return Layer(name, F, f'{detail} -- the LiDAR sees nothing',
                         'lens covered or dirty? sensor fault?')
        if frac < car.SCAN_MIN_FINITE_FRACTION:
            return Layer(name, W, f'{detail} -- fewer than '
                         f'{car.SCAN_MIN_FINITE_FRACTION:.0%}; fine in a big '
                         'open room, suspicious near walls')
        return Layer(name, P, detail)

    return ladder('LiDAR', [
        ('Ethernet link', link), ('host IP', host_ip), ('ping', ping),
        (f'TCP {car.LIDAR_PORT}', tcp),
        (f'ROS driver {car.EXE_URG}', lambda: _process_layer(snap, car.EXE_URG)),
        ('/scan rate', lambda: _rate_layer(live, '/scan', car.SCAN_MIN_HZ)),
        ('/scan content', content)])


# ---------------------------------------------------------------- joystick

def check_joystick(snap, live):
    pads = _usb(snap, car.JOY_VID, car.JOY_XINPUT_PID)
    pad = pads[0] if pads else None

    def usb():
        name = 'USB receiver'
        if pad is None:
            if _usb(snap, car.JOY_VID, car.JOY_DIRECTINPUT_PID):
                return Layer(name, F, 'F710 is in DirectInput mode, which '
                             'creates no joystick device on this Jetson',
                             'flip the X/D switch on the back of the '
                             'controller to X, then restart bringup')
            return Layer(name, F, 'F710 receiver not plugged in',
                         'plug in the USB receiver')
        return Layer(name, P, 'F710 in XInput mode (this cannot tell whether '
                     'the controller itself is switched on)')

    def kernel():
        name = 'kernel driver'
        if car.JOY_KERNEL_DRIVER not in pad.drivers or not pad.joysticks:
            return Layer(name, F, f'{car.JOY_KERNEL_DRIVER} has not created a '
                         '/dev/input/js* device', 're-plug the receiver; '
                         '`sudo modprobe xpad`')
        return Layer(name, P, f'{car.JOY_KERNEL_DRIVER} -> '
                     f'/dev/input/{pad.joysticks[0]}')

    def readable():
        js = pad.joysticks[0]
        name = f'/dev/input/{js}'
        if not snap.js_readable.get(js):
            return Layer(name, F, 'not readable by you',
                         '`sudo usermod -aG input $USER`, then log out and in')
        return Layer(name, P, 'readable')

    return ladder('Joystick', [
        ('USB receiver', usb), ('kernel driver', kernel),
        ('/dev/input/js*', readable),
        (f'ROS driver {car.EXE_JOY}', lambda: _process_layer(snap, car.EXE_JOY)),
        ('/joy rate', lambda: _rate_layer(live, '/joy', car.JOY_MIN_HZ))])


# ------------------------------------------------------------------ camera

def check_camera(snap, live):
    cams = _usb(snap, *car.CAMERA_USB)
    cam = cams[0] if cams else None

    def usb():
        if cam is None:
            return Layer('USB device', F, 'RealSense D435i not plugged in',
                         'plug it into a USB 3 (blue) port')
        return Layer('USB device', P, f'{cam.product or "D435i"} at {cam.sysname}')

    def speed():
        name = 'USB speed'
        if cam.speed_mbps < car.CAMERA_MIN_SPEED_MBPS:
            return Layer(name, W, f'running at USB 2 ({cam.speed_mbps:g} Mb/s): '
                         'works at the low 424x240@15 profile, no headroom',
                         'use a USB 3 cable and push the plug fully home; '
                         'a half-seated USB 3 plug falls back to USB 2')
        return Layer(name, P, f'{cam.speed_mbps:g} Mb/s')

    def kernel():
        name = 'kernel driver'
        if car.CAMERA_KERNEL_DRIVER not in cam.drivers or not cam.videos:
            return Layer(name, F, f'{car.CAMERA_KERNEL_DRIVER} has not created '
                         '/dev/video* nodes', 're-plug the camera')
        return Layer(name, P, f'{car.CAMERA_KERNEL_DRIVER} -> '
                     f'{len(cam.videos)} /dev/video* nodes')

    report = ladder('Camera', [
        ('USB device', usb), ('USB speed', speed), ('kernel driver', kernel),
        (f'ROS driver {car.EXE_REALSENSE}', lambda: _process_layer(
            snap, car.EXE_REALSENSE, optional_fix='ros2 launch racerbot_launch '
            'realsense_camera_launch.py')),
        (f'{car.CAMERA_TOPIC} rate', lambda: _rate_layer(
            live, car.CAMERA_TOPIC, car.CAMERA_MIN_HZ))])
    # The car drives without a camera (team decision 2026-10-01): report
    # every camera failure, never fail the run on one. Capped *after* the
    # ladder so a missing camera still blocks the layers above it.
    for layer in report.layers:
        if layer.status is F:
            layer.status = W
    return report


# ----------------------------------------------------------------- bringup

def check_bringup(snap, live):
    if not bringup_up(snap):
        return DeviceReport('Bringup', [Layer(
            'bringup', I, 'not running: device layers above stop at the '
            'ROS driver', BRINGUP_FIX)])
    layers = [_process_layer(snap, exe) for exe in
              (car.EXE_MUX, car.EXE_ACK_TO_VESC, car.EXE_VESC_TO_ODOM)]
    if _pids(snap, car.EXE_VESC_TO_ODOM):
        core = _stats(live, '/sensors/core')
        core_silent = live is not None and (core is None or core.count == 0)
        layers.append(_rate_layer(
            live, '/odom', car.ODOM_MIN_HZ, silent_detail=(
                'no messages -- /odom is computed from /sensors/core, which '
                'is also silent: fix the VESC first') if core_silent else None))
    else:
        layers.append(Layer('/odom rate', S,
                            f'blocked by: ROS driver {car.EXE_VESC_TO_ODOM}'))

    name = 'base_link->laser TF'
    if live is None:
        layers.append(Layer(name, S, NO_LIVE))
    elif live.laser_tf is None:
        layers.append(Layer(name, F, 'no base_link->laser transform on '
                            '/tf_static', 'the static_transform_publisher in '
                            'bringup_launch.py did not start'))
    else:
        got = tuple(live.laser_tf)
        off = max(abs(a - b) for a, b in zip(got, car.LASER_TF))
        want = ' '.join(f'{v:g}' for v in car.LASER_TF)
        have = ' '.join(f'{v:.4g}' for v in got)
        if off > car.LASER_TF_TOL_M:
            layers.append(Layer(name, F, f'{have}, expected {want} (measured '
                                '2026-08-24)', 'an upstream sync clobbered it? '
                                'see docs/hardware-reference.md'))
        else:
            layers.append(Layer(name, P, have))

    name = '/teleop'
    if live is None:
        layers.append(Layer(name, S, NO_LIVE))
    elif live.teleop_publishers:
        layers.append(Layer(name, I, 'teleop_launch.py is running: /teleop '
                            'outranks /drive, so autonomy cannot drive',
                            'Ctrl+C teleop_launch.py before starting autonomy'))
    else:
        layers.append(Layer(name, P, 'no publisher (autonomy can reach /drive)'))
    return DeviceReport('Bringup', layers)


# ------------------------------------------------------------- environment

def check_environment(snap, live):
    env = snap.env
    layers = []
    distro = env.get('ROS_DISTRO')
    if distro == 'jazzy':
        layers.append(Layer('ROS sourced', P, 'jazzy'))
    else:
        layers.append(Layer('ROS sourced', F, f'ROS_DISTRO={distro or "unset"}',
                            'source /opt/ros/jazzy/setup.bash'))

    prefixes = {os.path.basename(p.rstrip('/'))
                for p in env.get('AMENT_PREFIX_PATH', '').split(':') if p}
    missing = [p for p in ('vesc_driver', 'ackermann_mux', 'f1tenth_stack')
               if p not in prefixes]
    if missing:
        layers.append(Layer('workspace sourced', F, 'not found: '
                            + ', '.join(missing), 'cd ~/racerbot-ws && '
                            'colcon build --symlink-install && source '
                            'install/setup.bash'))
    else:
        layers.append(Layer('workspace sourced', P, 'driver packages found'))

    domain = env.get('ROS_DOMAIN_ID', '0')
    if domain == car.TEST_DOMAIN_ID:
        layers.append(Layer('ROS domain', W, f'ROS_DOMAIN_ID={domain} is the '
                            'test domain: you are not looking at the car',
                            'unset ROS_DOMAIN_ID'))
    else:
        layers.append(Layer('ROS domain', P, f'ROS_DOMAIN_ID={domain}'))

    sims = [exe for exe in car.SIM_EXES if _pids(snap, exe)]
    if sims and bringup_up(snap):
        layers.append(Layer('no simulator', F, f'{", ".join(sims)} running '
                            'beside the real drivers: forged /joy, second '
                            '/scan and /odom', 'stop racerbot_sim now'))
    elif sims:
        layers.append(Layer('no simulator', W, f'{", ".join(sims)} running: '
                            'live results describe the simulator, not the car'))
    else:
        layers.append(Layer('no simulator', P, 'racerbot_sim not running'))
    return DeviceReport('Environment', layers)


def run_all(snap, live):
    return [check(snap, live) for check in (
        check_environment, check_vesc, check_lidar, check_joystick,
        check_camera, check_bringup)]
