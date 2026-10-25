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


def _serial_layers(snap, label, usb_id, driver, rel_link, link_target,
                   link_rw, rule, owner_exe):
    """USB -> kernel driver -> /dev symlink -> port free, for a cdc_acm
    device behind a udev symlink (the VESC here; car 1 also uses it for its
    USB Hokuyo)."""
    devs = _usb(snap, *usb_id)
    dev = devs[0] if devs else None
    vid_pid = ':'.join(usb_id)
    link = f'/dev/{rel_link}'

    def usb():
        if dev is None:
            return Layer('USB device', F, f'not plugged in (no {vid_pid} on USB)',
                         f'plug in the {label} USB cable; `lsusb` should list '
                         f'{vid_pid}')
        return Layer('USB device', P, f'{vid_pid} at {dev.sysname}')

    def kernel():
        if driver not in dev.drivers or not dev.ttys:
            bound = ', '.join(dev.drivers) or 'none'
            return Layer('kernel driver', F, 'USB is present but '
                         f'{driver} has not created a serial '
                         f'port (bound: {bound})',
                         're-plug the cable; `sudo dmesg | tail` for USB '
                         f'errors; `sudo modprobe {driver}`')
        return Layer('kernel driver', P, f'{driver} -> /dev/{dev.ttys[0]}')

    def symlink():
        if link_target is None:
            return Layer(link, F, f'{link} missing although /dev/{dev.ttys[0]} '
                         f'exists', f'udev rule /etc/udev/rules.d/{rule}: '
                         '`sudo udevadm control --reload && sudo udevadm '
                         'trigger`, then re-plug')
        target = os.path.basename(link_target)
        if target not in dev.ttys:
            return Layer(link, F, f'{link} points at {target}, but the {label} '
                         f'is {", ".join(dev.ttys)}', f're-plug the {label} so '
                         f'udev rewrites the link; check {rule}')
        if not link_rw:
            return Layer(link, F, f'{link} is not readable+writable by you',
                         '`sudo usermod -aG dialout $USER`, then log out and '
                         f'in (or check MODE in {rule})')
        return Layer(link, P, f'-> {target}, read/write OK')

    def holders():
        name = 'serial port free'
        ttys = set(dev.ttys)
        pids = sorted({pid for path, ps in snap.tty_holders.items()
                       if os.path.basename(path) in ttys for pid in ps})
        owner_pids = set(_pids(snap, owner_exe))
        others = [pid for pid in pids if pid not in owner_pids]
        if others:
            by_pid = {p.pid: p for p in snap.processes}
            who = ', '.join(
                f'{pid} ({os.path.basename(by_pid[pid].argv[0])})'
                if pid in by_pid else str(pid) for pid in others)
            return Layer(name, F, f'serial port held by another process: '
                         f'pid {who}', 'close it (screen/minicom/VESC Tool/'
                         'a second urg_node?) or `kill` that pid')
        if pids:
            return Layer(name, P, f'held only by {owner_exe}')
        return Layer(name, P, 'no other process has it open')

    return [('USB device', usb), ('kernel driver', kernel), (link, symlink),
            ('serial port free', holders)]


def check_vesc(snap, live):
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

    return ladder('VESC', _serial_layers(
        snap, 'VESC', car.VESC_USB, car.VESC_KERNEL_DRIVER, car.VESC_SYMLINK,
        snap.vesc_symlink, snap.vesc_symlink_rw, '99-vesc.rules',
        car.EXE_VESC_DRIVER) + [
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
        ('/scan rate', lambda: _rate_layer(
            live, '/scan', car.SCAN_MIN_HZ, silent_detail='urg_node is '
            'running but publishes no scans: it could not reach or talk to '
            'the LiDAR -- usually the LiDAR has no power (battery off?) or '
            'was still booting when bringup started',
            silent_fix='read the bringup terminal for urg_node errors; '
            'check LiDAR power, then restart bringup')),
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
        # joy_node (Jazzy, SDL2) opens the *event* node; js* is only the old
        # joystick API. On this Jetson js0 is world-readable but event* is
        # root:input 0660 -- a readable js0 proves nothing (2026-10-20: /joy
        # silent, user not in `input`).
        name = 'input permissions'
        nodes = pad.joysticks + pad.events
        blocked = [n for n in nodes if not snap.input_readable.get(n)]
        if blocked:
            ev = [n for n in blocked if n.startswith('event')]
            why = (f'joy_node reads /dev/input/{ev[0]} and you cannot -- it '
                   'will run, find no controller, and publish no /joy, so '
                   'the car cannot be driven' if ev else
                   'not readable by you')
            return Layer(name, F, f'{", ".join(blocked)}: {why}',
                         f'`sudo usermod -aG input $USER`, log out and in '
                         '(or reboot), then restart bringup')
        return Layer(name, P, f'{", ".join(nodes)} readable')

    return ladder('Joystick', [
        ('USB receiver', usb), ('kernel driver', kernel),
        ('input permissions', readable),
        (f'ROS driver {car.EXE_JOY}', lambda: _process_layer(snap, car.EXE_JOY)),
        ('/joy rate', lambda: _rate_layer(
            live, '/joy', car.JOY_MIN_HZ, silent_detail='joy_node is running '
            'but publishes nothing: the controller is off or asleep, or '
            'joy_node could not open it',
            silent_fix='press the Logitech button in the middle of the pad '
            '(LED on); if still silent, read the bringup terminal for joy '
            'errors and restart bringup'))])


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
        servo = _stats(live, '/commands/servo/position')
        core_silent = live is not None and (core is None or core.count == 0)
        servo_silent = live is not None and (servo is None or servo.count == 0)
        odom = _stats(live, '/odom')
        if (not core_silent and servo_silent
                and (odom is None or odom.count == 0)):
            # vesc.yaml use_servo_cmd_to_calc_angular_velocity: true, and
            # vesc_to_odom.cpp:97 returns until the first servo command.
            layers.append(Layer('/odom rate', I, 'no messages yet: '
                                'vesc_to_odom_node waits for the first '
                                '/commands/servo/position, and nothing has '
                                'sent one -- normal until something drives',
                                'see the drive path section'))
        else:
            layers.append(_rate_layer(
                live, '/odom', car.ODOM_MIN_HZ, silent_detail=(
                    'no messages -- /odom is computed from /sensors/core, '
                    'which is also silent: fix the VESC first')
                if core_silent else None))
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


# -------------------------------------------------------------- drive path

def check_drive_path(snap, live):
    """Manual driving, hop by hop: /joy -> /teleop -> /ackermann_cmd ->
    /commands/*, then whether LB was held and the VESC was told to move.
    Answers "I'm holding LB and the car doesn't move" without the reader
    having to know the topic graph."""
    if not _pids(snap, car.EXE_JOY_TELEOP):
        return DeviceReport('Drive path (teleop)', [Layer(
            'teleop', I, 'teleop_launch.py is not running, so nothing turns '
            'the sticks into drive commands',
            'ros2 launch f1tenth_stack teleop_launch.py')])

    def hop(topic, silent):
        return lambda: _rate_layer(live, topic, car.DRIVE_MIN_HZ,
                                   silent_detail=silent)

    def lb():
        name = 'LB held'
        joy = _stats(live, '/joy')
        if live.joy_lb_held == 0:
            return Layer(name, I, 'LB was not held during the sample, so the '
                         'deadman path was not exercised',
                         're-run the doctor while holding LB and pushing the '
                         'left stick')
        return Layer(name, P, f'in {live.joy_lb_held} of {joy.count} /joy '
                     'messages')

    def motor():
        name = 'motor command'
        if live.motor_nonzero == 0:
            return Layer(name, W, 'LB held, but every motor speed command was '
                         'zero -- the left stick was not moved, or joy_teleop '
                         'reads a different axis (joy_teleop.yaml)',
                         'hold LB and push the left stick forward')
        return Layer(name, P, f'{live.motor_nonzero} non-zero speed commands '
                     'sent: ROS is telling the VESC to move. If the wheels '
                     'still do not turn, it is below ROS: battery switch, ESC '
                     'power, motor wiring')

    steps = [('/joy rate', hop('/joy', 'no /joy, so joy_teleop has nothing '
                               'to act on -- see Joystick above'))]
    if live is not None:
        steps += [
            ('/teleop rate', hop('/teleop', 'joy_teleop is running and /joy '
                                 'flows, but joy_teleop publishes nothing')),
            ('/ackermann_cmd rate', hop('/ackermann_cmd', '/teleop flows but '
                                        'ackermann_mux forwards nothing')),
            ('/commands/motor/speed rate', hop(
                '/commands/motor/speed', '/ackermann_cmd flows but '
                'ackermann_to_vesc_node sends the VESC nothing')),
            ('LB held', lb), ('motor command', motor)]
    return ladder('Drive path (teleop)', steps)


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
        check_camera, check_bringup, check_drive_path)]
