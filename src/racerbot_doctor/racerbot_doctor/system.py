"""Read the OS into a Snapshot. The only module that touches the machine.

Read-only by design, and stricter than that: it never open()s a tty, a
video node or a joystick node (opening a tty can toggle DTR; opening a
video node can take the camera from its driver) -- access is checked with
os.access only. It opens a TCP session to the Hokuyo only when told to,
and the CLI only asks for that when urg_node is not running, so it never
competes with the driver for the sensor.

Every root path and process boundary (subprocess.run, socket connect) is a
parameter, so tests run against a fake tree in tmp_path.
"""
import os
import socket
import subprocess

from . import car
from .snapshot import NetIface, Process, Snapshot, UsbDevice


def _read(path, default=''):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def _listdir(path):
    try:
        return sorted(os.listdir(path))
    except OSError:
        return []


def read_usb_devices(sys_root):
    base = os.path.join(sys_root, 'bus/usb/devices')
    devices = []
    names = _listdir(base)
    for name in names:
        if ':' in name:
            continue                    # an interface, not a device
        dev_dir = os.path.join(base, name)
        vid = _read(os.path.join(dev_dir, 'idVendor')).lower()
        if not vid:
            continue
        try:
            speed = float(_read(os.path.join(dev_dir, 'speed'), '0'))
        except ValueError:
            speed = 0.0
        dev = UsbDevice(
            sysname=name, vid=vid,
            pid=_read(os.path.join(dev_dir, 'idProduct')).lower(),
            product=_read(os.path.join(dev_dir, 'product')),
            speed_mbps=speed)
        for intf in names:
            if not intf.startswith(name + ':'):
                continue
            idir = os.path.join(base, intf)
            link = os.path.join(idir, 'driver')
            if os.path.islink(link):
                drv = os.path.basename(os.readlink(link))
                if drv not in dev.drivers:
                    dev.drivers.append(drv)
            dev.ttys += _listdir(os.path.join(idir, 'tty'))
            dev.videos += _listdir(os.path.join(idir, 'video4linux'))
            for inp in _listdir(os.path.join(idir, 'input')):
                dev.joysticks += [n for n in _listdir(
                    os.path.join(idir, 'input', inp)) if n.startswith('js')]
        devices.append(dev)
    return devices


def read_processes(proc_root):
    procs = []
    for name in _listdir(proc_root):
        if not name.isdigit():
            continue
        try:
            with open(os.path.join(proc_root, name, 'cmdline'), 'rb') as f:
                raw = f.read()
        except OSError:
            continue                    # exited between listdir and open
        argv = [a.decode(errors='replace') for a in raw.split(b'\0') if a]
        if argv:
            procs.append(Process(int(name), argv))
    return sorted(procs, key=lambda p: p.pid)


def tty_holders(proc_root, ttys):
    """{tty: [pid, ...]} for the given device paths. Other users' processes
    are invisible without root, so this can under-report, never over-report."""
    wanted = set(ttys)
    found = {}
    for name in _listdir(proc_root):
        if not name.isdigit():
            continue
        fd_dir = os.path.join(proc_root, name, 'fd')
        for fd in _listdir(fd_dir):
            try:
                target = os.readlink(os.path.join(fd_dir, fd))
            except OSError:
                continue
            if target in wanted:
                pids = found.setdefault(target, [])
                if int(name) not in pids:
                    pids.append(int(name))
    return {k: sorted(v) for k, v in found.items()}


def parse_ip_addr(text):
    """`ip -o -4 addr` -> {iface: ['a.b.c.d/nn', ...]}"""
    out = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 4 or parts[2] != 'inet' or not parts[0].endswith(':'):
            continue
        out.setdefault(parts[1], []).append(parts[3])
    return out


def read_carrier(sys_root, iface):
    """True/False from /sys/class/net/<iface>/carrier; None if the interface
    is missing or administratively down (the kernel answers EINVAL then)."""
    value = _read(os.path.join(sys_root, 'class/net', iface, 'carrier'), None)
    if value == '1':
        return True
    if value == '0':
        return False
    return None


def _run_quiet(run, argv, timeout):
    try:
        return run(argv, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None


def gather(*, sys_root='/sys', dev_root='/dev', proc_root='/proc',
           run=subprocess.run, connect=socket.create_connection,
           probe_lidar_tcp='auto', env=None):
    """probe_lidar_tcp: True, False, or 'auto' = only if urg_node is not
    running."""
    snap = Snapshot(env=dict(os.environ if env is None else env))
    snap.usb = read_usb_devices(sys_root)
    snap.processes = read_processes(proc_root)

    ip = _run_quiet(run, ['ip', '-o', '-4', 'addr'], 2.0)
    addrs = parse_ip_addr(ip.stdout) if ip is not None else {}
    for name in set(addrs) | {car.LIDAR_IFACE}:
        snap.ifaces[name] = NetIface(name, read_carrier(sys_root, name),
                                     addrs.get(name, []))

    link = os.path.join(dev_root, car.VESC_SYMLINK)
    if os.path.lexists(link):
        snap.vesc_symlink = os.path.realpath(link)
        snap.vesc_symlink_rw = os.access(link, os.R_OK | os.W_OK)

    ttys = [os.path.join(dev_root, t) for d in snap.usb for t in d.ttys]
    if snap.vesc_symlink:
        ttys.append(snap.vesc_symlink)
    snap.tty_holders = tty_holders(proc_root, ttys)

    for d in snap.usb:
        for js in d.joysticks:
            snap.js_readable[js] = os.access(
                os.path.join(dev_root, 'input', js), os.R_OK)

    ping = _run_quiet(run, ['ping', '-c', '1', '-W', '1', car.LIDAR_IP], 3.0)
    snap.lidar_ping = None if ping is None else ping.returncode == 0

    if probe_lidar_tcp == 'auto':
        # Never open a second session to a sensor the driver owns.
        probe_lidar_tcp = not any(p.runs(car.EXE_URG) for p in snap.processes)
    if probe_lidar_tcp:
        try:
            s = connect((car.LIDAR_IP, car.LIDAR_PORT), timeout=1.0)
            s.close()
            snap.lidar_tcp = True
        except OSError:
            snap.lidar_tcp = False
    return snap
