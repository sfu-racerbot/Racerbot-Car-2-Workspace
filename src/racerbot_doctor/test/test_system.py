"""
Tests for system.py -- reading the OS (sysfs, /proc, /dev, `ip`).

Pure filesystem + string handling, no rclpy:
    python3 -m pytest src/racerbot_doctor/test/ -v

Oracle: the fake sysfs tree below mirrors the real layout recorded on car 2
on 2026-10-01 (`ls /sys/bus/usb/devices`, `lsusb -t`, `ip -o -4 addr`):
the D435i at 1-2.1 (8086:0b3a, speed 480, uvcvideo, video4linux/video0..3
under interface 1.0), the F710 at 1-2.4 (046d:c21f, speed 12, xpad,
input/input5/js0). The VESC entry (0483:5740, cdc_acm, tty/ttyACM0) follows
the standard cdc_acm sysfs layout; it was unplugged that day. Tests never
touch the real /sys or /proc -- everything passes an explicit tmp_path root.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from racerbot_doctor.system import (  # noqa: E402
    gather, parse_ip_addr, read_carrier, read_processes, read_usb_devices,
    tty_holders)


def _usb(sys_root, name, vid, pid, speed, product, interfaces):
    """interfaces: {'1.0': {'driver': 'cdc_acm', 'tty': ['ttyACM0'], ...}}"""
    dev = sys_root / 'bus/usb/devices' / name
    dev.mkdir(parents=True)
    (dev / 'idVendor').write_text(vid + '\n')
    (dev / 'idProduct').write_text(pid + '\n')
    if speed is not None:
        (dev / 'speed').write_text(f'{speed}\n')
    (dev / 'product').write_text(product + '\n')
    drivers_dir = sys_root / 'bus/usb/drivers'
    for intf, spec in interfaces.items():
        idir = sys_root / 'bus/usb/devices' / f'{name}:{intf}'
        idir.mkdir()
        if spec.get('driver'):
            (drivers_dir / spec['driver']).mkdir(parents=True, exist_ok=True)
            os.symlink(drivers_dir / spec['driver'], idir / 'driver')
        for tty in spec.get('tty', []):
            (idir / 'tty' / tty).mkdir(parents=True)
        for video in spec.get('video', []):
            (idir / 'video4linux' / video).mkdir(parents=True)
        for inp, js in spec.get('input', {}).items():
            (idir / 'input' / inp / js).mkdir(parents=True)
            (idir / 'input' / inp / f'event{inp[-1]}').mkdir()


def _car2_tree(tmp_path):
    root = tmp_path / 'sys'
    _usb(root, '1-2', '0bda', '5489', 480, '4-Port USB 2.0 Hub',
         {'1.0': {'driver': 'hub'}})
    _usb(root, '1-2.1', '8086', '0b3a', 480,
         'Intel(R) RealSense(TM) Depth Camera 435i',
         {'1.0': {'driver': 'uvcvideo',
                  'video': ['video0', 'video1', 'video2', 'video3']},
          '1.1': {'driver': 'uvcvideo'},
          '1.3': {'driver': 'uvcvideo', 'video': ['video4', 'video5']},
          '1.5': {'driver': 'usbhid'}})
    _usb(root, '1-2.4', '046d', 'c21f', 12, 'Wireless Gamepad F710',
         {'1.0': {'driver': 'xpad', 'input': {'input5': 'js0'}}})
    _usb(root, '1-2.3', '0483', '5740', 12, 'ChibiOS/RT Virtual COM Port',
         {'1.0': {'driver': 'cdc_acm', 'tty': ['ttyACM0']},
          '1.1': {'driver': 'cdc_acm'}})
    return root


def _by_vid(devs):
    return {(d.vid, d.pid): d for d in devs}


def test_reads_vid_pid_speed_product(tmp_path):
    devs = _by_vid(read_usb_devices(_car2_tree(tmp_path)))
    cam = devs[('8086', '0b3a')]
    assert cam.sysname == '1-2.1'
    assert cam.speed_mbps == 480.0
    assert cam.product == 'Intel(R) RealSense(TM) Depth Camera 435i'
    pad = devs[('046d', 'c21f')]
    assert pad.speed_mbps == 12.0


def test_collects_drivers_and_device_nodes_across_interfaces(tmp_path):
    devs = _by_vid(read_usb_devices(_car2_tree(tmp_path)))
    cam = devs[('8086', '0b3a')]
    assert sorted(set(cam.drivers)) == ['usbhid', 'uvcvideo']
    assert sorted(cam.videos) == ['video0', 'video1', 'video2', 'video3',
                                  'video4', 'video5']
    assert devs[('046d', 'c21f')].joysticks == ['js0']
    assert devs[('046d', 'c21f')].drivers == ['xpad']
    vesc = devs[('0483', '5740')]
    assert vesc.ttys == ['ttyACM0']
    assert vesc.drivers == ['cdc_acm']   # deduplicated across interfaces


def test_interface_dirs_are_not_reported_as_devices(tmp_path):
    names = sorted(d.sysname for d in read_usb_devices(_car2_tree(tmp_path)))
    assert names == ['1-2', '1-2.1', '1-2.3', '1-2.4']


def test_device_without_driver_link_has_no_drivers(tmp_path):
    root = tmp_path / 'sys'
    _usb(root, '1-1', '0483', '5740', 12, 'VESC', {'1.0': {}})
    (dev,) = read_usb_devices(root)
    assert dev.drivers == []
    assert dev.ttys == []


def test_missing_speed_file_reads_as_zero(tmp_path):
    root = tmp_path / 'sys'
    _usb(root, '1-1', '0483', '5740', None, 'VESC', {})
    (dev,) = read_usb_devices(root)
    assert dev.speed_mbps == 0.0


def test_vid_pid_lowercased(tmp_path):
    root = tmp_path / 'sys'
    _usb(root, '1-1', '046D', 'C21F', 12, 'pad', {})
    (dev,) = read_usb_devices(root)
    assert (dev.vid, dev.pid) == ('046d', 'c21f')


def test_no_usb_tree_is_empty_not_a_crash(tmp_path):
    assert read_usb_devices(tmp_path / 'nothing') == []


def _proc(proc_root, pid, argv, fds=None):
    d = proc_root / str(pid)
    d.mkdir(parents=True)
    (d / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in argv) + b'\0')
    (d / 'fd').mkdir()
    for n, target in (fds or {}).items():
        os.symlink(target, d / 'fd' / str(n))


def test_read_processes_splits_nul_argv_and_ignores_non_pids(tmp_path):
    proc = tmp_path / 'proc'
    _proc(proc, 4242, ['/ws/install/vesc_driver/lib/vesc_driver/vesc_driver_node',
                       '--ros-args', '-r', '__node:=vesc_driver_node'])
    _proc(proc, 77, ['/usr/bin/python3', '/opt/ros/jazzy/lib/joy/joy_node'])
    (proc / 'self').mkdir()
    (proc / 'meminfo').write_text('x')
    procs = {p.pid: p for p in read_processes(proc)}
    assert sorted(procs) == [77, 4242]
    assert procs[4242].argv[0].endswith('vesc_driver_node')
    assert procs[4242].argv[1] == '--ros-args'
    assert procs[4242].runs('vesc_driver_node')
    assert procs[77].runs('joy_node')          # python interpreter argv[1]
    assert not procs[77].runs('joy')           # basename match, not substring


def test_kernel_threads_with_empty_cmdline_are_skipped(tmp_path):
    proc = tmp_path / 'proc'
    d = proc / '2'
    d.mkdir(parents=True)
    (d / 'cmdline').write_bytes(b'')
    assert read_processes(proc) == []


def test_tty_holders_finds_pids_with_the_tty_open(tmp_path):
    proc = tmp_path / 'proc'
    _proc(proc, 10, ['a'], {3: '/dev/ttyACM0', 4: '/dev/null'})
    _proc(proc, 11, ['b'], {3: '/dev/ttyACM1'})
    _proc(proc, 12, ['c'], {5: '/dev/ttyACM0'})
    assert tty_holders(proc, ['/dev/ttyACM0']) == {'/dev/ttyACM0': [10, 12]}


def test_tty_holders_unreadable_fd_dir_is_skipped(tmp_path):
    proc = tmp_path / 'proc'
    _proc(proc, 10, ['a'], {3: '/dev/ttyACM0'})
    (proc / '13').mkdir()                    # no fd dir: exited / no perms
    assert tty_holders(proc, ['/dev/ttyACM0']) == {'/dev/ttyACM0': [10]}


# Captured from `ip -o -4 addr` on car 2, 2026-10-01 (trimmed to 3 lines).
IP_O_4 = (
    '1: lo    inet 127.0.0.1/8 scope host lo\\       valid_lft forever preferred_lft forever\n'
    '4: enP8p1s0    inet 192.168.0.15/24 brd 192.168.0.255 scope global noprefixroute enP8p1s0\\       valid_lft forever preferred_lft forever\n'
    '9: docker0    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0\\       valid_lft forever preferred_lft forever\n'
)


def test_parse_ip_addr_real_output():
    assert parse_ip_addr(IP_O_4) == {
        'lo': ['127.0.0.1/8'],
        'enP8p1s0': ['192.168.0.15/24'],
        'docker0': ['172.17.0.1/16'],
    }


def test_parse_ip_addr_garbage_and_empty():
    assert parse_ip_addr('') == {}
    assert parse_ip_addr('not ip output\n\n3: x\n') == {}


def test_read_carrier(tmp_path):
    net = tmp_path / 'sys/class/net'
    (net / 'up').mkdir(parents=True)
    (net / 'up/carrier').write_text('1\n')
    (net / 'unplugged').mkdir()
    (net / 'unplugged/carrier').write_text('0\n')
    (net / 'admin_down').mkdir()            # real kernel: read gives EINVAL
    root = tmp_path / 'sys'
    assert read_carrier(root, 'up') is True
    assert read_carrier(root, 'unplugged') is False
    assert read_carrier(root, 'admin_down') is None
    assert read_carrier(root, 'missing') is None


class _Run:
    """Stand-in for subprocess.run at the process boundary."""
    def __init__(self, ip_out='', ping_rc=0):
        self.ip_out, self.ping_rc, self.calls = ip_out, ping_rc, []

    def __call__(self, argv, **kw):
        self.calls.append(argv)

        class R:
            pass
        r = R()
        r.stdout = self.ip_out if argv[0] == 'ip' else ''
        r.returncode = self.ping_rc if argv[0] == 'ping' else 0
        return r


def _gather(tmp_path, *, probe_lidar_tcp, connect_ok=True, run=None,
            vesc_link='ttyACM0', extra_procs=()):
    sys_root = _car2_tree(tmp_path)
    dev = tmp_path / 'dev'
    (dev / 'sensors').mkdir(parents=True)
    (dev / 'input').mkdir()
    (dev / 'ttyACM0').write_text('')
    (dev / 'input/js0').write_text('')
    if vesc_link:
        os.symlink(f'../{vesc_link}', dev / 'sensors/vesc')
    proc = tmp_path / 'proc'
    _proc(proc, 500, ['vesc_driver_node'], {3: str(dev / 'ttyACM0')})
    for pid, argv in extra_procs:
        _proc(proc, pid, argv)
    connects = []

    def connect(addr, timeout):
        connects.append(addr)
        if not connect_ok:
            raise ConnectionRefusedError(111, 'refused')

        class S:
            def close(self):
                pass
        return S()

    snap = gather(sys_root=sys_root, dev_root=dev, proc_root=proc,
                  run=run or _Run(IP_O_4), connect=connect,
                  probe_lidar_tcp=probe_lidar_tcp, env={'ROS_DISTRO': 'jazzy'})
    return snap, connects, dev


def test_gather_resolves_vesc_symlink_and_holders(tmp_path):
    snap, _, dev = _gather(tmp_path, probe_lidar_tcp=False)
    assert snap.vesc_symlink == str(dev / 'ttyACM0')
    assert snap.vesc_symlink_rw is True
    assert snap.tty_holders == {str(dev / 'ttyACM0'): [500]}
    assert snap.js_readable == {'js0': True}
    assert snap.ifaces['enP8p1s0'].ipv4 == ['192.168.0.15/24']
    assert snap.env == {'ROS_DISTRO': 'jazzy'}


def test_gather_missing_vesc_symlink_is_none(tmp_path):
    snap, _, _ = _gather(tmp_path, probe_lidar_tcp=False, vesc_link=None)
    assert snap.vesc_symlink is None
    assert snap.vesc_symlink_rw is False


def test_gather_dangling_vesc_symlink_reports_target(tmp_path):
    snap, _, dev = _gather(tmp_path, probe_lidar_tcp=False,
                           vesc_link='ttyACM9')
    assert snap.vesc_symlink == str(dev / 'ttyACM9')
    assert snap.vesc_symlink_rw is False


def test_gather_does_not_touch_hokuyo_tcp_unless_asked(tmp_path):
    snap, connects, _ = _gather(tmp_path, probe_lidar_tcp=False)
    assert connects == []
    assert snap.lidar_tcp is None


def test_gather_probes_hokuyo_tcp_when_asked(tmp_path):
    snap, connects, _ = _gather(tmp_path, probe_lidar_tcp=True)
    assert connects == [('192.168.0.10', 10940)]
    assert snap.lidar_tcp is True


def test_gather_tcp_refused_is_false(tmp_path):
    snap, _, _ = _gather(tmp_path, probe_lidar_tcp=True, connect_ok=False)
    assert snap.lidar_tcp is False


def test_gather_ping_result(tmp_path):
    run = _Run(IP_O_4, ping_rc=1)
    snap, _, _ = _gather(tmp_path, probe_lidar_tcp=False, run=run)
    assert snap.lidar_ping is False
    assert ['ping', '-c', '1', '-W', '1', '192.168.0.10'] in run.calls


def test_gather_survives_missing_ip_and_ping_binaries(tmp_path):
    def run(argv, **kw):
        raise FileNotFoundError(argv[0])
    snap, _, _ = _gather(tmp_path, probe_lidar_tcp=False, run=run)
    assert snap.lidar_ping is None
    assert snap.ifaces['enP8p1s0'].ipv4 == []


def test_gather_auto_probes_tcp_when_urg_node_is_down(tmp_path):
    snap, connects, _ = _gather(tmp_path, probe_lidar_tcp='auto')
    assert connects == [('192.168.0.10', 10940)]


def test_gather_auto_never_probes_tcp_while_urg_node_runs(tmp_path):
    snap, connects, _ = _gather(
        tmp_path, probe_lidar_tcp='auto',
        extra_procs=[(600, ['/opt/ros/jazzy/lib/urg_node/urg_node_driver',
                            '--ros-args'])])
    assert connects == []
    assert snap.lidar_tcp is None
