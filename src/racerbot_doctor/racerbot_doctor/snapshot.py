"""Plain data the doctor passes between its three layers.

system.py fills a Snapshot from the OS, live.py fills a LiveSample from the
ROS graph, checks.py turns both into DeviceReports. Nothing in this file
imports rclpy or touches the OS, so every verdict can be tested with
hand-built inputs.
"""
import enum
import math
from dataclasses import dataclass, field


class Status(enum.Enum):
    PASS = 'PASS'
    WARN = 'WARN'
    FAIL = 'FAIL'
    INFO = 'INFO'   # fine, but worth knowing (e.g. "bringup not running")
    SKIP = 'SKIP'   # not evaluated: a lower layer failed, or live stage off


# worst() ranks these; SKIP never makes a device look worse than INFO.
_RANK = {Status.PASS: 0, Status.SKIP: 1, Status.INFO: 1,
         Status.WARN: 2, Status.FAIL: 3}


@dataclass
class Layer:
    name: str
    status: Status
    detail: str
    fix: str = ''


@dataclass
class DeviceReport:
    device: str
    layers: list

    def worst(self):
        worst = Status.PASS
        for layer in self.layers:
            if _RANK[layer.status] > _RANK[worst]:
                worst = layer.status
        return worst


def ladder(device, steps):
    """Evaluate (name, fn) steps bottom-up; fn() -> Layer.

    The first FAIL or INFO blocks everything above it: those layers are
    reported SKIP and their fn is never called, because a probe above a
    broken layer can only mislead ("no /scan" means nothing when the
    cable is out). WARN does not block.
    """
    layers = []
    blocker = None
    for name, fn in steps:
        if blocker is not None:
            layers.append(Layer(name, Status.SKIP, f'blocked by: {blocker}'))
            continue
        layer = fn()
        layers.append(layer)
        if layer.status in (Status.FAIL, Status.INFO):
            blocker = layer.name
    return DeviceReport(device, layers)


@dataclass
class UsbDevice:
    sysname: str            # e.g. '1-2.1'
    vid: str                # lowercase hex, e.g. '0483'
    pid: str
    product: str = ''
    speed_mbps: float = 0.0
    drivers: list = field(default_factory=list)     # bound interface drivers
    ttys: list = field(default_factory=list)        # e.g. ['ttyACM0']
    videos: list = field(default_factory=list)      # e.g. ['video4']
    joysticks: list = field(default_factory=list)   # e.g. ['js0']


@dataclass
class Process:
    pid: int
    argv: list

    def runs(self, executable):
        """True if argv[0] (or argv[1] for an interpreter) ends in executable."""
        for arg in self.argv[:2]:
            if arg.rsplit('/', 1)[-1] == executable:
                return True
        return False


@dataclass
class NetIface:
    name: str
    carrier: object = None      # True / False / None (unknown, iface down)
    ipv4: list = field(default_factory=list)    # ['192.168.0.15/24']


@dataclass
class Snapshot:
    usb: list = field(default_factory=list)
    processes: list = field(default_factory=list)
    ifaces: dict = field(default_factory=dict)
    vesc_symlink: object = None         # resolved target path, or None if absent
    vesc_symlink_rw: bool = False
    tty_holders: dict = field(default_factory=dict)  # '/dev/ttyACM0' -> [pid]
    js_readable: dict = field(default_factory=dict)  # 'js0' -> bool
    lidar_ping: object = None           # True / False / None = not probed
    lidar_tcp: object = None            # True / False / None = not probed
    env: dict = field(default_factory=dict)


@dataclass
class TopicStats:
    count: int = 0
    window_s: float = 0.0
    last: object = None

    @property
    def hz(self):
        if self.window_s <= 0 or not math.isfinite(self.window_s):
            return 0.0
        return self.count / self.window_s


@dataclass
class LiveSample:
    topics: dict = field(default_factory=dict)      # topic -> TopicStats
    teleop_publishers: int = 0
    laser_tf: object = None             # (x, y, z) or None if never seen
