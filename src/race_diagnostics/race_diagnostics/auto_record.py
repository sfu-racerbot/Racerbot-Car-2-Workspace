"""
auto_record.py

When bringup's automatic recorder records -- the decision only, with no
rclpy, so it is unit-testable unsourced like run_events.py
(test/test_auto_record.py). auto_record_node.py polls the ROS graph, hands
each snapshot to AutoRecorder.update(), and starts or stops the recording
it is told to.

The rules (docs/run-diagnostics.md#automatic-recording):

  - An *algorithm* is any node publishing on a trigger topic (/drive and the
    other command topics), except the ignored ones: bringup's own
    ackermann_mux and ackermann_to_vesc_node, which publish downstream
    command topics all the time, and a bag being played back. Whose code it
    is does not matter: ours and team A's both publish /drive.
  - Record while an algorithm runs, once it has been up for
    start_delay_sec. The delay lets a launch that brings its own recorder
    (map_course, auto_map_race_launch.py) get that recorder onto the graph
    first.
  - Never record while another recorder is on the graph (record_run.py's
    race_diag_node, any `ros2 bag record`), and stop if one appears. Two
    recorders on one run double the load and split the data.
  - Stop once no algorithm has been seen for stop_delay_sec.
  - If the recording exits on its own, do not start another until the
    algorithm has gone and come back, so a recorder that cannot run (a full
    disk, a broken install) cannot restart in a loop.
"""

from dataclasses import dataclass, field
from fnmatch import fnmatchcase

IDLE = 'idle'            # no algorithm, or waiting out another recorder
WAITING = 'waiting'      # algorithm seen, start_delay_sec not yet up
RECORDING = 'recording'
HELD = 'held'            # the recording died on its own; wait for a restart

# The nodes record_run.py starts, by their unprefixed names. record_run.py
# names them through recorder_node_names(), so this list and the launch file
# cannot drift apart. The automatic recorder runs record_run.py with
# AUTO_PREFIX, which is how it tells its own recording from anyone else's.
RECORDER_NODES = ('race_diag_node', 'snapshot_params', 'rosbag2_recorder')
AUTO_PREFIX = 'auto_'


def recorder_node_names(prefix=''):
    return tuple(prefix + name for name in RECORDER_NODES)


@dataclass(frozen=True)
class Settings:
    trigger_topics: tuple
    ignore_publishers: tuple = ()
    other_recorders: tuple = ()
    # Node names our own recording starts, which are never "another
    # recorder" even when they match other_recorders.
    own_nodes: tuple = ()
    start_delay_sec: float = 2.0
    stop_delay_sec: float = 3.0


def settings_from_params(params, own_nodes=()):
    """Settings from auto_record.yaml's ros__parameters (or the node's
    declared parameters), so the node and the tests read the shipped
    config through the same code."""
    return Settings(
        trigger_topics=tuple(params['trigger_topics']),
        ignore_publishers=tuple(params.get('ignore_publishers', ())),
        other_recorders=tuple(params.get('other_recorders', ())),
        own_nodes=tuple(own_nodes),
        start_delay_sec=float(params['start_delay_sec']),
        stop_delay_sec=float(params['stop_delay_sec']),
    )


@dataclass
class Decision:
    action: str          # 'start', 'stop' or '' (nothing to do)
    state: str           # the state after this update
    reason: str
    algorithms: dict = field(default_factory=dict)   # {topic: [node, ...]}
    recorders: list = field(default_factory=list)    # other recorders seen


def base_name(full_name):
    """'/ns/name' -> 'name'. Patterns match the node's own name, so a
    namespace does not hide a recorder or an ignored node."""
    return full_name.rstrip('/').rsplit('/', 1)[-1]


def _matches(name, patterns):
    return any(fnmatchcase(name, pattern) for pattern in patterns)


def algorithms(publishers, settings):
    """{topic: [full node name, ...]} -> {topic: sorted names}, keeping only
    trigger topics and the publishers that count as an algorithm."""
    found = {}
    for topic in settings.trigger_topics:
        names = sorted({
            name for name in publishers.get(topic, ())
            if not _matches(base_name(name), settings.ignore_publishers)})
        if names:
            found[topic] = names
    return found


def other_recorders(nodes, settings):
    """Every node on the graph that is someone else's recorder."""
    return sorted({
        name for name in nodes
        if base_name(name) not in settings.own_nodes
        and _matches(base_name(name), settings.other_recorders)})


def describe(found):
    """{topic: [node, ...]} -> 'node on /topic, ...', for logs and run notes."""
    return ', '.join(f"{'+'.join(names)} on {topic}"
                     for topic, names in sorted(found.items()))


class AutoRecorder:
    """The start/stop state machine. Feed it every graph poll; it returns
    what to do. ``now`` is any monotonic clock in seconds."""

    def __init__(self, settings):
        self.settings = settings
        self.state = IDLE
        self._waiting_since = None
        self._last_algorithm = None

    def update(self, now, publishers, nodes, recording_alive=True):
        s = self.settings
        found = algorithms(publishers, s)
        others = other_recorders(nodes, s)

        def decide(action, reason):
            return Decision(action, self.state, reason, found, others)

        if self.state == RECORDING:
            if not recording_alive:
                self.state = HELD
                return decide('stop', 'the recording exited on its own')
            if others:
                self.state = IDLE
                return decide('stop', f"another recorder started: {', '.join(others)}")
            if found:
                self._last_algorithm = now
                return decide('', 'recording')
            gone = now - self._last_algorithm
            if gone >= s.stop_delay_sec:
                self.state = IDLE
                return decide('stop', f'no algorithm publishing for {gone:.1f} s')
            return decide('', 'algorithm gone, waiting before stopping')

        if self.state == HELD:
            if not found:
                self.state = IDLE
                return decide('', 'algorithm gone; the next one will be recorded')
            return decide('', 'held: the last recording failed, waiting for a restart')

        # IDLE or WAITING
        if not found:
            self.state = IDLE
            return decide('', 'no algorithm')
        if others:
            self.state = IDLE
            return decide('', f"another recorder is running: {', '.join(others)}")
        if self.state == IDLE:
            self.state = WAITING
            self._waiting_since = now
        if now - self._waiting_since >= s.start_delay_sec:
            self.state = RECORDING
            self._last_algorithm = now
            return decide('start', describe(found))
        return decide('', 'algorithm seen, waiting before starting')
