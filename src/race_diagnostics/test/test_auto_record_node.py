"""
The real auto_record executable on a real ROS graph: it records while an
algorithm publishes, closes the bag when it goes, defers to another recorder,
and closes a recording in progress when bringup shuts down. Needs ROS
sourced and the workspace built:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python3 -m pytest src/race_diagnostics/test/test_auto_record_node.py -v

Never on the car's graph: if ROS_DOMAIN_ID is unset or 0 this file moves
itself to domain 79, localhost only, before any node starts. The fake
algorithm publishes on /test_only/drive, never /drive (CLAUDE.md, "Build,
test, run"), and the node under test is told to trigger on that topic.

Oracles: the rules in docs/run-diagnostics.md#automatic-recording, and the
bag itself, read back with rosbag2_py (a recording must contain the
messages the fake algorithm published). About a minute on the Jetson.
"""
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import yaml

rclpy = pytest.importorskip(
    'rclpy', reason='needs ROS sourced (CLAUDE.md "Build, test, run"); runs '
                    'whenever rclpy imports')

if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0'):
    os.environ['ROS_DOMAIN_ID'] = '79'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

TOPIC = '/test_only/drive'
CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'auto_record.yaml'


class Graph:
    """Fake algorithms and recorders, spun in a background thread."""

    def __init__(self):
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.nodes = {}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        while not self._stop.is_set():
            self.executor.spin_once(timeout_sec=0.05)

    def add_algorithm(self, name='fake_algorithm'):
        node = Node(name, context=self.context)
        pub = node.create_publisher(AckermannDriveStamped, TOPIC, 10)
        node.sent = 0

        def tick():
            msg = AckermannDriveStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            node.sent += 1
        node.create_timer(0.05, tick)
        self._add(name, node)
        return node

    def add_recorder(self, name='race_diag_node'):
        self._add(name, Node(name, context=self.context))

    def _add(self, name, node):
        self.nodes[name] = node
        self.executor.add_node(node)

    def remove(self, name):
        node = self.nodes.pop(name)
        self.executor.remove_node(node)
        node.destroy_node()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=5.0)
        for name in list(self.nodes):
            self.remove(name)
        rclpy.shutdown(context=self.context)


@pytest.fixture
def graph():
    g = Graph()
    yield g
    g.close()


@pytest.fixture
def watcher(tmp_path):
    """The real auto_record node, triggering on TOPIC, fast delays."""
    out = tmp_path / 'runs'
    cmd = ['ros2', 'run', 'race_diagnostics', 'auto_record', '--ros-args',
           '--params-file', str(CONFIG),
           '-p', f'output_directory:={out}',
           '-p', f"trigger_topics:=['{TOPIC}']",
           '-p', 'start_delay_sec:=1.0', '-p', 'stop_delay_sec:=1.0',
           '-p', 'poll_period_sec:=0.2',
           '-p', f'topics:={TOPIC} /rosout']
    log = open(tmp_path / 'watcher.log', 'w')
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    proc.out = out
    proc.log = tmp_path / 'watcher.log'
    proc.groups = []   # recorder process groups seen, for cleanup
    time.sleep(3.0)    # the node is up and polling
    yield proc
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=25.0)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
    log.close()
    # Last-resort cleanup only; every test asserts there were no leftovers.
    for group in proc.groups:
        try:
            os.killpg(group, signal.SIGKILL)
        except ProcessLookupError:
            pass
    for pid, _args in _processes_mentioning(out):
        os.kill(pid, signal.SIGKILL)


def _ps(fields):
    done = subprocess.run(['ps', '-eo', fields], capture_output=True, text=True)
    return [line.split(None, fields.count(',')) for line in done.stdout.splitlines()[1:]]


def _processes_mentioning(text):
    return [(int(pid), args) for pid, args in _ps('pid,args')
            if str(text) in args and int(pid) != os.getpid()]


def recorder_group(watcher, run):
    """The process group of the recording of `run`. record_run.py is started
    in its own session, so its group holds every process it starts --
    including race_diag_node, whose command line never names the run."""
    for pid, _args in _processes_mentioning(f'run_directory:={run}'):
        group = os.getpgid(pid)
        watcher.groups.append(group)
        return group
    return None


def leftovers(run, group):
    """Every process still alive from the recording of `run`."""
    alive = {pid for pid, _args in _processes_mentioning(run)}
    alive |= {int(pid) for pid, pgid in _ps('pid,pgid') if int(pgid) == group}
    return sorted(alive)


def runs(out):
    return sorted(p for p in out.glob('2*') if p.is_dir()) if out.exists() else []


def info(run):
    path = run / 'auto_record.yaml'
    return yaml.safe_load(path.read_text()) if path.exists() else {}


def wait_for(predicate, timeout, period=0.2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(period)
    return predicate()


def bag_count(run, topic):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    n = 0
    while reader.has_next():
        reader.read_next()
        n += 1
    return n


def test_records_while_an_algorithm_runs_and_closes_the_bag_when_it_goes(graph, watcher):
    assert runs(watcher.out) == [], 'recorded with no algorithm running'
    algo = graph.add_algorithm()
    assert wait_for(lambda: len(runs(watcher.out)) == 1, timeout=10.0), \
        watcher.log.read_text()
    run = runs(watcher.out)[0]
    assert info(run)['algorithms'] == {TOPIC: ['/fake_algorithm']}
    time.sleep(6.0)    # record_run.py takes ~3 s to get its bag going
    group = recorder_group(watcher, run)
    assert group is not None, 'no record_run.py process for this run'
    sent = algo.sent
    graph.remove('fake_algorithm')

    assert wait_for(lambda: 'stopped' in info(run), timeout=25.0), watcher.log.read_text()
    done = info(run)
    assert done['stop_reason'].startswith('no algorithm publishing for')
    assert done['recorder_exit_code'] == 0 and done['recorder_killed'] is False
    assert (run / 'bag' / 'metadata.yaml').exists(), 'bag was not closed cleanly'
    recorded = bag_count(run, TOPIC)
    # The bag starts a few seconds after the algorithm; it must hold most
    # of what was published after that, and never more than was sent.
    assert 40 <= recorded <= sent, (recorded, sent)
    assert (run / 'metadata' / 'git' / 'racerbot-ws.txt').exists()
    assert (run / 'events.jsonl').exists()
    assert leftovers(run, group) == [], 'recorder processes left running'
    assert len(runs(watcher.out)) == 1


def test_defers_to_another_recorder_and_resumes_when_it_leaves(graph, watcher):
    graph.add_recorder('race_diag_node')        # e.g. a record_run.py started by hand
    time.sleep(1.0)
    graph.add_algorithm()
    time.sleep(5.0)                               # well past start_delay_sec
    assert runs(watcher.out) == [], 'recorded beside another recorder'
    assert 'another recorder is on the graph (/race_diag_node)' in watcher.log.read_text()

    graph.remove('race_diag_node')
    assert wait_for(lambda: len(runs(watcher.out)) == 1, timeout=10.0), \
        watcher.log.read_text()
    run = runs(watcher.out)[0]
    time.sleep(4.0)
    group = recorder_group(watcher, run)
    assert group is not None

    graph.add_recorder('rosbag2_recorder')       # a `ros2 bag record` appears mid-run
    assert wait_for(lambda: 'stopped' in info(run), timeout=25.0), watcher.log.read_text()
    assert info(run)['stop_reason'] == 'another recorder started: /rosbag2_recorder'
    assert (run / 'bag' / 'metadata.yaml').exists()
    time.sleep(3.0)
    assert len(runs(watcher.out)) == 1, 'restarted beside the other recorder'
    assert leftovers(run, group) == []


def test_bringup_shutdown_closes_a_recording_in_progress(graph, watcher):
    graph.add_algorithm()
    assert wait_for(lambda: len(runs(watcher.out)) == 1, timeout=10.0)
    run = runs(watcher.out)[0]
    time.sleep(6.0)
    group = recorder_group(watcher, run)
    assert group is not None
    os.killpg(watcher.pid, signal.SIGINT)        # bringup's Ctrl+C
    watcher.wait(timeout=25.0)
    done = info(run)
    assert done['stop_reason'] == 'bringup shut down'
    assert done['recorder_exit_code'] == 0
    assert (run / 'bag' / 'metadata.yaml').exists(), 'bag was not closed cleanly'
    assert bag_count(run, TOPIC) > 0
    assert leftovers(run, group) == [], 'recorder outlived bringup'
    assert 'Failed to publish log message' not in watcher.log.read_text()


def test_a_killed_watcher_does_not_leave_its_recorder_running(graph, watcher):
    graph.add_algorithm()
    assert wait_for(lambda: len(runs(watcher.out)) == 1, timeout=10.0)
    run = runs(watcher.out)[0]
    time.sleep(6.0)
    group = recorder_group(watcher, run)
    assert group is not None
    os.killpg(watcher.pid, signal.SIGKILL)       # no chance to stop anything
    watcher.wait(timeout=5.0)
    assert wait_for(lambda: leftovers(run, group) == [], timeout=25.0), \
        _ps('pid,pgid,args')
    assert (run / 'bag' / 'metadata.yaml').exists(), 'bag was not closed cleanly'
