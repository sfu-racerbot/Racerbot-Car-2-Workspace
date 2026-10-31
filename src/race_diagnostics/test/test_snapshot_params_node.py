"""Real parameter services and snapshot process; run with package pytest under rb."""
import os
import signal
import subprocess
import threading
import time

import pytest
import yaml

rclpy = pytest.importorskip('rclpy', reason='requires ROS Jazzy and a built workspace')
if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0'):
    os.environ['ROS_DOMAIN_ID'] = '79'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402


def wait_for_file(path, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if path.exists() and path.stat().st_size:
            return True
        time.sleep(0.1)
    return False


def test_late_node_restart_namespace_and_sigterm(tmp_path):
    context = rclpy.Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    stop = threading.Event()
    nodes = []

    def spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    out = tmp_path / 'params'
    proc = subprocess.Popen(['ros2', 'run', 'race_diagnostics', 'snapshot_params',
                             '--output', str(out), '--delay', '0', '--poll', '0.5'],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, start_new_session=True)

    def add(namespace, marker):
        node = Node('late_node', namespace=namespace, context=context)
        node.declare_parameter('marker', marker)
        nodes.append(node)
        executor.add_node(node)
        return node

    try:
        time.sleep(3.0)  # The node appears after the watcher has started.
        first = add('/b23s1', 41.5)
        base = out / 'b23s1__late_node.yaml'
        # oracle: first appearance filename and parameter value declared above.
        assert wait_for_file(base), 'late node was not snapshotted'
        assert yaml.safe_load(base.read_text())['/b23s1/late_node']['ros__parameters']['marker'] == 41.5
        executor.remove_node(first)
        first.destroy_node()
        nodes.remove(first)
        time.sleep(2.0)  # Allow the watcher to observe at least one absent poll.
        add('/b23s1', 42.5)
        add('/b23s1/dup', 43.5)
        restart = out / 'b23s1__late_node.1.yaml'
        twin = out / 'b23s1__dup__late_node.yaml'
        # oracle: restart suffix, namespace encoding and declared parameter values.
        assert wait_for_file(restart), 'restart was not snapshotted'
        assert wait_for_file(twin), 'namespaced twin was not snapshotted'
        assert yaml.safe_load(restart.read_text())['/b23s1/late_node']['ros__parameters']['marker'] == 42.5
        assert yaml.safe_load(base.read_text())['/b23s1/late_node']['ros__parameters']['marker'] == 41.5
        assert yaml.safe_load(twin.read_text())['/b23s1/dup/late_node']['ros__parameters']['marker'] == 43.5
        # ros2 run is a wrapper; signal its executable, not the wrapper itself.
        # pgrep -P, not /proc/<pid>/task/<pid>/children: car 2's Tegra kernel
        # is built without CONFIG_PROC_CHILDREN, so that file does not exist.
        child_pids = subprocess.run(['pgrep', '-P', str(proc.pid)], capture_output=True,
                                    text=True).stdout.split()
        assert len(child_pids) == 1, child_pids
        os.kill(int(child_pids[0]), signal.SIGTERM)
        stdout, stderr = proc.communicate(timeout=10.0)
        # oracle: SIGTERM clean shutdown returns zero without a traceback.
        assert proc.returncode == 0, (stdout, stderr)
        assert 'Traceback' not in stderr
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.communicate(timeout=10.0)
        stop.set()
        thread.join(timeout=5.0)
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown(context=context)


def test_default_delay_snapshots_a_node_during_a_short_run(tmp_path):
    # recorded measurement: runs 20261030-182504 (12.5 s) and -183053 (15 s) have
    # an empty metadata/params/ because the old default waited 15 s. By shutdown
    # the algorithm has gone, so the snapshot has to be taken early.
    context = rclpy.Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    node = Node('short_run_node', context=context)
    node.declare_parameter('marker', 7.5)
    executor.add_node(node)
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    out = tmp_path / 'params'
    proc = subprocess.Popen(['ros2', 'run', 'race_diagnostics', 'snapshot_params',
                             '--output', str(out)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)
    try:
        path = out / 'short_run_node.yaml'
        # Under the old delay's 15 s, with room for a slow start and DDS discovery.
        assert wait_for_file(path, timeout=12.0), 'no snapshot within a short run'
        # oracle: the value declared above.
        assert yaml.safe_load(path.read_text())['/short_run_node']['ros__parameters']['marker'] == 7.5
    finally:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=10.0)
        stop.set()
        thread.join(timeout=5.0)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)
