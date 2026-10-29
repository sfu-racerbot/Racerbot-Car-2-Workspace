"""
auto_record_node.py

Bringup's automatic run recorder. Every f1tenth_stack bringup starts it
(pass `record:=false` to leave it out). While an algorithm runs -- any node
publishing /drive or another trigger topic in config/auto_record.yaml, ours
or team A's -- it runs record_run.py into a new ~/racerbot-ws/data/racerbot_runs/<ts>/,
the same layout a hand-started recording writes, and stops it when the
algorithm goes. It never records beside another recorder.

The decisions are in auto_record.py (tested without ROS). This file only
reads the ROS graph and starts and stops the child process. It creates no
publisher and no subscription: the graph queries are the ones
`ros2 topic info` and `ros2 node list` make, so it cannot affect driving.

Each run also gets auto_record.yaml (what started it, what stopped it, the
nodes on the graph), metadata/processes.txt (the command line of every ROS
process, which names the launch files and params files that ran) and
recorder.log (record_run.py's own output).
"""

from datetime import datetime
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import yaml

from race_diagnostics import auto_record as ar


def _full(namespace, name):
    return namespace.rstrip('/') + '/' + name


def _dir_bytes(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


def _ros_processes():
    """Command lines of the ROS processes running now: which launch files,
    executables and params files make up this run."""
    try:
        done = subprocess.run(['ps', '-eo', 'pid,ppid,lstart,args', '--sort=start_time'],
                              capture_output=True, text=True, timeout=5.0)
    except (OSError, subprocess.TimeoutExpired) as e:
        return f'<ps failed: {e}>\n'
    lines = done.stdout.splitlines()
    keep = [line for line in lines[1:]
            if '--ros-args' in line or 'ros2 ' in line or '/opt/ros/' in line]
    return '\n'.join(lines[:1] + keep) + '\n'


class AutoRecordNode(Node):

    def __init__(self):
        super().__init__('auto_record')
        p = self.declare_parameter
        p('output_directory', '~/racerbot-ws/data/racerbot_runs')
        p('trigger_topics', ['/drive'])
        p('ignore_publishers', ['ackermann_mux'])
        p('other_recorders', ['race_diag_node', 'rosbag2_recorder*'])
        p('start_delay_sec', 3.0)
        p('stop_delay_sec', 3.0)
        p('poll_period_sec', 0.5)
        p('record_bag', True)
        p('topics', '')
        p('min_free_gb', 2.0)
        p('stop_timeout_sec', 15.0)

        def value(name):
            return self.get_parameter(name).value

        self.output_directory = Path(os.path.expanduser(value('output_directory')))
        self.record_bag = bool(value('record_bag'))
        self.topics = str(value('topics')).strip()
        self.min_free_gb = float(value('min_free_gb'))
        self.stop_timeout_sec = float(value('stop_timeout_sec'))
        self.settings = ar.settings_from_params(
            {name: value(name) for name in (
                'trigger_topics', 'ignore_publishers', 'other_recorders',
                'start_delay_sec', 'stop_delay_sec')},
            own_nodes=ar.recorder_node_names(ar.AUTO_PREFIX))
        self.machine = ar.AutoRecorder(self.settings)

        self.child = None          # the record_run.py launch process
        self.run_dir = None
        self.run_info = None
        self.started_at = None     # monotonic
        self.stopping_since = None
        self.stop_reason = None
        self._last_status = None

        self.create_timer(float(value('poll_period_sec')), self._tick)
        self.get_logger().info(
            f"auto_record ready: records a run while any node publishes "
            f"{', '.join(self.settings.trigger_topics)} (manual /teleop is not "
            f"recorded), into {self.output_directory}/<time>/; never beside "
            f"another recorder ({', '.join(self.settings.other_recorders)}). "
            f"Turn off with bringup record:=false.")

    # ------------------------------------------------------------ graph

    def _graph(self):
        publishers = {
            topic: [_full(info.node_namespace, info.node_name)
                    for info in self.get_publishers_info_by_topic(topic)]
            for topic in self.settings.trigger_topics}
        nodes = [_full(namespace, name)
                 for name, namespace in self.get_node_names_and_namespaces()]
        return publishers, nodes

    def _tick(self):
        if self.stopping_since is not None:
            self._wait_for_stop(block=False)
            return
        publishers, nodes = self._graph()
        alive = self.child is not None and self.child.poll() is None
        decision = self.machine.update(time.monotonic(), publishers, nodes,
                                       recording_alive=alive)
        if decision.action == 'start':
            self._start(decision, nodes)
        elif decision.action == 'stop':
            self._stop(decision.reason)
        else:
            self._log_status(decision)

    def _log_status(self, decision):
        # Only what someone at the bringup terminal needs to know about: an
        # algorithm that is NOT being recorded, and why.
        if decision.algorithms and decision.recorders:
            status = ('deferring', tuple(decision.recorders))
            text = (f"{ar.describe(decision.algorithms)} is running, but "
                    f"another recorder is on the graph "
                    f"({', '.join(decision.recorders)}): not recording it")
        elif decision.state == ar.HELD:
            status = ('held',)
            text = ('the last recording failed; the next algorithm started '
                    'will be recorded')
        else:
            status = None
            text = None
        if status != self._last_status and text:
            self.get_logger().info(text)
        self._last_status = status

    # ---------------------------------------------------------- recording

    def _new_run_dir(self):
        stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
        run_dir = self.output_directory / stamp
        n = 2
        while run_dir.exists():
            run_dir = self.output_directory / f'{stamp}-{n}'
            n += 1
        run_dir.mkdir(parents=True)
        return run_dir

    def _write_info(self):
        try:
            (self.run_dir / 'auto_record.yaml').write_text(
                yaml.safe_dump(self.run_info, sort_keys=False))
        except OSError as e:
            self.get_logger().warn(f'could not write auto_record.yaml: {e}')

    def _start(self, decision, nodes):
        self._last_status = None
        self.run_info = None
        try:
            self.output_directory.mkdir(parents=True, exist_ok=True)
            free_gb = shutil.disk_usage(self.output_directory).free / 1e9
            if free_gb < self.min_free_gb:
                self.get_logger().error(
                    f'not recording {ar.describe(decision.algorithms)}: only '
                    f'{free_gb:.1f} GB free under {self.output_directory} '
                    f'(min_free_gb {self.min_free_gb:.1f}). Delete old runs.')
                return
            self.run_dir = self._new_run_dir()
            self.run_info = {
                'started_by': 'auto_record (bringup); turn off with record:=false',
                'started': datetime.now().astimezone().isoformat(timespec='seconds'),
                'algorithms': decision.algorithms,
                'nodes': sorted(nodes),
            }
            self._write_info()
            meta = self.run_dir / 'metadata'
            meta.mkdir(exist_ok=True)
            (meta / 'processes.txt').write_text(_ros_processes())

            cmd = ['ros2', 'launch', 'race_diagnostics', 'record_run.py',
                   f'run_directory:={self.run_dir}',
                   f'name_prefix:={ar.AUTO_PREFIX}',
                   f'record_bag:={str(self.record_bag).lower()}']
            if self.topics:
                cmd.append(f'topics:={self.topics}')
            # If this process dies without stopping the recording (SIGKILL),
            # the kernel sends the recorder SIGINT, which closes the bag
            # cleanly instead of leaving it recording with nobody to stop it.
            if shutil.which('setpriv'):
                cmd = ['setpriv', '--pdeathsig', 'INT', '--'] + cmd
            with open(self.run_dir / 'recorder.log', 'w') as log:
                # Its own session: bringup's Ctrl+C reaches this node, which
                # stops the recording in order, and a last-resort kill of the
                # group can never touch bringup.
                self.child = subprocess.Popen(
                    cmd, stdout=log, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, start_new_session=True)
            self.started_at = time.monotonic()
        except OSError as e:
            self.get_logger().error(f'could not start recording: {e}')
            if self.run_info is not None:
                self.run_info['error'] = f'could not start recording: {e}'
                self._write_info()
            self.child = None
            return
        self.get_logger().info(
            f'recording {ar.describe(decision.algorithms)} -> {self.run_dir}')

    def _stop(self, reason):
        self.stop_reason = reason
        if self.child is None:
            return
        if self.child.poll() is None:
            self.get_logger().info(f'stopping the recording: {reason}')
            self.child.send_signal(signal.SIGINT)
            self.stopping_since = time.monotonic()
        else:
            self._finish(killed=False)

    def _wait_for_stop(self, block):
        waited = time.monotonic() - self.stopping_since
        remaining = self.stop_timeout_sec - waited
        if block and remaining > 0.0:
            try:
                self.child.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                pass
        if self.child.poll() is None:
            if time.monotonic() - self.stopping_since < self.stop_timeout_sec:
                return
            self.get_logger().error(
                f'the recording did not stop within {self.stop_timeout_sec:.0f} s; '
                f'killing it (the bag may need `ros2 bag reindex`)')
            try:
                os.killpg(self.child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.child.wait(timeout=5.0)
            self._finish(killed=True)
            return
        self._finish(killed=False)

    def _finish(self, killed):
        duration = time.monotonic() - self.started_at
        bag_bytes = _dir_bytes(self.run_dir / 'bag')
        self.run_info.update({
            'stopped': datetime.now().astimezone().isoformat(timespec='seconds'),
            'stop_reason': self.stop_reason,
            'duration_sec': round(duration, 1),
            'recorder_exit_code': self.child.returncode,
            'recorder_killed': killed,
            'bag_bytes': bag_bytes,
        })
        self._write_info()
        self.get_logger().info(
            f'saved run {self.run_dir} ({duration:.0f} s, bag '
            f'{bag_bytes / 1e6:.0f} MB): {self.stop_reason}. Analyse with: '
            f'python3 ~/racerbot-ws/tools/run_analysis/analyze_run.py {self.run_dir}')
        self.child = None
        self.run_dir = None
        self.run_info = None
        self.stopping_since = None

    def shutdown(self, reason):
        """Stop any recording in order before the node exits."""
        if self.child is None:
            return
        if self.stopping_since is None:
            self._stop(reason)
        if self.child is not None:
            self._wait_for_stop(block=True)


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Signals are handled here rather than by rclpy, whose handler shuts the
    # context down at once: the node must still be able to log while it
    # closes a recording in progress.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _interrupt)
    node = AutoRecordNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Bringup's launch follows its SIGINT with SIGTERM; neither may cut
        # short closing the bag. A SIGKILL still can, and then setpriv's
        # death signal closes it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.shutdown('bringup shut down')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
