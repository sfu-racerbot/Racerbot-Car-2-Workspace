"""
auto_record_node.py

Bringup's automatic run recorder. Every f1tenth_stack bringup starts it
(pass `record:=false` to leave it out). While an algorithm runs -- any node
publishing /drive or another trigger topic in config/auto_record.yaml, ours
or team A's -- it records into a new ~/racerbot-ws/data/racerbot_runs/<ts>/,
the same layout a hand-started recording writes, and stops when the algorithm
goes. It never records beside another recorder.

It starts `ros2 bag record` itself and, once that is listening, record_run.py
with record_bag:=false for everything else. Through record_run.py the bag took
5.4 to 5.9 s to subscribe to its topics, against 1.4 to 2.6 s alone (measured
2026-10-31), and started side by side they compete for the CPU. Those seconds are
the start of the run, which a bag cannot get back.

The decisions are in auto_record.py (tested without ROS). This file only
reads the ROS graph and starts and stops the child process. It creates no
publisher and no subscription: the graph queries are the ones
`ros2 topic info` and `ros2 node list` make, so it cannot affect driving.

Each run also gets auto_record.yaml (what started it, what stopped it, the
nodes on the graph), metadata/processes.txt (the command line of every ROS
process, which names the launch files and params files that ran) and
recorder.log (the recorders' own output). Afterwards run_activity.py adds what
the car did to auto_record.yaml. Every decision also goes in auto_record.log
beside the runs, including the refusals that leave no run behind.
"""

from datetime import datetime
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import yaml

from race_diagnostics import auto_record as ar
from race_diagnostics.recording import bag_record_command, DEFAULT_TOPICS


# record_run.py waits for the bag to be listening, but not forever: a run with
# nothing to subscribe to still deserves its metadata.
START_THE_REST_AFTER_SEC = 5.0
# `ros2 launch` signalled while still starting its processes can hang until it is
# killed (seen 2026-10-31), which would wrongly mark the recording killed. So it
# is left to get going first: until race_diag_node, its first process, has written
# events.jsonl, or this long. The bag is stopped at once either way.
LAUNCH_SETTLE_SEC = 8.0


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
        p('live_topics', ['/scan', '/odom', '/drive', '/joy'])
        p('topics', '')
        p('min_free_gb', 2.0)
        p('stop_timeout_sec', 15.0)

        def value(name):
            return self.get_parameter(name).value

        self.output_directory = Path(os.path.expanduser(value('output_directory')))
        self.record_bag = bool(value('record_bag'))
        self.topics = str(value('topics')).split() or list(DEFAULT_TOPICS)
        self.live_topics = [topic for topic in value('live_topics') if topic in self.topics]
        self.min_free_gb = float(value('min_free_gb'))
        self.stop_timeout_sec = float(value('stop_timeout_sec'))
        self.settings = ar.settings_from_params(
            {name: value(name) for name in (
                'trigger_topics', 'ignore_publishers', 'other_recorders',
                'start_delay_sec', 'stop_delay_sec')},
            own_nodes=ar.recorder_node_names(ar.AUTO_PREFIX))
        self.machine = ar.AutoRecorder(self.settings)

        self.bag_node = ar.recorder_node_names(ar.AUTO_PREFIX)[2]
        self.child = None          # the record_run.py launch process
        self.bag = None            # the `ros2 bag record` process
        self.rest_cmd = None       # record_run.py's command, until it is started
        self.child_started_at = None
        self.child_signal_due = False
        self.run_dir = None
        self.run_info = None
        self.started_at = None     # monotonic
        self.stopping_since = None
        self.stop_reason = None
        self.live_pending = False
        self._last_status = None

        self.create_timer(float(value('poll_period_sec')), self._tick)
        self._note(
            'info',
            f"auto_record ready: records a run while any node publishes "
            f"{', '.join(self.settings.trigger_topics)} (manual /teleop is not "
            f"recorded), into {self.output_directory}/<time>/; never beside "
            f"another recorder ({', '.join(self.settings.other_recorders)}). "
            f"Turn off with bringup record:=false.")

    # -------------------------------------------------------------- notes

    def _note(self, level, text):
        """Log a decision, and keep it where it stays with the runs. Node logs
        are ~/.ros/log/python3_<pid>_*.log, and a refusal leaves no run to
        explain it. Losing this line must never cost a recording."""
        # Two lines: rclpy refuses one call site logging at two severities.
        if level == 'error':
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)
        try:
            self.output_directory.mkdir(parents=True, exist_ok=True)
            with open(self.output_directory / 'auto_record.log', 'a') as log:
                now = datetime.now().astimezone().isoformat(timespec='seconds')
                log.write(f'{now} {level.upper()} {text}\n')
        except OSError as e:
            self.get_logger().warn(f'could not write auto_record.log: {e}')

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
        running = self._processes()
        alive = bool(running) and all(proc.poll() is None for proc in running)
        decision = self.machine.update(time.monotonic(), publishers, nodes,
                                       recording_alive=alive)
        if decision.action == 'start':
            self._start(decision, nodes)
        elif decision.action == 'stop':
            self._stop(decision.reason)
        else:
            self._log_status(decision)
            self._check_live()
            self._start_the_rest()

    def _processes(self):
        return [proc for proc in (self.bag, self.child) if proc is not None]

    def _start_the_rest(self):
        """record_run.py, once the bag no longer needs the CPU to itself."""
        if self.rest_cmd is None:
            return
        waited = time.monotonic() - self.started_at
        if self.live_pending and waited < START_THE_REST_AFTER_SEC:
            return
        cmd, self.rest_cmd = self.rest_cmd, None
        try:
            self.child = self._spawn(cmd)
            self.child_started_at = time.monotonic()
        except OSError as e:
            self._note('error', f'could not start record_run.py: {e}')
            self.run_info['error'] = f'could not start record_run.py: {e}'
            self._write_info()

    def _check_live(self):
        """Say when the bag has really subscribed, read off the graph, so the
        driver knows a press of LB from now on is in the recording."""
        if self.bag is None or self.live_pending is False or self.bag.poll() is not None:
            return
        # The graph can still list the last run's recorder, under the same name
        # on the same topics, for a moment after it exits. Only this run's bag
        # process creates this directory, as it opens the bag.
        if not (self.run_dir / 'bag').exists():
            return
        published = [t for t in self.live_topics if self.count_publishers(t) > 0]
        subscribed = [t for t in self.live_topics
                      if any(sub.node_name == self.bag_node
                             for sub in self.get_subscriptions_info_by_topic(t))]
        if not subscribed or ar.waiting_on(published, subscribed):
            return
        self.live_pending = False
        after = time.monotonic() - self.machine.algorithm_seen_at
        self.run_info['bag_live_sec'] = round(after, 1)
        self._write_info()
        self._note('info', f"recording live: {' '.join(sorted(subscribed))} are in the bag, "
                           f'{after:.1f} s after the algorithm appeared')

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
            self._note('info', text)
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

    def _spawn(self, cmd):
        # If this process dies without stopping the recording (SIGKILL),
        # the kernel sends the recorder SIGINT, which closes the bag
        # cleanly instead of leaving it recording with nobody to stop it.
        if shutil.which('setpriv'):
            cmd = ['setpriv', '--pdeathsig', 'INT', '--'] + cmd
        with open(self.run_dir / 'recorder.log', 'a') as log:
            # Its own session: bringup's Ctrl+C reaches this node, which
            # stops the recording in order, and a last-resort kill of the
            # group can never touch bringup.
            return subprocess.Popen(
                cmd, stdout=log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True)

    def _start(self, decision, nodes):
        self._last_status = None
        self.run_info = None
        self.live_pending = self.record_bag
        try:
            self.output_directory.mkdir(parents=True, exist_ok=True)
            free_gb = shutil.disk_usage(self.output_directory).free / 1e9
            if free_gb < self.min_free_gb:
                self._note(
                    'error',
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
                   'record_bag:=false']
            self.started_at = time.monotonic()
            # The bag first and alone: every moment before it is listening is lost.
            self.rest_cmd = cmd
            if self.record_bag:
                self.bag = self._spawn(bag_record_command(
                    self.run_dir / 'bag', self.bag_node, self.topics))
            else:
                self._start_the_rest()
        except OSError as e:
            self._note('error', f'could not start recording: {e}')
            if self.run_info is not None:
                self.run_info['error'] = f'could not start recording: {e}'
                self._write_info()
            self.bag = None
            self.child = None
            self.rest_cmd = None
            return
        self._note('info', f'recording {ar.describe(decision.algorithms)} -> {self.run_dir}')

    def _stop(self, reason):
        self.stop_reason = reason
        processes = self._processes()
        if not processes:
            return
        running = [proc for proc in processes if proc.poll() is None]
        if running:
            self._note('info', f'stopping the recording: {reason}')
            if self.bag is not None and self.bag.poll() is None:
                self.bag.send_signal(signal.SIGINT)
            self.child_signal_due = self.child is not None and self.child.poll() is None
            self.stopping_since = time.monotonic()
            self._signal_launch_when_settled()
        else:
            self._finish(killed=False)

    def _signal_launch_when_settled(self):
        if not self.child_signal_due:
            return
        settled = ((self.run_dir / 'events.jsonl').exists()
                   or time.monotonic() - self.child_started_at >= LAUNCH_SETTLE_SEC)
        if settled:
            self.child_signal_due = False
            if self.child.poll() is None:
                self.child.send_signal(signal.SIGINT)

    def _wait_for_stop(self, block):
        end = self.stopping_since + self.stop_timeout_sec
        self._signal_launch_when_settled()
        while block and time.monotonic() < end and any(
                proc.poll() is None for proc in self._processes()):
            time.sleep(0.1)
            self._signal_launch_when_settled()
        running = [proc for proc in self._processes() if proc.poll() is None]
        if running:
            if time.monotonic() - self.stopping_since < self.stop_timeout_sec:
                return
            self._note(
                'error',
                f'the recording did not stop within {self.stop_timeout_sec:.0f} s; '
                f'killing it (the bag may need `ros2 bag reindex`)')
            for proc in running:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=5.0)
            self._finish(killed=True)
            return
        self._finish(killed=False)

    def _failures(self):
        """Every recording process that failed, since launch's own exit code
        stays 0 when one of its children dies."""
        try:
            failures = ar.recorder_failures(
                (self.run_dir / 'recorder.log').read_text(errors='replace'))
        except OSError:
            failures = []
        if self.bag is not None and self.bag.returncode != 0:
            failures.append({'process': 'ros2 bag record', 'exit_code': self.bag.returncode})
        return failures

    def _start_activity_report(self):
        """What the car did during the run, read from the closed bag. Detached
        and at the lowest priority: it must never compete with the next run."""
        if not (self.run_dir / 'bag' / 'metadata.yaml').exists():
            return
        try:
            with open(self.run_dir / 'recorder.log', 'a') as log:
                subprocess.Popen(
                    ['nice', '-n', '19', sys.executable, '-m',
                     'race_diagnostics.run_activity', str(self.run_dir)],
                    stdout=log, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, start_new_session=True)
        except OSError as e:
            self.get_logger().warn(f'could not start run_activity: {e}')

    def _finish(self, killed):
        duration = time.monotonic() - self.started_at
        bag_bytes = _dir_bytes(self.run_dir / 'bag')
        self.run_info.update({
            'stopped': datetime.now().astimezone().isoformat(timespec='seconds'),
            'stop_reason': self.stop_reason,
            'duration_sec': round(duration, 1),
            'recorder_exit_code': self.child.returncode if self.child else None,
            'bag_exit_code': self.bag.returncode if self.bag else None,
            'recorder_failures': self._failures(),
            'recorder_killed': killed,
            'bag_bytes': bag_bytes,
        })
        if ar.is_superseded(self.stop_reason, duration):
            self.run_info['superseded'] = True
        self._write_info()
        self._start_activity_report()
        self._note(
            'info',
            f'saved run {self.run_dir} ({duration:.0f} s, bag '
            f'{bag_bytes / 1e6:.0f} MB): {self.stop_reason}. Analyse with: '
            f'python3 ~/racerbot-ws/tools/run_analysis/analyze_run.py {self.run_dir}')
        self.child = None
        self.bag = None
        self.rest_cmd = None
        self.child_signal_due = False
        self.run_dir = None
        self.run_info = None
        self.stopping_since = None

    def shutdown(self, reason):
        """Stop any recording in order before the node exits."""
        if not self._processes():
            return
        if self.stopping_since is None:
            self._stop(reason)
        if self._processes():
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
