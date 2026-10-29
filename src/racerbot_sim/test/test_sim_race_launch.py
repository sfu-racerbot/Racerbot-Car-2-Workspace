"""The two-car race launch, run for real: sim_race_launch.py on its own ROS domain.

Every test starts the actual launch file and watches the live graph,
/sim/status and the referee's race.json. Run it sourced and on an isolated
domain (the car's graph is domain 0, and this must never see it):

    RB_WS="$(git rev-parse --show-toplevel)" ~/bin/rb env ROS_DOMAIN_ID=78 \\
        python3 -m pytest src/racerbot_sim/test/test_sim_race_launch.py -v

Each launch is stopped through tools/racerbot_sim/harness_procs.py, which only
signals the process group this test started. The launch-level refusals (bad
opponent, bad track) run in a second or two; the two races take ~30 s each.

R6 acceptance measurement (2026-10-28, seed 12345, shipped YAML, recording on):
car1_course_2026_10_26, 3 laps, normal and swapped starts both timed out at
300 sim seconds with no winner or contacts. The front slot reached 1.916
laps, the trailer 0.949; orders were [1, 2] and [2, 1]. Both live CLI checks
of /car2/gap_follow_node max_speed returned 5.0. Artifacts: /tmp/oc-part-b1-r6/
normal/ and swapped/ (race.json, launch.log and recording/).

The deciding eval_gap_multi comparison (--workers 2, --swap-starts, --laps 3,
--set enable_steering_damping=false) finished both cars: normal 36.80/39.05 s,
swapped car 2 36.15 s then car 1 37.95 s. Shipped damping: normal 37.93/39.03 s,
swapped car 2 36.48 s then car 1 37.90 s. Fast output uses 0-based car indices.
Thus the orders agree but the winner/finishing verdict disagrees in both
lineups. ROS logs show swept-clearance caps falling to zero and 'no imu';
these observations do not establish the cause of the fidelity difference.
IMU simulation remains Part B3, outside R6; no controller settings were tuned.

Review round 1 mutation audit: stale separate poses failed the status-position
oracle before the fix; depth 10 and BEST_EFFORT each failed the status QoS test;
writing missed_steps=1 failed all three live race JSON checks; dropping probe
step 10 failed deadman completeness despite zero referee loss. Restoring broad
stray-process patterns failed the quoted-brief regression. All breaks restored.

R7 mutation audit: removing the brake remap or namespace push failed
test_team_a_graph_and_short_race; removing the joy remap failed
test_team_a_deadman_isolated[False] (car 2 reached 2.9999 m/s). Wiring the
car 2 hold argument to car 1 failed the same test (car 1 stayed at 0 m/s).
With the namespace removed the explicit drive remap still targets /car2/drive;
the graph check detects the nodes' incorrect root namespace. All breaks restored.

R7 acceptance races (2026-10-28): all four stopped on car 2 wall contact,
with zero missed steps and neither car finishing. Car 1 course normal/swapped:
4.950/2.925 sim s; indoor_wide normal/swapped: 4.050/2.850 sim s. Full logs,
race.json and recordings: /tmp/oc-part-b1-r7/<track>-<normal|swapped>/.

R7 review step 2: forcing car 2's hold false failed
test_team_a_deadman_isolated[True] at the movement assertion (peak 0 m/s).
Review step 3: removing the brake remap failed test_team_a_graph_is_isolated
waiting for the required endpoint; changing the referee lap target to 0.1
failed test_team_a_short_race_writes_result at its 0.2-lap assertion.
Review steps 1 and 5: swapping the hold arguments failed
test_team_a_car1_deadman_does_not_gate_car2 (car 1 peaked at 4.8927 m/s);
forcing car 2's hold true failed test_team_a_deadman_isolated[False] (3.0 m/s).
All review mutations were restored and all five team A cases passed together.
"""

import contextlib
import importlib.util
import json
import math
import os
import re
import subprocess
import sys
import time

import pytest
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'tools', 'racerbot_sim'))
import harness_procs  # noqa: E402
from conftest import require_isolation  # noqa: E402

SHIPPED_GAP_FOLLOW_YAML = os.path.join(ROOT, 'src', 'gap_follow', 'config', 'gap_follow.yaml')
# Our own nodes only: a leftover gym bridge or referee on this domain would
# publish a second /scan, /odom or /sim/status beside the race being run.
STRAY_PATTERNS = tuple(
    r'^([^ ]*/)?(python[0-9.]* +(-[^ ]+ +)*)?([^ ]*/)?' + name + r'( +|$)'
    for name in ('gym_bridge_node', 'race_referee_node', 'sim_joy_node'))
# The referee's graph footprint. Anything else on its publishers is a bug.
REFEREE_TOPICS = {'/sim/race_result', '/rosout', '/parameter_events'}
STATUS_QOS = QoSProfile(depth=5000, reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def _isolated_domain():
    require_isolation()


def _refuse_strays():
    try:
        harness_procs.refuse_if_strays(STRAY_PATTERNS)
    except SystemExit as refusal:
        pytest.fail(str(refusal))


def test_stray_patterns_match_node_executables_not_quoted_briefs():
    for name in ('gym_bridge_node', 'race_referee_node', 'sim_joy_node'):
        # oracle: process-boundary invariant: an installed node is refused,
        # but a parent shell merely quoting its name is not a running ROS node.
        for command in (name, f'/ws/install/racerbot_sim/lib/racerbot_sim/{name}',
                        f'/usr/bin/python3 /ws/install/racerbot_sim/lib/racerbot_sim/{name} --ros-args',
                        f'python3 -u /ws/{name} --ros-args'):
            assert any(re.search(pattern, command) for pattern in STRAY_PATTERNS), command
        assert not any(re.search(pattern, f'/bin/bash -c "review {name} now"')
                       for pattern in STRAY_PATTERNS)


def _tail(path, lines=60):
    with open(path, encoding='utf-8', errors='replace') as handle:
        return ''.join(handle.readlines()[-lines:])


@contextlib.contextmanager
def _race_launch(tmp_path, *arguments, car2_silence_after_sec=None):
    """Start sim_race_launch.py, yield its process, and stop its group on exit."""
    _isolated_domain()
    _refuse_strays()
    launch_command = ['ros2', 'launch', 'racerbot_sim', 'sim_race_launch.py']
    if car2_silence_after_sec is not None:
        # Scoped test-only parameter injection exercises the real launch and
        # sim_joy_node without adding a production launch control interface.
        params = tmp_path / 'car2_joy.yaml'
        params.write_text(yaml.safe_dump({'/car2/sim_joy_node': {'ros__parameters': {
            'silence_after_sec': float(car2_silence_after_sec)}}}), encoding='utf-8')
        wrapper = tmp_path / 'silent_joy.launch.py'
        wrapper.write_text(
            'from launch import LaunchDescription\n'
            'from launch.actions import IncludeLaunchDescription\n'
            'from launch.launch_description_sources import PythonLaunchDescriptionSource\n'
            'from launch_ros.actions import SetParametersFromFile\n'
            'def generate_launch_description():\n'
            f'    return LaunchDescription([SetParametersFromFile({str(params)!r}),\n'
            '        IncludeLaunchDescription(PythonLaunchDescriptionSource('
            f'{os.path.join(get_package_share_directory("racerbot_sim"), "launch", "sim_race_launch.py")!r}))])\n',
            encoding='utf-8')
        launch_command = ['ros2', 'launch', str(wrapper)]
    log_path = tmp_path / 'launch.log'
    with open(log_path, 'w', encoding='utf-8') as log:
        proc = subprocess.Popen(
            [*launch_command, *arguments,
             'drive_topic_prefix:=/test_only'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            yield proc
        finally:
            harness_procs.stop_group(proc)


class _Probe(Node):
    """Watches the graph and /sim/status. Subscribes only; publishes nothing."""

    def __init__(self):
        super().__init__('race_test_probe')
        self.status = []
        self.brakes = []
        self.car2_commands = []
        self.car2_joy = []
        self.create_subscription(String, '/sim/status', self._on_status, STATUS_QOS)
        self.create_subscription(Bool, '/car2/safety/brake', lambda msg: self.brakes.append(msg.data), 10)
        self.create_subscription(AckermannDriveStamped, '/test_only/car2/drive',
                                 lambda msg: self.car2_commands.append(
                                     (time.monotonic(), msg.drive.speed)), 1000)
        self.create_subscription(Joy, '/car2/joy',
                                 lambda msg: self.car2_joy.append(time.monotonic()), 1000)

    def _on_status(self, message):
        status = json.loads(message.data)
        status['_received_at'] = time.monotonic()
        self.status.append(status)


@pytest.fixture
def probe():
    _isolated_domain()
    rclpy.init()
    node = _Probe()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


class Graph:
    def __init__(self, node):
        self.node = node

    def spin_until(self, predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not predicate():
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return True

    def spin_for(self, seconds):
        self.spin_until(lambda: False, seconds)

    def publishers(self, topic):
        return sorted((e.node_namespace, e.node_name)
                      for e in self.node.get_publishers_info_by_topic(topic))

    def subscribers(self, topic):
        return sorted((e.node_namespace, e.node_name)
                      for e in self.node.get_subscriptions_info_by_topic(topic))


def _ros_param(node_name, parameter):
    done = subprocess.run(['ros2', 'param', 'get', node_name, parameter],
                          capture_output=True, text=True, timeout=60)
    match = re.search(r'value is: (.*)$', done.stdout, re.MULTILINE)
    assert match, f'ros2 param get {node_name} {parameter} said: {done.stdout!r} {done.stderr!r}'
    return match.group(1).strip()


def _parameters(probe, names):
    from rclpy.parameter_client import AsyncParameterClient
    client = AsyncParameterClient(probe, '/car2/gap_follow_node')
    assert client.wait_for_services(timeout_sec=10)
    future = client.get_parameters(names)
    rclpy.spin_until_future_complete(probe, future, timeout_sec=10)
    assert future.done(), 'namespaced node did not answer parameter request'
    return future.result().values


def _shipped_value(key):
    with open(SHIPPED_GAP_FOLLOW_YAML, encoding='utf-8') as handle:
        return yaml.safe_load(handle)['gap_follow_node']['ros__parameters'][key]


def _wired(graph):
    return (('/', 'gap_follow_node') in graph.publishers('/test_only/drive')
            and ('/car2', 'gap_follow_node') in graph.subscribers('/car2/joy')
            and ('/', 'race_referee_node') in graph.publishers('/sim/race_result'))


def test_short_race_keeps_each_car_in_its_own_namespace_and_finishes(tmp_path, probe):
    with _race_launch(tmp_path, 'track:=indoor_oval', 'opponent:=ours', 'laps:=0.2',
                      'record:=false', 'timeout_s:=60', f'output_dir:={tmp_path}') as proc:
        graph = Graph(probe)
        if not graph.spin_until(lambda: _wired(graph), 90):
            pytest.fail('the race never wired up. /drive publishers: '
                         f'{graph.publishers("/test_only/drive")}; car2 drive publishers: '
                         f'{graph.publishers("/test_only/car2/drive")}\n{_tail(tmp_path / "launch.log")}')
        graph.spin_for(2.0)  # let any late publisher appear before judging the graph

        # Car 1 owns the root names; car 2 owns only its own namespace.
        assert graph.publishers('/test_only/drive') == [('/', 'gap_follow_node')]
        assert graph.publishers('/test_only/car2/drive') == [('/car2', 'gap_follow_node')]
        # oracle: safety isolation -- no launched process publishes production drive names.
        for topic in ('/drive', '/car2/drive', '/ackermann_cmd', '/car2/ackermann_cmd'):
            assert graph.publishers(topic) == []
        for topic in ('/test_only/drive', '/test_only/ackermann_cmd', '/joy'):
            assert all(namespace != '/car2' for namespace, _ in graph.publishers(topic)), (
                f'a /car2 node publishes {topic}: {graph.publishers(topic)}')
        # Car 2's control layer hears car 2's sensors and car 2's deadman, and nothing of car 1's.
        assert ('/car2', 'gap_follow_node') in graph.subscribers('/car2/scan')
        assert ('/car2', 'gap_follow_node') in graph.subscribers('/car2/joy')
        assert ('/car2', 'gap_follow_node') not in graph.subscribers('/scan')
        assert ('/car2', 'gap_follow_node') not in graph.subscribers('/joy')
        assert ('/', 'gap_follow_node') in graph.subscribers('/scan')
        assert ('/', 'gap_follow_node') in graph.subscribers('/joy')

        # The shipped YAML must reach the namespaced node, and its drive topic must be car 2's.
        drive, speed = _parameters(probe, ['drive_topic', 'max_speed'])
        assert drive.string_value == '/test_only/car2/drive'
        assert speed.double_value == _shipped_value('max_speed')

        # The referee's footprint on the graph is its result topic and nothing that drives or senses.
        referee_topics = {topic for topic, _types in
                          graph.node.get_publisher_names_and_types_by_node('race_referee_node', '/')}
        assert referee_topics <= REFEREE_TOPICS, referee_topics

        assert proc.wait(timeout=150) == 0, _tail(tmp_path / 'launch.log')

    race = json.loads((tmp_path / 'race.json').read_text(encoding='utf-8'))
    assert race['track'] == 'indoor_oval'
    assert race['race_laps'] == 0.2
    assert len(race['laps']) == 2
    assert all(isinstance(lap, float) and math.isfinite(lap) for lap in race['laps'])
    assert sorted(race['finish_order']) == [1, 2]
    assert all(t is not None for t in race['finish_time_s']), race['finish_time_s']
    assert race['missed_steps'] == 0  # oracle: invariant, every simulator step was scored


def test_car2_without_lb_stays_still_while_car1_drives(tmp_path, probe):
    with _race_launch(tmp_path, 'track:=indoor_oval', 'laps:=0.2', 'record:=false',
                      'car2_hold_deadman:=false', 'timeout_s:=20', f'output_dir:={tmp_path}') as proc:
        graph = Graph(probe)
        deadline = time.monotonic() + 150
        while proc.poll() is None and time.monotonic() < deadline:
            graph.spin_for(0.2)
        assert proc.poll() is not None, 'the race did not end within its time limit'
        assert proc.returncode == 0, _tail(tmp_path / 'launch.log')

    race = json.loads((tmp_path / 'race.json').read_text(encoding='utf-8'))
    assert race['missed_steps'] == 0
    # Drain callbacks queued during process shutdown, through the scored final
    # step. The simulator may publish later steps before launch stops it.
    assert graph.spin_until(lambda: any(s['steps'] >= race['steps'] for s in probe.status), 5)
    statuses = [s for s in probe.status if s['steps'] <= race['steps']]
    # oracle: simulator starts at step 1; exact sequence proves every scored
    # step was observed, with neither duplicates nor holes hidden by filtering.
    assert [s['steps'] for s in statuses] == list(range(1, race['steps'] + 1))
    assert race['steps'] > 1
    assert all(len(s['cars']) == 2 for s in statuses)
    car1_speeds = [s['cars'][0]['speed'] for s in statuses]
    car2_speeds = [s['cars'][1]['speed'] for s in statuses]
    # Car 2 has no LB held, so its deadman refuses every drive command: it never moves.
    assert max(abs(speed) for speed in car2_speeds) < 0.05, 'car 2 moved without LB'
    # Car 1 holds LB and drives, so the car 2 result above is not a stalled simulator.
    assert max(car1_speeds) > 0.5, f'car 1 never got above 0.5 m/s: peak {max(car1_speeds):.3f}'
    assert race['finish_time_s'][0] is not None, 'car 1 should have finished 0.2 laps'
    assert race['finish_time_s'][1] is None, 'car 2 cannot finish 0.2 laps while it is stopped'
    assert race['timed_out'] is True


def test_integer_lap_argument_preserves_numeric_race_requirement(tmp_path):
    # oracle: spec, the documented laps:=3 CLI must reach the double ROS parameter.
    with _race_launch(tmp_path, 'track:=indoor_oval', 'laps:=3', 'record:=false',
                      'timeout_s:=1', f'output_dir:={tmp_path}') as proc:
        assert proc.wait(timeout=90) == 0, _tail(tmp_path / 'launch.log')
    assert (tmp_path / 'race.json').exists(), _tail(tmp_path / 'launch.log')
    race = json.loads((tmp_path / 'race.json').read_text())
    assert race['race_laps'] == 3.0
    assert race['timed_out'] is True
    assert race['sim_time_s'] == pytest.approx(1.0, abs=0.025)  # one simulator control step, seconds
    assert race['missed_steps'] == 0


@pytest.mark.parametrize('argument, expected', [
    ('opponent:=rival', "unknown opponent:='rival'"),
    ('track:=no_such_track', "track 'no_such_track'"),
])
def test_bad_arguments_fail_the_launch_with_a_clear_message(argument, expected):
    _isolated_domain()
    done = subprocess.run(['ros2', 'launch', 'racerbot_sim', 'sim_race_launch.py', argument],
                          capture_output=True, text=True, timeout=120)
    output = done.stdout + done.stderr
    assert done.returncode != 0, output[-2000:]
    assert expected in output, output[-2000:]
    if argument.startswith('track:='):
        assert os.path.join('tools', 'f1tenth_sim', 'maps') in output, output[-2000:]


@pytest.fixture
def team_a_packages():
    for package in ('disparity_extender', 'safety_node'):
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip(f'{package} is not installed; install it to run the team A integration tests')


def test_team_a_graph_is_isolated(tmp_path, probe, team_a_packages):
    with _race_launch(tmp_path, 'track:=indoor_oval', 'opponent:=team_a', 'laps:=0.2',
                      'record:=false', 'car1_hold_deadman:=false', 'car2_hold_deadman:=false',
                      'timeout_s:=60', f'output_dir:={tmp_path}') as proc:
        graph = Graph(probe)
        safety = ('/car2', 'safety_node')
        disparity = ('/car2', 'disparity_extender_node')
        required_subs = (('/car2/scan', safety), ('/car2/scan', disparity),
                         ('/car2/joy', safety), ('/car2/odom', safety),
                         ('/car2/safety/brake', disparity))
        def snapshot():
            return (
                tuple((t, tuple(graph.publishers(t))) for t in
                      ('/test_only/drive', '/test_only/car2/drive', '/safety/brake', '/car2/safety/brake')),
                tuple((t, tuple(graph.subscribers(t))) for t in
                      ('/car2/scan', '/car2/joy', '/car2/odom', '/car2/safety/brake',
                       '/scan', '/joy', '/ego_racecar/odom')),
                tuple((name, tuple(sorted(probe.get_publisher_names_and_types_by_node(name, namespace))))
                      for namespace, name in (safety, disparity)),
            )
        def ready():
            return (('/', 'gap_follow_node') in graph.publishers('/test_only/drive')
                    and disparity in graph.publishers('/test_only/car2/drive')
                    and safety in graph.publishers('/car2/safety/brake')
                    and all(node in graph.subscribers(topic) for topic, node in required_subs))
        assert graph.spin_until(lambda: proc.poll() is not None or ready(), 45), _tail(tmp_path / 'launch.log')
        assert proc.poll() is None, _tail(tmp_path / 'launch.log')
        # oracle: endpoint graph must be identical in two observations at least
        # 0.5 s apart, after all required endpoints have appeared.
        deadline = time.monotonic() + 15
        previous = snapshot()
        stable = False
        while time.monotonic() < deadline and proc.poll() is None:
            graph.spin_for(0.5)
            current = snapshot()
            if current == previous and ready():
                stable = True
                break
            previous = current
        assert stable, f'graph did not stabilize: {previous!r}\n{_tail(tmp_path / "launch.log")}'
        # oracle: race isolation contract: only car 1 publishes root drive and
        # every team A publisher belongs to car 2, except ROS infrastructure.
        assert graph.publishers('/test_only/drive') == [('/', 'gap_follow_node')]
        assert graph.publishers('/test_only/car2/drive') == [disparity]
        for topic in ('/drive', '/car2/drive', '/ackermann_cmd', '/car2/ackermann_cmd'):
            assert graph.publishers(topic) == []
        assert graph.publishers('/safety/brake') == []
        for namespace, name in (safety, disparity):
            topics = {topic for topic, _ in probe.get_publisher_names_and_types_by_node(name, namespace)}
            assert all(t.startswith('/car2/') or t in (
                '/test_only/car2/drive', '/rosout', '/parameter_events') for t in topics), topics
        for topic, node in required_subs:
            assert node in graph.subscribers(topic), (topic, graph.subscribers(topic))
        for topic in ('/scan', '/joy', '/ego_racecar/odom'):
            assert safety not in graph.subscribers(topic)
            assert disparity not in graph.subscribers(topic)


def test_team_a_short_race_writes_result(tmp_path, probe, team_a_packages):
    with _race_launch(tmp_path, 'track:=indoor_oval', 'opponent:=team_a', 'laps:=0.2',
                      'record:=false', 'timeout_s:=60', f'output_dir:={tmp_path}') as proc:
        graph = Graph(probe)
        assert graph.spin_until(lambda: proc.poll() is not None, 150), _tail(tmp_path / 'launch.log')
        assert proc.returncode == 0, _tail(tmp_path / 'launch.log')
    race = json.loads((tmp_path / 'race.json').read_text(encoding='utf-8'))
    # oracle: referee contract: both cars are scored and no steps are lost.
    assert race['track'] == 'indoor_oval'
    assert race['race_laps'] == 0.2
    assert len(race['laps']) == 2
    assert race['missed_steps'] == 0


@pytest.mark.parametrize('held', [False, True])
def test_team_a_deadman_isolated(tmp_path, probe, team_a_packages, held):
    with _race_launch(tmp_path, 'track:=indoor_oval', 'opponent:=team_a', 'laps:=0.2',
                      'record:=false', f'car2_hold_deadman:={str(held).lower()}',
                      'timeout_s:=20', f'output_dir:={tmp_path}') as proc:
        graph = Graph(probe)
        deadline = time.monotonic() + 150
        while proc.poll() is None and time.monotonic() < deadline:
            graph.spin_for(0.2)
        assert proc.poll() is not None, 'race did not end'
        assert proc.returncode == 0, _tail(tmp_path / 'launch.log')
    race = json.loads((tmp_path / 'race.json').read_text(encoding='utf-8'))
    assert graph.spin_until(lambda: any(s['steps'] >= race['steps'] for s in probe.status), 5)
    statuses = [s for s in probe.status if s['steps'] <= race['steps']]
    # oracle: race contract, every scored simulator step is observed once.
    assert race['missed_steps'] == 0
    assert [s['steps'] for s in statuses] == list(range(1, race['steps'] + 1))
    assert race['steps'] > 1
    assert all(len(s['cars']) == 2 for s in statuses)
    # oracle: R7 task brief in this conversation specifies these m/s thresholds;
    # task context: docs/superpowers/plans/2026-10-27-part-b1-racing.md, Task R7.
    assert max(s['cars'][0]['speed'] for s in statuses) > 0.5
    speeds = [abs(s['cars'][1]['speed']) for s in statuses]
    if held:
        assert max(speeds) > 0.5
        assert False in probe.brakes
    else:
        assert max(speeds) < 0.05


def test_team_a_car1_deadman_does_not_gate_car2(tmp_path, probe, team_a_packages):
    with _race_launch(tmp_path, 'track:=indoor_oval', 'opponent:=team_a', 'laps:=0.2',
                      'record:=false', 'swap_starts:=true', 'car1_hold_deadman:=false',
                      'timeout_s:=20', f'output_dir:={tmp_path}') as proc:
        graph = Graph(probe)
        deadline = time.monotonic() + 150
        while proc.poll() is None and time.monotonic() < deadline:
            graph.spin_for(0.2)
        assert proc.poll() is not None, 'race did not end'
        assert proc.returncode == 0, _tail(tmp_path / 'launch.log')
    race = json.loads((tmp_path / 'race.json').read_text(encoding='utf-8'))
    assert graph.spin_until(lambda: any(s['steps'] >= race['steps'] for s in probe.status), 5)
    statuses = [s for s in probe.status if s['steps'] <= race['steps']]
    # oracle: every scored step must be observed once, from the first step.
    assert race['missed_steps'] == 0
    assert race['steps'] > 1
    assert [s['steps'] for s in statuses] == list(range(1, race['steps'] + 1))
    assert all(len(s['cars']) == 2 for s in statuses)
    # oracle: R7 review request in this conversation specifies these m/s thresholds;
    # task context: docs/superpowers/plans/2026-10-27-part-b1-racing.md, Task R7.
    assert max(abs(s['cars'][0]['speed']) for s in statuses) < 0.05
    assert max(s['cars'][1]['speed'] for s in statuses) > 0.5


@pytest.mark.parametrize('silence_after', [0.0, 2.0], ids=['never_received', 'stale_after_motion'])
def test_team_a_missing_joy_denies_published_command_and_motion(
        tmp_path, probe, team_a_packages, silence_after):
    with _race_launch(tmp_path, 'track:=indoor_oval', 'opponent:=team_a', 'laps:=3',
                      'record:=false', 'timeout_s:=12', f'output_dir:={tmp_path}',
                      car2_silence_after_sec=0.0) as proc:
        graph = Graph(probe)
        with contextlib.ExitStack() as cleanup:
            if silence_after > 0.0:
                # Start the finite Joy window only after the bridge is stepping.
                # This real sim_joy_node retains its own 3 s hardware interlock.
                assert graph.spin_until(lambda: any(
                    s['cars'][0]['speed'] > 0.5 for s in probe.status), 45)
                log = cleanup.enter_context(open(tmp_path / 'car2_joy.log', 'w', encoding='utf-8'))
                joy = subprocess.Popen([
                    'ros2', 'run', 'racerbot_sim', 'sim_joy_node', '--ros-args',
                    '-r', '__ns:=/car2', '-r', '__node:=test_silent_joy',
                    '-p', 'joy_topic:=/car2/joy', '-p', f'silence_after_sec:={silence_after}',
                ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                cleanup.callback(harness_procs.stop_group, joy)
            assert graph.spin_until(lambda: proc.poll() is not None, 90), _tail(tmp_path / 'launch.log')
            assert proc.returncode == 0, _tail(tmp_path / 'launch.log')
    race = json.loads((tmp_path / 'race.json').read_text(encoding='utf-8'))
    assert graph.spin_until(lambda: any(s['steps'] >= race['steps'] for s in probe.status), 5)
    statuses = [s for s in probe.status if s['steps'] <= race['steps']]
    # oracle: invariant -- no missing steps may hide a denied car moving.
    assert race['missed_steps'] == 0
    assert [s['steps'] for s in statuses] == list(range(1, race['steps'] + 1))
    assert len(statuses) >= 100
    assert probe.car2_commands
    # oracle: invariant -- contact stop or early termination cannot supply the denial.
    assert race['timed_out'] is True
    assert not any(c['wall'] or c['car'] for c in race['contacts'])
    # oracle: permissive control -- car 1 holds its own LB and moves independently.
    assert max(s['cars'][0]['speed'] for s in statuses) > 0.5
    if silence_after == 0.0:
        assert probe.car2_joy == []
        commands = [speed for _, speed in probe.car2_commands]
        stopped = statuses
    else:
        assert probe.car2_joy
        last_joy = probe.car2_joy[-1]
        # oracle: the stale path must first have authorised actual motion.
        before = [s for s in statuses if s['_received_at'] <= last_joy]
        assert before and max(s['cars'][1]['speed'] for s in before) > 0.5
        # Team A safety_node's 50 ms watchdog + 150 ms callback/transport
        # budget. This budget is not a plant-speed tolerance: commands must be zero.
        commands = [speed for t, speed in probe.car2_commands if t >= last_joy + 0.2]
        # The plant needs finite braking time after a zero command. Require
        # at least one continuous second stopped, starting 1 s after Joy ends.
        stopped = [s for s in statuses if s['_received_at'] >= last_joy + 1.0]
        assert stopped and stopped[-1]['sim_time_s'] - stopped[0]['sim_time_s'] >= 1.0
    # oracle: CLAUDE.md LB policy, through disparity_extender + safety_node.
    assert commands and all(speed == 0.0 for speed in commands)
    assert stopped and max(abs(s['cars'][1]['speed']) for s in stopped) < 0.05


def test_referee_creates_only_the_result_publisher():
    # Static companion to the runtime check above. The brief asks for this grep.
    path = os.path.join(os.path.dirname(__file__), '..', 'racerbot_sim', 'race_referee_node.py')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()
    assert source.count('create_publisher(') == 1
    assert "'/sim/race_result'" in source
    for forbidden in ('/drive', '/ackermann_cmd', '/joy', '/scan', '/odom'):
        assert forbidden not in source


@pytest.mark.parametrize('check', ['no_writes', 'status_clock', 'status_qos'])
def test_referee_scores_each_new_status_immediately_without_writing_tracks(tmp_path, check):
    from geometry_msgs.msg import PoseStamped
    from racerbot_sim.race_referee_node import RaceRefereeNode
    from racerbot_sim import tracks

    _isolated_domain()
    rclpy.init(args=['--ros-args', '-p', f'track_directory:={tmp_path}',
                     '-p', f'output_dir:={tmp_path / "results"}'])
    node = None
    try:
        node = RaceRefereeNode()
        # oracle: ruling C, referee startup never creates track files.
        if check == 'no_writes':
            assert sorted(p.name for p in tmp_path.iterdir()) == ['results']
            return
        if check == 'status_qos':
            subscriptions = list(node.subscriptions)
            # oracle: review ruling 2, reliable backlog of at least 200 samples.
            assert [sub.topic_name for sub in subscriptions] == ['/sim/status']
            qos = subscriptions[0].qos_profile
            assert qos.reliability == QoSReliabilityPolicy.RELIABLE
            assert qos.depth >= 200
            return
        line = tracks.rounded_rectangle_centerline(5.0, 3.2, 1.8)
        # Deliver deliberately stale truth on the old separate pose topics.
        # oracle: review ruling 1, only the atomic status coordinates may be scored.
        publishers = [node.create_publisher(PoseStamped, topic, 10) for topic in
                      ('/sim/ground_truth_pose', '/sim/car2/ground_truth_pose')]
        stale = PoseStamped()
        stale.pose.position.x, stale.pose.position.y = map(float, line[0])
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            for publisher in publishers:
                publisher.publish(stale)
            rclpy.spin_once(node, timeout_sec=0.02)
        def offer(t, steps):
            node._status(String(data=json.dumps(dict(
                sim_time_s=t, steps=steps, cars=[dict(x=float(line[i * 10][0]),
                                        y=float(line[i * 10][1]), speed=1.0,
                                        wall_contact_now=False) for i in range(2)],
                car_contact_now=False))))
        offer(0.025, 1)
        # oracle: ruling B, the callback ticks immediately, even faster than wall time.
        assert node.referee.sim_time_s == 0.025
        assert node.referee.progress_m[1] == pytest.approx([0.5], abs=1e-9)  # metres along straight
        offer(0.05, 2)
        offer(0.075, 2)  # newer time cannot make a duplicate step a new sample
        offer(0.1, 1)
        assert node.referee.sim_time_s == 0.05
        assert [len(t) for t in node.referee.progress_m] == [2, 2]
        offer(0.125, 5)
        assert node.referee.result()['missed_steps'] == 2
        assert node.referee.sim_time_s == 0.125
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert [len(t) for t in node.referee.progress_m] == [3, 3]
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def _load_launch(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('check', ['arguments', 'recorder'])
def test_recorder_inherits_default_topics_and_launch_exposes_timeout(check):
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    race = _load_launch(os.path.join(ROOT, 'src/racerbot_sim/launch/sim_race_launch.py'), 'race_launch_test')
    record = _load_launch(os.path.join(get_package_share_directory('race_diagnostics'),
                                    'launch/record_run.py'), 'record_launch_test')
    context = LaunchContext()
    declarations = [a for a in race.generate_launch_description().entities
                    if isinstance(a, DeclareLaunchArgument)]
    for action in declarations:
        action.execute(context)
    # oracle: amended launch interface includes timeout, excludes unsolicited offsets.
    if check == 'arguments':
        assert 'timeout_s' in context.launch_configurations
        assert 'start_offsets' not in context.launch_configurations
        return
    actions = race._setup(context)
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 2  # bringup and recorder, as required by the launch contract
    recorder = includes[-1]
    topics = perform_substitutions(context, normalize_to_list_of_substitutions(
        dict(recorder.launch_arguments)['topics'])).split()
    # oracle: spec, every recorder default plus the specified second-car and truth topics.
    assert set(record.DEFAULT_TOPICS) <= set(topics)
    assert set('/car2/scan /car2/odom /car2/drive /car2/ackermann_cmd /car2/joy '
               '/car2/drive_intent /car2/teleop /car2/safety/brake /sim/ground_truth_pose '
               '/sim/car2/ground_truth_pose /sim/status /sim/race_result'.split()) <= set(topics)
