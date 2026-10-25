"""The map_course wrapper (scripts/map_course). No ROS needed:

    python3 -m pytest src/racerbot_launch/test/test_map_course.py -v

The one process boundary -- the ``ros2`` executable -- is replaced by a fake
script on PATH that behaves like a finished (or abandoned) mapping run, so
main() is exercised end to end.
"""
import importlib.machinery
import importlib.util
import os
import stat
import time
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'map_course'


def _load():
    loader = importlib.machinery.SourceFileLoader('map_course', str(SCRIPT))
    spec = importlib.util.spec_from_loader('map_course', loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


mc = _load()


def _run_dir(parent, name, text, mtime=None):
    d = parent / name
    d.mkdir(parents=True)
    f = d / 'commands.txt'
    f.write_text(text)
    if mtime is not None:
        os.utime(f, (mtime, mtime))
    return d


def test_new_run_picks_the_newest_commands_written_since_start(tmp_path):
    now = time.time()
    _run_dir(tmp_path, 'old', 'old', mtime=now - 100)
    _run_dir(tmp_path, 'a', 'a', mtime=now + 1)
    newest = _run_dir(tmp_path, 'b', 'b', mtime=now + 2)
    (tmp_path / 'no_commands').mkdir()
    assert mc.new_run(tmp_path, now) == newest
    assert mc.new_run(tmp_path, now + 3) is None        # nothing that new
    assert mc.new_run(tmp_path / 'missing', now) is None


def test_output_directory_argument_is_honoured():
    assert mc.output_directory(['laps:=4', 'output_directory:=/x/y']) == Path('/x/y')
    assert mc.output_directory(['laps:=4']) == Path(os.path.expanduser('~/.ros/racerbot_auto'))


def test_new_run_is_the_newest_whatever_order_the_directory_lists(tmp_path):
    # Ten runs written since start; the newest is neither first nor last by
    # name, so picking by listing order instead of by time is caught.
    now = time.time()
    stamps = [now + k for k in (3, 7, 1, 9, 5, 10, 2, 8, 4, 6)]
    dirs = [_run_dir(tmp_path, f'run{k:02d}', str(k), mtime=t)
            for k, t in enumerate(stamps)]
    assert mc.new_run(tmp_path, now) == dirs[stamps.index(max(stamps))]
    assert sorted(p.name for p in tmp_path.iterdir())[-1] != dirs[5].name


def test_report_without_a_new_run_says_nothing_was_saved(tmp_path):
    assert 'No map was saved' in mc.report(tmp_path, time.time())


def _fake_ros2(tmp_path, body):
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    fake = bin_dir / 'ros2'
    fake.write_text('#!/bin/sh\n' + body)
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    return bin_dir


def _drivers(monkeypatch, *running):
    """Pretend exactly these driver processes are running (pgrep is the
    process boundary)."""
    monkeypatch.setattr(mc, '_running', lambda pattern: pattern in running)


def test_hardware_counts_as_up_only_with_every_real_driver_or_the_sim(monkeypatch):
    _drivers(monkeypatch, 'vesc_driver_node', 'urg_node_driver')
    assert mc.missing_drivers() == []
    _drivers(monkeypatch, 'vesc_driver_node')
    assert mc.missing_drivers() == ['urg_node_driver']
    _drivers(monkeypatch)
    assert mc.missing_drivers() == ['vesc_driver_node', 'urg_node_driver']
    _drivers(monkeypatch, 'gym_bridge_node')
    assert mc.missing_drivers() == []


def test_refuses_without_bringup_and_never_launches_anything(tmp_path, monkeypatch, capsys):
    called = tmp_path / 'called'
    bin_dir = _fake_ros2(tmp_path, f'touch {called}\nexit 0\n')
    monkeypatch.setenv('PATH', f'{bin_dir}{os.pathsep}{os.environ["PATH"]}')
    _drivers(monkeypatch, 'vesc_driver_node')          # LiDAR missing
    assert mc.main([f'output_directory:={tmp_path / "runs"}']) == 2
    assert not called.exists()
    err = capsys.readouterr().err
    assert 'ros2 launch f1tenth_stack bringup_launch.py' in err
    assert 'urg_node_driver' in err


def test_the_launch_file_starts_no_drivers():
    # Static oracle: map_course_launch.py must not include bringup in any
    # form, so a second copy of the drivers can never start beside it.
    text = (SCRIPT.parents[1] / 'launch' / 'map_course_launch.py').read_text()
    code = '\n'.join(line.split('#', 1)[0] for line in text.splitlines())
    for forbidden in ("'bringup_launch.py')", 'include_bringup', 'vesc_driver', 'urg_node'):
        assert forbidden not in code, forbidden


def test_main_passes_arguments_through_and_prints_the_commands_last(
        tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(mc, 'LOG_DIRECTORY', str(tmp_path / 'logs'))
    out = tmp_path / 'runs'
    args_file = tmp_path / 'args'
    bin_dir = _fake_ros2(tmp_path, (
        f'echo "$@" > {args_file}\n'
        f'mkdir -p {out}/20261022-120000\n'
        f'echo "RACE COMMANDS HERE" > {out}/20261022-120000/commands.txt\n'
        'echo "[INFO] nodes shutting down"\n'
        'exit 0\n'))
    monkeypatch.setenv('PATH', f'{bin_dir}{os.pathsep}{os.environ["PATH"]}')
    _drivers(monkeypatch, 'vesc_driver_node', 'urg_node_driver')
    code = mc.main([f'output_directory:={out}', 'laps:=4'])
    assert code == 0
    assert args_file.read_text().split() == [
        'launch', 'racerbot_launch', 'map_course_launch.py',
        f'output_directory:={out}', 'laps:=4']
    printed = capsys.readouterr().out
    # The commands are the last thing that matters on screen; only the log
    # path follows them.
    body, _, last = printed.rstrip().rpartition('\n')
    assert body.rstrip().endswith('RACE COMMANDS HERE')
    assert last.startswith('Full log of this run: ')
    log = Path(last.split(': ', 1)[1])
    saved = log.read_text()
    assert '[INFO] nodes shutting down' in saved and 'RACE COMMANDS HERE' in saved


def test_main_reports_no_map_and_the_launch_exit_code(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(mc, 'LOG_DIRECTORY', str(tmp_path / 'logs'))
    out = tmp_path / 'runs'
    bin_dir = _fake_ros2(tmp_path, 'exit 3\n')
    monkeypatch.setenv('PATH', f'{bin_dir}{os.pathsep}{os.environ["PATH"]}')
    _drivers(monkeypatch, 'vesc_driver_node', 'urg_node_driver')
    assert mc.main([f'output_directory:={out}']) == 3
    assert 'No map was saved' in capsys.readouterr().out


def test_main_refuses_arguments_that_are_not_launch_arguments(capsys):
    assert mc.main(['--laps', '3']) == 2
    assert 'name:=value' in capsys.readouterr().err


def test_the_recorder_gets_its_own_folder_not_the_map_run_folder():
    # Launch arguments are shared with included files: without an explicit
    # output_directory the recorder wrote into ~/.ros/racerbot_auto/ (found
    # on the car, 2026-10-22).
    for name in ('map_course_launch.py', 'auto_map_race_launch.py'):
        text = (SCRIPT.parents[1] / 'launch' / name).read_text()
        assert "'output_directory': '~/.ros/racerbot_runs'" in text, name


def test_simulator_runs_get_imu_false_unless_imu_was_given():
    # oracle: spec -- map_course_launch.py docstring: no IMU in the simulator.
    assert mc.launch_arguments(['laps:=4'], simulated=True) == ['laps:=4', 'imu:=false']
    assert mc.launch_arguments(['imu:=true'], simulated=True) == ['imu:=true']
    assert mc.launch_arguments(['laps:=4'], simulated=False) == ['laps:=4']
    assert mc.launch_arguments([], simulated=False) == []


def test_main_in_the_simulator_launches_without_the_imu(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(mc, 'LOG_DIRECTORY', str(tmp_path / 'logs'))
    out = tmp_path / 'runs'
    args_file = tmp_path / 'args'
    bin_dir = _fake_ros2(tmp_path, f'echo "$@" > {args_file}\nexit 0\n')
    monkeypatch.setenv('PATH', f'{bin_dir}{os.pathsep}{os.environ["PATH"]}')
    _drivers(monkeypatch, 'gym_bridge_node')
    assert mc.main([f'output_directory:={out}']) == 0
    assert args_file.read_text().split() == [
        'launch', 'racerbot_launch', 'map_course_launch.py',
        f'output_directory:={out}', 'imu:=false']
