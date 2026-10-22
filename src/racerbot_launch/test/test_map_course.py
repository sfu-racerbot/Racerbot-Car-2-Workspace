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


def test_running_drivers_mean_no_second_bringup_unless_asked():
    assert mc.with_bringup_choice(['laps:=3'], running=True) == [
        'laps:=3', 'include_bringup:=false']
    assert mc.with_bringup_choice(['laps:=3'], running=False) == ['laps:=3']
    assert mc.with_bringup_choice(['include_bringup:=true'], running=True) == [
        'include_bringup:=true']


def test_main_skips_its_own_bringup_when_drivers_are_up(tmp_path, monkeypatch, capsys):
    out = tmp_path / 'runs'
    args_file = tmp_path / 'args'
    bin_dir = _fake_ros2(tmp_path, f'echo "$@" > {args_file}\nexit 0\n')
    monkeypatch.setenv('PATH', f'{bin_dir}{os.pathsep}{os.environ["PATH"]}')
    monkeypatch.setattr(mc, 'drivers_running', lambda: True)
    mc.main([f'output_directory:={out}'])
    assert args_file.read_text().split()[-1] == 'include_bringup:=false'
    assert 'already running' in capsys.readouterr().out


def test_main_passes_arguments_through_and_prints_the_commands_last(
        tmp_path, monkeypatch, capsys):
    out = tmp_path / 'runs'
    args_file = tmp_path / 'args'
    bin_dir = _fake_ros2(tmp_path, (
        f'echo "$@" > {args_file}\n'
        f'mkdir -p {out}/20261022-120000\n'
        f'echo "RACE COMMANDS HERE" > {out}/20261022-120000/commands.txt\n'
        'echo "[INFO] nodes shutting down"\n'
        'exit 0\n'))
    monkeypatch.setenv('PATH', f'{bin_dir}{os.pathsep}{os.environ["PATH"]}')
    monkeypatch.setattr(mc, 'drivers_running', lambda: False)
    code = mc.main([f'output_directory:={out}', 'laps:=4'])
    assert code == 0
    assert args_file.read_text().split() == [
        'launch', 'racerbot_launch', 'map_course_launch.py',
        f'output_directory:={out}', 'laps:=4']
    printed = capsys.readouterr().out
    assert printed.rstrip().endswith('RACE COMMANDS HERE')


def test_main_reports_no_map_and_the_launch_exit_code(tmp_path, monkeypatch, capsys):
    out = tmp_path / 'runs'
    bin_dir = _fake_ros2(tmp_path, 'exit 3\n')
    monkeypatch.setenv('PATH', f'{bin_dir}{os.pathsep}{os.environ["PATH"]}')
    monkeypatch.setattr(mc, 'drivers_running', lambda: False)
    assert mc.main([f'output_directory:={out}']) == 3
    assert 'No map was saved' in capsys.readouterr().out


def test_main_refuses_arguments_that_are_not_launch_arguments(capsys):
    assert mc.main(['--laps', '3']) == 2
    assert 'name:=value' in capsys.readouterr().err
