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
    assert mc.output_directory(['laps:=4']) == Path(os.path.expanduser('~/racerbot-ws/data/racerbot_auto'))


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
        assert "'output_directory': '~/racerbot-ws/data/racerbot_runs'" in text, name


def test_simulator_runs_get_imu_hardware_false_unless_imu_hardware_was_given():
    # oracle: spec -- map_course.launch_arguments docstring; approved 2026-10-29
    # (I1 part b): simulated IMU stays on, only its hardware readers stay off.
    assert mc.launch_arguments(['laps:=4'], simulated=True) == ['laps:=4', 'imu_hardware:=false']
    assert mc.launch_arguments(['imu_hardware:=true'], simulated=True) == ['imu_hardware:=true']
    assert mc.launch_arguments(['imu:=false'], simulated=True) == ['imu:=false', 'imu_hardware:=false']
    assert mc.launch_arguments(['laps:=4'], simulated=False) == ['laps:=4']
    assert mc.launch_arguments([], simulated=False) == []


def test_main_in_the_simulator_launches_without_imu_hardware(tmp_path, monkeypatch, capsys):
    # oracle: spec -- map_course.launch_arguments docstring; approved 2026-10-29
    # (I1 part b): the real launch receives imu_hardware:=false, keeping the IMU.
    monkeypatch.setattr(mc, 'LOG_DIRECTORY', str(tmp_path / 'logs'))
    out = tmp_path / 'runs'
    args_file = tmp_path / 'args'
    bin_dir = _fake_ros2(tmp_path, f'echo "$@" > {args_file}\nexit 0\n')
    monkeypatch.setenv('PATH', f'{bin_dir}{os.pathsep}{os.environ["PATH"]}')
    _drivers(monkeypatch, 'gym_bridge_node')
    assert mc.main([f'output_directory:={out}']) == 0
    assert args_file.read_text().split() == [
        'launch', 'racerbot_launch', 'map_course_launch.py',
        f'output_directory:={out}', 'imu_hardware:=false']


def _analysis_fixture(tmp_path, monkeypatch, recorder=True, fail=False):
    run = tmp_path / 'recorded run'
    run.mkdir()
    maps = tmp_path / 'maps'
    events = tmp_path / 'events'
    args = tmp_path / 'args'
    root = tmp_path / 'workspace'
    scripts = root / 'tools' / 'run_analysis'
    scripts.mkdir(parents=True)
    for name in ('imu_report', 'slam_report'):
        (scripts / f'{name}.py').write_text(
            'import sys\n'
            f'open({str(events)!r}, "a").write({name!r} + " " + repr(sys.argv[1:]) + "\\n")\n'
            f'print({name!r})\n'
            'print("\\n".join("line " + str(i) for i in range(20)))\n')
    bin_dir = _fake_ros2(tmp_path, (
        'if [ "$1" = run ]; then\n'
        f'  echo "summarize_run $4" >> "{events}"\n'
        '  echo summary-headline\n'
        f'  exit {7 if fail else 0}\n'
        'fi\n'
        f'printf "%s\\n" "$@" > "{args}"\n'
        f'mkdir -p "{maps}/latest"\n'
        f'echo RACE-COMMANDS > "{maps}/latest/commands.txt"\n'
        + (f'echo "Recording run to: {run}"\n' if recorder else '') +
        'exit 3\n'))
    monkeypatch.setenv('PATH', f'{bin_dir}{os.pathsep}{os.environ["PATH"]}')
    monkeypatch.setenv('RACERBOT_WS', str(root))
    monkeypatch.setattr(mc, 'LOG_DIRECTORY', str(tmp_path / 'logs'))
    _drivers(monkeypatch, 'gym_bridge_node')
    return run, maps, events, args, root


def test_analysis_reports_order_arguments_headlines_and_full_output(tmp_path, monkeypatch, capsys):
    # oracle: Task J1a specifies order, 15-line headlines, and commands last.
    run, maps, events, args, root = _analysis_fixture(tmp_path, monkeypatch)
    assert mc.main([f'output_directory:={maps}']) == 3
    printed = capsys.readouterr().out
    log = Path(printed.strip().split('Full log of this run: ')[-1])
    lines = events.read_text().splitlines()
    assert lines == [f'summarize_run {run}', f'imu_report {[str(run)]!r}',
                     f'slam_report {[str(run), "--log", str(log)]!r}']
    for name in ('imu_report', 'slam_report'):
        target = run / 'analysis' / 'map_course' / f'{name}.txt'
        assert target.read_text().splitlines() == [name] + [f'line {i}' for i in range(20)]
        assert '\n'.join(target.read_text().splitlines()[:15]) in printed
        assert 'line 14' not in printed
        assert str(target) in printed
        assert printed.index(str(target)) < printed.index('RACE-COMMANDS')
    assert 'summary-headline' in printed
    assert printed.index('summary-headline') < printed.index('RACE-COMMANDS')
    assert 'RACE-COMMANDS' in log.read_text()


def test_analysis_disabled_is_stripped_and_runs_no_reports(tmp_path, monkeypatch, capsys):
    # oracle: Task J1a: wrapper-only option and no subprocess reports.
    run, maps, events, args, root = _analysis_fixture(tmp_path, monkeypatch)
    assert mc.main([f'output_directory:={maps}', 'analysis:=false']) == 3
    assert 'analysis:=false' not in args.read_text()
    assert not events.exists()
    assert not (run / 'analysis').exists()


def test_analysis_invalid_values_never_launch(tmp_path, monkeypatch, capsys):
    # oracle: Task J1a: only true/false; empty is invalid too.
    run, maps, events, args, root = _analysis_fixture(tmp_path, monkeypatch)
    for value in ('maybe', ''):
        assert mc.main([f'analysis:={value}']) == 2
        assert 'analysis' in capsys.readouterr().err
        assert not args.exists()


def test_analysis_without_recording_reports_skip_and_keeps_exit(tmp_path, monkeypatch, capsys):
    # oracle: Task J1a: absent recorder banner skips analysis and preserves launch status.
    run, maps, events, args, root = _analysis_fixture(tmp_path, monkeypatch, recorder=False)
    assert mc.main([f'output_directory:={maps}']) == 3
    assert 'Analysis skipped' in capsys.readouterr().out
    assert not events.exists()


def test_analysis_failure_reports_command_without_changing_launch_status(tmp_path, monkeypatch, capsys):
    # oracle: Task J1a: report failures are advisory.
    run, maps, events, args, root = _analysis_fixture(tmp_path, monkeypatch, fail=True)
    assert mc.main([f'output_directory:={maps}']) == 3
    output = capsys.readouterr().out
    assert 'exit 7' in output
    assert f"ros2 run race_diagnostics summarize_run '{run}'" in output
    assert 'slam_report' in output


def _interrupt_analysis_at(boundary, tmp_path, monkeypatch, capsys):
    import builtins
    import pytest
    run, maps, events, _, _ = _analysis_fixture(tmp_path, monkeypatch)
    write_text, print_text = Path.write_text, builtins.print
    interrupted = []

    def write(path, *args, **kwargs):
        if boundary == 'save' and path.name == 'summarize_run.txt':
            interrupted.append('save')
            raise KeyboardInterrupt
        return write_text(path, *args, **kwargs)

    def emit(*args, **kwargs):
        if boundary == 'print' and args and str(args[0]).startswith('summarize_run:'):
            interrupted.append('print')
            raise KeyboardInterrupt
        return print_text(*args, **kwargs)

    monkeypatch.setattr(Path, 'write_text', write)
    monkeypatch.setattr(builtins, 'print', emit)
    try:
        code = mc.main([f'output_directory:={maps}'])
    except KeyboardInterrupt:
        pytest.fail(f'analysis {boundary} interrupt escaped before final commands')
    output = capsys.readouterr().out
    # oracle: spec R5a: keep launch status and final commands; skip all
    # remaining analyses after an interrupt anywhere in run/save/print.
    assert code == 3
    assert interrupted == [boundary]
    assert events.read_text().splitlines() == [f'summarize_run {run}']
    assert 'RACE-COMMANDS' in output
    body, _, last = output.rstrip().rpartition('\n')
    assert body.rstrip().endswith('RACE-COMMANDS')
    assert last.startswith('Full log of this run: ')
    log = Path(output.strip().split('Full log of this run: ')[-1])
    assert 'RACE-COMMANDS' in log.read_text()


def test_analysis_save_interrupt_preserves_commands_and_launch_status(tmp_path, monkeypatch, capsys):
    _interrupt_analysis_at('save', tmp_path, monkeypatch, capsys)


def test_analysis_print_interrupt_preserves_commands_and_launch_status(tmp_path, monkeypatch, capsys):
    _interrupt_analysis_at('print', tmp_path, monkeypatch, capsys)


def test_simulator_probe_matches_executable_not_shell_text(monkeypatch):
    import re
    from types import SimpleNamespace

    for command, expected in [
        ('/ws/install/racerbot_sim/lib/racerbot_sim/gym_bridge_node --ros-args', True),
        ('/usr/bin/python3 /ws/install/racerbot_sim/lib/racerbot_sim/gym_bridge_node', True),
        ('/bin/bash -c wait-for gym_bridge_node', False),
        ('/bin/bash -c wait-for /ws/install/racerbot_sim/lib/racerbot_sim/gym_bridge_node', False),
    ]:
        def probe(argv, **kwargs):
            assert argv[:2] == ['pgrep', '-f']
            return SimpleNamespace(returncode=0 if re.search(argv[2], command) else 1)
        monkeypatch.setattr(mc.subprocess, 'run', probe)
        # oracle: spec R5b: only an executable/script process argument counts.
        assert mc._running(mc.SIM_DRIVER) is expected, command
