"""System recording contract; run with pytest src/race_diagnostics/test/."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def system_log():
    from race_diagnostics import system_log as module
    return module


def stat_line(pid, ticks, threads, start, comm='node'):
    # oracle: proc(5), fields 3 through 22, with utime at 14.
    fields = ['S'] + ['0'] * 19
    fields[11] = str(ticks)
    fields[12] = '0'
    fields[17] = str(threads)
    fields[19] = str(start)
    return f'{pid} ({comm}) ' + ' '.join(fields)


def process(root, pid, argv, ticks=100, threads=3, start=10, rss=123):
    directory = root / str(pid)
    directory.mkdir(exist_ok=True)
    (directory / 'cmdline').write_bytes(bytes([0]).join(a.encode() for a in argv) + bytes([0]))
    (directory / 'stat').write_text(stat_line(pid, ticks, threads, start))
    (directory / 'status').write_text(f'Name: node\nVmRSS: {rss} kB\n')


def fake_tegrastats(tmp_path):
    script = tmp_path / 'fake_tegrastats'
    pidfile = tmp_path / 'tegra.pid'
    script.write_text('#!/bin/sh\n' + f'echo $$ > "{pidfile}"\n' +
                      'while :; do echo sample; sleep 0.1; done\n')
    script.chmod(0o755)
    return script, pidfile


def assert_gone(pidfile):
    pid = int(pidfile.read_text())
    # oracle: kill(pid, 0) reports ESRCH once the child was reaped.
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)


def test_node_name_remaps():
    node_name = system_log().node_name
    argv = ['python3', '/x/gap_follow_node', '--ros-args', '-r',
            '__node:=gap_follow_node', '--params-file', 'a.yaml']
    # oracle: launch_ros remap syntax in the task interface.
    assert node_name(argv) == '/gap_follow_node'
    assert node_name(argv + ['-r', '__ns:=/car2']) == '/car2/gap_follow_node'
    assert node_name(['p', '--ros-args', '--remap', '__node:=x']) == '/x'
    assert node_name(['p', '--ros-args', '-r', 'slam_toolbox:__node:=slam_toolbox']) == '/slam_toolbox'
    assert node_name(['p', '--ros-args', '-r', '__ns:=/car2']) is None
    assert node_name(['p', '--ros-args', '-r', '__node:=x', '-r', '__ns:=/']) == '/x'
    assert node_name([]) is None
    assert node_name(['p', '-r', '__node:=x']) is None


def test_parse_stat_comm_and_truncation():
    parse_stat = system_log().parse_stat
    line = stat_line(1, 120, 7, 42, 'a) b')
    fields = line.rsplit(')', 1)[1].split()
    fields[12] = '30'
    line = line.rsplit(')', 1)[0] + ') ' + ' '.join(fields)
    # oracle: proc(5) fields 14, 15 and 20; 120 + 30 ticks.
    assert parse_stat(line) == (150, 7)
    with pytest.raises(ValueError):
        parse_stat('1 (x) S')


def test_parse_rss():
    # oracle: proc(5) VmRSS is expressed in kB; absent means unknown.
    assert system_log().parse_rss_kb('Name: x\nVmRSS:\t  12345 kB\n') == 12345
    assert system_log().parse_rss_kb('Name: x\n') is None


def test_cpu_percent_and_nonpositive_time():
    cpu = system_log().cpu_percent
    # oracle: 150 ticks / 100 Hz / 1.5 seconds = one core.
    assert cpu(1000, 1150, 1.5, 100) == 100.0
    for dt in (0, -1):
        with pytest.raises(ValueError):
            cpu(1000, 1150, dt, 100)


def test_sampler_cpu_disappearance_and_pid_reuse(tmp_path):
    process(tmp_path, 10, ['p', '--ros-args', '-r', '__node:=a'], rss=100)
    process(tmp_path, 20, ['p', '--ros-args', '-r', '__node:=b', '-r', '__ns:=/car'], rss=200)
    process(tmp_path, 30, ['p'])
    now = [10.0]
    sampler = system_log().ProcSampler(tmp_path, clk_tck=100, clock=lambda: now[0], wall=lambda: 42.0)
    # oracle: constructed proc fields and first observation has no CPU baseline.
    assert sampler.sample() == [
        {'t': 42.0, 'pid': 10, 'node': '/a', 'cpu_pct': None, 'rss_kb': 100, 'threads': 3},
        {'t': 42.0, 'pid': 20, 'node': '/car/b', 'cpu_pct': None, 'rss_kb': 200, 'threads': 3}]
    now[0] += 2.0
    process(tmp_path, 10, ['p', '--ros-args', '-r', '__node:=a'], ticks=150, rss=100)
    process(tmp_path, 20, ['p', '--ros-args', '-r', '__node:=b', '-r', '__ns:=/car'], ticks=400, rss=200)
    records = sampler.sample()
    # oracle: 100 * ticks / (100 Hz * 2 seconds).
    assert [r['cpu_pct'] for r in records] == [25.0, 150.0]
    shutil.rmtree(tmp_path / '10')
    now[0] += 2
    assert [r['pid'] for r in sampler.sample()] == [20]  # oracle: only named live PID
    process(tmp_path, 10, ['p', '--ros-args', '-r', '__node:=new'], start=99)
    now[0] += 2
    records = sampler.sample()
    # oracle: proc(5) starttime identifies a new process, without a CPU baseline.
    assert records[0]['node'] == '/new' and records[0]['cpu_pct'] is None


def test_pid_reuse_between_samples_resets_baseline(tmp_path):
    process(tmp_path, 10, ['p', '--ros-args', '-r', '__node:=old'], ticks=100, start=1)
    now = [10.0]
    sampler = system_log().ProcSampler(tmp_path, clk_tck=100, clock=lambda: now[0])
    sampler.sample()
    process(tmp_path, 10, ['p', '--ros-args', '-r', '__node:=new'], ticks=300, start=2)
    now[0] += 2.0
    records = sampler.sample()
    # oracle: proc(5) starttime changes on reuse, so the new name has no baseline.
    assert [(r['node'], r['cpu_pct']) for r in records] == [('/new', None)]


def test_missing_tegrastats_and_recording_continues(tmp_path):
    module = system_log()
    warnings = []
    # oracle: missing executable is nonfatal and produces exactly one warning.
    assert module.start_tegrastats(tmp_path / 'tegra', binary='tegrastats-not-installed', log=warnings.append) is None
    assert len(warnings) == 1 and 'tegrastats-not-installed' in warnings[0]
    proc = tmp_path / 'proc'
    proc.mkdir()
    process(proc, 1, ['p', '--ros-args', '-r', '__node:=x'])
    out = tmp_path / 'out'
    assert module.main(['--output', str(out), '--proc-root', str(proc), '--tegrastats',
                        'tegrastats-not-installed', '--interval', '0.2', '--duration', '1.0']) == 0
    records = [json.loads(line) for line in (out / 'proc.jsonl').read_text().splitlines()]
    # oracle: 1 second at 0.2-second intervals yields at least three samples.
    assert len(records) >= 3
    assert all(set(r) == {'t', 'pid', 'node', 'cpu_pct', 'rss_kb', 'threads'} for r in records)


def test_tegrastats_output_and_cleanup(tmp_path):
    script, pidfile = fake_tegrastats(tmp_path)
    proc = tmp_path / 'proc'
    proc.mkdir()
    out = tmp_path / 'out'
    # oracle: the child emits every 0.1 seconds during a one-second run.
    assert system_log().main(['--output', str(out), '--proc-root', str(proc), '--tegrastats',
                              str(script), '--interval', '0.2', '--duration', '1']) == 0
    assert len((out / 'tegrastats.log').read_text().splitlines()) >= 3
    assert_gone(pidfile)


def test_sigterm_stops_logger_and_tegrastats(tmp_path):
    script, pidfile = fake_tegrastats(tmp_path)
    proc = tmp_path / 'proc'
    proc.mkdir()
    child = subprocess.Popen([sys.executable, '-m', 'race_diagnostics.system_log', '--output',
                              str(tmp_path / 'out'), '--proc-root', str(proc), '--tegrastats',
                              str(script), '--interval', '0.2'], stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 5
        while not pidfile.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        # oracle: SIGTERM is a normal shutdown and must reap the child.
        assert pidfile.exists()
        child.send_signal(signal.SIGTERM)
        _, stderr = child.communicate(timeout=5)
        assert child.returncode == 0 and 'Traceback' not in stderr
        assert_gone(pidfile)
    finally:
        if child.poll() is None:
            child.kill()
            child.communicate()


def test_record_run_wiring(tmp_path):
    from launch import LaunchContext
    from launch.actions import ExecuteProcess
    from launch.utilities import perform_substitutions

    path = Path(__file__).resolve().parents[1] / 'launch' / 'record_run.py'
    spec = importlib.util.spec_from_file_location('record_run_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({'run_directory': str(tmp_path / 'run'),
        'record_bag': 'false', 'name_prefix': '', 'system_log': 'true',
        'topics': ' '.join(module.DEFAULT_TOPICS)})

    def commands():
        actions = module._setup(context)
        return [[perform_substitutions(context, part) for part in action.cmd]
                for action in actions if type(action) is ExecuteProcess]

    # oracle: requested launch contract, with the run's system directory.
    assert commands() == [['ros2', 'run', 'race_diagnostics', 'system_log',
                           '--output', str(tmp_path / 'run' / 'system')]]
    context.launch_configurations['system_log'] = 'false'
    assert commands() == []
