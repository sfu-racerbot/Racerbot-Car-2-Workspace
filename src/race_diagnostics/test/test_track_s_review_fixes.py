"""Track S regressions; runner: pytest src/race_diagnostics/test/."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_error_file_failure_does_not_stop_other_metadata_writer(tmp_path, monkeypatch, capsys):
    from launch import LaunchContext
    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'record_run.py'
    spec = importlib.util.spec_from_file_location('record_run_review', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    run = tmp_path / 'run'
    (run / 'metadata' / 'ERROR.txt').mkdir(parents=True)
    def fail_packages(meta):
        raise OSError('package failure')
    monkeypatch.setattr(module, 'write_packages', fail_packages)
    context = LaunchContext()
    context.launch_configurations.update({
        'run_directory': str(run), 'record_bag': 'false', 'name_prefix': '',
        'system_log': 'false', 'topics': ' '.join(module.DEFAULT_TOPICS)})
    actions = module._setup(context)
    # oracle: each metadata writer is independent and recording must continue.
    assert len(actions) == 3
    assert 'src/racerbot_a' in (run / 'metadata' / 'submodules.txt').read_text()
    assert 'package failure' in capsys.readouterr().err


def test_tegrastats_child_checks_parent_after_signal_registration():
    # Fresh interpreters avoid forking pytest's ROS/executor threads. Compute
    # the real parent in the child, so this also checks the registration race.
    code = (
        'import os, sys; '
        f'sys.path.insert(0, {str(Path(__file__).resolve().parents[1])!r}); '
        'from race_diagnostics import system_log; '
        'system_log._prepare_tegrastats_child(os.getppid() + int(sys.argv[1]))')
    statuses = []
    for offset in (1, 0):
        result = subprocess.run([sys.executable, '-c', code, str(offset)],
                                capture_output=True, text=True, timeout=5)
        statuses.append(result.returncode)
    # oracle: a changed parent must exit 1; the original parent continues.
    assert statuses == [1, 0]


def test_backoff_stays_bounded_after_ten_thousand_failures():
    from race_diagnostics.run_metadata import NodeWatch
    watch = NodeWatch()
    for _ in range(10000):
        watch.failed('/unresponsive')
    # oracle: run_metadata.py's 32-poll cap permits at most 31 skipped polls;
    # counts above six have the same schedule and must not grow the exponent.
    assert watch._skip_polls['/unresponsive'] == 31
    assert watch._failures['/unresponsive'] <= 6


def test_parent_death_cases_use_bounded_fresh_interpreters(monkeypatch):
    def no_fork():
        raise AssertionError('must not fork the multithreaded pytest process')

    original = subprocess.run
    statuses = []

    def bounded_run(argv, **kwargs):
        assert 0 < kwargs['timeout'] <= 10  # seconds, bounds a wedged child
        result = original(argv, **kwargs)
        statuses.append(result.returncode)
        return result

    monkeypatch.setattr(os, 'fork', no_fork)
    monkeypatch.setattr(subprocess, 'run', bounded_run)
    test_tegrastats_child_checks_parent_after_signal_registration()
    # oracle: spec R5c: actual fresh-process results preserve the parent oracle.
    assert statuses == [1, 0]
