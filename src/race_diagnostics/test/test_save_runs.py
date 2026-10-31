"""
tools/save_runs.sh, run for real against a throwaway git repository. Only
`pgrep` is faked (a process boundary): whether a recorder happens to be running
on this machine must not decide the result. No ROS needed:

    python3 -m pytest src/race_diagnostics/test/test_save_runs.py -v

Oracles: data/README.md (what gets committed) and
docs/run-diagnostics.md#automatic-recording (idle runs are listed, committed
by default, and left uncommitted only when asked).
"""
import os
from pathlib import Path
import shutil
import stat
import subprocess

import pytest
import yaml

SCRIPT = Path(__file__).resolve().parents[3] / 'tools' / 'save_runs.sh'


def git(repo, *args):
    return subprocess.run(['git', *args], cwd=repo, check=True, capture_output=True,
                          text=True).stdout


def add_run(repo, name, info):
    run = repo / 'data' / 'racerbot_runs' / name
    run.mkdir(parents=True)
    (run / 'bag.mcap').write_text('x')
    if info is not None:
        (run / 'auto_record.yaml').write_text(yaml.safe_dump(info, sort_keys=False))


@pytest.fixture
def repo(tmp_path):
    root = tmp_path / 'repo'
    (root / 'tools').mkdir(parents=True)
    shutil.copy(SCRIPT, root / 'tools' / 'save_runs.sh')
    git(root, 'init', '-q')
    git(root, 'config', 'user.email', 'test@example.invalid')
    git(root, 'config', 'user.name', 'test')
    git(root, 'add', 'tools')
    git(root, 'commit', '-q', '-m', 'start')
    add_run(root, 'driven', {'activity': {'moved': True}})
    add_run(root, 'idle', {'activity': {'moved': False}})
    add_run(root, 'stub', {'superseded': True, 'activity': {'moved': True}})
    add_run(root, 'by_hand', None)         # record_run.py by hand: no auto_record.yaml
    add_run(root, 'unknown', {'activity': {'moved': None}})
    return root


def fake_pgrep(tmp_path, running=''):
    """A pgrep that matches only command patterns containing `running`."""
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir(exist_ok=True)
    fake = bin_dir / 'pgrep'
    fake.write_text(f'#!/bin/sh\ncase "$*" in\n  *"{running or "NOTHING_RUNS"}"*) exit 0;;\n'
                    'esac\nexit 1\n')
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    return bin_dir


def save(repo, tmp_path, *args, running=''):
    env = dict(os.environ, PATH=f"{fake_pgrep(tmp_path, running)}:{os.environ['PATH']}")
    return subprocess.run(['bash', 'tools/save_runs.sh', '--no-push', *args], cwd=repo,
                          env=env, capture_output=True, text=True)


def committed(repo):
    names = git(repo, 'ls-tree', '-r', '--name-only', 'HEAD', 'data').splitlines()
    return sorted({Path(n).parts[2] for n in names})


def test_idle_runs_are_listed_but_still_committed_by_default(repo, tmp_path):
    done = save(repo, tmp_path)
    assert done.returncode == 0, done.stderr
    assert committed(repo) == ['by_hand', 'driven', 'idle', 'stub', 'unknown']
    for name in ('data/racerbot_runs/idle', 'data/racerbot_runs/stub'):
        assert name in done.stdout
    assert 'data/racerbot_runs/driven\n' not in done.stdout
    assert 'data/racerbot_runs/by_hand' not in done.stdout
    assert 'data/racerbot_runs/unknown' not in done.stdout


def test_skip_idle_leaves_only_flagged_runs_uncommitted(repo, tmp_path):
    done = save(repo, tmp_path, '--skip-idle')
    assert done.returncode == 0, done.stderr
    # A run with no record, or an unknown one, must never be treated as idle.
    assert committed(repo) == ['by_hand', 'driven', 'unknown']
    assert (repo / 'data' / 'racerbot_runs' / 'idle' / 'bag.mcap').exists(), \
        'skipping must not delete anything'
    assert (repo / 'data' / 'racerbot_runs' / 'stub' / 'bag.mcap').exists()
    untracked = git(repo, 'status', '--short', 'data')
    assert 'data/racerbot_runs/idle/' in untracked
    assert 'data/racerbot_runs/stub/' in untracked


def test_an_already_committed_idle_run_is_not_listed_again(repo, tmp_path):
    assert save(repo, tmp_path).returncode == 0
    add_run(repo, 'driven2', {'activity': {'moved': True}})
    done = save(repo, tmp_path)
    assert done.returncode == 0, done.stderr
    assert 'data/racerbot_runs/idle' not in done.stdout


def test_skip_idle_with_nothing_else_commits_nothing(repo, tmp_path):
    for name in ('driven', 'by_hand', 'unknown'):
        shutil.rmtree(repo / 'data' / 'racerbot_runs' / name)
    before = git(repo, 'rev-parse', 'HEAD')
    done = save(repo, tmp_path, '--skip-idle')
    assert done.returncode == 0, done.stderr
    assert git(repo, 'rev-parse', 'HEAD') == before


def test_refuses_while_a_run_is_still_being_summarized(repo, tmp_path):
    # run_activity is about to rewrite auto_record.yaml. Committing now would
    # save a run with no verdict on it.
    done = save(repo, tmp_path, running='run_activity')
    assert done.returncode == 1
    assert 'still' in done.stderr
    assert committed(repo) == []


def test_still_refuses_while_a_bag_is_recording(repo, tmp_path):
    done = save(repo, tmp_path, running='ros2 bag record')
    assert done.returncode == 1
    assert committed(repo) == []


def test_an_unknown_option_is_refused_before_anything_is_committed(repo, tmp_path):
    done = save(repo, tmp_path, '--skip-idel')
    assert done.returncode == 2
    assert '--skip-idel' in done.stderr
    assert committed(repo) == []
