"""
race_diagnostics.run_metadata version capture: packages.txt and
submodules.txt. No ROS needed:

    python3 -m pytest src/race_diagnostics/test/test_run_versions.py -v

One runner: `python3 -m pytest src/race_diagnostics/test/` (sourced, via
rb) collects these -- they import no rclpy. Oracles: the SLAM-first rule,
the expected-tabular dpkg output format, git itself (shas committed in
throwaway repositories built here), and the repo's own .gitmodules
(copied verbatim, cited at each site).
"""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from race_diagnostics import run_metadata as rm  # noqa: E402

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'record_run.py'


def _git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), *args], check=True,
                          capture_output=True, text=True).stdout


def _init_git_repo(r):
    """git init + identity + one commit, so rev-parse HEAD works."""
    r.mkdir(parents=True)
    _git(r, 'init', '-q')
    _git(r, 'config', 'user.email', 't@example.com')
    _git(r, 'config', 'user.name', 'test')
    (r / 'README.md').write_text('throwaway\n')
    _git(r, 'add', 'README.md')
    _git(r, 'commit', '-q', '-m', 'first')
    return r


@pytest.fixture
def dpkg_output():
    # All quoting-cases the dpkg-query line format asked for, in
    # non-sorted order, slam_toolbox deliberately in the middle.
    return ('ros-jazzy-rcl\t9.1.0-1noble.20260801.000000\n'
            '\n'
            'ros-jazzy-slam-toolbox\t2.8.5-1noble.20260915.204151\n'
            'ros-jazzy-ament-cmake\t2.0.5-1\n'
            'dpkg-query: no matching packages found\n'
            'ros-jazzy-geogebra\t')


def test_parse_dpkg_versions_puts_slam_first_and_sorts_the_rest(dpkg_output):
    parsed = rm.parse_dpkg_versions(dpkg_output)
    # oracle: the rule -- SLAM_PACKAGE first, everything else sorted by
    # name; blank lines skipped; a tab-less line kept as (line, ''); a
    # package with an empty version kept with version ''.
    assert parsed == [
        ('ros-jazzy-slam-toolbox', '2.8.5-1noble.20260915.204151'),
        ('dpkg-query: no matching packages found', ''),
        ('ros-jazzy-ament-cmake', '2.0.5-1'),
        ('ros-jazzy-geogebra', ''),
        ('ros-jazzy-rcl', '9.1.0-1noble.20260801.000000'),
    ]


def test_parse_dpkg_versions_of_nothing_is_an_empty_list():
    assert rm.parse_dpkg_versions('') == []
    assert rm.parse_dpkg_versions('\n\n\n') == []


class _Done:
    def __init__(self, returncode=0, stdout='', stderr=''):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def _line(package, version):
    return f'{package}\t{version}'


def test_write_packages_writes_the_lines_in_parse_order(tmp_path):
    out_path = rm.write_packages(tmp_path, lambda cmd, capture_output, text,
                                 timeout: _Done(stdout=(
                                     'ros-jazzy-rcl\t9.1.0-1noble.20260801.000000\n'
                                     'ros-jazzy-slam-toolbox\t2.8.5-1noble.20260915.204151\n')))
    text = out_path.read_text()
    lines = [l for l in text.splitlines() if l and not l.startswith('#')]
    # oracle: the SLAM-first rule, same input as
    # test_parse_dpkg_versions_puts_slam_first_and_sorts_the_rest.
    assert lines == [
        _line('ros-jazzy-slam-toolbox', '2.8.5-1noble.20260915.204151'),
        _line('ros-jazzy-rcl', '9.1.0-1noble.20260801.000000'),
    ]
    assert 'dpkg-query' in text.split('\n')[0]  # header names the command


def test_write_packages_says_so_when_dpkg_query_is_unavailable(tmp_path):
    def no_dpkg(cmd, capture_output, text, timeout):
        raise FileNotFoundError('dpkg-query')
    out_path = rm.write_packages(tmp_path, no_dpkg)
    text = out_path.read_text()
    assert 'dpkg-query is not available' in text           # the file says why
    assert 'ros-jazzy-slam-toolbox\t' not in text          # nothing fabricated


def test_write_packages_names_the_timeout(tmp_path):
    def slow(cmd, capture_output, text, timeout):
        raise subprocess.TimeoutExpired(cmd, timeout)
    out_path = rm.write_packages(tmp_path, slow)
    text = out_path.read_text()
    assert 'timed out' in text
    assert f'{rm.DPKG_TIMEOUT_S} s' in text


def test_write_packages_records_stderr_on_nonzero_exit(tmp_path):
    def failing(cmd, capture_output, text, timeout):
        return _Done(returncode=2, stderr='error: nothing matched\n')
    out_path = rm.write_packages(tmp_path, failing)
    text = out_path.read_text()
    assert 'exited 2' in text
    assert 'error: nothing matched' in text


def test_write_packages_never_raises_even_for_an_unexpected_error(tmp_path):
    def broken(cmd, capture_output, text, timeout):
        raise IsADirectoryError(21, 'who knows')
    out_path = rm.write_packages(tmp_path, broken)
    text = out_path.read_text()
    assert 'not written' in text or 'failed' in text


REAL_GITMODULES = """[submodule "src/particle_filter"]
\tpath = src/particle_filter
\turl = https://github.com/f1tenth/particle_filter.git
\tbranch = humble-devel
[submodule "src/range_libc"]
\tpath = src/range_libc
\turl = https://github.com/f1tenth/range_libc.git
\tbranch = humble-devel
[submodule "src/transport_drivers"]
\tpath = src/transport_drivers
\turl = https://github.com/ros-drivers/transport_drivers.git
\tbranch = humble
[submodule "src/racerbot_a"]
\tpath = src/racerbot_a
\turl = https://github.com/sfu-racerbot/racerbot_a.git
\tbranch = main
[submodule "src/racerbot_b"]
\tpath = src/racerbot_b
\turl = https://github.com/sfu-racerbot/racerbot_b.git
\tbranch = main
[submodule "src/web_dashboards"]
\tpath = src/web_dashboards
\turl = https://github.com/sfu-racerbot/web-dashboards.git
\tbranch = main
"""


def test_parse_gitmodules_reads_this_repos_six_submodules():
    # oracle: this repo's .gitmodules as of 2026-10-29 (copied verbatim
    # above). file order, all six paths, and every branch.
    parsed = rm.parse_gitmodules(REAL_GITMODULES)
    assert [e['name'] for e in parsed] == [
        'src/particle_filter', 'src/range_libc', 'src/transport_drivers',
        'src/racerbot_a', 'src/racerbot_b', 'src/web_dashboards']
    assert [e['path'] for e in parsed] == [
        'src/particle_filter', 'src/range_libc', 'src/transport_drivers',
        'src/racerbot_a', 'src/racerbot_b', 'src/web_dashboards']
    assert [e['branch'] for e in parsed] == ['humble-devel', 'humble-devel',
                                             'humble', 'main', 'main', 'main']


def test_parse_gitmodules_a_section_without_a_url_still_parses():
    parsed = rm.parse_gitmodules('[submodule "src/something"]\n\tpath = src/something\n')
    # oracle: the spec -- url is optional in git-config format; the entry
    # must survive with url None.
    assert parsed == [{'name': 'src/something', 'path': 'src/something',
                       'url': None, 'branch': None}]


def test_parse_gitmodules_garbage_is_an_empty_list():
    assert rm.parse_gitmodules('not a gitmodules file at all\n') == []
    assert rm.parse_gitmodules('') == []


def _fake_workspace(tmp_path):
    """A workspace with two listed submodules: sub/a really added (its
    content lives in the local throwaway repo `local`), sub/uninit only
    named in .gitmodules."""
    local = _init_git_repo(tmp_path / 'team-x-src')
    ws = _init_git_repo(tmp_path / 'ws')
    _git(ws, '-c', 'protocol.file.allow=always', 'submodule', 'add', '-q',
         f'file://{local}', 'sub/a')
    with open(ws / '.gitmodules', 'a') as handle:
        handle.write('[submodule "sub/uninit"]\n\tpath = sub/uninit\n'
                     f'\turl = file://{tmp_path / "nowhere"}\n')
    _git(ws, 'add', '.gitmodules', 'sub/a')
    _git(ws, 'commit', '-q', '-m', 'add submodules')
    return ws, local


def test_submodule_states_on_a_fake_workspace(tmp_path):
    ws, local = _fake_workspace(tmp_path)
    states = {s['path']: s for s in rm.submodule_states(ws)}
    assert list(states) == ['sub/a', 'sub/uninit']
    assert states['sub/a']['initialised'] and not states['sub/uninit']['initialised']
    # oracle: the sha just committed in the throwaway source repo, which
    # the workspace commit above recorded verbatim.
    expected_sha = _git(local, 'rev-parse', 'HEAD').strip()
    assert states['sub/a']['checked_out'] == expected_sha
    assert states['sub/a']['recorded'] == expected_sha
    assert states['sub/a']['dirty'] is False
    assert states['sub/uninit']['checked_out'] is None
    assert states['sub/uninit']['recorded'] is None
    assert states['sub/uninit']['dirty'] is None


def test_a_submodule_advanced_past_the_workspace_is_flagged_moved(tmp_path):
    ws, _local = _fake_workspace(tmp_path)
    module = ws / 'sub' / 'a'
    _git(module, 'config', 'user.email', 't@example.com')
    _git(module, 'config', 'user.name', 'test')
    _git(module, 'commit', '-q', '--allow-empty', '-m', 'a newer commit')
    text = rm.write_submodules(tmp_path / 'meta', ws).read_text()
    # oracle: the two shas differ by construction -- recorded is what the
    # workspace commit holds, checked_out is the just-made commit.
    states = {s['path']: s for s in rm.submodule_states(ws)}
    assert states['sub/a']['checked_out'] != states['sub/a']['recorded']
    assert 'MOVED: sub/a' in text


def test_an_untracked_file_makes_a_submodule_dirty(tmp_path):
    ws, _local = _fake_workspace(tmp_path)
    (ws / 'sub' / 'a' / 'scratch.py').write_text('x = 1\n')
    assert rm.submodule_states(ws)[0]['dirty'] is True


def test_write_submodules_without_a_says_so(tmp_path):
    out_path = rm.write_submodules(tmp_path / 'meta', None)
    assert 'no workspace' in out_path.read_text().lower()


def test_write_submodules_without_gitmodules_says_so(tmp_path):
    ws = tmp_path / 'plain'
    ws.mkdir()
    out_path = rm.write_submodules(tmp_path / 'meta', ws)
    assert 'submodules' in out_path.read_text().lower()


def test_submodule_states_find_nothing_and_say_nothing(tmp_path):
    assert rm.submodule_states(None) == []
    assert rm.submodule_states(tmp_path) == []   # no .gitmodules


def _load_record_run():
    spec = importlib.util.spec_from_file_location('record_run_test_versions', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run_setup(module, run_dir):
    from launch import LaunchContext
    context = LaunchContext()
    context.launch_configurations.update({
        'run_directory': str(run_dir), 'record_bag': 'false',
        'name_prefix': '', 'system_log': 'false',
        'topics': ' '.join(module.DEFAULT_TOPICS)})
    module._setup(context)


def test_record_run_writes_packages_and_submodules(tmp_path):
    module = _load_record_run()
    run_dir = tmp_path / 'run'
    _run_setup(module, run_dir)
    packages = run_dir / 'metadata' / 'packages.txt'
    submodules = run_dir / 'metadata' / 'submodules.txt'          # files exist
    assert packages.exists() and submodules.exists()
    # oracle: the repo's own .gitmodules (copied verbatim above) names
    # src/racerbot_a and src/racerbot_b; the run must record both.
    listed = submodules.read_text()
    assert 'src/racerbot_a' in listed and 'src/racerbot_b' in listed


def test_record_run_one_writer_failing_does_not_skip_the_other(tmp_path):
    module = _load_record_run()
    run_dir = tmp_path / 'run2'

    def boom(meta_dir, run=subprocess.run):
        raise RuntimeError('boom')
    module.write_packages = boom
    _run_setup(module, run_dir)
    text = (run_dir / 'metadata' / 'submodules.txt').read_text()
    # oracle: .gitmodules names src/racerbot_a -- submodules.txt must
    # still have been written even though write_packages raised.
    assert 'src/racerbot_a' in text
    error = (run_dir / 'metadata' / 'ERROR.txt').read_text()
    assert 'boom' in error
