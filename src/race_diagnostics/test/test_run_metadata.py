"""
race_diagnostics.run_metadata: what a run was made with. No ROS needed:

    python3 -m pytest src/race_diagnostics/test/test_run_metadata.py -v

Oracles: git itself (the HEAD written must be `git rev-parse HEAD` of a repo
built here; the diff must contain the edit made here), file contents
compared byte for byte, and the `ros2 param dump` nesting (dotted names
become nested mappings).
"""
import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from race_diagnostics import run_metadata as rm  # noqa: E402

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'record_run.py'


def _git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), *args], check=True,
                          capture_output=True, text=True).stdout


@pytest.fixture
def repo(tmp_path):
    r = tmp_path / 'ws'
    r.mkdir()
    _git(r, 'init', '-q')
    _git(r, 'config', 'user.email', 't@example.com')
    _git(r, 'config', 'user.name', 'test')
    (r / 'tracked.yaml').write_text('gain: 1.0\n')
    _git(r, 'add', 'tracked.yaml')
    _git(r, 'commit', '-q', '-m', 'first')
    return r


def test_git_state_records_head_status_and_uncommitted_edits(repo, tmp_path):
    (repo / 'tracked.yaml').write_text('gain: -4040.0\n')       # uncommitted edit
    (repo / 'new_package.py').write_text('x = 1\n')             # untracked
    txt, diff = rm.write_git_state(repo, tmp_path / 'out', 'ws')
    text = txt.read_text()
    # oracle: git itself
    assert f'HEAD: {_git(repo, "rev-parse", "HEAD").strip()}' in text
    assert ' M tracked.yaml' in text
    assert '?? new_package.py' in text
    d = diff.read_text()
    assert '-gain: 1.0' in d and '+gain: -4040.0' in d
    assert 'new_package.py' not in d          # untracked files are not in a diff


def test_a_clean_repo_has_an_empty_diff(repo, tmp_path):
    _, diff = rm.write_git_state(repo, tmp_path / 'out', 'ws')
    assert diff.read_text() == ''


@pytest.mark.parametrize('where', ['missing', 'plain', None])
def test_not_a_repository_is_written_down_not_raised(tmp_path, where):
    path = None if where is None else tmp_path / where
    if where == 'plain':
        path.mkdir()
    txt, diff = rm.write_git_state(path, tmp_path / 'out', 'other')
    assert txt.read_text().startswith('not a git repository')
    assert diff.read_text() == ''


def test_find_repo_root_walks_up_to_the_git_directory(repo):
    deep = repo / 'install' / 'pkg' / 'share' / 'launch'
    deep.mkdir(parents=True)
    assert rm.find_repo_root(deep / 'record_run.py') == repo.resolve()
    assert rm.find_repo_root(repo) == repo.resolve()


def test_find_repo_root_outside_any_repo_is_none(tmp_path):
    assert rm.find_repo_root(tmp_path / 'nowhere') is None


def test_copy_files_copies_byte_for_byte_and_reports_missing(tmp_path):
    src = tmp_path / 'src'
    src.mkdir()
    (src / 'car.yaml').write_bytes(b'rpy_deg: [92.44, 5.22, 90.11]\n')
    result = rm.copy_files([src / 'car.yaml', src / 'gone.json'], tmp_path / 'out')
    assert (tmp_path / 'out' / 'car.yaml').read_bytes() == (src / 'car.yaml').read_bytes()
    assert result[str(src / 'gone.json')] == 'missing'


def test_imu_calibration_files_are_car_yaml_and_every_calib_json(tmp_path):
    (tmp_path / 'config').mkdir()
    (tmp_path / 'calib').mkdir()
    for name in ('bno_calib.json', 'vesc_calib.json', 'notes.txt'):
        (tmp_path / 'calib' / name).write_text('{}')
    files = rm.imu_calibration_files(tmp_path)
    assert files == [tmp_path / 'config' / 'car.yaml',
                     tmp_path / 'calib' / 'bno_calib.json',
                     tmp_path / 'calib' / 'vesc_calib.json']


def test_write_run_metadata_layout(repo, tmp_path):
    (tmp_path / 'imu' / 'config').mkdir(parents=True)
    (tmp_path / 'imu' / 'config' / 'car.yaml').write_text('car: {}\n')
    meta = rm.write_run_metadata(tmp_path / 'run', {'racerbot-ws': repo,
                                                    'imu-calibration': tmp_path / 'imu'},
                                 rm.imu_calibration_files(tmp_path / 'imu'))
    assert (meta / 'git' / 'racerbot-ws.txt').exists()
    assert (meta / 'git' / 'imu-calibration.txt').read_text().startswith('not a git')
    assert (meta / 'files' / 'car.yaml').read_text() == 'car: {}\n'
    assert 'car.yaml ->' in (meta / 'files' / 'MANIFEST.txt').read_text()


def test_params_tree_nests_dotted_names_like_ros2_param_dump():
    # oracle: spec -- ros2 param dump: 'a.b' is written as a: {b: ...}
    tree = rm.params_tree({'wheelbase': 0.36, 'qos_overrides./tf.depth': 100,
                           'gyro.bias.z': -1e-5, 'gyro.rate': 250})
    assert tree == {'wheelbase': 0.36, 'qos_overrides': {'/tf': {'depth': 100}},
                    'gyro': {'bias': {'z': -1e-5}, 'rate': 250}}


def test_params_tree_keeps_a_value_that_is_also_a_prefix():
    tree = rm.params_tree({'a': 1, 'a.b': 2})
    assert tree == {'a': {'': 1, 'b': 2}}
    tree = rm.params_tree({'a.b': 2, 'a': 1})
    assert tree == {'a': {'': 1, 'b': 2}}


@pytest.mark.parametrize('name,expected', [
    ('/vehicle_state_node', 'vehicle_state_node.yaml'),
    ('/ns/slam_toolbox', 'ns__slam_toolbox.yaml'),
    ('/../etc/passwd', '..__etc__passwd.yaml'),
    ('/', 'unnamed.yaml'),
])
def test_node_file_name_is_a_name_never_a_path(name, expected):
    out = rm.node_file_name(name)
    assert out == expected and '/' not in out


def test_params_tree_round_trips_through_yaml():
    tree = rm.params_tree({'heading_sources': ['bno', 'vesc'], 'publish_odom_tf': False})
    assert yaml.safe_load(yaml.safe_dump(tree)) == tree


def test_record_run_writes_metadata_and_snapshots_parameters():
    # Static: the recorder calls the metadata writer for both repos and the
    # IMU calibration files, never lets it stop a run, and starts the
    # parameter snapshot into that folder.
    code = LAUNCH.read_text()
    assert "'racerbot-ws': find_repo_root(os.path.realpath(__file__))" in code
    assert "'imu-calibration': racer_imu_root" in code
    assert 'imu_calibration_files(racer_imu_root)' in code
    assert "except Exception as e:  # noqa: BLE001 -- a metadata failure must not stop a run" in code
    assert "executable='snapshot_params'" in code
    assert "arguments=['--output', os.path.join(str(meta), 'params')]" in code


def test_staged_edits_are_in_the_diff_too(repo, tmp_path):
    # `git add`-ed but not committed is still not what HEAD says.
    (repo / 'tracked.yaml').write_text('gain: 3650.0\n')
    _git(repo, 'add', 'tracked.yaml')
    _, diff = rm.write_git_state(repo, tmp_path / 'out', 'ws')
    assert '+gain: 3650.0' in diff.read_text()
