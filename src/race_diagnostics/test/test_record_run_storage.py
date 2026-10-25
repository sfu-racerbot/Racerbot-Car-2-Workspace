"""
Where record_run.py writes, and that what it writes can be pushed to GitHub.

Read out of the launch file with `ast`, like test_record_run_topics.py, so
this runs without ROS.

The requirement (data/README.md): runs land in the workspace's data/ folder
and are committed, and GitHub refuses any single file over 100 MB
(https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github).
So the bag must be split, at a size with room for rosbag2's overshoot.

    python3 -m pytest src/race_diagnostics/test/test_record_run_storage.py -v
"""
import ast
import os

_LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'record_run.py')
GITHUB_FILE_LIMIT_BYTES = 100 * 1000 * 1000


def _tree():
    with open(_LAUNCH) as handle:
        return ast.parse(handle.read())


def _max_bag_bytes():
    for node in _tree().body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'MAX_BAG_FILE_BYTES' for t in node.targets):
            return eval(compile(ast.Expression(node.value), _LAUNCH, 'eval'), {})
    raise AssertionError('record_run.py no longer defines MAX_BAG_FILE_BYTES')


def _bag_record_command():
    """The list literal that starts `ros2 bag record`."""
    for node in ast.walk(_tree()):
        if isinstance(node, ast.List):
            words = [e.value for e in node.elts if isinstance(e, ast.Constant)]
            if words[:3] == ['ros2', 'bag', 'record']:
                return node
    raise AssertionError('record_run.py no longer runs `ros2 bag record`')


def test_bag_files_split_well_under_githubs_file_limit():
    # oracle: spec -- GitHub's 100 MB hard limit (module docstring). rosbag2
    # checks the size only after a buffered batch, so files overshoot it
    # (record_run.py's comment has the measurement); at most half the limit
    # leaves room for that.
    # 0 is rosbag2's "never split" (`ros2 bag record --help`), so it must be
    # refused too.
    size = _max_bag_bytes()
    assert 0 < size <= GITHUB_FILE_LIMIT_BYTES // 2, size


def test_the_recorder_is_actually_told_to_split():
    # A constant nothing passes to rosbag2 would let the previous test pass
    # on a recorder that still writes one unbounded file.
    elts = _bag_record_command().elts
    flags = [i for i, e in enumerate(elts)
             if isinstance(e, ast.Constant) and e.value in ('--max-bag-size', '-b')]
    assert len(flags) == 1, 'ros2 bag record is not given --max-bag-size'
    value = elts[flags[0] + 1]
    assert isinstance(value, ast.Call) and getattr(value.func, 'id', None) == 'str'
    assert [getattr(a, 'id', None) for a in value.args] == ['MAX_BAG_FILE_BYTES']


def test_runs_are_written_into_the_workspace_data_folder():
    # oracle: spec -- data/README.md. Not ~/.ros, which git never sees.
    for node in ast.walk(_tree()):
        if (isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'DeclareLaunchArgument'
                and node.args and getattr(node.args[0], 'value', None) == 'output_directory'):
            default = {k.arg: k.value.value for k in node.keywords}['default_value']
            assert default == '~/racerbot-ws/data/racerbot_runs'
            return
    raise AssertionError('record_run.py no longer declares output_directory')
