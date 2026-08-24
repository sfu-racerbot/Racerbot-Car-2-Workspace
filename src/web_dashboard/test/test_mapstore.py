"""
Unit tests for web_dashboard/mapstore.py -- finding, vetting and deleting
saved SLAM map run directories.

No ROS, no Tornado, no network, no browser: mapstore.py imports none of
them, so these run under plain `python3 -m pytest src/web_dashboard/test/`
with nothing sourced. Nothing here deletes anything outside pytest's own
tmp_path.

Nothing is mocked. mapstore.py was written ROS-free precisely so it could be
called for real against a real directory tree, and a fake filesystem would
test the fake rather than the path handling that is the whole point.

Oracles used below, per TEST_QUALITY_STANDARDS.md:
  * recorded measurement -- byte-for-byte headers copied out of this car's
    own saved maps, named at the point of use
  * cited spec -- the Netpbm graymap header format
  * invariant -- properties that must hold for every input, e.g. nothing
    sanitize_roots returns may sit inside a git working tree
  * closed form -- sizes and counts recomputed in the test
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from web_dashboard import mapstore  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class _Logger:
    """Just enough of an rclpy logger to record what was refused."""

    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


class _Target:
    """Just enough of a proccontrol.Target for in_use_by()."""

    def __init__(self, pid, name, cmdline):
        self.pid = pid
        self.name = name
        self.cmdline = cmdline


def _make_run(root, run_id, *, map_yaml=True, pgm=b'P5 127 219 255\n',
              posegraph=True, raceline=True, filler=b''):
    """Build a run directory the way auto_map_race_node writes one."""
    path = os.path.join(str(root), run_id)
    os.makedirs(path, exist_ok=True)
    if map_yaml:
        # Copied verbatim from ~/.ros/racerbot_auto/20260727-200103/map.yaml.
        with open(os.path.join(path, 'map.yaml'), 'w') as handle:
            handle.write('image: map.pgm\n'
                         'mode: trinary\n'
                         'resolution: 0.050\n'
                         'origin: [-2.561, -2.054, 0]\n'
                         'negate: 0\n'
                         'occupied_thresh: 0.65\n'
                         'free_thresh: 0.196\n')
    if pgm is not None:
        with open(os.path.join(path, 'map.pgm'), 'wb') as handle:
            handle.write(pgm + filler)
    if posegraph:
        for name in ('posegraph.posegraph', 'posegraph.data'):
            with open(os.path.join(path, name), 'wb') as handle:
                handle.write(b'x' * 10)
    if raceline:
        for name in ('raceline_raw.csv', 'raceline_profiled.csv'):
            with open(os.path.join(path, name), 'w') as handle:
                handle.write('x,y\n0,0\n')
    return path


# ---------------------------------------------------------------------------
# read_pgm_size -- the one real parser in this module
#
# Oracle: the Netpbm graymap spec (magic, width, height, maxval, with '#'
# comments allowed wherever whitespace is), plus one recorded header.
# ---------------------------------------------------------------------------

def test_reads_the_dimensions_this_cars_own_map_actually_has(tmp_path):
    """Recorded measurement: the first bytes of
    ~/.ros/racerbot_auto/20260727-200103/map.pgm are `P5 127 219 255`."""
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P5 127 219 255\n' + b'\xff' * 64)
    assert mapstore.read_pgm_size(str(path)) == (127, 219)


def test_reads_a_header_split_across_lines():
    """map_saver writes one line; other tools write four. Both are the spec."""
    raw = b'P5\n640\n480\n255\n'
    assert mapstore._pgm_tokens(raw, 3) == [b'P5', b'640', b'480']


def test_a_comment_between_the_numbers_is_skipped(tmp_path):
    """Cited spec: '#' runs to end of line and may appear anywhere
    whitespace may. A parser that misses this reads the comment as a width."""
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P5\n# Created by a tool\n127 219\n255\n')
    assert mapstore.read_pgm_size(str(path)) == (127, 219)


def test_a_comment_before_the_magic_is_skipped(tmp_path):
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'# leading comment\nP5 8 9 255\n')
    assert mapstore.read_pgm_size(str(path)) == (8, 9)


def test_an_ascii_graymap_is_accepted(tmp_path):
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P2 4 5 255\n')
    assert mapstore.read_pgm_size(str(path)) == (4, 5)


@pytest.mark.parametrize('magic', [b'P1', b'P3', b'P4', b'P6', b'PX', b'\x89PNG'])
def test_anything_that_is_not_a_graymap_is_refused(tmp_path, magic):
    """A PPM, a bitmap or a PNG is not a ROS occupancy map. The simulator's
    own tracks really are .png, so this case is reachable."""
    path = tmp_path / 'image'
    path.write_bytes(magic + b' 100 100 255\n')
    assert mapstore.read_pgm_size(str(path)) is None


def test_a_truncated_header_is_refused(tmp_path):
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P5 127')
    assert mapstore.read_pgm_size(str(path)) is None


def test_an_empty_file_is_refused(tmp_path):
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'')
    assert mapstore.read_pgm_size(str(path)) is None


def test_a_missing_file_is_refused_rather_than_raising(tmp_path):
    assert mapstore.read_pgm_size(str(tmp_path / 'nope.pgm')) is None


@pytest.mark.parametrize('dims', [b'wide tall', b'12x9 4', b'- -'])
def test_non_numeric_dimensions_are_refused(tmp_path, dims):
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P5 ' + dims + b' 255\n')
    assert mapstore.read_pgm_size(str(path)) is None


@pytest.mark.parametrize('dims', [b'0 0', b'0 219', b'127 0'])
def test_a_zero_dimension_is_refused(tmp_path, dims):
    """Boundary: zero is exactly one step below the smallest real map."""
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P5 ' + dims + b' 255\n')
    assert mapstore.read_pgm_size(str(path)) is None


def test_the_smallest_positive_dimension_is_accepted(tmp_path):
    """One step the other side of the same boundary."""
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P5 1 1 255\n')
    assert mapstore.read_pgm_size(str(path)) == (1, 1)


@pytest.mark.parametrize('dims', [b'-127 219', b'127 -219'])
def test_a_negative_dimension_is_refused(tmp_path, dims):
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P5 ' + dims + b' 255\n')
    assert mapstore.read_pgm_size(str(path)) is None


def test_the_largest_plausible_dimension_is_accepted(tmp_path):
    """Boundary, from the other side: _PGM_MAX_DIM itself must still pass,
    or the limit is off by one against the largest grid this workspace
    produces (2048x2048)."""
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P5 %d 1 255\n' % mapstore._PGM_MAX_DIM)
    assert mapstore.read_pgm_size(str(path)) == (mapstore._PGM_MAX_DIM, 1)


def test_an_absurd_dimension_is_refused(tmp_path):
    """One step past the boundary. A corrupt header must not be reported as
    a map the size of a continent."""
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P5 %d 1 255\n' % (mapstore._PGM_MAX_DIM + 1))
    assert mapstore.read_pgm_size(str(path)) is None


def test_malformed_utf8_where_a_number_belongs_is_refused(tmp_path):
    """Bytes that are not valid ASCII must come back as None, never as an
    exception out of a display-only code path."""
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P5 \xff\xfe\x80 219 255\n')
    assert mapstore.read_pgm_size(str(path)) is None


def test_a_header_of_only_comments_is_refused(tmp_path):
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'# nothing\n# but\n# comments\n')
    assert mapstore.read_pgm_size(str(path)) is None


def test_a_huge_image_is_never_read_past_its_header(tmp_path):
    """Invariant: the read is bounded. A 2048x2048 map is 4.2MB and this is
    called for every run directory in the list, on the car."""
    path = tmp_path / 'map.pgm'
    path.write_bytes(b'P5 2048 2048 255\n' + b'\x00' * (2048 * 2048))
    assert mapstore.read_pgm_size(str(path)) == (2048, 2048)
    assert mapstore._PGM_HEADER_BYTES < 4096


# ---------------------------------------------------------------------------
# read_map_yaml
# ---------------------------------------------------------------------------

def test_reads_the_resolution_and_origin_this_car_actually_wrote(tmp_path):
    """Recorded measurement: the whole content of
    ~/.ros/racerbot_auto/20260727-200103/map.yaml."""
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\n'
                    'mode: trinary\n'
                    'resolution: 0.050\n'
                    'origin: [-2.561, -2.054, 0]\n'
                    'negate: 0\n'
                    'occupied_thresh: 0.65\n'
                    'free_thresh: 0.196\n')
    meta = mapstore.read_map_yaml(str(path))
    assert meta['resolution'] == pytest.approx(0.05, abs=1e-9)  # cell size, m
    assert meta['origin'] == pytest.approx([-2.561, -2.054, 0.0], abs=1e-9)  # m
    assert meta['image'] == 'map.pgm'


def test_a_commented_out_key_is_not_read(tmp_path):
    """The regex is anchored at line start for exactly this: a commented
    line must stay commented. A parser that searches anywhere in the text
    reads 9.0 here and reports a map 180x too large."""
    path = tmp_path / 'map.yaml'
    path.write_text('# resolution: 9.0\n'
                    '# origin: [99.0, 99.0, 0]\n'
                    'resolution: 0.05\n')
    meta = mapstore.read_map_yaml(str(path))
    assert meta['resolution'] == pytest.approx(0.05, abs=1e-9)  # m
    assert meta['origin'] is None


def test_an_indented_key_is_still_read(tmp_path):
    path = tmp_path / 'map.yaml'
    path.write_text('  resolution: 0.025\n')
    assert mapstore.read_map_yaml(str(path))['resolution'] == pytest.approx(
        0.025, abs=1e-9)  # m


def test_missing_keys_come_back_as_none_not_as_an_error(tmp_path):
    path = tmp_path / 'map.yaml'
    path.write_text('mode: trinary\n')
    meta = mapstore.read_map_yaml(str(path))
    assert meta == {'resolution': None, 'origin': None, 'image': None}


def test_an_empty_yaml_gives_all_none(tmp_path):
    path = tmp_path / 'map.yaml'
    path.write_text('')
    assert mapstore.read_map_yaml(str(path)) == {
        'resolution': None, 'origin': None, 'image': None}


def test_an_unreadable_yaml_gives_none(tmp_path):
    assert mapstore.read_map_yaml(str(tmp_path / 'missing.yaml')) is None


def test_malformed_utf8_in_the_yaml_does_not_raise(tmp_path):
    """errors='replace': one corrupt byte costs one key, not the panel."""
    path = tmp_path / 'map.yaml'
    path.write_bytes(b'image: map.pgm\n\xff\xfe\nresolution: 0.05\n')
    meta = mapstore.read_map_yaml(str(path))
    assert meta['resolution'] == pytest.approx(0.05, abs=1e-9)  # m


def test_a_non_numeric_resolution_is_dropped_not_guessed(tmp_path):
    path = tmp_path / 'map.yaml'
    path.write_text('resolution: ...\n')
    assert mapstore.read_map_yaml(str(path))['resolution'] is None


# ---------------------------------------------------------------------------
# is_plain_component -- what the browser is allowed to name
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('name', [
    '20260727-200103', 'run1', 'a', 'map_run.2',
])
def test_an_ordinary_directory_name_is_accepted(name):
    assert mapstore.is_plain_component(name) is True


@pytest.mark.parametrize('name', [
    '', '.', '..', '../..', '../etc', 'a/b', '/etc', 'a\\b', '.hidden',
    'a\x00b', './run', 'run/', '~',
])
def test_anything_that_could_be_a_path_is_refused(name):
    """The browser sends a name, never a path. This is the check that makes
    that true rather than merely intended."""
    assert mapstore.is_plain_component(name) is False


# ---------------------------------------------------------------------------
# sanitize_roots -- what may be managed at all
# ---------------------------------------------------------------------------

def test_an_ordinary_root_is_kept(tmp_path):
    root = tmp_path / 'racerbot_auto'
    root.mkdir()
    assert mapstore.sanitize_roots([str(root)]) == [os.path.realpath(str(root))]


def test_a_root_that_does_not_exist_yet_is_kept(tmp_path):
    """~/.ros/racerbot_auto is created by the first auto_map_race run, and
    this dashboard is started once and left running across sessions."""
    root = tmp_path / 'not_yet'
    assert mapstore.sanitize_roots([str(root)]) == [os.path.realpath(str(root))]


def test_a_root_inside_a_git_working_tree_is_refused_and_logged(tmp_path):
    """src/particle_filter/maps holds tracked upstream demo maps. A browser
    deleting those would surface as a mystery `git status`."""
    (tmp_path / '.git').mkdir()
    root = tmp_path / 'src' / 'particle_filter' / 'maps'
    root.mkdir(parents=True)
    logger = _Logger()
    assert mapstore.sanitize_roots([str(root)], logger) == []
    assert len(logger.warnings) == 1
    assert 'git working tree' in logger.warnings[0]


def test_a_git_worktree_whose_dot_git_is_a_file_is_also_refused(tmp_path):
    """`git worktree add` writes .git as a FILE. Checking for a directory
    would let a worktree of this repo through."""
    (tmp_path / '.git').write_text('gitdir: /elsewhere\n')
    root = tmp_path / 'maps'
    root.mkdir()
    assert mapstore.sanitize_roots([str(root)]) == []


def test_the_simulator_track_directory_is_refused(tmp_path):
    """~/.ros/racerbot_sim/tracks holds generated INPUTS to the simulator,
    not run output. Deleting one breaks the sim rather than freeing a
    stale result."""
    root = tmp_path / 'racerbot_sim' / 'tracks'
    root.mkdir(parents=True)
    logger = _Logger()
    assert mapstore.sanitize_roots([str(root)], logger) == []
    assert 'simulator track inputs' in logger.warnings[0]


def test_the_home_directory_is_refused():
    logger = _Logger()
    assert mapstore.sanitize_roots(['~'], logger) == []
    assert 'home directory' in logger.warnings[0]


def test_the_filesystem_root_is_refused():
    assert mapstore.sanitize_roots(['/']) == []


@pytest.mark.parametrize('root', ['/home', '/mnt', '/tmp'])
def test_a_top_level_system_directory_is_refused(root):
    """Boundary: fewer than two components below /. A typo in map_roots must
    not be able to point at one of these."""
    assert mapstore.sanitize_roots([root]) == []


def test_two_components_below_the_root_is_the_first_accepted_depth(tmp_path):
    """One step the other side of the same boundary."""
    assert mapstore.sanitize_roots(['/tmp/somewhere']) == ['/tmp/somewhere']


def test_blank_and_whitespace_entries_are_dropped_quietly():
    assert mapstore.sanitize_roots(['', '   ', '\t']) == []


def test_the_same_root_written_two_ways_appears_once(tmp_path):
    root = tmp_path / 'maps'
    root.mkdir()
    kept = mapstore.sanitize_roots([str(root), str(root) + os.sep,
                                    str(root / '.' )])
    assert kept == [os.path.realpath(str(root))]


def test_order_is_preserved(tmp_path):
    first = tmp_path / 'a'
    second = tmp_path / 'b'
    first.mkdir()
    second.mkdir()
    assert mapstore.sanitize_roots([str(second), str(first)]) == [
        os.path.realpath(str(second)), os.path.realpath(str(first))]


def test_nothing_returned_is_ever_inside_a_git_tree(tmp_path):
    """Invariant over a mixed list, rather than one case at a time."""
    (tmp_path / 'repo' / '.git').mkdir(parents=True)
    (tmp_path / 'repo' / 'maps').mkdir()
    (tmp_path / 'plain').mkdir()
    kept = mapstore.sanitize_roots([
        str(tmp_path / 'repo' / 'maps'), str(tmp_path / 'plain'), '/', '~'])
    assert kept
    for root in kept:
        assert not mapstore.in_git_tree(root)


# ---------------------------------------------------------------------------
# scan
# ---------------------------------------------------------------------------

def test_a_complete_run_is_described_by_what_it_holds(tmp_path):
    _make_run(tmp_path, '20260727-200103')
    run, = mapstore.scan([str(tmp_path)])
    assert run.run_id == '20260727-200103'
    assert run.has_map is True
    assert run.has_posegraph is True
    assert run.contents == ['map', 'pose graph', 'recorded lap', 'racing line']
    assert run.width == 127 and run.height == 219  # from the .pgm header
    assert run.resolution == pytest.approx(0.05, abs=1e-9)  # m per cell


def test_the_span_in_metres_is_cells_times_resolution(tmp_path):
    """Closed form: 127 * 0.05 = 6.35 m, 219 * 0.05 = 10.95 m."""
    _make_run(tmp_path, 'run')
    run, = mapstore.scan([str(tmp_path)])
    assert run.size_m == pytest.approx((6.35, 10.95), abs=1e-9)  # m
    assert run.as_dict()['span_m'] == pytest.approx([6.35, 10.95], abs=1e-9)


def test_a_run_with_no_map_is_still_listed(tmp_path):
    """~/.ros/racerbot_auto/20260727-202458 has a pose graph and a racing
    line but no map, from a map_saver race. It is 38MB and the whole reason
    someone opens this panel, so hiding it would be exactly wrong."""
    _make_run(tmp_path, 'racey', map_yaml=False, pgm=None)
    run, = mapstore.scan([str(tmp_path)])
    assert run.has_map is False
    assert run.has_posegraph is True
    assert 'map' not in run.contents


def test_the_reported_size_is_the_sum_of_the_files(tmp_path):
    """Closed form, recomputed here rather than trusted."""
    path = _make_run(tmp_path, 'run')
    expected = sum(os.path.getsize(os.path.join(path, name))
                   for name in os.listdir(path))
    run, = mapstore.scan([str(tmp_path)])
    assert run.bytes == expected


def test_an_empty_root_yields_nothing(tmp_path):
    assert mapstore.scan([str(tmp_path)]) == []


def test_a_root_that_does_not_exist_is_skipped_rather_than_raising(tmp_path):
    assert mapstore.scan([str(tmp_path / 'gone')]) == []


def test_a_single_run_is_handled(tmp_path):
    _make_run(tmp_path, 'only')
    assert len(mapstore.scan([str(tmp_path)])) == 1


def test_loose_files_in_a_root_are_not_runs(tmp_path):
    (tmp_path / 'stray.txt').write_text('x')
    _make_run(tmp_path, 'run')
    assert [r.run_id for r in mapstore.scan([str(tmp_path)])] == ['run']


def test_a_symlinked_directory_is_never_listed_as_a_run(tmp_path):
    """A link dropped into a root could otherwise redirect a delete
    anywhere the dashboard's user can write."""
    outside = tmp_path / 'outside'
    outside.mkdir()
    root = tmp_path / 'root'
    root.mkdir()
    os.symlink(str(outside), str(root / 'sneaky'))
    assert mapstore.scan([str(root)]) == []


def test_a_hidden_directory_is_not_a_run(tmp_path):
    os.makedirs(str(tmp_path / '.cache'))
    assert mapstore.scan([str(tmp_path)]) == []


def test_runs_come_back_newest_first(tmp_path):
    older = _make_run(tmp_path, '20260101-000000')
    newer = _make_run(tmp_path, '20260202-000000')
    os.utime(older, (1000, 1000))
    os.utime(newer, (2000, 2000))
    assert [r.run_id for r in mapstore.scan([str(tmp_path)])] == [
        '20260202-000000', '20260101-000000']


def test_runs_from_several_roots_are_merged(tmp_path):
    first = tmp_path / 'auto'
    second = tmp_path / 'sim'
    first.mkdir()
    second.mkdir()
    _make_run(first, 'a')
    _make_run(second, 'b')
    assert {r.run_id for r in mapstore.scan([str(first), str(second)])} == {'a', 'b'}


def test_a_yaml_pointing_outside_its_own_directory_is_not_followed(tmp_path):
    """map.yaml names its image relatively. One naming ../../etc/passwd is
    not a map this panel reads dimensions out of."""
    path = _make_run(tmp_path, 'run')
    with open(os.path.join(path, 'map.yaml'), 'w') as handle:
        handle.write('image: ../../elsewhere.pgm\nresolution: 0.05\n')
    run, = mapstore.scan([str(tmp_path)])
    assert run.has_map is True
    assert run.width is None and run.height is None


# ---------------------------------------------------------------------------
# resolve_delete -- the gate
# ---------------------------------------------------------------------------

def _scanned(tmp_path, run_id='20260727-200103'):
    _make_run(tmp_path, run_id)
    roots = [os.path.realpath(str(tmp_path))]
    return mapstore.scan(roots), roots


def test_the_exact_name_typed_exactly_is_accepted(tmp_path):
    runs, roots = _scanned(tmp_path)
    run, reason = mapstore.resolve_delete(
        runs, '20260727-200103', '20260727-200103', roots)
    assert reason == ''
    assert run is not None and run.run_id == '20260727-200103'


def test_the_wrong_typed_name_is_refused(tmp_path):
    """A8: the refusal is the half that matters. Typing the name is the
    whole guard, so a near-miss must not pass."""
    runs, roots = _scanned(tmp_path)
    run, reason = mapstore.resolve_delete(
        runs, '20260727-200103', '20260727-20010', roots)
    assert run is None
    assert '20260727-200103' in reason


def test_a_typed_name_differing_only_in_case_is_refused(tmp_path):
    runs, roots = _scanned(tmp_path, 'RunOne')
    run, reason = mapstore.resolve_delete(runs, 'RunOne', 'runone', roots)
    assert run is None
    assert 'does not match' in reason


def test_a_typed_name_with_surrounding_space_is_refused(tmp_path):
    """Deliberately not trimmed. The comparison is exact so that what the
    server checks is precisely what the person was asked to type."""
    runs, roots = _scanned(tmp_path)
    run, _ = mapstore.resolve_delete(
        runs, '20260727-200103', ' 20260727-200103 ', roots)
    assert run is None


def test_no_typed_name_at_all_is_refused(tmp_path):
    """A hand-rolled WebSocket client that just sends an id lands here."""
    runs, roots = _scanned(tmp_path)
    for typed in ('', None):
        run, reason = mapstore.resolve_delete(
            runs, '20260727-200103', typed, roots)
        assert run is None
        assert 'type the map name' in reason


def test_a_truthy_confirm_flag_cannot_stand_in_for_the_name(tmp_path):
    """There is no 'confirmed: true' path. Passing one as the typed name is
    just a wrong name."""
    runs, roots = _scanned(tmp_path)
    run, _ = mapstore.resolve_delete(runs, '20260727-200103', 'true', roots)
    assert run is None


def test_an_unknown_name_is_refused(tmp_path):
    """A stale browser tab naming a run someone else already deleted."""
    runs, roots = _scanned(tmp_path)
    run, reason = mapstore.resolve_delete(runs, 'ghost', 'ghost', roots)
    assert run is None
    assert 're-read' in reason


def test_no_name_at_all_is_refused(tmp_path):
    runs, roots = _scanned(tmp_path)
    for run_id in ('', None):
        run, reason = mapstore.resolve_delete(runs, run_id, 'anything', roots)
        assert run is None
        assert reason == 'no map was named'


@pytest.mark.parametrize('run_id', ['..', '../..', 'a/b', '/etc', '.hidden'])
def test_a_path_shaped_name_is_refused_before_anything_is_looked_up(
        tmp_path, run_id):
    runs, roots = _scanned(tmp_path)
    run, reason = mapstore.resolve_delete(runs, run_id, run_id, roots)
    assert run is None
    assert 'plain directory name' in reason


def test_a_run_whose_path_escaped_the_roots_is_refused(tmp_path):
    """Defence in depth: even a SavedRun that came from somewhere other than
    scan() cannot name a path outside the managed roots."""
    outside = tmp_path / 'outside'
    outside.mkdir()
    root = tmp_path / 'root'
    root.mkdir()
    roots = [os.path.realpath(str(root))]
    rogue = mapstore.SavedRun('outside', str(root), str(outside))
    run, reason = mapstore.resolve_delete([rogue], 'outside', 'outside', roots)
    assert run is None
    assert 'outside every directory' in reason


def test_a_run_that_is_a_symlink_is_refused(tmp_path):
    real = tmp_path / 'real'
    real.mkdir()
    root = tmp_path / 'root'
    root.mkdir()
    link = root / 'link'
    os.symlink(str(real), str(link))
    roots = [os.path.realpath(str(root))]
    rogue = mapstore.SavedRun('link', str(root), str(link))
    run, reason = mapstore.resolve_delete([rogue], 'link', 'link', roots)
    assert run is None
    assert 'symlink' in reason


def test_an_empty_run_list_refuses_everything(tmp_path):
    run, reason = mapstore.resolve_delete([], 'anything', 'anything',
                                          [str(tmp_path)])
    assert run is None
    assert 're-read' in reason


# ---------------------------------------------------------------------------
# delete_run
# ---------------------------------------------------------------------------

def test_deleting_removes_the_whole_directory_and_reports_what_it_freed(
        tmp_path):
    path = _make_run(tmp_path, 'run', filler=b'z' * 5000)
    roots = [os.path.realpath(str(tmp_path))]
    runs = mapstore.scan(roots)
    expected = mapstore.directory_size(path)
    ok, detail, freed = mapstore.delete_run(runs[0], roots)
    assert ok is True, detail
    assert freed == expected
    assert not os.path.exists(path)


def test_deleting_takes_the_racing_line_and_pose_graph_with_it(tmp_path):
    """The unit is the run, not the map. This is the behaviour the panel
    warns about before asking for the confirmation -- see the module
    docstring on why a partial delete is the one outcome to prevent."""
    path = _make_run(tmp_path, 'run')
    roots = [os.path.realpath(str(tmp_path))]
    runs = mapstore.scan(roots)
    mapstore.delete_run(runs[0], roots)
    for name in ('map.yaml', 'map.pgm', 'posegraph.posegraph',
                 'raceline_profiled.csv'):
        assert not os.path.exists(os.path.join(path, name))


def test_deleting_a_path_outside_the_roots_is_refused(tmp_path):
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'keep.txt').write_text('x')
    root = tmp_path / 'root'
    root.mkdir()
    rogue = mapstore.SavedRun('outside', str(root), str(outside))
    ok, reason, freed = mapstore.delete_run(rogue, [os.path.realpath(str(root))])
    assert ok is False
    assert freed == 0
    assert os.path.exists(str(outside / 'keep.txt'))
    assert 'outside every directory' in reason


def test_deleting_a_symlink_is_refused_and_the_target_survives(tmp_path):
    """rmtree on a link would be refused by shutil anyway; this refuses it
    first, with a reason a person can act on."""
    real = tmp_path / 'real'
    real.mkdir()
    (real / 'keep.txt').write_text('x')
    root = tmp_path / 'root'
    root.mkdir()
    link = root / 'link'
    os.symlink(str(real), str(link))
    rogue = mapstore.SavedRun('link', str(root), str(link))
    ok, reason, _ = mapstore.delete_run(rogue, [os.path.realpath(str(root))])
    assert ok is False
    assert 'symlink' in reason
    assert os.path.exists(str(real / 'keep.txt'))


def test_deleting_something_already_gone_is_refused_not_reported_as_success(
        tmp_path):
    root = tmp_path / 'root'
    root.mkdir()
    rogue = mapstore.SavedRun('gone', str(root), str(root / 'gone'))
    ok, reason, _ = mapstore.delete_run(rogue, [os.path.realpath(str(root))])
    assert ok is False
    assert 'no longer exists' in reason


def test_deleting_a_root_itself_is_refused(tmp_path):
    """_contains is strict: a root is not inside itself, so naming one
    cannot wipe every run under it at once."""
    root = tmp_path / 'root'
    root.mkdir()
    roots = [os.path.realpath(str(root))]
    rogue = mapstore.SavedRun('root', str(root), str(root))
    ok, reason, _ = mapstore.delete_run(rogue, roots)
    assert ok is False
    assert os.path.isdir(str(root))


# ---------------------------------------------------------------------------
# directory_size
# ---------------------------------------------------------------------------

def test_directory_size_sums_nested_files(tmp_path):
    """Closed form: 10 + 20 + 30 bytes across two levels."""
    (tmp_path / 'a').write_bytes(b'x' * 10)
    nested = tmp_path / 'deeper'
    nested.mkdir()
    (nested / 'b').write_bytes(b'x' * 20)
    (nested / 'c').write_bytes(b'x' * 30)
    assert mapstore.directory_size(str(tmp_path)) == 60


def test_directory_size_of_an_empty_directory_is_zero(tmp_path):
    assert mapstore.directory_size(str(tmp_path)) == 0


def test_directory_size_counts_a_symlink_not_what_it_points_at(tmp_path):
    """lstat, not stat: a link to a 26MB pose graph is a few bytes, and
    counting the target would report freeing memory that stays used."""
    big = tmp_path / 'big'
    big.write_bytes(b'x' * 10000)
    root = tmp_path / 'root'
    root.mkdir()
    os.symlink(str(big), str(root / 'link'))
    assert mapstore.directory_size(str(root)) < 1000


# ---------------------------------------------------------------------------
# in_use_by
# ---------------------------------------------------------------------------

def test_a_map_server_serving_this_run_is_reported(tmp_path):
    """Deleting the map a running map_server holds is the documented way to
    wedge particle_filter in its constructor."""
    path = str(tmp_path / '20260727-200103')
    targets = [_Target(4242, 'map_server',
                       f'map_server --ros-args -p yaml_filename:={path}/map.yaml')]
    assert [t.pid for t in mapstore.in_use_by(targets, path)] == [4242]


def test_a_controller_following_this_runs_racing_line_is_reported(tmp_path):
    path = str(tmp_path / 'run')
    targets = [_Target(7, 'pure_pursuit_node',
                       f'pure_pursuit_node -p waypoints_file:={path}/raceline_profiled.csv')]
    assert len(mapstore.in_use_by(targets, path)) == 1


def test_an_unrelated_process_is_not_reported(tmp_path):
    targets = [_Target(9, 'gap_follow_node', 'gap_follow_node --ros-args')]
    assert mapstore.in_use_by(targets, str(tmp_path / 'run')) == []


def test_a_process_using_a_different_run_is_not_reported(tmp_path):
    targets = [_Target(9, 'map_server', f'map_server {tmp_path}/other/map.yaml')]
    assert mapstore.in_use_by(targets, str(tmp_path / 'run')) == []


def test_no_processes_at_all_reports_nothing(tmp_path):
    assert mapstore.in_use_by([], str(tmp_path / 'run')) == []


def test_a_target_with_no_cmdline_is_survived(tmp_path):
    targets = [_Target(9, 'x', None)]
    assert mapstore.in_use_by(targets, str(tmp_path / 'run')) == []


# ---------------------------------------------------------------------------
# Classification: what may be deleted at all
#
# delete_run() unlinks the files it classified and rmdir's the directory, so
# "what counts as run output" is a safety boundary, not a display detail.
# ---------------------------------------------------------------------------

def test_a_run_holding_only_known_files_is_deletable(tmp_path):
    _make_run(tmp_path, 'run')
    run, = mapstore.scan([str(tmp_path)])
    assert run.unknown == []
    assert run.deletable_reason == ''
    assert run.as_dict()['deletable'] is True


def test_a_foreign_file_makes_a_run_undeletable_and_names_it(tmp_path):
    """The panel removes run output. A directory holding someone's notes is
    not one this module understands, and rmdir would fail anyway -- so it
    says so up front instead of half-deleting and then failing."""
    path = _make_run(tmp_path, 'run')
    open(os.path.join(path, 'notes.txt'), 'w').close()
    run, = mapstore.scan([str(tmp_path)])
    assert run.unknown == ['notes.txt']
    assert 'notes.txt' in run.deletable_reason
    assert run.as_dict()['deletable'] is False


def test_an_unknown_subdirectory_makes_a_run_undeletable(tmp_path):
    path = _make_run(tmp_path, 'run')
    os.makedirs(os.path.join(path, 'scratch'))
    run, = mapstore.scan([str(tmp_path)])
    assert 'scratch' in run.unknown
    assert run.as_dict()['deletable'] is False


def test_a_rosbag_subdirectory_is_recognised_run_output(tmp_path):
    """race_diagnostics records a bag into the run directory (see
    race_diagnostics/summarize_run.py, which reads it back as 'rosbag for
    offline replay'). Without this every diagnosed run would be
    undeletable -- and a bag is the largest thing there after the pose
    graph, so those are exactly the runs worth removing."""
    path = _make_run(tmp_path, 'run')
    bag = os.path.join(path, 'bag')
    os.makedirs(bag)
    open(os.path.join(bag, 'bag_0.mcap'), 'w').close()
    open(os.path.join(bag, 'metadata.yaml'), 'w').close()
    run, = mapstore.scan([str(tmp_path)])
    assert run.unknown == []
    assert run.known_dirs == ['bag']
    assert 'rosbag' in run.contents
    assert run.deletable_reason == ''


def test_a_symlink_inside_a_run_makes_it_undeletable(tmp_path):
    """Even named map.pgm. Unlinking a link is harmless, but classifying one
    as run output would let a link sit inside an otherwise clean run."""
    path = _make_run(tmp_path, 'run', pgm=None)
    outside = tmp_path / 'elsewhere.pgm'
    outside.write_bytes(b'P5 1 1 255\n')
    os.symlink(str(outside), os.path.join(path, 'map.pgm'))
    run, = mapstore.scan([str(tmp_path)])
    assert 'map.pgm' in run.unknown
    assert run.as_dict()['deletable'] is False


def test_an_empty_run_directory_is_not_deletable(tmp_path):
    """A4, empty input. Nothing to remove is not the same as 'delete me'."""
    os.makedirs(str(tmp_path / 'hollow'))
    run, = mapstore.scan([str(tmp_path)])
    assert 'is empty' in run.deletable_reason


def test_an_undeletable_run_is_refused_by_resolve_delete(tmp_path):
    """The classifier verdict is enforced, not merely displayed."""
    path = _make_run(tmp_path, 'run')
    open(os.path.join(path, 'notes.txt'), 'w').close()
    roots = [os.path.realpath(str(tmp_path))]
    runs = mapstore.scan(roots)
    run, reason = mapstore.resolve_delete(runs, 'run', 'run', roots)
    assert run is None
    assert 'notes.txt' in reason


def test_a_foreign_file_appearing_after_the_scan_stops_the_delete(tmp_path):
    """delete_run re-classifies immediately before touching anything, so a
    file written between the scan and the press aborts rather than being
    swept up. Nothing may be removed in that case."""
    path = _make_run(tmp_path, 'run')
    roots = [os.path.realpath(str(tmp_path))]
    runs = mapstore.scan(roots)
    open(os.path.join(path, 'appeared.bin'), 'w').close()
    ok, reason, freed = mapstore.delete_run(runs[0], roots)
    assert ok is False
    assert freed == 0
    assert 'appeared.bin' in reason
    for name in ('map.yaml', 'map.pgm', 'posegraph.posegraph', 'appeared.bin'):
        assert os.path.exists(os.path.join(path, name))


def test_deleting_removes_a_rosbag_subdirectory_too(tmp_path):
    path = _make_run(tmp_path, 'run')
    bag = os.path.join(path, 'bag')
    os.makedirs(bag)
    open(os.path.join(bag, 'bag_0.mcap'), 'w').close()
    roots = [os.path.realpath(str(tmp_path))]
    runs = mapstore.scan(roots)
    ok, detail, _ = mapstore.delete_run(runs[0], roots)
    assert ok is True, detail
    assert not os.path.exists(path)


def test_deleting_leaves_the_root_itself_alone(tmp_path):
    """Invariant: the unit is the run, never the directory holding runs."""
    _make_run(tmp_path, 'a')
    _make_run(tmp_path, 'b')
    roots = [os.path.realpath(str(tmp_path))]
    runs = mapstore.scan(roots)
    mapstore.delete_run(mapstore.find(runs, 'a'), roots)
    assert os.path.isdir(str(tmp_path))
    assert os.path.isdir(str(tmp_path / 'b'))


# ---------------------------------------------------------------------------
# The digest -- what actually catches a stale tab
# ---------------------------------------------------------------------------

def test_the_digest_is_stable_across_two_scans_of_an_unchanged_run(tmp_path):
    """Invariant: it must not churn, or every delete would be refused."""
    _make_run(tmp_path, 'run')
    first, = mapstore.scan([str(tmp_path)])
    second, = mapstore.scan([str(tmp_path)])
    assert first.digest == second.digest
    assert first.digest != ''


def test_the_digest_changes_when_a_file_is_added(tmp_path):
    path = _make_run(tmp_path, 'run')
    before, = mapstore.scan([str(tmp_path)])
    open(os.path.join(path, 'raceline_optimized.csv'), 'w').close()
    after, = mapstore.scan([str(tmp_path)])
    assert after.digest != before.digest


def test_the_digest_changes_when_a_file_grows(tmp_path):
    """The case this exists for: map_saver still writing when the tab
    listed the run."""
    path = _make_run(tmp_path, 'run')
    before, = mapstore.scan([str(tmp_path)])
    with open(os.path.join(path, 'map.pgm'), 'ab') as handle:
        handle.write(b'x' * 100)
    after, = mapstore.scan([str(tmp_path)])
    assert after.digest != before.digest


def test_two_different_runs_have_different_digests(tmp_path):
    _make_run(tmp_path, 'a')
    _make_run(tmp_path, 'b', filler=b'different')
    runs = mapstore.scan([str(tmp_path)])
    assert runs[0].digest != runs[1].digest


def test_a_stale_digest_refuses_the_delete_and_says_to_refresh(tmp_path):
    """A8: the refusal is the half that matters. A tab that listed this run
    before its map existed must not be able to delete the map."""
    path = _make_run(tmp_path, 'run', map_yaml=False, pgm=None)
    roots = [os.path.realpath(str(tmp_path))]
    stale = mapstore.scan(roots)
    # The map lands after that listing.
    _make_run(tmp_path, 'run')
    fresh = mapstore.scan(roots)
    run, reason = mapstore.resolve_delete(
        fresh, 'run', 'run', roots, digest=stale[0].digest)
    assert run is None
    assert 'refresh' in reason
    assert os.path.exists(os.path.join(path, 'map.yaml'))


def test_a_matching_digest_allows_the_delete(tmp_path):
    _make_run(tmp_path, 'run')
    roots = [os.path.realpath(str(tmp_path))]
    runs = mapstore.scan(roots)
    run, reason = mapstore.resolve_delete(
        runs, 'run', 'run', roots, digest=runs[0].digest)
    assert reason == ''
    assert run is not None


def test_no_digest_at_all_still_works_for_a_caller_without_a_snapshot(tmp_path):
    """A test or a future CLI has nothing to compare. The browser always
    sends one; that is a caller property, not a module one."""
    _make_run(tmp_path, 'run')
    roots = [os.path.realpath(str(tmp_path))]
    runs = mapstore.scan(roots)
    run, reason = mapstore.resolve_delete(runs, 'run', 'run', roots, digest=None)
    assert run is not None and reason == ''


def test_an_empty_string_digest_is_treated_as_a_mismatch_not_as_absent(tmp_path):
    """Boundary between "no digest" (None) and "a digest that is wrong" ('').
    Conflating them would let a client opt out of the check by sending ''."""
    _make_run(tmp_path, 'run')
    roots = [os.path.realpath(str(tmp_path))]
    runs = mapstore.scan(roots)
    run, reason = mapstore.resolve_delete(runs, 'run', 'run', roots, digest='')
    assert run is None
    assert 'refresh' in reason


# ---------------------------------------------------------------------------
# cited_by -- a run some test reads as its oracle
# ---------------------------------------------------------------------------

def test_a_run_cited_by_a_test_is_flagged(tmp_path):
    """Spec: src/pure_pursuit/test/test_map_despeckle.py names
    ~/.ros/racerbot_auto/20260727-200103/map.yaml as its recorded-map oracle
    and skips when it is absent. Deleting it turns a real test into a
    permanent skip, which is worth seeing before the confirm."""
    root = tmp_path / 'racerbot_auto'
    root.mkdir()
    _make_run(root, '20260727-200103')
    run, = mapstore.scan([str(root)])
    assert 'test_map_despeckle' in run.cited_by
    assert run.as_dict()['cited_by']


def test_an_uncited_run_carries_no_citation(tmp_path):
    root = tmp_path / 'racerbot_auto'
    root.mkdir()
    _make_run(root, '20260101-000000')
    run, = mapstore.scan([str(root)])
    assert run.cited_by == ''


def test_a_cited_run_is_still_deletable(tmp_path):
    """Flagged, never refused. Keeping a map forever is not this panel's
    call to make -- but it must not be a surprise either."""
    root = tmp_path / 'racerbot_auto'
    root.mkdir()
    _make_run(root, '20260727-200103')
    roots = [os.path.realpath(str(root))]
    runs = mapstore.scan(roots)
    run, reason = mapstore.resolve_delete(
        runs, '20260727-200103', '20260727-200103', roots)
    assert run is not None, reason
