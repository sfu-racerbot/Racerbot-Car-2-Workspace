"""Exact replay contracts. Runner: rb python3 -m pytest tools/run_analysis/tests/.

S4 step 6 mutation checks, observed failing individually and reverted:
  a. silently skip missing scans -> missing_scan_refused_including_first_pose (both cases)
  b. omit yaw wrapping -> yaw_wrap (359.98 degrees instead of 0.02)
  c. omit PGM row flip -> map_bytes_and_yaml (first raster byte differs)
  d. omit feeder child_env -> exact_children_isolated (domain 79 instead of 77)
The added bracket and pose-edge tests also failed against deliberate function stubs.

Round-1 step 4, each observed failing individually, then reverted and rerun green:
  a. wait at every scan -> test_feed_every_scan_wait_only_at_planned
  b. make extra poses fatal -> test_extra_pose_counted_not_fatal
Checkpoint and boundary-step stubs also failed the timeout/reset/late-pose and
unbracketed-boundary tests, respectively. No original assertions were weakened.

Round-2 step 4, observed failing separately, reverted and rerun green:
  a. skip sacrificial scans -> test_forced_plan_consumes_three_startup_callbacks
  b. override all-scans admission -> test_all_scans_keeps_recorded_admission
Forced-plan/override stubs failed the remaining new cases; using profile values
instead of the snapshot failed test_admission_audit_uses_snapshot_not_profile.

Round-3 step 4, observed failing separately, reverted and rerun green:
  a. retain map-parented TF -> test_tf_filter_drops_only_map_parented_transforms
  b. bracket only the first pair -> test_tf_brackets_cover_every_needed_pair_in_recorded_order
Chain/preparation stubs and a constant bracket result failed the remaining new
TF edge and integration tests. All original assertions remain in place.

Review step 3: review-fix1 on the copied 20261026-194913 matched all 315 poses,
with zero missing/extra and exactly the round-3 maxima: 1.171361057304152e-13 m,
3.053332494204976e-13 degrees, no first_over_bar. The unchanged bag fallback
also ran after three service failures and recorded map_saved='from_bag'.

Review step 4: reverting containment failed nine cases, including
test_replay_output_inside_existing_run_refused[source]; reverting the forced
plan fix failed four, including test_forced_plan_refuses_reusing_first_scan_as_sacrifice.
Removing the analysis exception failed both test_replay_output_under_analysis_allowed
cases; a <=3 fencepost break failed the exact-three-intervening acceptance test.
Every deliberate break was restored and its focused tests rerun green.
"""
import math
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from run_analysis import slam_replay as replay


def test_plan_preserves_pose_order():
    # oracle: invariant - only processed stamps, in recorded pose order.
    assert replay.exact_scan_plan([10, 20, 30], [30, 10]) == [30, 10]


@pytest.mark.parametrize('poses', [[10, 99], [99, 10]])
def test_missing_scan_refused_including_first_pose(poses):
    # oracle: spec S4 controller ruling - every processed pose has its scan.
    with pytest.raises(replay.StampMismatch, match=r'1.*99'):
        replay.exact_scan_plan([10, 20], poses)


def test_duplicate_scan_refused():
    # oracle: invariant - a stamp must identify exactly one scan.
    with pytest.raises(replay.StampMismatch, match='duplicate.*10'):
        replay.exact_scan_plan([10, 10], [10])


def test_empty_poses():
    # oracle: closed form - no processed poses implies no scans to feed.
    assert replay.exact_scan_plan([10], []) == []


def test_identical_pose_sequence():
    # oracle: closed form - subtraction of identical poses is zero.
    assert replay.pose_sequence_error([(10, 1, -2, 0)], [(10, 1, -2, 0)]) == {
        'matched': 1, 'missing_in_replay': 0, 'extra_in_replay': 0,
        'max_position_m': 0.0, 'max_yaw_deg': 0.0, 'first_over_bar': None}


def test_position_over_bar():
    # oracle: closed form - displacement sqrt(.002**2) = 2 mm exceeds 1 mm.
    result = replay.pose_sequence_error([(10, 0, 0, 0)], [(10, .002, 0, 0)])
    assert result['max_position_m'] == pytest.approx(.002, abs=1e-12)
    assert result['first_over_bar'] == 10


def test_yaw_wrap():
    # oracle: closed form - these angles differ by .02 degrees on the circle.
    result = replay.pose_sequence_error([(10, 0, 0, math.radians(179.99))],
                                        [(10, 0, 0, math.radians(-179.99))])
    assert result['max_yaw_deg'] == pytest.approx(.02, abs=1e-10)  # degrees
    assert result['first_over_bar'] is None


def test_missing_extra():
    # oracle: set difference {10,20} vs {20,30} has one member each way.
    result = replay.pose_sequence_error([(10, 0, 0, 0), (20, 0, 0, 0)],
                                        [(20, 0, 0, 0), (30, 0, 0, 0)])
    assert (result['matched'], result['missing_in_replay'], result['extra_in_replay']) == (1, 1, 1)


def test_map_bytes_and_yaml(tmp_path):
    # oracle: spec nav2_map_server/src/map_io.cpp saveMapToFile trinary, y flipped.
    pgm, meta = replay.map_from_bag_files([[0, 100, -1], [-1, 0, 100]], .05, (-2, 3), tmp_path)
    assert pgm.read_bytes() == b'P5\n3 2\n255\n' + bytes([205, 254, 0, 254, 0, 205])
    assert yaml.safe_load(meta.read_text()) == {
        'image': 'map.pgm', 'mode': 'trinary', 'resolution': .05,
        'origin': [-2, 3, 0], 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .25}


def test_exact_rate_refused(capsys):
    # oracle: spec - lockstep has no playback rate.
    with pytest.raises(SystemExit) as exc:
        replay.main(['unused', '--exact', '--rate', '2'])
    assert exc.value.code == 2
    assert 'not allowed' in capsys.readouterr().err


def test_exact_driver_refusal(tmp_path, monkeypatch, capsys):
    # oracle: spec - same fake /proc boundary as test_slam_replay.py.
    proc = tmp_path / '123'
    proc.mkdir()
    (proc / 'cmdline').write_bytes(b'/opt/ros/urg_node')
    monkeypatch.setattr(replay, 'PROC_ROOT', tmp_path)
    assert replay.main(['unused', '--exact']) == 2
    assert 'driver processes' in capsys.readouterr().err


def test_exact_children_isolated():
    # oracle: spec - every planned process, including feeder, domain 77 localhost.
    children = replay.exact_children('params.yaml', 'bag', '/odom/imu', {'ROS_DOMAIN_ID': '79'})
    assert {name for name, argv, env in children} == {'slam_toolbox', 'exact_feed'}
    for name, argv, env in children:
        assert env['ROS_DOMAIN_ID'] == '77', name
        assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST', name
        assert env['ROS_STATIC_PEERS'] == '', name


def test_odometry_brackets():
    # oracle: closed form - first sample >= scan, including equality, no later sample.
    assert replay.odometry_brackets([10, 20, 30, 40], [15, 20, 31]) == [2, 2, 4]


@pytest.mark.parametrize('odoms', [[], [10, 20]])
def test_missing_odometry_bracket_refused(odoms):
    # oracle: spec controller ruling - no future bracket refuses with scan stamp.
    with pytest.raises(replay.StampMismatch, match='bracket.*21'):
        replay.odometry_brackets(odoms, [21])


@pytest.mark.parametrize('distance,first', [(0, None), (-.001, None), (.0010001, 10)])
def test_position_bar_boundary(distance, first):
    # oracle: spec 1 mm, strict exceedance; sign does not change distance.
    result = replay.pose_sequence_error([(10, 0, 0, 0)], [(10, distance, 0, 0)])
    assert result['max_position_m'] == abs(distance)
    assert result['first_over_bar'] == first


def test_empty_pose_sequences():
    # oracle: closed form - empty sets have no matches, differences or errors.
    assert replay.pose_sequence_error([], []) == {
        'matched': 0, 'missing_in_replay': 0, 'extra_in_replay': 0,
        'max_position_m': 0.0, 'max_yaw_deg': 0.0, 'first_over_bar': None}


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_pose_refused(value):
    # oracle: invariant - undefined error must not appear as zero error.
    with pytest.raises(ValueError, match='non-finite.*10'):
        replay.pose_sequence_error([(10, 0, 0, 0)], [(10, value, 0, 0)])


def _write_map_bag(path, grids):
    import rosbag2_py
    from nav_msgs.msg import OccupancyGrid
    from rclpy.serialization import serialize_message
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name='/map', type='nav_msgs/msg/OccupancyGrid', serialization_format='cdr'))
    for index, grid in enumerate(grids):
        msg = OccupancyGrid()
        msg.info.width, msg.info.height = 3, 2
        msg.info.resolution = .5
        msg.info.origin.position.x, msg.info.origin.position.y = -2., 3.
        msg.info.origin.orientation.w = 1.
        msg.data = grid
        writer.write('/map', serialize_message(msg), index + 1)
    del writer  # close and flush before the real reader opens it


def test_fallback_uses_last_map(tmp_path):
    # oracle: spec - use the LAST replay map, not the first; pixel conversion closed form.
    bag = tmp_path / 'bag'
    _write_map_bag(bag, [[0] * 6, [0, 100, -1, -1, 0, 100]])
    out = tmp_path / 'out'
    assert replay.save_map_from_bag(bag, out) == 'from_bag'
    assert (out / 'map.pgm').read_bytes() == b'P5\n3 2\n255\n' + bytes([205, 254, 0, 254, 0, 205])
    assert yaml.safe_load((out / 'map.yaml').read_text())['origin'] == [-2, 3, 0]


def test_fallback_without_map(tmp_path):
    # oracle: invariant - no recorded map cannot be reported as saved.
    bag = tmp_path / 'bag'
    _write_map_bag(bag, [])
    out = tmp_path / 'out'
    assert replay.save_map_from_bag(bag, out) is False
    assert not out.exists()


def test_map_threshold_boundaries(tmp_path):
    # oracle: spec nav2 map_io.cpp: valid <=25 free, >=65 occupied; other values unknown.
    pgm, _ = replay.map_from_bag_files([[-2, -1, 0, 25, 26, 64, 65, 100, 101]], 1, (0, 0), tmp_path)
    assert pgm.read_bytes() == b'P5\n9 1\n255\n' + bytes([205, 205, 254, 254, 205, 205, 0, 0, 205])


@pytest.mark.parametrize('resolution', [0, -.1, math.nan, math.inf])
def test_invalid_map_resolution(tmp_path, resolution):
    # oracle: invariant - a grid cannot have zero, negative or nonfinite cell size.
    with pytest.raises(ValueError, match='resolution.*positive'):
        replay.map_from_bag_files([[0]], resolution, (0, 0), tmp_path)


@pytest.mark.parametrize('grid', [[], [[]]])
def test_empty_map_refused(tmp_path, grid):
    # oracle: invariant - no raster can be saved from an empty grid.
    with pytest.raises(ValueError, match='nonempty 2D'):
        replay.map_from_bag_files(grid, 1, (0, 0), tmp_path)


@pytest.mark.parametrize('key', ['mode', 'sync', 'async', 'synchronous', 'slam_mode'])
def test_exact_profile_mode_override_refused(tmp_path, monkeypatch, capsys, key):
    from .test_slam_replay import make_run
    # oracle: spec - the synchronous choice cannot be changed by a profile.
    run = make_run(tmp_path / 'source')
    profile = tmp_path / 'profile.yaml'
    profile.write_text(yaml.safe_dump({key: 'async'}))
    proc = tmp_path / 'proc'
    proc.mkdir()
    monkeypatch.setattr(replay, 'PROC_ROOT', proc)
    assert replay.main([str(run), '--exact', '--profile', str(profile),
                        '--out-root', str(tmp_path / 'out')]) == 2
    assert 'profile overrides of sync/async mode' in capsys.readouterr().err
    assert not (tmp_path / 'out').exists()


def test_feed_every_scan_wait_only_at_planned():
    # oracle: spec round-1 ruling - preserve bag order; pose stamps are wait checkpoints only.
    assert replay.exact_feed_order([10, 20, 30, 40], [20, 40]) == [
        (10, False), (20, True), (30, False), (40, True)]


def test_extra_pose_counted_not_fatal():
    # oracle: set differences - one matched stamp, one extra, no missing poses.
    accounting = replay.ExactPoseAccounting([20])
    accounting.observe(10)
    accounting.observe(20)
    assert accounting.checkpoint(20) is True
    assert accounting.summary() == {
        'matched_planned': 1, 'missing_planned_stamps': [],
        'extra_pose_stamps': [10], 'timed_out_stamps': []}


def test_ten_consecutive_missing_continue():
    # oracle: spec - failure is strictly MORE than 10 consecutive missing checkpoints.
    accounting = replay.ExactPoseAccounting(range(1, 12))
    results = [accounting.checkpoint(stamp) for stamp in range(1, 11)]
    accounting.observe(11)
    assert accounting.checkpoint(11) is True
    assert results == [False] * 10
    assert accounting.summary()['missing_planned_stamps'] == list(range(1, 11))


def test_eleven_consecutive_missing_fail_with_stamps():
    # oracle: spec - the eleventh consecutive timeout fails, naming all eleven stamps.
    accounting = replay.ExactPoseAccounting(range(101, 112))
    for stamp in range(101, 111):
        assert accounting.checkpoint(stamp) is False
    with pytest.raises(replay.ReplayFailed, match='11 consecutive') as exc:
        accounting.checkpoint(111)
    assert str(list(range(101, 112))) in str(exc.value)


def test_matched_checkpoint_resets_missing_streak():
    # oracle: invariant - two blocks of ten separated by a match are not 20 consecutive misses.
    accounting = replay.ExactPoseAccounting(range(1, 22))
    for stamp in range(1, 11):
        assert accounting.checkpoint(stamp) is False
    accounting.observe(11)
    assert accounting.checkpoint(11) is True
    for stamp in range(12, 22):
        assert accounting.checkpoint(stamp) is False
    assert accounting.summary()['matched_planned'] == 1


def test_late_pose_resolves_final_missing_but_retains_timeout():
    # oracle: invariant - an observed late pose is matched in final accounting, not lost.
    accounting = replay.ExactPoseAccounting([20])
    assert accounting.checkpoint(20) is False
    accounting.observe(20)
    assert accounting.summary() == {
        'matched_planned': 1, 'missing_planned_stamps': [],
        'extra_pose_stamps': [], 'timed_out_stamps': [20]}


def test_unplanned_boundary_scans_sent_without_brackets():
    # oracle: controller boundary ruling - include both edges, never invent odometry.
    assert replay.exact_feed_steps([10, 20], [5, 10, 15, 20, 25], [15]) == [
        (5, False, 1, False), (10, False, 1, True), (15, True, 2, True),
        (20, False, 2, True), (25, False, 2, False)]


@pytest.mark.parametrize('stamp', [5, 25])
def test_planned_boundary_scans_refused(stamp):
    # oracle: controller ruling - ONLY unplanned scans may lack an interpolation bracket.
    with pytest.raises(replay.StampMismatch, match=f'bracket.*{stamp}'):
        replay.exact_feed_steps([10, 20], [5, 15, 25], [stamp])


def test_preflight_refuses_planned_scan_before_first_odometry():
    # oracle: controller coverage rule - a future sample alone cannot bracket a planned scan.
    with pytest.raises(replay.StampMismatch, match='bracket.*5'):
        replay.odometry_brackets([10, 20], [5])


def test_forced_plan_consumes_three_startup_callbacks():
    # oracle: controller ruling and 2.8.5 common.cpp startup counter: first, callbacks 2-4, rest.
    assert replay.forced_scan_plan([5, 10, 20, 30, 40, 50, 60, 70], [10, 50, 70]) == [
        (10, True), (20, False), (30, False), (40, False), (50, True), (70, True)]


@pytest.mark.parametrize('scans', [[10, 20], [5, 10, 20]])
def test_forced_plan_refuses_too_few_sacrificial_scans(scans):
    # oracle: controller rule - three recorded scans immediately before second planned stamp required.
    with pytest.raises(replay.ReplayRefused, match='three.*20'):
        replay.forced_scan_plan(scans, [10, 20])


@pytest.mark.parametrize('poses,expected', [([], []), ([10], [(10, True)])])
def test_forced_plan_without_second_pose(poses, expected):
    # oracle: closed form - no second checkpoint means no startup callbacks need consuming.
    assert replay.forced_scan_plan([10, 20], poses) == expected


def test_forced_admission_overrides_and_audit():
    # oracle: measured 20261026-194913 snapshot; forced values from controller ruling, not output.
    recorded = {'throttle_scans': 2, 'minimum_time_interval': .2,
                'minimum_travel_distance': .25, 'minimum_travel_heading': .25,
                'resolution': .05}
    params = {'slam_toolbox': {'ros__parameters': dict(recorded)}}
    changed, audit = replay.exact_admission_parameters(params, 'forced')
    assert changed == {'slam_toolbox': {'ros__parameters': {
        **recorded, 'throttle_scans': 1, 'minimum_time_interval': 0.0,
        'minimum_travel_distance': 0.0, 'minimum_travel_heading': 0.0}}}
    assert audit == {
        'throttle_scans': {'recorded': 2, 'override': 1},
        'minimum_time_interval': {'recorded': .2, 'override': 0.0},
        'minimum_travel_distance': {'recorded': .25, 'override': 0.0},
        'minimum_travel_heading': {'recorded': .25, 'override': 0.0}}
    assert params == {'slam_toolbox': {'ros__parameters': recorded}}


def test_all_scans_keeps_recorded_admission():
    # oracle: invariant - faithful-admission experiment must not receive forced-mode overrides.
    params = {'slam_toolbox': {'ros__parameters': {
        'throttle_scans': 2, 'minimum_time_interval': .2,
        'minimum_travel_distance': .25, 'minimum_travel_heading': .25}}}
    unchanged, audit = replay.exact_admission_parameters(params, 'all-scans')
    assert unchanged == params
    assert audit == {}


def test_admission_audit_uses_snapshot_not_profile():
    # oracle: spec - 'recorded' names the source snapshot, even if a profile changed a value.
    params = {'slam_toolbox': {'ros__parameters': {'throttle_scans': 9}}}
    changed, audit = replay.exact_admission_parameters(params, 'forced', {'throttle_scans': 2})
    assert changed['slam_toolbox']['ros__parameters']['throttle_scans'] == 1
    assert audit['throttle_scans'] == {'recorded': 2, 'override': 1}


def test_all_scans_selector_requires_exact(tmp_path, monkeypatch, capsys):
    # oracle: invariant - an exact-mode selector must not silently start a rate replay.
    monkeypatch.setattr(replay, 'PROC_ROOT', tmp_path)
    with pytest.raises(SystemExit) as exc:
        replay.main(['unused', '--exact-mode', 'all-scans'])
    assert exc.value.code == 2
    assert 'requires --exact' in capsys.readouterr().err


def _tf(parent, child, stamp=0):
    from types import SimpleNamespace
    return SimpleNamespace(header=SimpleNamespace(
        frame_id=parent, stamp=SimpleNamespace(sec=0, nanosec=stamp)), child_frame_id=child)


def test_tf_filter_drops_only_map_parented_transforms():
    # oracle: spec round-3 ruling - retain exact non-map payloads, including in a mixed message.
    correction = _tf('map', 'world_odom')
    imu = _tf('world_odom', 'wheels')
    wheel = _tf('wheels', 'chassis')
    mixed = [correction, imu, wheel]
    assert replay.filter_tf_transforms(mixed) == [imu, wheel]
    assert replay.filter_tf_transforms([correction]) == []
    assert mixed == [correction, imu, wheel]  # input message is not mutated


def test_tf_chain_derived_from_recorded_tree_and_static_edges():
    # oracle: closed form - sensor edge is static; the two dynamic edges need brackets.
    assert replay.needed_tf_pairs(
        [('wheels', 'chassis'), ('world_odom', 'wheels'), ('elsewhere', 'camera')],
        [('chassis', 'sensor')], 'world_odom', 'sensor') == [
            ('world_odom', 'wheels'), ('wheels', 'chassis')]


def test_tf_brackets_cover_every_needed_pair_in_recorded_order():
    # oracle: closed form - A reaches 15 at message 3; B only reaches it at message 4.
    a, b = ('world_odom', 'wheels'), ('wheels', 'chassis')
    messages = [[(*a, 10)], [(*b, 10)], [(*a, 20)], [(*b, 16)], [(*b, 30)]]
    assert replay.tf_bracket_plan(messages, [a, b], [15, 20]) == [4, 5]


def test_tf_bracket_missing_pair_refuses_with_pair_and_stamp():
    # oracle: spec - another pair's future sample cannot stand in for the missing pair.
    a, b = ('world_odom', 'wheels'), ('wheels', 'chassis')
    with pytest.raises(replay.StampMismatch, match=r'wheels.*chassis.*15'):
        replay.tf_bracket_plan([[(*a, 10), (*b, 10)], [(*a, 20)]], [a, b], [15])


def test_tf_bracket_accepts_exact_stamp_for_each_pair():
    # oracle: closed form - samples exactly at the requested time need no interpolation.
    pair = ('world_odom', 'chassis')
    assert replay.tf_bracket_plan([[(*pair, 15)]], [pair], [15]) == [1]


def test_tf_bracket_requires_earlier_sample_too():
    # oracle: invariant - future-only data cannot interpolate a transform at a past scan.
    pair = ('world_odom', 'chassis')
    with pytest.raises(replay.StampMismatch, match=r'world_odom.*chassis.*15'):
        replay.tf_bracket_plan([[(*pair, 20)]], [pair], [15])


def test_tf_chain_refuses_disconnected_frames():
    # oracle: invariant - a missing transform link cannot be reconstructed by guessing.
    with pytest.raises(replay.ReplayRefused, match=r'world_odom.*sensor'):
        replay.needed_tf_pairs([('world_odom', 'wheels')], [], 'world_odom', 'sensor')


def test_prepare_recorded_tf_preserves_payloads_and_reports_mixed_message():
    from types import SimpleNamespace
    # oracle: closed form - one map correction removed, one mixed message, two dynamic brackets.
    a10, a20 = _tf('world_odom', 'wheels', 10), _tf('world_odom', 'wheels', 20)
    b10, b20 = _tf('wheels', 'chassis', 10), _tf('wheels', 'chassis', 20)
    correction = _tf('map', 'world_odom', 10)
    messages = [SimpleNamespace(transforms=values) for values in
                ([correction, a10, b10], [a20], [b20])]
    filtered, ends, info = replay.prepare_recorded_tf(
        messages, [_tf('chassis', 'sensor')], 'world_odom', 'sensor', [15])
    assert filtered == [[a10, b10], [a20], [b20]]
    assert ends == [3]
    assert info == {'needed_tf_pairs': [['world_odom', 'wheels'], ['wheels', 'chassis']],
                    'map_transforms_dropped': 1, 'mixed_tf_messages': 1, 'tf_messages_available': 3}


@pytest.mark.parametrize('location', ['source', 'source-child', 'other-child'])
def test_replay_output_inside_existing_run_refused(tmp_path, location):
    from .test_slam_replay import make_run
    # oracle: spec storage constraint - only analysis/ may receive files inside existing runs.
    source = make_run(tmp_path / 'source')
    other = make_run(tmp_path / 'other')
    roots = {'source': source, 'source-child': source / 'products',
             'other-child': other / 'products'}
    with pytest.raises(replay.ReplayRefused, match=r'inside existing run.*analysis/'):
        replay.plan_replay(source, 'odom', None, roots[location])


@pytest.mark.parametrize('marker', ['bag', 'metadata'])
def test_output_containment_recognizes_either_run_marker(tmp_path, marker):
    from .test_slam_replay import make_run
    # oracle: spec record_run.py:111-116,180-183 - metadata exists even with record_bag=false.
    source = make_run(tmp_path / 'source')
    other = tmp_path / 'probe-only-or-partial-run'
    (other / marker).mkdir(parents=True)
    with pytest.raises(replay.ReplayRefused, match=r'inside existing run.*analysis/'):
        replay.plan_replay(source, 'odom', None, other / 'products')


def test_output_symlink_into_existing_run_refused(tmp_path):
    from .test_slam_replay import make_run
    # oracle: invariant - a symlink alias cannot evade physical output containment.
    source = make_run(tmp_path / 'source')
    other = make_run(tmp_path / 'other')
    (other / 'products').mkdir()
    alias = tmp_path / 'output-alias'
    alias.symlink_to(other / 'products', target_is_directory=True)
    with pytest.raises(replay.ReplayRefused, match=r'inside existing run.*analysis/'):
        replay.plan_replay(source, 'odom', None, alias)


@pytest.mark.parametrize('which_run', ['source', 'other'])
def test_replay_output_under_analysis_allowed(tmp_path, which_run):
    from .test_slam_replay import make_run
    # oracle: spec - the analysis/ exception permits nested analysis outputs in any run.
    source = make_run(tmp_path / 'source')
    other = make_run(tmp_path / 'other')
    root = {'source': source, 'other': other}[which_run] / 'analysis' / 'replays'
    plan = replay.plan_replay(source, 'odom', None, root)
    assert plan.out_dir == root / f'{source.name}-replay-odom-default'
    assert not root.exists()  # planning is read-only


def test_analysis_symlink_escape_to_another_run_refused(tmp_path):
    from .test_slam_replay import make_run
    # oracle: invariant - a logical analysis/ name does not authorize another run's data directory.
    source = make_run(tmp_path / 'source')
    other = make_run(tmp_path / 'other')
    (source / 'analysis').symlink_to(other, target_is_directory=True)
    with pytest.raises(replay.ReplayRefused, match=r'inside existing run.*analysis/'):
        replay.plan_replay(source, 'odom', None, source / 'analysis' / 'products')


def test_output_containment_checks_every_run_ancestor(tmp_path):
    from .test_slam_replay import make_run
    # oracle: invariant - an inner analysis/ exception cannot bypass an outer run's boundary.
    source = make_run(tmp_path / 'source')
    outer = make_run(tmp_path / 'outer')
    inner = make_run(outer / 'nested')
    with pytest.raises(replay.ReplayRefused, match=r'inside existing run.*analysis/'):
        replay.plan_replay(source, 'odom', None, inner / 'analysis')


def test_replay_output_path_is_resolved(tmp_path):
    from .test_slam_replay import make_run
    # oracle: invariant - store the resolved destination, not an alias that can later be retargeted.
    source = make_run(tmp_path / 'source')
    physical = tmp_path / 'physical-output'
    physical.mkdir()
    alias = tmp_path / 'alias-output'
    alias.symlink_to(physical, target_is_directory=True)
    plan = replay.plan_replay(source, 'odom', None, alias)
    assert plan.out_dir == physical / f'{source.name}-replay-odom-default'


def test_forced_plan_refuses_reusing_first_scan_as_sacrifice():
    # oracle: spec review regression - no scans between 3 and 4, so startup cannot be consumed.
    with pytest.raises(replay.ReplayRefused, match='second planned stamp 4'):
        replay.forced_scan_plan([1, 2, 3, 4], [3, 4])


def test_forced_plan_accepts_exactly_three_intervening_scans_in_order():
    # oracle: closed form - 2,3,4 are the three distinct intervening scans; unused 6,7 stay out.
    plan = replay.forced_scan_plan([0, 1, 2, 3, 4, 5, 6, 7, 8], [1, 5, 8])
    assert plan == [(1, True), (2, False), (3, False), (4, False), (5, True), (8, True)]
    stamps = [stamp for stamp, _ in plan]
    assert all(left < right for left, right in zip(stamps, stamps[1:]))


@pytest.mark.parametrize('scans,poses,bad_stamp', [
    ([1, 2, 3, 4, 5, 6], [1, 5, 4], 4),  # later planned pose would rewind the clock
    ([1, 3, 2, 4, 5], [1, 5], 2),  # sacrificial stamps themselves are out of order
    ([1, 2, 3, 4, 5], [1, 5, 5], 5),  # equality is not strict progress
])
def test_forced_plan_refuses_non_increasing_feed(scans, poses, bad_stamp):
    # oracle: invariant - all planned and sacrificial timestamps must strictly advance replay time.
    with pytest.raises(replay.ReplayRefused, match=f'strictly increasing.*{bad_stamp}'):
        replay.forced_scan_plan(scans, poses)
