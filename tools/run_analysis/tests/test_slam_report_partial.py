"""Partial SLAM report regressions; runner: pytest tools/run_analysis/tests/."""
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from run_analysis import slam_report as report


def _metadata(path, topics):
    bag = path / 'bag'
    bag.mkdir()
    entries = [{'topic_metadata': {'name': topic}} for topic in topics]
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(
        {'rosbag2_bagfile_information': {'topics_with_message_count': entries}}))


def test_missing_scans_keeps_partial_report_and_cli(tmp_path, capsys):
    _metadata(tmp_path, ['/odom', '/odom/imu', '/rosout'])
    def no_slam(path):
        raise report.slam_bag.NoSlamMapping('SLAM did not map this run')
    # oracle: a 3-4-5 triangle followed by a 3 m leg has total length 8 m.
    odom = np.array([[0, 0, 0, 0], [1, 3, 4, 0], [2, 6, 4, 0]])
    gyro = np.array([[0, 0, 0, 0], [1, 3, 4, .1], [2, 6, 4, .2]])
    kwargs = dict(load_slam=no_slam, read_inputs=lambda bag, topics: {
        'odom': odom, 'odom_imu': gyro, 'rosout': []})
    run = report.load_inputs(tmp_path, **kwargs)
    lines, summary = report.build_report(run)
    assert lines[0] == 'No SLAM in this run: SLAM did not map this run'
    assert lines[lines.index('Map consistency:') + 1] == '  no scans: topic /scan not recorded'
    assert summary['Setup']['distance_m'] == 8
    assert '  /odom distance: 8.00 m' in '\n'.join(lines)
    assert summary['Wheel vs scan-matched distance']['wheel_m'] == 8
    assert '/odom vs /odom/imu' in summary['Heading drift']
    assert 'Laps' in summary and 'Setup' in summary and 'Heading drift' in summary
    assert report.main([str(tmp_path), '--no-plots'], **kwargs) == 0
    assert capsys.readouterr().out.startswith(lines[0] + '\n')


def test_scans_are_usable_without_drive(tmp_path):
    _metadata(tmp_path, ['/scan'])
    grid = np.zeros((40, 40), dtype=np.int8)
    grid[:, 20] = 100
    pose = np.array([[0, 0, .5, 0], [2, 0, .5, 0]])
    slam = {'pose': pose, 'map': (.05, 0., 0., grid)}
    scans = {'scan_t': np.array([1.]), 'scan': np.array([[.76]]),
             'angle_min': 0., 'angle_increment': 0., 'range_min': .05, 'range_max': 10.}
    run = report.load_inputs(tmp_path, load_slam=lambda p: slam,
                             read_inputs=lambda bag, topics: scans)
    lines, summary = report.build_report(run)
    # oracle: .26 m laser offset + .76 m range hits wall at x=1.0 m.
    assert summary['Map consistency']['overall'] == 1.
    assert '  overall: 100.0%' in lines


def test_input_reader_errors_propagate(tmp_path):
    _metadata(tmp_path, ['/scan'])
    def broken(path):
        raise RuntimeError('broken bag')
    def broken_reader(bag, topics):
        raise RuntimeError('broken bag')
    # oracle: only missing topics are optional; reader errors are not.
    with pytest.raises(RuntimeError, match='broken bag'):
        report.load_inputs(tmp_path, load_slam=lambda p: {}, read_inputs=broken_reader)
    with pytest.raises(RuntimeError, match='broken bag'):
        report.load_inputs(tmp_path, load_slam=broken, read_inputs=broken_reader)
    def malformed_reader(bag, topics):
        raise ValueError('malformed scan')
    with pytest.raises(ValueError, match='malformed scan'):
        report.load_inputs(tmp_path, load_slam=lambda p: {}, read_inputs=malformed_reader)


def test_cached_inputs_reused_and_refresh_reads_again(tmp_path):
    _metadata(tmp_path, ['/odom', '/rosout'])
    rows = np.array([[0., 0., 0., 0.], [1., 3., 4., 0.]])
    records = [{'t': 1., 'name': 'slam_toolbox', 'msg': 'Message Filter dropping message'}]
    def reader(bag, topics):
        assert topics == {'/odom', '/rosout'}  # oracle: bag metadata topic list
        return {'odom': rows, 'rosout': records}
    first = report.load_inputs(tmp_path, load_slam=lambda p: {}, read_inputs=reader)
    def unavailable(bag, topics):
        raise RuntimeError('reader invoked')
    second = report.load_inputs(tmp_path, load_slam=lambda p: {}, read_inputs=unavailable)
    # oracle: same persisted topic inputs yield the same 3-4-5 path and drop.
    assert report.build_report(first)[1]['Setup']['distance_m'] == 5
    assert report.build_report(second)[1]['Setup']['distance_m'] == 5
    assert second['drops'] == [1.]
    assert second['missing_topics']['/odom/imu'] == 'topic /odom/imu not recorded'
    with pytest.raises(RuntimeError, match='reader invoked'):
        report.load_inputs(tmp_path, load_slam=lambda p: {}, read_inputs=unavailable, refresh=True)


def test_scan_drops_only_count_slam_toolbox_logger(tmp_path):
    text = 'Message Filter dropping message: queue full'
    records = [dict(t=1, name='slam_toolbox', msg=text),
               dict(t=2, name='other_node', msg=text),
               dict(t=3, name='slam_toolbox', msg='other message')]
    run = report.load_inputs(tmp_path, load_slam=lambda p: {},
                             load_scans=lambda p: None, load_rosout=lambda p: records)
    # oracle: slam_launch.py names the node slam_toolbox; only its drop counts.
    assert run['drops'] == [1]
    assert report.build_report(run)[1]['Scan drops']['count'] == 1


def test_plain_value_error_with_no_slam_text_propagates(tmp_path):
    def broken(path):
        raise ValueError(f'{path}: unrelated invalid input')
    # oracle: only the distinct no-map exception denotes absent SLAM.
    with pytest.raises(ValueError, match='SLAM did not map this run: unrelated invalid input'):
        report.load_inputs(tmp_path / 'SLAM did not map this run', load_slam=broken,
                           load_scans=lambda p: None,
                           load_rosout=lambda p: [])


def test_empty_bag_raises_typed_no_slam_error(tmp_path):
    import rosbag2_py
    bag = tmp_path / 'bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    del writer
    # oracle: no messages means no map-to-odometry transform; the typed
    # exception remains a ValueError for existing callers.
    with pytest.raises(report.slam_bag.NoSlamMapping, match='SLAM did not map this run') as exc:
        report.slam_bag.read_bag(bag)
    assert isinstance(exc.value, ValueError)
