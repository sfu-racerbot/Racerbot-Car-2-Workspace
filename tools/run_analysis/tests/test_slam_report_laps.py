"""Lap verdicts from the supervisor's recorded messages; run with pytest tools/run_analysis/tests/."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src' / 'race_diagnostics'))
from race_diagnostics import run_events
from run_analysis import slam_report

# oracle: source auto_map_race_node.py:1627-1631, values filled in.
CEILING = ('Lap 5 did not pass the quality check (heading_error=29.2deg > 15.0deg), '
           'but the 5-lap ceiling is reached -- accepting it anyway rather than mapping indefinitely.')
# oracle: source auto_map_race_node.py:1818-1822, values filled in.
PROFILE = 'Generated 100-point racing profile at /tmp/raceline_profiled.csv (0.50-2.00m/s).'


def _closure(n, total=3):
    # oracle: source auto_map_race_node.py:1581-1591, values filled in.
    return (f'Closed mapping lap {n}/{total} detected (29.0m, 365deg of turning, '
            '1.01 revolutions, 163 samples, closest approach to the start 0.77m '
            'against a 1.50m gate, 11 SLAM corrections absorbed).')


def _run(tmp_path, messages):
    records = [dict(t=float(i + 1), msg=msg) for i, msg in enumerate(messages)]
    return slam_report.load_inputs(tmp_path, load_slam=lambda p: None,
                                   load_scans=lambda p: None, load_rosout=lambda p: records)


def test_ceiling_failure_parsed():
    # oracle: source auto_map_race_node.py:1623-1631.
    assert run_events.parse_lap_quality(CEILING) == dict(lap=5, passed=False,
        metric='heading_error', value=29.2, limit=15.0, accepted_at_ceiling=True)


def test_intermediate_lap_message_parsed():
    # oracle: source auto_map_race_node.py:1613-1620.
    assert run_events.parse_lap_unjudged('Lap 2/3 closed; keep holding LB.') == {
        'lap': 2, 'of': 3, 'judged': False}
    assert run_events.parse_lap_unjudged('Lap 2/3 closed') is None


def test_recorded_style_lap_verdicts(tmp_path):
    # oracle: source auto_map_race_node.py:1592-1623 and 1650-1654; values from 19:49 log.
    messages = [_closure(1), _closure(2), 'Lap 2/3 closed; keep holding LB.',
                _closure(3), 'Lap 3 did not pass the quality check (heading_error=29.5deg > 15.0deg (50% of the 30deg gate)) and the 5-lap ceiling has not been reached -- recording one more lap instead of accepting it. Keep holding LB.',
                _closure(4), 'Lap 4 did not pass the quality check (heading_error=29.2deg > 15.0deg (50% of the 30deg gate)) and the 5-lap ceiling has not been reached -- recording one more lap instead of accepting it. Keep holding LB.']
    lines, summary = slam_report.build_report(_run(tmp_path, messages))
    assert 'quality not judged (only the last lap is)' in next(x for x in lines if 'lap 1/3:' in x)
    assert 'quality not judged (only the last lap is)' in next(x for x in lines if 'lap 2/3:' in x)
    assert 'quality failed heading_error=29.5' in next(x for x in lines if 'lap 3/3:' in x)
    assert 'quality failed heading_error=29.2' in next(x for x in lines if 'lap 4/3:' in x)
    assert [x['verdict'] for x in summary['Laps']] == ['not judged', 'not judged', 'failed', 'failed']


def test_profile_evidence_proves_pass(tmp_path):
    # oracle: source auto_map_race_node.py:1623-1634 and 1818-1822.
    lines, summary = slam_report.build_report(_run(tmp_path, [_closure(3), PROFILE]))
    assert 'quality passed' in next(x for x in lines if 'lap 3/3:' in x)
    assert summary['Laps'][0]['verdict'] == 'passed'


def test_no_failure_or_profile_is_unknown(tmp_path):
    # oracle: source auto_map_race_node.py:1623-1643: no success line at the check itself.
    lines, summary = slam_report.build_report(_run(tmp_path, [_closure(3)]))
    assert 'quality unknown' in next(x for x in lines if 'lap 3/3:' in x)
    assert summary['Laps'][0]['verdict'] == 'unknown'


def test_ceiling_failure_verdict(tmp_path):
    # oracle: source auto_map_race_node.py:1625-1634: failure is accepted, not passed.
    lines, summary = slam_report.build_report(_run(tmp_path, [_closure(5), CEILING, PROFILE]))
    assert 'failed heading_error=29.2 deg > 15.0 deg (accepted at the ceiling)' in next(x for x in lines if 'lap 5/3:' in x)
    assert summary['Laps'][0]['verdict'] == 'failed (accepted at the ceiling)'
