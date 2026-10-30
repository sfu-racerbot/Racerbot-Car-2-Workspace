"""Recorded lap messages, runnable with pytest without ROS."""
from race_diagnostics.run_events import parse_lap_closed, parse_lap_quality


LAP2 = ('Closed mapping lap 2/3 detected (28.8m, 353deg of turning, 0.98 revolutions, '
        '166 samples, closest approach to the start 1.09m against a 1.50m gate, '
        '14 SLAM corrections absorbed).')
LAP1 = ('Closed mapping lap 1/3 detected (29.0m, 365deg of turning, 1.01 revolutions, '
        '163 samples, closest approach to the start 0.77m against a 1.50m gate, '
        '11 SLAM corrections absorbed, trimming 30 samples back to the final revolutio...')
FAIL = ('Lap 3 did not pass the quality check (heading_error=29.5deg > 15.0deg '
        '(50% of the 30deg gate)) and the 5-lap ceiling has not been reached -- '
        'recording one more lap instead of accepting it. Keep holding LB.')


def test_closed_recorded_laps_and_truncation():
    # oracle: log map_course-20261026-194912.log, values transcribed above.
    assert parse_lap_closed(LAP2) == dict(lap=2, of=3, distance_m=28.8, turn_deg=353,
        revolutions=0.98, samples=166, closest_m=1.09, gate_m=1.5, corrections=14)
    assert parse_lap_closed(LAP1)['closest_m'] == 0.77
    assert parse_lap_closed(LAP1.split(', trimming')[0])['corrections'] == 11


def test_quality_recorded_failure():
    # oracle: log map_course-20261026-194912.log.
    assert parse_lap_quality(FAIL) == dict(lap=3, passed=False, metric='heading_error',
                                          value=29.5, limit=15.0, accepted_at_ceiling=False)


def test_invalid_lap_lines():
    # oracle: spec: unrelated lines and malformed numbers do not match.
    assert parse_lap_closed('ordinary log') is None
    assert parse_lap_quality('ordinary log') is None
    assert parse_lap_closed(LAP2.replace('1.09m', 'garbagem')) is None
    assert parse_lap_quality(FAIL.replace('29.5deg', 'garbagedeg')) is None
