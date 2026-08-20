"""The racing-line message the dashboard draws.

Its whole purpose is to answer "is pure pursuit actually racing yet"
visually: the line only exists once a controller has loaded a profile. On
the 2026-08-19 run pure pursuit never got control -- it logged
`waiting_for_profile` 176 times while gap_follow drove the entire session
-- and nothing on screen distinguished that from a slow race.

    python3 -m pytest src/web_dashboard/test/test_racing_line_protocol.py -v
"""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from web_dashboard import protocol  # noqa: E402


def a_line(points=None, **overrides):
    payload = {
        'node': 'pure_pursuit_node',
        'source': '/home/x/raceline_profiled.csv',
        'closed': True,
        'points': points if points is not None else [
            [0.0, 0.0, 2.0], [1.0, 0.0, 1.8], [1.0, 1.0, 1.2],
        ],
        'decimation': 1,
        'length_m': 4.0,
        'speed_min': 1.2,
        'speed_max': 2.0,
        'stamp': 1.0,
    }
    payload.update(overrides)
    return payload


def test_the_line_is_passed_through_unchanged():
    """Rounding and decimation happen where the waypoint count is known --
    on the publishing side. Re-encoding here would only lose precision."""
    line = a_line()
    message = protocol.racing_line_message(line)
    assert message['type'] == 'racing_line'
    assert message['line'] == line
    assert message['line']['points'][0] == [0.0, 0.0, 2.0]


def test_the_message_is_json_serializable():
    # It goes straight down a WebSocket as text.
    encoded = json.dumps(protocol.racing_line_message(a_line()))
    assert json.loads(encoded)['line']['node'] == 'pure_pursuit_node'


def test_none_clears_the_line():
    """A controller shutting down should take its line off the map, not
    leave a stale one there implying it is still racing."""
    message = protocol.racing_line_message(None)
    assert message['type'] == 'racing_line'
    assert message['line'] is None


def test_the_message_carries_what_the_readout_needs():
    message = protocol.racing_line_message(a_line())
    line = message['line']
    for key in ('points', 'length_m', 'speed_min', 'speed_max', 'node', 'closed'):
        assert key in line, f'the browser readout reads {key}'


def test_a_stamp_is_added():
    assert protocol.racing_line_message(a_line())['stamp'] > 0
