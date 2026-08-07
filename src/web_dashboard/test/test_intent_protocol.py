"""
Unit tests for the dashboard's handling of /drive_intent.

Two separate concerns are covered here:

  * `protocol.intent_message` -- the envelope the browser receives, and
  * the ingest guard: the dashboard treats every intent message as
    untrusted input, because the publisher may be a teammate's node from
    racerbot_a/racerbot_b or a schema version this dashboard predates.
    A malformed message must be dropped, not rendered and not raised.

The ingest logic itself lives in DashboardNode.intent_callback, which
needs rclpy; what is tested here is the decode/validate pair that
callback is built from, exercised against exactly the payloads a
hand-rolled C++ publisher is most likely to get wrong.

    python3 -m pytest src/web_dashboard/test/test_intent_protocol.py -v
"""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'drive_intent'))
from drive_intent import schema  # noqa: E402
from web_dashboard import protocol  # noqa: E402


def valid_payload(**overrides):
    payload = schema.build(
        'gap_follow_node', 'gap_follow',
        reason='clear corridor ahead',
        path=[(0.0, 0.0, 0.0, 2.0), (1.0, 0.05, 0.1, 2.0)],
        commanded_path=[(0.0, 0.0, 0.0, 1.8), (0.9, 0.04, 0.1, 1.8)],
        desired_speed=2.0, commanded_speed=1.8, horizon_s=1.5,
        factors=schema.bind_min([
            schema.factor('curve cap', 2.4),
            schema.factor('clearance cap', 1.8),
        ]),
    )
    payload.update(overrides)
    return payload


# ============================================================================
# The envelope
# ============================================================================

def test_intent_message_wraps_the_payload_without_rewriting_it():
    """A pass-through on purpose: the schema is owned by
    drive_intent/schema.py. If this file paraphrased it there would be a
    third copy of the field names to keep in sync."""
    payload = valid_payload()
    msg = protocol.intent_message(payload)
    assert msg['type'] == 'intent'
    assert msg['intent'] == payload


def test_intent_message_carries_a_server_stamp_alongside_the_cars():
    """Two clocks, two fields. A laptop whose clock disagrees with the
    Jetson's can still tell a stale arrow from a fresh one."""
    payload = valid_payload(stamp=1000.0)
    msg = protocol.intent_message(payload)
    assert msg['intent']['stamp'] == pytest.approx(1000.0)
    assert msg['stamp'] != pytest.approx(1000.0)


def test_the_envelope_survives_json_serialization():
    """It goes out over a WebSocket as JSON text; anything json.dumps
    cannot express would break the socket, not just this message."""
    msg = protocol.intent_message(valid_payload())
    assert json.loads(json.dumps(msg))['intent']['state'] == 'gap_follow'


# ============================================================================
# The ingest guard
# ============================================================================

def test_a_well_formed_message_is_accepted():
    text = schema.encode(valid_payload())
    assert schema.validate(schema.decode(text)) is None


@pytest.mark.parametrize('text', [
    '',
    'not json at all',
    '{"v": 1',
    '[]',
    'null',
    '3',
])
def test_undecodable_data_raises_valueerror_rather_than_something_exotic(text):
    """intent_callback catches ValueError specifically; anything else
    would escape into a subscription callback."""
    with pytest.raises(ValueError):
        schema.decode(text)


def test_a_payload_from_a_newer_schema_is_refused_not_half_drawn():
    assert schema.validate(valid_payload(v=2)) is not None


def test_a_missing_severity_is_refused():
    payload = valid_payload()
    del payload['severity']
    assert schema.validate(payload) is not None


def test_a_path_of_the_wrong_shape_is_refused():
    """The most likely mistake in a hand-rolled C++ JSON writer: emitting
    an array of arrays instead of an array of objects."""
    assert schema.validate(valid_payload(path=[[0.0, 0.0, 1.0]])) is not None


def test_a_nan_from_a_printf_style_publisher_is_refused():
    """C's printf("%f", nan) writes `nan`; a slightly better writer emits
    the JSON5-ish bare NaN that Python accepts and browsers reject. Either
    way it must not reach the browser."""
    payload = json.loads(json.dumps(valid_payload()).replace(
        '"commanded_speed": 1.8', '"commanded_speed": NaN'))
    assert schema.validate(payload) is not None


def test_an_oversized_path_is_refused_before_it_reaches_a_phone():
    payload = valid_payload(
        path=[{'x': 0.0, 'y': 0.0, 'v': 1.0}] * (schema.MAX_PATH_POINTS + 1))
    assert schema.validate(payload) is not None


def test_the_reason_may_legitimately_be_absent():
    """The car only re-sends its explanation on state changes and on a
    slow period, so most messages carry no reason at all. Treating that as
    malformed would drop nearly every message."""
    payload = valid_payload()
    del payload['reason']
    assert schema.validate(payload) is None


def test_validation_does_not_depend_on_optional_geometry():
    payload = valid_payload()
    payload.pop('targets')
    payload.pop('commanded_path')
    assert schema.validate(payload) is None
