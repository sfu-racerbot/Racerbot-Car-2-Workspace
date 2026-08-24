"""
Wire-format tests for the saved-map and SLAM-reset messages in
web_dashboard/protocol.py.

Same style as test_protocol.py: no ROS, no Tornado, no network, no browser.
Fake runs are SimpleNamespace-with-as_dict, carrying only what the
constructors read.

The one non-obvious assertion in here is that every message survives
`json.dumps` with no `default=`. dashboard_node.py serialises these on a
timer, on the car; a pathlib.Path or a set that slipped into as_dict()
would raise there and nowhere else, and a shape-only test would never see
it.
"""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from web_dashboard import protocol  # noqa: E402


class _Run:
    """Just enough of a mapstore.SavedRun for the constructors."""

    def __init__(self, run_id='20260727-200103', **extra):
        self.payload = {'id': run_id, 'bytes': 1024, 'deletable': True}
        self.payload.update(extra)

    def as_dict(self):
        return dict(self.payload)


def _roundtrip(message):
    """Serialise the way dashboard_node does, and read it back."""
    return json.loads(json.dumps(message))


# ---------------------------------------------------------------------------
# saved_maps_message
# ---------------------------------------------------------------------------

def test_the_saved_map_list_names_its_type_and_carries_a_stamp():
    message = protocol.saved_maps_message([], True, True, True)
    assert message['type'] == 'saved_maps'
    assert isinstance(message['stamp'], float)


def test_every_run_is_expanded_through_as_dict():
    message = protocol.saved_maps_message(
        [_Run('a'), _Run('b')], True, True, True)
    assert [r['id'] for r in message['runs']] == ['a', 'b']


def test_a_run_that_is_already_a_dict_is_passed_through():
    """process_state_message accepts both; matching it keeps the node free
    to hand over either without a conversion at the call site."""
    message = protocol.saved_maps_message([{'id': 'plain'}], True, True, True)
    assert message['runs'] == [{'id': 'plain'}]


def test_an_empty_list_is_a_valid_panel_not_a_missing_one():
    """A4, empty input. The browser must be able to tell "no maps" from
    "no message yet"."""
    message = protocol.saved_maps_message([], True, True, True)
    assert message['runs'] == []
    assert message['enabled'] is True


def test_the_roots_travel_with_the_list():
    """"no maps" and "no directories configured" look identical without
    this, and they call for completely different fixes."""
    message = protocol.saved_maps_message(
        [], True, True, True, roots=['/home/x/.ros/racerbot_auto'])
    assert message['roots'] == ['/home/x/.ros/racerbot_auto']


def test_roots_default_to_empty_rather_than_missing():
    assert protocol.saved_maps_message([], False, False, False)['roots'] == []


def test_each_capability_flag_is_carried_independently():
    """Deleting and resetting are separate parameters and separate risks;
    one being off must not imply the other."""
    message = protocol.saved_maps_message([], True, False, True)
    assert message['can_delete'] is False
    assert message['can_reset_slam'] is True


def test_the_flags_are_real_booleans_not_whatever_was_passed():
    message = protocol.saved_maps_message([], 1, 'yes', 0)
    assert message['enabled'] is True
    assert message['can_delete'] is True
    assert message['can_reset_slam'] is False


def test_a_disabled_panel_still_sends_a_snapshot():
    """The browser hides the section on `enabled: false`. Sending nothing
    would leave an empty list on screen, which reads as "no maps"."""
    message = protocol.saved_maps_message([], False, False, False)
    assert message['enabled'] is False
    assert _roundtrip(message)['type'] == 'saved_maps'


def test_the_saved_map_list_survives_json_dumps():
    message = protocol.saved_maps_message([_Run()], True, True, True)
    assert _roundtrip(message)['runs'][0]['id'] == '20260727-200103'


def test_a_run_carrying_an_unserialisable_value_fails_loudly_here():
    """Guard on the guard: if this ever stops raising, the serialisation
    test above has stopped proving anything."""
    class _Bad:
        def as_dict(self):
            return {'id': 'x', 'path': {1, 2}}   # a set is not JSON

    with pytest.raises(TypeError, match='not JSON serializable'):
        json.dumps(protocol.saved_maps_message([_Bad()], True, True, True))


# ---------------------------------------------------------------------------
# map_delete_result_message
# ---------------------------------------------------------------------------

def test_a_successful_delete_reports_what_it_freed():
    message = protocol.map_delete_result_message('20260727-200103', True,
                                                 'deleted', 37919937)
    assert message['type'] == 'map_delete_result'
    assert message['ok'] is True
    assert message['freed_bytes'] == 37919937  # bytes


def test_a_refused_delete_frees_nothing_by_default():
    message = protocol.map_delete_result_message('x', False, 'refused')
    assert message['freed_bytes'] == 0


def test_a_refusal_always_carries_a_reason():
    """A refusal with no reason is a panel that says nothing and invites a
    second press."""
    message = protocol.map_delete_result_message('x', False, 'outside the roots')
    assert message['ok'] is False
    assert message['detail']


def test_the_run_id_is_echoed_so_a_tab_can_match_the_reply():
    message = protocol.map_delete_result_message('20260101-000000', False, 'no')
    assert message['id'] == '20260101-000000'


def test_a_missing_id_becomes_a_string_not_a_none():
    """The browser keys rows by this; None would stringify as "None" in one
    place and stay null in another."""
    assert protocol.map_delete_result_message(None, False, 'x')['id'] == 'None'


def test_the_delete_result_survives_json_dumps():
    assert _roundtrip(
        protocol.map_delete_result_message('a', True, 'deleted', 10))['ok'] is True


# ---------------------------------------------------------------------------
# slam_reset_result_message
# ---------------------------------------------------------------------------

def test_a_reset_in_flight_is_marked_not_done():
    """slam_toolbox blocks its own executor while this runs, so the pose
    freezes. Saying "resetting..." is what stops that reading as a crash."""
    message = protocol.slam_reset_result_message(False, 'resetting...', done=False)
    assert message['type'] == 'slam_reset_result'
    assert message['done'] is False


def test_a_reset_defaults_to_done():
    assert protocol.slam_reset_result_message(True, 'reset')['done'] is True


def test_a_refused_reset_carries_the_reason_it_was_refused():
    message = protocol.slam_reset_result_message(
        False, 'refused -- pure_pursuit_node (pid 42) is running')
    assert message['ok'] is False
    assert 'pure_pursuit_node' in message['detail']


def test_the_reset_result_survives_json_dumps():
    assert _roundtrip(protocol.slam_reset_result_message(True, 'ok'))['ok'] is True


# ---------------------------------------------------------------------------
# map_cleared_message
# ---------------------------------------------------------------------------

def test_clearing_reports_whether_the_car_still_has_a_map():
    """Both answers are useful: back at once means the car is still
    publishing it, so the staleness was never the browser's."""
    assert protocol.map_cleared_message(True)['has_map'] is True
    assert protocol.map_cleared_message(False)['has_map'] is False


def test_the_cleared_message_names_its_type_and_survives_json_dumps():
    message = protocol.map_cleared_message(False)
    assert message['type'] == 'map_cleared'
    assert _roundtrip(message)['has_map'] is False


# ---------------------------------------------------------------------------
# Shared shape
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('message', [
    protocol.saved_maps_message([], True, True, True),
    protocol.map_delete_result_message('a', True, 'deleted'),
    protocol.slam_reset_result_message(True, 'ok'),
    protocol.map_cleared_message(True),
])
def test_every_new_message_has_a_type_and_a_stamp(message):
    """Invariant across the whole protocol: the browser dispatches on
    `type` and ages every panel off `stamp`."""
    assert isinstance(message['type'], str) and message['type']
    assert isinstance(message['stamp'], float)
