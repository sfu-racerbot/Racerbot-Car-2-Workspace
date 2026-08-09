"""
Unit tests for web_dashboard.batching -- coalescing telemetry frames.

No ROS, no Tornado, no network. Run with:

    python3 -m pytest src/web_dashboard/test/test_batching.py -v

The rule under test is "latest wins, except that no /drive_intent state
transition may ever be dropped". The browser builds its decision log out
of those transitions, so a collapsed one is a missing line in a
safety-adjacent diagnostic -- and it is exactly the brief, surprising
states (a 30ms emergency stop) that somebody scrolls that log looking
for.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from web_dashboard.batching import TelemetryBatcher  # noqa: E402


def _pose(x):
    return {'type': 'pose', 'x': x, 'y': 0.0, 'yaw': 0.0}


def _intent(state, speed=1.0):
    # Shaped like protocol.intent_message(): the driving node's payload is
    # nested under 'intent' so it cannot collide with envelope fields.
    return {'type': 'intent', 'intent': {'state': state, 'commanded_speed': speed}}


def _items_of(batch, kind):
    return [item for item in batch['items'] if item['type'] == kind]


# --------------------------------------------------------------------------
# Latest-wins
# --------------------------------------------------------------------------

def test_nothing_queued_flushes_to_nothing():
    assert TelemetryBatcher().flush() is None


def test_repeated_values_of_one_type_collapse_to_the_newest():
    batcher = TelemetryBatcher()
    for x in range(10):
        batcher.add(_pose(float(x)))
    batch = batcher.flush()
    poses = _items_of(batch, 'pose')
    assert len(poses) == 1
    # The newest, not the oldest: a display shows the current value, and
    # holding an older one back would only delay the truth.
    assert poses[0]['x'] == 9.0


def test_different_types_all_survive_one_flush():
    batcher = TelemetryBatcher()
    batcher.add({'type': 'pose', 'x': 1.0})
    batcher.add({'type': 'drive', 'speed': 2.0})
    batcher.add({'type': 'speed', 'speed': 1.5})
    batcher.add({'type': 'stats', 'cpu_percent': 30.0})
    batch = batcher.flush()
    assert {item['type'] for item in batch['items']} == {
        'pose', 'drive', 'speed', 'stats'}


def test_the_batch_is_a_single_frame_with_a_batch_type():
    batcher = TelemetryBatcher()
    batcher.add(_pose(1.0))
    batch = batcher.flush()
    assert batch['type'] == 'batch'
    assert isinstance(batch['items'], list)


def test_flushing_empties_the_queue():
    batcher = TelemetryBatcher()
    batcher.add(_pose(1.0))
    assert batcher.flush() is not None
    assert batcher.flush() is None


def test_clear_discards_without_sending():
    batcher = TelemetryBatcher()
    batcher.add(_pose(1.0))
    batcher.clear()
    assert batcher.flush() is None


def test_a_non_dict_message_is_rejected_rather_than_queued():
    with pytest.raises(TypeError):
        TelemetryBatcher().add('pose')


# --------------------------------------------------------------------------
# Intent: transitions are sacred
# --------------------------------------------------------------------------

def test_every_intent_state_transition_survives_coalescing():
    batcher = TelemetryBatcher()
    for state in ['racing', 'racing', 'racing', 'corner', 'corner',
                  'stop', 'racing', 'racing']:
        batcher.add(_intent(state))
    states = [item['intent']['state'] for item in _items_of(batcher.flush(), 'intent')]
    assert states == ['racing', 'corner', 'stop', 'racing']


def test_a_single_tick_blip_is_not_swallowed():
    # The case this rule exists for: one emergency stop between two
    # ordinary frames, inside a single 50ms batch window.
    batcher = TelemetryBatcher()
    batcher.add(_intent('racing'))
    batcher.add(_intent('emergency_stop'))
    batcher.add(_intent('racing'))
    states = [item['intent']['state'] for item in _items_of(batcher.flush(), 'intent')]
    assert 'emergency_stop' in states
    assert states == ['racing', 'emergency_stop', 'racing']


def test_repeats_of_one_state_keep_the_newest_sample():
    batcher = TelemetryBatcher()
    batcher.add(_intent('racing', speed=1.0))
    batcher.add(_intent('racing', speed=2.0))
    batcher.add(_intent('racing', speed=3.0))
    intents = _items_of(batcher.flush(), 'intent')
    assert len(intents) == 1
    # Same decision, fresher numbers -- the speeds and path shown must be
    # current even though no transition happened.
    assert intents[0]['intent']['commanded_speed'] == 3.0


def test_transitions_are_delivered_in_the_order_they_happened():
    batcher = TelemetryBatcher()
    for state in ['a', 'b', 'c', 'd']:
        batcher.add(_intent(state))
    states = [item['intent']['state'] for item in _items_of(batcher.flush(), 'intent')]
    assert states == ['a', 'b', 'c', 'd']


def test_a_state_repeated_after_flushing_is_still_sent():
    # Across a flush boundary the browser needs the current sample even if
    # the state did not change -- its speeds and path are what move.
    batcher = TelemetryBatcher()
    batcher.add(_intent('racing'))
    batcher.flush()
    batcher.add(_intent('racing'))
    assert len(_items_of(batcher.flush(), 'intent')) == 1


def test_intents_are_applied_after_the_pose_they_are_drawn_against():
    batcher = TelemetryBatcher()
    batcher.add(_intent('racing'))
    batcher.add(_pose(5.0))
    types = [item['type'] for item in batcher.flush()['items']]
    # The intent arrow is drawn relative to the car's pose, so the pose in
    # the same frame should land first.
    assert types.index('pose') < types.index('intent')


def test_an_intent_without_a_recognisable_state_still_batches():
    batcher = TelemetryBatcher()
    batcher.add({'type': 'intent', 'intent': {}})
    batcher.add({'type': 'intent'})
    assert batcher.flush() is not None


# --------------------------------------------------------------------------
# Safety valve
# --------------------------------------------------------------------------

def test_unflushed_intents_are_capped_rather_than_growing_without_bound():
    batcher = TelemetryBatcher(max_queued_intents=4)
    for i in range(20):
        batcher.add(_intent(f'state_{i}'))
    intents = _items_of(batcher.flush(), 'intent')
    assert len(intents) == 4
    # Newest kept: if flushing has stopped, the recent past is what a
    # reconnecting browser can still act on.
    assert [item['intent']['state'] for item in intents] == [
        'state_16', 'state_17', 'state_18', 'state_19']


def test_dropping_is_counted_so_it_can_be_noticed_instead_of_hidden():
    batcher = TelemetryBatcher(max_queued_intents=2)
    assert batcher.dropped_intents == 0
    for i in range(5):
        batcher.add(_intent(f'state_{i}'))
    assert batcher.dropped_intents == 3


def test_the_cap_is_never_reached_at_realistic_rates():
    # 18Hz of intent against a 20Hz flush: one entry per batch, and even a
    # burst of transitions stays far under the valve.
    batcher = TelemetryBatcher()
    for _ in range(20):
        batcher.add(_intent('racing'))
        batcher.flush()
    assert batcher.dropped_intents == 0


def test_len_reports_everything_waiting():
    batcher = TelemetryBatcher()
    batcher.add(_pose(1.0))
    batcher.add(_intent('racing'))
    batcher.add(_intent('stop'))
    assert len(batcher) == 3
