"""
batching.py

Collapses the dashboard's many small telemetry messages into one
WebSocket frame per tick.

Measured on this car, the node was emitting about 155 frames a second:
`/slam_pose` at 40Hz, `/ackermann_cmd` at 44, `/odom` at 32,
`/drive_intent` at 18, plus the stopwatch and scan. Each one was its own
JSON encode, its own cross-thread wake of Tornado's IOLoop, and its own
TCP segment. The interesting part is that a WebSocket frame costs about
158us of server CPU almost regardless of how big it is -- the framing
dominates, not the payload -- so 155 tiny frames cost roughly eight times
what the same data costs as 20 batched ones, and carry about 9 kB/s of
pure TCP/IP header overhead with them.

Nothing here is lost by batching, because every one of these messages is
a *display* value: at 20Hz the browser still updates faster than a screen
refreshes and far faster than a person reads. Latest-wins is therefore
the right rule -- with exactly one exception.

`/drive_intent` is not purely a display value. The browser builds its
rolling decision log out of intent *state transitions*
(dashboard.js:applyIntent), so collapsing a 30ms `stop` blip down to
"whatever the state was at the end of the tick" would silently erase an
entry from a safety-adjacent diagnostic -- precisely the kind of brief
event somebody scrolls that log looking for. So intents are queued as a
short ordered list: a new state is appended, a repeat of the state
already at the tail replaces it. Every transition survives; the steady
18Hz stream of "still racing" does not.

No ROS, no Tornado -- see test/test_batching.py.
"""


#: Safety valve. At a 20Hz flush against an 18Hz publisher this holds one
#: entry, so reaching this cap means flushing has stopped happening at all
#: (no clients, a wedged IOLoop) and the oldest transitions are the ones
#: worth dropping -- the browser's own log only keeps the newest 20 anyway.
MAX_QUEUED_INTENTS = 32

#: The message type whose transitions must survive coalescing.
ORDERED_TYPE = 'intent'


class TelemetryBatcher:
    """Accumulate compact telemetry; hand it over one frame at a time.

    Not thread-safe by itself: in the dashboard every `add()` and the
    `flush()` timer all run on the rclpy executor thread.
    """

    def __init__(self, max_queued_intents=MAX_QUEUED_INTENTS):
        self.max_queued_intents = int(max_queued_intents)
        self._latest = {}     # type -> newest message of that type
        self._intents = []    # ordered, transition-preserving
        self._dropped_intents = 0

    def __len__(self):
        return len(self._latest) + len(self._intents)

    @property
    def dropped_intents(self):
        """Intent messages discarded by the safety valve. Non-zero means
        flush() is not being called, which is worth surfacing rather than
        hiding."""
        return self._dropped_intents

    def add(self, message):
        """Queue one compact telemetry message for the next flush."""
        if not isinstance(message, dict):
            raise TypeError(f'expected a message dict, got {type(message).__name__}')
        kind = message.get('type')
        if kind == ORDERED_TYPE:
            self._add_intent(message)
        else:
            # Latest wins: a display only ever shows the newest value, so
            # holding an older one back would just delay the truth.
            self._latest[kind] = message

    def _add_intent(self, message):
        state = _intent_state(message)
        if self._intents and _intent_state(self._intents[-1]) == state:
            # Same state as the one already queued: this message is a fresh
            # sample of an ongoing situation, not a new decision. Keep the
            # newest (its speeds and path are the current ones) but do not
            # grow the list.
            self._intents[-1] = message
            return
        self._intents.append(message)
        if len(self._intents) > self.max_queued_intents:
            del self._intents[0]
            self._dropped_intents += 1

    def flush(self):
        """The batch frame to send, or None if nothing is queued.

        Clears the queue: whatever this returns is now the caller's
        responsibility to deliver.
        """
        if not self._latest and not self._intents:
            return None
        # Intents last, so that within one frame the browser applies the
        # new pose/speed before the intent drawn relative to it.
        items = list(self._latest.values()) + self._intents
        self._latest = {}
        self._intents = []
        return {'type': 'batch', 'items': items}

    def clear(self):
        """Throw the queue away -- used when the last browser disconnects,
        so a reconnecting tab is not handed telemetry from minutes ago."""
        self._latest = {}
        self._intents = []


def _intent_state(message):
    """The driving state inside an `intent` envelope, or None.

    protocol.intent_message() nests the driving node's own payload under
    'intent' so the dashboard's envelope fields cannot collide with the
    schema's; the state that matters for transitions lives in there.
    """
    payload = message.get(ORDERED_TYPE)
    if isinstance(payload, dict):
        return payload.get('state')
    return None
