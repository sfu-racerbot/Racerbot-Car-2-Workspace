"""
throttle.py

Rate limiting and failure containment for intent publishing.

Both of these exist for the same reason: `/drive_intent` is diagnostics
bolted onto nodes that can move a physical car, and diagnostics must
never be able to slow that car's control loop down or take it off the
air. The rules this module enforces:

  * intent publishes at its own modest rate, decoupled from the control
    rate (gap_follow runs at scan rate, pure_pursuit at control_rate_hz --
    both faster than any browser needs to redraw), and
  * a bug in intent generation disables intent generation, not the node.

Pure Python, no rclpy: both classes take an explicit `now` in seconds so
they work under sim time, wall time, or a test's fake clock.
"""


class IntentThrottle:
    """Decides when to publish an intent message, and when to attach a
    fresh `reason` string to it.

    Two separate rates on purpose. The geometry (path, speeds, factors)
    is cheap and wants to be smooth, so it goes out at `rate_hz`. The
    reason string can be expensive to produce -- gap_follow's TTC stop
    reason re-runs the whole gap pipeline to report an escape route -- so
    it is attached only when the state actually changes or when
    `reason_period_sec` has elapsed, mirroring exactly what the terminal
    decision log already does. A browser holds the last reason it saw for
    the current state, so nothing looks stale.
    """

    def __init__(self, rate_hz: float, reason_period_sec: float):
        self.rate_hz = float(rate_hz)
        self.reason_period_sec = float(reason_period_sec)
        self._last_publish = None
        self._last_reason_time = None
        self._last_state = None

    @property
    def min_period(self) -> float:
        # A non-positive rate means "every tick", not "never" -- the way
        # to turn intent off is the publish_intent parameter, and a rate
        # of 0 silently publishing nothing would be a trap.
        return 0.0 if self.rate_hz <= 0.0 else 1.0 / self.rate_hz

    def should_publish(self, now: float) -> bool:
        """True at most `rate_hz` times per second. Records the decision."""
        if self._last_publish is not None and now - self._last_publish < self.min_period:
            return False
        self._last_publish = now
        return True

    def wants_reason(self, now: float, state: str) -> bool:
        """True on every state transition, plus every `reason_period_sec`."""
        changed = state != self._last_state
        elapsed = (
            self.reason_period_sec > 0.0
            and (self._last_reason_time is None
                 or now - self._last_reason_time >= self.reason_period_sec)
        )
        # A transition must always carry its reason: the transition *is*
        # the diagnostic event, and it is the one moment where "why?" is
        # being asked out loud.
        if not (changed or elapsed):
            return False
        self._last_state = state
        self._last_reason_time = now
        return True

    def reset(self):
        """Forget all history, so the next call publishes and explains.

        Used when a browser connects: whoever just opened the page has no
        context at all, and making them wait up to a second for the first
        reason string is the wrong default.
        """
        self._last_publish = None
        self._last_reason_time = None
        self._last_state = None


class FailureLatch:
    """Disable a non-essential subsystem after it fails repeatedly.

    The alternative -- letting an exception in intent generation escape
    into `scan_callback` or `control_loop` -- would take down a node that
    is holding a moving car's steering and throttle, to protect a
    drawing. One transient failure is tolerated and logged; a run of them
    means the intent code is broken for this configuration and should
    stop trying, loudly, once.
    """

    def __init__(self, max_failures: int = 5):
        if max_failures < 1:
            raise ValueError(f'max_failures must be at least 1, got {max_failures!r}')
        self.max_failures = int(max_failures)
        self.consecutive_failures = 0
        self.disabled = False

    def record_failure(self) -> bool:
        """Count one failure. Returns True if this one tripped the latch."""
        if self.disabled:
            return False
        self.consecutive_failures += 1
        if self.consecutive_failures >= self.max_failures:
            self.disabled = True
            return True
        return False

    def record_success(self):
        """A working call clears the count -- the latch is for sustained
        breakage, not for one bad scan in a thousand."""
        self.consecutive_failures = 0
