"""Deadman-gated stopwatch logic, independent of ROS and the web server."""


class DeadmanStopwatch:
    """Accumulate time only while enabled and a live LB input is held.

    Callers supply monotonic timestamps so this stays deterministic in unit
    tests and cannot jump when the system wall clock is adjusted.
    """

    def __init__(self, joy_timeout_sec: float):
        self.joy_timeout_sec = max(0.0, float(joy_timeout_sec))
        self.enabled = False
        self.elapsed_s = 0.0
        self._last_update = None
        self._last_joy_time = None
        self._lb_held = False
        self._button_available = False

    def _advance(self, now: float) -> None:
        now = float(now)
        if self._last_update is None:
            self._last_update = now
            return

        if (
            self.enabled
            and self._lb_held
            and self._button_available
            and self._last_joy_time is not None
        ):
            # If joystick messages stop while LB was held, count only up to
            # the watchdog deadline rather than the whole time since the last
            # update. This mirrors the car's own deadman freshness rule.
            active_until = min(now, self._last_joy_time + self.joy_timeout_sec)
            active_from = max(self._last_update, self._last_joy_time)
            if active_until > active_from:
                self.elapsed_s += active_until - active_from

        self._last_update = now

    def update_joy(self, buttons, deadman_button: int, now: float) -> None:
        self._advance(now)
        self._last_joy_time = float(now)
        self._button_available = 0 <= deadman_button < len(buttons)
        self._lb_held = (
            self._button_available and bool(buttons[deadman_button])
        )

    def set_enabled(self, enabled: bool, now: float) -> None:
        self._advance(now)
        self.enabled = bool(enabled)

    def reset(self, now: float) -> None:
        self._advance(now)
        self.elapsed_s = 0.0

    def snapshot(self, now: float) -> dict:
        self._advance(now)
        joy_fresh = (
            self._last_joy_time is not None
            and float(now) - self._last_joy_time < self.joy_timeout_sec
        )
        lb_held = self._lb_held and self._button_available and joy_fresh
        return {
            'elapsed_s': self.elapsed_s,
            'enabled': self.enabled,
            'running': self.enabled and lb_held,
            'lb_held': lb_held,
            'joy_fresh': joy_fresh,
            'button_available': self._button_available,
        }
