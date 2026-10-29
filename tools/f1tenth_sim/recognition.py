"""Counting rules for the opponent-recognition measures.

eval_gap.py and eval_gap_multi.py record per-tick facts (which tracks are
moving, whether the other car is visible, whether it is recognised) and hand
them here to become the counts and delays their reports print. Pure Python:
no gym, no ROS, so the rules are tested without a simulator run
(tests/test_recognition.py).
"""

from __future__ import annotations

import math
import statistics


def false_movers(moving_tracks_per_tick) -> dict:
    """One car alone on track: every moving track is a false one.

    ``moving_tracks_per_tick[i]`` is the list of track ids with ``moving`` True
    at tick i. Returns {"false_mover_ticks": ticks with any moving track,
    "false_mover_tracks": distinct track ids ever moving}.
    """
    ticks = 0
    ids = set()
    for moving in moving_tracks_per_tick:
        if moving:
            ticks += 1
            ids.update(moving)
    return {"false_mover_ticks": ticks, "false_mover_tracks": len(ids)}


def moving_track_ids(tracks) -> list:
    """The ids of the tracks whose ``moving`` flag is set, in their order.

    ``tracks`` are objects with ``track_id`` and ``moving`` attributes, as
    gap_follow's opponent_tracking.Track has.
    """
    return [track.track_id for track in tracks if track.moving]


def recognition_delays(leader_visible, recognised, dt_s: float) -> list:
    """Seconds from each time the leader becomes visible (``leader_visible``
    goes False -> True, or is True at tick 0) to the first tick at or after it
    where ``recognised`` is True, for each visibility episode. An episode that
    ends (visible goes False) or the run ending before recognition gives
    ``math.inf`` for that episode. Unequal lengths raise ValueError.
    """
    _same_length(leader_visible, recognised)
    delays = []
    n = len(leader_visible)
    i = 0
    while i < n:
        if not leader_visible[i]:
            i += 1
            continue
        start = i
        while i < n and leader_visible[i]:
            i += 1
        # The episode is the ticks [start, i). Recognition outside it does not count.
        hit = next((k for k in range(start, i) if recognised[k]), None)
        delays.append(math.inf if hit is None else (hit - start) * dt_s)
    return delays


def recognition_counts(leader_visible, recognised) -> dict:
    """How many ticks the leader was visible, and how many of those were recognised."""
    _same_length(leader_visible, recognised)
    visible_ticks = 0
    recognised_ticks = 0
    for visible, seen in zip(leader_visible, recognised):
        if visible:
            visible_ticks += 1
            if seen:
                recognised_ticks += 1
    return {"visible_ticks": visible_ticks, "recognised_ticks": recognised_ticks}


def recognised_fraction(visible_ticks: int, recognised_ticks: int):
    """Recognised ticks over visible ticks, or None when the leader was never visible."""
    if visible_ticks == 0:
        return None
    return recognised_ticks / visible_ticks


def median_delay(delays):
    """The median of the episode delays, where inf sorts above any finite delay.

    None when there are no episodes.
    """
    if not delays:
        return None
    return statistics.median(delays)


def _same_length(leader_visible, recognised) -> None:
    if len(leader_visible) != len(recognised):
        raise ValueError(
            "visible and recognised differ in length: "
            f"{len(leader_visible)} and {len(recognised)}")
