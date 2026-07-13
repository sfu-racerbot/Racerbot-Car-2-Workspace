"""
gap_logic.py

All of gap_follow's scan-processing math, importable and unit-testable
without rclpy -- the same split pure_pursuit uses for racing_math.py
(see docs/writing-your-own-node.md). gap_follow_node.py owns the ROS
plumbing (parameters, topics, the deadman gate) and composes the
functions here; this file owns everything that turns a range array into
a steering target and can therefore be tested with plain numpy arrays:

    python3 -m pytest src/gap_follow/test/ -v

The processing pipeline, in the order the node applies it:

  sanitize_ranges     -> which beams are trustworthy, and a gap-safe copy
  closest_valid       -> emergency-stop / bubble anchor (valid beams only)
  disparity_extend    -> widen every obstacle *edge* by half a car width
  safety_bubble       -> zero out a car-width bubble around the closest hit
  find_best_gap       -> the best (widest*deepest, car-sized) opening
"""

import math

import numpy as np


def sanitize_ranges(ranges, max_range: float, range_min: float = 0.0):
    """Split a raw scan into a gap-safe range array plus a validity mask.

    The two output semantics exist because an *invalid* beam (NaN, or a
    sub-range_min reading -- the sensor's own "no valid return" encoding)
    means two different things to the two consumers:

      - For the emergency stop / closest-obstacle check it is *unknown*,
        not an obstacle at 0.0m: counting it would slam the brakes on
        every scan dropout (the phantom-obstacle bug this replaces).
        The `valid` mask excludes those beams from that check.
      - For gap selection it must stay *non-free* (kept at 0.0 in
        `clean`): steering into a blind spot because it "looked open"
        is worse than stopping.

    +inf is different from NaN: it is a real measurement ("nothing within
    the sensor's reach"), so it becomes max_range -- genuinely free space.

    Returns (clean, valid): `clean` is float64, invalid beams 0.0, clipped
    to [0, max_range]; `valid` is a boolean mask of trustworthy beams.
    """
    raw = np.asarray(ranges, dtype=np.float64)
    valid = np.isfinite(raw) & (raw >= range_min) & (raw > 0.0)
    clean = np.where(valid, raw, 0.0)
    clean[np.isposinf(raw)] = max_range
    return np.clip(clean, 0.0, max_range), valid


def closest_valid(clean: np.ndarray, valid: np.ndarray):
    """Index and distance of the closest *trustworthy* reading, or
    (None, inf) if nothing in the window is valid. This is what the
    emergency stop and the safety bubble anchor on -- invalid beams
    never trigger a stop, and never get a bubble carved around them.
    """
    if clean.size == 0 or not np.any(valid):
        return None, math.inf
    masked = np.where(valid, clean, np.inf)
    idx = int(np.argmin(masked))
    return idx, float(masked[idx])


def disparity_extend(clean: np.ndarray, angle_increment: float,
                     disparity_threshold: float, extend_width_m: float) -> np.ndarray:
    """The standard follow-the-gap 'disparity extender' preprocessing:
    at every sharp jump between adjacent ranges (an obstacle *edge*),
    overwrite the far side with the near side's distance for as many
    beams as `extend_width_m` subtends at that distance.

    A raw scan reports where each beam lands, but the car is not a beam
    -- it is ~30cm wide, and steering right next to an obstacle's edge
    clips it with the side of the chassis. Extending every edge by half
    a car width (plus margin) makes the range array describe where the
    *car's center* can safely go, so anything downstream (bubble, gap
    picking) is automatically clearance-aware at every edge, not just
    around the single closest point.

    Values are only ever lowered (np.minimum), never raised, so this can
    never invent free space. Returns a new array; the input is untouched.
    """
    extended = clean.copy()
    n = len(clean)
    if n < 2 or angle_increment <= 0.0:
        return extended

    jumps = np.abs(np.diff(clean))
    for i in np.nonzero(jumps > disparity_threshold)[0]:
        near = min(clean[i], clean[i + 1])
        if near <= 0.0:
            # The near side is an invalid/contact beam -- there's no
            # meaningful distance to extend at (and atan2(w, 0) would
            # smear a half-circle). The bubble/e-stop path owns this case.
            continue
        num_beams = int(math.ceil(math.atan2(extend_width_m, near) / angle_increment))
        if clean[i] < clean[i + 1]:
            lo, hi = i + 1, min(n, i + 1 + num_beams)
        else:
            lo, hi = max(0, i + 1 - num_beams), i + 1
        extended[lo:hi] = np.minimum(extended[lo:hi], near)
    return extended


def safety_bubble(window: np.ndarray, closest_idx: int, closest_dist: float,
                  angle_increment: float, bubble_width_m: float) -> np.ndarray:
    """Zero out the beams around the closest obstacle so no chosen gap
    can graze it. The bubble's angular radius is whatever half a car
    width (plus margin) actually subtends *at the obstacle's distance* --
    atan2(width, dist) -- rather than a fixed angle: a fixed 20 degrees
    is far too little clearance at 0.3m and wastefully much at 5m.
    Returns a new array; the input is untouched.
    """
    out = window.copy()
    if out.size == 0 or angle_increment <= 0.0:
        return out
    if closest_dist <= 0.0:
        radius_idx = out.size  # contact distance: everything is too close
    else:
        radius_idx = int(math.ceil(math.atan2(bubble_width_m, closest_dist) / angle_increment))
    radius_idx = max(1, radius_idx)
    lo = max(0, closest_idx - radius_idx)
    hi = min(out.size, closest_idx + radius_idx + 1)
    out[lo:hi] = 0.0
    return out


def find_best_gap(window: np.ndarray, min_gap_distance: float,
                  angle_increment: float = 0.0, min_gap_width_m: float = 0.0):
    """Pick the best drivable opening, not just the widest one.

    A shallow dead end (e.g. a ~1m doorway alcove) can be angularly
    wider than a genuine, much deeper corridor or track opening. Scoring
    candidates by width * average_depth rather than width alone means a
    gap has to actually be open for a while, not just wide at the mouth,
    to win -- so the car stops driving into pockets it can't get back
    out of.

    If `min_gap_width_m` is set (and `angle_increment` supplied), any
    candidate whose *physical* width -- angular width times average depth
    -- is narrower than that is discarded outright: a gap the car cannot
    physically fit through must never win just because nothing better
    exists. Returns (start, end) indices into `window`, or (None, None).
    """
    free = window > min_gap_distance
    candidates = []
    run_start = None
    for i, is_free in enumerate(free):
        if is_free and run_start is None:
            run_start = i
        elif not is_free and run_start is not None:
            candidates.append((run_start, i - 1))
            run_start = None
    if run_start is not None:
        candidates.append((run_start, len(free) - 1))

    def physical_width(run):
        start, end = run
        avg_depth = float(np.mean(window[start:end + 1]))
        return (end - start + 1) * angle_increment * avg_depth

    if min_gap_width_m > 0.0 and angle_increment > 0.0:
        candidates = [run for run in candidates if physical_width(run) >= min_gap_width_m]

    if not candidates:
        return None, None

    def score(run):
        start, end = run
        segment = window[start:end + 1]
        width = end - start + 1
        avg_depth = float(np.mean(segment))
        return width * avg_depth

    best_start, best_end = max(candidates, key=score)
    return best_start, best_end
