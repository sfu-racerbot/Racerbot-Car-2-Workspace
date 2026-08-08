"""Turn a lap recorded from a live SLAM pose into a line the car can steer.

`auto_map_race_node` records the map-frame pose every `waypoint_spacing`
metres and hands the result to `racing_math.compute_velocity_profile`. That
skips a step, and the step matters:

**A live SLAM pose is not a trajectory.** slam_toolbox re-optimises its pose
graph continuously, and every correction moves `map->odom`, which moves the
car's map-frame pose without the car having moved. Recorded verbatim, those
corrections become *geometry*. Measured on this car's own three recorded
laps (`~/.ros/racerbot_auto`, 2026-07-27):

| | 195630 | 200103 | 202458 |
|---|---:|---:|---:|
| revolutions in the recorded "lap" | 1.98 | 1.96 | 1.98 |
| median heading change per 0.15m sample | 10.1 deg | 8.8 deg | 15.5 deg |
| 95th-percentile steering the line demands | 27.2 deg | 29.7 deg | 30.5 deg |
| waypoints demanding more steer than the rack has | 34% | 34% | 33% |
| start/finish seam heading mismatch | 34.8 deg | 38.6 deg | 110.1 deg |

The rack reaches 14.9 deg. **A third of every line this car has ever
generated was physically unfollowable**, and the seam -- the first thing
pure pursuit drives, because closure is detected exactly there -- was a
corner up to 110 degrees wide. `smooth_path(half_window=3)` on its own does
not come close to fixing that.

So this module does the missing step, out of the node so it is testable
without ROS (`python3 -m pytest src/pure_pursuit/test/`):

1. trim to the final complete revolution (every real recording is two --
   see `last_revolution`);
2. drop repeated points;
3. resample to uniform arc-length spacing, which also replaces the one long
   straight closing segment with ordinary ones;
4. low-pass the closed loop in *space*, discarding wiggles shorter than the
   car's own turning circle;
5. filter harder while the line still demands more curvature than
   `tan(delta_max)/L`; and
6. report whether it ever got there.

Step 4 is a Fourier low-pass rather than repeated moving-average smoothing,
and the difference is not cosmetic: repeated averaging of a closed curve is
curve-shortening flow, and it collapses the loop to a point. Run over these
same three recorded laps it reduced a 30m lap to a 0.0m dot in 19-25 passes
-- while cheerfully reporting zero peak curvature, because a point is very
feasible. A spatial low-pass instead leaves the low harmonics (the track)
untouched and removes the high ones (the jitter), so filtering harder makes
the line rounder, never smaller.

The cutoff has a physical meaning rather than a tuned one: the car cannot
drive a wavelength shorter than roughly its minimum turning circle, so
anything shorter in a recorded line is measurement noise by definition.

Step 6 is the one that changes behaviour most: the caller is expected to
*refuse* an infeasible line rather than load it. Handing pure pursuit a
line it cannot steer does not degrade gracefully -- it saturates the rack,
runs wide, and latches on its own emergency stop.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from pure_pursuit import racing_math


@dataclass
class PreparedPath:
    """A cleaned closed line plus everything needed to judge it."""

    xy: np.ndarray
    raw_points: int
    trimmed_points: int
    length_m: float
    harmonics_kept: int
    shortest_wavelength_m: float
    max_deviation_m: float
    min_wall_clearance_m: float
    driven_wall_clearance_m: float
    required_wall_clearance_m: float
    configured_wall_clearance_m: float
    max_curvature: float
    curvature_limit: float
    rack_curvature_limit: float
    fraction_over_limit: float
    reject_ratio: float
    reject_fraction: float
    max_steering_rad: float
    max_steering_limit_rad: float
    seam_heading_error_rad: float
    max_segment_m: float

    @property
    def feasible(self) -> bool:
        """Every waypoint is inside the steering budget the caller asked for."""
        return self.max_curvature <= self.curvature_limit

    @property
    def fits_the_track(self) -> bool:
        """The line stays far enough from a wall to put the car on it.

        Unchecked when no map was supplied (reported as -1). This is the
        constraint the low-pass fights: filtering rounds a corner *inward*,
        toward the chord, and on a course whose corners are near the car's
        own turning circle that inward pull is straight into the wall. A
        measured run put the finished line 0.05m from a wall -- the car's
        half-width alone is 0.155m, so the body was inside it, and no speed
        setting fixes that.
        """
        if self.min_wall_clearance_m < 0.0:
            return True
        return self.min_wall_clearance_m >= self.required_wall_clearance_m

    @property
    def map_has_ghosts(self) -> bool:
        """The map says the car drove through a wall, so it has phantoms.

        Mapping with other cars on the track paints them into the grid: a
        moving opponent leaves a smear of occupied cells along a line the
        ego demonstrably drove. Measured directly -- the two-car scenario
        produced a recorded line with 0.00m of clearance by the map's
        reckoning, on a line the car had just driven twice without touching
        anything.
        """
        return (0.0 <= self.driven_wall_clearance_m
                < self.configured_wall_clearance_m)

    @property
    def acceptable(self) -> bool:
        """Drivable, even if it uses more of the rack than was budgeted.

        Pure pursuit clamps steering and corrects cross-track error, so a
        line a little past the limit at one apex understeers and recovers.
        One that is far past it does not: it saturates the rack, runs
        wide, and latches on the emergency stop.

        These are "clearly undrivable" gates, not quality gates. The number
        worth watching is `max_curvature / rack_curvature_limit`: cleanup
        takes this car's own recorded laps from 2.7-4.0x the rack down to
        1.2-1.8x, and the one still at 1.8x is the one refused here.
        """
        return (self.fits_the_track
                and self.max_curvature <= self.rack_curvature_limit * self.reject_ratio
                and self.fraction_over_limit <= self.reject_fraction)

    def describe(self) -> str:
        return (
            f'{len(self.xy)} points over {self.length_m:.1f}m '
            f'(from {self.raw_points} recorded, {self.trimmed_points} after trimming to '
            f'one lap); kept {self.harmonics_kept} harmonics '
            f'(nothing shorter than {self.shortest_wavelength_m:.2f}m), moving the line '
            f'at most {self.max_deviation_m:.2f}m off the recorded one; peak curvature '
            f'{self.max_curvature:.3f}/m of the {self.curvature_limit:.3f}/m the rack '
            f'can reach (needs {math.degrees(self.max_steering_rad):.1f}deg steering, '
            f'limit {math.degrees(self.max_steering_limit_rad):.1f}deg) on '
            f'{self.fraction_over_limit * 100.0:.1f}% of waypoints; seam heading '
            f'error {math.degrees(self.seam_heading_error_rad):.1f}deg; '
            + ('wall clearance not checked (no map)'
               if self.min_wall_clearance_m < 0.0 else
               f'closest wall {self.min_wall_clearance_m:.2f}m, needs '
               f'{self.required_wall_clearance_m:.2f}m'
               + (f' (relaxed: the driven line itself measured '
                  f'{self.driven_wall_clearance_m:.2f}m, so the map has '
                  f'phantom obstacles in it -- other cars, most likely)'
                  if self.map_has_ghosts else '')))


def curvature_limit(max_steering_angle: float, wheelbase: float) -> float:
    """Tightest curvature the rack can ask for: tan(delta_max) / L.

    Same inversion of the bicycle model as
    `raceline_optimizer.curvature_limit`, repeated here rather than imported
    because that module pulls in scipy for the offline optimiser and this one
    runs inside a node on the car.
    """
    if not (math.isfinite(max_steering_angle)
            and 0.0 < max_steering_angle < math.pi / 2.0):
        raise ValueError('max_steering_angle must be finite and in (0, pi/2)')
    if not (math.isfinite(wheelbase) and wheelbase > 0.0):
        raise ValueError('wheelbase must be finite and positive')
    return math.tan(max_steering_angle) / wheelbase


def drop_repeated_points(xy: np.ndarray, tolerance: float = 1e-6) -> np.ndarray:
    """Remove consecutive duplicates, including one wrapping the seam.

    A duplicate leaves the tangent undefined and makes the three-point
    curvature estimate divide by ~0, which is how a stationary car recorded
    twice in the same place becomes an infinitely tight corner.
    """
    points = np.asarray(xy, dtype=np.float64)
    if len(points) < 2:
        return points.copy()
    keep = [0]
    for index in range(1, len(points)):
        if np.hypot(*(points[index] - points[keep[-1]])) > tolerance:
            keep.append(index)
    kept = points[keep]
    if len(kept) > 2 and np.hypot(*(kept[-1] - kept[0])) <= tolerance:
        kept = kept[:-1]
    return kept


def resample_closed(xy: np.ndarray, spacing: float) -> np.ndarray:
    """Uniform arc-length resampling of a closed polyline.

    A plain polyline walk, not a spline: the input is a dense, noisy
    recorded lap, and a spline through noisy points overshoots between them.
    Smoothing is step 3's job, not this one's.
    """
    points = np.asarray(xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 4:
        raise ValueError('a closed path needs at least 4 xy points to resample')
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError('spacing must be finite and positive')

    loop = np.vstack([points, points[:1]])
    steps = np.hypot(np.diff(loop[:, 0]), np.diff(loop[:, 1]))
    arc = np.concatenate([[0.0], np.cumsum(steps)])
    total = float(arc[-1])
    if total <= 0.0:
        raise ValueError('recorded path has zero length')
    count = max(4, int(round(total / spacing)))
    targets = np.linspace(0.0, total, count, endpoint=False)
    return np.column_stack([np.interp(targets, arc, loop[:, 0]),
                            np.interp(targets, arc, loop[:, 1])])


def seam_heading_error(xy: np.ndarray) -> float:
    """Heading change across the start/finish join, in radians.

    Zero on a properly closed loop. This is the number that was 110 degrees
    on one of this car's recorded laps, and it is worth reporting separately
    from peak curvature because it says *where* the problem is.
    """
    points = np.asarray(xy, dtype=np.float64)
    if len(points) < 3:
        return 0.0
    incoming = math.atan2(*(points[0] - points[-1])[::-1])
    outgoing = math.atan2(*(points[1] - points[0])[::-1])
    return abs((outgoing - incoming + math.pi) % (2.0 * math.pi) - math.pi)


def lowpass_closed(xy: np.ndarray, harmonics: int) -> np.ndarray:
    """Keep the lowest `harmonics` spatial frequencies of a closed loop.

    The loop is treated as one complex periodic signal z(s) = x + iy sampled
    at uniform arc length, so a single real FFT filters both coordinates
    consistently. Harmonic k is one full wiggle every `length/k` metres;
    zeroing everything above K therefore erases exactly the features shorter
    than `length/K` and leaves the rest untouched -- including the mean, so
    the loop keeps its position, and the first harmonic, so it keeps its
    size. That is the property repeated averaging does not have.
    """
    points = np.asarray(xy, dtype=np.float64)
    count = len(points)
    if count < 4:
        return points.copy()
    harmonics = int(max(1, min(harmonics, count // 2)))
    spectrum = np.fft.rfft(points, axis=0)
    spectrum[harmonics + 1:] = 0.0
    return np.fft.irfft(spectrum, n=count, axis=0)


def signed_turn(xy: np.ndarray, smoothing_half_window: int = 0) -> np.ndarray:
    """Cumulative signed heading change along an open path, in radians.

    By the turning-tangent theorem a simple closed circuit accumulates
    exactly +/-2*pi of turning however big or small it is, which makes this
    the one closure test that does not need to be told how long the course
    is. Localisation jitter adds turning in both directions and largely
    cancels in the signed sum, unlike the path *length*, which jitter only
    ever inflates.
    """
    points = np.asarray(xy, dtype=np.float64)
    if len(points) < 3:
        return np.zeros(max(0, len(points) - 1))
    if smoothing_half_window > 0:
        # Per-sample headings from a jittering pose are close to noise --
        # measured median 8.8-15.5 degrees of change between 0.15m samples on
        # this car. The *signed* sum still cancels most of it, but not enough
        # for the result to be monotonic, and trimming searches it. Smooth
        # first; this copy is only ever used to count revolutions, never as
        # geometry.
        points = racing_math.smooth_path(points, smoothing_half_window, closed=False)
    deltas = np.diff(points, axis=0)
    headings = np.arctan2(deltas[:, 1], deltas[:, 0])
    steps = (np.diff(headings) + math.pi) % (2.0 * math.pi) - math.pi
    return np.concatenate([[0.0], np.cumsum(steps)])


def last_revolution(xy: np.ndarray, search_fraction: float = 0.7,
                    min_discarded_fraction: float = 0.2) -> np.ndarray:
    """Trim a recorded path back to its final complete lap.

    `auto_map_race_node` gates closure on `minimum_lap_distance`, and when
    that is longer than the course -- 20m against this car's roughly 15m
    room -- the gate cannot open until the car has been round twice. All
    three laps this car has ever recorded are two revolutions (1.96-1.98
    turns about their own centroid). Two overlapping passes are not a
    closed racing line: the loop doubles back on itself, so
    `find_nearest_index` can jump between passes and the three-point
    curvature estimate is meaningless.

    The cut is the point in the first `search_fraction` of the path closest
    to the final point -- for a two-lap recording that is where the previous
    lap ended, to within a few centimetres; for a one-lap recording it is
    the start, and nothing is discarded. No thresholds to tune and no
    heading integration: an earlier attempt integrated turn angle and had to
    be abandoned, because on a pose jittering 8-15 degrees per 0.15m sample
    the cumulative turn is not monotonic, so the search landed 20-30 degrees
    either side of a full lap and left a hairpin in the seam.

    Measured on the three recorded laps: cut gaps of 0.04-0.12m and kept
    lengths of 15.2-15.6m out of ~30m recorded.

    Trimming is independent of whatever closure test the caller used, so a
    profile is a single loop even if closure fired late.
    """
    points = np.asarray(xy, dtype=np.float64)
    if len(points) < 8:
        return points.copy()

    distance = np.hypot(points[:, 0] - points[-1, 0], points[:, 1] - points[-1, 1])
    horizon = max(4, int(len(points) * float(np.clip(search_fraction, 0.2, 0.9))))
    cut = int(np.argmin(distance[:horizon]))
    if cut == 0:
        return points.copy()

    total = float(np.hypot(*np.diff(points, axis=0).T).sum())
    kept = float(np.hypot(*np.diff(points[cut:], axis=0).T).sum())
    if total <= 0.0 or (total - kept) < min_discarded_fraction * total:
        # The nearest approach was near the start after all: one lap, or
        # near enough that trimming would only shave noise off the front.
        return points.copy()
    return points[cut:].copy()


def prepare(points, *, spacing: float, max_steering_angle: float,
            wheelbase: float, min_feature_wavelength: float = 1.5,
            curvature_margin: float = 1.0, max_deviation: float = 0.35,
            reject_ratio: float = 1.5, reject_fraction: float = 0.25,
            clearance_fn=None, required_clearance: float = 0.0) -> PreparedPath:
    """Clean a recorded lap into a closed line the rack can actually follow.

    `min_feature_wavelength` sets the finest detail considered before
    filtering harder: 1.5m is a little over the 1.22m minimum turning circle
    this car's steering limit implies, so anything tighter is not something
    the car drove -- it is something localisation did.

    `clearance_fn(xs, ys) -> metres to the nearest obstacle` and
    `required_clearance` bring the map into the choice, and they are what
    make the search a trade rather than a slide: filtering harder always
    improves curvature, and always rounds corners *inward*, toward the
    wall. Without the map the strongest filter looks best on every reported
    number while quietly putting the line 0.05m from a wall -- measured, on
    a course whose corners sit near the car's own turning circle. Omit them
    and clearance is simply not checked, and says so.

    `curvature_margin` scales the rack limit to give the target. 1.0 means
    "as tight as the rack physically reaches"; below 1.0 reserves steering
    for pure pursuit's cross-track correction, at the cost of refusing
    courses whose corners are near the car's own turning circle. This car's
    room is one of those -- its ~1.4m corner radius passes at 1.0 and fails
    at 0.8 -- so the default does not reserve any.

    `max_deviation` bounds how far the cleanup may move the line off the one
    the car actually drove. Filtering harder always looks better by
    curvature and eventually stops describing the same track: the strongest
    filter here shortens this car's 15.2m lap to 12.8m and moves it 0.4m
    sideways, which in a 1.4m corridor is a wall.

    The result is graded rather than pass/fail -- see `PreparedPath.feasible`
    and `.acceptable`. `reject_ratio` bounds how far past the rack limit the
    worst single point may be; `reject_run_m` bounds how far the car may be
    asked to over-steer without a break.
    """
    raw = np.asarray(points, dtype=np.float64)
    if raw.ndim != 2 or raw.shape[1] != 2:
        raise ValueError('recorded points must be an (n, 2) array')
    raw_count = len(raw)

    trimmed = last_revolution(raw)
    cleaned = drop_repeated_points(trimmed)
    if len(cleaned) < 4:
        raise ValueError(
            f'only {len(cleaned)} distinct waypoint(s) were recorded; '
            'a closed racing line needs at least 4')
    if not (math.isfinite(min_feature_wavelength) and min_feature_wavelength > 0.0):
        raise ValueError('min_feature_wavelength must be finite and positive')

    limit = curvature_limit(max_steering_angle, wheelbase)
    target = limit * float(np.clip(curvature_margin, 0.05, 1.0))

    uniform = resample_closed(cleaned, spacing)
    # What the car actually drove is the proof that a line fits: the ego
    # completed this lap. So the requirement is capped by the driven line's
    # own clearance. A map with other cars smeared into it reports the
    # driven line as passing through a wall, and refusing on that would be
    # refusing to believe the lap that just happened -- while the absolute
    # requirement still catches the real failure, which is the *cleanup*
    # moving the line closer to a wall than driving ever took it.
    driven_clearance = (-1.0 if clearance_fn is None
                        else float(np.min(clearance_fn(uniform[:, 0], uniform[:, 1]))))
    configured_clearance = required_clearance
    if driven_clearance >= 0.0:
        required_clearance = min(required_clearance, driven_clearance)
    loop_length = float(racing_math.compute_segment_lengths(uniform, closed=True).sum())
    highest = max(1, int(loop_length / min_feature_wavelength))

    # Every cutoff is evaluated rather than stopping at the first that
    # passes, because peak curvature is not monotonic in the cutoff: with
    # only one harmonic left the loop is an ellipse, and an ellipse through
    # a room this shape is *tighter* at its ends than the rounded rectangle
    # the car actually drove.
    best = None
    for harmonics in range(highest, 0, -1):
        candidate = resample_closed(lowpass_closed(uniform, harmonics), spacing)
        curvature = racing_math.estimate_path_curvature(candidate, closed=True)
        peak = float(curvature.max())
        deviation = _max_deviation(uniform, candidate)
        clearance = (-1.0 if clearance_fn is None
                     else float(np.min(clearance_fn(candidate[:, 0], candidate[:, 1]))))
        if deviation > max_deviation and best is not None:
            break            # filtering harder only moves it further off
        fits = clearance < 0.0 or clearance >= required_clearance
        # Ranked, not first-past-the-post: peak curvature is not monotonic
        # in the cutoff, and clearance moves the opposite way to curvature,
        # so the best answer is not necessarily the first acceptable one.
        # Staying inside the track outranks everything -- a line through a
        # wall is not a slower racing line, it is not a racing line.
        score = (fits, peak <= target,
                 harmonics if peak <= target else -peak)
        if best is None or score > best[0]:
            best = (score, harmonics, candidate, curvature, peak, deviation,
                    clearance)
        if fits and peak <= target:
            break            # highest cutoff that fits: keep the detail

    _score, harmonics, line, curvature, peak, deviation, clearance = best
    segments = racing_math.compute_segment_lengths(line, closed=True)
    length = float(segments.sum())
    over_limit = float(np.mean(curvature > limit))
    return PreparedPath(
        xy=line,
        raw_points=raw_count,
        trimmed_points=len(trimmed),
        length_m=length,
        harmonics_kept=harmonics,
        shortest_wavelength_m=length / max(1, harmonics),
        max_deviation_m=deviation,
        min_wall_clearance_m=clearance,
        driven_wall_clearance_m=driven_clearance,
        required_wall_clearance_m=required_clearance,
        configured_wall_clearance_m=configured_clearance,
        max_curvature=peak,
        curvature_limit=target,
        rack_curvature_limit=limit,
        fraction_over_limit=over_limit,
        reject_ratio=reject_ratio,
        reject_fraction=reject_fraction,
        max_steering_rad=math.atan(peak * wheelbase),
        max_steering_limit_rad=math.atan(target * wheelbase),
        seam_heading_error_rad=seam_heading_error(line),
        max_segment_m=float(segments.max()),
    )


def _max_deviation(reference: np.ndarray, line: np.ndarray) -> float:
    """Largest distance from any filtered point to the recorded polyline."""
    loop = np.vstack([reference, reference[:1]])
    starts = loop[:-1]
    ends = loop[1:]
    deltas = ends - starts
    lengths_sq = np.maximum((deltas ** 2).sum(axis=1), 1e-12)
    worst = 0.0
    for point in line:
        t = np.clip(((point - starts) * deltas).sum(axis=1) / lengths_sq, 0.0, 1.0)
        projections = starts + t[:, None] * deltas
        worst = max(worst, float(np.hypot(*(point - projections).T).min()))
    return worst
