"""
raceline_optimizer.py

The geometry half of "what is the best line around this track": an
implementation of **iterative minimum-curvature trajectory optimization**,
the method from Heilmeier et al., *Minimum curvature trajectory planning and
control for an autonomous race car* (Vehicle System Dynamics, 2019,
DOI 10.1080/00423114.2019.1631455), as implemented in TUM's
`global_racetrajectory_optimization`.

Framework-agnostic and rclpy-free, like racing_math.py:

    python3 -m pytest src/pure_pursuit/test/ -v

Why minimum curvature. The genuinely time-optimal line needs a nonlinear
optimizer over path *and* speed together. Minimum curvature is the standard
convex stand-in, and the reason it works is that cornering speed goes as
``v = sqrt(a_lat_max / kappa)`` -- so minimising curvature maximises the
speed ceiling everywhere at once. Heilmeier et al. measured it within a few
tenths of a second per lap of the true minimum-time line, and it is a convex
problem that solves in one shot instead of an intractable one that may not
solve at all. It gives up something only where the limit is engine power
rather than grip, which is not this car's problem.

The formulation. The track is a centerline with a known drivable width on
each side. Every candidate line is written as a lateral offset ``alpha``
from that centerline along its normals, one scalar per waypoint, so staying
on the track is just a box constraint ``-w_right <= alpha <= +w_left``.
Curvature is nonlinear in ``alpha``, so it is linearised about the current
iterate; the objective ``sum(kappa^2)`` is then a linear least-squares in
``alpha``, and least-squares with box constraints is exactly what
``scipy.optimize.lsq_linear`` solves. Re-linearising about the answer and
re-solving a few times (the "iterative" in iterative minimum curvature)
removes the linearisation error -- the paper's own recommendation, and the
reported error after a handful of passes is negligible.

Solving it as a bounded least-squares rather than assembling the normal
equations ``H = A^T A`` and calling a QP solver is the same problem with a
better condition number, and it needs only scipy, which is already here --
no `quadprog`/`cvxpy` dependency to install on the Jetson.

What this module does *not* do: pick speeds. The finished geometry hands off
to racing_math.compute_velocity_profile, which already implements the
cornering limit plus the forward/backward friction-ellipse passes.
"""

import math

import numpy as np
from scipy.optimize import lsq_linear

from pure_pursuit import racing_math


def resample_closed_path(xy: np.ndarray, spacing: float,
                         smooth: bool = True) -> np.ndarray:
    """Re-sample a closed path to uniform arc-length `spacing`.

    Everything downstream assumes uniform spacing: the difference operators
    below are derived for it, and the box constraints are only exact when
    each waypoint owns an equal share of the path. Recorded laps are sampled
    by distance travelled but never exactly uniformly, and each optimizer
    pass moves points sideways by different amounts, so this runs before
    every pass rather than once.

    ``smooth`` fits a periodic cubic spline through the points and samples
    *that*, instead of walking the straight-line polyline between them. It
    matters most when re-sampling to a finer spacing than the input, which is
    exactly what writing the finished raceline does. Interpolating linearly
    from 0.5m control points down to 0.15m waypoints puts a corner at every
    original point; those corners are invisible on a plot and enormous to a
    three-point curvature estimate, which read them as 0.63m-radius kinks --
    tighter than the car can steer -- and the feasibility check correctly
    refused a line that was, geometrically, perfectly smooth. The spline is
    what the polyline was always meant to approximate.

    Pass ``smooth=False`` for the plain polyline walk, which is the right
    choice when the input is already dense and may be noisy, since a spline
    through noisy points overshoots between them.
    """
    points = np.asarray(xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError('path must be an (n, 2) array of xy points')
    if len(points) < 4:
        raise ValueError('a closed path needs at least 4 points to resample')
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError('spacing must be finite and positive')

    loop = np.vstack([points, points[:1]])
    steps = np.hypot(np.diff(loop[:, 0]), np.diff(loop[:, 1]))
    arc = np.concatenate([[0.0], np.cumsum(steps)])
    total = float(arc[-1])
    if total <= 0.0:
        raise ValueError('path has zero length')
    count = max(4, int(round(total / spacing)))

    if not smooth:
        targets = np.linspace(0.0, total, count, endpoint=False)
        return np.column_stack([np.interp(targets, arc, loop[:, 0]),
                                np.interp(targets, arc, loop[:, 1])])

    from scipy.interpolate import CubicSpline
    curve = CubicSpline(arc, loop, bc_type='periodic', axis=0)
    # The spline is parameterised by chord length, which is not arc length,
    # so sample it densely, measure the arc length that actually results, and
    # re-interpolate to get points that really are evenly spaced.
    dense = curve(np.linspace(0.0, total, max(16, 10 * count) + 1))
    dense_steps = np.hypot(*np.diff(dense, axis=0).T)
    dense_arc = np.concatenate([[0.0], np.cumsum(dense_steps)])
    targets = np.linspace(0.0, float(dense_arc[-1]), count, endpoint=False)
    return np.column_stack([np.interp(targets, dense_arc, dense[:, 0]),
                            np.interp(targets, dense_arc, dense[:, 1])])


def path_frames(xy: np.ndarray):
    """Unit tangents and left-pointing unit normals at every point.

    The normal convention (+alpha moves the line to the left of the driving
    direction) matches lateral_offset_point() in racing_math and REP-103's
    "positive y is left", so a positive number means the same thing here as
    everywhere else in this package.
    """
    points = np.asarray(xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        raise ValueError('need at least 3 xy points to build a frame')

    deltas = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)
    lengths = np.hypot(deltas[:, 0], deltas[:, 1])
    if np.any(lengths <= 0.0):
        raise ValueError('duplicate consecutive points leave the tangent undefined')
    tangents = deltas / lengths[:, None]
    normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    return tangents, normals


def signed_curvature(xy: np.ndarray) -> np.ndarray:
    """Signed curvature at every point of a closed path, in 1/m.

    The Menger curvature racing_math.estimate_path_curvature already
    computes, but keeping the sign of the turn: positive is a left turn
    (counter-clockwise), matching the left-positive normal from path_frames
    and REP-103. The optimizer needs the sign, because which side of the
    track reduces curvature depends entirely on which way the corner goes.
    """
    points = np.asarray(xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        raise ValueError('need at least 3 xy points')
    previous = np.roll(points, 1, axis=0)
    following = np.roll(points, -1, axis=0)

    a = np.hypot(*(following - points).T)
    b = np.hypot(*(points - previous).T)
    c = np.hypot(*(following - previous).T)
    cross = ((points[:, 0] - previous[:, 0]) * (following[:, 1] - previous[:, 1])
             - (points[:, 1] - previous[:, 1]) * (following[:, 0] - previous[:, 0]))

    denominator = a * b * c
    curvature = np.zeros(len(points))
    usable = denominator > 1e-12
    curvature[usable] = 2.0 * cross[usable] / denominator[usable]
    return curvature


def second_difference_operator(n: int, h: float):
    """Sparse d2/ds2 matrix for a closed, uniformly spaced path."""
    from scipy import sparse
    if n < 3:
        raise ValueError('need at least 3 points')
    if not math.isfinite(h) or h <= 0.0:
        raise ValueError('spacing must be finite and positive')
    index = np.arange(n)
    rows = np.concatenate([index, index, index])
    cols = np.concatenate([index, (index + 1) % n, (index - 1) % n])
    data = np.concatenate([np.full(n, -2.0), np.ones(n), np.ones(n)]) / (h * h)
    return sparse.csr_matrix((data, (rows, cols)), shape=(n, n))


def linearized_curvature_system(reference: np.ndarray, h: float):
    """Linearise curvature in the lateral offsets: ``kappa ~= A @ alpha + c``.

    A candidate line is the reference offset sideways by ``alpha(s)`` along
    its normals. For that *parallel offset curve* the exact curvature follows
    from the Frenet relations (``T' = kappa*N``, ``N' = -kappa*T``):

        P'  = (1 - alpha*kappa) T + alpha' N
        P'' = -(2 alpha' kappa + alpha kappa') T
              + ((1 - alpha*kappa) kappa + alpha'') N

    which gives, to first order in alpha,

        kappa_P  ~=  kappa + alpha'' + alpha * kappa^2

    So ``A = d2/ds2 + diag(kappa^2)`` and ``c = kappa``. Three terms with
    three plain meanings: the curvature already there, the bending caused by
    *changing* the offset, and the fact that a fixed offset toward the inside
    of a corner tightens it.

    That last term is the one worth being careful about. An earlier version
    here linearised the general curvature quotient with its denominator
    frozen at the reference, which is algebraically tempting and wrong: it
    drops the denominator's own dependence on alpha and so gets
    ``-2*alpha*kappa^2`` where the truth is ``+alpha*kappa^2``. Inverted, not
    merely inaccurate. On a circular test track -- where the answer is
    obviously the outer wall, since that is the largest circle that fits --
    it confidently converged on the *inner* wall instead. A closed-form case
    with a known answer is what caught it, which is why one is kept in the
    tests.

    Returns ``(A, c)`` with A sparse, both in units of 1/m.
    """
    from scipy import sparse
    reference = np.asarray(reference, dtype=np.float64)
    curvature = signed_curvature(reference)
    n = len(reference)
    a_matrix = (second_difference_operator(n, h)
                + sparse.diags(curvature ** 2))
    return a_matrix.tocsr(), curvature


def solve_lateral_offsets(a_matrix: np.ndarray, offset: np.ndarray,
                          lower: np.ndarray, upper: np.ndarray,
                          smoothing_weight: float, h: float) -> np.ndarray:
    """Bounded least-squares for the offsets that minimise total curvature.

    The ``alpha''`` term already in the objective is what keeps this
    well-posed along a straight, where ``kappa`` is zero and the rest of the
    objective has no opinion about where the line sits. Zero penalty falls on
    an alpha that varies *linearly*, so across a straight the cheapest answer
    is a straight diagonal from the exit of one corner to the entry of the
    next -- which is what a racing line does anyway.

    ``smoothing_weight`` adds more of the same penalty on top. It defaults to
    off and is worth raising only to damp a line that looks restless on a
    noisy hand-recorded centerline; it buys smoothness by giving up
    curvature, so it is not free.
    """
    from scipy import sparse
    n = len(offset)
    if not math.isfinite(smoothing_weight) or smoothing_weight < 0.0:
        raise ValueError('smoothing_weight must be finite and non-negative')
    if np.any(lower > upper):
        raise ValueError('lower bounds must not exceed upper bounds')

    design = a_matrix
    target = -offset
    if smoothing_weight > 0.0:
        design = sparse.vstack([
            a_matrix,
            math.sqrt(smoothing_weight) * second_difference_operator(n, h),
        ]).tocsr()
        target = np.concatenate([target, np.zeros(n)])

    result = lsq_linear(design, target, bounds=(lower, upper),
                        method='trf', lsq_solver='lsmr', tol=1e-10)
    return np.asarray(result.x, dtype=np.float64)


def optimize_minimum_curvature(centerline: np.ndarray,
                               width_left: np.ndarray,
                               width_right: np.ndarray,
                               vehicle_half_width: float,
                               safety_margin: float,
                               spacing: float,
                               iterations: int = 8,
                               smoothing_weight: float = 0.0,
                               trust_region: float = 0.30):
    """Iterative minimum-curvature optimization over a closed track.

    Args:
        centerline: (n, 2) closed reference line, any spacing.
        width_left/width_right: drivable distance from each centerline point
            to the wall on that side, in meters.
        vehicle_half_width: half the car's (padded) width.
        safety_margin: extra clearance to hold off both walls.
        spacing: arc-length resolution to optimize at.
        iterations: re-linearisation passes.
        smoothing_weight: see solve_lateral_offsets.
        trust_region: largest lateral step one pass may take, or None for
            no cap. See the comment at the clamp below.

    Returns a dict with the optimized ``line``, the total lateral ``alpha``
    from the original centerline, the per-iteration ``curvature_history``
    (the arc-length integral of squared curvature, which is the objective
    being minimised and should settle downward), and ``clamped_fraction`` --
    how much of the track was too narrow for the car plus its margin, where
    the corridor collapsed to a single line.

    Each pass re-samples the previous solution into a fresh arc-length
    reference and solves for a *correction* from there, rather than solving
    once more for the whole offset from the original centerline. This is the
    iterative scheme from the paper, and the re-parameterisation is what
    keeps the linearisation valid -- see linearized_curvature_system.
    """
    if not (math.isfinite(vehicle_half_width) and vehicle_half_width > 0.0):
        raise ValueError('vehicle_half_width must be finite and positive')
    if not (math.isfinite(safety_margin) and safety_margin >= 0.0):
        raise ValueError('safety_margin must be finite and non-negative')
    if iterations < 1:
        raise ValueError('iterations must be at least 1')

    original = resample_closed_path(centerline, spacing)
    n = len(original)
    base_left = _resample_scalar_to(
        np.asarray(centerline, dtype=np.float64), np.asarray(width_left, float), n)
    base_right = _resample_scalar_to(
        np.asarray(centerline, dtype=np.float64), np.asarray(width_right, float), n)

    keep_out = vehicle_half_width + safety_margin
    reference = original
    curvature_history = []
    clamped_fraction = 0.0

    for _ in range(iterations):
        count = len(reference)
        _, normals = path_frames(reference)
        h = _mean_spacing(reference)

        # Re-derive the room left on each side from the *original* track
        # definition every pass, rather than carrying it forward. Updating it
        # incrementally (subtract this pass's step, re-sample, repeat) looks
        # equivalent and is not: the first-order width update and the
        # re-sampling each leave a small error, they accumulate in the same
        # direction, and after six passes on Spielberg the finished line sat
        # 0.045m outside the corridor it was supposed to be constrained to.
        # A constraint that drifts is not a constraint.
        remaining_left, remaining_right = _remaining_width(
            original, base_left, base_right, reference)

        upper = remaining_left - keep_out
        lower = -(remaining_right - keep_out)
        # Where the corridor is narrower than the car plus its margin the
        # bounds cross over. Collapsing both to the midpoint rather than
        # erroring keeps one pinched gate from throwing away the whole track,
        # and the caller is told how much of the lap it happened on.
        # Where the corridor is narrower than the car plus its margin the
        # bounds cross over. Collapsing them to the midpoint rather than
        # erroring keeps one pinched gate from throwing away the whole track,
        # and the caller is told how much of the lap it happened on. This is
        # measured before the trust region so the number keeps meaning
        # "too narrow for the car", not "clipped by a solver setting".
        too_narrow = lower > upper
        clamped_fraction = float(np.count_nonzero(too_narrow)) / count
        upper, lower = _order_bounds(upper, lower)

        # Trust region. The curvature linearisation is only good for small
        # offsets, and the widest corner on a track is not a small offset.
        # Capping the step keeps every pass inside the range its own
        # linearisation is valid over, which is the whole reason the
        # iteration exists. Measured on Spielberg it does not change where
        # the optimization converges; it bounds how wildly it can get there.
        #
        # Re-ordering afterwards is not belt-and-braces: if a previous pass
        # left the line outside the corridor, the clamp can pull the upper
        # bound past the lower one, and the solver rejects crossed bounds
        # outright rather than doing something sensible with them.
        if trust_region is not None:
            if not (math.isfinite(trust_region) and trust_region > 0.0):
                raise ValueError('trust_region must be finite and positive, or None')
            upper, lower = _order_bounds(np.minimum(upper, trust_region),
                                         np.maximum(lower, -trust_region))

        a_matrix, offset = linearized_curvature_system(reference, h)
        step = solve_lateral_offsets(
            a_matrix, offset, lower, upper, smoothing_weight, h)

        line = reference + step[:, None] * normals
        # The arc-length integral of squared curvature, not the plain sum:
        # each pass re-samples, so the point count changes between passes and
        # a bare sum would compare two different quantities.
        curvature_history.append(float(
            np.sum(racing_math.estimate_path_curvature(line) ** 2)
            * _mean_spacing(line)))

        # Re-parameterise to uniform arc length for the next pass. This is
        # what keeps the linearisation valid -- see
        # linearized_curvature_system.
        reference = resample_closed_path(line, spacing)

    line = reference
    final_left, final_right = _remaining_width(
        original, base_left, base_right, line)
    return {
        'line': line,
        'alpha': _signed_offset(original, line),
        'centerline': original,
        'remaining_left': final_left,
        'remaining_right': final_right,
        'curvature_history': curvature_history,
        'clamped_fraction': clamped_fraction,
        'spacing': _mean_spacing(line),
    }


def _order_bounds(upper: np.ndarray, lower: np.ndarray):
    """Guarantee ``lower < upper`` everywhere, collapsing any crossed pair to
    a hair-wide interval about its midpoint. The solver rejects crossed
    bounds rather than interpreting them."""
    crossed = lower > upper
    if not np.any(crossed):
        return upper, lower
    midpoint = (upper + lower) / 2.0
    return (np.where(crossed, midpoint + 1e-6, upper),
            np.where(crossed, midpoint - 1e-6, lower))


def _nearest_index(centerline: np.ndarray, line: np.ndarray) -> np.ndarray:
    """Index of the nearest centerline point for every point of `line`."""
    deltas = line[:, None, :] - centerline[None, :, :]
    return np.argmin(np.hypot(deltas[:, :, 0], deltas[:, :, 1]), axis=1)


def _signed_offset(centerline: np.ndarray, line: np.ndarray) -> np.ndarray:
    """Signed lateral distance from the centerline to each point of `line`,
    positive to the left of the direction of travel."""
    _, normals = path_frames(centerline)
    nearest = _nearest_index(centerline, line)
    delta = line - centerline[nearest]
    return (delta[:, 0] * normals[nearest, 0]
            + delta[:, 1] * normals[nearest, 1])


def _remaining_width(centerline: np.ndarray, width_left: np.ndarray,
                     width_right: np.ndarray, line: np.ndarray):
    """Room still available either side of `line`, from the track definition.

    Projects each point of the current line onto the original centerline and
    spends its offset out of that point's measured widths. Because it always
    reads the original widths, repeated calls cannot drift.
    """
    nearest = _nearest_index(centerline, line)
    offset = _signed_offset(centerline, line)
    return width_left[nearest] - offset, width_right[nearest] + offset


def refine_centerline(occ_map, seed_path: np.ndarray, spacing: float,
                      max_width: float, iterations: int = 4,
                      smoothing_window: int = 3):
    """Turn a recorded lap into a centerline with per-point track widths.

    A recorded lap is a poor centerline -- it is wherever the car happened to
    be driven, often much closer to one wall than the other -- but it is an
    excellent *seed*, because it is guaranteed to lie inside the track, to go
    around it exactly once, and to run in the racing direction. None of those
    are things a skeletonisation of the occupancy grid gives you for free.

    So: measure the walls either side of the seed, step the seed to the
    middle of what it measured, and repeat. Each pass makes the normals more
    nearly perpendicular to the track, which makes the next width measurement
    more accurate, and it converges in a handful of passes.

    Two things make that loop unstable on a real map, and both are handled
    here rather than hoped away:

      * **A ray that escapes.** A pit entry, an unmapped doorway or a hole in
        a thin wall lets a ray run to `max_width`, which reads as an enormous
        amount of room on that side and throws the point clean out of the
        track. Its neighbours are then measured from outside too, and the
        damage spreads: measured on Spielberg, 3 escaped rays became 13 in
        four passes. Points where either ray escaped get no vote -- their
        correction is interpolated from the points either side, which do.
      * **Corrections much larger than the track.** The step is smoothed
        along the path and capped, because a centerline is a smooth thing and
        no single correct measurement ever asks for a metre of jump.

    Where a side genuinely has no wall, the reported width falls back to the
    map's clearance field -- the radius of the largest free disc at that
    point, which is a guaranteed-safe half-width -- instead of `max_width`.
    Handing the optimizer `max_width` there would authorise it to plan the
    line straight out through the gap.

    Returns ``(centerline, width_left, width_right)``.
    """
    from pure_pursuit import occupancy_map as occ

    line = resample_closed_path(seed_path, spacing)
    for _ in range(max(1, iterations)):
        _, normals = path_frames(line)
        width_left, width_right, found_left, found_right = occ.measure_track_widths(
            occ_map, line, normals, max_width)

        shift = (width_left - width_right) / 2.0
        shift = _interpolate_over(shift, found_left & found_right)
        shift = _smooth_circular(shift, half_window=2)
        np.clip(shift, -max_width / 2.0, max_width / 2.0, out=shift)
        line = resample_closed_path(line + shift[:, None] * normals, spacing)

    # Every ray cast is quantised to the cell size, so the re-centred line
    # carries a cell-scale wiggle. That is invisible on a plot and loud to a
    # three-point curvature estimate -- on Spielberg's 0.058m grid it put
    # 3.84/m of pure noise into a centerline whose real maximum is 1.5/m,
    # which then reads as phantom hairpins in the baseline comparison. Same
    # cleanup, and same reasoning, as racing_math.smooth_path applies to a
    # recorded lap. Smooth first, then measure the widths, so the widths
    # belong to the line actually being returned.
    line = resample_closed_path(
        racing_math.smooth_path(line, smoothing_window), spacing)

    _, normals = path_frames(line)
    width_left, width_right, found_left, found_right = occ.measure_track_widths(
        occ_map, line, normals, max_width)
    if not (found_left.all() and found_right.all()):
        clearance = occ_map.clearance_at(line[:, 0], line[:, 1])
        width_left = np.where(found_left, width_left, clearance)
        width_right = np.where(found_right, width_right, clearance)
    return line, width_left, width_right


def _interpolate_over(values: np.ndarray, trusted: np.ndarray) -> np.ndarray:
    """Replace untrusted entries by interpolating the trusted ones around the
    loop. All-untrusted returns zeros: no information, so no correction."""
    values = np.asarray(values, dtype=np.float64)
    trusted = np.asarray(trusted, dtype=bool)
    if trusted.all():
        return values.copy()
    if not trusted.any():
        return np.zeros_like(values)
    n = len(values)
    index = np.arange(n)
    good = index[trusted]
    # Wrap one trusted sample around each end so the seam interpolates too.
    extended_index = np.concatenate([good - n, good, good + n])
    extended_values = np.tile(values[trusted], 3)
    return np.interp(index, extended_index, extended_values)


def _smooth_circular(values: np.ndarray, half_window: int) -> np.ndarray:
    """Circular moving average of a per-point scalar."""
    if half_window <= 0:
        return np.asarray(values, dtype=np.float64).copy()
    size = 2 * half_window + 1
    padded = np.concatenate([values[-half_window:], values, values[:half_window]])
    kernel = np.ones(size) / size
    return np.convolve(padded, kernel, mode='valid')


def curvature_limit(max_steering_angle: float, wheelbase: float) -> float:
    """Tightest curvature the steering rack can physically ask for.

    The bicycle model the controller already uses inverted: the car cannot
    drive a line tighter than ``tan(delta_max) / L``, whatever the optimizer
    thinks. On this car that is tan(0.26)/0.324 = 0.82 1/m, a 1.22m radius.
    """
    if not (math.isfinite(max_steering_angle)
            and 0.0 < max_steering_angle < math.pi / 2.0):
        raise ValueError('max_steering_angle must be finite and in (0, pi/2)')
    if not (math.isfinite(wheelbase) and wheelbase > 0.0):
        raise ValueError('wheelbase must be finite and positive')
    return math.tan(max_steering_angle) / wheelbase


def _mean_spacing(xy: np.ndarray) -> float:
    loop = np.vstack([xy, xy[:1]])
    return float(np.mean(np.hypot(np.diff(loop[:, 0]), np.diff(loop[:, 1]))))


def _resample_scalar_to(xy: np.ndarray, values: np.ndarray, count: int) -> np.ndarray:
    """Re-sample a per-point scalar onto `count` uniform arc-length samples."""
    if len(values) != len(xy):
        raise ValueError('per-point values must match the path length')
    loop = np.vstack([xy, xy[:1]])
    steps = np.hypot(np.diff(loop[:, 0]), np.diff(loop[:, 1]))
    arc = np.concatenate([[0.0], np.cumsum(steps)])
    total = float(arc[-1])
    targets = np.linspace(0.0, total, count, endpoint=False)
    return np.interp(targets, arc, np.concatenate([values, values[:1]]))
