"""Procedural closed-loop tracks for the ROS-level simulator.

Why generate a track instead of using F1TENTH Gym's shipped ones: the
official tracks (Spielberg, Silverstone, Brands Hatch) are 300m+ laps.
`auto_map_race_launch.py` maps at 1.0m/s by default, so two mapping laps
plus racing laps on one of those is over 20 minutes of wall clock per
validation run -- long enough that nobody runs it, which is how the
automatic mode got into the state it was in.

The tracks here are room-sized closed loops, deliberately in the same
size class as the space this car is actually driven and mapped in: a
~30m lap that takes ~30s at mapping speed. That keeps a full
map-then-race validation under five minutes while still exercising every
gate `auto_map_race_node` applies (`minimum_lap_distance: 20.0m`,
`minimum_lap_duration_sec: 15.0`, `departure_distance`, `closure_*`).

Output is a plain ROS map_server-style triplet that F1TENTH Gym's
`Track.from_track_path` reads directly:

    <dir>/<name>.png             occupancy image, white = free
    <dir>/<name>_map.yaml        resolution/origin/thresholds
    <dir>/<name>_centerline.csv  x_m,y_m -- the loop, for scoring and
                                 for scripted opponents to follow

No ROS and no F1TENTH Gym imports here, so this module is unit-testable
on its own (see test/test_tracks.py).
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

# Image convention: PIL loads the map with a FLIP_TOP_BOTTOM, so the file
# is written in the usual ROS orientation (row 0 = largest world Y) and
# ends up the right way round inside the simulator.
FREE = 255
OCCUPIED = 0


def rounded_rectangle_centerline(half_x: float, half_y: float, radius: float,
                                 spacing: float = 0.05) -> np.ndarray:
    """Closed centerline of a rounded rectangle, sampled at ~`spacing`.

    Index 0 sits at the middle of the bottom straight heading +x, so a car
    spawned on the first point starts on a straight rather than mid-corner
    -- the same courtesy a human gives the car when placing it on the line
    before a mapping run.
    """
    if radius <= 0.0 or radius > min(half_x, half_y):
        raise ValueError('radius must be positive and fit inside the rectangle')
    if spacing <= 0.0:
        raise ValueError('spacing must be positive')

    straight_x = 2.0 * (half_x - radius)   # length of top/bottom straights
    straight_y = 2.0 * (half_y - radius)   # length of left/right straights
    quarter = 0.5 * math.pi * radius

    # Walk the loop counter-clockwise from the middle of the bottom edge.
    segments = [
        ('straight', straight_x / 2.0, (0.0, -half_y), (1.0, 0.0)),
        ('corner', quarter, (half_x - radius, -(half_y - radius)), -0.5 * math.pi),
        ('straight', straight_y, (half_x, -(half_y - radius)), (0.0, 1.0)),
        ('corner', quarter, (half_x - radius, half_y - radius), 0.0),
        ('straight', straight_x, (half_x - radius, half_y), (-1.0, 0.0)),
        ('corner', quarter, (-(half_x - radius), half_y - radius), 0.5 * math.pi),
        ('straight', straight_y, (-half_x, half_y - radius), (0.0, -1.0)),
        ('corner', quarter, (-(half_x - radius), -(half_y - radius)), math.pi),
        ('straight', straight_x / 2.0, (-(half_x - radius), -half_y), (1.0, 0.0)),
    ]

    points = []
    for kind, length, anchor, extra in segments:
        count = max(2, int(round(length / spacing)))
        # Drop the duplicated endpoint shared with the next segment.
        ts = np.linspace(0.0, length, count, endpoint=False)
        if kind == 'straight':
            ax, ay = anchor
            dx, dy = extra
            for t in ts:
                points.append((ax + dx * t, ay + dy * t))
        else:
            cx, cy = anchor
            start_angle = extra
            for t in ts:
                angle = start_angle + t / radius
                points.append((cx + radius * math.cos(angle),
                               cy + radius * math.sin(angle)))
    return np.asarray(points, dtype=np.float64)


def occupancy_from_centerline(centerline: np.ndarray, corridor_width: float,
                              resolution: float, margin: float):
    """Rasterize "everything more than half a corridor from the loop is wall".

    Returns (image, origin_x, origin_y) with the image in ROS orientation
    (row 0 = largest world Y).
    """
    if corridor_width <= 0.0 or resolution <= 0.0:
        raise ValueError('corridor_width and resolution must be positive')

    half_corridor = 0.5 * corridor_width
    pad = half_corridor + margin
    min_x = float(centerline[:, 0].min()) - pad
    max_x = float(centerline[:, 0].max()) + pad
    min_y = float(centerline[:, 1].min()) - pad
    max_y = float(centerline[:, 1].max()) + pad

    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))

    # Cell centres in world coordinates. Row 0 is the largest Y (ROS image
    # convention); the simulator flips it back on load.
    xs = min_x + (np.arange(width) + 0.5) * resolution
    ys = max_y - (np.arange(height) + 0.5) * resolution
    grid_x, grid_y = np.meshgrid(xs, ys)

    # Distance from every cell to the nearest centerline sample. The loop is
    # sampled far more finely than the corridor is wide, so nearest-sample
    # distance is a faithful stand-in for distance to the polyline and costs
    # one broadcast instead of a segment-projection pass.
    flat = np.stack([grid_x.ravel(), grid_y.ravel()], axis=1)
    nearest = np.full(flat.shape[0], np.inf)
    # Chunked so a fine grid never allocates cells x samples at once.
    chunk = max(1, int(4_000_000 // max(1, len(centerline))))
    for start in range(0, flat.shape[0], chunk):
        block = flat[start:start + chunk]
        d = np.hypot(block[:, None, 0] - centerline[None, :, 0],
                     block[:, None, 1] - centerline[None, :, 1])
        nearest[start:start + chunk] = d.min(axis=1)

    image = np.where(nearest.reshape(height, width) <= half_corridor,
                     FREE, OCCUPIED).astype(np.uint8)
    return image, min_x, min_y


def write_track(directory, name: str, centerline: np.ndarray,
                corridor_width: float, resolution: float = 0.05,
                margin: float = 0.5) -> Path:
    """Write the png/yaml/centerline triplet; return the path to pass to Gym.

    The returned path is `<dir>/<name>.yaml`, which is what
    `Track.from_track_path` wants: it takes the *stem* from that path and
    then opens `<stem>_map.yaml` beside it. The file itself is never read,
    so it is not created.
    """
    from PIL import Image  # local: keeps this module importable without PIL

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    image, origin_x, origin_y = occupancy_from_centerline(
        centerline, corridor_width, resolution, margin)

    image_name = f'{name}.png'
    Image.fromarray(image, mode='L').save(directory / image_name)

    (directory / f'{name}_map.yaml').write_text(
        f'image: {image_name}\n'
        f'resolution: {resolution}\n'
        f'origin: [{origin_x}, {origin_y}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
    )

    lines = ['x_m,y_m']
    lines += [f'{x:.6f},{y:.6f}' for x, y in centerline]
    (directory / f'{name}_centerline.csv').write_text('\n'.join(lines) + '\n')

    return directory / f'{name}.yaml'


# Named layouts. `half_x`/`half_y` are the centerline's half-extents, so the
# room a track needs is roughly 2*half + corridor_width + 1m of margin.
LAYOUTS = {
    # ~29.7m lap in a 14 x 10m room -- the default. Long enough to clear
    # minimum_lap_distance (20m) with real headroom, small enough that a
    # mapping lap at 1.0m/s takes about half a minute.
    'indoor_oval': dict(half_x=5.0, half_y=3.2, radius=1.8, corridor_width=1.8),
    # Same idea, tighter corners and a narrower corridor: this is the one
    # that makes gap_follow work for its lap and exercises the corner
    # fallback path rather than cruising the whole way round.
    'indoor_tight': dict(half_x=4.5, half_y=2.6, radius=1.1, corridor_width=1.4),
    # Bigger loop for two- and three-car traffic, where cars need somewhere
    # to actually complete a pass.
    'indoor_wide': dict(half_x=7.0, half_y=4.5, radius=2.5, corridor_width=2.6),
}


def build(name: str, directory) -> Path:
    """Build a named layout into `directory` and return its Gym path."""
    if name not in LAYOUTS:
        raise KeyError(f"unknown track layout '{name}'; have {sorted(LAYOUTS)}")
    spec = dict(LAYOUTS[name])
    corridor_width = spec.pop('corridor_width')
    centerline = rounded_rectangle_centerline(**spec)
    return write_track(directory, name, centerline, corridor_width)


def centerline_path(directory, name: str) -> Path:
    return Path(directory) / f'{name}_centerline.csv'


def load_centerline(directory, name: str) -> np.ndarray:
    return np.loadtxt(centerline_path(directory, name), delimiter=',',
                      skiprows=1, dtype=np.float64)
