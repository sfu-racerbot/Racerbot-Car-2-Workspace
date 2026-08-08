"""
occupancy_map.py

Reading a saved SLAM map off disk and asking geometric questions of it,
with no ROS dependency -- the offline half of what map_subtraction.py does
online against the `/map` topic. Used by optimize_raceline.py to turn a
recorded lap plus a map into a centerline with track widths, which is the
input the minimum-curvature optimizer needs.

Importable and testable with plain pytest:

    python3 -m pytest src/pure_pursuit/test/ -v

The map format is the standard ROS `map_server` pair that
`slam_toolbox`/`nav2_map_server` writes and that the particle filter already
consumes: a `.yaml` describing an image file. See
src/particle_filter/maps/*.yaml for real examples.
"""

import math
import os

import numpy as np
import yaml


# Cells the mapper never observed. Anything not positively known to be free
# is treated as blocking here: an unobserved cell beside the track is far
# more likely to be the wall the LiDAR could not see past than it is to be
# drivable, and the cost of that assumption is a slightly narrower corridor
# rather than a raceline through a wall.
UNKNOWN = -1
FREE = 0
OCCUPIED = 1


class OccupancyMap:
    """A saved occupancy grid, in world coordinates.

    Attributes:
        grid: (h, w) int8 array of FREE / OCCUPIED / UNKNOWN.
        resolution: meters per cell.
        origin_x, origin_y: world coordinates of the grid's lower-left corner.

    Row 0 of the image file is the *top* row, i.e. the highest y -- the
    array is flipped on load so that ``grid[row, col]`` indexes with row
    increasing in +y, matching the world frame instead of the image frame.
    A yaw in the map's ``origin`` is rejected rather than silently ignored;
    every map this workspace produces has yaw 0, and quietly dropping a
    non-zero one would rotate the whole raceline off the track.
    """

    def __init__(self, grid: np.ndarray, resolution: float,
                 origin_x: float, origin_y: float, inflate_cells: int = 1):
        self.grid = np.asarray(grid, dtype=np.int8)
        if self.grid.ndim != 2 or self.grid.size == 0:
            raise ValueError('occupancy grid must be a non-empty 2-D array')
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError('map resolution must be finite and positive')
        if not (math.isfinite(origin_x) and math.isfinite(origin_y)):
            raise ValueError('map origin must be finite')
        if inflate_cells < 0:
            raise ValueError('inflate_cells must be non-negative')
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

        # Walls in a SLAM map are often a single cell thick, and a ray
        # crossing a one-cell diagonal wall can pass clean through the corner
        # between two diagonally-touching cells without ever sampling an
        # occupied one. On Spielberg two rays per lap escaped that way, and
        # because a ray that escapes reports "6m of room over there", the
        # centerline refinement then threw that point right out of the track
        # and the error spread to its neighbours on the next pass -- 3
        # escapes became 13 in four passes.
        #
        # Dilating the blocked set by one cell closes those corners. It
        # costs one cell of measured track width, in the conservative
        # direction, which is the right way to be wrong about a wall.
        blocked = self.grid != FREE
        if inflate_cells > 0:
            from scipy import ndimage
            blocked = ndimage.binary_dilation(blocked, iterations=inflate_cells)
        self._blocked = blocked

    @property
    def height(self) -> int:
        return int(self.grid.shape[0])

    @property
    def width(self) -> int:
        return int(self.grid.shape[1])

    @classmethod
    def from_yaml(cls, yaml_path: str, inflate_cells: int = 1) -> 'OccupancyMap':
        """Load the standard ROS map_server yaml + image pair."""
        with open(yaml_path, 'r') as handle:
            meta = yaml.safe_load(handle)

        for key in ('image', 'resolution', 'origin'):
            if key not in meta:
                raise ValueError(f"map yaml '{yaml_path}' is missing '{key}'")

        origin = meta['origin']
        if len(origin) >= 3 and abs(float(origin[2])) > 1e-9:
            raise ValueError(
                f"map yaml '{yaml_path}' has a rotated origin (yaw="
                f"{float(origin[2]):.4f}rad). This loader does not apply that "
                'rotation, and ignoring it would place the raceline off the '
                'track -- re-save the map with yaw 0.')

        image_path = meta['image']
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                                      image_path)

        from PIL import Image  # local import: only needed for file-backed maps
        with Image.open(image_path) as handle:
            pixels = np.array(handle.convert('L'), dtype=np.float64)

        # map_server convention: p is the probability the cell is occupied.
        negate = int(meta.get('negate', 0))
        p = pixels / 255.0 if negate else (255.0 - pixels) / 255.0

        occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        free_thresh = float(meta.get('free_thresh', 0.196))

        grid = np.full(p.shape, UNKNOWN, dtype=np.int8)
        grid[p > occupied_thresh] = OCCUPIED
        grid[p < free_thresh] = FREE

        # Image row 0 is the top (max y); flip so row increases with +y.
        grid = np.flipud(grid)

        return cls(grid, float(meta['resolution']),
                   float(origin[0]), float(origin[1]),
                   inflate_cells=inflate_cells)

    @classmethod
    def from_grid_message(cls, data, width: int, height: int, resolution: float,
                          origin_x: float, origin_y: float,
                          occupied_threshold: int = 50,
                          inflate_cells: int = 1) -> 'OccupancyMap':
        """Build from a live `nav_msgs/OccupancyGrid`'s fields.

        Takes the message's contents rather than the message, so this module
        stays importable without ROS. The layout is already what this class
        wants -- row-major with row 0 at the smallest world Y -- and only the
        cell values need mapping: an OccupancyGrid carries -1 for unknown and
        an occupancy probability 0-100 otherwise.

        Unknown stays UNKNOWN, which this class blocks: mid-run, a SLAM map
        has plenty of unobserved cells just beyond the walls, and treating
        them as drivable is how a racing line ends up outside the track.
        """
        values = np.asarray(data, dtype=np.int16).reshape(int(height), int(width))
        grid = np.where(values < 0, UNKNOWN,
                        np.where(values >= occupied_threshold, OCCUPIED, FREE))
        return cls(grid.astype(np.int8), resolution, origin_x, origin_y,
                   inflate_cells=inflate_cells)

    def world_to_cell(self, x, y):
        """World (x, y) -> integer (col, row). Vectorized over arrays."""
        col = np.floor((np.asarray(x, dtype=np.float64) - self.origin_x)
                       / self.resolution).astype(int)
        row = np.floor((np.asarray(y, dtype=np.float64) - self.origin_y)
                       / self.resolution).astype(int)
        return col, row

    def is_blocked(self, x, y):
        """True where the world point is occupied, unknown, or off the grid.

        Off-grid counts as blocked for the same reason unknown does: it is
        not somewhere the car has been shown it can drive.
        """
        col, row = self.world_to_cell(x, y)
        inside = ((col >= 0) & (col < self.width)
                  & (row >= 0) & (row < self.height))
        safe_col = np.clip(col, 0, self.width - 1)
        safe_row = np.clip(row, 0, self.height - 1)
        return np.where(inside, self._blocked[safe_row, safe_col], True)

    def cast_ray(self, x: float, y: float, angle: float,
                 max_distance: float, step: float = None) -> float:
        """Distance from (x, y) along `angle` to the first blocked cell.

        Returns `max_distance` if nothing blocks within that range. Stepping
        at half the cell size cannot skip a cell, which a full-resolution
        step can do on a diagonal.
        """
        if step is None:
            step = self.resolution / 2.0
        if not math.isfinite(step) or step <= 0.0:
            raise ValueError('ray step must be finite and positive')
        if not math.isfinite(max_distance) or max_distance <= 0.0:
            raise ValueError('max_distance must be finite and positive')

        n_steps = int(math.ceil(max_distance / step))
        distances = np.arange(1, n_steps + 1, dtype=np.float64) * step
        np.minimum(distances, max_distance, out=distances)
        blocked = self.is_blocked(x + distances * math.cos(angle),
                                  y + distances * math.sin(angle))
        hit = np.flatnonzero(blocked)
        if hit.size == 0:
            return float(max_distance)
        return float(distances[hit[0]])

    def clearance_field(self) -> np.ndarray:
        """Distance in meters from every cell to the nearest blocked cell.

        A Euclidean distance transform over the whole grid. Used to check the
        finished raceline's clearance everywhere at once, which is both far
        cheaper and less brittle than re-casting rays per waypoint.
        """
        from scipy import ndimage
        return ndimage.distance_transform_edt(
            ~self._blocked, sampling=self.resolution)

    def clearance_at(self, x, y, field: np.ndarray = None):
        """Sample the clearance field at world points. 0.0 where blocked."""
        if field is None:
            field = self.clearance_field()
        col, row = self.world_to_cell(x, y)
        inside = ((col >= 0) & (col < self.width)
                  & (row >= 0) & (row < self.height))
        safe_col = np.clip(col, 0, self.width - 1)
        safe_row = np.clip(row, 0, self.height - 1)
        return np.where(inside, field[safe_row, safe_col], 0.0)


def measure_track_widths(occ_map: OccupancyMap, xy: np.ndarray,
                         normals: np.ndarray, max_width: float):
    """Distance to the wall on each side of every reference point.

    Casts one ray along +normal (left) and one along -normal (right) from
    each point. Returns ``(width_left, width_right)`` in meters, each capped
    at `max_width` where no wall was found within that range.

    Also returns ``(found_left, found_right)`` -- whether each ray actually
    struck something. A ray that runs the full `max_width` without hitting
    anything has not measured the track, it has escaped the track: through a
    pit entry, an unmapped doorway, or a hole in a one-cell-thick wall. The
    distinction has to survive to the caller, because "6m of room over
    there" and "no idea" lead to opposite decisions, and silently returning
    the cap for both is what lets a raceline get planned out through a gap.
    """
    points = np.asarray(xy, dtype=np.float64)
    unit_normals = np.asarray(normals, dtype=np.float64)
    if points.shape != unit_normals.shape or points.ndim != 2 or points.shape[1] != 2:
        raise ValueError('xy and normals must both be (n, 2) and the same length')

    width_left = np.empty(len(points))
    width_right = np.empty(len(points))
    for i, ((x, y), (nx, ny)) in enumerate(zip(points, unit_normals)):
        angle = math.atan2(ny, nx)
        width_left[i] = occ_map.cast_ray(x, y, angle, max_width)
        width_right[i] = occ_map.cast_ray(x, y, angle + math.pi, max_width)
    reached_cap = max_width - 1e-9
    return (width_left, width_right,
            width_left < reached_cap, width_right < reached_cap)
