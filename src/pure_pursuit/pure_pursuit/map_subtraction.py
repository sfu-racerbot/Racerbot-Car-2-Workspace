"""
map_subtraction.py

The one range_libc-touching piece of map-subtraction opponent detection:
given the saved map and a localized pose, compute the scan the LIDAR
*should* see if the world matched the map exactly. Comparing that
against the real scan (racing_math.detect_dynamic_cluster) exposes
anything that is not in the map -- an opponent car, wherever it is,
regardless of what the wall behind it looks like.

Kept separate from racing_math.py on purpose: everything in that file is
importable and unit-testable with numpy alone, while this file needs the
workspace-built range_libc extension (the same ray-casting library the
particle filter already uses for exactly this kind of query -- see
src/particle_filter/particle_filter/particle_filter.py's get_omap()).
pure_pursuit_node only imports this module when
opponent_detection_mode == 'map', so the heuristic mode keeps working
even on a machine where range_libc isn't built.
"""

import numpy as np
import range_libc


class MapRayCaster:
    """Ray-casts expected LIDAR ranges from a nav_msgs/OccupancyGrid.

    Built once per received map (the map is static for the whole run).
    PyRayMarching is used rather than the fancier CDDT variants because
    the query volume here is tiny -- a few hundred beams per control
    tick, versus the particle filter's beams x thousands-of-particles --
    so the simplest method with no precomputation cost wins.
    """

    def __init__(self, map_msg, max_range_m: float):
        max_range_px = int(max_range_m / map_msg.info.resolution)
        self._range_method = range_libc.PyRayMarching(
            range_libc.PyOMap(map_msg), max_range_px)
        self.max_range_m = float(max_range_m)

    def expected_ranges(self, laser_x: float, laser_y: float, yaw: float,
                        beam_angles: np.ndarray) -> np.ndarray:
        """Map-predicted range for each beam, in meters, from the LIDAR's
        world-frame position. `beam_angles` are scan-frame angles (as in
        LaserScan: 0 = straight ahead); world heading is added here.
        """
        n = len(beam_angles)
        queries = np.zeros((n, 3), dtype=np.float32)
        queries[:, 0] = laser_x
        queries[:, 1] = laser_y
        queries[:, 2] = yaw + beam_angles
        expected = np.zeros(n, dtype=np.float32)
        self._range_method.calc_range_many(queries, expected)
        return expected.astype(np.float64)
