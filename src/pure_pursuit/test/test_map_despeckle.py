"""Stray-return cleanup for saved occupancy maps.

The 2026-08-19 report was of "stray lidar beams that detected stuff in the
middle of the track that wasn't actually there". Measured on this car's own
saved map (~/.ros/racerbot_auto/20260727-200103/map.pgm): 46 of its 56
occupied blobs are 4 cells or smaller, 2.3% of the occupied area but 82% of
the distinct objects in the map.

These are deletions from an obstacle map, so the tests that matter most are
the ones asserting what is *kept*.

    python3 -m pytest src/pure_pursuit/test/test_map_despeckle.py -v
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pure_pursuit.occupancy_map import (  # noqa: E402
    FREE, OCCUPIED, UNKNOWN, DEFAULT_SPECKLE_MAX_CELLS, OccupancyMap,
    despeckle_grid, despeckle_map_file)


def free_field(height=20, width=20):
    """An observed, empty room with a wall all the way round.

    The border matters: `despeckle_grid` never touches a blob against the
    edge of the grid, so every fixture needs real margin.
    """
    grid = np.full((height, width), FREE, dtype=np.int8)
    grid[0, :] = grid[-1, :] = OCCUPIED
    grid[:, 0] = grid[:, -1] = OCCUPIED
    return grid


def test_a_single_stray_cell_in_clear_space_is_removed():
    grid = free_field()
    grid[10, 10] = OCCUPIED
    cleaned, blobs, cells = despeckle_grid(grid)
    assert (blobs, cells) == (1, 1)
    assert cleaned[10, 10] == FREE


def test_the_walls_are_never_touched():
    grid = free_field()
    grid[10, 10] = OCCUPIED
    cleaned, _, _ = despeckle_grid(grid)
    # Every wall cell survives: they are one connected component far bigger
    # than max_cells, and they run along the grid edge.
    assert (cleaned[0, :] == OCCUPIED).all()
    assert (cleaned[-1, :] == OCCUPIED).all()
    assert (cleaned[:, 0] == OCCUPIED).all()
    assert (cleaned[:, -1] == OCCUPIED).all()


def test_a_blob_that_casts_a_shadow_is_kept():
    """The safety case: a real object occludes, so unknown cells sit behind
    it. A cone is small too -- being small is not what makes a return
    stray, having been seen straight through is."""
    grid = free_field()
    grid[10, 10] = OCCUPIED
    grid[11, 10] = UNKNOWN          # its shadow
    cleaned, blobs, cells = despeckle_grid(grid)
    assert (blobs, cells) == (0, 0)
    assert cleaned[10, 10] == OCCUPIED


def test_the_halo_is_exactly_one_cell_wide_by_default():
    """A blob touching unobserved space is kept; one clear cell of margin
    is enough to judge it. halo_cells is what sets that reach."""
    def blobs_removed(gap, halo):
        grid = free_field()
        grid[10, 10] = OCCUPIED
        grid[10, 10 + gap] = UNKNOWN
        return despeckle_grid(grid, halo_cells=halo)[1]

    assert blobs_removed(gap=1, halo=1) == 0, 'unknown right beside it'
    assert blobs_removed(gap=2, halo=1) == 1, 'one clear cell is enough'
    assert blobs_removed(gap=2, halo=2) == 0, 'a wider halo reaches further'
    # halo 0 judges on size alone, which is the unsafe setting and is why
    # it is not the default.
    assert blobs_removed(gap=1, halo=0) == 1


def test_a_blob_larger_than_the_threshold_is_kept():
    grid = free_field()
    grid[9:12, 9:12] = OCCUPIED     # 9 cells
    cleaned, blobs, _ = despeckle_grid(grid, max_cells=4)
    assert blobs == 0
    assert (cleaned[9:12, 9:12] == OCCUPIED).all()


def test_the_threshold_is_inclusive_and_counts_diagonals():
    grid = free_field()
    # Four cells in an L, touching only diagonally at one corner.
    grid[10, 10] = grid[10, 11] = grid[11, 11] = grid[12, 12] = OCCUPIED
    cleaned, blobs, cells = despeckle_grid(grid, max_cells=4)
    assert (blobs, cells) == (1, 4), 'diagonal touching is one blob, not two'
    assert (cleaned[10:13, 10:13] != OCCUPIED).all()


def test_max_cells_zero_disables_the_filter_entirely():
    grid = free_field()
    grid[10, 10] = OCCUPIED
    cleaned, blobs, cells = despeckle_grid(grid, max_cells=0)
    assert (blobs, cells) == (0, 0)
    assert np.array_equal(cleaned, grid)


def test_the_input_grid_is_never_modified():
    grid = free_field()
    grid[10, 10] = OCCUPIED
    original = grid.copy()
    despeckle_grid(grid)
    assert np.array_equal(grid, original)


def test_only_occupied_cells_ever_change_and_only_to_free():
    grid = free_field()
    grid[10, 10] = OCCUPIED
    grid[5, 5] = UNKNOWN
    cleaned, _, _ = despeckle_grid(grid)
    changed = grid != cleaned
    assert (grid[changed] == OCCUPIED).all()
    assert (cleaned[changed] == FREE).all()


def test_unknown_cells_are_left_alone():
    grid = free_field()
    grid[5:8, 5:8] = UNKNOWN
    cleaned, blobs, _ = despeckle_grid(grid)
    assert blobs == 0
    assert (cleaned[5:8, 5:8] == UNKNOWN).all()


def test_a_clean_map_is_returned_unchanged():
    grid = free_field()
    cleaned, blobs, cells = despeckle_grid(grid)
    assert (blobs, cells) == (0, 0)
    assert np.array_equal(cleaned, grid)


def test_the_default_threshold_is_a_10cm_patch():
    # 4 cells at the 0.05m resolution this car maps at.
    assert DEFAULT_SPECKLE_MAX_CELLS == 4


def test_a_despeckled_map_no_longer_blocks_the_cell():
    """The point of all this: the phantom stops costing clearance."""
    grid = free_field(40, 40)
    grid[20, 20] = OCCUPIED
    speckled = OccupancyMap(grid, 0.05, 0.0, 0.0, inflate_cells=0)
    cleaned_grid, _, _ = despeckle_grid(grid)
    cleaned = OccupancyMap(cleaned_grid, 0.05, 0.0, 0.0, inflate_cells=0)
    world = (20 * 0.05 + 0.025, 20 * 0.05 + 0.025)
    assert speckled.is_blocked(*world)
    assert not cleaned.is_blocked(*world)


def write_map(tmp_path, grid, free_value=254, occupied_value=0,
              unknown_value=205):
    """Write a grid as the map_server yaml+pgm pair, image-row-0-is-top."""
    from PIL import Image
    pixels = np.full(grid.shape, unknown_value, dtype=np.uint8)
    pixels[grid == FREE] = free_value
    pixels[grid == OCCUPIED] = occupied_value
    image_path = tmp_path / 'map.pgm'
    Image.fromarray(np.flipud(pixels), mode='L').save(str(image_path))
    yaml_path = tmp_path / 'map.yaml'
    yaml_path.write_text(
        'image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return str(yaml_path)


def test_despeckle_map_file_cleans_the_image_in_place(tmp_path):
    grid = free_field(30, 30)
    grid[15, 15] = OCCUPIED
    yaml_path = write_map(tmp_path, grid)

    before = OccupancyMap.from_yaml(yaml_path, inflate_cells=0)
    assert before.grid[15, 15] == OCCUPIED

    blobs, cells = despeckle_map_file(yaml_path)
    assert (blobs, cells) == (1, 1)

    after = OccupancyMap.from_yaml(yaml_path, inflate_cells=0)
    assert after.grid[15, 15] == FREE
    # Nothing else moved: same shape, same walls, same metadata.
    assert after.grid.shape == before.grid.shape
    assert after.resolution == before.resolution
    assert (after.grid[0, :] == OCCUPIED).all()
    assert int((before.grid != after.grid).sum()) == 1


def test_despeckle_map_file_leaves_a_clean_map_byte_identical(tmp_path):
    grid = free_field(30, 30)
    yaml_path = write_map(tmp_path, grid)
    image_path = tmp_path / 'map.pgm'
    original = image_path.read_bytes()
    assert despeckle_map_file(yaml_path) == (0, 0)
    assert image_path.read_bytes() == original


def test_despeckle_map_file_keeps_an_object_with_a_shadow(tmp_path):
    grid = free_field(30, 30)
    grid[15, 15] = OCCUPIED
    grid[16, 15] = UNKNOWN
    yaml_path = write_map(tmp_path, grid)
    assert despeckle_map_file(yaml_path) == (0, 0)
    assert OccupancyMap.from_yaml(yaml_path, inflate_cells=0).grid[15, 15] == OCCUPIED


def test_from_grid_message_despeckles_only_when_asked():
    """Off by default, because most callers read /map to avoid things."""
    width = height = 30
    values = np.zeros((height, width), dtype=np.int16)
    values[0, :] = values[-1, :] = values[:, 0] = values[:, -1] = 100
    values[15, 15] = 100

    kept = OccupancyMap.from_grid_message(
        values.ravel().tolist(), width, height, 0.05, 0.0, 0.0,
        inflate_cells=0)
    assert kept.grid[15, 15] == OCCUPIED

    cleaned = OccupancyMap.from_grid_message(
        values.ravel().tolist(), width, height, 0.05, 0.0, 0.0,
        inflate_cells=0, despeckle_max_cells=4)
    assert cleaned.grid[15, 15] == FREE


def test_the_real_saved_map_loses_only_phantoms_in_clear_track():
    """Regression against this car's actual map, if it is still on disk.

    12 blobs in observed-clear track go; the 34 small ones against walls
    and the map edge stay, because unobserved space behind them is the
    honest answer and removing them would be a guess.
    """
    path = os.path.expanduser(
        '~/.ros/racerbot_auto/20260727-200103/map.yaml')
    if not os.path.exists(path):
        pytest.skip('the recorded reference map is not on this machine')
    before = OccupancyMap.from_yaml(path, inflate_cells=0)
    after = OccupancyMap.from_yaml(path, inflate_cells=0, despeckle_max_cells=4)
    removed = int((before.grid == OCCUPIED).sum() - (after.grid == OCCUPIED).sum())
    assert 0 < removed < 0.02 * int((before.grid == OCCUPIED).sum()), (
        'a filter taking more than 2% of the occupied area off a real map '
        'is not removing speckle any more')
