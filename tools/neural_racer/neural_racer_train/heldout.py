"""The held-out set (plan Phase 1 and 4): tracks no training stage ever sees.

    python -m neural_racer_train.heldout          # (re)write heldout.yaml and the maps

  real        the four imported course maps in tools/f1tenth_sim/maps
  procedural  20 tracks from seeds at or above learn.HELDOUT_SEED_BASE (a range
              training never draws from), written as gym map triplets into
              .sim/neural_racer_heldout/ (gitignored; regenerated from the seeds)

heldout.yaml records each procedural track's digest, and the training pool
refuses any track with one of those digests (learn.Runner._insert).
"""
import sys
from pathlib import Path

import numpy as np
import yaml

from neural_racer_train import common, tracks
from neural_racer_train.learn import HELDOUT_FILE, HELDOUT_SEED_BASE

REAL = ('room_2026_07_27', 'levine_blocked', 'levine_obs', 'circular_obs')
N_PROCEDURAL = 20
MAP_DIR = common.REPO / '.sim' / 'neural_racer_heldout'


def name_of(k: int) -> str:
    return f'heldout_{k:02d}'


def write_gym_map(raster, directory: Path, name: str) -> None:
    """Same triplet as tools/f1tenth_sim/track_import.write_gym_track: PNG
    (white free, image row 0 at the top), ROS map yaml, centreline CSV."""
    from PIL import Image
    directory.mkdir(parents=True, exist_ok=True)
    img = np.where(raster.occupancy, 0, 255).astype(np.uint8)[::-1]
    Image.fromarray(img, mode='L').save(directory / f'{name}.png')
    (directory / f'{name}_map.yaml').write_text(
        f'image: {name}.png\nresolution: {raster.resolution}\n'
        f'origin: [{raster.origin[0]}, {raster.origin[1]}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    lines = ['x_m,y_m'] + [f'{x:.4f},{y:.4f}' for x, y in raster.spec.centre[::2]]
    (directory / f'{name}_centerline.csv').write_text('\n'.join(lines) + '\n')


def build() -> dict:
    entries = []
    for k in range(N_PROCEDURAL):
        seed = HELDOUT_SEED_BASE + k
        r = tracks.rasterize(tracks.generate(seed, kind='smooth_loop' if k % 2 == 0 else 'filleted_polygon'))
        write_gym_map(r, MAP_DIR, name_of(k))
        entries.append({'name': name_of(k), 'seed': seed, 'digest': r.digest,
                        'lap_length_m': round(r.lap_length, 2), 'kind': r.spec.kind})
    data = {'real': list(REAL), 'procedural': entries,
            'note': 'Never train on these. Regenerate with python -m neural_racer_train.heldout.'}
    HELDOUT_FILE.write_text(yaml.safe_dump(data, sort_keys=False))
    return data


def ensure_maps() -> None:
    if not (MAP_DIR / f'{name_of(N_PROCEDURAL - 1)}_map.yaml').exists():
        build()


def names() -> list:
    data = yaml.safe_load(HELDOUT_FILE.read_text()) if HELDOUT_FILE.exists() else build()
    return list(data['real']) + [e['name'] for e in data['procedural']]


def main(argv=None) -> int:
    data = build()
    print(f"wrote {len(data['procedural'])} procedural held-out tracks to {MAP_DIR} and {HELDOUT_FILE}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
