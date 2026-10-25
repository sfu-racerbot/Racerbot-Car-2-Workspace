"""Neural racer training (docs/superpowers/specs/2026-10-23-neural-racer-plan.md).

Importing the package puts the car-side packages (neural_racer, gap_follow,
drive_intent) on sys.path, so every module here can use their ROS-free code
-- the observation spec and safety wrapper -- whatever order it imports in.
"""
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
for _rel in ('src/neural_racer', 'src/gap_follow', 'src/drive_intent'):
    _p = str(_REPO / _rel)
    if _p not in sys.path:
        sys.path.insert(0, _p)
