"""Puts the training package and the car-side packages on sys.path, so
`python3 -m pytest tools/neural_racer/tests/` works from the workspace root
with PyTorch on PYTHONPATH (no colcon build, no ROS)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
for rel in ('tools/neural_racer', 'src/neural_racer', 'src/gap_follow', 'src/drive_intent'):
    p = os.path.join(ROOT, rel)
    if p not in sys.path:
        sys.path.insert(0, p)
