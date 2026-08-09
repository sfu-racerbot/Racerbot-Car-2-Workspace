"""Import bootstrap shared by the validation runner and any probe script.

Keeps the ``sys.path`` juggling for the gitignored ``.sim/`` checkout in one
place so probes reproduce the runner's import environment exactly.
"""

from __future__ import annotations

import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
SIM_ROOT = ROOT / ".sim"


def bootstrap() -> Path:
    """Put the gym checkout and the car's packages on ``sys.path``.

    Idempotent. Returns the workspace root.
    """
    for import_path in (
        SIM_ROOT / "python",
        SIM_ROOT / "f1tenth_gym",
        ROOT / "src" / "gap_follow",
        ROOT / "src" / "pure_pursuit",
    ):
        text = str(import_path)
        if text not in sys.path:
            sys.path.insert(0, text)
    os.environ.setdefault("NUMBA_CACHE_DIR", str(SIM_ROOT / "numba-cache"))
    return ROOT
