"""Exercise real wizard JavaScript using mocked DOM and HTTP boundaries."""

from pathlib import Path
import shutil
import subprocess

import pytest


def test_calibration_selection_and_sidebar():
    node = shutil.which('node')
    if node is None:
        pytest.skip('UI regression runner requires Node.js; install Node to run it.')
    result = subprocess.run(
        [node, str(Path(__file__).with_suffix('.js'))],
        capture_output=True, text=True, timeout=15,
    )
    assert result.returncode == 0, result.stdout + result.stderr
