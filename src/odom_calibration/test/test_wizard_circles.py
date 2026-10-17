"""Run the wizard's multi-setting circle checks (test_wizard_circles.js) in Node."""

from pathlib import Path
import shutil
import subprocess

import pytest


def test_multi_setting_circles_in_the_page():
    node = shutil.which('node')
    if node is None:
        pytest.skip('UI regression runner requires Node.js; install Node to run it.')
    result = subprocess.run(
        [node, str(Path(__file__).with_suffix('.js'))],
        capture_output=True, text=True, timeout=15,
    )
    assert result.returncode == 0, result.stdout + result.stderr
