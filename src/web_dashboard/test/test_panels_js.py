"""
Runs the browser-side geometry suite for the panel manager
(test/browser/panels_test.js) as part of the normal pytest run.

`web/panels.js` is what lets a section be popped out of the info panel,
dragged, snapped against its neighbours and resized. The arithmetic
underneath that is easy to get subtly wrong in ways nobody reports as a
bug -- a snap that picks the wrong edge, or a resize that walks the panel
across the screen while you drag its left side -- so it is checked
directly rather than by dragging things about and squinting.

Skipped, not failed, where node is unavailable: this is a convenience
wrapper, and `node src/web_dashboard/test/browser/panels_test.js` is
always the direct way to run it.
"""
import os
import shutil
import subprocess

import pytest

_HERE = os.path.dirname(__file__)
_TEST_JS = os.path.join(_HERE, 'browser', 'panels_test.js')
_NODE = shutil.which('node') or shutil.which('nodejs')


@pytest.mark.skipif(_NODE is None, reason='node is not installed')
def test_browser_panel_geometry():
    result = subprocess.run(
        [_NODE, _TEST_JS],
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, (
        f'panel geometry tests failed (exit {result.returncode})\n'
        f'--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}'
    )
    assert 'checks passed' in result.stdout, result.stdout
