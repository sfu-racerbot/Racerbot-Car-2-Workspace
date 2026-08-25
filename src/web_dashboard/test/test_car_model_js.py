"""
Runs test/browser/car_model_test.js as part of the normal pytest run.

That suite covers two things about the car icon that nothing else does.

The first is paint order. The icon is drawn to scale -- a real 0.36 x 0.30 m
footprint -- so at any zoom close enough to be useful it covers the LIDAR
points inside its own outline, and those are the points reading a wall the
car is about to touch. The car used to be painted after the scan, so the
picture hid exactly the beams worth seeing.

The second is the geometry itself. The old icon was a pile of multiples of
a `size` that tracked the zoom level: it was never any particular size, its
origin was not `base_link`, and the LIDAR was not drawn at all -- so a scan
that was 0.26 m out of place relative to the car looked fine. Every measured
number is now checked against the measurement it came from or against a
closed form (the Ackermann identity cot(outer) - cot(inner) = track /
wheelbase), never against a value recorded from the code.

Skipped, not failed, where node is unavailable:
`node src/web_dashboard/test/browser/car_model_test.js` is always the
direct way to run it.
"""
import os
import shutil
import subprocess

import pytest

_HERE = os.path.dirname(__file__)
_TEST_JS = os.path.join(_HERE, 'browser', 'car_model_test.js')
_NODE = shutil.which('node') or shutil.which('nodejs')


@pytest.mark.skipif(_NODE is None, reason='node is not installed')
def test_browser_car_model():
    result = subprocess.run(
        [_NODE, _TEST_JS],
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, (
        f'car-model tests failed (exit {result.returncode})\n'
        f'--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}'
    )
    assert 'checks passed' in result.stdout, result.stdout
