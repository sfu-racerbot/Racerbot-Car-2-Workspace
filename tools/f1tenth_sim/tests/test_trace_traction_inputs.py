"""Exercise the actual caller expressions with real traction logic.

Collected by pytest tools/f1tenth_sim/tests/. Compiling just the command call
keeps these wiring regressions independent of gym physics and map assets;
neither the controller nor its ROS-free logic is mocked.
"""
import ast
from pathlib import Path
import sys

import numpy as np
import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))
from gap_mirror import GapController, node_params  # noqa: E402


@pytest.mark.parametrize('filename,caller,receiver', [
    ('trace_gap.py', 'trace', 'ctrl'),
    ('run_validation.py', 'run_gap_solo', 'controller'),
])
def test_simulator_caller_supplies_trusted_inputs_and_engages_traction(
        filename, caller, receiver):
    tree = ast.parse((SIM / filename).read_text())
    function = next(n for n in tree.body
                    if isinstance(n, ast.FunctionDef) and n.name == caller)
    calls = [n for n in ast.walk(function) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr == 'command'
             and isinstance(n.func.value, ast.Name) and n.func.value.id == receiver]
    assert len(calls) == 1
    command = compile(ast.Expression(calls[0]), str(SIM / filename), 'eval')

    class Lidar:
        angle_min = -3 * np.pi / 4
        angle_increment = 3 * np.pi / 2 / 1080
        range_min = 0.023

    class Truth:
        # Synthetic slip at the plant boundary: wheel 2.0, true/fused 1.7.
        # The real gym cannot spin wheels, so an ordinary lap cannot cover it.
        def truth(self, index):
            return np.array([0.0, 0.0, 0.0, 1.7, 0.0, 0.0, 0.0])

    params = dict(node_params(), enable_imu_traction=True)
    controller = GapController(params, Lidar())
    controller.last_commanded_speed = 1.5
    state = np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0])
    ego = {'std_state': state, 'scan': np.full(1081, 10.0)}
    inputs = dict(np=np, ctrl=controller, controller=controller, ego=ego,
                  state=state, speed_meas=2.0, step=0, plant=Truth())
    _, speed = eval(command, inputs)
    # oracle: spec I5 ruling: slip .3 > shipped .246, trusted IMU -> hold;
    # open space has no lower ceiling, so min(desired,last=1.5) = 1.5 m/s.
    assert controller.telemetry['imu_traction_holding']
    assert speed == 1.5
