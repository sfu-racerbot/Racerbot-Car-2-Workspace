"""Access to the existing CPU simulator (tools/f1tenth_sim) -- the referee.

It is imported, never copied or edited: its sim_fidelity layer is calibrated
to the car and carries its own collision geometry, which is why it, not the
GPU training simulator, judges every candidate (plan A7, Phase 9).
"""
import sys

from neural_racer_train.common import REPO

TOOLS_SIM = REPO / 'tools' / 'f1tenth_sim'


def ensure_paths() -> None:
    if str(TOOLS_SIM) not in sys.path:
        sys.path.insert(0, str(TOOLS_SIM))
    from sim_fidelity import bootstrap
    bootstrap.bootstrap()


def calibration_module():
    ensure_paths()
    from sim_fidelity import calibration
    return calibration


def gym_available() -> bool:
    try:
        ensure_paths()
        import f1tenth_gym  # noqa: F401
        return True
    except Exception:  # noqa: BLE001
        return False
