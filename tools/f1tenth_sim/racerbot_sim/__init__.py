"""RacerBot's fidelity layer over the stock F1TENTH Gym.

The gym checkout under ``.sim/`` is upstream code at a pinned commit and is
kept pristine -- ``setup.sh`` refuses to run if it has local modifications.
Everything in this package layers on top of it from the outside, so the
simulator can be made to behave like *this* car without forking upstream.

See ``tools/f1tenth_sim/README.md`` for what each fix does and why, and
``docs/sim-fidelity-audit.md`` for the measurements that motivated them.
"""

from .calibration import CALIBRATION, CarCalibration, Provenance
from .grip import GripEnvelope
from .plant import FidelityPlant, FidelityProfile, PROFILES

__all__ = [
    "CALIBRATION",
    "CarCalibration",
    "Provenance",
    "GripEnvelope",
    "FidelityPlant",
    "FidelityProfile",
    "PROFILES",
]
