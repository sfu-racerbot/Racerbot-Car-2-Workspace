"""Observation spec v1 for the neural racer: the one place its constants live.

What the network sees each tick, as fixed-scale float32 blocks
(docs/superpowers/specs/2026-10-23-neural-racer-plan.md, section A4):

    scan     (4, 270)  log-range of the last three scans + validity of the newest
    ego      (23,)     speeds, last commands, IMU, mode, speed cap, camera flags
    preview  (4, 32)   the next 16 m of the reference path (map mode only)
    race     (6,)      lap fraction, place, gaps (map mode only)
    camera   (64,)     camera embedding (zeros until the camera is added)

Every scale is a constant, never a running statistic, so the car's NumPy copy
and the training code's batched copy (gpu_sim/observation_torch.py, which
imports everything below) cannot drift apart. A parity test holds them
together.

No rclpy import: this runs unsourced, on the car and on FIR.
"""
import math
from collections import deque

import numpy as np

OBS_SPEC_VERSION = 1

# --- scan block ---------------------------------------------------------
N_BEAMS = 1081            # Hokuyo UST-30LX, 270 deg at 0.25 deg
POOL = 4                  # beams per bin; beam 1080 is dropped
N_BINS = 270
R_MIN = 0.05              # m, the encoding's floor
R_MAX = 10.0              # m, the encoding's ceiling and the "no return" value
LOG_SPAN = math.log(R_MAX / R_MIN)
HISTORY = 3               # scans, speeds, commands and IMU samples kept
SCAN_CHANNELS = HISTORY + 1

# --- ego vector ----------------------------------------------------------
V_SCALE = 5.0             # m/s
STEER_LIMIT = 0.33        # rad, autonomy's max_steering_angle on car 1
YAW_SCALE = 3.0           # rad/s
ACC_SCALE = 9.81          # m/s^2
CAMERA_AGE_SCALE = 0.2    # s
EGO_DIM = 23
EGO = {
    'speed': slice(0, 3),         # t, t-1, t-2
    'steer_cmd': slice(3, 6),     # t-1, t-2, t-3
    'speed_cmd': slice(6, 9),     # t-1, t-2, t-3
    'gyro_z': slice(9, 12),       # t, t-1, t-2
    'acc_x': slice(12, 15),
    'acc_y': slice(15, 18),
    'imu_present': 18,
    'mode': 19,
    'v_cap': 20,
    'camera_present': 21,
    'camera_age': 22,
}
MODE_REACTIVE = 0
MODE_MAP = 1

# --- map-mode blocks -----------------------------------------------------
PREVIEW_POINTS = 32
PREVIEW_SPACING = 0.5     # m between preview points
PREVIEW_XY_SCALE = 16.0   # m
PREVIEW_WIDTH_SCALE = 2.0  # m
RACE_DIM = 6
CAMERA_DIM = 64

# --- auxiliary outputs ---------------------------------------------------
AUX_PATH_POINTS = 6
AUX_DT = 0.25             # s between forecast points
AUX_POS_SCALE = 4.0       # m: network output x this = metres
OPP_SLOTS = 4


class ObservationError(ValueError):
    """The inputs cannot make a trustworthy observation this tick."""


def beam_validity(ranges, range_min: float, range_max: float) -> np.ndarray:
    """True where a beam is a real return: finite, above range_min, below range_max."""
    r = np.asarray(ranges, dtype=np.float64)
    with np.errstate(invalid='ignore'):
        return np.isfinite(r) & (r > range_min) & (r < range_max)


def encode_range(r) -> np.ndarray:
    """Metres -> [0, 1] on a log scale: 0.05 m -> 0, 10 m -> 1, clipped both ends."""
    clipped = np.clip(np.asarray(r, dtype=np.float64), R_MIN, R_MAX)
    return np.log(clipped / R_MIN) / LOG_SPAN


def pool_scan(ranges, range_min: float, range_max: float):
    """1081 raw ranges -> (270 encoded log-ranges, 270 bin validities), float32.

    Each bin takes the *minimum* valid range of its four beams, so the nearest
    return survives -- a thin chair leg or a car's corner cannot vanish into an
    average. A bin with no valid beam reads R_MAX with validity 0: "no echo"
    and "far away" stay distinguishable.
    """
    r = np.asarray(ranges, dtype=np.float64)
    if r.ndim != 1 or r.size != N_BEAMS:
        raise ObservationError(
            f'beam count {r.size} != {N_BEAMS}: the observation spec assumes '
            'the UST-30LX at full resolution')
    valid = beam_validity(r, range_min, range_max)
    usable = np.where(valid, r, np.inf)[:N_BINS * POOL].reshape(N_BINS, POOL)
    nearest = usable.min(axis=1)
    bin_valid = np.isfinite(nearest)
    nearest = np.where(bin_valid, nearest, R_MAX)
    return encode_range(nearest).astype(np.float32), bin_valid.astype(np.float32)


def _last(seq, k: int):
    """k-th most recent entry (0 = newest), repeating the oldest if short."""
    if not seq:
        raise ObservationError('history is empty')
    return seq[max(len(seq) - 1 - k, 0)]


class History:
    """What the node keeps between ticks. Missing history at the start of a
    run is filled by repeating the oldest entry -- the training simulator does
    exactly the same at the start of every episode."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.scans = deque(maxlen=HISTORY)
        self.valid = None
        self.speeds = deque(maxlen=HISTORY)
        self.steer_cmds = deque([0.0] * HISTORY, maxlen=HISTORY)
        self.speed_cmds = deque([0.0] * HISTORY, maxlen=HISTORY)
        self.imu = deque(maxlen=HISTORY)

    def push_tick(self, encoded_scan, bin_valid, speed: float, imu=None) -> None:
        """One scan's worth of inputs. `imu` is (gyro_z, acc_x, acc_y) in SI,
        base_link axes, or None when this tick has no usable IMU sample."""
        if not math.isfinite(speed):
            raise ObservationError('speed is not finite')
        if imu is not None and not all(math.isfinite(v) for v in imu):
            raise ObservationError('IMU sample is not finite')
        self.scans.append(np.asarray(encoded_scan, dtype=np.float32))
        self.valid = np.asarray(bin_valid, dtype=np.float32)
        self.speeds.append(float(speed))
        self.imu.append(None if imu is None else tuple(float(v) for v in imu))

    def push_command(self, steering: float, speed: float) -> None:
        """The command actually published this tick (after the safety wrapper)."""
        self.steer_cmds.append(float(steering))
        self.speed_cmds.append(float(speed))

    def imu_complete(self) -> bool:
        return len(self.imu) > 0 and all(s is not None for s in self.imu)


def build_observation(history: History, *, imu_present: bool, mode: int, v_cap: float,
                      preview=None, race=None, camera=None,
                      camera_age: float = 0.0) -> dict:
    """The observation dict the policy consumes, every block float32.

    Blocks a mode does not use are exactly zero, and their flags say so.
    """
    if mode not in (MODE_REACTIVE, MODE_MAP):
        raise ObservationError(f'unknown mode {mode!r}')
    if not (math.isfinite(v_cap) and v_cap >= 0.0):
        raise ObservationError('v_cap must be finite and non-negative')
    if not history.scans or history.valid is None:
        raise ObservationError('no scan in history')

    scan = np.empty((SCAN_CHANNELS, N_BINS), dtype=np.float32)
    for k in range(HISTORY):
        scan[k] = _last(history.scans, k)
    scan[HISTORY] = history.valid

    ego = np.zeros(EGO_DIM, dtype=np.float32)
    for k in range(HISTORY):
        ego[EGO['speed'].start + k] = _last(history.speeds, k) / V_SCALE
        ego[EGO['steer_cmd'].start + k] = _last(history.steer_cmds, k) / STEER_LIMIT
        ego[EGO['speed_cmd'].start + k] = _last(history.speed_cmds, k) / V_SCALE
    if imu_present:
        if not history.imu_complete():
            raise ObservationError('imu_present but the IMU history has gaps')
        for k in range(HISTORY):
            gz, ax, ay = _last(history.imu, k)
            ego[EGO['gyro_z'].start + k] = gz / YAW_SCALE
            ego[EGO['acc_x'].start + k] = ax / ACC_SCALE
            ego[EGO['acc_y'].start + k] = ay / ACC_SCALE
        ego[EGO['imu_present']] = 1.0
    ego[EGO['mode']] = float(mode)
    ego[EGO['v_cap']] = v_cap / V_SCALE

    preview_block = np.zeros((4, PREVIEW_POINTS), dtype=np.float32)
    race_block = np.zeros(RACE_DIM, dtype=np.float32)
    if mode == MODE_MAP:
        if preview is None or race is None:
            raise ObservationError('map mode needs a preview and race context')
        preview_block[:] = np.asarray(preview, dtype=np.float32).reshape(4, PREVIEW_POINTS)
        race_block[:] = np.asarray(race, dtype=np.float32).reshape(RACE_DIM)
        if not (np.all(np.isfinite(preview_block)) and np.all(np.isfinite(race_block))):
            raise ObservationError('preview or race context is not finite')

    camera_block = np.zeros(CAMERA_DIM, dtype=np.float32)
    if camera is not None:
        camera_block[:] = np.asarray(camera, dtype=np.float32).reshape(CAMERA_DIM)
        if not np.all(np.isfinite(camera_block)) or not math.isfinite(camera_age):
            raise ObservationError('camera embedding is not finite')
        ego[EGO['camera_present']] = 1.0
        ego[EGO['camera_age']] = min(max(camera_age, 0.0) / CAMERA_AGE_SCALE, 1.0)

    return {'scan': scan, 'ego': ego, 'preview': preview_block,
            'race': race_block, 'camera': camera_block}


def race_context(lap_fraction: float, place: int, gap_ahead_s, gap_behind_s) -> np.ndarray:
    """The race block. A gap of None means no car there."""
    def gap(g):
        return 0.0 if g is None else min(max(g, 0.0) / 10.0, 1.0)
    return np.array([
        min(max(lap_fraction, 0.0), 1.0), place / 4.0,
        gap(gap_ahead_s), gap(gap_behind_s),
        0.0 if gap_ahead_s is None else 1.0,
        0.0 if gap_behind_s is None else 1.0,
    ], dtype=np.float32)
