"""Batched observation spec v1 -- the training side of neural_racer/observation.py.

Every constant is imported from that module; nothing is restated. The car
builds one observation a tick in NumPy, this builds (E, C) of them at once,
and tests/test_observation_parity.py holds the two to atol 1e-6.
History semantics match observation.History exactly: missing history repeats
the oldest entry, and the command history starts at zero.
"""
import torch

from neural_racer import observation as obs


def pool_scan(ranges, range_min: float, range_max: float):
    """(..., 1081) metres -> ((..., 270) encoded, (..., 270) bin validity)."""
    valid = torch.isfinite(ranges) & (ranges > range_min) & (ranges < range_max)
    usable = torch.where(valid, ranges, torch.full_like(ranges, float('inf')))
    usable = usable[..., :obs.N_BINS * obs.POOL].reshape(*ranges.shape[:-1], obs.N_BINS, obs.POOL)
    nearest = usable.min(dim=-1).values
    bin_valid = torch.isfinite(nearest)
    nearest = torch.where(bin_valid, nearest, torch.full_like(nearest, obs.R_MAX))
    enc = torch.log(nearest.clamp(obs.R_MIN, obs.R_MAX) / obs.R_MIN) / obs.LOG_SPAN
    return enc, bin_valid.to(enc.dtype)


class ObsHistory:
    """Per-car history tensors, newest at index 0."""

    def __init__(self, E: int, C: int, device='cpu'):
        H, B = obs.HISTORY, obs.N_BINS
        z = lambda *s: torch.zeros(E, C, *s, device=device)   # noqa: E731
        self.scans, self.valid = z(H, B), z(B)
        self.speeds, self.steer, self.speed_cmd = z(H), z(H), z(H)
        self.imu = z(H, 3)
        self.count = torch.zeros(E, C, dtype=torch.long, device=device)

    def reset(self, mask):
        """mask (E, C) bool: those cars start a fresh history."""
        m = mask
        for t in (self.scans, self.valid, self.speeds, self.steer, self.speed_cmd, self.imu):
            t[m] = 0.0
        self.count[m] = 0

    @staticmethod
    def _shift_in(buf, new, count):
        """Newest into slot 0; slots past the `count` real entries repeat the
        oldest real one (observation.History's rule). buf (E, C, H, ...)."""
        buf[:] = torch.roll(buf, 1, dims=2)
        buf[:, :, 0] = new
        extra = buf.dim() - 3
        idx = (count - 1).clamp(0, buf.shape[2] - 1).view(*count.shape, 1, *([1] * extra))
        oldest = torch.gather(buf, 2, idx.expand(*count.shape, 1, *buf.shape[3:])).squeeze(2)
        for k in range(1, buf.shape[2]):
            short = (count <= k).view(*count.shape, *([1] * extra))
            buf[:, :, k] = torch.where(short, oldest, buf[:, :, k])

    def push_tick(self, enc, bin_valid, speed, imu):
        """enc/bin_valid (E, C, 270), speed (E, C), imu (E, C, 3)."""
        cnt = self.count + 1
        self._shift_in(self.scans, enc, cnt)
        self._shift_in(self.speeds, speed, cnt)
        self._shift_in(self.imu, imu, cnt)
        self.valid[:] = bin_valid
        self.count = cnt.clamp(max=obs.HISTORY)

    def push_command(self, steer, speed):
        self.steer[:] = torch.roll(self.steer, 1, dims=2)
        self.speed_cmd[:] = torch.roll(self.speed_cmd, 1, dims=2)
        self.steer[:, :, 0] = steer
        self.speed_cmd[:, :, 0] = speed


def build(hist: ObsHistory, imu_present, mode, v_cap, preview=None, race=None, camera=None,
          camera_present=None, camera_age=None):
    """The A4 blocks, each (E, C, ...). imu_present/mode/v_cap are (E, C)."""
    E, C = mode.shape
    dev = mode.device
    scan = torch.cat([hist.scans, hist.valid.unsqueeze(2)], dim=2)
    ego = torch.zeros(E, C, obs.EGO_DIM, device=dev)
    ego[..., obs.EGO['speed']] = hist.speeds / obs.V_SCALE
    ego[..., obs.EGO['steer_cmd']] = hist.steer / obs.STEER_LIMIT
    ego[..., obs.EGO['speed_cmd']] = hist.speed_cmd / obs.V_SCALE
    ip = imu_present.to(ego.dtype).unsqueeze(-1)
    ego[..., obs.EGO['gyro_z']] = hist.imu[..., 0] / obs.YAW_SCALE * ip
    ego[..., obs.EGO['acc_x']] = hist.imu[..., 1] / obs.ACC_SCALE * ip
    ego[..., obs.EGO['acc_y']] = hist.imu[..., 2] / obs.ACC_SCALE * ip
    ego[..., obs.EGO['imu_present']] = ip.squeeze(-1)
    ego[..., obs.EGO['mode']] = mode.to(ego.dtype)
    ego[..., obs.EGO['v_cap']] = v_cap / obs.V_SCALE
    m = mode.to(ego.dtype).view(E, C, 1, 1)
    prev = (torch.zeros(E, C, 4, obs.PREVIEW_POINTS, device=dev) if preview is None else preview * m)
    rc = torch.zeros(E, C, obs.RACE_DIM, device=dev) if race is None else race * m.view(E, C, 1)
    cam = torch.zeros(E, C, obs.CAMERA_DIM, device=dev)
    if camera is not None:
        cp = camera_present.to(ego.dtype)
        cam = camera * cp.unsqueeze(-1)
        ego[..., obs.EGO['camera_present']] = cp
        ego[..., obs.EGO['camera_age']] = (camera_age / obs.CAMERA_AGE_SCALE).clamp(0, 1) * cp
    return {'scan': scan, 'ego': ego, 'preview': prev, 'race': rc, 'camera': cam}
