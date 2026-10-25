"""Tracks resident on the GPU: signed-distance fields plus centreline tables.

Every environment points at one slot. Sampling is manual bilinear
interpolation into one flat tensor, so 16k environments on 4k different
tracks cost one gather, with no per-environment copy of any map.
"""
import torch

from neural_racer_train.tracks import RasterTrack


class TrackPool:
    def __init__(self, slots: int, canvas=(1024, 1024), max_points: int = 1400,
                 resolution: float = 0.05, device='cpu', dtype=torch.float16):
        self.slots = slots
        self.H, self.W = canvas
        self.res = resolution
        self.device = device
        self.sdf = torch.full((slots, self.H, self.W), -1.0, dtype=dtype, device=device)
        self.origin = torch.zeros((slots, 2), device=device)
        self.centre = torch.zeros((slots, max_points, 5), device=device)   # x, y, s, w_left, w_right
        self.n_points = torch.ones(slots, dtype=torch.long, device=device)
        self.length = torch.ones(slots, device=device)
        self.max_points = max_points
        self.digest = [None] * slots

    def insert(self, raster: RasterTrack, slot: int) -> None:
        h, w = raster.sdf.shape
        if h > self.H or w > self.W:
            raise ValueError(f'track {raster.digest} is {h}x{w} cells; the canvas is {self.H}x{self.W}')
        n = len(raster.spec.centre)
        if n > self.max_points:
            raise ValueError(f'track has {n} centreline points; the pool holds {self.max_points}')
        self.sdf[slot].fill_(-1.0)
        self.sdf[slot, :h, :w] = torch.as_tensor(raster.sdf, device=self.device).to(self.sdf.dtype)
        self.origin[slot] = torch.tensor(raster.origin, device=self.device)
        c = torch.as_tensor(raster.spec.centre, dtype=torch.float32, device=self.device)
        seg = torch.linalg.norm(torch.roll(c, -1, 0) - c, dim=1)
        s = torch.cat([torch.zeros(1, device=self.device), torch.cumsum(seg, 0)[:-1]])
        self.centre[slot].zero_()
        self.centre[slot, :n, 0:2] = c
        self.centre[slot, :n, 2] = s
        self.centre[slot, :n, 3] = torch.as_tensor(raster.spec.w_left, dtype=torch.float32, device=self.device)
        self.centre[slot, :n, 4] = torch.as_tensor(raster.spec.w_right, dtype=torch.float32, device=self.device)
        self.n_points[slot] = n
        self.length[slot] = float(seg.sum())
        self.digest[slot] = raster.digest

    def sample(self, slot, x, y):
        """Bilinear SDF at world (x, y) on each element's track; outside the canvas is wall."""
        shape = x.shape
        slot = slot.expand(shape).reshape(-1)
        x, y = x.reshape(-1), y.reshape(-1)
        u = (x - self.origin[slot, 0]) / self.res - 0.5
        v = (y - self.origin[slot, 1]) / self.res - 0.5
        c0, r0 = torch.floor(u), torch.floor(v)
        fu, fv = u - c0, v - r0
        c0, r0 = c0.long(), r0.long()
        flat = self.sdf.view(-1)
        base = slot * (self.H * self.W)

        def at(r, c):
            inside = (r >= 0) & (r < self.H) & (c >= 0) & (c < self.W)
            idx = base + r.clamp(0, self.H - 1) * self.W + c.clamp(0, self.W - 1)
            return torch.where(inside, flat[idx].float(), torch.full_like(fu, -1.0))
        val = (at(r0, c0) * (1 - fu) * (1 - fv) + at(r0, c0 + 1) * fu * (1 - fv)
               + at(r0 + 1, c0) * (1 - fu) * fv + at(r0 + 1, c0 + 1) * fu * fv)
        return val.reshape(shape)
