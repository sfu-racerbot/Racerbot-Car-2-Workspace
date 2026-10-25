"""Batched LiDAR: sphere tracing on each track's signed-distance field, plus
exact ray-versus-rectangle hits on the other cars.

Beams that reach `cast_limit` without a hit read as a valid return at that
range (indoors the real UST-30LX nearly always hits something within its
30 m; the encoding saturates at 10 m anyway). Only dropout makes a beam
invalid, as a dark or glancing surface does on the car.
"""
import math

import torch


def beam_angles(n: int = 1081, fov: float = math.radians(270.0), device='cpu'):
    return torch.linspace(-fov / 2, fov / 2, n, device=device)


def ray_box(ox, oy, dx, dy, cx, cy, cyaw, hx, hy):
    """Distance along unit rays (o, d) to oriented boxes (centre c, yaw, half
    sizes); inf on a miss. Broadcasts over leading dims."""
    c, s = torch.cos(cyaw), torch.sin(cyaw)
    px, py = ox - cx, oy - cy
    lx, ly = c * px + s * py, -s * px + c * py          # origin in the box frame
    ddx, ddy = c * dx + s * dy, -s * dx + c * dy
    big = torch.full_like(lx, 1e9)
    inv_x = torch.where(ddx.abs() > 1e-12, 1.0 / ddx, big * torch.sign(ddx + 1e-30))
    inv_y = torch.where(ddy.abs() > 1e-12, 1.0 / ddy, big * torch.sign(ddy + 1e-30))
    tx1, tx2 = (-hx - lx) * inv_x, (hx - lx) * inv_x
    ty1, ty2 = (-hy - ly) * inv_y, (hy - ly) * inv_y
    t_near = torch.maximum(torch.minimum(tx1, tx2), torch.minimum(ty1, ty2))
    t_far = torch.minimum(torch.maximum(tx1, tx2), torch.maximum(ty1, ty2))
    hit = (t_near <= t_far) & (t_far > 0)
    return torch.where(hit, t_near.clamp(min=0.0), torch.full_like(t_near, math.inf))


def cast(pool, slot, ox, oy, yaw, angles, cars=None, cast_limit: float = 12.0,
         iters: int = 64, eps: float = 0.005, refine: int = 5):
    """Ranges (E, C, B) for LiDARs at (ox, oy, yaw) of shape (E, C).

    cars: optional (cx, cy, cyaw, hx, hy, active) each (E, C) -- every car's
    body; a car never sees its own.
    """
    E, C = ox.shape
    B = angles.shape[0]
    a = yaw.unsqueeze(-1) + angles                      # (E, C, B)
    dx, dy = torch.cos(a), torch.sin(a)
    x0, y0 = ox.unsqueeze(-1).expand_as(a), oy.unsqueeze(-1).expand_as(a)
    sl = slot.view(E, 1, 1).expand_as(a)
    t = torch.zeros_like(a)
    last = torch.zeros_like(a)
    hit = torch.zeros_like(a, dtype=torch.bool)
    for _ in range(iters):
        d = pool.sample(sl, x0 + t * dx, y0 + t * dy)
        hit = hit | (d < eps)
        # relaxed sphere tracing: a small floor on the step keeps grazing rays
        # moving; the overshoot it allows is bounded by the floor itself, and
        # the bisection below takes most of it back
        step = torch.maximum(d, 0.005 + 0.002 * t)
        moving = ~(hit | (t >= cast_limit))
        last = torch.where(moving, step, last)
        t = torch.where(moving, t + step, t)
    # bisect each hit between the last free point and where it stopped
    lo, hi = (t - last).clamp(min=0.0), t
    for _ in range(refine):
        mid = 0.5 * (lo + hi)
        inside = pool.sample(sl, x0 + mid * dx, y0 + mid * dy) < 0.0
        hi = torch.where(hit & inside, mid, hi)
        lo = torch.where(hit & ~inside, mid, lo)
    t = torch.where(hit, hi, t)
    walls = torch.where(hit, t, torch.full_like(t, cast_limit)).clamp(max=cast_limit)
    if cars is None:
        return walls
    cx, cy, cyaw, hx, hy, active = cars
    best = walls
    for j in range(C):
        tj = ray_box(x0, y0, dx, dy, cx[:, j].view(E, 1, 1), cy[:, j].view(E, 1, 1),
                     cyaw[:, j].view(E, 1, 1), hx[:, j].view(E, 1, 1), hy[:, j].view(E, 1, 1))
        own = torch.zeros(C, dtype=torch.bool, device=ox.device)
        own[j] = True
        usable = active[:, j].view(E, 1, 1) & ~own.view(1, C, 1)
        best = torch.where(usable, torch.minimum(best, tj), best)
    return best


def add_noise(ranges, noise_std, dropout, gen, range_max_invalid: float):
    """Gaussian range noise, and dropped beams set to range_max_invalid (which
    the observation reads as 'no return')."""
    n = torch.randn(ranges.shape, generator=gen, device=gen.device).to(ranges.device)
    r = ranges + n * noise_std.view(-1, *([1] * (ranges.dim() - 1)))
    u = torch.rand(ranges.shape, generator=gen, device=gen.device).to(ranges.device)
    drop = u < dropout.view(-1, *([1] * (ranges.dim() - 1)))
    return torch.where(drop, torch.full_like(r, range_max_invalid), r.clamp(min=0.05))
