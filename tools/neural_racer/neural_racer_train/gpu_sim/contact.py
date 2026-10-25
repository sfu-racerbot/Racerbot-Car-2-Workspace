"""Contact: chassis against walls (via the SDF), chassis against chassis (the
separating-axis test, as tools/f1tenth_sim/sim_fidelity/plant.py does), and
fault by closing speed (plan A8).

Bodies are the real chassis (0.30 x 0.535 m), centred on the model's centre
of gravity -- the gym state's (x, y). Contact between two cars must use the
real chassis: padding both double-counts the margin (calibration.py).
"""
import torch


def corners(cx, cy, yaw, hx, hy):
    """(..., 4, 2) corners of oriented rectangles."""
    c, s = torch.cos(yaw), torch.sin(yaw)
    lx = torch.stack([hx, hx, -hx, -hx], dim=-1)
    ly = torch.stack([hy, -hy, -hy, hy], dim=-1)
    x = cx.unsqueeze(-1) + c.unsqueeze(-1) * lx - s.unsqueeze(-1) * ly
    y = cy.unsqueeze(-1) + s.unsqueeze(-1) * lx + c.unsqueeze(-1) * ly
    return torch.stack([x, y], dim=-1)


def outline_points(cx, cy, yaw, hx, hy):
    """Corners plus edge midpoints: (..., 8, 2)."""
    k = corners(cx, cy, yaw, hx, hy)
    mids = 0.5 * (k + torch.roll(k, -1, dims=-2))
    return torch.cat([k, mids], dim=-2)


def wall_contact(pool, slot, cx, cy, yaw, hx, hy):
    """(E, C) True where any outline point is inside a wall."""
    pts = outline_points(cx, cy, yaw, hx, hy)               # (E, C, 8, 2)
    E = cx.shape[0]
    d = pool.sample(slot.view(E, *([1] * (pts.dim() - 2))), pts[..., 0], pts[..., 1])
    return (d < 0.0).any(dim=-1)


def _project(k, axis):
    p = (k * axis.unsqueeze(-2)).sum(-1)
    return p.min(-1).values, p.max(-1).values


def rect_overlap(ka, kb):
    """Separating-axis test for rectangles (..., 4, 2): True where they overlap."""
    axes = []
    for k in (ka, kb):
        e = k[..., 1, :] - k[..., 0, :]
        f = k[..., 3, :] - k[..., 0, :]
        axes += [e / torch.linalg.norm(e, dim=-1, keepdim=True),
                 f / torch.linalg.norm(f, dim=-1, keepdim=True)]
    overlap = torch.ones(ka.shape[:-2], dtype=torch.bool, device=ka.device)
    for ax in axes:
        a0, a1 = _project(ka, ax)
        b0, b1 = _project(kb, ax)
        overlap &= (a1 >= b0) & (b1 >= a0)
    return overlap


def car_contacts(cx, cy, yaw, vx, vy, hx, hy, active):
    """(contact (E, C), at_fault (E, C)) over every pair of active cars.

    Fault: in a contact between A and B, A is at fault when its closing speed
    toward B, max(0, v_A . n_AB), exceeds B's toward A -- a rear-ender, or a
    car that turns into another (plan A8; replace with the rulebook's rule).
    """
    E, C = cx.shape
    k = corners(cx, cy, yaw, hx, hy)
    contact = torch.zeros((E, C), dtype=torch.bool, device=cx.device)
    fault = torch.zeros_like(contact)
    for i in range(C):
        for j in range(i + 1, C):
            both = active[:, i] & active[:, j]
            hit = rect_overlap(k[:, i], k[:, j]) & both
            nx, ny = cx[:, j] - cx[:, i], cy[:, j] - cy[:, i]
            norm = torch.sqrt(nx * nx + ny * ny).clamp(min=1e-9)
            nx, ny = nx / norm, ny / norm
            close_i = (vx[:, i] * nx + vy[:, i] * ny).clamp(min=0.0)
            close_j = (-(vx[:, j] * nx + vy[:, j] * ny)).clamp(min=0.0)
            contact[:, i] |= hit
            contact[:, j] |= hit
            fault[:, i] |= hit & (close_i > close_j)
            fault[:, j] |= hit & (close_j > close_i)
    return contact, fault
