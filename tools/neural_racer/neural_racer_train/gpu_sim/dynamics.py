"""Batched single-track vehicle dynamics: a PyTorch copy of the pinned F1TENTH
Gym's vehicle_dynamics_st, steering_constraint, accl_constraints and
pid_accl (f1tenth_gym/envs/dynamic_models), integrated with the same RK4.

State x (..., 7): X, Y, DELTA (steering angle), V, PSI (yaw), PSI_DOT, BETA (slip).
Control u (..., 2): steering velocity, longitudinal acceleration.

Every branch of gym's scalar code is a torch.where here, so one call steps
every environment. Parity with gym's own functions is
tests/test_dynamics_parity.py; nothing here is trusted without it.
"""
import torch

G = 9.81


def steering_constraint(delta, sv, s_min, s_max, sv_min, sv_max):
    at_stop = ((delta <= s_min) & (sv <= 0)) | ((delta >= s_max) & (sv >= 0))
    return torch.where(at_stop, torch.zeros_like(sv), torch.clamp(sv, sv_min, sv_max))


def accl_constraints(v, a, v_switch, a_max, v_min, v_max):
    upper = torch.where(v > v_switch, a_max * v_switch / v.clamp(min=1e-9), a_max)
    hold = ((v <= v_min) & (a <= 0)) | ((v >= v_max) & (a >= 0))
    out = torch.where(a <= -a_max, -a_max, torch.where(a >= upper, upper, a))
    return torch.where(hold, torch.zeros_like(a), out)


def pid_accl(speed_cmd, v, a_max, v_max, v_min):
    diff = speed_cmd - v
    t = lambda x: torch.as_tensor(x, dtype=diff.dtype, device=diff.device)   # noqa: E731
    a_max, v_max, v_min = t(a_max), t(v_max), t(v_min)     # never round the gains to float32
    fwd = torch.where(diff > 0, 10.0 * a_max / v_max, 10.0 * a_max / (-v_min))
    back = torch.where(diff > 0, 2.0 * a_max / v_max, 2.0 * a_max / (-v_min))
    return torch.where(v > 0.0, fwd, back) * diff


def st_derivative(x, u, p):
    """Right-hand side of gym's single-track model, constraints applied to the
    stage state exactly as gym applies them inside its derivative."""
    delta, v, psi, psi_dot, beta = x[..., 2], x[..., 3], x[..., 4], x[..., 5], x[..., 6]
    sv = steering_constraint(delta, u[..., 0], p.steering_max_right, p.steering_max_left,
                             -p.steering_rate_max, p.steering_rate_max)
    a = accl_constraints(v, u[..., 1], p.v_switch, p.a_max, p.gym_v_min, p.gym_v_max)
    lf, lr, h, mu, csf, csr, m, inertia = p.lf, p.lr, p.h, p.mu, p.C_Sf, p.C_Sr, p.m, p.I
    lwb = lf + lr

    # kinematic branch, V < 0.5 m/s
    tan_d = torch.tan(delta)
    beta_hat = torch.atan(tan_d * lr / lwb)
    beta_dot_k = (1.0 / (1.0 + (tan_d * (lr / lwb)) ** 2)) * (lr / (lwb * torch.cos(delta) ** 2)) * sv
    kin = torch.stack([
        v * torch.cos(psi + beta_hat),
        v * torch.sin(psi + beta_hat),
        sv,
        a,
        v * torch.cos(beta_hat) * tan_d / lwb,
        (1.0 / lwb) * (a * torch.cos(beta) * tan_d - v * torch.sin(beta) * tan_d * beta_dot_k
                       + (v * torch.cos(beta) * sv) / (torch.cos(delta) ** 2)),
        beta_dot_k,
    ], dim=-1)

    # dynamic branch
    vs = torch.where(v.abs() < 1e-6, torch.full_like(v, 1e-6), v)   # unused where v < 0.5
    glr = G * lr - a * h
    glf = G * lf + a * h
    dyn = torch.stack([
        v * torch.cos(psi + beta),
        v * torch.sin(psi + beta),
        sv,
        a,
        psi_dot,
        ((mu * m) / (inertia * lwb)) * (
            lf * csf * glr * delta + (lr * csr * glf - lf * csf * glr) * beta
            - (lf * lf * csf * glr + lr * lr * csr * glf) * (psi_dot / vs)),
        (mu / (vs * lwb)) * (
            csf * glr * delta - (csr * glf + csf * glr) * beta
            + (csr * glf * lr - csf * glr * lf) * (psi_dot / vs)) - psi_dot,
    ], dim=-1)
    return torch.where((v < 0.5).unsqueeze(-1), kin, dyn)


def rk4(x, u, p, dt: float):
    k1 = st_derivative(x, u, p)
    k2 = st_derivative(x + dt * (k1 / 2), u, p)
    k3 = st_derivative(x + dt * (k2 / 2), u, p)
    k4 = st_derivative(x + dt * k3, u, p)
    return x + dt * (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
