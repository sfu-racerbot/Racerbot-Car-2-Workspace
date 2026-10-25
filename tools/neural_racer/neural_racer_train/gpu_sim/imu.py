"""The simulated IMU: yaw rate and specific force at the IMU's mount point.

From the single-track state x and its derivative f (dynamics.st_derivative)
at the centre of gravity, in the body frame:

    v_x = V cos(beta),  v_y = V sin(beta)
    a_x = d/dt(v_x) - w v_y,     a_y = d/dt(v_y) + w v_x
    at a point d ahead of the CoG:  a_x,P = a_x - w^2 d,   a_y,P = a_y + w_dot d

The mount point is measured from the rear axle (base_link); the CoG sits lr
ahead of it, so d = imu_offset_x - lr. At the front bumper (~0.45 m) that is
~0.27 m: in a 2 rad/s turn the IMU reads ~1.1 m/s^2 more than the CoG
(plan A3). Simulating the sensor where it sits means neither side corrects.
"""
import torch


def specific_force(x, f, d):
    """(gyro_z, a_x, a_y) at a point d metres ahead of the CoG; all (...,)."""
    v, beta = x[..., 3], x[..., 6]
    v_dot, w, w_dot, beta_dot = f[..., 3], f[..., 4], f[..., 5], f[..., 6]
    vx, vy = v * torch.cos(beta), v * torch.sin(beta)
    vx_dot = v_dot * torch.cos(beta) - v * torch.sin(beta) * beta_dot
    vy_dot = v_dot * torch.sin(beta) + v * torch.cos(beta) * beta_dot
    ax = vx_dot - w * vy - w * w * d
    ay = vy_dot + w * vx + w_dot * d
    return w, ax, ay
