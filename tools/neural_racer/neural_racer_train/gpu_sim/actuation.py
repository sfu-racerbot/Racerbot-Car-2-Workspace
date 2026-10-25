"""Batched actuation: the chain FidelityPlant.step applies before gym, in the
same order (tools/f1tenth_sim/sim_fidelity/plant.py):

    command delays -> speed clamp -> measured speed loop -> grip clamp on
    steering -> servo velocity -> gym's pid_accl once per tick -> RK4 substeps
"""
import math

import torch

from neural_racer_train.gpu_sim.dynamics import accl_constraints, pid_accl

KINEMATIC_SPEED = 0.5      # grip.KINEMATIC_SPEED: no grip limit below this


class DelayLine:
    """Per-environment FIFO delay of 0..max_ticks control ticks, on the GPU.

    push(x) stores x and returns the value pushed `delay` ticks ago (the
    initial value until the line has filled), like TransportDelay.push.
    """

    def __init__(self, shape, max_ticks: int, delay: torch.Tensor, initial: float = 0.0, device='cpu'):
        self.max = int(max_ticks)
        self.buf = torch.full((self.max + 1, *shape), float(initial), device=device)
        self.head = 0
        self.delay = delay.long().clamp(0, self.max)

    def reset(self, env_ids, value: float = 0.0):
        self.buf[:, env_ids] = value

    def push(self, x):
        self.head = (self.head + 1) % (self.max + 1)
        self.buf[self.head] = x
        idx = (self.head - self.delay) % (self.max + 1)
        while idx.dim() < x.dim():
            idx = idx.unsqueeze(-1)
        idx = idx.expand_as(x)
        return torch.gather(self.buf, 0, idx.unsqueeze(0)).squeeze(0)


def measured_speed_loop(command, measured, p):
    """FidelityPlant._measured_speed_loop: the gym speed command that makes
    gym's pid_accl produce this car's fitted acceleration for `command`."""
    diff = command - measured
    accel_up = torch.minimum(p.speed_loop_accel_gain * diff, p.speed_loop_accel_max)
    accel_dn = torch.maximum(p.speed_loop_brake_gain * diff, -p.speed_loop_brake_max)
    gain_up = 10.0 * p.a_max / p.gym_v_max
    gain_dn = 10.0 * p.a_max / (-p.gym_v_min)
    out = measured + torch.where(diff >= 0.0, accel_up / gain_up, accel_dn / gain_dn)
    return torch.where((measured <= 0.0) & (command <= 0.0), command, out)


def longitudinal_accel(speed_cmd, measured, p):
    """grip.longitudinal_accel: what gym's longitudinal controller is about to do."""
    demand = pid_accl(speed_cmd, measured, p.a_max, p.gym_v_max, p.gym_v_min)
    return accl_constraints(measured, demand, p.v_switch, p.a_max, p.gym_v_min, p.gym_v_max)


def grip_clamp(steering, speed, accel, p):
    """GripEnvelope.apply: clamp steering to the friction circle's lateral budget."""
    limit = p.a_max                      # friction_accel_limit = mu * g
    used = accel * p.longitudinal_grip_coupling
    remaining = limit * limit - used * used
    budget = torch.sqrt(remaining.clamp(min=0.0))
    max_steer = torch.atan(p.wheelbase * budget / (speed * speed).clamp(min=1e-12))
    no_limit = speed < KINEMATIC_SPEED
    clamped = torch.copysign(max_steer, steering)
    return torch.where(no_limit | (steering.abs() <= max_steer), steering, clamped)


def servo_velocity(target, current, p):
    """SteeringServo.velocity: proportional position loop with a slew ceiling."""
    t = torch.clamp(target, p.steering_max_right, p.steering_max_left)
    demand = p.servo_gain * (t - current)
    return torch.clamp(demand, -p.steering_rate_max, p.steering_rate_max)


def actuate(steer_cmd_delayed, speed_cmd_delayed, measured_speed, current_steer, p):
    """The (steering velocity, acceleration) control applied for the next tick."""
    speed = torch.clamp(speed_cmd_delayed, p.speed_min, p.speed_max)
    speed = measured_speed_loop(speed, measured_speed, p)
    accel_est = longitudinal_accel(speed, measured_speed, p)
    steering = grip_clamp(steer_cmd_delayed * p.steer_gain, measured_speed, accel_est, p)
    sv = servo_velocity(steering, current_steer, p)
    accl = pid_accl(speed, measured_speed, p.a_max, p.gym_v_max, p.gym_v_min)
    return torch.stack([sv, accl], dim=-1)


def wrap_angle(a):
    return torch.remainder(a + math.pi, 2 * math.pi) - math.pi
