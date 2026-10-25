"""The batched simulator: trajectory parity with the CPU referee, LiDAR, contact,
IMU, race bookkeeping, and the isolation of one env's reset from the others.

Runner: PYTHONPATH=<torch> python3 -m pytest tools/neural_racer/tests/test_sim.py -v   (CPU)

Oracles: FidelityPlant itself (trajectory), a brute-force ray march on the
same occupancy grid (LiDAR), closed-form ray/box and turning-circle
kinematics, gym's own separating-axis collision() (contact).
"""
import math

import conftest  # noqa: F401
import numpy as np
import pytest
import torch

from neural_racer_train import fidelity, tracks
from neural_racer_train.gpu_sim import contact, imu, lidar, race
from neural_racer_train.gpu_sim.params import CarParams
from neural_racer_train.gpu_sim.sim import BatchedRaceSim, Level
from neural_racer_train.track_pool import TrackPool

torch.set_num_threads(4)


def _ring_spec(radius=12.0, half_width=4.0):
    t = np.linspace(0, 2 * math.pi, int(2 * math.pi * radius / tracks.SPACING), endpoint=False)
    c = np.stack([radius * np.cos(t), radius * np.sin(t)], axis=1)
    w = np.full(len(c), half_width)
    return tracks.TrackSpec('ring', 0, c, w, w)


@pytest.fixture(scope='module')
def arena():
    """A ring 30 m wide: open space for physics tests, where no command
    sequence below can reach a wall (walls stop our car, not the plant)."""
    spec = _ring_spec(radius=20.0, half_width=15.0)
    r = tracks.rasterize(spec)
    pool = TrackPool(1, canvas=r.sdf.shape, max_points=len(spec.centre) + 1, dtype=torch.float32)
    pool.insert(r, 0)
    return spec, r, pool


@pytest.fixture(scope='module')
def ring():
    spec = _ring_spec()
    r = tracks.rasterize(spec)
    pool = TrackPool(1, canvas=r.sdf.shape, max_points=len(spec.centre) + 1, dtype=torch.float32)
    pool.insert(r, 0)
    return spec, r, pool


# --- trajectory parity with the referee ----------------------------------------------------

@pytest.mark.skipif(not fidelity.gym_available(), reason='needs the pinned gym: tools/f1tenth_sim/setup.sh')
def test_trajectory_matches_fidelity_plant(arena):
    """10 s of steering and speed commands through FidelityPlant ('car' profile)
    and through this sim, from the same start: same ST state within 2 cm and
    0.5 deg (plan, Phase 3). Physics only: walls affect neither."""
    fidelity.ensure_paths()
    import run_validation as rv
    cal = rv.CALIBRATION
    plant = rv.make_plant('Spielberg', 1, 0)
    start = np.array([[0.0, 0.0, 0.3]])
    plant.reset(start)
    spec, _, pool = arena
    p = CarParams(steering_max_left=cal.steering_max_left, steering_max_right=cal.steering_max_right,
                  speed_max=cal.speed_max, speed_min=cal.speed_min)
    sim = BatchedRaceSim(1, 1, pool, car=p, randomize=False, wrapper_cfg=_open_wrapper())
    # put our CoG at the ring's edge point, heading along it; compare displacements
    k = 0
    pose = np.array([[spec.centre[k, 0] - p.lr * math.cos(math.pi / 2), spec.centre[k, 1] - p.lr * math.sin(math.pi / 2), math.pi / 2]])
    # v_cap high: the parity test compares raw physics, not the speed ladder
    sim.reset_envs([0], [Level(slot=0, poses=pose, start_idx=np.array([k]), v_cap=10.0)])
    x0p = np.array(plant._truth[0], dtype=np.float64)
    x0s = sim.x[0, 0].double().numpy().copy()
    worst_pos = worst_yaw = 0.0
    for tick in range(400):
        t = tick * 0.025
        steer = 0.10 + 0.20 * math.sin(2 * math.pi * t / 4.0)
        speed = min(0.6 * t, 3.0)
        plant.step([(steer, speed)])
        sim.step(torch.tensor([[[steer, speed]]], dtype=torch.float32))
        xp = np.array(plant._truth[0], dtype=np.float64)
        xs = sim.x[0, 0].double().numpy()
        # rotate our displacement into the plant's start heading
        rot = x0p[4] - x0s[4]
        c, s = math.cos(rot), math.sin(rot)
        d = xs[:2] - x0s[:2]
        ds = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
        worst_pos = max(worst_pos, float(np.hypot(*(ds - (xp[:2] - x0p[:2])))))
        dyaw = (xs[4] - x0s[4]) - (xp[4] - x0p[4])
        worst_yaw = max(worst_yaw, abs(math.degrees(math.atan2(math.sin(dyaw), math.cos(dyaw)))))
    assert bool(sim.alive[0, 0]), 'our car hit a wall: the parity comparison is invalid'
    assert worst_pos < 0.02, f'position diverged by {worst_pos:.4f} m'
    assert worst_yaw < 0.5, f'heading diverged by {worst_yaw:.3f} deg'


def _open_wrapper():
    """A wrapper that never caps (the parity test compares raw physics)."""
    from neural_racer.safety_wrapper import WrapperConfig
    return WrapperConfig(max_speed=10.0, max_steering_angle=1.0, max_steering_rate=1e6,
                         max_lateral_accel=1e6, max_braking_decel=1e6, reserve_distance=0.0,
                         braking_latency_sec=0.0, max_invalid_fraction=1.0)


# --- LiDAR -----------------------------------------------------------------------------------

def _brute_ray(r, x, y, a, limit, step=0.005):
    t = 0.0
    while t < limit:
        px, py = x + t * math.cos(a), y + t * math.sin(a)
        col = int((px - r.origin[0]) / r.resolution)
        row = int((py - r.origin[1]) / r.resolution)
        if not (0 <= row < r.occupancy.shape[0] and 0 <= col < r.occupancy.shape[1]) or r.occupancy[row, col]:
            return t
        t += step
    return limit


def test_scan_matches_brute_force(ring):
    """Sphere tracing vs an independent 5 mm ray march on the same occupancy grid.

    Criterion, measured perpendicular to the wall (the ring's walls are
    circles, so the normal is radial): every beam within one cell (5 cm), and
    no systematic bias over 1 cm. Along the beam the two representations --
    the grid's staircase and the smooth SDF -- legitimately differ by
    (perpendicular error) / sin(incidence), which is why the along-beam error
    is not the criterion (measured 2026-10-23: misses only at 18-44 deg).
    """
    spec, r, pool = ring
    rng = np.random.default_rng(0)
    perp, signed = [], []
    for _ in range(6):
        k = int(rng.integers(len(spec.centre)))
        off = rng.uniform(-2.5, 2.5)
        n = tracks.normals(spec.centre)[k]
        x, y = spec.centre[k] + n * off
        yaw = rng.uniform(-math.pi, math.pi)
        angles = lidar.beam_angles(181)
        got = lidar.cast(pool, torch.tensor([0]), torch.tensor([[x]], dtype=torch.float32),
                         torch.tensor([[y]], dtype=torch.float32), torch.tensor([[yaw]], dtype=torch.float32),
                         angles, cast_limit=12.0)[0, 0].numpy()
        for a, g in zip(angles.numpy(), got):
            beam = yaw + float(a)
            w = _brute_ray(r, x, y, beam, 12.0)
            if w >= 12.0:
                continue
            hx, hy = x + w * math.cos(beam), y + w * math.sin(beam)
            sin_inc = abs(math.cos(beam - math.atan2(hy, hx)))     # beam vs the radial wall normal
            perp.append(abs(g - w) * sin_inc)
            signed.append(g - w)
    perp = np.array(perp)
    assert len(perp) > 500           # non-vacuous: 798 of 1086 beams hit a wall within 12 m here
    assert perp.max() <= 0.05, f'worst perpendicular error {perp.max():.3f} m'
    assert abs(float(np.mean(signed))) <= 0.01, f'bias {np.mean(signed):+.4f} m'


def test_ray_box_closed_form():
    """Ray from the origin along +x at an axis-aligned box centred at (3, 0),
    half sizes (0.25, 0.15): hits at 2.75. Rotated 90 deg: half-length 0.15 -> 2.85."""
    t = lidar.ray_box(*(torch.tensor(v) for v in (0.0, 0.0, 1.0, 0.0, 3.0, 0.0, 0.0, 0.25, 0.15)))
    assert float(t) == pytest.approx(2.75, abs=1e-6)
    t = lidar.ray_box(*(torch.tensor(v) for v in (0.0, 0.0, 1.0, 0.0, 3.0, 0.0, math.pi / 2, 0.25, 0.15)))
    assert float(t) == pytest.approx(2.85, abs=1e-6)
    t = lidar.ray_box(*(torch.tensor(v) for v in (0.0, 0.0, -1.0, 0.0, 3.0, 0.0, 0.0, 0.25, 0.15)))
    assert math.isinf(float(t))                              # pointing away: a miss


def test_other_cars_appear_in_the_scan_but_not_yourself(ring):
    spec, _, pool = ring
    sim = BatchedRaceSim(1, 2, pool, randomize=False)
    k = 0
    tangent = math.pi / 2
    p0 = spec.centre[k]
    poses = np.array([[p0[0], p0[1], tangent], [p0[0], p0[1] + 2.0, tangent]])   # second car 2 m ahead
    sim.reset_envs([0], [Level(slot=0, poses=poses, start_idx=np.array([k, k + 20]))])
    r = sim.scan_now[0]
    centre_beam = 540
    # car 0's LiDAR is 0.26 m ahead of its rear axle; car 1's body centre is
    # lr (0.18) ahead of its rear axle at +2.0 m, rear face 0.535/2 behind that
    expected = 2.0 + 0.18 - 0.535 / 2 - 0.26
    assert float(r[0, centre_beam]) == pytest.approx(expected, abs=0.05)
    assert float(r[1, 0]) > 0.3                              # car 1 never sees its own body


# --- contact ----------------------------------------------------------------------------------

@pytest.mark.skipif(not fidelity.gym_available(), reason='needs the pinned gym: tools/f1tenth_sim/setup.sh')
def test_car_overlap_matches_gym_sat():
    fidelity.ensure_paths()
    from f1tenth_gym.envs.collision_models import collision, get_vertices
    rng = np.random.default_rng(1)
    n = 2000
    a = np.column_stack([np.zeros(n), np.zeros(n), rng.uniform(-math.pi, math.pi, n)])
    b = np.column_stack([rng.uniform(-0.7, 0.7, n), rng.uniform(-0.7, 0.7, n), rng.uniform(-math.pi, math.pi, n)])
    L, W = 0.535, 0.30
    want = np.array([collision(get_vertices(a[i], L, W), get_vertices(b[i], L, W)) for i in range(n)])
    T = lambda v: torch.tensor(v, dtype=torch.float64)   # noqa: E731
    ka = contact.corners(T(a[:, 0]), T(a[:, 1]), T(a[:, 2]), T(np.full(n, L / 2)), T(np.full(n, W / 2)))
    kb = contact.corners(T(b[:, 0]), T(b[:, 1]), T(b[:, 2]), T(np.full(n, L / 2)), T(np.full(n, W / 2)))
    got = contact.rect_overlap(ka, kb).numpy()
    assert 0.2 < want.mean() < 0.8                           # both outcomes well represented
    assert np.array_equal(got, want)


def test_fault_rear_ender():
    """A at 2 m/s drives into B's rear (B at 0.5 m/s): A at fault, B not."""
    T = lambda *v: torch.tensor([list(v)], dtype=torch.float32)   # noqa: E731
    cx, cy, yaw = T(0.0, 0.5), T(0.0, 0.0), T(0.0, 0.0)
    vx, vy = T(2.0, 0.5), T(0.0, 0.0)
    hx, hy = T(0.2675, 0.2675), T(0.15, 0.15)
    touch, fault = contact.car_contacts(cx, cy, yaw, vx, vy, hx, hy, torch.tensor([[True, True]]))
    assert touch.tolist() == [[True, True]] and fault.tolist() == [[True, False]]
    # B reversing into a slow A: B closes at 2 m/s, A at 0.5 -> B at fault
    touch, fault = contact.car_contacts(cx, cy, yaw, T(0.5, -2.0), vy, hx, hy, torch.tensor([[True, True]]))
    assert fault.tolist() == [[False, True]]


def test_wall_contact_on_the_ring(ring):
    spec, _, pool = ring
    T = lambda v: torch.tensor([[v]], dtype=torch.float32)   # noqa: E731
    c = spec.centre[0]
    on_line = contact.wall_contact(pool, torch.tensor([0]), T(c[0]), T(c[1]), T(math.pi / 2), T(0.27), T(0.15))
    near_wall = contact.wall_contact(pool, torch.tensor([0]), T(c[0] + 3.95), T(c[1]), T(math.pi / 2), T(0.27), T(0.15))
    assert not bool(on_line) and bool(near_wall)


# --- IMU -------------------------------------------------------------------------------------------

def test_imu_steady_turn_closed_form():
    """Steady turn, constant V, beta, w: a_y at the CoG = w * v_x, and a point
    d ahead of the CoG reads a_x = -w v_y - w^2 d."""
    V, beta, w, d = 2.0, 0.05, 1.5, 0.27
    x = torch.tensor([[0, 0, 0.2, V, 0, w, beta]], dtype=torch.float64)
    f = torch.tensor([[0, 0, 0, 0.0, w, 0.0, 0.0]], dtype=torch.float64)
    gz, ax, ay = imu.specific_force(x, f, torch.tensor(d, dtype=torch.float64))
    vx, vy = V * math.cos(beta), V * math.sin(beta)
    assert float(gz) == pytest.approx(w)
    assert float(ay) == pytest.approx(w * vx, abs=1e-12)
    assert float(ax) == pytest.approx(-w * vy - w * w * d, abs=1e-12)


def test_imu_straight_acceleration():
    x = torch.tensor([[0, 0, 0.0, 1.0, 0, 0.0, 0.0]], dtype=torch.float64)
    f = torch.tensor([[0, 0, 0, 2.5, 0.0, 0.0, 0.0]], dtype=torch.float64)
    gz, ax, ay = imu.specific_force(x, f, torch.tensor(0.27, dtype=torch.float64))
    assert (float(gz), float(ax), float(ay)) == pytest.approx((0.0, 2.5, 0.0), abs=1e-12)


def test_sim_imu_reads_the_turn(arena):
    """Constant command on the ring's open space: after settling, the IMU yaw
    rate matches the state's, and a_y at the CoG point ~ V * w."""
    spec, _, pool = arena
    p = CarParams(imu_gyro_noise=0.0, imu_accel_noise=0.0, imu_gyro_bias=0.0, imu_offset_x=CarParams().lr)
    sim = BatchedRaceSim(1, 1, pool, car=p, randomize=False, wrapper_cfg=_open_wrapper())
    c = spec.centre[0]
    sim.reset_envs([0], [Level(slot=0, poses=np.array([[c[0] - 2.0, c[1], math.pi / 2]]), start_idx=np.array([0]))])
    for _ in range(160):
        sim.step(torch.tensor([[[0.2, 1.5]]]))
    assert bool(sim.alive[0, 0])
    w_state = float(sim.x[0, 0, 5])
    assert w_state > 0.3                                     # really turning
    gz, ax, ay = (float(v) for v in sim.imu_held[0, 0])
    assert gz == pytest.approx(w_state, rel=0.05)
    assert ay == pytest.approx(float(sim.x[0, 0, 3]) * w_state, rel=0.10)


# --- race bookkeeping -----------------------------------------------------------------------------------

def test_progress_wraps_at_the_seam_and_counts_laps():
    rs = race.RaceState(1, 1)
    L = torch.tensor([10.0])
    rs.reset(torch.tensor([True]), torch.zeros(1, 1, dtype=torch.long), torch.tensor([[9.0]]))
    on = torch.tensor([[True]])
    total = 0.0
    for s in (9.5, 0.2, 1.0, 9.9, 0.1):       # crosses the seam twice going forward... and back
        ds, _ = rs.update(torch.zeros(1, 1, dtype=torch.long), torch.tensor([[s]]), L, on, on)
        total += float(ds)
    assert total == pytest.approx(float(rs.total), abs=1e-6)
    assert float(rs.total) == pytest.approx(0.5 + 0.7 + 0.8 - 1.1 + 0.2, abs=1e-5)


def test_place_hysteresis():
    rs = race.RaceState(1, 2)
    L = torch.tensor([100.0])
    on = torch.tensor([[True, True]])
    rs.reset(torch.tensor([True]), torch.zeros(1, 2, dtype=torch.long), torch.tensor([[0.0, 0.0]]))
    _, g = rs.update(torch.zeros(1, 2, dtype=torch.long), torch.tensor([[0.2, 0.0]]), L, on, on)
    assert g.tolist() == [[0, 0]]                  # 0.2 m ahead: inside the 0.3 m hysteresis
    _, g = rs.update(torch.zeros(1, 2, dtype=torch.long), torch.tensor([[0.5, 0.0]]), L, on, on)
    assert g.tolist() == [[1, 0]] and rs.place(on).tolist() == [[1, 2]]
    _, g = rs.update(torch.zeros(1, 2, dtype=torch.long), torch.tensor([[0.5, 0.4]]), L, on, on)
    assert g.tolist() == [[0, 0]]                  # still 0.1 m ahead: no flicker


def test_passing_a_crashed_car_gains_a_place():
    rs = race.RaceState(1, 2)
    L = torch.tensor([100.0])
    rs.reset(torch.tensor([True]), torch.zeros(1, 2, dtype=torch.long), torch.tensor([[0.0, 5.0]]))
    both = torch.tensor([[True, True]])
    rs.update(torch.zeros(1, 2, dtype=torch.long), torch.tensor([[0.0, 5.0]]), L, both, both)
    moving = torch.tensor([[True, False]])          # car 1 crashed, still in the race
    _, g = rs.update(torch.zeros(1, 2, dtype=torch.long), torch.tensor([[6.0, 5.0]]), L, moving, both)
    assert g.tolist() == [[1, 0]]


# --- isolation ------------------------------------------------------------------------------------------

def test_resetting_one_env_does_not_disturb_another(arena):
    """Regression for a bug found in review: resets used to advance the shared
    IMU delay line for every env. Noise is off, because the envs share one
    noise stream and a reset legitimately draws from it."""
    spec, _, pool = arena
    c = spec.centre[0]
    lvl = Level(slot=0, poses=np.array([[c[0] - 2.0, c[1], math.pi / 2]]), start_idx=np.array([0]))
    quiet = CarParams(imu_gyro_noise=0.0, imu_accel_noise=0.0, imu_gyro_bias=0.0, odom_speed_noise=0.0,
                      lidar_noise_std=0.0, lidar_dropout_rate=0.0, imu_delay_ticks=2)
    a = BatchedRaceSim(2, 1, pool, car=quiet, randomize=False, seed=3)
    b = BatchedRaceSim(2, 1, pool, car=quiet, randomize=False, seed=3)
    a.reset_envs([0, 1], [lvl, lvl])
    b.reset_envs([0, 1], [lvl, lvl])
    for k in range(30):
        act = torch.tensor([[[0.15, 1.0]], [[0.15, 1.0]]])
        a.step(act)
        b.step(act)
        if k == 10:
            b.reset_envs([0], [lvl])                # only env 0 restarts in b
    assert torch.equal(a.x[1], b.x[1])
    assert torch.equal(a.hist.imu[1], b.hist.imu[1]) and torch.equal(a.imu_delay.buf[:, 1], b.imu_delay.buf[:, 1])
