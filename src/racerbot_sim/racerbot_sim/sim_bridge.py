"""F1TENTH Gym wrapped for the ROS bridge -- no rclpy in this file.

`tools/f1tenth_sim/run_validation.py` calls the controllers' *math* and
skips ROS entirely. That is the right shape for tuning a control law and
the wrong shape for the thing that broke here: `auto_map_race_launch.py`
failed in the wiring -- SLAM, TF, topic timing, a runtime parameter
handover between two nodes -- none of which that harness can see.

This module supplies the missing half: physics, LiDAR and multi-car
collisions behind a plain step() call, so `gym_bridge_node` can be a thin
ROS shell over it and this part stays unit-testable (test/test_sim_bridge.py).

Deliberately shares the vehicle geometry, LiDAR geometry and integrator
settings with `run_validation.py` rather than inventing a second set of
numbers -- see docs/sim-fidelity-audit.md for how far any of them are
from the real car.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from pathlib import Path
import sys

import numpy as np


def add_sim_python_path(workspace_root=None) -> None:
    """Put the pinned F1TENTH Gym and its deps on sys.path.

    The gym lives under `.sim/` (gitignored, installed by
    tools/f1tenth_sim/setup.sh) rather than in the ROS environment, so a
    ROS node that wants it has to say so explicitly. Idempotent.
    """
    if workspace_root is None:
        workspace_root = os.environ.get('RACERBOT_WS')
    if workspace_root is None:
        # .../src/racerbot_sim/racerbot_sim/sim_bridge.py -> workspace root,
        # or the installed copy, in which case fall back to the env var/cwd.
        here = Path(__file__).resolve()
        for parent in here.parents:
            if (parent / '.sim' / 'f1tenth_gym').is_dir():
                workspace_root = parent
                break
        else:
            workspace_root = Path.cwd()
    root = Path(workspace_root)
    sim_root = root / '.sim'
    for path in (sim_root / 'python', sim_root / 'f1tenth_gym'):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)
    os.environ.setdefault('NUMBA_CACHE_DIR', str(sim_root / 'numba-cache'))


# --- Geometry and timing, shared with tools/f1tenth_sim/run_validation.py ---
WHEELBASE = 0.324
CAR_WIDTH = 0.31
CAR_LENGTH = 0.58
LIDAR_OFFSET_X = 0.33
LIDAR_OFFSET_Z = 0.11
INTEGRATOR_DT = 0.005
# The car's Hokuyo UST-10LX: 1081 beams at 0.25 deg over 270 deg. The older
# harness uses 819; docs/sim-fidelity-audit.md measured the correction as
# free, so this bridge takes the accurate number.
LIDAR_BEAMS = 1081
LIDAR_HALF_FOV = math.radians(135.0)
LIDAR_RANGE_MIN = 0.05
LIDAR_RANGE_MAX = 25.0
LIDAR_NOISE_STD = 0.01


@dataclass
class OpponentPlan:
    """A scripted car: follows the loop at a fixed speed, or parks on it.

    Not a controller under test. Opponents stay dumb and repeatable on
    purpose, so a change in a run's outcome is a change in the ego's
    behavior and not in the traffic it happened to meet.
    """
    start_offset_m: float = 8.0
    speed: float = 1.0
    lateral_offset_m: float = 0.0
    lookahead_m: float = 1.0


@dataclass
class SimConfig:
    track_path: str
    centerline: np.ndarray
    control_dt: float = 0.025
    seed: int = 12345
    ego_start_offset_m: float = 0.0
    opponents: list = field(default_factory=list)
    # Odometry is dead-reckoned exactly like vesc_to_odom does on the car,
    # not handed the true pose: SLAM's whole job is to correct that drift,
    # and a bridge that publishes ground truth on /odom would hide every
    # mapping problem it is supposed to expose.
    odom_speed_scale: float = 1.0
    odom_speed_noise_std: float = 0.01
    odom_yaw_rate_scale: float = 1.0
    scan_dropout_probability: float = 0.0


@dataclass
class AgentState:
    x: float
    y: float
    yaw: float
    speed: float
    yaw_rate: float
    steering: float
    collision: bool


def _closed_arc_lengths(xy: np.ndarray) -> np.ndarray:
    deltas = np.diff(np.vstack([xy, xy[:1]]), axis=0)
    seg = np.hypot(deltas[:, 0], deltas[:, 1])
    return np.concatenate([[0.0], np.cumsum(seg)[:-1]]), float(seg.sum())


class CenterlineFollower:
    """Fixed-speed pure-pursuit around the loop, for scripted opponents."""

    def __init__(self, centerline: np.ndarray, plan: OpponentPlan):
        self.xy = np.asarray(centerline, dtype=np.float64)
        self.arc, self.length = _closed_arc_lengths(self.xy)
        self.plan = plan

    def _point_at(self, s: float) -> np.ndarray:
        s = s % self.length
        index = int(np.searchsorted(self.arc, s, side='right')) - 1
        index = max(0, min(index, len(self.xy) - 1))
        nxt = (index + 1) % len(self.xy)
        span = (self.arc[nxt] if nxt else self.length) - self.arc[index]
        t = 0.0 if span <= 0.0 else (s - self.arc[index]) / span
        return self.xy[index] + t * (self.xy[nxt] - self.xy[index])

    def _offset_point(self, s: float) -> np.ndarray:
        point = self._point_at(s)
        if not self.plan.lateral_offset_m:
            return point
        ahead = self._point_at(s + 0.1)
        heading = ahead - point
        norm = float(np.hypot(*heading))
        if norm < 1e-9:
            return point
        normal = np.array([-heading[1], heading[0]]) / norm
        return point + normal * self.plan.lateral_offset_m

    def start_pose(self) -> np.ndarray:
        s = self.plan.start_offset_m
        here = self._offset_point(s)
        ahead = self._offset_point(s + 0.2)
        return np.array([here[0], here[1], math.atan2(*(ahead - here)[::-1])])

    def command(self, x: float, y: float, yaw: float) -> tuple:
        """(steering, speed) toward a point `lookahead_m` along the loop."""
        if self.plan.speed <= 0.0:
            return 0.0, 0.0
        distances = np.hypot(self.xy[:, 0] - x, self.xy[:, 1] - y)
        nearest = int(np.argmin(distances))
        target = self._offset_point(self.arc[nearest] + self.plan.lookahead_m)
        dx, dy = target[0] - x, target[1] - y
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        distance_sq = local_x * local_x + local_y * local_y
        if distance_sq < 1e-6:
            return 0.0, self.plan.speed
        curvature = 2.0 * local_y / distance_sq
        steering = math.atan(curvature * WHEELBASE)
        return float(np.clip(steering, -0.4, 0.4)), self.plan.speed


class DeadReckonedOdometry:
    """What `vesc_to_odom` publishes: integrated wheel speed + servo angle.

    Mirrors vesc_to_odom.cpp with `use_servo_cmd_to_calc_angular_velocity`
    on -- yaw rate comes from the *commanded* steering angle and the
    measured speed, never from a gyro -- so the drift SLAM has to absorb is
    the same shape as the car's.
    """

    def __init__(self, x: float, y: float, yaw: float, config: SimConfig, rng):
        self.x, self.y, self.yaw = x, y, yaw
        self.speed = 0.0
        self.yaw_rate = 0.0
        self._config = config
        self._rng = rng

    def update(self, true_speed: float, commanded_steering: float, dt: float):
        speed = true_speed * self._config.odom_speed_scale
        if self._config.odom_speed_noise_std > 0.0:
            speed += self._rng.normal(0.0, self._config.odom_speed_noise_std)
        yaw_rate = (speed * math.tan(commanded_steering) / WHEELBASE
                    * self._config.odom_yaw_rate_scale)
        self.x += speed * math.cos(self.yaw) * dt
        self.y += speed * math.sin(self.yaw) * dt
        self.yaw = (self.yaw + yaw_rate * dt + math.pi) % (2.0 * math.pi) - math.pi
        self.speed = speed
        self.yaw_rate = yaw_rate


class SimBridge:
    """One ego car plus zero or more scripted opponents on a closed loop."""

    def __init__(self, config: SimConfig):
        add_sim_python_path()
        import gymnasium as gym
        import f1tenth_gym  # noqa: F401  registers the environment
        from f1tenth_gym.envs.dynamic_models import (
            DynamicModel, F1TENTH_VEHICLE_PARAMETERS)
        from f1tenth_gym.envs.env_config import (
            ControlConfig, EnvConfig, ObservationConfig, SimulationConfig)
        from f1tenth_gym.envs.integrators import IntegratorType
        from f1tenth_gym.envs.lidar import LiDARConfig
        from f1tenth_gym.envs.observation import ObservationType

        self.config = config
        self.centerline = np.asarray(config.centerline, dtype=np.float64)
        self.followers = [CenterlineFollower(self.centerline, plan)
                          for plan in config.opponents]
        self.num_agents = 1 + len(self.followers)
        self._rng = np.random.default_rng(config.seed)

        self.lidar_config = LiDARConfig(
            enabled=True,
            num_beams=LIDAR_BEAMS,
            angle_min=-LIDAR_HALF_FOV,
            angle_max=LIDAR_HALF_FOV,
            range_min=LIDAR_RANGE_MIN,
            range_max=LIDAR_RANGE_MAX,
            noise_std=LIDAR_NOISE_STD,
            base_link_to_lidar_tf=(LIDAR_OFFSET_X, 0.0, 0.0),
        )
        params = F1TENTH_VEHICLE_PARAMETERS.with_updates(
            lf=WHEELBASE / 2.0, lr=WHEELBASE / 2.0,
            width=CAR_WIDTH, length=CAR_LENGTH,
            collision_body_center_x=WHEELBASE / 2.0,
        )
        env_config = EnvConfig(
            seed=config.seed,
            map_name=str(config.track_path),
            params=params,
            num_agents=self.num_agents,
            control_config=ControlConfig(steer_delay_steps=1),
            simulation_config=SimulationConfig(
                timestep=config.control_dt,
                integrator_timestep=INTEGRATOR_DT,
                integrator=IntegratorType.RK4,
                dynamics_model=DynamicModel.ST,
                max_laps=10_000,
            ),
            observation_config=ObservationConfig(type=ObservationType.DIRECT),
            lidar_config=self.lidar_config,
            render_enabled=False,
        )
        self.env = gym.make('f1tenth_gym:f1tenth-v0', config=env_config,
                            render_mode=None)

        # Body-vs-wall contact is checked here rather than trusted to the
        # environment. In this pinned Gym revision the wall check is
        # `raw_scan - side_distances <= 0.005`, and with the vehicle
        # parameters this workspace uses (`collision_body_center_x =
        # wheelbase/2` against the ST model's own -lr offset) the LiDAR
        # lands 0.04m *outside* the collision rectangle, so
        # `side_distances` comes out all zeros and the test reduces to
        # "did a beam return less than 5mm" -- which it cannot, because
        # range_min is 0.05m. Wall collisions therefore never fire. A
        # simulator that cannot notice the car driving through a wall is
        # no use for validating a mapping run, so the padded body is
        # sampled against the occupancy grid directly below.
        track = self.env.unwrapped.track
        self._occupancy = np.asarray(track.occupancy_map)
        self._map_resolution = float(track.spec.resolution)
        self._map_origin = (float(track.spec.origin[0]), float(track.spec.origin[1]))
        # Body-local sample points, dense enough that no sample can straddle
        # a one-cell wall.
        step = max(1, int(math.ceil(CAR_LENGTH / self._map_resolution)))
        lateral = max(1, int(math.ceil(CAR_WIDTH / self._map_resolution)))
        us = np.linspace(-CAR_LENGTH / 2.0, CAR_LENGTH / 2.0, step + 1)
        vs = np.linspace(-CAR_WIDTH / 2.0, CAR_WIDTH / 2.0, lateral + 1)
        grid_u, grid_v = np.meshgrid(us, vs)
        self._body_samples = np.stack([grid_u.ravel(), grid_v.ravel()], axis=1)

        self.sim_time = 0.0
        self.steps = 0
        self._obs = None
        self._last_ego_steering = 0.0
        self.odom = None
        self.reset()

    # -- lifecycle ---------------------------------------------------------

    def _pose_on_loop(self, offset_m: float) -> np.ndarray:
        follower = CenterlineFollower(self.centerline,
                                      OpponentPlan(start_offset_m=offset_m))
        return follower.start_pose()

    def reset(self):
        poses = [self._pose_on_loop(self.config.ego_start_offset_m)]
        poses += [follower.start_pose() for follower in self.followers]
        self._obs, _ = self.env.reset(
            options={'poses': np.asarray(poses, dtype=float)})
        # The environment zero-fills every scan until the first step, and a
        # zeroed LaserScan reads as "obstacle at 0m in every direction" to
        # every safety layer downstream. Take one stationary step so the
        # first scan the bridge can publish is a real one.
        self._obs, _reward, _done, _truncated, _info = self.env.step(
            np.zeros((self.num_agents, 2), dtype=np.float32))
        self.sim_time = 0.0
        self.steps = 0
        self._last_ego_steering = 0.0
        ego = self.agent_state(0)
        self.odom = DeadReckonedOdometry(ego.x, ego.y, ego.yaw,
                                         self.config, self._rng)
        return self._obs

    def close(self):
        self.env.close()

    # -- accessors ---------------------------------------------------------

    def agent_state(self, index: int) -> AgentState:
        agent = self._obs[f'agent_{index}']
        state = np.asarray(agent['std_state'], dtype=float)
        return AgentState(
            x=float(state[0]), y=float(state[1]), yaw=float(state[4]),
            speed=float(state[3]), yaw_rate=float(state[5]),
            steering=float(state[2]), collision=bool(agent['collision']),
        )

    def ego_scan(self) -> np.ndarray:
        scan = np.asarray(self._obs['agent_0']['scan'], dtype=np.float64)
        if self.config.scan_dropout_probability > 0.0:
            mask = self._rng.random(scan.shape) < self.config.scan_dropout_probability
            scan = np.where(mask, np.inf, scan)
        return scan

    @property
    def beam_angles(self) -> np.ndarray:
        return np.linspace(-LIDAR_HALF_FOV, LIDAR_HALF_FOV, LIDAR_BEAMS)

    def occupied(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Occupancy lookup in world coordinates; off-map counts as occupied.

        Off-map is a wall on purpose: the generated rooms are closed, so a
        car outside the image has left the world rather than found space.
        """
        cols = np.floor((np.asarray(xs) - self._map_origin[0]) / self._map_resolution)
        rows = np.floor((np.asarray(ys) - self._map_origin[1]) / self._map_resolution)
        height, width = self._occupancy.shape
        inside = ((cols >= 0) & (cols < width) & (rows >= 0) & (rows < height))
        result = np.ones(np.shape(xs), dtype=bool)
        if np.any(inside):
            r = rows[inside].astype(int)
            c = cols[inside].astype(int)
            result[inside] = self._occupancy[r, c] <= 128
        return result

    def body_contact(self, index: int = 0) -> bool:
        """True when any part of the padded body is in an occupied cell."""
        state = self.agent_state(index)
        cos_yaw, sin_yaw = math.cos(state.yaw), math.sin(state.yaw)
        # The collision rectangle is centred on base_link for this workspace's
        # vehicle parameters -- see the note in __init__.
        xs = state.x + self._body_samples[:, 0] * cos_yaw - self._body_samples[:, 1] * sin_yaw
        ys = state.y + self._body_samples[:, 0] * sin_yaw + self._body_samples[:, 1] * cos_yaw
        return bool(self.occupied(xs, ys).any())

    def _body_corners(self, index: int) -> np.ndarray:
        state = self.agent_state(index)
        cos_yaw, sin_yaw = math.cos(state.yaw), math.sin(state.yaw)
        half_length, half_width = CAR_LENGTH / 2.0, CAR_WIDTH / 2.0
        local = np.array([(+half_length, +half_width), (+half_length, -half_width),
                          (-half_length, -half_width), (-half_length, +half_width)])
        return np.column_stack([
            state.x + local[:, 0] * cos_yaw - local[:, 1] * sin_yaw,
            state.y + local[:, 0] * sin_yaw + local[:, 1] * cos_yaw,
        ])

    def car_contact(self) -> bool:
        """True when any two cars' padded bodies overlap.

        Separate from `body_contact`, which only ever tests a body against
        the *map*. Without this, an ego wedged against a parked opponent
        reports no contact at all -- which is how a validation run of the
        static-obstacle scenario passed while the car spent its entire
        racing phase stopped with its nose against the other car.
        """
        for first in range(self.num_agents):
            for second in range(first + 1, self.num_agents):
                if _rectangles_overlap(self._body_corners(first),
                                       self._body_corners(second)):
                    return True
        return False

    def any_collision(self) -> bool:
        """Any car touching a wall or another car, by any available test."""
        return (self.car_contact()
                or any(self.agent_state(i).collision or self.body_contact(i)
                       for i in range(self.num_agents)))

    # -- stepping ----------------------------------------------------------

    def step(self, ego_steering: float, ego_speed: float):
        """Advance one control period with the ego's latest drive command."""
        actions = [[float(ego_steering), float(ego_speed)]]
        for index, follower in enumerate(self.followers, start=1):
            state = self.agent_state(index)
            steering, speed = follower.command(state.x, state.y, state.yaw)
            actions.append([steering, speed])

        self._obs, _reward, _done, _truncated, _info = self.env.step(
            np.asarray(actions, dtype=np.float32))
        self.sim_time += self.config.control_dt
        self.steps += 1

        ego = self.agent_state(0)
        self._last_ego_steering = float(ego_steering)
        self.odom.update(ego.speed, float(ego_steering), self.config.control_dt)
        return self._obs


def _rectangles_overlap(first: np.ndarray, second: np.ndarray) -> bool:
    """Separating-axis test for two convex polygons given as corner arrays."""
    for polygon in (first, second):
        edges = np.roll(polygon, -1, axis=0) - polygon
        for edge in edges:
            axis = np.array([-edge[1], edge[0]])
            norm = float(np.hypot(*axis))
            if norm < 1e-12:
                continue
            axis = axis / norm
            a = first @ axis
            b = second @ axis
            if a.max() < b.min() or b.max() < a.min():
                return False
    return True
