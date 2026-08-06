"""The fidelity layer that sits between the controllers and the gym.

Everything the audit found wrong with the simulated vehicle is either a
parameter (fixed in :mod:`racerbot_sim.calibration`) or something that has to
happen on the way in or out of ``env.step()``. This module is that path.

Per control tick, for each agent:

1. the controller's steering and speed commands are held for the transport
   lag between publishing a command and the VESC acting on it (R7);
2. the steering angle is clamped to the friction circle at the current speed
   and braking effort (R4, F2/F4);
3. a proportional servo turns the clamped angle into a steering velocity,
   replacing upstream's bang-bang actuator (R3, F6);
4. the plant steps;
5. pose, speed and scan are degraded before the controllers see them (R5,
   F7/F8/F9), while ground truth is kept aside for the metrics -- otherwise
   sensor noise would corrupt the measurement of its own effect.

Profiles let a run pick how much of this to apply. ``legacy`` reproduces the
original harness exactly, so ``docs/f1tenth-sim-results.json`` stays
reproducible; ``car`` is the honest configuration.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .actuation import SteeringServo, TransportDelay
from .calibration import CALIBRATION, CarCalibration
from .grip import GripEnvelope, longitudinal_accel
from .sensing import OdomSensor, PoseSensor, ScanSensor


@dataclass(frozen=True)
class FidelityProfile:
    """Which fidelity fixes a run applies.

    Attributes:
        name: profile name, recorded in the results.
        calibrated_vehicle: use this car's parameters instead of gym stock.
        real_beam_count: 1081 beams, matching the Hokuyo UST-10LX.
        servo_model: proportional steering servo instead of bang-bang.
        transport_delay: model command lag to the actuator.
        grip_envelope: enforce a friction circle.
        sensor_error: degrade pose, speed and scan.
        body_collisions: detect agent contact by real-chassis overlap, all
            round the car, instead of only where the LiDAR can see.
    """

    name: str
    calibrated_vehicle: bool
    real_beam_count: bool
    servo_model: bool
    transport_delay: bool
    grip_envelope: bool
    sensor_error: bool
    body_collisions: bool


PROFILES: dict[str, FidelityProfile] = {
    # Bit-for-bit the harness as it was before the fidelity work. Keep this
    # working: it is what the checked-in results were produced with.
    "legacy": FidelityProfile(
        name="legacy",
        calibrated_vehicle=False,
        real_beam_count=False,
        servo_model=False,
        transport_delay=False,
        grip_envelope=False,
        sensor_error=False,
        body_collisions=False,
    ),
    # Everything that is a property of the vehicle, with perfect sensing.
    # Useful for telling a physics regression apart from a sensing one.
    "plant": FidelityProfile(
        name="plant",
        calibrated_vehicle=True,
        real_beam_count=True,
        servo_model=True,
        transport_delay=True,
        grip_envelope=True,
        sensor_error=False,
        body_collisions=True,
    ),
    # This car, as closely as the harness can represent it.
    "car": FidelityProfile(
        name="car",
        calibrated_vehicle=True,
        real_beam_count=True,
        servo_model=True,
        transport_delay=True,
        grip_envelope=True,
        sensor_error=True,
        body_collisions=True,
    ),
}

DEFAULT_PROFILE = "car"


def env_components(
    profile: FidelityProfile,
    calibration: CarCalibration = CALIBRATION,
    *,
    legacy_beams: int = 819,
    legacy_wheelbase: float = 0.324,
):
    """Build the gym config pieces this profile needs.

    Returns a dict with ``params``, ``control_config``, ``lidar_config`` and
    ``collision_check``, ready to drop into an ``EnvConfig``.
    """
    from f1tenth_gym.envs.action import SteerActionType
    from f1tenth_gym.envs.collision_models import CollisionCheckMode
    from f1tenth_gym.envs.dynamic_models import F1TENTH_VEHICLE_PARAMETERS
    from f1tenth_gym.envs.env_config import ControlConfig
    from f1tenth_gym.envs.lidar import LiDARConfig

    if profile.calibrated_vehicle:
        params = calibration.vehicle_parameters(F1TENTH_VEHICLE_PARAMETERS)
    else:
        # The original harness forced the centre of gravity to the wheelbase
        # midpoint, discarding the stock 48/52 split along with the stock
        # wheelbase it belonged to.
        params = F1TENTH_VEHICLE_PARAMETERS.with_updates(
            lf=legacy_wheelbase / 2.0,
            lr=legacy_wheelbase / 2.0,
            width=calibration.width,
            length=calibration.length,
            collision_body_center_x=legacy_wheelbase / 2.0,
        )

    beams = calibration.lidar_beams if profile.real_beam_count else legacy_beams
    half_fov = calibration.lidar_fov / 2.0
    lidar_config = LiDARConfig(
        enabled=True,
        num_beams=beams,
        angle_min=-half_fov,
        angle_max=half_fov,
        range_min=0.05,
        range_max=calibration.lidar_range_max,
        noise_std=calibration.lidar_noise_std,
        base_link_to_lidar_tf=(calibration.lidar_offset_x, 0.0, 0.0),
    )

    if profile.servo_model:
        # The actuator model lives in actuation.py, so the plant is handed a
        # steering velocity directly and upstream's pid_steer is bypassed.
        # Transport lag is applied to the command, ahead of the servo, rather
        # than to the servo's output as steer_delay_steps would.
        control_config = ControlConfig(
            steering_mode=SteerActionType.STEERING_SPEED,
            steer_delay_steps=0,
        )
    else:
        control_config = ControlConfig(steer_delay_steps=1)

    return {
        "params": params,
        "control_config": control_config,
        "lidar_config": lidar_config,
        # Deliberately always LIDAR_SCAN. Gym's BOUNDING_BOX mode does check
        # agent contact all round the car -- which is the F10 gap -- but it
        # does so with vehicle_params.width/length, the padded safety
        # envelope. Applied to both cars that double-counts the padding and
        # flags contact with 7 mm of real air still between the chassis,
        # measured during an overtake on this track. FidelityPlant runs the
        # same overlap test against the real chassis instead.
        "collision_check": CollisionCheckMode.LIDAR_SCAN,
    }


class FidelityPlant:
    """Wraps a gym env so the controllers meet this car instead of gym's.

    It also supplies the collision detection, because upstream's does not
    work in this harness's geometry. ``check_ttc_jit`` compares each beam
    against ``side_distances``, the distance from the LiDAR to the car's own
    outline -- but that array is computed by intersecting each ray with the
    collision rectangle *from the LiDAR's position inside it*, and this car's
    LiDAR sits 0.33 m forward of base_link while the 0.58 m collision box is
    centred there, spanning only +/-0.29 m. The sensor is outside its own
    collision body, no intersection is found, and the helper returns 0.0 for
    every beam. The test then degenerates to "is any beam below 0.005 m",
    which ``ScanSimulator2D.scan`` makes impossible by clipping every range
    to ``range_min`` = 0.05 m.

    Measured: a car driven straight into the Spielberg barrier travelled
    35.5 m through it, reaching 0.058 m from a wall, and was never flagged.
    Every ``"collision": false`` this harness has ever reported was therefore
    true by construction. ``_wall_contact`` and ``_chassis_contact`` replace
    it with real geometry against the real chassis.
    """

    def __init__(
        self,
        env,
        profile: FidelityProfile,
        num_agents: int,
        seed: int,
        control_dt: float,
        calibration: CarCalibration = CALIBRATION,
        degrade_agents: frozenset[int] = frozenset({0}),
    ):
        self.env = env
        self.profile = profile
        self.calibration = calibration
        self.num_agents = num_agents
        self.control_dt = control_dt
        self.params = env.unwrapped.sim.vehicle_params
        # Only the car under test gets sensor error. The traffic scenario's
        # opponent is a scripted reference trajectory rather than a vehicle
        # being validated, and noising it would perturb the baseline the ego
        # is measured against without saying anything about the real car.
        self.degrade_agents = degrade_agents

        self.grip = [
            GripEnvelope(
                friction_accel_limit=calibration.friction_accel_limit,
                wheelbase=calibration.wheelbase,
                longitudinal_coupling=calibration.longitudinal_grip_coupling,
            )
            for _ in range(num_agents)
        ]
        self.servos = [
            SteeringServo(
                gain=calibration.steering_servo_gain,
                rate_max=calibration.steering_rate_max,
                angle_min=calibration.steering_max_right,
                angle_max=calibration.steering_max_left,
            )
            for _ in range(num_agents)
        ]
        steer_ticks = (
            calibration.steering_delay_ticks if profile.transport_delay else 0
        )
        speed_ticks = (
            calibration.speed_delay_ticks if profile.transport_delay else 0
        )
        self.steer_delays = [TransportDelay(steer_ticks) for _ in range(num_agents)]
        self.speed_delays = [TransportDelay(speed_ticks) for _ in range(num_agents)]

        self.pose_sensors = []
        self.odom_sensors = []
        self.scan_sensors = []
        for index in range(num_agents):
            rng = np.random.default_rng(seed + 7919 * (index + 1))
            self.pose_sensors.append(
                PoseSensor(
                    calibration.pose_delay_ticks,
                    calibration.pose_noise_xy,
                    calibration.pose_noise_yaw,
                    rng,
                )
            )
            self.odom_sensors.append(
                OdomSensor(calibration.odom_speed_noise, rng)
            )
            self.scan_sensors.append(
                ScanSensor(
                    calibration.lidar_delay_ticks,
                    calibration.lidar_dropout_rate,
                    rng,
                )
            )

        self._truth: dict[int, np.ndarray] = {}
        self._estimated_poses: dict[int, np.ndarray] = {}
        self.chassis_contact_steps = 0
        # Latched: the scenario result reads the final observation, so a
        # contact that the cars slid back out of must not vanish from it.
        self._contacted: set[int] = set()

        # Imported here rather than at module scope: this package is on the
        # path before bootstrap() has put the gym checkout there.
        from f1tenth_gym.envs.collision_models import collision, get_vertices
        from f1tenth_gym.envs.lidar.laser_models import distance_transform

        self._collision = collision
        self._get_vertices = get_vertices
        self._distance_transform = distance_transform

    # -- lifecycle -------------------------------------------------------

    def reset(self, poses: np.ndarray):
        obs, info = self.env.reset(options={"poses": poses})
        self._contacted = set()
        self.chassis_contact_steps = 0
        for index in range(self.num_agents):
            self.steer_delays[index].reset()
            self.speed_delays[index].reset()
            self.pose_sensors[index].reset()
            self.scan_sensors[index].reset()
        return self._observe(obs), info

    def close(self) -> None:
        self.env.close()

    # -- stepping --------------------------------------------------------

    def step(self, commands):
        """Advance one control tick.

        Args:
            commands: ``[(steering_angle, speed), ...]``, one per agent, as
                the controllers produced them.
        """
        actions = np.zeros((self.num_agents, 2), dtype=np.float32)
        for index, (steering, speed) in enumerate(commands):
            steering = float(steering)
            speed = float(speed)
            state = self._truth[index]
            measured_speed = float(state[3])
            current_steering = float(state[2])

            steering = self.steer_delays[index].push(steering)
            speed = self.speed_delays[index].push(speed)

            if self.profile.calibrated_vehicle:
                # The VESC's eRPM ceiling, applied to the command rather than
                # through gym's v_max -- see calibration.vehicle_parameters()
                # for why routing it through v_max would be a trap.
                speed = min(
                    max(speed, self.calibration.speed_min),
                    self.calibration.speed_max,
                )

            if self.profile.grip_envelope:
                accel = longitudinal_accel(speed, measured_speed, self.params)
                steering = self.grip[index].apply(steering, measured_speed, accel)

            if self.profile.servo_model:
                actions[index, 0] = self.servos[index].velocity(
                    steering, current_steering
                )
            else:
                actions[index, 0] = steering
            actions[index, 1] = speed

        obs, reward, done, truncated, info = self.env.step(actions)
        return self._observe(obs), reward, done, truncated, info

    # -- observation -----------------------------------------------------

    def _chassis_outline(self, index: int) -> np.ndarray:
        """The agent's real chassis corners in world coordinates."""
        simulator = self.env.unwrapped.sim
        pose = simulator._collision_pose_from_base(  # official internal
            simulator.state.poses[index]
        )
        return self._get_vertices(
            np.array([pose[0], pose[1], pose[2]], dtype=np.float64),
            self.calibration.body_length,
            self.calibration.body_width,
        )

    def _wall_contact(self) -> set[int]:
        """Agents whose chassis has reached occupied space on the map.

        Upstream's wall check is inoperative in this harness's geometry --
        see the note in FidelityPlant.__doc__ -- so contact is tested against
        the map directly. The distance transform gives metres to the nearest
        obstacle from any world point; sampling it around the chassis outline
        catches the car reaching a barrier from any direction, including the
        rear arc the LiDAR cannot see.
        """
        simulator = self.env.unwrapped.sim
        scan_sim = simulator.scan_sims[0]
        if scan_sim.dt is None:
            return set()
        touching: set[int] = set()
        for index in range(self.num_agents):
            corners = self._chassis_outline(index)
            # Corners plus edge midpoints: a barrier can intrude on an edge
            # without containing either corner that bounds it.
            samples = np.vstack((corners, (corners + np.roll(corners, -1, 0)) / 2.0))
            for x, y in samples:
                clearance = self._distance_transform(
                    float(x), float(y),
                    scan_sim.orig_x, scan_sim.orig_y,
                    scan_sim.orig_c, scan_sim.orig_s,
                    scan_sim.map_height, scan_sim.map_width,
                    scan_sim.map_resolution, scan_sim.dt,
                )
                if clearance <= 0.0:
                    touching.add(index)
                    break
        return touching

    def _chassis_contact(self) -> set[int]:
        """Agents whose real chassis overlap, checked all round the car.

        Upstream detects agent contact from the LiDAR scan, so it inherits
        the sensor's +/-135 deg field of view and cannot see the 90 deg wedge
        behind the car (F10) -- an opponent rear-ended there never registers.
        This is the same separating-axis test gym uses, run over the whole
        circle and against the true chassis rather than the padded envelope.
        """
        if self.num_agents < 2:
            return set()
        vertices = [self._chassis_outline(i) for i in range(self.num_agents)]
        touching: set[int] = set()
        for a in range(self.num_agents):
            for b in range(a + 1, self.num_agents):
                if self._collision(vertices[a], vertices[b]):
                    touching.update((a, b))
        return touching

    def _observe(self, obs: dict) -> dict:
        """Record ground truth, then hand back what the sensors would report."""
        degraded = dict(obs)
        contact: set[int] = set()
        if self.profile.body_collisions:
            contact = self._wall_contact() | self._chassis_contact()
        if contact:
            self.chassis_contact_steps += 1
            self._contacted |= contact
        contact = self._contacted

        for index in range(self.num_agents):
            key = f"agent_{index}"
            agent = obs[key]
            truth = np.asarray(agent["std_state"], dtype=float)
            self._truth[index] = truth
            self._estimated_poses[index] = truth[[0, 1, 4]].copy()

            if index in contact and not agent["collision"]:
                agent = dict(agent)
                agent["collision"] = True
                degraded[key] = agent

            if not self.profile.sensor_error or index not in self.degrade_agents:
                continue

            estimate = self.pose_sensors[index].update(
                float(truth[0]), float(truth[1]), float(truth[4])
            )
            self._estimated_poses[index] = estimate
            speed = self.odom_sensors[index].update(float(truth[3]))

            state = truth.copy()
            state[0], state[1], state[4] = estimate
            state[3] = speed

            agent = dict(agent)
            agent["std_state"] = state
            agent["scan"] = self.scan_sensors[index].update(agent["scan"])
            degraded[key] = agent
        return degraded

    def truth(self, index: int = 0) -> np.ndarray:
        """Ground-truth standard state, for metrics and assertions only."""
        return self._truth[index]

    def freeze_pose(self, index: int = 0, frozen: bool = True) -> None:
        """Stop the pose estimate updating, as a stalled SLAM transform does."""
        self.pose_sensors[index].frozen = frozen

    def expected_scan(self, index: int = 0) -> np.ndarray:
        """Map prediction for opponent detection, from the *estimated* pose.

        The original harness ray-cast from ground truth against the same map
        object that produced the measurement, so map subtraction was exact by
        construction and ``map_subtraction_margin`` faced nothing to absorb
        (F8). Casting from the estimated pose gives it the dominant real error
        source -- localisation -- to cope with.
        """
        simulator = self.env.unwrapped.sim
        pose = np.asarray(self._estimated_poses[index], dtype=np.float32)
        lidar_pose = simulator._lidar_pose_from_base(pose)  # official internal
        return simulator.scan_sims[index].scan(lidar_pose, rng=None)

    # -- reporting -------------------------------------------------------

    def report(self) -> dict:
        """Fidelity metrics to fold into the scenario result."""
        report = {"fidelity_profile": self.profile.name}
        if self.profile.body_collisions:
            report["contact_steps"] = self.chassis_contact_steps
        if self.profile.grip_envelope:
            report.update(self.grip[0].report())
        if self.profile.sensor_error:
            report["max_pose_error_m"] = round(
                self.pose_sensors[0].max_error, 4
            )
            report["dropped_beams"] = self.scan_sensors[0].dropped_beams
        return report
