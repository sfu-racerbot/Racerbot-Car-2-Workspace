"""This car's physical parameters, each tagged with where it came from.

The point of this module is honesty about what is known. Stock F1TENTH Gym
ships one parameter set describing *a* small car; almost none of it was ever
checked against the vehicle in this workshop. Rather than silently inheriting
those numbers, every value here carries a :class:`Provenance` saying whether it
was measured on this car, derived from this car's committed configs, taken from
a datasheet for a part actually fitted, estimated from physics, or simply
inherited from upstream.

``python3 -m racerbot_sim.calibration`` prints the table, worst provenance
first. Anything still marked ``STOCK`` is a number nobody has justified.

Filling in a measurement means editing one line here. See
``docs/sim-fidelity-audit.md`` R1 for the procedures.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from enum import Enum
import math

# Standard gravity, matching the figure used throughout the fidelity audit.
G = 9.81


class Provenance(Enum):
    """How much a number is worth, in descending order of trust."""

    MEASURED = "measured"     # measured on this physical car
    DERIVED = "derived"       # computed from this car's committed configs
    SPEC = "spec"             # datasheet for a part actually fitted
    ESTIMATED = "estimated"   # physics-based estimate, not this car
    STOCK = "stock"           # F1TENTH Gym default; describes another vehicle

    @property
    def rank(self) -> int:
        order = [
            Provenance.MEASURED,
            Provenance.DERIVED,
            Provenance.SPEC,
            Provenance.ESTIMATED,
            Provenance.STOCK,
        ]
        return order.index(self)


# --------------------------------------------------------------------------
# Raw constants copied from this car's committed configuration files. Every
# derived value below is computed from these rather than hard-coded, so that
# re-deriving after a config change is a matter of updating one number.
# --------------------------------------------------------------------------

# src/f1tenth_system/f1tenth_stack/config/vesc.yaml
SPEED_TO_ERPM_GAIN = 4614.0
SPEED_MAX_ERPM = 23250.0
STEERING_ANGLE_TO_SERVO_GAIN = -1.2135
STEERING_ANGLE_TO_SERVO_OFFSET = 0.5304
SERVO_MIN = 0.15
SERVO_MAX = 0.85
MAX_SERVO_SPEED = 3.2  # throttle_interpolator block, rad/s

# Stock F1TENTH_VEHICLE_PARAMETERS, retained here so the fraction of the
# wheelbase ahead of the centre of gravity survives our wheelbase override.
STOCK_LF = 0.15875
STOCK_LR = 0.17145


def _servo_to_steering(servo: float) -> float:
    """Invert vesc.yaml's servo map: servo = gain * angle + offset."""
    return (servo - STEERING_ANGLE_TO_SERVO_OFFSET) / STEERING_ANGLE_TO_SERVO_GAIN


@dataclass(frozen=True)
class CarCalibration:
    """Every physical constant the simulator needs for this car."""

    # --- Geometry -------------------------------------------------------
    wheelbase: float = 0.324
    # The padded safety envelope, used for clearance to walls.
    width: float = 0.31
    length: float = 0.58
    # The actual chassis. Contact between two cars has to use this: applying
    # the padded envelope to both bodies counts the safety margin twice and
    # reports a collision while roughly 7 mm of real air remains.
    body_width: float = 0.281
    body_length: float = 0.535
    lidar_offset_x: float = 0.33

    # --- Mass properties ------------------------------------------------
    mass: float = 3.74
    yaw_inertia: float = 0.04712
    cog_height: float = 0.074
    # Fraction of the wheelbase between the front axle and the centre of
    # gravity; 0.5 puts the CoG exactly at the wheelbase midpoint. Held at
    # 0.5 deliberately. The stock F1TENTH ratio is 0.4808 (STOCK_LF/STOCK_LR
    # below) and the audit rightly called 0.5 an assumption -- but 0.4808
    # belongs to a car with a 0.3302 m wheelbase, so it is no more a
    # measurement of this vehicle than 0.5 is. Swapping one unmeasured
    # number for another would move every result without improving fidelity.
    # Weigh each axle and put the real figure here; see R1.
    cog_front_fraction: float = 0.5

    # --- Tyres ----------------------------------------------------------
    cornering_stiffness_front: float = 4.718
    cornering_stiffness_rear: float = 5.4562
    # The friction coefficient between these tyres and the floor the car
    # actually runs on. Drives cornering, acceleration and braking limits
    # together, exactly as a friction circle says it should.
    surface_friction: float = 0.70
    # Share of the longitudinal demand that competes with cornering for grip.
    # See GripEnvelope.longitudinal_coupling for why this is not simply 1.
    longitudinal_grip_coupling: float = 1.0

    # --- Actuation ------------------------------------------------------
    steering_max_left: float = _servo_to_steering(SERVO_MIN)
    steering_max_right: float = _servo_to_steering(SERVO_MAX)
    steering_rate_max: float = MAX_SERVO_SPEED
    # Closed-loop proportional gain of the steering servo's own position
    # loop, 1/s. Must stay below 1/control_dt (40 for a 40 Hz loop) or the
    # discrete servo overshoots -- which is the bug this replaces.
    steering_servo_gain: float = 25.0
    # Whole-loop transport lag, in control ticks, from a command being
    # published to the actuator acting on it: ROS -> USB -> VESC firmware.
    steering_delay_ticks: int = 1
    speed_delay_ticks: int = 1

    # --- Speed envelope -------------------------------------------------
    speed_max: float = SPEED_MAX_ERPM / SPEED_TO_ERPM_GAIN
    speed_min: float = -SPEED_MAX_ERPM / SPEED_TO_ERPM_GAIN

    # --- Sensing --------------------------------------------------------
    lidar_beams: int = 1081
    lidar_fov: float = math.radians(270.0)
    lidar_range_max: float = 25.0
    lidar_noise_std: float = 0.01
    lidar_dropout_rate: float = 0.002
    lidar_delay_ticks: int = 1
    pose_delay_ticks: int = 2
    pose_noise_xy: float = 0.02
    pose_noise_yaw: float = 0.01
    odom_speed_noise: float = 0.05

    # ------------------------------------------------------------------
    # Quantities the simulator needs that follow from the above
    # ------------------------------------------------------------------

    @property
    def lf(self) -> float:
        """Front axle to centre of gravity, metres."""
        return self.wheelbase * self.cog_front_fraction

    @property
    def lr(self) -> float:
        """Centre of gravity to rear axle, metres."""
        return self.wheelbase * (1.0 - self.cog_front_fraction)

    @property
    def friction_accel_limit(self) -> float:
        """Largest acceleration of any direction the floor can support.

        The friction circle's radius. This chassis is four-wheel drive
        (confirmed 2026-08-05), so the motor drives and brakes all four
        wheels through the transmission and the longitudinal extreme is mu*g
        just as the lateral one is -- a single number bounds acceleration,
        braking and cornering alike. On a rear-drive car it would not: rear
        axle load would cap braking near mu*g*lf/(L + mu*h), well under half
        this figure.
        """
        return self.surface_friction * G

    def vehicle_parameters(self, stock):
        """Stock gym :class:`VehicleParameters` updated to describe this car."""
        return stock.with_updates(
            mu=self.surface_friction,
            C_Sf=self.cornering_stiffness_front,
            C_Sr=self.cornering_stiffness_rear,
            lf=self.lf,
            lr=self.lr,
            h=self.cog_height,
            m=self.mass,
            I=self.yaw_inertia,
            s_min=self.steering_max_right,
            s_max=self.steering_max_left,
            sv_min=-self.steering_rate_max,
            sv_max=self.steering_rate_max,
            a_max=self.friction_accel_limit,
            # v_min/v_max are deliberately NOT overridden here. In gym they
            # are not only the speed envelope: pid_accl derives its
            # proportional gain from them as 10*a_max/v_max, so writing this
            # car's real 5.039 m/s ceiling into v_max would quietly multiply
            # the plant's speed-loop gain by four. That gain is an upstream
            # invention, not a model of this car's VESC, so perturbing it is
            # not a fidelity improvement -- it just moves every result. The
            # real ceiling is enforced by FidelityPlant clamping the speed
            # command instead, which bounds the speed without touching the
            # gain. Measure the VESC's actual step response (R7) to replace
            # the whole arrangement with something justified.
            width=self.width,
            length=self.length,
            collision_body_center_x=self.wheelbase / 2.0,
        )


#: Where every field of :class:`CarCalibration` came from.
PROVENANCE: dict[str, tuple[Provenance, str]] = {
    "wheelbase": (
        Provenance.DERIVED,
        "vesc.yaml wheelbase: .324 (Traxxas 74276-4 published figure)",
    ),
    "width": (
        Provenance.DERIVED,
        "pure_pursuit.yaml car_width -- deliberately padded past the real body",
    ),
    "length": (
        Provenance.DERIVED,
        "pure_pursuit.yaml car_length -- deliberately padded past the real body",
    ),
    "body_width": (
        Provenance.SPEC,
        "Traxxas 74276-4 published body width, per pure_pursuit.yaml",
    ),
    "body_length": (
        Provenance.SPEC,
        "Traxxas 74276-4 published body length, per pure_pursuit.yaml",
    ),
    "lidar_offset_x": (
        Provenance.DERIVED,
        "pure_pursuit.yaml laser_offset_x (itself recorded as an estimate)",
    ),
    "mass": (
        Provenance.STOCK,
        "gym default. NEVER WEIGHED -- put the car on a kitchen scale",
    ),
    "yaw_inertia": (
        Provenance.STOCK,
        "gym default. A uniform slab of this footprint would be 0.135 kg m^2, "
        "2.9x higher; a bifilar pendulum test would settle it",
    ),
    "cog_height": (Provenance.STOCK, "gym default. Never measured"),
    "cog_front_fraction": (
        Provenance.STOCK,
        "assumed midpoint. Weigh each axle separately to replace it -- this "
        "ratio IS the understeer/oversteer balance, and the traffic scenario "
        "is measurably sensitive to it",
    ),
    "cornering_stiffness_front": (
        Provenance.STOCK,
        "gym default, original F1TENTH tyres",
    ),
    "cornering_stiffness_rear": (
        Provenance.STOCK,
        "gym default, original F1TENTH tyres",
    ),
    "surface_friction": (
        Provenance.ESTIMATED,
        "rubber RC tyres on smooth sealed concrete. Plausible band 0.5-0.9; "
        "measure with a straight-line hard-brake run (R1)",
    ),
    "longitudinal_grip_coupling": (
        Provenance.ESTIMATED,
        "how much of the friction circle the drivetrain can claim. Full "
        "coupling is the honest ellipse but bites far too often against "
        "upstream's all-or-nothing speed controller",
    ),
    "steering_max_left": (
        Provenance.DERIVED,
        "vesc.yaml servo_min 0.15 through the servo map",
    ),
    "steering_max_right": (
        Provenance.DERIVED,
        "vesc.yaml servo_max 0.85 through the servo map",
    ),
    "steering_rate_max": (
        Provenance.DERIVED,
        "vesc.yaml throttle_interpolator max_servo_speed 3.2 rad/s",
    ),
    "steering_servo_gain": (
        Provenance.ESTIMATED,
        "40 ms servo position-loop time constant. Bounded above by 1/dt = 40",
    ),
    "steering_delay_ticks": (
        Provenance.ESTIMATED,
        "one 25 ms tick of ROS -> USB -> VESC transport lag",
    ),
    "speed_delay_ticks": (
        Provenance.ESTIMATED,
        "one 25 ms tick; measure from a step-response run (R7)",
    ),
    "speed_max": (
        Provenance.DERIVED,
        "vesc.yaml speed_max 23250 eRPM / speed_to_erpm_gain 4614. Enforced "
        "by clamping the speed command, not through gym's v_max -- see "
        "vehicle_parameters()",
    ),
    "speed_min": (Provenance.DERIVED, "vesc.yaml speed_min, same conversion"),
    "lidar_beams": (
        Provenance.SPEC,
        "Hokuyo UST-10LX: 1081 steps of 0.25 deg over 270 deg, and "
        "sensors.yaml asks for full resolution (cluster 1, skip 0). Confirm "
        "with: ros2 topic echo /scan --once --field angle_increment",
    ),
    "lidar_fov": (Provenance.SPEC, "Hokuyo UST-10LX 270 deg sweep"),
    "lidar_range_max": (
        Provenance.SPEC,
        "UST-10LX rated 10 m guaranteed / 30 m maximum; 25 m kept from the "
        "original harness",
    ),
    "lidar_noise_std": (
        Provenance.SPEC,
        "UST-10LX +/-40 mm accuracy under 10 m implies roughly 1 cm sigma",
    ),
    "lidar_dropout_rate": (
        Provenance.ESTIMATED,
        "fraction of beams returning no echo off dark or glancing surfaces",
    ),
    "lidar_delay_ticks": (
        Provenance.ESTIMATED,
        "one tick: the scan is up to one 40 Hz period stale when used",
    ),
    "pose_delay_ticks": (
        Provenance.ESTIMATED,
        "two ticks (50 ms) of particle-filter latency. Measure with "
        "race_diagnostics' pose-lag report",
    ),
    "pose_noise_xy": (
        Provenance.ESTIMATED,
        "2 cm particle-filter position jitter against a good map",
    ),
    "pose_noise_yaw": (
        Provenance.ESTIMATED,
        "0.01 rad (0.6 deg) particle-filter heading jitter",
    ),
    "odom_speed_noise": (
        Provenance.ESTIMATED,
        "VESC eRPM speed estimate; the 2026-07-27 log tracked commands "
        "within about 5%",
    ),
}


CALIBRATION = CarCalibration()


def describe(calibration: CarCalibration = CALIBRATION) -> list[tuple]:
    """(provenance, name, value, source) rows, least trustworthy first."""
    rows = []
    for field in fields(calibration):
        provenance, source = PROVENANCE[field.name]
        rows.append(
            (provenance, field.name, getattr(calibration, field.name), source)
        )
    rows.sort(key=lambda row: (-row[0].rank, row[1]))
    return rows


def unmeasured(calibration: CarCalibration = CALIBRATION) -> list[str]:
    """Fields still inherited from upstream rather than justified here."""
    return [
        name
        for provenance, name, _value, _source in describe(calibration)
        if provenance is Provenance.STOCK
    ]


def main() -> int:
    rows = describe()
    width = max(len(name) for _p, name, _v, _s in rows)
    current = None
    for provenance, name, value, source in rows:
        if provenance is not current:
            print(f"\n[{provenance.value.upper()}]")
            current = provenance
        shown = f"{value:.5g}" if isinstance(value, float) else str(value)
        print(f"  {name:<{width}}  {shown:>10}   {source}")
    print(f"\n{len(unmeasured())} of {len(rows)} values are still gym stock.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
