"""The gap_follow decision pipeline, without ROS, for the simulators.

gap_follow_node._scan_step composes gap_logic's pure functions in a fixed
order, with state carried between ticks. GapController below composes the
same functions in the same order, so tools/f1tenth_sim/eval_gap.py and
run_validation.py's gap_solo scenario can drive a simulated car with what the
real node would command -- without rclpy, at many times real time.

It is still a mirror of _scan_step's *ordering* and must be kept in step with
it by hand (docs/gap-follow.md#keeping-the-mirror-honest). The *parameters*
are not mirrored: node_params() reads them from src/gap_follow/config/
gap_follow.yaml, the file the car loads, so a tuning change there is what the
simulators measure with no second copy to forget.

Pure numpy + gap_logic + PyYAML: no gym import, importable unsourced.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

from gap_follow import gap_logic
from gap_follow import opponent_tracking

CONTROL_DT = 0.025
ROOT = Path(__file__).resolve().parents[2]
NODE_CONFIG = ROOT / "src" / "gap_follow" / "config" / "gap_follow.yaml"

# Node parameters the mirror does not use (topics, deadman, intent, logging)
# or that only the ROS node can act on.
NOT_MIRRORED = {
    "scan_topic", "drive_topic", "odom_topic", "joy_topic", "deadman_button",
    "joy_timeout_sec", "enable_deadman", "odom_timeout_sec",
    "decision_log_period_sec", "scan_timeout_sec", "publish_intent",
    "intent_topic", "intent_rate_hz", "intent_horizon_sec", "intent_samples",
    "intent_max_length", "enable_cornering_anticipation",
    "anticipation_near_depth",
}


# Every parameter GapController reads. test_gap_mirror.py checks the YAML
# provides each one and that nothing else in the YAML goes unread.
REQUIRED_PARAMS = frozenset((
    "adaptive_width_narrow",
    "adaptive_width_reference",
    "arc_guard_blocked_fallback",
    "narrow_corridor_clearance",
    "narrow_corridor_speed",
    "open_corridor_clearance",
    "arc_guard_candidates",
    "arc_guard_length",
    "arc_guard_margin",
    "arc_guard_max_correction",
    "braking_latency_sec",
    "car_length",
    "car_width",
    "centering_full_bearing_deg",
    "centering_full_forward_depth",
    "centering_full_side_distance",
    "centering_gain",
    "centering_max_steering",
    "centering_side_fov_deg",
    "centering_zero_bearing_deg",
    "centering_zero_forward_depth",
    "centering_zero_side_distance",
    "command_slew_max_dt",
    "corner_setup_excess_threshold",
    "corner_setup_far_full",
    "corner_setup_far_zero",
    "corner_setup_fraction",
    "corner_setup_full_side_distance",
    "corner_setup_full_signal",
    "corner_setup_gain",
    "corner_setup_max_steering",
    "corner_setup_near_full",
    "corner_setup_near_zero",
    "corner_setup_sector_hi_deg",
    "corner_setup_sector_lo_deg",
    "corner_setup_wall_margin",
    "corner_setup_zero_side_distance",
    "corner_setup_zero_signal",
    "corner_speed",
    "corner_speed_wide",
    "disparity_threshold",
    "emergency_stop_clearance",
    "enable_adaptive_width",
    "enable_arc_guard",
    "enable_centering",
    "enable_corner_setup",
    "enable_swept_clearance",
    "enable_ttc",
    "escape_creep_speed",
    "fallback_min_gap_distance",
    "forward_fov_deg",
    "forward_stop_clearance",
    "forward_stop_fov_deg",
    "gap_switch_margin",
    "laser_offset_x",
    "laser_offset_y",
    "max_acceleration",
    "max_braking_decel",
    "max_lateral_accel",
    "max_range",
    "max_speed",
    "max_steering_angle",
    "max_steering_rate",
    "min_centerline_gap_width",
    "min_gap_distance",
    "min_safety_margin",
    "min_speed",
    "safety_margin",
    "speed_hold_sec",
    "steering_gain",
    "steering_gain_speed_ref",
    "swept_clearance_margin",
    "swept_clearance_max_length",
    "ttc_command_fallback_max_odom_speed",
    "ttc_command_speed_timeout_sec",
    "ttc_min_brake_speed",
    "ttc_min_closing_speed",
    "ttc_path_margin",
    "ttc_threshold_sec",
    "wheelbase",

    "enable_opponent_awareness",
    "opponent_min_width",
    "opponent_max_width",
    "opponent_max_range",
    "opponent_segment_jump",
    "opponent_min_beams",
    "opponent_gate",
    "opponent_alpha",
    "opponent_beta",
    "opponent_min_hits",
    "opponent_max_misses",
    "opponent_moving_speed",
    "opponent_radius",
    "opponent_passing_margin",
    "opponent_prediction_horizon",
    "opponent_prediction_step",
    "opponent_follow_min_gap",
    "opponent_follow_time_gap",
    "opponent_follow_lateral",
    "opponent_relevant_lateral",
    "opponent_pass_max_steering",
    "opponent_pass_max_heading_deg",
    "opponent_max_track_speed",
    "opponent_steer_delay",
    "opponent_steer_gain",
    "opponent_steer_rate_max",
))


# Parameters added after the first real-car run of the October 2026 rework,
# with the value that reproduces the behaviour from before they existed. Lets
# an older YAML (git show <commit>:src/gap_follow/config/gap_follow.yaml) be
# replayed against the run it drove -- see tools/run_analysis/replay.py.
LEGACY_VALUES = {
    "arc_guard_blocked_fallback": "freest",
    "braking_latency_sec": 0.0,
    "speed_hold_sec": 0.0,
    "steering_gain_speed_ref": 0.0,
    "narrow_corridor_speed": 1e9,  # off
    "narrow_corridor_clearance": 0.05,
    "open_corridor_clearance": 0.15,
    "ttc_path_margin": None,  # None: the old corridor, safety_margin itself
    # Opponent awareness (2026-10-22) did not exist: off. Its opponent_*
    # values are then never read, but the mirror requires them present.
    "enable_opponent_awareness": False,
    **{f"opponent_{name}": getattr(opponent_tracking.OpponentParams(), name)
       for name in opponent_tracking.OpponentParams.__dataclass_fields__},
}


def node_params(config_path: Path = NODE_CONFIG, fill_legacy: bool = False) -> dict:
    """gap_follow_node's ros__parameters from the shipped YAML, minus the
    ones the mirror has no use for.

    Refuses a config that enables something the mirror does not model, so a
    simulator run can never quietly measure a different controller.
    """
    data = yaml.safe_load(Path(config_path).read_text())
    params = dict(data["gap_follow_node"]["ros__parameters"])
    if fill_legacy:
        for name, value in LEGACY_VALUES.items():
            if name not in params:
                params[name] = params["safety_margin"] if value is None else value
    if params.get("enable_cornering_anticipation", False):
        raise ValueError(
            "gap_follow.yaml enables cornering anticipation, which "
            "gap_mirror.GapController does not model -- add it to the mirror "
            "before trusting a simulator run")
    return {k: v for k, v in params.items() if k not in NOT_MIRRORED}


class GapController:
    """Composable replica of gap_follow_node._scan_step's decision pipeline.

    ``params`` is node_params() (optionally with overrides); ``lidar`` needs
    ``angle_min``, ``angle_increment`` and ``range_min``. ``command(scan,
    measured_speed, step)`` returns ``(steering, speed)`` for one 40 Hz tick.

    Holds parameters as attributes and reproduces the node's per-scan chain:
    sanitize -> side walls/adaptive width -> footprint/TTC -> disparity extend
    -> safety bubble -> gap pick -> bearing steering -> speed ceilings -> slew.
    State that the node keeps between ticks (previous aim for hysteresis,
    steering basis, last command) lives here too.
    """

    def __init__(self, params: dict, lidar):
        missing = sorted(REQUIRED_PARAMS - set(params))
        if missing:
            raise KeyError(f"GapController is missing parameters: {missing}")
        self.p = dict(params)
        self.lidar = lidar
        self.previous_target_idx = None
        self.steering_basis = 0.0
        self.last_commanded_speed = 0.0
        self.last_command_time = None  # in sim steps
        self.stopped = False
        self.stop_reason = None
        self.speed_hold = gap_logic.SpeedHold(self.p["speed_hold_sec"])
        self.speed_hold_clock = 0.0
        # Which speed ceiling won this tick, for the harness's binding-cap
        # histogram: 'curve', 'clearance', 'corner', 'creep', 'accel', 'stop'.
        self.binding_cap = None
        self.cap_counts = None
        # Everything this tick decided, for trace_gap.py. Rebuilt every call.
        self.telemetry = {}
        # Moving-obstacle awareness (gap_follow/opponent_tracking.py), only
        # when the YAML enables it -- otherwise not even constructed.
        self.opponents = None
        if self.p.get("enable_opponent_awareness", False):
            self.opponents = opponent_tracking.OpponentAwareness(
                opponent_tracking.OpponentParams.from_mapping(self.p),
                self.p["wheelbase"], self.p["laser_offset_x"],
                self.p["laser_offset_y"], self.p["car_length"])
        self.opponent_mode = None

    def command(self, scan, current_speed, step):
        p = self.p
        tel = self.telemetry = {}
        clean, valid = gap_logic.sanitize_ranges(
            scan, max_range=p["max_range"], range_min=self.lidar.range_min)

        angles_all = self.lidar.angle_min + np.arange(
            clean.size, dtype=float) * self.lidar.angle_increment
        if self.opponents is not None:
            self.opponents.observe(clean, valid, angles_all, float(current_speed),
                                   self.steering_basis, CONTROL_DT)

        if (p["enable_centering"] or p["enable_adaptive_width"]
                or p["enable_corner_setup"]
                or p["narrow_corridor_speed"] < p["max_speed"]):
            half_span = math.radians(p["centering_side_fov_deg"]) / 2.0
            side_left = gap_logic.side_wall_distance(
                clean, valid, angles_all, math.pi / 2.0, half_span)
            side_right = gap_logic.side_wall_distance(
                clean, valid, angles_all, -math.pi / 2.0, half_span)
        else:
            side_left = side_right = math.inf

        width_factor, effective_margin, effective_corner = self._adaptive_width(
            side_left, side_right)

        half_fov = math.radians(p["forward_fov_deg"]) / 2.0
        lo = int((-half_fov - self.lidar.angle_min) / self.lidar.angle_increment)
        hi = int((half_fov - self.lidar.angle_min) / self.lidar.angle_increment)
        lo = max(0, lo)
        hi = min(clean.size - 1, hi)
        window = clean[lo:hi + 1]
        window_valid = valid[lo:hi + 1]
        beam_angles = angles_all[lo:hi + 1]

        boundaries = gap_logic.vehicle_boundary_distances(
            beam_angles, p["car_width"], p["car_length"], p["wheelbase"],
            p["laser_offset_x"], p["laser_offset_y"])

        min_clearance = gap_logic.minimum_footprint_clearance(
            window, window_valid, boundaries)
        forward_clearance = gap_logic.minimum_footprint_clearance_in_cone(
            window, window_valid, beam_angles, boundaries,
            math.radians(p["forward_stop_fov_deg"]))

        directional = math.isclose(
            forward_clearance, min_clearance, rel_tol=0.0, abs_tol=1e-6)
        tel.update(side_left=side_left, side_right=side_right,
                   width_factor=width_factor, min_clear=min_clearance,
                   fwd_clear=forward_clearance)

        emergency_tripped = False
        if min_clearance <= p["emergency_stop_clearance"]:
            if not directional:
                return self._stop(step, "emergency_clearance")
            emergency_tripped = True

        creeping = forward_clearance <= p["forward_stop_clearance"]

        ttc_tripped = False
        if p["enable_ttc"]:
            command_age = (
                math.inf if self.last_command_time is None
                else (step - self.last_command_time) * CONTROL_DT)
            effective_speed = gap_logic.conservative_ttc_speed(
                current_speed, self.last_commanded_speed, command_age,
                p["ttc_command_speed_timeout_sec"],
                p["ttc_command_fallback_max_odom_speed"])
            if effective_speed >= p["ttc_min_brake_speed"]:
                min_ttc = gap_logic.minimum_ttc(
                    window, window_valid, beam_angles, effective_speed,
                    boundaries, p["ttc_min_closing_speed"],
                    p["car_width"] / 2.0 + p["ttc_path_margin"],
                    math.tan(self.steering_basis) / p["wheelbase"],
                    p["laser_offset_x"], p["laser_offset_y"])
                tel["min_ttc"] = min_ttc
                if min_ttc <= p["ttc_threshold_sec"]:
                    if not directional:
                        return self._stop(step, "ttc_brake")
                    ttc_tripped = True

        # Gap selection
        closest_idx, closest_dist = gap_logic.closest_valid(window, window_valid)
        half_width = p["car_width"] / 2.0 + effective_margin

        def select(search_window):
            processed = gap_logic.disparity_extend(
                search_window, self.lidar.angle_increment,
                p["disparity_threshold"], half_width)
            if closest_idx is not None:
                processed = gap_logic.safety_bubble(
                    processed, closest_idx, closest_dist,
                    self.lidar.angle_increment, half_width)
            return (processed,) + gap_logic.find_gap_with_fallback(
                processed, p["min_gap_distance"], p["fallback_min_gap_distance"],
                self.lidar.angle_increment, p["min_centerline_gap_width"],
                previous_target_idx=self.previous_target_idx,
                switch_margin=p["gap_switch_margin"])

        follow_cap = math.inf
        self.opponent_mode = None
        if self.opponents is not None and self.opponents.moving_body():
            avoid = select(self.opponents.augment(window, beam_angles))
            plain = select(window)
            # Use the avoiding gap only to pick a side within the opening
            # the car would take anyway: it must overlap the plain gap and be
            # no shallower. Anything else -- no gap, a fallback where the
            # real scan offers a deep one, a different opening altogether --
            # means follow instead.
            # And only on a straight (opponent_tracking.pass_allowed).
            passing = (self.opponents.may_pass(self.steering_basis)
                       and opponent_tracking.avoidance_usable(avoid[1:], plain[1:]))
            if passing:
                processed, gap_start, gap_end, used_fallback = avoid
                self.opponent_mode = "clear"
            else:
                processed, gap_start, gap_end, used_fallback = plain
                self.opponent_mode = "follow"
            follow_cap = self.opponents.follow_cap(passing)
            tel["follow_cap"] = follow_cap
        else:
            processed, gap_start, gap_end, used_fallback = select(window)

        if gap_start is None:
            return self._stop(step, "no_safe_gap")

        if emergency_tripped or ttc_tripped:
            creeping = True

        target_in_window = gap_logic.aim_within_gap(
            processed, gap_start, gap_end, beam_angles)
        self.previous_target_idx = target_in_window
        target_angle = float(beam_angles[target_in_window])
        target_distance = float(processed[target_in_window])

        centering_bias = 0.0
        if p["enable_centering"] and not creeping:
            centering_bias, _w = gap_logic.corridor_centering_bias(
                side_left, side_right, target_angle, target_distance,
                p["centering_gain"], p["centering_max_steering"],
                math.radians(p["centering_full_bearing_deg"]),
                math.radians(p["centering_zero_bearing_deg"]),
                p["centering_full_forward_depth"],
                p["centering_zero_forward_depth"],
                p["centering_full_side_distance"],
                p["centering_zero_side_distance"])

        setup_bias = 0.0
        if p["enable_corner_setup"] and not creeping:
            straight_clear = gap_logic.swept_path_clearance(
                window, window_valid, beam_angles, 0.0,
                p["car_width"] / 2.0 + p["swept_clearance_margin"],
                p["car_length"], p["wheelbase"], p["laser_offset_x"],
                p["laser_offset_y"], p["corner_setup_far_zero"] + 0.5)
            turn_signal = gap_logic.side_opening_signal(
                window, window_valid, beam_angles, side_left, side_right,
                math.radians(p["corner_setup_sector_lo_deg"]),
                math.radians(p["corner_setup_sector_hi_deg"]),
                p["corner_setup_excess_threshold"])
            setup_bias, setup_w = gap_logic.corner_setup_bias(
                side_left, side_right, turn_signal, straight_clear,
                p["car_width"], p["corner_setup_wall_margin"],
                p["corner_setup_fraction"], p["corner_setup_gain"],
                p["corner_setup_max_steering"],
                p["corner_setup_zero_signal"], p["corner_setup_full_signal"],
                p["corner_setup_far_zero"], p["corner_setup_far_full"],
                p["corner_setup_near_full"], p["corner_setup_near_zero"],
                p["corner_setup_full_side_distance"],
                p["corner_setup_zero_side_distance"])
            centering_bias *= (1.0 - setup_w)
            tel["corner_setup_bias"] = setup_bias
            tel["straight_clear"] = straight_clear

        bearing_gain = p["steering_gain"]
        gain_basis = max(self.last_commanded_speed,
                         min(abs(float(current_speed)), p["max_speed"]))
        bearing_gain *= gap_logic.speed_scheduled_gain(
            gain_basis, p["steering_gain_speed_ref"])
        desired_steering = float(np.clip(
            bearing_gain * target_angle + centering_bias + setup_bias,
            -p["max_steering_angle"], p["max_steering_angle"]))
        if p["enable_arc_guard"] and not creeping:
            speed_basis = max(self.last_commanded_speed,
                              min(abs(float(current_speed)), p["max_speed"]))
            required = max(p["arc_guard_length"],
                           speed_basis ** 2 / (2.0 * p["max_braking_decel"])
                           + p["forward_stop_clearance"])
            guarded = gap_logic.arc_guard_steering(
                window, window_valid, beam_angles, desired_steering, required,
                p["car_width"] / 2.0 + p["arc_guard_margin"],
                p["car_length"], p["wheelbase"], p["laser_offset_x"],
                p["laser_offset_y"], p["max_steering_angle"],
                p["arc_guard_max_correction"], int(p["arc_guard_candidates"]),
                p["arc_guard_blocked_fallback"])
            tel["arc_guard"] = guarded - desired_steering
            desired_steering = guarded

        dt = CONTROL_DT
        if self.last_command_time is not None:
            dt = min(p["command_slew_max_dt"],
                     max(0.0, (step - self.last_command_time) * CONTROL_DT))
        else:
            dt = p["command_slew_max_dt"]

        steering = gap_logic.slew_rate_limit(
            desired_steering, self.steering_basis, dt, p["max_steering_rate"])
        self.steering_basis = steering

        limited_curvature = math.tan(desired_steering) / p["wheelbase"]
        curve_speed = gap_logic.curvature_speed_limit(
            limited_curvature, p["max_lateral_accel"], p["max_speed"])
        normal_speed = max(p["min_speed"], curve_speed)

        braking_clearance = forward_clearance
        if p["enable_swept_clearance"]:
            swept_hw = p["car_width"] / 2.0 + p["swept_clearance_margin"]
            braking_clearance = min(
                gap_logic.swept_path_clearance(
                    window, window_valid, beam_angles, curvature, swept_hw,
                    p["car_length"], p["wheelbase"], p["laser_offset_x"],
                    p["laser_offset_y"], p["swept_clearance_max_length"])
                for curvature in (limited_curvature,
                                  math.tan(steering) / p["wheelbase"]))
            tel["arc_clear"] = braking_clearance
        clearance_speed = gap_logic.braking_speed_limit(
            braking_clearance, p["forward_stop_clearance"],
            p["max_braking_decel"], p["max_speed"],
            latency=p["braking_latency_sec"])
        if creeping:
            creep_reserve = max(p["emergency_stop_clearance"],
                                p["forward_stop_clearance"] / 2.0)
            clearance_speed = min(
                p["escape_creep_speed"],
                gap_logic.braking_speed_limit(
                    forward_clearance, creep_reserve,
                    p["max_braking_decel"], p["max_speed"]))

        desired_speed = min(normal_speed, clearance_speed)
        binding = 'curve' if normal_speed <= clearance_speed else 'clearance'
        if p["narrow_corridor_speed"] < p["max_speed"]:
            side_cap = gap_logic.side_clearance_speed_limit(
                min(side_left, side_right) - p["car_width"] / 2.0,
                p["narrow_corridor_speed"], p["narrow_corridor_clearance"],
                p["open_corridor_clearance"], p["max_speed"])
            tel["side_cap"] = side_cap
            if side_cap < desired_speed:
                desired_speed = side_cap
                binding = 'side'
        if follow_cap < desired_speed:
            desired_speed = follow_cap
            binding = 'follow'
        if creeping:
            binding = 'creep'
        if used_fallback and effective_corner < desired_speed:
            desired_speed = min(desired_speed, effective_corner)
            binding = 'corner'
        else:
            desired_speed = min(desired_speed, effective_corner) if used_fallback else desired_speed

        if p["speed_hold_sec"] > 0.0:
            self.speed_hold_clock += dt
            desired_speed = self.speed_hold.update(self.speed_hold_clock, desired_speed)

        ramp_basis = max(self.last_commanded_speed,
                         min(abs(float(current_speed)), p["max_speed"]))
        accel_ceiling = ramp_basis + p["max_acceleration"] * dt
        speed = min(desired_speed, accel_ceiling)
        if accel_ceiling < desired_speed:
            binding = 'accel'
        self.binding_cap = binding
        tel.update(target_angle=target_angle, target_distance=target_distance,
                   gap_width=(gap_end - gap_start + 1) * self.lidar.angle_increment,
                   fallback=used_fallback, creeping=creeping,
                   centering_bias=centering_bias, desired_steering=desired_steering,
                   curve_cap=normal_speed, clearance_cap=clearance_speed,
                   corner_cap=effective_corner, binding=binding)

        self.last_commanded_speed = speed
        self.last_command_time = step
        self.stopped = False
        return steering, float(speed)

    def _adaptive_width(self, left, right):
        p = self.p
        if not p["enable_adaptive_width"]:
            return 1.0, p["safety_margin"], p["corner_speed"]
        width_factor = gap_logic.corridor_width_factor(
            left, right, p["adaptive_width_narrow"], p["adaptive_width_reference"])
        margin = gap_logic.scale_between(
            width_factor, p["min_safety_margin"], p["safety_margin"])
        corner = gap_logic.scale_between(
            width_factor, p["corner_speed"], p["corner_speed_wide"])
        return width_factor, margin, corner

    def _stop(self, step, reason):
        self.speed_hold.reset()
        self.opponent_mode = None
        self.last_commanded_speed = 0.0
        self.last_command_time = step
        self.stopped = True
        self.stop_reason = reason
        self.binding_cap = 'stop'
        self.telemetry.update(binding='stop', stop_reason=reason)
        return self.steering_basis, 0.0
