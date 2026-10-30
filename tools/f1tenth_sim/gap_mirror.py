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
from gap_follow import launch_boost
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
    # Steering damping's gyro topics and their freshness check. The simulator
    # hands the mirror the true yaw rate every tick, so there is no stale
    # sample to time out (docs/gap-follow.md#steering-damping-gyro).
    "imu_topic", "imu_status_topic", "imu_timeout_sec",
    'fused_odom_topic', 'wheel_odom_topic', 'imu_traction_timeout_sec',
}


# Every parameter GapController reads. test_gap_mirror.py checks the YAML
# provides each one and that nothing else in the YAML goes unread.
REQUIRED_PARAMS = frozenset((
    'enable_imu_impact_stop', 'imu_impact_threshold', 'imu_impact_min_samples',
    'enable_imu_traction', 'imu_traction_slip_threshold', 'imu_traction_release_slip',
    'imu_traction_min_speed', 'imu_traction_max_hold_sec',
    'enable_imu_understeer',
    'imu_understeer_engage_ratio',
    'imu_understeer_release_ratio',
    'imu_understeer_speed_factor',
    'imu_understeer_hold_sec',
    'imu_understeer_timeout_sec',
    'imu_understeer_steering_lag_sec',
    'imu_understeer_max_steering_change',
    "adaptive_grip_ceiling",
    "adaptive_grip_cut_factor",
    "adaptive_grip_hold_ratio",
    "adaptive_grip_judge_fraction",
    "adaptive_grip_max_steering_change",
    "adaptive_grip_rise_per_s",
    "adaptive_grip_slip_ratio",
    "adaptive_grip_steering_lag_sec",
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
    "enable_adaptive_grip",
    "enable_corner_setup",
    "enable_steering_damping",
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
    "steering_damping_gain",
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
    "enable_launch_boost",
    "launch_boost_max_sec",
    "launch_boost_max_distance",
    "launch_boost_end_steering",
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
    "opponent_stopped_release_sec",
    "opponent_radius",
    "opponent_length",
    "opponent_width",
    "opponent_passing_margin",
    "opponent_prediction_horizon",
    "opponent_prediction_step",
    "opponent_follow_min_gap",
    "opponent_follow_time_gap",
    "opponent_abort_speed_margin",
    "opponent_follow_lateral",
    "opponent_relevant_lateral",
    "opponent_pass_max_steering",
    "opponent_pass_max_heading_deg",
    "opponent_max_track_speed",
    "opponent_steer_delay",
    "opponent_steer_gain",
    "opponent_steer_rate_max",
    "opponent_follow_memory_sec",
))


# Parameters added after the first real-car run of the October 2026 rework,
# with the value that reproduces the behaviour from before they existed. Lets
# an older YAML (git show <commit>:src/gap_follow/config/gap_follow.yaml) be
# replayed against the run it drove -- see tools/run_analysis/replay.py.
LEGACY_VALUES = {
    'enable_imu_impact_stop': False,
    'imu_impact_threshold': 33.7257297468959,
    'imu_impact_min_samples': 1,
    'enable_imu_traction': False,
    'imu_traction_slip_threshold': 0.246,
    'imu_traction_release_slip': 0.123,
    'imu_traction_min_speed': 0.3,
    'imu_traction_max_hold_sec': 0.5,
    'enable_imu_understeer': False,
    'imu_understeer_engage_ratio': 0.75,
    'imu_understeer_release_ratio': 0.85,
    'imu_understeer_speed_factor': 0.85,
    'imu_understeer_hold_sec': 0.2,
    'imu_understeer_timeout_sec': 0.1,
    'imu_understeer_steering_lag_sec': 0.15,
    'imu_understeer_max_steering_change': 0.05,
    "arc_guard_blocked_fallback": "freest",
    "braking_latency_sec": 0.0,
    "speed_hold_sec": 0.0,
    "steering_gain_speed_ref": 0.0,
    "narrow_corridor_speed": 1e9,  # off
    "narrow_corridor_clearance": 0.05,
    "open_corridor_clearance": 0.15,
    "ttc_path_margin": None,  # None: the old corridor, safety_margin itself
    # Steering damping (2026-10-27) did not exist: off.
    "enable_steering_damping": False,
    "steering_damping_gain": 0.0,
    # Adaptive grip (2026-10-27) did not exist: off. Its values are inert while
    # it is off, so these are the shipped defaults.
    "enable_adaptive_grip": False,
    "adaptive_grip_ceiling": 3.0,
    "adaptive_grip_rise_per_s": 0.2,
    "adaptive_grip_cut_factor": 0.85,
    "adaptive_grip_hold_ratio": 0.95,
    "adaptive_grip_slip_ratio": 0.90,
    "adaptive_grip_judge_fraction": 0.8,
    "adaptive_grip_steering_lag_sec": 0.15,
    "adaptive_grip_max_steering_change": 0.05,
    # Opponent awareness (2026-10-22) did not exist: off. Its opponent_*
    # values are then never read, but the mirror requires them present.
    "enable_opponent_awareness": False,
    **{f"opponent_{name}": getattr(opponent_tracking.OpponentParams(), name)
       for name in opponent_tracking.OpponentParams.__dataclass_fields__},
    # The follow-cap memory (2026-10-28) did not exist: 0 is no hold, the old behaviour.
    "opponent_follow_memory_sec": 0.0,
    # The launch boost (2026-10-30) did not exist: off. Its limits are inert
    # while it is off, so these are the shipped defaults.
    "enable_launch_boost": False,
    "launch_boost_max_sec": 3.0,
    "launch_boost_max_distance": 5.0,
    "launch_boost_end_steering": 0.0,
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
    measured_speed, step, yaw_rate=None, measured_steering=None,
    fused_speed=None)`` returns
    ``(steering, speed)`` for one
    40 Hz tick; ``yaw_rate`` is read when steering damping or the adaptive grip
    is on, and None means no healthy IMU sample this tick. Traction also requires
    a finite fused_speed supplied by the caller; None means no fresh fused input.

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
        # Launch boost (gap_follow/launch_boost.py). The simulator's LB is held
        # from the first tick, so the first command is the first press.
        self.launch_boost = launch_boost.LaunchBoost(
            self.p["enable_launch_boost"], self.p["launch_boost_max_sec"],
            self.p["launch_boost_max_distance"], self.p["launch_boost_end_steering"])
        # Adaptive grip, built as gap_follow_node builds it: the lateral limit
        # and the low-pass on its ratio. Read only when the flag is on.
        self.grip = gap_logic.AdaptiveGrip(
            base=self.p["max_lateral_accel"],
            ceiling=max(self.p["adaptive_grip_ceiling"], self.p["max_lateral_accel"]),
            rise_per_s=self.p["adaptive_grip_rise_per_s"],
            cut_factor=self.p["adaptive_grip_cut_factor"],
            hold_ratio=self.p["adaptive_grip_hold_ratio"],
            slip_ratio=self.p["adaptive_grip_slip_ratio"],
            judge_fraction=self.p["adaptive_grip_judge_fraction"])
        self.grip_ratio_filter = gap_logic.LowPass(gap_logic.GRIP_RATIO_TAU_SEC)
        self.steering_lag = gap_logic.SteeringLag(self.p["adaptive_grip_steering_lag_sec"])
        self.understeer_check = gap_logic.UndersteerCheck(
            self.p['imu_understeer_engage_ratio'], self.p['imu_understeer_release_ratio'],
            self.p['imu_understeer_hold_sec'])
        self.understeer_steering_lag = gap_logic.SteeringLag(
            self.p['imu_understeer_steering_lag_sec'])
        self.traction_hold = gap_logic.TractionHold(
            self.p['imu_traction_slip_threshold'], self.p['imu_traction_release_slip'],
            self.p['imu_traction_min_speed'], self.p['imu_traction_max_hold_sec'])
        self.impact_latch = gap_logic.ImpactLatch(
            self.p['imu_impact_threshold'], self.p['imu_impact_min_samples'])

    def command(self, scan, current_speed, step, yaw_rate=None, measured_steering=None,
                fused_speed=None, horizontal_accel=None, lb_pressed=True):
        """Caller supplies horizontal force (m/s²), or None when untrusted.

        Gym supplies zero unless a test injects an impact. LB defaults held for
        existing simulation callers; None is a missing Joy button, not release.
        """
        p = self.p
        tel = self.telemetry = {}
        if p['enable_imu_impact_stop']:
            self.impact_latch.on_lb(lb_pressed)
            trusted = (horizontal_accel is not None and yaw_rate is not None
                       and math.isfinite(yaw_rate))
            self.impact_latch.on_sample(
                horizontal_accel if horizontal_accel is not None else math.nan,
                0.0, trusted)
        if lb_pressed is not True:
            return self._stop(step, 'deadman_released' if lb_pressed is False
                              else 'waiting_for_joy')
        if self.impact_latch.latched:
            return self._stop(step, 'imu_impact_stop')
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
            if (fused_speed is not None and math.isfinite(fused_speed)
                    and yaw_rate is not None and math.isfinite(yaw_rate)):
                # current_speed is raw wheels; an optional trusted fused
                # sample can only add vigilance. Combine fallback results,
                # never measurements, just as the node's fused path does.
                fused_effective_speed = gap_logic.conservative_ttc_speed(
                    fused_speed, self.last_commanded_speed, command_age,
                    p["ttc_command_speed_timeout_sec"],
                    p["ttc_command_fallback_max_odom_speed"])
                effective_speed = max(effective_speed, fused_effective_speed)
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
        alongside_cap = math.inf
        self.opponent_mode = None
        # gap_follow_node: opponents ignored while the launch runs, or on the
        # driving tick that starts it (not creeping, not an escape).
        launching = self.launch_boost.active or (
            self.launch_boost.armed and not creeping
            and not (emergency_tripped or ttc_tripped))
        if launching:
            # gap_follow_node: opponents ignored for the start, plain search.
            processed, gap_start, gap_end, used_fallback = select(window)
            if self.opponents is not None:
                self.opponent_mode = "launch"
        elif self.opponents is not None and self.opponents.moving_body():
            avoid = select(self.opponents.augment(window, beam_angles))
            plain = select(window)
            # Use the avoiding gap only to pick a side within the opening
            # the car would take anyway: it must overlap the plain gap and be
            # no shallower. Anything else -- no gap, a fallback where the
            # real scan offers a deep one, a different opening altogether --
            # means follow instead.
            # And only on a straight (opponent_tracking.pass_allowed).
            passing = (self.opponents.may_pass(self.steering_basis)
                       and opponent_tracking.avoidance_usable(
                           avoid[1:], plain[1:],
                           gap_width=self.opponents.passing_gap_width(
                               window, beam_angles, avoid[1:]),
                           required_width=p["car_width"]
                           + 2 * self.opponents.params.passing_margin))
            if passing:
                processed, gap_start, gap_end, used_fallback = avoid
                self.opponent_mode = "clear"
            else:
                processed, gap_start, gap_end, used_fallback = plain
                self.opponent_mode = "follow"
            follow_cap = self.opponents.follow_law_cap(passing)
            alongside_cap = self.opponents.alongside_cap(passing)
            tel["follow_cap"] = min(follow_cap, alongside_cap)
        else:
            processed, gap_start, gap_end, used_fallback = select(window)

        if (self.opponents is not None and self.opponent_mode is None
                and not launching and self.opponents.followed_body()):
            self.opponent_mode = "follow"
            follow_cap = self.opponents.follow_law_cap(passing=False)
            alongside_cap = self.opponents.alongside_cap(passing=False)
            tel["follow_cap"] = min(follow_cap, alongside_cap)

        # The follow cap outlives its car for opponent_follow_memory_sec, as in
        # gap_follow_node: speed only, on the same tick, before the no_safe_gap stop.
        # The alongside-abort ceiling is added after the memory, never held by it.
        if self.opponents is not None:
            follow_cap, held = self.opponents.follow_memory.update(
                follow_cap, step * CONTROL_DT)
            follow_cap = min(follow_cap, alongside_cap)
            if held:
                self.opponent_mode = "follow_memory"
                tel["follow_cap"] = follow_cap

        if gap_start is None:
            return self._stop(step, "no_safe_gap")

        if emergency_tripped or ttc_tripped:
            creeping = True
            self.launch_boost.end("emergency_escape" if emergency_tripped
                                  else "ttc_escape")

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
        # Steering damping (gap_follow_node._steering_damping_term): minus the
        # gain times the gyro's yaw rate. yaw_rate None means no healthy IMU
        # sample this tick, which the car answers with exactly zero.
        damping = 0.0
        if p["enable_steering_damping"] and yaw_rate is not None:
            damping = gap_logic.steering_damping(yaw_rate, p["steering_damping_gain"])
        desired_steering = float(np.clip(
            bearing_gain * target_angle + centering_bias + setup_bias + damping,
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

        # gap_follow_node: during the launch, hold straight (damping only)
        # while the body's straight path is clear for the stopping distance.
        tel["launch_straight"] = False
        if launching and not creeping:
            straight_room = gap_logic.swept_path_clearance(
                window, window_valid, beam_angles, 0.0,
                p["car_width"] / 2.0 + p["swept_clearance_margin"],
                p["car_length"], p["wheelbase"], p["laser_offset_x"],
                p["laser_offset_y"], p["swept_clearance_max_length"])
            basis = max(self.last_commanded_speed,
                        min(abs(float(current_speed)), p["max_speed"]))
            needed = min(p["swept_clearance_max_length"], max(
                p["arc_guard_length"],
                basis ** 2 / (2.0 * p["max_braking_decel"])
                + p["forward_stop_clearance"]))
            if straight_room >= needed:
                desired_steering = float(np.clip(
                    damping, -p["max_steering_angle"], p["max_steering_angle"]))
                tel["launch_straight"] = True

        dt = CONTROL_DT
        if self.last_command_time is not None:
            dt = min(p["command_slew_max_dt"],
                     max(0.0, (step - self.last_command_time) * CONTROL_DT))
        else:
            dt = p["command_slew_max_dt"]

        steering = gap_logic.slew_rate_limit(
            desired_steering, self.steering_basis, dt, p["max_steering_rate"])
        self.steering_basis = steering

        lateral_limit = self._lateral_limit(steering, dt, current_speed, yaw_rate)
        limited_curvature = math.tan(desired_steering) / p["wheelbase"]
        curve_speed = gap_logic.curvature_speed_limit(
            limited_curvature, lateral_limit, p["max_speed"])
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
        understeer_cap = self._understeer_cap(
            steering, dt, current_speed, yaw_rate, measured_steering)
        tel['imu_understeer_engaged'] = math.isfinite(understeer_cap)
        tel['imu_understeer_cap'] = understeer_cap
        if launching and clearance_speed > float(current_speed):
            # As in the node: only a tick with room to speed up starts it.
            self.launch_boost.begin()
        launching = self.launch_boost.step(
            dt, float(current_speed), desired_steering, clearance_speed)
        tel["launch_boost"] = launching
        if launching:
            # gap_follow_node's launch boost: only the curvature, braking and
            # understeer ceilings; no corridor, follow or corner cap, speed
            # hold, ramp or traction hold.
            if creeping:
                binding = 'creep'
        else:
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
        # An untrusted sensor must not leave a cap behind in the speed hold.
        if understeer_cap < desired_speed:
            desired_speed = understeer_cap
            binding = 'understeer'

        off_basis = max(self.last_commanded_speed,
                        min(abs(float(current_speed)), p['max_speed']))
        traction_off_ceiling = off_basis + p['max_acceleration'] * dt
        holding = False
        basis_speed = abs(float(current_speed))
        if p['enable_imu_traction']:
            trusted = (yaw_rate is not None and math.isfinite(yaw_rate)
                       and fused_speed is not None and math.isfinite(fused_speed)
                       and math.isfinite(current_speed))
            if trusted:
                holding = self.traction_hold.update(current_speed, fused_speed, dt)
                if (current_speed >= p['imu_traction_min_speed']
                        and fused_speed >= p['imu_traction_min_speed']):
                    basis_speed = fused_speed
            else:
                self.traction_hold.reset()
        tel['imu_traction_holding'] = holding
        if holding:
            speed = gap_logic.traction_limited_speed(desired_speed, self.last_commanded_speed, True)
            binding = 'traction'
        else:
            ramp_basis = max(self.last_commanded_speed, min(basis_speed, p['max_speed']))
            accel_ceiling = ramp_basis + p["max_acceleration"] * dt
            speed = min(desired_speed, accel_ceiling)
            if accel_ceiling < desired_speed:
                binding = 'accel'
        if traction_off_ceiling < speed:
            binding = 'traction_off'
        speed = min(speed, traction_off_ceiling)
        if launching:
            # As in the node: no ramp and no traction hold during the launch.
            speed = desired_speed
            if binding in ('accel', 'traction', 'traction_off'):
                binding = 'curve' if normal_speed <= clearance_speed else 'clearance'
        self.binding_cap = binding
        tel.update(target_angle=target_angle, target_distance=target_distance,
                   gap_width=(gap_end - gap_start + 1) * self.lidar.angle_increment,
                   fallback=used_fallback, creeping=creeping,
                   centering_bias=centering_bias, steering_damping=damping,
                   lateral_limit=lateral_limit,
                   desired_steering=desired_steering,
                   requested_steering=steering,  # after the slew limit: what this tick returns
                   curve_cap=normal_speed, clearance_cap=clearance_speed,
                   corner_cap=effective_corner, binding=binding)

        self.last_commanded_speed = speed
        self.last_command_time = step
        self.stopped = False
        return steering, float(speed)

    def _understeer_cap(self, steering, dt, current_speed, yaw_rate, measured_steering):
        """Fresh samples arrive per tick; None is an untrusted/missing sample."""
        p = self.p
        if not p['enable_imu_understeer']:
            return math.inf
        lagged = self.understeer_steering_lag.step(
            dt, steering, p['imu_understeer_max_steering_change'])
        trusted = (yaw_rate is not None and math.isfinite(yaw_rate)
                   and measured_steering is not None)
        ratio = (gap_logic.understeer_ratio(measured_steering, lagged, current_speed)
                 if trusted else math.nan)
        engaged = self.understeer_check.update(ratio, dt)
        if not math.isfinite(ratio):
            return math.inf
        return gap_logic.understeer_speed_cap(
            engaged, current_speed, p['imu_understeer_speed_factor'])

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

    def _lateral_limit(self, steering, dt, current_speed, yaw_rate):
        """gap_follow_node._lateral_limit, as the simulator runs it. The
        simulator's odometry is always fresh, so the limit restarts only when
        the gyro is untrusted (yaw_rate None) or max_lateral_accel changes."""
        p = self.p
        if not p["enable_adaptive_grip"]:
            return p["max_lateral_accel"]
        self.grip.follow_base(p["max_lateral_accel"], p["adaptive_grip_ceiling"])
        judged = self.steering_lag.step(
            dt, steering, p["adaptive_grip_max_steering_change"])
        if yaw_rate is None:
            self.grip.limit = self.grip.base
            self.grip_ratio_filter.reset()
            return self.grip.limit
        return gap_logic.adaptive_grip_step(
            self.grip, self.grip_ratio_filter, yaw_rate, float(current_speed),
            judged, p["wheelbase"], dt)

    def _stop(self, step, reason):
        self.speed_hold.reset()
        self.launch_boost.end(reason)
        self.opponent_mode = None
        self.last_commanded_speed = 0.0
        self.last_command_time = step
        self.stopped = True
        self.stop_reason = reason
        self.binding_cap = 'stop'
        self.telemetry.update(binding='stop', stop_reason=reason,
                              requested_steering=self.steering_basis)
        return self.steering_basis, 0.0
