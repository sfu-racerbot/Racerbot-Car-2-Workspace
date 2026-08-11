import math

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Joy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String

from drive_intent import predict, schema
from drive_intent.throttle import FailureLatch, IntentThrottle
from gap_follow import gap_logic, live_tuning


class GapFollowNode(Node):
    """Reactive 'follow the gap' driver: steers at the middle of the best
    obstacle-free gap in the LIDAR scan, with no map or localization
    needed. All the scan-processing math lives in gap_logic.py (importable
    and unit-tested without rclpy); this class owns the ROS plumbing and
    the LB deadman gate, and composes the gap_logic pipeline:
    sanitize -> footprint clearance/TTC -> disparity extend -> safety
    bubble -> deep gap or tight-corner fallback -> steer at its middle.
    """

    def __init__(self):
        super().__init__('gap_follow_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('forward_fov_deg', 180.0)
        self.declare_parameter('min_gap_distance', 1.0)
        self.declare_parameter('fallback_min_gap_distance', 0.8)
        # Gap-selection hysteresis: how much better a new candidate must
        # score than the one containing the previous tick's aim before the
        # car re-aims at it. 1.0 disables this (plain best score, every
        # tick, independent of the last one). >1.0 is a bounded fix for a
        # specific failure class: two similarly-open candidates trading the
        # top score by a beam of LiDAR noise flips the aim -- and therefore
        # the steering command -- at scan rate. That is exactly the shape
        # of the 2026-07-27 incident aim_within_gap's docstring describes
        # (there, a FOV-clipped edge caused it; here, ordinary near-tied
        # candidates can too), and a real-world report of the car "driving
        # kind of drunk, side to side" on a narrow course pointed back at
        # this same undamped discrete choice -- gap selection has no
        # equivalent of corridor_centering_bias's fade-in/out ramps.
        # 1.2 was chosen, not measured: it is a bounded, easily-tuned first
        # answer (a candidate needs a genuine 20% edge to win the car away
        # from where it was already headed), not a value backed by a real
        # weaving log -- see gap_logic.find_best_gap. Validate low-speed,
        # wheels off the ground, before trusting this on the floor, same as
        # any other new driving parameter.
        self.declare_parameter('gap_switch_margin', 1.2)
        self.declare_parameter('corner_speed', 0.8)
        self.declare_parameter('max_speed', 2.5)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_steering_angle', 0.26)
        # Dynamic control: bearing-proportional steering, physical speed
        # ceilings, and bounded normal command changes. Stops bypass shaping.
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('max_lateral_accel', 1.0)
        self.declare_parameter('max_acceleration', 3.0)
        self.declare_parameter('max_braking_decel', 3.0)
        self.declare_parameter('max_steering_rate', 1.0)
        self.declare_parameter('command_slew_max_dt', 0.10)
        # Padded Traxxas 74276-4 footprint (physical: 0.281 x 0.535 m),
        # combined with its rear-axle base_link and estimated LiDAR transform.
        self.declare_parameter('car_width', 0.31)
        self.declare_parameter('car_length', 0.58)
        self.declare_parameter('wheelbase', 0.324)
        self.declare_parameter('laser_offset_x', 0.33)
        self.declare_parameter('laser_offset_y', 0.0)
        self.declare_parameter('safety_margin', 0.18)
        # Live corridor-width adaptation: raise corner_speed's ceiling (and,
        # if configured to, shrink safety_margin) on a sensed-wide corner,
        # using the same side_wall_distance measurement corridor centering
        # already takes every tick. See _select_gap and
        # docs/asb-10000-sim-results.json.
        #
        # Measured on the ASB 10000-level course's real 1.5m hallway: the
        # corner_speed_wide ceiling below is a clean win on a genuinely wide
        # course (indoor_wide: 0 wall contact either way, ~9% higher average
        # speed) with no downside, because it never touches the margin.
        # Shrinking the margin itself is a *different* trade, and measured
        # worse on this specific course -- a uniformly narrow hallway, not
        # narrow-in-spots, senses as "narrow" almost continuously rather
        # than only where it would actually unlock a gap, so it just drove
        # the car permanently closer to the walls: wall-contact steps rose
        # 209 -> 275 with no speed or distance gain, because what actually
        # limited this course was corner curvature near the car's own
        # turning circle, not a margin-driven "no gap found" stop. So
        # min_safety_margin below defaults to safety_margin -- shrinking is
        # a no-op out of the box -- kept live rather than deleted because a
        # course narrow-in-spots-only (unlike this one) is exactly the case
        # test_smaller_effective_margin_recovers_a_gap_a_static_one_would_
        # miss (test_gap_follow_node.py) demonstrates it can still help.
        self.declare_parameter('enable_adaptive_width', True)
        # Sensed corridor width (left_distance + right_distance) at or below
        # which corner_speed tops out at corner_speed_wide (and, if
        # min_safety_margin has been lowered below safety_margin in a
        # per-course config, the margin bottoms out there too). 1.4m is
        # indoor_tight's corridor -- the narrowest layout already validated
        # in the simulator (src/racerbot_sim/racerbot_sim/tracks.py).
        self.declare_parameter('adaptive_width_narrow', 1.4)
        # Sensed width at or above which nothing changes: safety_margin and
        # corner_speed are exactly today's static values. 2.6m is
        # indoor_wide's corridor -- the one those defaults are already
        # tuned against (docs/ros-simulator.md).
        self.declare_parameter('adaptive_width_reference', 2.6)
        # The floor safety_margin may shrink to. Defaults to safety_margin
        # itself -- shrinking is off out of the box, see the comment above
        # enable_adaptive_width. If a specific course's config lowers this,
        # keep it well short of the old pre-0.18 regime: safety_margin's own
        # comment below records that a smaller margin than today's default
        # previously left the car "clipping corner apexes and then sitting
        # ~0.08m off the wall".
        self.declare_parameter('min_safety_margin', 0.18)
        # The ceiling corner_speed may rise to on a sensed-wide corner (one
        # the fallback gap took for lack of forward visibility, not for lack
        # of room). corner_speed itself remains the floor -- today's already
        # -tuned narrow-corner behaviour never gets slower because of this.
        self.declare_parameter('corner_speed_wide', 1.4)
        self.declare_parameter('disparity_threshold', 0.4)
        # Obstacle inflation already accounts for the full car width. This
        # threshold is only the remaining centerline corridor after inflation.
        self.declare_parameter('min_centerline_gap_width', 0.10)
        self.declare_parameter('emergency_stop_clearance', 0.02)
        self.declare_parameter('forward_stop_clearance', 0.25)
        self.declare_parameter('forward_stop_fov_deg', 60.0)
        # Crawl allowed once inside forward_stop_clearance, so a car that has
        # found its way out of a corner can take it instead of latching.
        self.declare_parameter('escape_creep_speed', 0.25)

        # Cornering anticipation: an extra speed cap from how much the
        # chosen gap's near-only bearing (gap_logic.near_gap_bearing)
        # disagrees with aim_within_gap's own choice -- the whole gap's
        # angular midpoint in the ordinary case, which is what actually
        # steers. Converts that disagreement into a curvature the same way
        # real steering does (steering_gain, then tan(.)/wheelbase) and
        # caps speed the same way curvature_speed_limit already caps it for
        # the real command, so this adds no new physics, only an earlier
        # look. Added after a real-hallway report of corners taken "too
        # slowly and too tightly" with no anticipatory slow-down/speed-up
        # at all -- see docs/asb-10000-sim-results.json.
        #
        # Ships OFF: measured on indoor_wide, this capped speed on 49 of 65
        # driving ticks and was the actually-binding cap on most of them,
        # in a wide-open, gently-curving room with zero wall contact either
        # way -- an average speed drop of about 25% versus the same run
        # with only the corner_speed_wide change below, for no matching
        # safety benefit there. A wide gap's own angular span naturally
        # spans tens of degrees toward whichever edge is more open, which
        # near_gap_bearing's "whole gap vs near-only" comparison reads as
        # "a bend is coming" even in a straight, open room -- it needs a
        # better-normalized trigger (e.g. scaled by the gap's own angular
        # width) before this is worth enabling, not just a real-hallway
        # weaving report to justify writing it in the first place. The
        # mechanism and its tests stay in place for that future tuning;
        # turning this on again is a code change, not a config flip to
        # make lightly.
        self.declare_parameter('enable_cornering_anticipation', False)
        # How close counts as "near" for the comparison above. Deliberately
        # *above* min_gap_distance (2.0m default), not below it: every beam
        # inside a preferred-tier gap already exceeds min_gap_distance by
        # construction, so a near_depth under that threshold could never
        # match any of them -- near_gap_bearing would always return None
        # and this would silently do nothing until the car was already in
        # the fallback gap, i.e. already committed to the tight turn,
        # defeating the entire point of anticipating it first. 2.5m
        # matches centering_full_forward_depth's existing "is there a
        # genuine stretch ahead" threshold below.
        self.declare_parameter('anticipation_near_depth', 2.5)

        # Corridor centering: the cross-track half of a lane-centring law,
        # faded in only while the car is running straight down a corridor with
        # a wall on each side. See gap_logic.corridor_centering_bias.
        self.declare_parameter('enable_centering', True)
        self.declare_parameter('centering_gain', 0.25)
        self.declare_parameter('centering_max_steering', 0.08)
        self.declare_parameter('centering_side_fov_deg', 60.0)
        self.declare_parameter('centering_full_bearing_deg', 4.0)
        self.declare_parameter('centering_zero_bearing_deg', 15.0)
        self.declare_parameter('centering_full_forward_depth', 2.5)
        self.declare_parameter('centering_zero_forward_depth', 1.5)
        self.declare_parameter('centering_full_side_distance', 4.0)
        self.declare_parameter('centering_zero_side_distance', 5.0)

        # F1TENTH instantaneous TTC, using the safest recent speed estimate.
        self.declare_parameter('enable_ttc', True)
        self.declare_parameter('ttc_threshold_sec', 0.35)
        self.declare_parameter('ttc_min_closing_speed', 0.05)
        self.declare_parameter('ttc_command_speed_timeout_sec', 0.5)
        self.declare_parameter('ttc_command_fallback_max_odom_speed', 0.10)
        self.declare_parameter('ttc_min_brake_speed', 0.6)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('joy_timeout_sec', 0.5)
        # Diagnostics are deliberately slower than the LIDAR callback:
        # state changes are logged immediately, then the current decision is
        # repeated at this interval so launch output stays readable.
        self.declare_parameter('decision_log_period_sec', 1.0)
        # Gap follow is callback-driven. If scans stop, /drive naturally
        # times out at the mux; this timeout lets the status timer explain
        # that otherwise-silent stop in the launch terminal.
        self.declare_parameter('scan_timeout_sec', 0.5)
        # --- Drive intent (docs/drive-intent.md) ---
        # A read-only JSON description of what this controller is *trying*
        # to do, for the web dashboard's intent arrow and decision panel.
        # Publishing it cannot change what the car does -- see
        # _publish_intent() for the three rules that guarantee that.
        self.declare_parameter('publish_intent', True)
        self.declare_parameter('intent_topic', '/drive_intent')
        # Deliberately well under the scan rate. No browser benefits from
        # redrawing an arrow 40 times a second, and this shares an 8GB
        # Jetson with the code driving the car.
        self.declare_parameter('intent_rate_hz', 20.0)
        # How far ahead the arrow predicts: long enough to show the plan
        # through a corner, short enough that it stays a claim about *now*
        # rather than a lap projection.
        self.declare_parameter('intent_horizon_sec', 1.5)
        self.declare_parameter('intent_samples', 16)
        self.declare_parameter('intent_max_length', 8.0)
        # Workspace policy (see docs/architecture.md): the deadman button
        # stays enabled until the team has enough confidence in the car's
        # behavior to deliberately relax it -- don't set this false otherwise.
        self.declare_parameter('enable_deadman', True)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.max_range = self.get_parameter('max_range').value
        self.forward_fov = math.radians(self.get_parameter('forward_fov_deg').value)
        self.min_gap_distance = float(
            self.get_parameter('min_gap_distance').value)
        self.fallback_min_gap_distance = float(
            self.get_parameter('fallback_min_gap_distance').value)
        self.gap_switch_margin = float(
            self.get_parameter('gap_switch_margin').value)
        self.corner_speed = float(
            self.get_parameter('corner_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value)
        self.steering_gain = float(
            self.get_parameter('steering_gain').value)
        self.max_lateral_accel = float(
            self.get_parameter('max_lateral_accel').value)
        self.max_acceleration = float(
            self.get_parameter('max_acceleration').value)
        self.max_braking_decel = float(
            self.get_parameter('max_braking_decel').value)
        self.max_steering_rate = float(
            self.get_parameter('max_steering_rate').value)
        self.command_slew_max_dt = float(
            self.get_parameter('command_slew_max_dt').value)
        self.car_width = float(self.get_parameter('car_width').value)
        self.car_length = float(self.get_parameter('car_length').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.laser_offset_x = float(
            self.get_parameter('laser_offset_x').value)
        self.laser_offset_y = float(
            self.get_parameter('laser_offset_y').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.enable_adaptive_width = bool(
            self.get_parameter('enable_adaptive_width').value)
        self.adaptive_width_narrow = float(
            self.get_parameter('adaptive_width_narrow').value)
        self.adaptive_width_reference = float(
            self.get_parameter('adaptive_width_reference').value)
        self.min_safety_margin = float(
            self.get_parameter('min_safety_margin').value)
        self.corner_speed_wide = float(
            self.get_parameter('corner_speed_wide').value)
        self.disparity_threshold = float(
            self.get_parameter('disparity_threshold').value)
        self.min_centerline_gap_width = float(
            self.get_parameter('min_centerline_gap_width').value)
        self.emergency_stop_clearance = float(
            self.get_parameter('emergency_stop_clearance').value)
        self.forward_stop_clearance = float(
            self.get_parameter('forward_stop_clearance').value)
        self.forward_stop_fov = math.radians(float(
            self.get_parameter('forward_stop_fov_deg').value))
        self.escape_creep_speed = float(
            self.get_parameter('escape_creep_speed').value)
        self.enable_cornering_anticipation = bool(
            self.get_parameter('enable_cornering_anticipation').value)
        self.anticipation_near_depth = float(
            self.get_parameter('anticipation_near_depth').value)
        self.enable_centering = bool(
            self.get_parameter('enable_centering').value)
        self.centering_gain = float(
            self.get_parameter('centering_gain').value)
        self.centering_max_steering = float(
            self.get_parameter('centering_max_steering').value)
        self.centering_side_half_span = math.radians(float(
            self.get_parameter('centering_side_fov_deg').value)) / 2.0
        self.centering_full_bearing = math.radians(float(
            self.get_parameter('centering_full_bearing_deg').value))
        self.centering_zero_bearing = math.radians(float(
            self.get_parameter('centering_zero_bearing_deg').value))
        self.centering_full_forward_depth = float(
            self.get_parameter('centering_full_forward_depth').value)
        self.centering_zero_forward_depth = float(
            self.get_parameter('centering_zero_forward_depth').value)
        self.centering_full_side_distance = float(
            self.get_parameter('centering_full_side_distance').value)
        self.centering_zero_side_distance = float(
            self.get_parameter('centering_zero_side_distance').value)
        self.enable_ttc = bool(self.get_parameter('enable_ttc').value)
        self.ttc_threshold_sec = float(
            self.get_parameter('ttc_threshold_sec').value)
        self.ttc_min_closing_speed = float(
            self.get_parameter('ttc_min_closing_speed').value)
        self.ttc_command_speed_timeout_sec = float(
            self.get_parameter('ttc_command_speed_timeout_sec').value)
        self.ttc_command_fallback_max_odom_speed = float(self.get_parameter(
            'ttc_command_fallback_max_odom_speed').value)
        self.ttc_min_brake_speed = float(
            self.get_parameter('ttc_min_brake_speed').value)
        self.odom_timeout_sec = float(
            self.get_parameter('odom_timeout_sec').value)
        self.joy_topic = self.get_parameter('joy_topic').value
        self.deadman_button = self.get_parameter('deadman_button').value
        self.joy_timeout_sec = self.get_parameter('joy_timeout_sec').value
        self.decision_log_period_sec = max(
            0.0, float(self.get_parameter('decision_log_period_sec').value))
        self.scan_timeout_sec = max(
            0.05, float(self.get_parameter('scan_timeout_sec').value))
        self.enable_deadman = bool(self.get_parameter('enable_deadman').value)
        self.publish_intent = bool(self.get_parameter('publish_intent').value)
        self.intent_topic = self.get_parameter('intent_topic').value
        self.intent_rate_hz = float(self.get_parameter('intent_rate_hz').value)
        self.intent_horizon_sec = max(
            0.05, float(self.get_parameter('intent_horizon_sec').value))
        self.intent_samples = max(
            2, int(self.get_parameter('intent_samples').value))
        self.intent_max_length = max(
            0.1, float(self.get_parameter('intent_max_length').value))

        # Validate the footprint and sensor origin once at startup.
        gap_logic.vehicle_boundary_distances(
            np.array([0.0]),
            self.car_width,
            self.car_length,
            self.wheelbase,
            self.laser_offset_x,
            self.laser_offset_y,
        )
        if not math.isfinite(self.forward_stop_clearance) or (
                self.forward_stop_clearance < self.emergency_stop_clearance):
            raise ValueError(
                'forward_stop_clearance must be finite and no smaller than '
                'emergency_stop_clearance')
        # The creep is the one speed allowed inside the forward reserve, so it
        # has to stay a crawl. Bounding it by min_speed keeps it from silently
        # becoming the ordinary driving speed if someone tunes it upward.
        if not (math.isfinite(self.escape_creep_speed)
                and 0.0 < self.escape_creep_speed <= max(self.min_speed, 0.5)):
            raise ValueError(
                'escape_creep_speed must be finite, positive, and no greater '
                'than max(min_speed, 0.5) -- it is a crawl, not a drive speed')
        if not math.isfinite(self.forward_stop_fov) or not (
                0.0 < self.forward_stop_fov <= self.forward_fov):
            raise ValueError(
                'forward_stop_fov_deg must be positive and no wider than '
                'forward_fov_deg')
        if not (math.isfinite(self.anticipation_near_depth)
                and self.anticipation_near_depth > 0.0):
            raise ValueError('anticipation_near_depth must be finite and positive')
        dynamic_limits = (
            self.steering_gain,
            self.max_lateral_accel,
            self.max_acceleration,
            self.max_braking_decel,
            self.max_steering_rate,
            self.command_slew_max_dt,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in dynamic_limits):
            raise ValueError('dynamic steering/speed limits must be finite and positive')
        if not (math.isfinite(self.gap_switch_margin) and self.gap_switch_margin >= 1.0):
            raise ValueError(
                'gap_switch_margin must be finite and at least 1.0 -- below that, a '
                'strictly worse candidate could win, which is backwards')
        if not (math.isfinite(self.min_speed) and math.isfinite(self.max_speed)
                and 0.0 <= self.min_speed <= self.max_speed):
            raise ValueError('speed limits must be finite with 0 <= min_speed <= max_speed')
        # The upper bound is not cosmetic. Speed capping recovers curvature as
        # tan(delta)/wheelbase; at exactly pi/2 that is ~1e16 (the car would be
        # capped to a standstill forever) and past pi/2 tan goes negative, so
        # the curvature silently inverts. Both fail quietly, so reject them.
        if not (math.isfinite(self.max_steering_angle)
                and 0.0 < self.max_steering_angle < math.pi / 2.0):
            raise ValueError(
                'max_steering_angle must be finite and in (0, pi/2) radians')
        if self.enable_centering:
            self._validate_centering_parameters()
        if self.enable_adaptive_width:
            self._validate_adaptive_width_parameters()
        if not math.isfinite(self.ttc_command_speed_timeout_sec) or (
                self.ttc_command_speed_timeout_sec < 0.0):
            raise ValueError(
                'ttc_command_speed_timeout_sec must be finite and non-negative')
        if not math.isfinite(self.ttc_command_fallback_max_odom_speed) or (
                self.ttc_command_fallback_max_odom_speed < 0.0):
            raise ValueError(
                'ttc_command_fallback_max_odom_speed must be finite and '
                'non-negative')
        if not math.isfinite(self.ttc_min_brake_speed) or (
                self.ttc_min_brake_speed < 0.0):
            raise ValueError(
                'ttc_min_brake_speed must be finite and non-negative')

        # Deadman state: gap_follow only drives while this button is held on
        # a live /joy stream. Defaults to "not engaged" so the car never
        # drives before a held-button signal has actually been seen.
        self.deadman_held = False
        self.last_joy_time = None
        self.joy_button_available = False
        self.current_speed = 0.0
        self.last_odom_time = None
        self.last_commanded_speed = 0.0
        # Basis the steering slew limiter rate-limits away from. Deliberately
        # NOT "the last steering angle published": a safety stop publishes 0
        # instantly, and feeding that back in lets a single transient stop
        # collapse the next steering command to max_steering_rate*dt. See
        # _stop() for the failure that caused.
        self.steering_basis = 0.0
        # Window-relative index of the last tick's chosen aim, for gap-
        # selection hysteresis (see gap_logic.find_best_gap). Only ever
        # updated when a gap was actually found -- a tick that stops for an
        # unrelated reason (TTC, clearance) leaves it pointing at the last
        # known-good aim rather than resetting, so resuming still prefers
        # continuity with it.
        self.previous_target_idx = None
        # Last measured corridor, for the decision log only.
        self.centering_left_distance = math.inf
        self.centering_right_distance = math.inf
        self.last_command_time = None
        # One-shot latch for the odometry direction warning below.
        self.odom_direction_warned = False

        # Runtime diagnostics. The scan watchdog is informational: when a
        # callback-driven controller stops receiving scans it publishes no
        # new command, and ackermann_mux stops the car when /drive times out.
        self.last_scan_time = None
        self.last_decision_state = None
        self.last_decision_log_time = None
        # Intent throttles on its own clock, independent of the decision
        # log, and latches itself off if it ever starts failing.
        self._intent_throttle = IntentThrottle(
            self.intent_rate_hz, self.decision_log_period_sec)
        self._intent_failures = FailureLatch()

        # Live tuning: publish the catalogue of parameters this node will
        # accept changes to *while driving*, so the web dashboard can build
        # its panel from the node itself rather than from a hardcoded copy
        # of this list that would quietly rot. Read-only: the catalogue
        # describes what may change, and is not itself one of them.
        self._tunables = live_tuning.by_name(live_tuning.GAP_FOLLOW_TUNABLES)
        # Parameter-unit values (never the transformed attribute -- e.g.
        # forward_stop_fov_deg, not the radians it is stored as), so the
        # cross-parameter invariants always compare like with like.
        # Includes the read-only context values those invariants need.
        self._tunable_values = {
            name: self.get_parameter(name).value
            for name in tuple(self._tunables) + live_tuning.GAP_FOLLOW_INVARIANT_CONTEXT
        }
        self.declare_parameter(
            'live_tunable_spec',
            live_tuning.spec_json('gap_follow_node', live_tuning.GAP_FOLLOW_TUNABLES),
            ParameterDescriptor(
                read_only=True,
                description='JSON catalogue of the parameters this node can '
                            'apply live. Read by web_dashboard; see '
                            'gap_follow/live_tuning.py.'))
        # Registered only after every attribute it writes has been set and
        # validated above, so a change can never land on a half-built node.
        self.add_on_set_parameters_callback(self._parameter_callback)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.intent_pub = (
            self.create_publisher(String, self.intent_topic, 10)
            if self.publish_intent else None)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, self.joy_topic, self.joy_callback, 10)
        self.status_timer = self.create_timer(
            min(0.25, self.scan_timeout_sec / 2.0), self._sensor_status_callback)

        self.get_logger().info(
            f"gap_follow_node ready: scan='{self.scan_topic}' -> drive='{self.drive_topic}', "
            f"speed={self.min_speed:.2f}-{self.max_speed:.2f}m/s, "
            f"deadman {'ENABLED (LB must be held)' if self.enable_deadman else 'DISABLED'}, "
            f"decision logs every {self.decision_log_period_sec:.1f}s "
            "(plus immediate state changes).")

    def _parameter_callback(self, parameters):
        """Apply a live tune, or refuse it.

        Anything outside the live-tunable catalogue is *refused*, not
        ignored. This node caches every parameter on an attribute at
        startup (see __init__), so accepting a change it does not know how
        to apply would update the value the parameter server reports while
        the scan callback kept driving on the old one -- a dashboard
        reading back "max_speed: 1.0" from a car still doing 2.0.
        Rejecting says so out loud instead. See live_tuning.py.
        """
        requested = {parameter.name: parameter.value for parameter in parameters}
        accepted, reason = live_tuning.review(
            self._tunables, requested, self._tunable_values,
            passthrough=('use_sim_time',),
            invariants=live_tuning.GAP_FOLLOW_INVARIANTS)
        if reason is not None:
            self.get_logger().warn(f'live tune rejected: {reason}')
            return SetParametersResult(successful=False, reason=reason)

        for name, value in accepted.items():
            tunable = self._tunables[name]
            previous = self._tunable_values[name]
            setattr(self, tunable.target_attr, tunable.store(value))
            self._tunable_values[name] = value
            self.get_logger().info(f'live tune: {name} {previous} -> {value}')
        return SetParametersResult(successful=True)

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        self.joy_button_available = len(msg.buttons) > self.deadman_button
        if self.joy_button_available:
            self.deadman_held = bool(msg.buttons[self.deadman_button])
        else:
            self.deadman_held = False

    def odom_callback(self, msg: Odometry):
        self.current_speed = float(msg.twist.twist.linear.x)
        self.last_odom_time = self.get_clock().now()

    def _odom_fresh(self) -> bool:
        if self.last_odom_time is None or not math.isfinite(self.current_speed):
            return False
        age_sec = (
            self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        return age_sec < self.odom_timeout_sec

    def _effective_ttc_speed(self):
        command_age_sec = math.inf
        if self.last_command_time is not None:
            command_age_sec = (
                self.get_clock().now() - self.last_command_time
            ).nanoseconds / 1e9

        recent_command_speed = 0.0
        if (
            self.last_commanded_speed > 0.0
            and 0.0 <= command_age_sec <= self.ttc_command_speed_timeout_sec
        ):
            recent_command_speed = self.last_commanded_speed
        effective_speed = gap_logic.conservative_ttc_speed(
            self.current_speed,
            self.last_commanded_speed,
            command_age_sec,
            self.ttc_command_speed_timeout_sec,
            self.ttc_command_fallback_max_odom_speed,
        )
        return effective_speed, recent_command_speed

    def _check_odom_direction(self):
        """Warn once if /odom reports motion opposing the commanded direction.

        Every speed-aware layer here reads /odom: TTC braking and the
        acceleration ramp. A sign-inverted odometry source does not fail
        loudly -- it reports a forward-driving car as reversing, which reads
        downstream as "no collision risk" and quietly degrades the brake. Say
        so in the terminal instead of letting it present as a tuning problem.
        """
        if self.odom_direction_warned or not self._odom_fresh():
            return
        if self.last_commanded_speed > 0.5 and self.current_speed < -0.5:
            self.odom_direction_warned = True
            self.get_logger().error(
                f"ODOMETRY DIRECTION: commanding "
                f"{self.last_commanded_speed:+.2f}m/s while '{self.odom_topic}' "
                f"reports {self.current_speed:+.2f}m/s. Driving forward must "
                f"read positive. Check speed_to_erpm_gain for "
                f"vesc_to_odom_node in f1tenth_stack/config/vesc.yaml. Until "
                f"that is corrected, odometry-derived speed is unreliable and "
                f"every layer built on it is degraded.")

    def _deadman_engaged(self) -> bool:
        return self._deadman_status()[0]

    def _deadman_status(self):
        """Return engagement plus a precise diagnostic when it is false."""
        if not self.enable_deadman:
            return True, None, None
        if self.last_joy_time is None:
            return (
                False,
                'waiting_for_joy',
                f"no Joy messages received on '{self.joy_topic}'; LB cannot be verified",
            )
        age_sec = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if age_sec >= self.joy_timeout_sec:
            return (
                False,
                'joy_stale',
                f"last Joy message is {age_sec:.2f}s old (limit {self.joy_timeout_sec:.2f}s)",
            )
        if not self.joy_button_available:
            return (
                False,
                'deadman_button_missing',
                f"Joy message has no button index {self.deadman_button} (LB)",
            )
        if not self.deadman_held:
            return False, 'deadman_released', 'LB deadman button is not held'
        return True, None, None

    def scan_callback(self, scan: LaserScan):
        self.last_scan_time = self.get_clock().now()

        deadman_ok, stop_state, stop_detail = self._deadman_status()
        if not deadman_ok:
            self._stop(stop_state, stop_detail)
            return
        self._check_odom_direction()
        if self.enable_ttc and not self._odom_fresh():
            self._stop(
                'odometry_stale',
                f"TTC is enabled but no odometry newer than "
                f"{self.odom_timeout_sec:.2f}s is available on "
                f"'{self.odom_topic}'",
            )
            return

        if not scan.ranges:
            self._stop('scan_empty', 'LaserScan contains no range beams')
            return
        if not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0.0:
            self._stop(
                'scan_invalid',
                f"LaserScan angle_increment={scan.angle_increment!r} is not positive and finite",
            )
            return

        # Invalid beams (NaN, sub-range_min) are *unknown*, not contact:
        # they must never trigger the emergency stop, but must stay
        # non-free for gap selection -- see gap_logic.sanitize_ranges.
        clean, valid = gap_logic.sanitize_ranges(scan.ranges, self.max_range, scan.range_min)

        # Measured once, ahead of gap selection, so _select_gap's inflation
        # and the fallback corner-speed cap both see this tick's sensed
        # corridor -- and so a later stop's escape report (which re-runs gap
        # selection) reads the same margin the drive decision did, not a
        # second, possibly different, sample of the wall.
        if self.enable_centering or self.enable_adaptive_width:
            side_left, side_right = self._side_wall_distances(clean, valid, scan)
        else:
            side_left = side_right = math.inf
        self.centering_left_distance = side_left
        self.centering_right_distance = side_right
        (self.width_factor, self.effective_safety_margin,
         self.effective_corner_speed) = self._adaptive_width(side_left, side_right)

        # Restrict processing to a forward-facing window so the car never
        # steers toward a "gap" that is behind or to the side of it.
        lo_idx, hi_idx = self._fov_indices(scan)
        window = clean[lo_idx:hi_idx + 1]
        window_valid = valid[lo_idx:hi_idx + 1]

        if window.size == 0:
            self._stop(
                'scan_window_empty',
                f"no beams fall inside the configured {math.degrees(self.forward_fov):.1f}deg FOV",
            )
            return

        beam_indices = np.arange(lo_idx, hi_idx + 1, dtype=np.float64)
        beam_angles = scan.angle_min + beam_indices * scan.angle_increment
        body_boundaries = gap_logic.vehicle_boundary_distances(
            beam_angles,
            self.car_width,
            self.car_length,
            self.wheelbase,
            self.laser_offset_x,
            self.laser_offset_y,
        )

        # Hard contact layer using actual clearance from the rectangular body,
        # rather than raw distance from the offset LiDAR origin.
        min_clearance = gap_logic.minimum_footprint_clearance(
            window, window_valid, body_boundaries)
        if min_clearance <= self.emergency_stop_clearance:
            self._stop(
                'emergency_clearance',
                f"minimum body clearance {min_clearance:.3f}m is at or below "
                f"the {self.emergency_stop_clearance:.3f}m threshold",
            )
            return

        # Odom-independent fallback: only the forward cone gets the larger
        # fixed threshold, so a close side wall during a turn does not brake.
        forward_clearance = gap_logic.minimum_footprint_clearance_in_cone(
            window,
            window_valid,
            beam_angles,
            body_boundaries,
            self.forward_stop_fov,
        )
        # Below the reserve the car creeps instead of latching. A hard stop
        # here is a trap: this cone points where the car is *aimed*, not where
        # it is *going*, so a car turning out of a corner gets frozen by the
        # wall it is already steering around -- measured in simulation at
        # 0.58m from the nearest wall, full lock dialled in, exit found, stuck
        # forever. It cannot recover, because the stop removes the very motion
        # that would clear the cone, and nothing external will rescue it.
        #
        # Creeping keeps every independent layer intact: contact clearance and
        # TTC are both checked above and still stop the car outright, and if
        # no gap can be found at all the no_safe_gap stop below still fires.
        # What changes is only that a car with a visible way out is allowed to
        # inch toward it. The creep speed itself is bounded below.
        creeping = forward_clearance <= self.forward_stop_clearance

        # Independent speed-aware layer from F1TENTH Lab 2. A recent positive
        # command backs up fresh odometry only if it is effectively near zero.
        if self.enable_ttc:
            effective_speed, recent_command_speed = self._effective_ttc_speed()
            # Below ttc_min_brake_speed the clock is not consulted at all.
            # TTC is a *closing-speed* brake, and at a crawl it stops being
            # one. Measured on the 2026-08-06 run, 10 of 12 sampled TTC stops
            # fired between 0.15 and 0.39m/s, where the 0.5s threshold is only
            # 0.08-0.20m of clearance -- so a car that had already eased up to
            # the inside of a corner could never re-commit to the turn. Worse,
            # those samples repeatedly read "odom 0.00m/s, recent command
            # 0.16m/s": the car was stopped, the command fallback supplied the
            # speed, TTC braked, the command went to zero, the brake released,
            # and the pair alternated at scan rate.
            #
            # The clearance layers own this regime and are built for it:
            # emergency_stop_clearance still stops outright on contact and the
            # forward reserve still creeps, which is what lets a car with a
            # visible exit inch toward it. This gate removes the brake only
            # where it was blocking that escape, not where it is doing work.
            if effective_speed >= self.ttc_min_brake_speed:
                min_ttc = gap_logic.minimum_ttc(
                    window,
                    window_valid,
                    beam_angles,
                    effective_speed,
                    body_boundaries,
                    self.ttc_min_closing_speed,
                    # Deliberately self.safety_margin, not
                    # self.effective_safety_margin: this is the reactive
                    # collision brake, not the proactive route margin, and it
                    # must not get less vigilant on a sensed-narrow straight
                    # just because gap selection is using less padding there.
                    self.car_width / 2.0 + self.safety_margin,
                    # The rack is where the last command left it, so that is
                    # the arc the car is about to sweep. Straight-line TTC on a
                    # car at full lock brakes for the outside of the very
                    # corner it is negotiating.
                    math.tan(self.steering_basis) / self.wheelbase,
                    self.laser_offset_x,
                    self.laser_offset_y,
                )
                if min_ttc <= self.ttc_threshold_sec:
                    self._stop(
                        'ttc_brake',
                        lambda: (
                            f"minimum footprint-aware TTC {min_ttc:.3f}s is at "
                            f"or below the {self.ttc_threshold_sec:.3f}s "
                            f"threshold at effective speed "
                            f"{effective_speed:.2f}m/s "
                            f"(odom {self.current_speed:.2f}m/s, recent command "
                            f"{recent_command_speed:.2f}m/s)"
                            + self._escape_report(
                                window, window_valid, scan, lo_idx,
                                beam_angles)),
                    )
                    return

        (window, closest_dist, gap_start, gap_end, used_fallback,
         target_idx_in_window, near_bearing) = self._select_gap(
            window, window_valid, scan.angle_increment, beam_angles)
        if gap_start is None:
            closest_text = (
                f"{closest_dist:.2f}m"
                if math.isfinite(closest_dist)
                else "no valid return")
            self._stop(
                'no_safe_gap',
                f"no gap exceeds either the preferred "
                f"{self.min_gap_distance:.2f}m depth or tight-corner "
                f"{self.fallback_min_gap_distance:.2f}m depth with "
                f"{self.min_centerline_gap_width:.2f}m of center corridor; "
                f"closest={closest_text}",
            )
            return

        # Only the real driving decision updates the hysteresis basis --
        # _escape_report below re-runs this same selection purely to
        # describe a stop in the log, and must not make that hypothetical
        # look-up count as this tick's actual aim.
        self.previous_target_idx = target_idx_in_window

        target_idx = lo_idx + target_idx_in_window
        target_angle = scan.angle_min + target_idx * scan.angle_increment
        target_distance = float(window[target_idx_in_window])

        # Steer proportionally to the gap's bearing, rather than by pure
        # pursuit curvature to a point on it. Follow-the-gap produces a
        # *direction to head*, not a path to converge onto, and the two laws
        # disagree exactly where it matters. Pure pursuit reads a 13deg gap
        # bearing as a gentle 2.9m-radius arc even when the car is 0.24m from
        # the wall it is trying to leave -- and that is its *ceiling*, not its
        # tuning: with the LiDAR laser_offset_x ahead of the rear axle, the
        # achievable curvature to a near-axial target is bounded, so no
        # lookahead value recovers the authority. Measured on the 2026-07-27
        # run, that capped every command between 0.064 and 0.118rad while the
        # rack had 0.26rad available, and the car wall-hugged until its
        # forward cone closed. Bearing steering asks for the 1.2m radius the
        # car can actually turn.
        #
        # Bearing steering answers "which way", never "where in the corridor".
        # On a straight the aim is the deepest beam, which runs parallel to the
        # walls and therefore preserves whatever lateral offset the car entered
        # with -- so it will happily hold 0.15m off one wall for the length of
        # a straight. corridor_centering_bias adds the missing cross-track
        # term, bounded and faded out the moment the car is actually turning.
        # It is suppressed entirely while creeping: inside the forward reserve
        # the gap logic is threading a specific way out at 0.25m/s, and that is
        # not the moment to add an opinion about lateral position.
        centering_bias, centering_weight = (
            (0.0, 0.0) if creeping
            else self._centering_bias(side_left, side_right, target_angle,
                                      target_distance))
        desired_steering = float(np.clip(
            self.steering_gain * target_angle + centering_bias,
            -self.max_steering_angle,
            self.max_steering_angle,
        ))
        now, command_dt = self._command_timing()
        steering_basis_before = self.steering_basis
        steering_angle = gap_logic.slew_rate_limit(
            desired_steering,
            steering_basis_before,
            command_dt,
            self.max_steering_rate,
        )
        self.steering_basis = steering_angle

        # Use the clipped requested curvature for the lateral-acceleration
        # ceiling, so speed falls before the rate-limited rack reaches a newly
        # requested turn. Clearance supplies an independent stopping-distance
        # ceiling; unlike acceleration shaping, either ceiling may reduce the
        # command immediately.
        limited_curvature = math.tan(desired_steering) / self.wheelbase
        curve_speed = gap_logic.curvature_speed_limit(
            limited_curvature, self.max_lateral_accel, self.max_speed)
        normal_speed = max(self.min_speed, curve_speed)

        # Cornering anticipation: an *earlier* look at the same curvature
        # cap above, not a different one. near_bearing is the chosen gap's
        # own bearing restricted to what's immediately in front
        # (anticipation_near_depth); target_angle is aim_within_gap's own
        # choice -- the whole gap's angular midpoint in the ordinary case,
        # which is what actually steers. On a straight the two agree and
        # this changes nothing. Approaching a bend, the gap's own angular
        # span leans toward it as soon as any part of the corridor bends
        # away, which the near-only view (still looking at what's directly
        # in front) has not caught up to yet -- fed through the exact same
        # steering_gain -> clip -> tan(.)/wheelbase -> curvature_speed_limit
        # chain the real command already uses, which is what makes this an
        # earlier sample of an existing cap rather than a new one. As the
        # car reaches the bend the near view catches up (the gap closes)
        # and curve_speed above takes back over as the binding constraint;
        # once through, both bearings realign to straight ahead and this
        # releases entirely.
        anticipated_speed = self.max_speed
        if self.enable_cornering_anticipation and near_bearing is not None:
            anticipated_steering = float(np.clip(
                self.steering_gain * (target_angle - near_bearing),
                -self.max_steering_angle,
                self.max_steering_angle,
            ))
            anticipated_curvature = math.tan(anticipated_steering) / self.wheelbase
            anticipated_speed = gap_logic.curvature_speed_limit(
                anticipated_curvature, self.max_lateral_accel, self.max_speed)

        clearance_speed = gap_logic.braking_speed_limit(
            forward_clearance,
            self.forward_stop_clearance,
            self.max_braking_decel,
            self.max_speed,
        )
        if creeping:
            # Inside the reserve the stopping-distance formula returns zero by
            # construction -- that is what latched the car. Keep the identical
            # physics, but let the creep spend half the reserve and no more,
            # and hard-cap it to a crawl on top.
            #
            # Half, rather than all the way down to the contact floor: at the
            # 0.02m contact floor the creep's own stopping distance plus one
            # scan period of latency (~0.017m at 0.25m/s) very nearly consumes
            # the entire remaining gap, which is no margin at all. Halving the
            # reserve keeps the speed tapering smoothly to zero while the car
            # still has 0.125m of padded-body clearance in hand -- enough to
            # crawl out of a corner, never enough to nose into a dead end.
            creep_reserve = max(
                self.emergency_stop_clearance, self.forward_stop_clearance / 2.0)
            clearance_speed = min(
                self.escape_creep_speed,
                gap_logic.braking_speed_limit(
                    forward_clearance,
                    creep_reserve,
                    self.max_braking_decel,
                    self.max_speed,
                ),
            )
        desired_speed = min(normal_speed, clearance_speed, anticipated_speed)
        if used_fallback:
            desired_speed = min(desired_speed, self.effective_corner_speed)

        # Acceleration is deliberately gradual. A lower curvature/clearance
        # ceiling takes effect immediately; delaying a safety-related slowdown
        # merely to make the command look smooth would be the wrong tradeoff.
        #
        # The ramp starts from where the car actually *is*, not from the last
        # command. A ceiling (or a stop) drops the command instantly while the
        # car is still rolling at nearly its old speed; ramping up from that
        # command would hold the throttle below the car's real speed, braking
        # a car that never actually slowed. Fresh odometry closes that gap. It
        # only ever raises the basis, is clamped to max_speed so a bad reading
        # cannot inflate it, and every ceiling above still applies.
        ramp_basis = self.last_commanded_speed
        if self._odom_fresh():
            ramp_basis = max(
                ramp_basis, min(abs(self.current_speed), self.max_speed))
        acceleration_ceiling = ramp_basis + self.max_acceleration * command_dt
        speed = min(desired_speed, acceleration_ceiling)

        self._publish_drive(steering_angle, speed, now=now)
        gap_lo_angle = scan.angle_min + (lo_idx + gap_start) * scan.angle_increment
        gap_hi_angle = scan.angle_min + (lo_idx + gap_end) * scan.angle_increment
        closest_text = (
            f"{closest_dist:.2f}m" if math.isfinite(closest_dist) else "no valid return")
        # Name the *limiter* that shortened the turn, not just the fact that
        # one did: a command pinned at the slew bound every tick means the
        # basis keeps getting reset, which looks identical to gentle shaping
        # unless the basis and the interval are both on screen.
        steering_shape_text = (
            f", steering shaped from {desired_steering:+.3f}rad "
            f"(slew-limited from basis {steering_basis_before:+.3f}rad "
            f"over {command_dt:.3f}s at {self.max_steering_rate:.2f}rad/s)"
            if not math.isclose(steering_angle, desired_steering) else "")
        speed_shape_text = (
            f", acceleration-shaped from {desired_speed:.2f}m/s"
            if not math.isclose(speed, desired_speed) else "")
        # Report the measured corridor whenever centering is even partially
        # engaged, so a car that is drifting to one side on a straight can be
        # diagnosed from the log alone rather than by watching it.
        centering_text = (
            f", centering {centering_bias:+.3f}rad at weight "
            f"{centering_weight:.2f} (left {self.centering_left_distance:.2f}m, "
            f"right {self.centering_right_distance:.2f}m)"
            if centering_weight > 0.0 else "")
        # Only worth a line when it is actually doing something -- on an
        # open room or mid-corner (width_factor == 1.0, no wall reliably on
        # both sides) this fragment would just repeat the static defaults.
        adaptive_text = (
            f", adaptive width {self.width_factor:.2f} (margin "
            f"{self.effective_safety_margin:.3f}m of {self.safety_margin:.2f}m, "
            f"corner cap {self.effective_corner_speed:.2f}m/s, left "
            f"{side_left:.2f}m, right {side_right:.2f}m)"
            if self.enable_adaptive_width and self.width_factor < 1.0 else "")
        gap_mode = 'corner_fallback' if used_fallback else 'gap_follow'
        depth_text = (
            f"fallback depth {self.fallback_min_gap_distance:.2f}m"
            if used_fallback
            else f"preferred depth {self.min_gap_distance:.2f}m")
        cap_text = (
            f"curve cap={curve_speed:.2f}m/s, "
            f"clearance cap={clearance_speed:.2f}m/s"
            + (f", corner cap={self.effective_corner_speed:.2f}m/s" if used_fallback else "")
            + (f", anticipation cap={anticipated_speed:.2f}m/s "
               f"(near {math.degrees(near_bearing):+.1f}deg vs far "
               f"{math.degrees(target_angle):+.1f}deg)"
               if near_bearing is not None and anticipated_speed < self.max_speed else "")
            + (f", CREEP (forward clearance {forward_clearance:.3f}m inside "
               f"the {self.forward_stop_clearance:.3f}m reserve; crawling out)"
               if creeping else ""))
        decision_detail = (
            f"selected {depth_text} gap "
            f"{math.degrees(gap_lo_angle):+.1f}deg to "
            f"{math.degrees(gap_hi_angle):+.1f}deg; target="
            f"{target_distance:.2f}m at {math.degrees(target_angle):+.1f}deg, "
            f"curvature={limited_curvature:+.3f}/m, closest={closest_text}, "
            f"odom={self.current_speed:+.2f}m/s; "
            f"{cap_text}{adaptive_text}{centering_text}{steering_shape_text}{speed_shape_text}")
        self._log_decision(gap_mode, decision_detail, steering_angle, speed)

        # Every speed ceiling that competed for this tick, so the dashboard
        # can name the one that actually won instead of leaving someone to
        # infer it from four numbers in a log line. Only true ceilings go in
        # the list -- they are combined with min(), which is what makes
        # "smallest one is binding" the correct reading. The min_speed floor
        # is folded into normal_speed rather than listed, precisely so it
        # cannot be mistaken for a cap.
        intent_factors = [
            schema.factor('curve cap', normal_speed),
            schema.factor('creep cap' if creeping else 'clearance cap',
                          clearance_speed),
        ]
        if used_fallback:
            intent_factors.append(schema.factor('corner cap', self.effective_corner_speed))
        if near_bearing is not None:
            intent_factors.append(schema.factor('anticipation cap', anticipated_speed))
        intent_factors.append(schema.factor('accel ceiling', acceleration_ceiling))
        self._publish_intent(
            gap_mode,
            decision_detail,
            desired_steering=desired_steering,
            desired_speed=desired_speed,
            commanded_steering=steering_angle,
            commanded_speed=speed,
            factors=schema.bind_min(intent_factors),
            targets=[schema.target('gap_target', *predict.polar_to_body(
                target_angle, target_distance,
                self.laser_offset_x, self.laser_offset_y))],
            wedge={
                'x': self.laser_offset_x,
                'y': self.laser_offset_y,
                'a0': gap_lo_angle,
                'a1': gap_hi_angle,
                'r': target_distance,
            },
        )

    def _validate_centering_parameters(self):
        """Reject a centering configuration that could fight the gap logic.

        The important one is the authority bound. Centering is a *bias* on a
        direction obstacle avoidance already chose; if it were allowed to reach
        the full steering limit it could cancel or invert that choice, which
        makes it a second, unreviewed driving policy rather than a refinement.
        Half the steering limit is the ceiling, and the shipped default
        (0.08 of 0.26 rad) is well inside it.
        """
        if not (math.isfinite(self.centering_gain) and self.centering_gain >= 0.0):
            raise ValueError('centering_gain must be finite and non-negative')
        if not (math.isfinite(self.centering_max_steering)
                and 0.0 <= self.centering_max_steering
                <= self.max_steering_angle / 2.0):
            raise ValueError(
                'centering_max_steering must be finite, non-negative, and no '
                'more than half of max_steering_angle -- a centering bias that '
                'can outvote the chosen gap is not a bias')
        if not (math.isfinite(self.centering_side_half_span)
                and 0.0 < self.centering_side_half_span <= math.pi / 2.0):
            raise ValueError(
                'centering_side_fov_deg must be positive and no wider than '
                '180deg, so the side windows stay on their own side of the car')
        if not (0.0 <= self.centering_full_bearing < self.centering_zero_bearing):
            raise ValueError(
                'centering_full_bearing_deg must be non-negative and strictly '
                'below centering_zero_bearing_deg')
        if not (0.0 <= self.centering_zero_forward_depth
                < self.centering_full_forward_depth):
            raise ValueError(
                'centering_zero_forward_depth must be non-negative and strictly '
                'below centering_full_forward_depth')
        if not (0.0 < self.centering_full_side_distance
                < self.centering_zero_side_distance):
            raise ValueError(
                'centering_full_side_distance must be positive and strictly '
                'below centering_zero_side_distance')

    def _validate_adaptive_width_parameters(self):
        """Reject an adaptive-width configuration that could out-loosen the
        margin it is meant to be a *bounded* relaxation of.

        The important one is the margin floor staying at or below the
        static default -- this feature only ever narrows toward
        min_safety_margin on a sensed-narrow straight, never widens beyond
        safety_margin on a sensed-wide one. There is no scenario where a
        live sensor reading should be trusted to *loosen* the tuned default
        upward; the wide case is simply "change nothing".
        """
        if not (math.isfinite(self.adaptive_width_narrow)
                and math.isfinite(self.adaptive_width_reference)
                and 0.0 < self.adaptive_width_narrow < self.adaptive_width_reference):
            raise ValueError(
                'adaptive_width_narrow must be finite, positive, and strictly '
                'below adaptive_width_reference')
        if not (math.isfinite(self.min_safety_margin)
                and 0.0 < self.min_safety_margin <= self.safety_margin):
            raise ValueError(
                'min_safety_margin must be finite, positive, and no greater '
                'than safety_margin -- it is a floor the margin may shrink '
                'to, not a value it may exceed')
        if not (math.isfinite(self.corner_speed_wide)
                and self.corner_speed <= self.corner_speed_wide <= self.max_speed):
            raise ValueError(
                'corner_speed_wide must be finite and between corner_speed '
                'and max_speed -- it is a ceiling corner_speed may rise to, '
                'not a value below it or above the car\'s own top speed')

    def _side_wall_distances(self, clean, valid, scan):
        """Perpendicular distance to each side wall, shared by corridor
        centering and adaptive-width sensing so a tick that uses both pays
        for the measurement once.

        Deliberately measured against the *full* scan rather than the
        forward window the gap search uses. The window is clipped to
        ``forward_fov_deg`` (180deg), which puts both side directions
        exactly on its boundary, so a window centred on +/-90deg would be
        half empty and would lose the yaw tolerance that makes the
        minimum-range estimator work. This Hokuyo sweeps 270deg and has the
        beams to spare.
        """
        angles = scan.angle_min + np.arange(
            clean.size, dtype=np.float64) * scan.angle_increment
        left = gap_logic.side_wall_distance(
            clean, valid, angles, math.pi / 2.0, self.centering_side_half_span)
        right = gap_logic.side_wall_distance(
            clean, valid, angles, -math.pi / 2.0, self.centering_side_half_span)
        return left, right

    def _centering_bias(self, left, right, aim_bearing, aim_depth):
        """Cross-track centering bias for the current scan, or (0.0, 0.0)."""
        if not self.enable_centering:
            return 0.0, 0.0
        return gap_logic.corridor_centering_bias(
            left, right, aim_bearing, aim_depth,
            self.centering_gain, self.centering_max_steering,
            self.centering_full_bearing, self.centering_zero_bearing,
            self.centering_full_forward_depth, self.centering_zero_forward_depth,
            self.centering_full_side_distance, self.centering_zero_side_distance,
        )

    def _adaptive_width(self, left, right):
        """(width_factor, effective_safety_margin, effective_corner_speed)
        for the sensed corridor this tick, or the static defaults unchanged
        if the feature is off.

        Only feeds the *proactive* gap-selection margin and the fallback
        corner-speed cap -- see _select_gap and scan_callback. The reactive
        safety backstops (TTC's swept width, the all-round contact floor,
        the forward-reserve creep) all keep reading the static
        safety_margin/thresholds regardless of this, on purpose: they are
        the last line, not the route-planning margin, and a sensed-narrow
        reading is exactly the wrong moment to make the last line less
        vigilant too.
        """
        if not self.enable_adaptive_width:
            return 1.0, self.safety_margin, self.corner_speed
        width_factor = gap_logic.corridor_width_factor(
            left, right, self.adaptive_width_narrow, self.adaptive_width_reference)
        effective_safety_margin = gap_logic.scale_between(
            width_factor, self.min_safety_margin, self.safety_margin)
        effective_corner_speed = gap_logic.scale_between(
            width_factor, self.corner_speed, self.corner_speed_wide)
        return width_factor, effective_safety_margin, effective_corner_speed

    def _select_gap(self, window, window_valid, angle_increment, beam_angles):
        """Inflate obstacle edges, bubble the closest hit, and pick a gap.

        Split out of scan_callback so a *blocking stop* can ask the same
        question the driving path asks -- "is there a way out of here?" --
        and put the answer in the log. Returns the processed window plus
        ``(closest_dist, gap_start, gap_end, used_fallback, target,
        near_bearing)`` -- the last for cornering anticipation, see
        gap_logic.near_gap_bearing.

        Also applies gap-selection hysteresis against
        ``self.previous_target_idx``/``self.gap_switch_margin`` -- reading
        it here is safe from both call sites (the escape-report path is
        read-only against it too), but only scan_callback's real driving
        decision may ever *write* self.previous_target_idx.

        Uses ``self.effective_safety_margin``, not ``self.safety_margin``
        directly -- on a sensed-narrow straight the two can differ (see
        _adaptive_width); scan_callback refreshes the former every tick
        before this is ever called, including from the escape-report path.
        """
        closest_idx, closest_dist = gap_logic.closest_valid(window, window_valid)

        # Inflate each edge by half the car width plus one side's margin.
        # Remaining ranges represent valid car-center positions.
        half_width = self.car_width / 2.0 + self.effective_safety_margin
        processed = gap_logic.disparity_extend(
            window, angle_increment, self.disparity_threshold, half_width)
        if closest_idx is not None:
            processed = gap_logic.safety_bubble(
                processed, closest_idx, closest_dist, angle_increment,
                half_width)

        gap_start, gap_end, used_fallback = gap_logic.find_gap_with_fallback(
            processed,
            self.min_gap_distance,
            self.fallback_min_gap_distance,
            angle_increment,
            self.min_centerline_gap_width,
            previous_target_idx=self.previous_target_idx,
            switch_margin=self.gap_switch_margin,
        )
        target = gap_logic.aim_within_gap(
            processed, gap_start, gap_end, beam_angles)
        near_bearing = gap_logic.near_gap_bearing(
            processed, gap_start, gap_end, beam_angles, self.anticipation_near_depth)
        return (processed, closest_dist, gap_start, gap_end, used_fallback, target,
                near_bearing)

    def _escape_report(self, window, window_valid, scan, lo_idx,
                       beam_angles) -> str:
        """Say whether a stopped car can see a way out, and where.

        A blocking stop that prints only the clearance that tripped it cannot
        be told apart from a genuine dead end. When the car is sitting still
        the question that actually matters is whether it is boxed in or
        holding station in front of an escape it has already found -- so run
        the same gap search the driving path runs and report the answer.

        Only ever called from the logging path, so the extra work happens at
        the log rate, not the scan rate.
        """
        _, _, gap_start, _, used_fallback, target, _ = self._select_gap(
            window, window_valid, scan.angle_increment, beam_angles)
        if gap_start is None:
            return ('; NO ESCAPE VISIBLE: no gap clears either depth '
                    'threshold, so the car cannot steer out of this unaided')
        target_angle = scan.angle_min + (
            lo_idx + target) * scan.angle_increment
        steering = float(np.clip(
            self.steering_gain * target_angle,
            -self.max_steering_angle,
            self.max_steering_angle,
        ))
        depth = 'fallback-depth' if used_fallback else 'preferred-depth'
        return (f"; escape visible: {depth} gap at "
                f"{math.degrees(target_angle):+.1f}deg, would steer "
                f"{steering:+.3f}rad as soon as this clears")

    def _sensor_status_callback(self):
        """Explain a missing scan stream even though scan_callback is idle."""
        if self.last_scan_time is None:
            detail = (f"no LaserScan received on '{self.scan_topic}'; "
                      "no drive command is being generated")
            self._log_decision(
                'waiting_for_scan', detail, 0.0, 0.0, command_published=False)
            self._publish_intent(
                'waiting_for_scan', detail,
                desired_steering=0.0, desired_speed=0.0,
                commanded_steering=0.0, commanded_speed=0.0)
            return
        age_sec = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if age_sec >= self.scan_timeout_sec:
            detail = (
                f"last LaserScan is {age_sec:.2f}s old "
                f"(limit {self.scan_timeout_sec:.2f}s); /drive has gone quiet "
                "and the mux will stop the car")
            self._log_decision(
                'scan_stale', detail, 0.0, 0.0, command_published=False)
            self._publish_intent(
                'scan_stale', detail,
                desired_steering=0.0, desired_speed=0.0,
                commanded_steering=0.0, commanded_speed=0.0)

    def _fov_indices(self, scan: LaserScan):
        half_fov = self.forward_fov / 2.0
        lo_angle = max(scan.angle_min, -half_fov)
        hi_angle = min(scan.angle_max, half_fov)
        num_points = len(scan.ranges)
        lo_idx = int((lo_angle - scan.angle_min) / scan.angle_increment)
        hi_idx = int((hi_angle - scan.angle_min) / scan.angle_increment)
        lo_idx = max(0, min(lo_idx, num_points - 1))
        hi_idx = max(0, min(hi_idx, num_points - 1))
        return lo_idx, hi_idx

    def _command_timing(self):
        """Return one bounded command interval and the clock sample behind it."""
        now = self.get_clock().now()
        if self.last_command_time is None:
            return now, self.command_slew_max_dt
        elapsed = (now - self.last_command_time).nanoseconds / 1e9
        return now, min(self.command_slew_max_dt, max(0.0, elapsed))

    def _publish_drive(self, steering_angle: float, speed: float, now=None):
        if now is None:
            now = self.get_clock().now()
        if math.isfinite(speed) and math.isfinite(steering_angle):
            # Every command supersedes the previous one. In particular, an
            # emergency zero must not leave an older positive command latched.
            # The steering slew basis is deliberately not updated here -- the
            # drive path and _stop() each own how it evolves.
            self.last_commanded_speed = float(speed)
            self.last_command_time = now

        msg = AckermannDriveStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

    def _stop(self, state: str, detail: str):
        # Hold the rack where it is; only the speed goes to zero. A stationary
        # car's steering angle is inert -- it cannot cause motion, and the mux
        # stops the car regardless -- so centring it buys no safety, and it
        # costs the one thing the car needs to get out of trouble.
        #
        # Centring here is what defeated two runs. TTC braking near a wall
        # alternates stop/drive at scan rate, and every stop returned the rack
        # to centre, so each drive command in between started its rate limit
        # from 0 and never exceeded max_steering_rate*dt (~0.03rad) while the
        # speed ramp -- which reads measured speed, not the last command --
        # recovered in full. The car kept its throttle and lost its steering
        # precisely while pinned against a wall.
        #
        # Holding is also the honest model: publishing 0 would really drive
        # the servo to centre, so the basis would have to follow it down.
        # Nothing here needs resetting by an operator -- the basis tracks the
        # rack, the rack holds while stopped, and the next drive command slews
        # from where the wheels actually are. Recovery must never depend on
        # someone noticing the car and cycling LB.
        self._publish_drive(self.steering_basis, 0.0)
        # Memoized because some stop reasons are deliberately expensive
        # thunks -- _escape_report re-runs the whole gap pipeline -- and the
        # log and the intent publisher both want that same string on the
        # ticks where their independent throttles happen to coincide.
        detail = schema.memoize_reason(detail)
        self._log_decision(state, detail, self.steering_basis, 0.0)
        self._publish_intent(
            state,
            detail,
            desired_steering=self.steering_basis,
            desired_speed=0.0,
            commanded_steering=self.steering_basis,
            commanded_speed=0.0,
        )

    def _publish_intent(self, state: str, reason, *, desired_steering: float,
                        desired_speed: float, commanded_steering: float,
                        commanded_speed: float, factors=(), targets=(),
                        wedge=None):
        """Publish what this controller is *trying* to do, for the dashboard.

        Three rules make this safe to run inside a node that steers a
        physical car, and they are why this method looks the way it does
        (the full reasoning is in docs/drive-intent.md):

        1. It is only ever called *after* the drive command for this tick
           has already been published, so nothing in here -- however slow
           or however broken -- can delay a command, including a stop.
        2. Everything is wrapped in one try/except. An exception raised by
           a diagnostic drawing must not propagate into scan_callback and
           take down the node holding this car's steering. After a run of
           failures it latches itself off and says so once.
        3. It only reads values the driving path already computed, and
           writes no state the driving path reads back.

        `reason` may be a string or a thunk; it is resolved only on the
        ticks that actually carry one (state changes, then every
        decision_log_period_sec), so an expensive explanation costs nothing
        on the ticks in between.
        """
        if self.intent_pub is None or self._intent_failures.disabled:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if not self._intent_throttle.should_publish(now):
            return
        try:
            payload = schema.build(
                'gap_follow_node',
                state,
                reason=(reason
                        if self._intent_throttle.wants_reason(now, state)
                        else None),
                # A reactive controller chooses a *heading*, not a path, so
                # one constant-curvature arc is the honest prediction here --
                # unlike pure_pursuit, which really is following a line and
                # re-asks its steering law along the way.
                path=predict.constant_arc(
                    desired_steering, desired_speed, self.wheelbase,
                    self.intent_horizon_sec, self.intent_samples,
                    max_length_m=self.intent_max_length),
                commanded_path=predict.constant_arc(
                    commanded_steering, commanded_speed, self.wheelbase,
                    self.intent_horizon_sec, self.intent_samples,
                    max_length_m=self.intent_max_length),
                desired_steering=desired_steering,
                commanded_steering=commanded_steering,
                desired_speed=desired_speed,
                commanded_speed=commanded_speed,
                horizon_s=self.intent_horizon_sec,
                factors=factors,
                targets=targets,
                wedge=wedge,
            )
            self.intent_pub.publish(String(data=schema.encode(payload)))
            self._intent_failures.record_success()
        except Exception as exc:  # noqa: BLE001 -- see rule 2 above
            if self._intent_failures.record_failure():
                self.get_logger().error(
                    f'drive intent publishing failed repeatedly '
                    f'({type(exc).__name__}: {exc}); switching it off for the '
                    f'rest of this run. Driving is unaffected.')
            else:
                self.get_logger().warn(
                    f'drive intent skipped this tick: '
                    f'{type(exc).__name__}: {exc}')

    def _log_decision(self, state: str, detail: str, steering_angle: float,
                      speed: float, command_published: bool = True):
        """Log state transitions immediately and steady decisions periodically."""
        now = self.get_clock().now()
        state_changed = state != self.last_decision_state
        period_elapsed = (
            self.decision_log_period_sec > 0.0
            and (
                self.last_decision_log_time is None
                or (now - self.last_decision_log_time).nanoseconds / 1e9
                >= self.decision_log_period_sec
            )
        )
        if not state_changed and not period_elapsed:
            return

        # Detail may be a thunk so that an expensive diagnostic (the escape
        # report re-runs the whole gap pipeline) is only paid for on the ticks
        # that actually print, not on every scan.
        if callable(detail):
            detail = detail()

        stopped = speed <= 0.0
        if command_published:
            output = f"command: steering={steering_angle:+.3f}rad, speed={speed:.2f}m/s"
        else:
            output = "command: none (the mux stops when /drive times out)"
        message = f"{'STOP' if stopped else 'DRIVE'} [{state}] {detail}; {output}"
        if stopped:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)
        self.last_decision_state = state
        self.last_decision_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # The default SIGINT handler may already have shut the context down.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
