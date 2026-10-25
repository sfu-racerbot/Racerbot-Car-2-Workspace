"""Fused forward speed and measured steering angle, with no ROS imports.

Everything vehicle_state_node decides lives here, so it can be tested with a
bare `python3 -m pytest src/imu_bridge/test/` and no ROS install. The node
only moves numbers between topics and this module.

Two outputs:

- **Speed** (/odom/fused). A two-state Kalman filter on [v, b]: base_link's
  forward speed and the accelerometer's forward bias. The accelerometer drives
  the prediction (fast, sees wheelspin and lockup), the wheel speed from /odom
  corrects it (slow, but never drifts). A wheel speed further from the
  prediction than the filter's own uncertainty allows (`slip_gate` sigmas) is
  treated as wheelspin or lockup and skipped, for at most `max_slip_time`.
  The published value is always kept within `max_divergence` of the wheel
  speed, and falls back to the wheel speed exactly whenever the accelerometer
  is missing, stale or untrusted -- so the worst this can do to a consumer is
  shift today's speed by max_divergence.

- **Steering** (/steering/measured). The kinematic bicycle model, solved for
  the angle: yaw_rate = v * tan(delta) / L, so delta = atan(L * yaw_rate / v).
  That is the angle the car's path is actually curving at, slip and
  understeer included, which is exactly what the servo command cannot tell
  you. It is NaN below `min_steering_speed`: there v is near zero and the
  division is meaningless, and a stopped car's steering cannot be seen by an
  IMU at all.

- **Gyro odometry** (/odom/imu, and optionally the odom_imu -> odom
  transform). The car's pose integrated with the heading from a gyro instead
  of from the steering command: vesc_to_odom's yaw is
  speed * tan(commanded steering) / wheelbase, so it knows nothing of servo
  lag or understeer (car 2, 2026-10-19: 0.15 s lag, 0.95 turning gain, 0.44
  rad/s RMS turn-rate error). Distance is vesc_to_odom's own, unchanged, so
  the only difference between the two odometries is where heading comes
  from. Without a usable gyro it falls back to vesc_to_odom's heading
  change, i.e. exactly today's odometry.

Both gyros measure the same yaw rate (a rigid body turns at one rate at
every point), so they are averaged for noise and cross-checked for faults,
not combined for geometry. The accelerometer is different: one mounted ahead
of base_link also feels centripetal acceleration along x, which
accel_at_base_link removes.
"""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import math

# Below this many m/s^2 of averaged +z acceleration a source is taken to be
# mounted wrong (upside down, or a wrong rpy_deg in car.yaml): at rest a
# correctly mounted IMU reads about +9.8 on z. 0.7 g leaves room for bumps and
# a tilted floor; an upside-down IMU reads -9.8 and fails it by a mile. An
# upside-down gyro also reads yaw rate with the wrong sign, which is why the
# check gates the gyro too, not only the accelerometer.
MIN_GRAVITY_Z = 0.7 * 9.80665
# Time constant of that running average [s].
GRAVITY_TAU_S = 1.0
# Samples a source must deliver before the gravity check can pass.
GRAVITY_MIN_SAMPLES = 10

MODE_FUSED = 'fused'
MODE_CLAMPED = 'clamped'          # fused, but held to max_divergence of the wheels
MODE_SLIP = 'slip'                # wheels skipped as slipping; accel only, clamped
MODE_WHEEL_ONLY = 'wheel_only'    # no usable accelerometer: wheel speed exactly
MODE_INVALID = 'invalid'          # wheel speed itself non-finite: passed through


def check_output_topics(odom_topic, fused_odom_topic, steering_topic, imu_odom_topic):
    """Refuse output topics that could reach the drive path or shadow /odom.

    The node publishes nav_msgs/Odometry and sensor_msgs/JointState, which no
    drive consumer accepts -- but a remap is one typo away, so the names are
    checked too: nothing named like a drive input (.../drive, .../teleop,
    /ackermann_cmd, /commands/...), never the odometry it reads from (that
    would be a second /odom publisher, feeding itself), and no two outputs
    on one topic.
    """
    outputs = {'fused_odom_topic': fused_odom_topic, 'steering_topic': steering_topic,
               'imu_odom_topic': imu_odom_topic}
    for name, topic in outputs.items():
        _require(isinstance(topic, str) and topic.startswith('/') and len(topic) > 1,
                 f'{name} must be an absolute topic name, got {topic!r}')
        last = topic.rstrip('/').rsplit('/', 1)[-1]
        _require(last not in ('drive', 'teleop') and topic != '/ackermann_cmd'
                 and not topic.startswith('/commands/'),
                 f'{name} {topic!r} is named like a drive command topic')
        _require(topic != odom_topic,
                 f'{name} must differ from odom_topic ({odom_topic!r})')
    _require(len(set(outputs.values())) == len(outputs),
             f'output topics must all differ, got {outputs}')


def check_frames(imu_odom_frame, odom_frame, base_frame):
    """Refuse an imu_odom_frame that would break the transform tree.

    Its transform is imu_odom_frame -> odom_frame, so it must be a new frame:
    not odom_frame itself or base_frame (a loop), and not `map` (slam_toolbox
    or a particle filter already own map -> odom: two parents for one frame).
    """
    for name, frame in (('imu_odom_frame', imu_odom_frame), ('odom_frame', odom_frame),
                        ('base_frame', base_frame)):
        _require(isinstance(frame, str) and frame and not frame.startswith('/')
                 and ' ' not in frame,
                 f'{name} must be a plain frame name, got {frame!r}')
    _require(imu_odom_frame not in (odom_frame, base_frame, 'map'),
             f'imu_odom_frame {imu_odom_frame!r} must be a new frame, not '
             f'{odom_frame!r}, {base_frame!r} or map')
    _require(odom_frame != base_frame, 'odom_frame and base_frame must differ')


def _finite(*values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def _require(condition, message):
    if not condition:
        raise ValueError(message)


@dataclass(frozen=True)
class EstimatorConfig:
    """Every tuning knob, in SI units. Defaults match config/vehicle_state.yaml."""

    # Car 1's wheelbase [m] (vesc.yaml; docs/hardware-reference.md).
    wheelbase: float = 0.36
    # Which IMU's accelerometer drives the speed prediction: 'bno', 'vesc' or
    # 'none' (= wheel speed only, the filter off).
    accel_source: str = 'bno'
    # How far ahead of base_link (the rear axle) each IMU sits, along x [m].
    # Only the accel_source's matters, for the centripetal correction.
    x_offset: tuple = (('bno', 0.36), ('vesc', 0.18))
    # Weight of each IMU's gyro in the yaw-rate average. 0 leaves one out.
    gyro_weight: tuple = (('bno', 1.0), ('vesc', 1.0))
    # Speed random walk from accelerometer error [m/s per sqrt(s)]. White
    # vibration of 1 m/s^2 rms at 250 Hz integrates to only ~0.06; the rest
    # is pitch leaking gravity in under hard acceleration and lever-arm
    # residue. It also sets how long slip can be told apart from a real
    # speed change (the gate widens as this uncertainty grows).
    sigma_accel: float = 0.15
    # How fast the accelerometer bias may creep [m/s^2 per sqrt(s)].
    sigma_bias_rw: float = 0.02
    # Wheel-speed noise, 1 sigma [m/s].
    sigma_wheel: float = 0.08
    # Prior on the accelerometer's forward bias, 1 sigma [m/s^2].
    sigma_bias0: float = 0.3
    # The published speed never differs from the wheel speed by more [m/s].
    max_divergence: float = 0.3
    # Past this gap between filter and wheels the filter restarts from the
    # wheel speed [m/s]: something is wrong, not slipping.
    reset_divergence: float = 1.0
    # A wheel speed more than this many sigmas (of the predicted speed and
    # the wheel noise together) from the prediction is slip [-]...
    slip_gate: float = 4.0
    # ...but only for this long: wheels "slipping" any longer are believed
    # again and the filter restarts from them [s].
    max_slip_time: float = 0.5
    # An IMU sample older than this is stale [s].
    imu_timeout: float = 0.1
    # A wheel speed older than this is stale [s].
    wheel_timeout: float = 0.2
    # Accelerometer samples further apart than this are not integrated, and
    # the filter restarts from the next wheel speed [s].
    max_accel_dt: float = 0.05
    # |speed| below this gives a NaN steering angle [m/s].
    min_steering_speed: float = 0.3
    # Running mean of (bno - vesc) yaw rate past which the gyros disagree and
    # the steering angle is NaN [rad/s], and its time constant [s].
    gyro_disagree_threshold: float = 0.15
    gyro_disagree_tau: float = 0.5
    # Gyro odometry: which gyros may supply heading, in order of preference.
    # ('none',) = always vesc_to_odom's own (steering-model) heading. (ROS
    # cannot load an empty list from YAML, so there is a word for it.)
    heading_sources: tuple = ('bno', 'vesc')
    # Two samples of one gyro further apart than this are a gap: nothing is
    # integrated across it, and an odometry step that spans it falls back [s].
    max_gyro_gap: float = 0.05
    # How far past its newest sample a gyro's heading may be extrapolated, at
    # its newest rate, to reach an odometry stamp [s]. The next step corrects
    # the extrapolation, so heading never accumulates its error.
    max_heading_extrapolation: float = 0.05

    def __post_init__(self):
        for name in ('wheelbase', 'sigma_accel', 'sigma_bias_rw', 'sigma_wheel',
                     'sigma_bias0', 'max_divergence', 'reset_divergence',
                     'slip_gate', 'max_slip_time', 'imu_timeout', 'wheel_timeout', 'max_accel_dt',
                     'min_steering_speed', 'gyro_disagree_threshold',
                     'gyro_disagree_tau', 'max_gyro_gap', 'max_heading_extrapolation'):
            value = getattr(self, name)
            _require(_finite(value) and value > 0.0,
                     f'{name} must be finite and > 0, got {value!r}')
        _require(self.reset_divergence > self.max_divergence,
                 f'reset_divergence ({self.reset_divergence}) must exceed '
                 f'max_divergence ({self.max_divergence})')
        _require(self.accel_source in ('bno', 'vesc', 'none'),
                 f"accel_source must be 'bno', 'vesc' or 'none', got {self.accel_source!r}")
        for name in ('x_offset', 'gyro_weight'):
            table = dict(getattr(self, name))
            _require(set(table) == {'bno', 'vesc'},
                     f'{name} needs exactly bno and vesc, got {sorted(table)}')
            _require(all(_finite(v) for v in table.values()),
                     f'{name} values must be finite, got {table}')
        sources = tuple(self.heading_sources)
        _require(sources == ('none',) or (
                     len(sources) > 0 and len(set(sources)) == len(sources)
                     and all(name in ('bno', 'vesc') for name in sources)),
                 "heading_sources must be ['none'] or distinct names from 'bno', 'vesc', "
                 f"got {sources!r}")
        weights = dict(self.gyro_weight)
        _require(all(w >= 0.0 for w in weights.values()) and sum(weights.values()) > 0.0,
                 f'gyro_weight must be >= 0 with at least one > 0, got {weights}')


def accel_at_base_link(accel_x, yaw_rate, x_offset):
    """Forward acceleration of base_link from an accelerometer x_offset ahead of it.

    A point at r = (d, 0, 0) on a body turning at w = (0, 0, yaw_rate)
    accelerates by a_ref + w x (w x r) (+ an alpha x r term, which has no x
    component), and w x (w x r) = (-yaw_rate^2 d, 0, 0). So the sensor reads
    yaw_rate^2 * d less than base_link along x, and that is added back.
    Unhandled, it is -0.36 m/s^2 at 1 rad/s on car 1's front axle -- read as
    braking in every corner.
    """
    return accel_x + yaw_rate * yaw_rate * x_offset


def steering_from_yaw_rate(yaw_rate, speed, wheelbase, min_speed):
    """Effective steering angle [rad], or NaN when it cannot be measured.

    Kinematic bicycle (base_link on the rear axle): yaw_rate = v tan(delta) / L.
    Valid in reverse too -- v and yaw_rate both flip, delta keeps its sign.
    """
    if not _finite(yaw_rate, speed) or abs(speed) < min_speed:
        return math.nan
    return math.atan(wheelbase * yaw_rate / speed)


def bounded(value, centre, half_width):
    """value clamped to [centre - half_width, centre + half_width]."""
    return min(max(value, centre - half_width), centre + half_width)


class SpeedFilter:
    """Kalman filter on x = [v, b]: forward speed and accelerometer bias.

    predict:  v += (a - b) dt;   F = [[1, -dt], [0, 1]]
              Q = diag(sigma_accel^2 dt, sigma_bias_rw^2 dt)
    update:   z = v + noise(sigma_wheel)
    """

    def __init__(self, cfg: EstimatorConfig):
        self.cfg = cfg
        self.initialized = False
        self.v = 0.0
        self.b = 0.0
        self.p_bb = cfg.sigma_bias0 ** 2
        self.P = [[0.0, 0.0], [0.0, self.p_bb]]

    def reset(self):
        """Forget the speed; keep the learned bias. The next update re-seeds v."""
        if self.initialized:
            self.p_bb = self.P[1][1]
        self.initialized = False

    def predict(self, accel, dt):
        if not self.initialized or not _finite(accel, dt) or dt <= 0.0:
            return False
        cfg = self.cfg
        self.v += (accel - self.b) * dt
        (p00, p01), (p10, p11) = self.P
        # F P F^T + Q, written out for F = [[1, -dt], [0, 1]].
        n00 = p00 - dt * (p01 + p10) + dt * dt * p11 + cfg.sigma_accel ** 2 * dt
        n01 = p01 - dt * p11
        n11 = p11 + cfg.sigma_bias_rw ** 2 * dt
        self.P = [[n00, n01], [n01, n11]]
        return True

    def update(self, z):
        cfg = self.cfg
        if not _finite(z):
            return
        if not self.initialized or abs(z - self.v) > cfg.reset_divergence:
            self.v = z
            self.P = [[cfg.sigma_wheel ** 2, 0.0], [0.0, self.p_bb]]
            self.initialized = True
            return
        (p00, p01), (p10, p11) = self.P
        s = p00 + cfg.sigma_wheel ** 2
        k0, k1 = p00 / s, p10 / s
        y = z - self.v
        self.v += k0 * y
        self.b += k1 * y
        self.P = [[p00 - k0 * p00, p01 - k0 * p01],
                  [p10 - k1 * p00, p11 - k1 * p01]]

    @property
    def variance(self):
        return self.P[0][0]


@dataclass
class SpeedEstimate:
    speed: float
    variance: float
    mode: str


class HeadingTrack:
    """One gyro's yaw rate integrated into a heading, against its own stamps.

    The heading starts at 0 at the first sample; only differences are ever
    used. Trapezoidal integration between samples; nothing across a gap
    longer than max_gap (that span is remembered, so a step over it can
    refuse). value(t) interpolates inside the history and extrapolates a
    little past its newest sample at the newest rate.
    """

    def __init__(self, history=2000):
        self.stamps = deque(maxlen=history)
        self.headings = deque(maxlen=history)
        self.rate = math.nan
        self.gaps = deque(maxlen=16)      # (start, end) stamps of each gap

    def add(self, stamp, rate, max_gap):
        if not self.stamps:
            self.stamps.append(stamp)
            self.headings.append(0.0)
            self.rate = rate
            return
        last = self.stamps[-1]
        dt = stamp - last
        if dt <= 0.0:
            return                        # duplicate or out of order: ignored
        heading = self.headings[-1]
        if dt > max_gap:
            self.gaps.append((last, stamp))
        else:
            heading += 0.5 * (self.rate + rate) * dt
        self.stamps.append(stamp)
        self.headings.append(heading)
        self.rate = rate

    def value(self, t, max_extrapolation):
        """Heading at stamp t [rad], or None when t is outside what is known."""
        if not self.stamps or t < self.stamps[0]:
            return None
        last = self.stamps[-1]
        if t >= last:
            if t - last > max_extrapolation:
                return None
            return self.headings[-1] + self.rate * (t - last)
        i = bisect_left(self.stamps, t)
        t1, h1 = self.stamps[i], self.headings[i]
        if t1 == t:
            return h1
        t0, h0 = self.stamps[i - 1], self.headings[i - 1]
        return h0 + (h1 - h0) * (t - t0) / (t1 - t0)

    def has_gap(self, t0, t1):
        return any(a < t1 and b > t0 for a, b in self.gaps)


class _Source:
    def __init__(self):
        self.t_recv = -math.inf     # receipt time of the last sample [s]
        self.stamp = None           # its header stamp [s]
        self.yaw_rate = math.nan
        self.gravity_z = 0.0
        self.samples = 0
        self.track = HeadingTrack()
        self.last_eval = None       # (stamp, heading) this source gave the last odometry step

    @property
    def mount_ok(self):
        return self.samples >= GRAVITY_MIN_SAMPLES and self.gravity_z >= MIN_GRAVITY_Z


class VehicleStateEstimator:
    """Fused speed and measured steering from wheel speed and two IMUs.

    Times: t_recv is when the node received a message (one monotonic clock,
    used for every staleness check); stamp is the message's header stamp,
    used only for the time between two samples of the same IMU (the BNO's
    stamps come from its own clock, so they stay evenly spaced even when
    samples arrive in bursts).
    """

    SOURCES = ('bno', 'vesc')

    def __init__(self, cfg: EstimatorConfig):
        self.cfg = cfg
        self.filter = SpeedFilter(cfg)
        self.src = {name: _Source() for name in self.SOURCES}
        self._weights = dict(cfg.gyro_weight)
        self._offsets = dict(cfg.x_offset)
        self._accel_t = -math.inf    # receipt time of the last integrated accel sample
        self._wheel_t = -math.inf
        self._speed = math.nan       # last published speed
        self._diff_mean = 0.0        # running mean of (bno - vesc) yaw rate
        self._diff_t = None
        self._slip_since = None      # receipt time the current slip began

    # ------------------------------------------------------------ inputs

    def on_imu(self, source, t_recv, stamp, accel_x, accel_z, yaw_rate):
        """One calibrated, car-axis IMU sample (m/s^2, rad/s)."""
        s = self.src[source]
        if not _finite(t_recv, stamp, accel_x, accel_z, yaw_rate):
            return
        alpha = 1.0 if s.samples == 0 else 1.0 - math.exp(
            -max(t_recv - s.t_recv, 0.0) / GRAVITY_TAU_S)
        s.gravity_z += alpha * (accel_z - s.gravity_z)
        s.samples += 1
        previous_stamp, s.stamp = s.stamp, stamp
        s.t_recv = t_recv
        s.yaw_rate = yaw_rate
        s.track.add(stamp, yaw_rate, self.cfg.max_gyro_gap)
        self._update_gyro_agreement(t_recv)

        if source != self.cfg.accel_source or not s.mount_ok or previous_stamp is None:
            return
        dt = stamp - previous_stamp
        if not 0.0 < dt <= self.cfg.max_accel_dt:
            # A gap (or a stamp going backwards): whatever happened during it
            # was not integrated, so restart from the next wheel speed.
            self.filter.reset()
            return
        # The lever arm takes this IMU's own gyro, not the fused yaw rate: same
        # chip, same instant as the accelerometer, none of the VESC's lag.
        a = accel_at_base_link(accel_x, yaw_rate, self._offsets[source])
        if self.filter.predict(a, dt):
            self._accel_t = t_recv

    def on_wheel(self, t_recv, wheel_speed) -> SpeedEstimate:
        """One /odom wheel speed [m/s]; returns what /odom/fused should carry."""
        cfg = self.cfg
        if not _finite(wheel_speed):
            # Passed through untouched: a consumer must see exactly what it
            # would have seen on /odom (gap_follow treats non-finite as stale).
            self._speed = math.nan
            return SpeedEstimate(wheel_speed, cfg.sigma_wheel ** 2, MODE_INVALID)
        self._wheel_t = t_recv
        f = self.filter
        if not self.accel_fresh(t_recv):
            self._slip_since = None
            f.reset()
            f.update(wheel_speed)
            self._speed = wheel_speed
            return SpeedEstimate(wheel_speed, cfg.sigma_wheel ** 2, MODE_WHEEL_ONLY)
        if f.initialized and self._slipping(t_recv, wheel_speed):
            mode = MODE_SLIP
        else:
            self._slip_since = None
            f.update(wheel_speed)
            mode = MODE_FUSED
        speed = bounded(f.v, wheel_speed, cfg.max_divergence)
        if mode == MODE_FUSED and speed != f.v:
            mode = MODE_CLAMPED
        self._speed = speed
        return SpeedEstimate(speed, f.variance, mode)

    # ------------------------------------------------------------ outputs

    def accel_fresh(self, t):
        return self.cfg.accel_source != 'none' and t - self._accel_t <= self.cfg.imu_timeout

    def gyro_usable(self, source, t):
        s = self.src[source]
        return (self._weights[source] > 0.0 and s.mount_ok
                and t - s.t_recv <= self.cfg.imu_timeout)

    def yaw_rate(self, t):
        """Weighted mean of the usable gyros' yaw rate [rad/s], or None."""
        total = weight = 0.0
        for name in self.SOURCES:
            if self.gyro_usable(name, t):
                w = self._weights[name]
                total += w * self.src[name].yaw_rate
                weight += w
        return total / weight if weight > 0.0 else None

    @property
    def gyro_difference(self):
        """Running mean of (bno - vesc) yaw rate [rad/s], for the logs."""
        return self._diff_mean

    def gyros_disagree(self, t):
        """Both gyros usable and steadily apart. With one gone there is
        nothing to disagree with, and the survivor is used alone."""
        return (all(self.gyro_usable(name, t) for name in self.SOURCES)
                and abs(self._diff_mean) > self.cfg.gyro_disagree_threshold)

    def speed(self, t):
        """The last published speed, or NaN once the wheels have gone stale."""
        if t - self._wheel_t > self.cfg.wheel_timeout:
            return math.nan
        return self._speed

    def steering(self, t):
        """Effective steering angle [rad], or NaN when it cannot be measured."""
        r = self.yaw_rate(t)
        if r is None or self.gyros_disagree(t):
            return math.nan
        return steering_from_yaw_rate(r, self.speed(t), self.cfg.wheelbase,
                                      self.cfg.min_steering_speed)

    def heading_change(self, t_recv, t0, t1):
        """(source, heading change [rad]) from stamp t0 to t1, or None.

        The first usable source in cfg.heading_sources: fresh, mounted the
        right way up, no gap inside (t0, t1], and its heading known at both
        ends. None while the two gyros disagree (one of them is wrong, and
        which is unknown) -- the caller then uses the steering model.
        """
        if self.gyros_disagree(t_recv):
            return None
        for name in self.cfg.heading_sources:
            if name == 'none':
                break
            s = self.src[name]
            if not self.gyro_usable(name, t_recv) or s.track.has_gap(t0, t1):
                continue
            ext = self.cfg.max_heading_extrapolation
            h1 = s.track.value(t1, ext)
            if s.last_eval is not None and s.last_eval[0] == t0:
                # What this source said for t0 last step, even if that was an
                # extrapolation: differences then telescope, and an
                # extrapolation error is corrected here instead of kept.
                h0 = s.last_eval[1]
            else:
                h0 = s.track.value(t0, ext)
            if h0 is None or h1 is None:
                continue
            s.last_eval = (t1, h1)
            return name, h1 - h0
        return None

    # ------------------------------------------------------------ internals

    def _slipping(self, t, z):
        """True while wheel speed z is too far from the prediction to believe."""
        f, cfg = self.filter, self.cfg
        gate = cfg.slip_gate * math.sqrt(f.variance + cfg.sigma_wheel ** 2)
        if abs(z - f.v) <= gate:
            return False
        if self._slip_since is None:
            self._slip_since = t
        if t - self._slip_since < cfg.max_slip_time:
            return True
        # Slipping this long is not slip -- more likely the accelerometer is
        # wrong. Believe the wheels: the update that follows re-seeds from them.
        f.reset()
        return False

    def _update_gyro_agreement(self, t):
        if not all(self.gyro_usable(name, t) for name in self.SOURCES):
            self._diff_t = None
            return
        diff = self.src['bno'].yaw_rate - self.src['vesc'].yaw_rate
        if self._diff_t is None:
            self._diff_t = t
            return
        # Signed mean: the VESC is late by its serial round-trip, so in a
        # slalom the instantaneous difference swings both ways and averages
        # out. A wrong sign or scale shows up as a steady offset in a turn.
        alpha = 1.0 - math.exp(-max(t - self._diff_t, 0.0) / self.cfg.gyro_disagree_tau)
        self._diff_mean += alpha * (diff - self._diff_mean)
        self._diff_t = t


# ---------------------------------------------------------------- gyro odometry

MODEL = 'model'      # odometry step whose heading came from vesc_to_odom (steering model)


def wrap_angle(a):
    """a wrapped into (-pi, pi]."""
    w = math.atan2(math.sin(a), math.cos(a))
    return w + 2.0 * math.pi if w <= -math.pi else w


def compose(a, b):
    """2-D pose composition a * b; poses are (x, y, yaw)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, at + bt)


def inverse(p):
    x, y, t = p
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, -t)


@dataclass
class OdomStep:
    stamp: float
    pose: tuple        # (x, y, yaw) of base_link in odom_imu
    correction: tuple  # (x, y, yaw) of odom in odom_imu: the odom_imu -> odom transform
    speed: float       # distance / dt [m/s]
    yaw_rate: float    # heading change / dt [rad/s]
    source: str        # 'bno', 'vesc' or MODEL


class GyroOdometry:
    """Re-integrates vesc_to_odom's odometry with a gyro's heading.

    Feed it every odom -> base_link pose vesc_to_odom publishes, with that
    transform's own stamp. Each step:
      distance = the pose change along the previous wheel heading. That is
                 exactly speed * dt as vesc_to_odom integrated it, so the
                 distance travelled is identical to /odom's.
      heading  = the gyro's change over the same stamps (heading_change), or
                 vesc_to_odom's own yaw change when no gyro is usable.
    The correction is pose * inverse(wheel pose): published as odom_imu ->
    odom, the chain odom_imu -> odom -> base_link is exactly `pose`, while
    vesc_to_odom keeps sole ownership of odom -> base_link. Both start equal.
    """

    def __init__(self, estimator: VehicleStateEstimator):
        self.est = estimator
        self.pose = None
        self._wheel = None       # (stamp, x, y, yaw) of the previous wheel pose

    def on_wheel_pose(self, t_recv, stamp, x, y, yaw):
        """One vesc_to_odom pose; returns an OdomStep, or None if unusable."""
        if not _finite(t_recv, stamp, x, y, yaw):
            return None
        if self._wheel is None:
            self._wheel = (stamp, x, y, yaw)
            self.pose = (x, y, yaw)
            return OdomStep(stamp, self.pose, (0.0, 0.0, 0.0), 0.0, 0.0, MODEL)
        t0, x0, y0, yaw0 = self._wheel
        dt = stamp - t0
        if dt <= 0.0:
            return None                      # duplicate or out of order
        self._wheel = (stamp, x, y, yaw)
        distance = (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)
        found = self.est.heading_change(t_recv, t0, stamp)
        if found is None:
            source, dheading = MODEL, wrap_angle(yaw - yaw0)
        else:
            source, dheading = found
        px, py, pt = self.pose
        # A gyro step moves along its mid-step heading (exact for an arc to
        # second order). A model step moves along its start heading, as
        # vesc_to_odom itself does, so with no gyro at all the pose IS /odom's.
        along = pt if source == MODEL else pt + 0.5 * dheading
        self.pose = (px + distance * math.cos(along), py + distance * math.sin(along),
                     pt + dheading)
        correction = compose(self.pose, inverse((x, y, yaw)))
        return OdomStep(stamp, self.pose, correction, distance / dt, dheading / dt, source)


def foreign_parent(transforms, our_parent, child):
    """The parent frame of `child` in a /tf message that isn't ours, or None.

    transforms: (parent, child) frame-name pairs. A frame has one parent in
    tf2; a second publisher of a parent for `child` (slam_toolbox's
    map -> odom when SLAM was not moved to odom_imu, a particle filter's)
    would make the tree flip between the two.
    """
    for parent, kid in transforms:
        if kid == child and parent != our_parent:
            return parent
    return None
