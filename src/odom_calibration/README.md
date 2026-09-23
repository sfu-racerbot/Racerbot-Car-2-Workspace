# Odom calibration wizard

> **Who this is for:** someone reading or changing this package's code.
> **Read first:** [docs/odom-calibration.md](../../docs/odom-calibration.md) for the calibration procedure itself.
> **What's in it:** how the wizard is put together and what it writes.

`odom_calibration` is a guided browser wizard for the RacerBot race car.
It calibrates the VESC (the motor controller board; see
[glossary.md](../../docs/glossary.md)) odometry (the car's estimate of how
far it has travelled and turned; see [glossary.md](../../docs/glossary.md))
and, optionally, the steering conversion.

A human drives the car with the physical remote and measures ground truth
with a tape measure.

The wizard only reads ROS topics (each topic is a named message stream; see
[glossary.md](../../docs/glossary.md)): it creates no publishers and cannot
command the car.

## Highlights

- A browser wizard at `http://<car-ip>:8090/`. You drive with the physical remote. The page only records.
- **Wheel calibration**: a stationary baseline plus tape-measured 5–10 m distance runs. Three runs are recommended (two forward, one reverse).
- **Steering calibration**: a straight-line drift run plus full-lock left and right circles. One full-lock circle per side is required before the report.
- The report suggests parameter values but writes nothing. You copy them into `vesc.yaml` by hand.
- Captures stop automatically at five minutes. Closing the browser or losing Wi-Fi never erases accepted trials.
- Limits: a tape measure cannot see tire slip, surface changes, tire wear, battery state, or drivetrain backlash. Odometry stays an estimate.

## Start it

Build once in **terminal 1**, inside the `~/racerbot-ws` folder (the
workspace, which holds every package; see
[glossary.md](../../docs/glossary.md)):

```bash
cd ~/racerbot-ws
colcon build --packages-select odom_calibration --symlink-install
source install/setup.bash
```

It worked when `colcon` finishes and reports the package built without
errors.

Start the car's normal F1TENTH bringup in **terminal 1** (see
[operations.md](../../docs/operations.md)). Then, in **terminal 2**:

```bash
cd ~/racerbot-ws
source install/setup.bash
ros2 launch odom_calibration odom_calibration_launch.py
```

It worked when terminal 2 prints the `http://<car-ip>:8090/` address and
confirms that the node (a running ROS program) is read-only.

Open `http://<car-ip>:8090/` from a browser on the same trusted network.

Reports and the active crash-recovery session are stored in
`~/.ros/odom_calibration/`. The browser can download the final report as JSON
or Markdown.

## What the wizard records

LB is the deadman button: the car only moves while it is held (see
[glossary.md](../../docs/glossary.md#deadman)). The wizard watches it but
never presses it for you.

| Topic | Purpose |
|---|---|
| `/odom` | Integrated signed distance, measured speed, yaw, and topic health |
| `/sensors/core` | Raw VESC electrical RPM used for a scale estimate independent of the current odometry gain |
| `/sensors/servo_position_command` | Actual servo value used for steering fitting |
| `/ackermann_cmd` | Selected speed/steering intent, for diagnostics only |
| `/joy` | Confirms remote presence and records when LB was held |

The node attempts to read the live conversion parameters from
`vesc_to_odom_node`. If that parameter service is unavailable, the setup screen
shows the configured fallback values and lets the operator correct them before
creating a session.

## Guided workflow

### 1. Choose a mode

At setup the operator ticks one or both boxes: **Wheel calibration** and
**Steering calibration**. The internal mode names are still `movement`,
`steering`, and `movement_steering`.

- **Wheel calibration only** (mode `movement`): stationary baseline plus forward/reverse distance scale. Pick this when distances read wrong.
- **Steering calibration only** (mode `steering`): straight-line drift run plus left/right circle measurements. Pick this when the car pulls to one side or its turns don't match.
- **Both** (mode `movement_steering`): movement tests followed by the steering tests.

The stages follow the mode (`MODE_STAGES` in `session_store.py` and
`web/wizard.js`).

`movement` runs preflight, stationary, movement, report. `steering` runs
preflight, steering, report. `movement_steering` runs all five.

Verify the current parameter values shown on the setup screen. The five
editable values are `speed_to_erpm_gain`, `speed_to_erpm_offset`,
`steering_angle_to_servo_gain`, `steering_angle_to_servo_offset`, and
`wheelbase`.

The servo clamp `servo_min` (`0.15`) and `servo_max` (`0.85`)
come from this node's own configuration in
`config/odom_calibration.yaml` and must match `servo_min`/`servo_max` in
`vesc.yaml`. The report compares its suggestions against this baseline.

### 2. Preflight

Start normal vehicle bringup, turn on the remote, keep the controls neutral, and
leave the car stationary. The Web UI reports message rate, age, invalid values,
and timestamp regressions for every topic.

`vesc_to_odom` does not send `/odom` until it has received at least one servo
command. If raw VESC data is healthy but odometry says "missing," briefly hold
LB with throttle and steering neutral so the normal teleop (hand-driving with
the gamepad) path sends a neutral command.

### 3. Stationary baseline

Keep every wheel completely still, release LB, and record at least five
seconds. The median forward-positive raw ERPM (electrical RPM reported by the
VESC) estimates `speed_to_erpm_offset`. A large stationary spread creates a warning rather than
a misleading offset suggestion.

### 4. Known-distance movement

1. Mark a straight 5–10 m lane.
2. Measure from the centre of the rear axle (the point on the ground halfway
   between the two rear wheels) at the start to the same point at the finish.
3. Start recording before the car moves.
4. Hold LB and drive smoothly with the physical remote.
5. Stop on the mark, release LB, then stop recording.
6. Confirm forward or reverse and enter the positive tape-measured magnitude.

Use at least three trials, ideally two forward and one reverse. The report keeps
all readings signed.

A candidate whose sign disagrees with the configured odometry gain is
excluded as a direction/sign fault; it is never silently converted with
`abs()`. This car uses a **negative** gain in `vesc_to_odom_node` to cancel
the driver's raw-ERPM negation; the motor-command gain remains positive.

The local VESC odometry code uses:

```text
raw_forward_erpm = -VescState.speed
speed = (raw_forward_erpm - speed_to_erpm_offset) / speed_to_erpm_gain
```

For each trial, the wizard integrates raw ERPM after subtracting the stationary
offset:

```text
candidate_gain =
    integral(raw_forward_erpm - suggested_offset) / signed_tape_distance
```

If raw VESC samples are unavailable but `/odom` is usable, it degrades to:

```text
candidate_gain =
    current_gain * integrated_odom_distance / signed_tape_distance
```

Multiple sign-consistent candidates are combined with a median/MAD (median
absolute deviation) estimator
on their magnitudes, then restored to the configured gain sign.
Statistical outliers, non-finite samples, timestamp gaps, and sign disagreements
remain visible in the report.

### 5. Steering

Steering calibration fits the line that turns a steering angle into the servo
command for the steering servo (the small actuator that points the front
wheels, driven with values from 0 to 1):

```text
servo_value =
    steering_angle_to_servo_gain * steering_angle
    + steering_angle_to_servo_offset
```

Left is positive and right is negative.

Steering calibration uses the fixed physical wheelbase (`0.36 m`, measured
2026-08-24). It does not
misuse wheelbase as a tuning parameter.

A stationary capture with the wheels visually centred (`steering_center`,
kept for old sessions) cannot find the true centre. With the stick neutral
the servo command is computed *from* the configured offset, so the capture
just reads that offset back.

Readings within `0.002` of the configured
offset get a warning ("only reads the current setting back") that points at
the drift test instead.

The drift test measures the true centre from how the car actually drives.
Tape a straight line at least 6 m long. Put the rear-axle centre on the line
with the car pointing along it.

Hold LB and drive slowly about 5 m forward without touching the steering
stick. Then measure the forward distance `s`
along the line and the sideways offset `d` at the end (left positive).

For a path that bends at a steady rate, the radius is:

```text
R = (s^2 + d^2) / (2d)
steering_angle = atan(wheelbase / R)
```

Line the car up carefully before a drift run. Even 1 degree of heading
misalignment is about 9 cm of sideways error over 5 m, which looks exactly
like a steering offset that is not really there.

<details>
<summary>Where the drift formula comes from (safe to skip)</summary>

A circle that starts tangent to the lane line and passes through the end
point `(s, d)` has radius `R = (s^2 + d^2) / (2d)`. The kinematic bicycle
model then gives `steering_angle = atan(wheelbase / R)`. The code computes
the same thing as `atan(wheelbase * 2d / (s^2 + d^2))`.

</details>

The circle test measures how far the wheels really turn. Mark the ground
under the rear-axle centre. Hold LB, hold the steering stick fully to one
side, and drive one slow full circle back to the mark. Then measure the
circle with either method the wizard offers.

The **rear-axle centre** method takes one diameter: the circle traced by the
rear-axle centre, so `radius = diameter / 2`. The **inner and outer rear
tires** method takes two diameters and the wizard uses their midpoint, so
`radius = (inner + outer) / 4`.

The tire-midpoint method needs no track-width constant. The axle centre sits
exactly midway between the two rear tires, so averaging the two tire radii
gives the centre directly.

The difference is the rear track, which the report shows as a free
consistency check: circles whose implied track
widths disagree by more than 3 cm produce a re-measure warning.

<details>
<summary>Why the midpoint needs no constant (safe to skip)</summary>

If the inner tire traces radius `r` and the outer traces `r + track`, the
centre traces `r + track / 2`, which is exactly the mean of the two radii.
No separate track-width number is needed; `circle_radius` returns the track
as a bonus check instead.

</details>

Hold the stick fully over for the graded circles (the "full lock" checkbox).
The steering page only offers "Build report" once at least one full-lock
left circle and one full-lock right circle are accepted.

Full lock measures the real tightest turn because of the servo clamp. The
servo command the wizard records is the value after `vesc_driver` applies
the `servo_min`/`servo_max` clamp, so pushing the stick past the clamp
cannot turn the wheels any further.

The report's per-side table lists each side's full-lock radius and angle
plus an "On servo limit" column (within
`0.002` of a limit counts as on it).

The report also fits per-side gains from the centre/drift points plus each
side's circles. If the left and right gains differ by more than 10%, the
report warns that one straight line does not describe the linkage well and
calls the suggestion a compromise. Re-measure before applying it, and check
the linkage and trim for a physical cause.

The "Servo limits allow up to … deg left and … deg right" line runs the
`servo_min`/`servo_max` limits through the suggested line. It is the
steering the car can actually reach with those values.

Compare it against `max_steering_angle` in the driving nodes' configs (for example
`src/gap_follow/config/gap_follow.yaml` and
`src/pure_pursuit/config/pure_pursuit.yaml`, both `0.26`). Do not raise a
driving node's `max_steering_angle` past what the car reaches: the planner
must only ask for angles the servo can produce.

While a circle records, the page shows a live "turns so far" readout. It is
an odometry estimate (integrated odom yaw divided by 2π), good enough to
tell you when the circle is complete, but not ground truth.

A second circle with the stick held only halfway over is allowed (uncheck
full lock). It does not count toward the full-lock requirement, but it shows
whether the steering is linear.

It warns about unstable servo input, missing left/right coverage, unexpected
yaw signs, high fit residual, or a fitted gain whose sign reverses the current
configuration.

## Resilience and safety behavior

- Active session state is atomically written after every accepted action.
- Closing the browser or losing Wi-Fi does not stop an active recording or
  erase accepted trials.
- A backend restart marks an in-progress capture interrupted and never accepts
  its partial data.
- Captures automatically stop at five minutes and have bounded sample memory.
- Large message gaps are excluded from integration rather than filled with an
  assumed value.
- Every capture must be reviewed and explicitly confirmed.
- Accepted trials can be removed before regenerating the report.
- Replacing a session archives the old session JSON first.
- Suggested values are never written to `vesc.yaml` automatically.

## Applying a suggestion

Only apply a report marked `ready` after reviewing every warning. Apply speed gain/offset suggestions to the **`vesc_to_odom_node` override**
only. Do not copy its negative gain into the shared motor-command parameters.

Steering suggestions apply to the shared steering conversion values in
`src/f1tenth_system/f1tenth_stack/config/vesc.yaml`, rebuild/restart the stack,
then:

1. test forward/reverse sign and steering with wheels off the ground;
2. repeat a short, low-speed tape-measure trial;
3. keep a spotter and hold LB for every powered movement; and
4. retain the old parameter values so the change is reversible.

Tape measurement cannot detect every source of odometry error. Tire slip,
surface changes, tire wear, battery state, drivetrain backlash, and steering
flex can all change results. The report quantifies repeatability but does not
turn wheel odometry into absolute localization (a map-based position
estimate; see [glossary.md](../../docs/glossary.md)).
