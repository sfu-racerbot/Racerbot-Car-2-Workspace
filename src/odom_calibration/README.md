# Odom calibration wizard

`odom_calibration` is a guided browser wizard for calibrating the RacerBot
VESC odometry and, optionally, the steering conversion. A human drives the car
with the physical remote and measures ground truth with a tape measure. The
wizard only subscribes to ROS topics: it creates no publishers and cannot
command the car.

## Start it

Build once:

```bash
cd ~/racerbot-ws
colcon build --packages-select odom_calibration --symlink-install
source install/setup.bash
```

Start the car's normal F1TENTH bringup in its usual terminal. In a second
terminal:

```bash
cd ~/racerbot-ws
source install/setup.bash
ros2 launch odom_calibration odom_calibration_launch.py
```

Open `http://<car-ip>:8090/` from a browser on the same trusted network. The
launch terminal prints the address and confirms that the node is read-only.

Reports and the active crash-recovery session are stored in
`~/.ros/odom_calibration/`. The browser can download the final report as JSON
or Markdown.

## What the wizard records

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

- **Movement only:** stationary offset plus forward/reverse distance scale.
- **Movement + steering:** movement tests followed by centred wheels and
  left/right circle measurements.

Verify the current parameter values shown on the setup screen. The report
compares its suggestions against this baseline.

### 2. Preflight

Start normal vehicle bringup, turn on the remote, keep the controls neutral, and
leave the car stationary. The Web UI reports message rate, age, invalid values,
and timestamp regressions for every topic.

`vesc_to_odom` does not publish `/odom` until it has received at least one servo
command. If raw VESC data is healthy but odometry says "missing," briefly hold
LB with throttle and steering neutral so the normal teleop path sends a neutral
command.

### 3. Stationary baseline

Keep every wheel completely still, release LB, and record at least five
seconds. The median forward-positive raw ERPM estimates
`speed_to_erpm_offset`. A large stationary spread creates a warning rather than
a misleading offset suggestion.

### 4. Known-distance movement

1. Mark a straight 5–10 m lane.
2. Measure from the centre of the rear axle at the start to the same point at
   the finish.
3. Start recording before the car moves.
4. Hold LB and drive smoothly with the physical remote.
5. Stop on the mark, release LB, then stop recording.
6. Confirm forward or reverse and enter the positive tape-measured magnitude.

Use at least three trials, ideally two forward and one reverse. The report keeps
all readings signed. A negative gain candidate is excluded and reported as a
direction/sign fault; it is never silently converted with `abs()`.

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

Multiple positive candidates are combined with a median/MAD estimator.
Statistical outliers, non-finite samples, timestamp gaps, and sign disagreements
remain visible in the report.

### 5. Optional steering

Steering calibration uses the fixed physical wheelbase (`0.324 m`). It does not
misuse wheelbase as a tuning parameter.

1. **Centre:** visually align both front wheels, leave the car stationary, and
   record 3–5 seconds.
2. **Left circle:** mark the centre of the rear axle, hold a steady left input,
   and drive one slow complete circle back to the starting heading.
3. Measure the diameter traced by the **rear-axle centre**, not the body or
   outside tire.
4. Repeat to the right.

For measured rear-axle path radius `R`, the physical steering angle is:

```text
steering_angle = atan(wheelbase / R)
```

Left is positive and right is negative. The wizard fits:

```text
servo_value =
    steering_angle_to_servo_gain * steering_angle
    + steering_angle_to_servo_offset
```

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

Only apply a report marked `ready` after reviewing every warning. Update the
shared conversion values in
`src/f1tenth_system/f1tenth_stack/config/vesc.yaml`, rebuild/restart the stack,
then:

1. test forward/reverse sign and steering with wheels off the ground;
2. repeat a short, low-speed tape-measure trial;
3. keep a spotter and hold LB for every powered movement; and
4. retain the old parameter values so the change is reversible.

Tape measurement cannot detect every source of odometry error. Tire slip,
surface changes, tire wear, battery state, drivetrain backlash, and steering
flex can all change results. The report quantifies repeatability but does not
turn wheel odometry into absolute localization.
