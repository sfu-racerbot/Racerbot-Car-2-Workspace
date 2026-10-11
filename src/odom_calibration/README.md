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
command the car. The one thing it can write is `vesc.yaml`, and only after
you confirm in the page; see [Applying a suggestion](#applying-a-suggestion).

## Highlights

- A browser wizard at `http://<car-ip>:8090/`. You drive with the physical remote. The page only records.
- **Wheel calibration**: a stationary baseline plus tape-measured 5–10 m distance runs. Three runs are recommended (two forward, one reverse).
- **Steering calibration**: a wheels-up steering range check, a straight-line drift run, then full-lock left and right half circles. All four are required before the report unlocks.
- The steering range check says whether unequal left/right steering comes from software (stick, teleop scale, servo clamp) or hardware (servo, linkage), layer by layer.
- Steering runs are fitted only from samples taken while LB is held and the car moves at 0.15 m/s or more.
- Impossible suggestions are blocked with a reason; plausible, confident ones can be written to `vesc.yaml` with a backup and an undo.
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

The wizard launch uses UDP for Fast DDS so it can receive telemetry when
bringup runs under a different Linux account. Shared-memory permissions can
otherwise leave every topic missing, with the wizard waiting for odometry
and never seeing LB. If running the node directly rather than this launch,
set `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` in that terminal first.

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
| `/sensors/servo_position_command` | Servo value after `vesc_driver`'s `servo_min`/`servo_max` clamp, used for steering fitting |
| `/commands/servo/position` | Servo value before the clamp; compared with the one above to spot clipping |
| `/ackermann_cmd` | Selected speed/steering command |
| `/joy` | Confirms remote presence, records when LB was held, and records the steering stick axis (`steering_axis`, 3) |

The node reads the live conversion values every 2 seconds, each from the node
that uses it:

- `vesc_to_odom_node`: speed gain and offset, wheelbase;
- `ackermann_to_vesc_node`: steering gain and offset;
- `vesc_driver_node`: `servo_min`, `servo_max`.

It keeps polling after the first answer, so a
bringup restart with new values is noticed. The session's current values
follow the car, and any report built on the old values is cleared. Until the
nodes answer, the setup screen shows the fallbacks from
`config/odom_calibration.yaml` and lets the operator correct them.

## Guided workflow

### 1. Choose a mode

At setup choose **Wheel calibration**, **Steering calibration**, or
**Both calibrations**.

Use **Change calibration** in the header or sidebar
to reopen this choice for an existing session. Cancel keeps that session;
starting the selected calibration archives it and begins at preflight.
Finish any active recording before changing calibration.

The internal mode names are still `movement`,
`steering`, and `movement_steering`.

- **Wheel calibration only** (mode `movement`): stationary baseline plus forward/reverse distance scale. Pick this when distances read wrong.
- **Steering calibration only** (mode `steering`): straight-line drift run plus left/right circle measurements. Pick this when the car pulls to one side or its turns don't match.
- **Both** (mode `movement_steering`): movement tests followed by the steering tests.

The stages follow the mode (`MODE_STAGES` in `session_store.py` and
`web/wizard.js`).

The left sidebar lists **every test as its own numbered step**, in the order
to do them. Do them all, top to bottom:

| Mode | Sidebar steps |
|---|---|
| Wheels | Check the car → Stand still → Distance runs → Report |
| Steering | Check the car → Steering range → Drive straight → Left circle → Right circle → Report |
| Both | Check the car → Stand still → Distance runs → Steering range → Drive straight → Left circle → Right circle → Report |

The **Report** step stays locked until every test step is done: one
stand-still run, two distance runs, one steering range check, one drift run,
and one full-lock circle per side. A half-lock circle does not count.

Every test page has the same layout:

- a one-line **What this finds**;
- a progress strip (for example *Line up → Record → Drive → Stop → Measure*);
- a top-down setup diagram;
- numbered instructions.

The measuring screen after **Stop** shows a second diagram with each distance
lettered **A**, **B** or **C**. The input boxes carry the same letters.

The four steering tests share the server's single `steering` stage. Which
of them is open is browser state only (`app.steeringTest` in
`web/wizard.js`). A reopened session lands on its first unfinished steering
test.

UI regression checks run with `python3 -m pytest src/odom_calibration/test/`
and require Node.js.

- `test_wizard_ui`: calibration selection, replacement confirmation, cancel,
  and the recording guard.
- `test_wizard_steps`: the per-test sidebar steps, the report lock, resuming,
  and the circle measuring form.
- `test_wizard_apply`: the range-check form, blocked suggestions, the write
  buttons and their confirmations, the centre offset card, and the restart
  banner.

None of the package's tests need ROS sourced:
`calibration_math.py`, `steering_chain.py`, `vesc_yaml.py` and
`wizard_actions.py` hold every rule; `calibration_node.py` only wires them to
ROS and HTTP.

`movement` runs preflight, stationary, movement, report. `steering` runs
preflight, steering, report. `movement_steering` runs all five.

Verify the current parameter values shown on the setup screen. The five
editable values are `speed_to_erpm_gain`, `speed_to_erpm_offset`,
`steering_angle_to_servo_gain`, `steering_angle_to_servo_offset`, and
`wheelbase`.

The servo clamp `servo_min` (`0.15`) and `servo_max` (`0.85`) is read live
from `vesc_driver_node`; `config/odom_calibration.yaml` holds the fallback.
Car-specific settings (`vehicle_model`, `physical_width_m`,
`physical_length_m`, `planner_max_steering_rad`) are in the same file, so the
wizard runs on another car by changing that one file.

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

### 5. Steering range check (wheels off the ground)

This test answers "is uneven steering a software or a hardware problem?". A
steering command passes through five places before the wheels move:

```text
stick -> joy_teleop scale -> /ackermann_cmd angle
      -> servo = gain * angle + offset -> vesc_driver clamp (servo_min/servo_max)
      -> servo, horn, tie rods, wheels
```

The first four are software and visible on ROS topics. The last is hardware,
and only a measurement on the wheel can see it.

1. Put the car on a stand with the front wheels off the ground.
2. Zero a phone angle app against the outside of a front tyre, wheels straight.
3. Press **Record**. Hold LB, hold the stick fully left for about 3 seconds
   while a helper reads both front wheels, then fully right for 3 seconds.
4. Centre, release LB, press **Stop**, type the angles (any box may stay
   empty), and tick "servo buzzed or strained" for a side if it did.

For each side the wizard takes the stretch where the command is within 90% of
that side's largest command, so the stick moving out and back is ignored.
It then reports, for each layer, the full-left and full-right value and
whether they match:

| Layer | Flagged when |
|---|---|
| Remote stick travel | either side reaches less than 0.95 of full |
| Steering command | the two sides differ by more than 3% |
| Servo before the clamp | it differs from `gain × angle + offset` by more than 0.01 (the car runs different values from the ones the wizard read) |
| Servo clamp | `servo_min`/`servo_max` changes the value by more than 0.002 |
| Steering the software commands | the two sides differ by more than 1° |
| Wheels, measured | the two sides differ by more than 1.5°, or a wheel turns less than 80% of its command |

The verdict compares the last two rows:

- unequal software, wheels that follow it: **software**;
- equal software, unequal wheels: **hardware**;
- both unequal, by ratios more than 10% apart: **software and hardware**;
- unequal software, equal wheels: something mechanical stops the bigger side early.

Without wheel measurements the verdict covers software only.

The check is recorded with the settings live at the time. After a
`vesc.yaml` change the result says it is out of date until you record it
again.

### 6. Steering

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

After each accepted drift run, the **Centre offset** card shows the offset
that would have driven that run straight:

```text
straight_offset = servo_during_run - gain * drift_angle
```

using the gain that was live for that run. Write it (or copy it), restart
bringup, and drive again. The card says the car drives straight once a run's
drift angle is within 0.0025 rad (0.14°, about 9 cm sideways over 5 m).

A suggestion that moves the offset by more than 0.10 in one step is refused:
that is almost always a line-up or measuring mistake.

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

The circle tests measure how far the wheels really turn. Each one is a
**half circle** (180 degrees), measured straight across:

1. Put a small tape mark on the floor right behind the middle of each rear
   tire.
2. Start recording. Hold LB, hold the steering stick fully to one side, and
   drive slowly until the car points back the way it came.
3. Stop, release LB, stop recording.
4. Mark behind each rear tire again.
5. Measure across the circle between the two inner-tire marks (**A**) and
   between the two outer-tire marks (**B**).

Only the recorded servo value and the typed diameters feed the calculation,
so half a circle gives the same answer as a full one. It is far easier to
measure across.

The recorded servo value is the median of the samples taken **while LB was
held and the car was moving** (at least 0.15 m/s on `/odom`, matched within
0.5 s). Before LB is pressed and after it is released, teleop sends a centred
command, so the servo sits on the offset.

On 2026-10-06 that waiting time
outweighed the driving in every circle, every circle read back the offset,
and the fitted gain came out as 0. A run with fewer than 10 such samples
is not used. Sessions recorded before this filter still load and fall back to
the whole recording, with a warning on each run.

Three more checks catch measuring mistakes:

- a rear-axle radius outside 0.43–20 m is rejected (a typing or unit error);
- odometry yaw short of 0.375 turns warns that the car did not finish a half circle;
- repeat full-lock circles on one side whose radii differ by more than 10% are warned, and lower the confidence.

The default **inner and outer rear tires** method takes those two diameters
and the wizard uses their midpoint, so `radius = (inner + outer) / 4`. The
measuring screen also offers a **rear-axle centre** method behind a link:
mark under the rear-axle centre at both ends, measure one diameter (**C**),
so `radius = diameter / 2`.

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
The Report step only unlocks once the drift run, at least one full-lock left
circle, and one full-lock right circle are accepted.

Full lock means "the stick held fully over", which is not always the servo
limit. The stick asks for at most the `joy_teleop.yaml` steering scale
(0.34 rad); on one side that may stop short of the clamp, on the other the
clamp may cut it.

The servo value the wizard records is the one after the
`vesc_driver` clamp, so it is what the servo really received. The steering
range check shows which side is limited by what.

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

The live steering readout on the circle pages is the **commanded** angle:
the servo value after the clamp, converted back through the current gain and
offset. It is not a measurement of the wheels.

While a circle records, the page shows a live "turns so far" readout. It is
an odometry estimate (integrated odom yaw divided by 2π). It reads about
`0.50` at half a circle: good enough to tell you when to stop, but not
ground truth.

A second circle with the stick held only halfway over is allowed. Use the
"Record a half-lock circle" link on the circle page, or untick full lock
when measuring. It does not count toward the full-lock requirement, but it shows
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
- Suggested values are written to `vesc.yaml` only on an explicit, confirmed request, with a backup first; blocked and low-confidence values cannot be written at all.

## Applying a suggestion

Each suggestion is checked before it is shown:

| Value | Blocked when |
|---|---|
| `speed_to_erpm_gain` | zero, the opposite sign to the configured gain, or outside 0.5–2× of it |
| `speed_to_erpm_offset` | more than 1000 ERPM (the wheels moved during the baseline) |
| `steering_angle_to_servo_gain` | magnitude outside 0.1–5, or the opposite sign to the configured gain |
| `steering_angle_to_servo_offset` | not strictly between `servo_min` and `servo_max` |
| any | its fit is `invalid` or `insufficient` |

A blocked value is listed with its reason and never written as a YAML value
in the page or the downloaded report. A value whose section has `low`
confidence is shown, marked "not ready to apply", and offered for neither
writing nor copying without that note.

**Write to vesc.yaml** (in the report, and on the drive-straight step for the
centre offset) asks for a confirmation, then:

1. copies the current `vesc.yaml` to
   `~/.ros/odom_calibration/backups/vesc.yaml.<UTC time>.<source>.bak`;
2. edits only the requested lines, keeping every comment;
3. re-reads the result and refuses to save it unless exactly the requested
   values changed;
4. saves it atomically.

Speed values go into the **`vesc_to_odom_node` override** only (the odometry
offset line is added after the gain if it is not there yet). The shared `/**`
block's +4614 speed gain sets the motor command and is never touched.
Steering values go into the shared `/**` block.

The file is the one bringup
loads, followed through the `--symlink-install` link back to `src/` (or
`vesc_yaml_path` if set). If it resolves inside `install/` instead, the page
warns that the next `colcon build` will overwrite the change.

**Undo the last change** puts the latest backup back. Writing is refused while
a recording runs, and the whole feature is off with `allow_apply: false`. The
API only accepts `application/json` requests, so another web page open in the
same browser cannot trigger a write.

The car keeps its old values until bringup restarts; every test page shows a
**Restart bringup** banner until the live values match. After that:

1. test forward/reverse sign and steering with wheels off the ground;
2. repeat a short, low-speed tape-measure trial;
3. keep a spotter and hold LB for every powered movement; and
4. keep the backup (or the old values) so the change is reversible.

To copy values by hand instead, the report's YAML blocks list the same values
for the `vesc_to_odom_node` section and the shared steering section of
`src/f1tenth_system/f1tenth_stack/config/vesc.yaml`.

Tape measurement cannot detect every source of odometry error. Tire slip,
surface changes, tire wear, battery state, drivetrain backlash, and steering
flex can all change results. The report quantifies repeatability but does not
turn wheel odometry into absolute localization (a map-based position
estimate; see [glossary.md](../../docs/glossary.md)).
