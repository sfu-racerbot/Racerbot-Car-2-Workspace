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
- **Steering calibration**: a straight-line drift run, then full-lock left and right half circles, each measured across the inner rear tire only. All three are required before the report unlocks.
- **Finds how far the steering goes**: extra full-lock circles at raised `servo_max` (or lowered `servo_min`) values form a per-side curve. The first step that adds less than 1° marks the linkage's stop, and the report suggests the servo limit and a full-stick angle both sides reach.
- **Check circles**: every circle is compared with the angle the gain believed it commanded; the latest circle on both sides within 1° confirms the calibration.
- **Servo clamp from the settings alone**: warns whenever `servo_min`/`servo_max` would clip full stick or the driving nodes' `max_steering_angle` on one side.
- Steering runs are fitted only from samples taken while LB is held and the car moves at 0.15 m/s or more.
- Impossible suggestions are blocked with a reason; plausible, confident ones can be written to `vesc.yaml` and `joy_teleop.yaml` with a backup of each and one undo for both.
- **Overrides**: step the servo limit from the circle page, type your own value for any suggestion or setting, keep a flagged run, change the rear track, and undo everything from the session. Typed values still get the hard checks below.
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
- `vesc_driver_node`: `servo_min`, `servo_max`;
- `joy_teleop`: the full-stick steering angle
  (`human_control.axis_mappings.drive-steering_angle.scale`).

It keeps polling after the first answer, so a
bringup or teleop restart with new values is noticed. A request that gets no
answer within 5 s (its node was killed by a restart) is sent again.

The session's current values follow the car, and any report built on the old
values is cleared. Until the nodes answer, the setup screen shows the fallbacks from
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
| Steering | Check the car → Drive straight → Left circle → Right circle → Report |
| Both | Check the car → Stand still → Distance runs → Drive straight → Left circle → Right circle → Report |

The **Report** step stays locked until every test step is done: one
stand-still run, two distance runs, one drift run, and one full-lock circle
per side. A half-lock circle does not count. More circles per side are
welcome: they are how the steering limit is found (section 6).

Every test page has the same layout:

- a one-line **What this finds**;
- a progress strip (for example *Line up → Record → Drive → Stop → Measure*);
- a top-down setup diagram;
- numbered instructions.

The measuring screen after **Stop** shows a second diagram with each distance
lettered **A**, **B** or **C**. The input boxes carry the same letters.

The three steering tests share the server's single `steering` stage. Which
of them is open is browser state only (`app.steeringTest` in
`web/wizard.js`). A reopened session lands on its first unfinished steering
test.

UI regression checks run with `python3 -m pytest src/odom_calibration/test/`
and require Node.js.

- `test_wizard_ui`: calibration selection, replacement confirmation, cancel,
  and the recording guard.
- `test_wizard_steps`: the per-test sidebar steps, the report lock, resuming,
  and the circle measuring form.
- `test_wizard_apply`: blocked suggestions, the write buttons and their
  confirmations, the centre offset card, and the restart banner.
- `test_wizard_circles`: the three steering steps and the inner-tire circle
  form. Also the expected-diameter card, the curve and check tables, the
  servo-limit and full-stick write, and the bringup/teleop restart banner.

None of the package's tests need ROS sourced. `calibration_math.py`,
`steering_curve.py`, `steering_chain.py`, `vesc_yaml.py`, `teleop_yaml.py`
and `wizard_actions.py` hold every rule. `calibration_node.py` only wires
them to ROS and HTTP.

`movement` runs preflight, stationary, movement, report. `steering` runs
preflight, steering, report. `movement_steering` runs all five.

Verify the current parameter values shown on the setup screen. The five
editable values are `speed_to_erpm_gain`, `speed_to_erpm_offset`,
`steering_angle_to_servo_gain`, `steering_angle_to_servo_offset`, and
`wheelbase`.

The servo clamp `servo_min` (`0.15`) and `servo_max` (`0.95`) is read live
from `vesc_driver_node`; `config/odom_calibration.yaml` holds the fallback.
Car-specific settings (`vehicle_model`, `physical_width_m`,
`physical_length_m`, `planner_max_steering_rad`, `rear_track_m`) are in the
same file, so the wizard runs on another car by changing that one file.

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

### 5. Servo clamp (no test needed)

A steering command passes through five places before the wheels move:

```text
stick -> joy_teleop scale -> /ackermann_cmd angle
      -> servo = gain * angle + offset -> vesc_driver clamp (servo_min/servo_max)
      -> servo, horn, tie rods, wheels
```

The first four are settings, so the wizard works out the clamp's effect
without driving. It shows how far each side can be commanded before
`servo_min`/`servo_max` clips it. It warns when full stick, or the driving
nodes' `max_steering_angle`, asks for more on either side. The card is on
every circle step and in the report (`steering_chain.clamp_check`).

The wheels-up "steering range" capture with a phone angle app was removed on
2026-10-16. Sessions that contain one still load; the capture is ignored.
Circles at several servo limits (section 6) measure the hardware half.

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

1. Put a small tape mark on the floor right behind the middle of the inner
   rear tire (the one on the inside of the turn).
2. Start recording. Hold LB, hold the steering stick fully to one side, and
   drive slowly until the car points back the way it came.
3. Stop, release LB, stop recording.
4. Mark behind the same tire again.
5. Measure across the circle between the two marks (**A**).

The page shows, before you drive, the diameter **A** a correct calibration
gives with the settings the car runs now.

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
- repeat circles at one setting (servo values within 0.01) whose radii differ by more than 10% are warned, and lower the confidence. Circles at different settings are expected to differ.

The default **inner tire** method adds the configured rear track
(`rear_track_m`, 0.27 m on car 2), so `radius = (inner + track) / 2`.

The measuring screen also offers two other methods behind links:

- **both rear tires**: A and B, so `radius = (inner + outer) / 4`;
- **rear-axle centre**: mark under the rear-axle centre at both ends and
  measure one diameter **C**, so `radius = diameter / 2`.

The both-tires method needs no track-width constant. The axle centre sits
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
(0.436 rad since 2026-10-16); on one side that may stop short of the clamp, on
the other the clamp may cut it.

The servo value the wizard records is the one after the
`vesc_driver` clamp, so it is what the servo really received. The servo clamp
card shows which side is limited by the clamp.

**The fit.** With a drift run, the gain is the least-squares line through the
drift runs' mean point (servo, drift angle), fitted to the circles. Without
one, it falls back to a free line through every point. Circles past a side's
stop (below) are listed but not fitted.

**Finding the stop.** Each side's circles are merged by setting and ordered by
servo travel from the centre. The report lists the wheel angle each step
added. The first step of at least 0.02 servo that adds less than 1.0° marks
the stop at its inner end. The report then suggests:

- `servo_max` (or `servo_min`) at that stop, only ever a value a circle was
  driven at, and only if it differs from the current limit;
- `steering_stick_scale`, the full-stick angle: the smaller side's reach,
  less 0.5°, rounded down to a whole degree, lowered further if the suggested
  limits would clip it under the suggested gain.

When a side's outermost circle is on its servo limit and still gaining, the
report says so. Each circle page's **Find how far the left/right side
steers** card then offers the next step.

That step is `source: 'limit_step'` (`wizard_actions.limit_step_updates`). It moves that side's limit by the step
(0.005–0.1, default 0.05), stopping at 0 or 1. Which limit belongs to which
side follows the gain's sign. When full stick would no longer reach the new
limit, it also raises the full-stick angle to 0.01 rad past it, rounded up to
0.01.

The step-by-step procedure is in
[docs/odom-calibration.md](../../docs/odom-calibration.md#finding-how-far-the-steering-really-goes).

**Check circles.** Every circle stores the angle the gain and offset live at
the time believed they commanded. The report compares the latest full-lock
circle per side with it; both within 1.0° marks the calibration confirmed.

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
`src/pure_pursuit/config/pure_pursuit.yaml`, both `0.349` since 2026-10-16). Do not raise a
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
| `servo_min` / `servo_max` | outside 0–1, or on or past the other limit |
| `steering_stick_scale` | not in 0–0.70 rad |
| any | its fit is `invalid` or `insufficient` |

A blocked value is listed with its reason and never written as a YAML value
in the page or the downloaded report. A value whose section has `low`
confidence is shown, marked "not ready to apply", and offered for neither
writing nor copying without that note.

**Write** (in the report, and on the drive-straight step for the centre
offset) asks for a confirmation, then, for each file it touches:

1. copies the current file to
   `~/.ros/odom_calibration/backups/<file>.<UTC time>.<source>.bak`;
2. edits only the requested lines, keeping every comment;
3. re-reads the result and refuses to save it unless exactly the requested
   values changed;
4. saves it atomically.

Speed values go into the **`vesc_to_odom_node` override** only (the odometry
offset line is added after the gain if it is not there yet). The shared `/**`
block's +4614 speed gain sets the motor command and is never touched.
Steering values and `servo_min`/`servo_max` go into the shared `/**` block.

The full-stick angle goes into `joy_teleop.yaml`, on the one
`human_control` → `drive-steering_angle` → `scale` line
(`teleop_yaml.py`). The `default` profile is never touched. If the second
file fails, the first is put back, so a write lands whole or not at all.

The file is the one bringup
loads, followed through the `--symlink-install` link back to `src/` (or
`vesc_yaml_path` if set). If it resolves inside `install/` instead, the page
warns that the next `colcon build` will overwrite the change.

**Undo the last change** puts back every file the latest write touched.
**Undo everything from this session** (`everything: true`) puts each file
back to its backup from before the session's first write still open, and
closes every write since.

Writing is refused while
a recording runs, and the whole feature is off with `allow_apply: false`. The
API only accepts `application/json` requests, so another web page open in the
same browser cannot trigger a write.

The car keeps its old values until bringup restarts (teleop, for the
full-stick angle); every test page shows a **Restart bringup** or **Restart
teleop** banner until the live values match. After that:

1. test forward/reverse sign and steering with wheels off the ground;
2. repeat a short, low-speed tape-measure trial;
3. keep a spotter and hold LB for every powered movement; and
4. keep the backup (or the old values) so the change is reversible.

### Overrides

| Action | Payload | Checked |
|---|---|---|
| `apply_parameters`, `source: 'manual'` | `values: {name: number}` from the **Change a setting by hand** and **Edit before writing** tables | every *Blocked when* row above except confidence, plus: the offset must stay inside the servo limits as changed |
| `apply_parameters`, `source: 'limit_step'` | `side`, `step`, `raise_stick` | as `manual`, after the step is computed |
| `accept_capture` | `use_despite_warnings: true` | a run with too few LB-held moving samples is used if it has at least one; a disagreeing repeat counts as kept only when every circle at that setting is kept |
| `set_trial_override` | `trial_id`, `value` | the same, for a run already accepted; drops the old report |
| `update_vehicle` | `rear_track_m` (0.10–0.60 m) | this session only; drops the old report |

A kept run's warning stays in the report, reworded to "used anyway, as
chosen". The snapshot's `steering_preview` carries the live curves and each
run's warnings, so the circle pages update before the report is generated.

To copy values by hand instead, the report's YAML blocks list the same values
for the `vesc_to_odom_node` section and the shared steering section of
`src/f1tenth_system/f1tenth_stack/config/vesc.yaml`.

Tape measurement cannot detect every source of odometry error. Tire slip,
surface changes, tire wear, battery state, drivetrain backlash, and steering
flex can all change results. The report quantifies repeatability but does not
turn wheel odometry into absolute localization (a map-based position
estimate; see [glossary.md](../../docs/glossary.md)).
