# Guided odometry calibration

> **Who this is for:** anyone whose car reports the wrong distance travelled, or whose steering angle doesn't match reality.
> **Read first:** [operations.md](operations.md) — you need to be able to bring the car up and drive it first.
> **You'll be able to:** calibrate VESC speed odometry and the steering conversion with a tape measure, using a browser wizard. You'll also find how far the steering really goes on each side, and write the results into `vesc.yaml` and `joy_teleop.yaml`.
> **Time:** about 15 minutes for wheels, 15 for steering (plus about 5 per extra circle when finding the steering limit), 30 for both, with the car.

## Highlights

- A browser wizard at `http://<car-ip>:8090/`. You drive the car with the physical remote. The page only records.
- **Wheel calibration**: a stationary baseline plus 5–10 m tape-measured distance runs. Three runs are recommended (two forward, one reverse).
- **Steering calibration**: a straight-line drift run, then full-lock left and right half circles. All three are required before the report unlocks.
- **One tape measurement per circle.** You mark only the inner rear tire and measure across the circle; the wizard adds the car's rear track (0.27 m on car 2, set in `odom_calibration.yaml`). Measuring both tires or the axle centre is still offered.
- **Finding how far the steering goes**: drive more full-lock circles while raising `servo_max` (or lowering `servo_min`) a step at a time. The first step that adds less than 1° is the linkage's stop. The report then suggests a servo limit that stays off it, and a full-stick angle both sides reach.
- **The fit goes through the drive-straight run.** The gain is the line through the measured straight-ahead point that best fits the circles; circles past the stop are shown but left out.
- **Check circles confirm it.** Each circle is compared with the angle the gain *believed* it commanded. When the latest circle on both sides is within 1°, the report says the calibration is confirmed.
- **Servo clamp, from the settings alone**: the wizard always works out whether `servo_min`/`servo_max` cut full stick or the driving nodes' `max_steering_angle` short on one side. That is the problem behind the 2026-10-10 12.3° right side, found with no test at all.
- **Centre offset, step by step**: each drive-straight run suggests a new `steering_angle_to_servo_offset`. Write it, restart bringup, drive again, until the wizard says the car drives straight (within 0.14°).
- Steering runs use only the samples taken **while LB is held and the car is moving**. Before this, time spent waiting with LB released could swamp a circle and give a steering gain of 0.
- Every test is its own numbered step in the left sidebar. Do them top to bottom. Each step has a setup diagram, and each measuring screen has a lettered diagram (A, B, C) that matches the input boxes.
- A suggestion that cannot be right (zero or wrong-sign gain, offset outside the servo range, a failed fit) is **blocked**: shown with its reason, never written as a value. A weak one (low confidence) is shown but not offered for writing.
- **Write** buttons put accepted values in place after a confirmation: gain, offset and servo limits into `vesc.yaml`, the full-stick angle into `joy_teleop.yaml`. A timestamped backup of each file is saved first, and **Undo the last change** puts back every file that write touched. Set `allow_apply: false` in `odom_calibration.yaml` to turn writing off.
- **You stay in charge of every number** (see [Overriding the wizard](#overriding-the-wizard)):
  - type your own value for any suggestion before writing it;
  - change any setting by hand from any page;
  - keep a run the wizard flags;
  - change the rear track for this session;
  - **Undo everything from this session** puts every file back the way it was before the first write.

  The same safety checks apply to a number you type as to a suggestion.
- The wizard creates no ROS publishers and cannot move the car. Captures stop by themselves at five minutes.
- Limits: a tape measure cannot see tire slip, surface changes, tire wear, battery state, or drivetrain backlash.

## Why it exists

Odometry (the car's estimate of how far it has travelled and turned; see
[glossary.md](glossary.md)) comes from the VESC (the motor controller board;
see [glossary.md](glossary.md)). Two conversions stand between the raw
readings and reality: the wheel-speed gain and offset, and the
steering-angle-to-servo line.

Both conversions drift from the physical car over time. The wizard compares
measured ground truth — a tape on the floor — against what the car counted,
and suggests corrected values.

## Why does one side steer less than the other?

On 2026-10-10 the wizard showed about 19.5° of steering to the left and only
12.3° to the right. Both numbers came from software settings, not from the
wheels:

- Full stick asks for 0.34 rad (19.5°) each way, set by the steering
  `scale` in `joy_teleop.yaml`.
- The servo value is `-1.2135 × angle + 0.5904`. Full right needs 1.003, but
  `vesc_driver` clips every servo value to `servo_max: 0.85`. That leaves
  only 12.3° on the right.
- Raising the offset from 0.5304 to 0.5904 to stop the left drift moved the
  whole range towards `servo_max` and used up the right-hand headroom.

The wizard now finds this from the settings alone. Every circle step and the
report show a **Servo clamp** card. It says how far each side can be
commanded before `servo_min`/`servo_max` clips it.

It also warns when full stick, or the driving nodes' `max_steering_angle`
(0.349 rad since 2026-10-16), asks for more than that on either side. The car
clips those commands without any warning, so it turns wider than it was told.

Fixed on 2026-10-16 with full-lock circles at several `servo_max` values.
The gain itself was about 1.5× too large (the car really turned 29.2° left),
so the fix was a new gain of `-0.78`, `servo_max: 0.95` and a 25° full-stick
scale, not only a wider clamp. Numbers and the measured curve are in
[hardware-reference.md](hardware-reference.md).

## Finding how far the steering really goes

The servo itself may travel 180°, but the VESC's 0–1 servo range and the
steering linkage stop the wheels much sooner. Near full lock each servo step
turns the wheels less, until a step adds nothing. To find that point:

You never edit a file by hand for this. Each circle page has a
**Find how far the left/right side steers** card that does it:

1. Do the drive-straight run and one full-lock circle each way as normal.
   The card shows a table: each servo value you drove, the wheel angle it
   gave, and how much each step added.
2. Press **Write servo_max 1.000 (now 0.950)** (or `servo_min`, on the other
   side). The step is 0.05; change it in the **Step size** box (0.005–0.1).
   If full stick would no longer reach the new limit, a ticked box also
   raises the full-stick angle just past it, for now. The report suggests
   the final full-stick angle later.
3. Restart bringup (and teleop, `teleop_launch.py`, the manual-driving
   program, if full stick was raised). The banner on the page says which.
4. **Wheels off the ground first:** hold LB and full lock for 3 seconds.
   Stop at any buzzing, heat, or the linkage touching something, and press
   **Undo the last change**.
5. Put the car down, drive another full-lock circle on that side and accept it.
6. Repeat until the card says that side **levels off**: a step that adds
   less than 1°, about one tape measurement's error. The button stops at the
   end of the 0–1 servo signal.

The report then suggests:

- the `servo_max`/`servo_min` at the last circle before the stop (only ever a
  value you drove);
- a full-stick angle: the smaller side's reach, less 0.5°, rounded down to a
  whole degree, and never clipped by the suggested limits.

Write them, restart bringup and teleop, and drive one more full-lock circle
each way. When both match what the car commanded within 1°, the report says
the calibration is confirmed. Autonomy's `max_steering_angle` is not written
by the wizard: set it by hand, below what both sides reach.

<details>
<summary><b>Where the numbers come from</b> (safe to skip)</summary>

An inner-tire circle of diameter A, with rear track t, has a rear-axle radius
R = (A + t) / 2. The bicycle model gives the wheel angle as
atan(wheelbase / R). With car 2's 0.36 m wheelbase and 0.27 m track, a
1.33 m inner circle is 24.2°.

The gain is the least-squares slope through the drive-straight run's measured
point: its servo value and its small drift angle. Pinning the line there means
a few full-lock circles cannot move the centre the car really drives straight
at. Circles at servo
values within 0.01 of each other count as repeats of one setting.

</details>

## Overriding the wizard

The wizard suggests; you decide. Every override below is written to the
same files with the same backup and undo as a suggestion.

| You want to | Where | What happens |
|---|---|---|
| Write your own number instead of a suggestion | Report, **Edit before writing** under *How far the steering goes* | Boxes start at the suggestions. Tick the ones to write and change any number, including one marked *not ready*. |
| Change any setting at any time | **Change a setting by hand**, folded at the bottom of every page | Gain, offset, `servo_min`, `servo_max`, full stick (in degrees) and the odometry speed values. The table shows what the car runs now. |
| Use a run the wizard flags | Tick **Use this run even though it has the warnings above** when accepting, or press **Use anyway** in the runs table | The warning stays visible and says *used anyway, as chosen*. A run with no LB-held moving samples is still never used. |
| Use a different rear track | Circle pages, **Car measurements** | For this session only. To change it for good, edit `rear_track_m` in `odom_calibration.yaml`. |
| Go back to where you started | **Undo everything from this session**, under *Changes written* | Each file goes back to its backup from before this session's first write. Then restart bringup and teleop. |

Typing a number skips only the *confidence* check; it is your evidence, not
the fit's. These are still refused, because the car could not steer or
would steer the wrong way:

- a gain of zero or the wrong sign;
- an offset outside `servo_min`–`servo_max`;
- a servo limit outside 0–1, or one that meets the other limit;
- full stick past 0.70 rad (40°).

## Which part do you need?

Pick **Wheel calibration only** when distances read wrong (the car counts 4 m
for a 5 m lane). Pick **Steering calibration only** when the car pulls to one
side with the stick centred, or its turns don't match. The full procedure for
whichever you pick is in the
[package README](../src/odom_calibration/README.md).

## Start it

Start the car's normal bringup in **terminal 1** (see
[operations.md](operations.md)). Then, in **terminal 2**:

```bash
cd ~/racerbot-ws
source install/setup.bash
ros2 launch odom_calibration odom_calibration_launch.py
```

It worked when terminal 2 prints the `http://<car-ip>:8090/` address and
confirms the node (a running ROS program) is read-only.

Open `http://<car-ip>:8090/`. The wizard never publishes a ROS command; the
operator drives with the physical remote and LB (the deadman button: the car
only moves while it is held; see [glossary.md](glossary.md#deadman)).

## Safety

Every powered movement needs LB held and a spotter watching the car. After
applying new values, check steering and drive direction with the wheels off
the ground first, then repeat a short low-speed tape trial.

Writing to `vesc.yaml` changes the values every driving node uses. The car
only picks them up after bringup is restarted; until then every test page
shows a **Restart bringup** banner. A new full-stick angle is in
`joy_teleop.yaml` and needs teleop restarted, which the banner also says.
The backups are in `~/.ros/odom_calibration/backups/`, outside git.

Raising `servo_max` (or lowering `servo_min`) moves the servo somewhere it has
never been. Always check it with the wheels off the ground first, and stop at
any buzzing, heat or contact: a servo held against a stop overheats.

## How it was tested

`tools/odom_calibration_e2e/run_e2e.py` runs the wizard against the car's
real VESC conversion nodes, `ackermann_mux` and `joy_teleop`, with only the
hardware simulated. The simulated linkage has a hard stop on the right, and
the run raises `servo_max` from 0.85 to 1.00 one circle at a time, with
the page's own step button.

It checks that the wizard recovers the simulated car's true steering and wheel
values, and finds the stop at the right step. It also checks the servo_max and
full-stick suggestions, writes to both files, undo, restarts and the check
circles.

It also checks each override: a kept run, the rear track, a typed value (and
a refused one), and undo-everything returning both files exactly. See
[its README](../tools/odom_calibration_e2e/README.md).

## Using it on another car

Everything specific to one car is in
`src/odom_calibration/config/odom_calibration.yaml`. That is `vehicle_model`,
`physical_width_m`, `physical_length_m`, `planner_max_steering_rad` and
`rear_track_m`. It also holds the fallback values used until the live ones
are read.

`rear_track_m` is the rear tire centre-to-centre distance. Measure it once
with the "both rear tires" option: outer minus inner diameter is twice the
track.

The live values are read from the running nodes every 2 seconds:

- speed values from `vesc_to_odom_node`;
- steering gain and offset from `ackermann_to_vesc_node`;
- `servo_min`/`servo_max` from `vesc_driver_node`;
- the full-stick angle from `joy_teleop`.
 On a new car, read the **Servo clamp** card before driving any
circle: it catches a clamp problem with no test at all.

To start the wizard with another settings file:
`ros2 launch odom_calibration odom_calibration_launch.py config:=/path/to/file.yaml`.
