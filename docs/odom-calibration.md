# Guided odometry calibration

> **Who this is for:** anyone whose car reports the wrong distance travelled, or whose steering angle doesn't match reality.
> **Read first:** [operations.md](operations.md) — you need to be able to bring the car up and drive it first.
> **You'll be able to:** calibrate VESC speed odometry and the steering conversion with a tape measure, using a browser wizard. You'll also find out whether uneven steering is a software or a hardware problem, and write the results into `vesc.yaml`.
> **Time:** about 15 minutes for wheels, 20 for steering, 35 for both, with the car.

## Highlights

- A browser wizard at `http://<car-ip>:8090/`. You drive the car with the physical remote. The page only records.
- **Wheel calibration**: a stationary baseline plus 5–10 m tape-measured distance runs. Three runs are recommended (two forward, one reverse).
- **Steering calibration**: a wheels-up **steering range check**, a straight-line drift run, then full-lock left and right half circles. All four are required before the report unlocks.
- **Hardware or software?** The range check follows full left and full right through every software layer: stick, steering command, and servo value before and after the `servo_min`/`servo_max` clamp. It compares them with wheel angles you read with a phone, and names the first layer where the two sides stop matching.
- **Centre offset, step by step**: each drive-straight run suggests a new `steering_angle_to_servo_offset`. Write it, restart bringup, drive again, until the wizard says the car drives straight (within 0.14°).
- Steering runs use only the samples taken **while LB is held and the car is moving**. Before this, time spent waiting with LB released could swamp a circle and give a steering gain of 0.
- Every test is its own numbered step in the left sidebar. Do them top to bottom. Each step has a setup diagram, and each measuring screen has a lettered diagram (A, B, C) that matches the input boxes.
- A suggestion that cannot be right (zero or wrong-sign gain, offset outside the servo range, a failed fit) is **blocked**: shown with its reason, never written as a value. A weak one (low confidence) is shown but not offered for writing.
- **Write to vesc.yaml** buttons put accepted values into `vesc.yaml` after a confirmation. A timestamped backup is saved first, and **Undo the last change** puts it back. Set `allow_apply: false` in `odom_calibration.yaml` to turn writing off.
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

## Is my steering problem hardware or software?

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

The **Steering range** step finds this kind of problem for you. Put the car
on a stand with the front wheels off the ground, hold LB, and hold the stick
fully left and then fully right for about 3 seconds each. A helper reads each
front wheel's angle with a phone angle app. The report then shows every
layer from stick to wheels and gives one verdict:

| Verdict | Meaning |
|---|---|
| Software | The settings make the sides unequal; the wheels follow them. Fix the settings. |
| Hardware | Software asks for equal angles; the wheels do not turn equally. Check the servo horn, tie rods, bellcrank and physical stops. |
| Software and hardware | Both are unequal, by different amounts. Fix the settings first, then measure again. |
| Software limit, hardware stops early | Software is unequal but the wheels turn equally, so something stops the bigger side. |

When the clamp is the cause, the report gives three ways out:

- re-centre the servo horn so the offset comes back near the middle of the servo range;
- widen `servo_min`/`servo_max`, only after checking by hand that the linkage still moves there;
- cap every steering command at the angle both sides can reach.

It also warns when `gap_follow`/`pure_pursuit` ask for more steering
(`max_steering_angle`, 0.26 rad) than both sides can reach, because those
commands are clipped on one side without any warning.

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
shows a **Restart bringup** banner. The backups are in
`~/.ros/odom_calibration/backups/`, outside git.

## How it was tested

`tools/odom_calibration_e2e/run_e2e.py` runs the wizard against the car's
real VESC conversion nodes, `ackermann_mux` and `joy_teleop`, with only the
hardware simulated. It checks that the wizard recovers a simulated car's true
steering and wheel values, names the right cause of uneven steering, and that
writes, undo and bringup restarts all work. See
[its README](../tools/odom_calibration_e2e/README.md).

## Using it on another car

Everything specific to one car is in
`src/odom_calibration/config/odom_calibration.yaml`. That is `vehicle_model`,
`physical_width_m`, `physical_length_m` and `planner_max_steering_rad`. It also
holds the fallback values used until the live ones are read.

The live values are read
from the running nodes every 2 seconds: speed values from
`vesc_to_odom_node`, steering values from `ackermann_to_vesc_node`, and
`servo_min`/`servo_max` from `vesc_driver_node`. Run the steering range check
first on a new car: it catches a clamp or linkage problem before you spend
time on circles.

To start the wizard with another settings file:
`ros2 launch odom_calibration odom_calibration_launch.py config:=/path/to/file.yaml`.
