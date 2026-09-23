# Guided odometry calibration

> **Who this is for:** anyone whose car reports the wrong distance travelled, or whose steering angle doesn't match reality.
> **Read first:** [operations.md](operations.md) — you need to be able to bring the car up and drive it first.
> **You'll be able to:** calibrate VESC speed odometry and the steering conversion with a tape measure, using a browser wizard.
> **Time:** about 15 minutes per section, about 30 minutes for both, with the car.

## Highlights

- A browser wizard at `http://<car-ip>:8090/`. You drive the car with the physical remote. The page only records.
- **Wheel calibration**: a stationary baseline plus 5–10 m tape-measured distance runs. Three runs are recommended (two forward, one reverse).
- **Steering calibration**: a straight-line drift run plus full-lock left and right circles. One full-lock circle per side is required before the report.
- The report suggests new values but writes nothing itself. You copy them into `vesc.yaml` by hand.
- The wizard creates no ROS publishers. Captures stop by themselves at five minutes.
- Limits: a tape measure cannot see tire slip, surface changes, tire wear, battery state, or drivetrain backlash.

## Why it exists

Odometry (the car's estimate of how far it has travelled and turned; see
[glossary.md](glossary.md)) comes from the VESC (the motor controller board;
see [glossary.md](glossary.md)). Two conversions stand between the raw
readings and reality: the wheel-speed gain and offset, and the
steering-angle-to-servo line.

Both conversions drift from the physical car over time. The wizard drives
measured ground truth — a tape on the floor — against what the car counted,
and suggests corrected values.

## Which part do you need?

Pick **Wheel calibration only** when distances read wrong (the car counts 4 m
for a 5 m lane). Pick **Steering calibration only** when the car pulls to one
side with the stick centred, or its turns don't match. The full procedure for
whichever you pick is in the
[package README](../src/odom_calibration/README.md).

## Start it

Start the car's normal bringup in **terminal 1** (see
[operations.md](operations.md)). Then, in **terminal 2**, after the normal car
bringup:

```bash
cd ~/racerbot-ws
source install/setup.bash
ros2 launch odom_calibration odom_calibration_launch.py
```

It worked when terminal 2 prints the `http://<car-ip>:8090/` address and
confirms the node is read-only.

Open `http://<car-ip>:8090/`. The wizard never publishes a ROS command; the
operator drives with the physical remote and LB (the deadman button: the car
only moves while it is held; see [glossary.md](glossary.md#deadman)).

## Safety

Every powered movement needs LB held and a spotter watching the car. After
applying new values, check steering and drive direction with the wheels off
the ground first, then repeat a short low-speed tape trial.
