# How the car works out where it is

> **Who this is for:** anyone whose car "knows where it is" too slowly, jumps around on the dashboard map, or maps a loop that never closes.
> **Read first:** [glossary.md](glossary.md) — you need *frame*, *TF*, *odometry*, and *SLAM*. [racing-autonomy.md](racing-autonomy.md) explains what uses the answer.
> **You'll be able to:** explain where the car's position estimate comes from, which knob affects which part of it, and what would improve it next.
> **Time:** 20 minutes for the first half; the deep dive is reference material.

Every autonomous thing this car does needs one number: **where am I on the map, right now.** This doc is about how that number is produced, why it used to arrive late, and what was done about it.

## Contents

- [What "localization" actually means here](#what-localization-actually-means-here)
- [The two halves, and which one is slow](#the-two-halves-and-which-one-is-slow)
- [What was changed, and why](#what-was-changed-and-why)
- [Checking whether it helped](#checking-whether-it-helped)
- [Deep dive: where the error actually comes from](#deep-dive-where-the-error-actually-comes-from)
- [Deep dive: what would improve this next](#deep-dive-what-would-improve-this-next)
- [Troubleshooting](#troubleshooting)

---

## What "localization" actually means here

A **frame** ([glossary](glossary.md)) is just a coordinate system with a name. This car uses three that matter:

- **`map`** — fixed to the building. The origin is wherever the car was when mapping started. This is the one a **racing line** ([glossary](glossary.md)) — the path the car races round — is stored in.
- **`odom`** — fixed to wherever the car powered up. It drifts over time, but it moves *smoothly*: no jumps, ever.
- **`base_link`** — the car itself.

**TF** ([glossary](glossary.md)) is ROS2's bookkeeping for how these frames relate to each other. The chain is:

```
map ──(SLAM corrects this)──> odom ──(wheel odometry updates this)──> base_link
```

Reading the car's position on the map means composing both links: `map -> odom -> base_link`.

That split is the important idea, and it is worth a second:

- **`odom -> base_link`** is **dead reckoning**. The car counts wheel revolutions and reads its own steering command, then integrates. It updates fast and smoothly, and it is *wrong by a little more every second*.
- **`map -> odom`** is the **correction**. It updates slowly, and each update arrives as a jump.

  **SLAM** ([glossary](glossary.md)) is the software that builds the map and locates the car in it at the same time. It matches the current **LiDAR scan** — one sweep of laser distance readings ([glossary](glossary.md)) — against the map it has already built. The difference tells it how far the dead reckoning has drifted.

So the car always has a position. What it does not always have is a *recently corrected* one.

## The two halves, and which one is slow

| | Who publishes it | How often | Failure mode |
|---|---|---|---|
| `odom -> base_link` | `vesc_to_odom_node` | Every report from the VESC motor controller ([glossary](glossary.md)), fast | Drifts, especially in heading |
| `map -> odom` | `slam_toolbox` | Only when its gates open | Arrives late, as a visible jump |

The complaint that started this work was "finding where it is with respect to the map is really slow." Measured on the 2026-08-19 run, that was the second row.

`slam_toolbox`'s stock configuration will not even *attempt* a correction until the car has travelled 0.5 m, **or** turned 0.5 rad (29°), **and** at least 0.5 s has passed. Between corrections the car is running on dead reckoning alone.

The visible result: `auto_map_race_node` recorded **106 pose jumps larger than 0.12 m in a single 136-second lap** — roughly one snap per second. `slam_toolbox` was also logging `Message Filter dropping message ... queue is full` throughout, i.e. it was throwing scans away.

## What was changed, and why

All of it lives in one file this workspace owns:

**[`src/racerbot_launch/config/slam_tracking.yaml`](../src/racerbot_launch/config/slam_tracking.yaml)**

`slam_launch.py` loads the vendored `f1tenth_stack` config first, then layers this file on top, so later values win.

> **Why a separate file and not an edit to the vendored one?** `f1tenth_system` is vendored as plain tracked files and already carries two local fixes that a careless upstream sync silently overwrites — see [git-setup.md](git-setup.md). This file is ours, so a sync cannot take it, and everything this workspace changed about SLAM is in one place to read.

| Setting | Was | Now | What it does |
|---|---|---|---|
| `minimum_time_interval` | 0.5 | **0.2** | Shortest gap between correction attempts. This was the main cause of "slow". |
| `minimum_travel_distance` | 0.5 m | **0.25 m** | How far the car must move before a new scan is added. |
| `minimum_travel_heading` | 0.5 rad | **0.25 rad** | Same, for turning. 0.5 rad is most of a corner on this car, and cornering is where dead reckoning is worst. |
| `throttle_scans` | 1 | **2** | Feed SLAM every 2nd scan instead of all ~40/s. Fewer scans discarded downstream, less queue pressure. |
| `loop_search_maximum_distance` | 3.0 m | **8.0 m** | How far away SLAM will look for an earlier scan to close a loop against. |
| `loop_match_minimum_chain_size` | 10 | **20** | How many consecutive scans count as evidence for a loop closure. |
| `ceres_loss_function` | None | **HuberLoss** | Caps how far one bad match can drag the whole pose graph. |

Two of those deserve their own explanation, because they are about a different symptom.

### Why the map was not closing

"The map does not close" is a different problem from "localization is slow", and it has its own cause.

The course this car maps is a hallway loop around a building block: **126 m round, with the far side 38.8 m from the start.** Those are measured numbers, read straight off the lap recorder in the 2026-08-19 run.

When the car finally arrives back where it started, SLAM has to *recognise* that it has been there before. It does that by looking for an earlier scan near its current estimated position. Stock, "near" meant **within 3 m**.

After 126 m of driving, the estimate is very unlikely to still be within 3 m of the truth — this car's dead reckoning has no gyro in it at all (see the deep dive below). Nothing inside 3 m means no candidate, means no loop closure, however good the scans are.

8.0 m matches the window SLAM already uses to *verify* a candidate once it finds one (`loop_search_space_dimension`, 8 m, unchanged). Searching a smaller radius than the matcher can correct over was simply leaving that capability unused.

### Why the chain size moved with it

This one is a coupling that is easy to miss.

A "chain" is a run of consecutive scans. Its length **in metres** is `chain_size × minimum_travel_distance`. At the old 0.5 m spacing, 10 scans meant 5 m of corridor as evidence.

Halving the spacing to 0.25 m would have quietly halved that evidence to 2.5 m — in a building full of corridors that look alike, which is exactly where a false loop closure folds the map onto itself. Raising the chain size to 20 keeps the evidence at 5 m of travel.

**If you change `minimum_travel_distance`, change `loop_match_minimum_chain_size` to match.**

## Checking whether it helped

`race_diagnostics` measures pose lag directly, and `auto_map_race_launch.py` now starts it by default.

**Terminal 1** — the run itself. Nothing else needed.

```bash
ros2 launch racerbot_launch auto_map_race_launch.py
```

**Working when:** among the startup lines you see `Recording run to: /home/.../racerbot_runs/<timestamp>`. That directory is where the numbers land.

**If it doesn't:** pass `diagnostics:=false` to turn it off, and check that `race_diagnostics` built (`colcon build --packages-select race_diagnostics`).

**Terminal 2** — after the run, read the result.

```bash
ros2 run race_diagnostics summarize_run ~/.ros/racerbot_runs/<timestamp>
```

**Working when:** it prints a pose section with `lag_max_sec`. Under 0.15 s is healthy; that threshold is `race_diagnostics`' own.

Full detail on what else it records is in [run-diagnostics.md](run-diagnostics.md).

---

## Deep dive: where the error actually comes from

> **Skip this unless** you are trying to improve localization rather than use it. Nothing later depends on it.

The dead-reckoning half — **odometry** ([glossary](glossary.md)), the car's own estimate of how far it has moved — is worth understanding precisely, because it is where nearly all the error is born.

`vesc_to_odom_node` computes the car's heading like this ([`vesc_to_odom.cpp`](../src/f1tenth_system/vesc/vesc_ackermann/src/vesc_to_odom.cpp), lines 110 and 129):

```cpp
current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
...
yaw_ += current_angular_velocity * dt.seconds();
```

That is the **bicycle model**: given a speed and a steering angle, predict how fast the car rotates, then integrate that to get heading.

Now look at where `current_steering_angle` comes from. The config sets `use_servo_cmd_to_calc_angular_velocity: true`, so it is **the last steering command sent to the servo**. Not a measured wheel angle. Not a gyro.

So the car's heading is an open-loop integration of what it *asked* the steering to do. Every one of these becomes unbounded heading error:

- servo calibration error (the `-1.2135` gain and `0.5304` offset in [hardware-reference.md](hardware-reference.md))
- mechanical slop in the steering linkage
- tyre slip, which grows with speed and cornering force
- understeer — at speed the car simply does not turn as tightly as the model says

`vesc_to_odom.cpp` is honest about this: it publishes a yaw covariance of `0.4`, which is large.

**There is no working gyro on this car.** A directly measured yaw rate would beat one inferred from a commanded steering angle by a wide margin, and the VESC publishes `/sensors/imu/raw` at a steady 50 Hz, so it looks like the obvious fix.

Measured on 2026-08-19, **every field of that message is exactly zero.** All three gyro axes, all three accelerometer axes, and no gravity vector anywhere — the VESC is answering the poll with an empty packet.

That is either a board with no IMU fitted, or an IMU disabled in firmware (**VESC Tool → App Settings → IMU**, sample rate 0 means off). It can only be checked in VESC Tool, not from ROS — see [hardware-reference.md](hardware-reference.md#vesc-motor--steering-controller) for the exact re-check procedure.

**Check that before designing anything around an IMU**, because it decides which of the options below is even available.

## Deep dive: what would improve this next

> **Skip this unless** you are picking up localization work. This is a ranked plan, not something already built.

Everything above was tuning: config only, no new dependencies, reversible. Here is what comes after, ranked by how much accuracy they buy per unit of work.

### First, the thing an EKF is not

The obvious-sounding idea is "fuse odometry and LiDAR with an EKF". It is worth being precise about why that specific sentence does not describe a change.

**Odometry and LiDAR are already fused. That is what SLAM is.** `slam_toolbox` takes the odometry as its motion prior, matches the scan against its map, and publishes the difference as `map -> odom`. There is no missing fusion step between those two.

Worse, adding an EKF *between* them would be actively wrong. SLAM's pose output is already a function of the odometry, so feeding both into one filter counts the same information twice. A Kalman filter assumes its inputs are independent; two correlated inputs make it confident in proportion to how often it has fooled itself.

**An EKF is not the missing piece. A second independent sensor is.** With only wheel odometry there is nothing to fuse: an EKF over one sensor is an expensive copy of that sensor.

It becomes worth installing the moment there is a second, genuinely independent measurement of *rotation* — the one channel this car has no sensor for at all.

### 1. Get a real rotation measurement, then fuse it — the big one

**The problem:** heading comes from a commanded steering angle, as above. Nothing on this car measures how much it actually rotated.

**The fix, in the order worth trying:**

**a. Enable the VESC's own IMU, if it has one.** Costs one check in VESC Tool. If the gyro comes alive, everything below becomes easy: `ros-jazzy-robot-localization` (confirmed installable here), one config file, done.

**b. Add an external IMU** if the VESC has no usable one. A USB or I2C gyro is cheap and this is its exact job.

**c. Laser odometry, if no IMU is possible.** Scan-to-scan matching gives an *incremental* rotation from the LiDAR the car already has. That is genuinely independent of the wheels, so it is legitimate to fuse — unlike SLAM's global pose.

It is only *mostly* independent of SLAM, though, which consumes the same scans. Keep the correlation small by fusing **only** the laser's yaw rate, and leaving translation to the wheels.

Neither `rf2o_laser_odometry` nor `laser_scan_matcher` has a Jazzy apt package, so this route means a source build.

**Then the EKF.** `robot_localization`'s `ekf_node` fuses wheel speed (good at *how fast*) with the new sensor (good at *how much it turned*).

It lets you pick exactly which fields of each input to trust: `vx` from `/odom`, `vyaw` from the gyro or laser, nothing else.

An EKF is not exotic. It keeps a running estimate plus an uncertainty, predicts both forward with a motion model, and weights each correction by which it currently trusts more. Two sensors that fail *differently* give an estimate better than either.

SLAM stays exactly where it is, owning `map -> odom`. The EKF only improves what SLAM is correcting, which is the whole point: less drift to correct means loop closure has a chance on a 126 m lap.

**Cost:** one apt package, one config file, and a change to which node owns `odom -> base_link`. `vesc_to_odom_node`'s `publish_tf` goes to `false` so two nodes are not fighting over the same transform.

**Risk:** moderate — it changes the transform every driving node reads. Validate in [`racerbot_sim`](ros-simulator.md) before the car.

### 2. Motion-compensate ("de-skew") the LiDAR scan

A Hokuyo sweep is not instantaneous. At 40 Hz one sweep takes about 25 ms, and every beam in it is taken from a slightly different place because the car kept moving.

At 2.5 m/s the car travels **6 cm during a single sweep**, and in a corner it also rotates several degrees. Scan matching treats all those beams as if they came from one position, which biases every match slightly.

De-skewing uses the odometry (ideally the EKF from item 1) to project each beam back to where the car actually was when that beam was taken. It is standard in fast-moving LiDAR systems and would need a small node between `urg_node` and `slam_toolbox`.

**Do item 1 first** — de-skewing with bad odometry can be worse than not de-skewing at all.

### 3. Use the particle filter for the racing phase — done, 2026-08-22

A **particle filter** ([glossary](glossary.md)) tracks many guesses at once and keeps the ones the LiDAR agrees with. `particle_filter` does this against a *finished* map.

That is a different and easier problem than SLAM's. SLAM builds and localizes at once; a particle filter localizes against a map that is already known and correct, so it can run much faster.

**The auto-map-then-race flow now hands over to it automatically.** Once the map is saved, `auto_map_race_node` starts the particle filter against that map, seeds it with the pose SLAM already knows, and waits for it to settle.

It then republishes *the filter's* estimate on the [topic](glossary.md#topic) that [pure pursuit](glossary.md#pure-pursuit) — the race controller — reads.

`pure_pursuit_node` is not reconfigured and does not notice. The supervisor was always the thing publishing that topic, so the handover changes the source, not the wiring.

Two things had to be true first, and both are now:

- **It had to be affordable.** The filter's ray casting was costing 28.9 ms per scan on the CPU, against 25 ms between scans — it could not keep up. On the GPU the same work takes 6.0 ms. See [gpu-acceleration.md](gpu-acceleration.md).
- **It had to fail safe.** No saved map, a filter that will not start, one that never converges, or one that goes quiet mid-race: each logs the reason and leaves the car racing on `slam_toolbox`, exactly as before. One lapse at racing speed demotes it for the rest of the run rather than letting the car flap between two pose sources.

Turn it off with `localize_after_mapping: false` in `auto_map_race.yaml`.

### 4. Tune further, with measurements

The values in `slam_tracking.yaml` are a deliberately moderate first step, not a floor. Going lower costs CPU: every gate opening adds a node to the pose graph, and at 0.25 m spacing this car's 126 m course is already about 500 nodes per lap for Ceres to optimize on an 8 GB Jetson.

Measure before going further. `race_diagnostics` gives pose lag; `htop` gives the CPU headroom that limits it.

### What was considered and not done

- **Raising `loop_search_maximum_distance` past 8 m.** Pointless without also raising `loop_search_space_dimension`: a candidate found outside the correlation window cannot be verified.
- **Lowering the loop-closure response thresholds** (`loop_match_minimum_response_coarse`/`_fine`). Would accept more closures, including wrong ones. In a building of near-identical corridors a false closure folds the map onto itself, which is much worse than no closure.
- **Replacing the bicycle model in `vesc_to_odom`.** It is vendored code, and an EKF layered on top (item 1) achieves the same thing without a local fix that upstream syncs would clobber.

---

## Safety

**A pose that is stale or frozen is more dangerous than no pose at all.**

**Pure pursuit** ([glossary](glossary.md)), the racing controller, steers toward a point ahead on the racing line based on where it believes the car is. If the belief is a second out of date at 2 m/s, the car is steering from 2 m behind itself — it will cut the corner it thinks it is still approaching. On 2026-07-27 that put this car into a wall.

`pure_pursuit_node` therefore has **watchdogs** ([glossary](glossary.md)): checks that stop the car when something they guard looks wrong. One fires when the pose is too old. Another fires when odometry says the car is moving while localization says it is not.

**Do not loosen those to work around a localization problem.** Fix the localization. The watchdog is the thing that noticed.

Everything in this doc is read-only tuning of an estimate. None of it removes a watchdog, and none of it is a reason to.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Car's dot on the dashboard jumps around | Corrections arriving in large infrequent lumps | Expected before this tuning; if it persists, check `summarize_run` pose lag and consider lowering `minimum_time_interval` further |
| Map never closes into a loop | Loop-closure candidate search too small for the drift | Already raised to 8 m; for a course much bigger than 126 m, raise it *and* `loop_search_space_dimension` together |
| Map suddenly folds onto itself | A false loop closure | Raise `loop_match_minimum_chain_size`, or raise the response thresholds. Repetitive corridors are the usual trigger |
| `Message Filter dropping message ... queue is full` | Scans arriving faster than they can be processed | Raise `throttle_scans` |
| Pose lag fine, car still drives badly | Not a localization problem | See [racing-autonomy.md](racing-autonomy.md) for the control side |

## See also

- [racing-autonomy.md](racing-autonomy.md) — what consumes the pose, and the watchdogs that guard it
- [run-diagnostics.md](run-diagnostics.md) — recording a run and reading the numbers back
- [git-setup.md](git-setup.md) — why the SLAM overrides live outside the vendored config
- [architecture.md](architecture.md) — the full node and topic graph
