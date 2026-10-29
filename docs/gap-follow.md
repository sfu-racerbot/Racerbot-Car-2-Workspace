# Follow-the-gap: swept clearance, arc guard and corner setup

> **Who this is for:** anyone about to run `gap_follow` (the reactive, no-map autonomy) after 2026-10-19, or tune it.
> **Read first:** [src/gap_follow/README.md](../src/gap_follow/README.md) — what follow-the-gap is and how the basic pipeline picks a gap.
> **You'll be able to:** say what the three newer features do and what changed after the first real run, turn any of them off, check them in the simulator, and bring them up on the car in the safe order with the recorder on.
> **Time:** 20 minutes to read; the simulator checks take a few minutes each.

In October 2026 `gap_follow` got three additions that change how it drives through corridors. In simulation the car went from hitting walls in 24 of 60 indoor runs to 0 of 60, and lapped about 20% faster.

They were first driven on the car on 2026-10-19. That run had no contact, but the car hard-stopped seven times. The [2026-10-19 run](#the-2026-10-19-run) section explains why, and what was changed on 2026-10-20 to stop it. **Those 2026-10-20 changes have only run in the simulator and in a replay of that run's own scans.**

## Contents

1. [Highlights](#highlights)
2. [Race defaults (2026-10-27)](#race-defaults-2026-10-27) — what changed, the evidence, and how to go back
3. [What changed, in plain words](#what-changed-in-plain-words)
4. [Safety](#safety)
5. [Validation](#validation) — the order to bring it up on the car
6. [Turning a feature off](#turning-a-feature-off)
7. [The 2026-10-19 run](#the-2026-10-19-run) — why the car braked, and what changed
8. [Several cars at once](#several-cars-at-once) — the other-car layer: on by default, and how to turn it off
9. [Checking it in the simulator](#checking-it-in-the-simulator)
10. [Swept-path clearance](#swept-path-clearance)
11. [Arc guard](#arc-guard)
12. [Corner setup](#corner-setup)
13. [Measured results](#measured-results)
14. [Ideas that were tried and dropped](#ideas-that-were-tried-and-dropped)
15. [Keeping the mirror honest](#keeping-the-mirror-honest)
16. [Steering damping (gyro)](#steering-damping-gyro)
17. [Adaptive grip (IMU)](#adaptive-grip-imu)

## Highlights

- **Every simulated indoor lap completes with no wall contact.** That is 5 layouts × 2 directions × 6 seeds. Before the change, `indoor_tight` and `asb_sharp` touched a wall on every run.
- **Faster where the old ceiling was wrong.** The `asb_10000` hallway lap dropped from 24.6 s to 18.9 s. The old speed limit read the hallway's *side* walls as if they were in front of the car.
- **More room, not less.** The closest the body came to anything went up on every layout. On `asb_10000` it went from 0.07 m to 0.17 m.
- **Each feature has its own off switch** in `config/gap_follow.yaml`, and the simulator measures what each one contributes.
- **Bounded authority.** Arc guard and corner setup can each move the steering by at most half of `max_steering_angle`. The node refuses to start if they are configured for more.
- **Cheap.** About 1.4 ms per LiDAR scan ([glossary](glossary.md#scan)) on average on the Jetson, and about 2.2 ms when the arc guard searches.
- **Tested against the real node.** A test drives the real ROS node (a running program on the ROS graph — [glossary](glossary.md#node)) and the simulator's copy through the same scans, and requires identical commands to 1e-6.

- **No more false hard stops in corridors (2026-10-20).** On the 2026-10-19 run, all 97 stop ticks were the TTC brake reacting to the inside wall of a bend 0.12–0.18 m *beside* the car. Replayed through the final config, the same scans produce none.
- **Other cars: on by default since 2026-10-28.** Two simulated cars on the shipped settings passed 32 of 32 runs, with no contact and no stalls. With the layer off, 24 of 32 passed, with 3 contacts and 5 stalls. It follows slower cars and rarely passes them. Off switch: `opponent_awareness:=false`. It has never run on the floor. See [Several cars at once](#several-cars-at-once).
- **More of every track completes (2026-10-20).** Across 11 simulated tracks, 6 seeds each, both directions, 108 of 132 runs pass, against 96 for the 2026-10-19 config. The hall loop `levine_blocked` laps in 23.6 s instead of 34.7 s.

**Limits:** these numbers come from the simulator only. `max_lateral_accel` was raised from 1.0 to 2.0 m/s² for this work, then to 2.5 m/s² on 2026-10-26.

**Car 2 took the same 2.5 on 2026-10-29** (team decision), although its own floor run (2026-10-19) read 0.91 of the commanded turn at 2.0–2.5. Bring it up in the order in [Validation](#validation).

On car 1, the 2.5 step rests on a map run's grip reading (`20261026-194514`). That run was driven at a 2.0 ceiling, so no run has yet used 2.5 on the floor. The 2026-10-19 floor run and that map run disagree on grip above 2.0 m/s². [The 2026-10-19 run](#the-2026-10-19-run) has both readings.

## Race defaults (2026-10-27)

> **Who this is for:** anyone about to run `gap_follow` on the car, or quoting a number from its simulator runs.
> **Read first:** [Validation](#validation). The new defaults are a speed increase, so the same order applies.
> **You'll be able to:** say what changed on 2026-10-27, read the evidence, and go back to the old setup.
> **Time:** 10 minutes.

The team asked for a faster `gap_follow` with less weave, one that can win races. The shipped setup changed to the one that was fastest in the simulator with no wall contact. **Nothing has been driven on the car at these values yet.**

> **On car 2** (ported from car 1 on 2026-10-29): the speeds, the speed ramp, `max_lateral_accel` 2.5 and corner setup off are the same as car 1's. Two things differ:
>
> - **Steering damping is off** (`enable_steering_damping: false`, `steering_damping:=` defaults to `false`). Car 2 has no BNO085 or `~/imu-calibration` yet, and the IMU stack refuses to start without them. Turn it on together with the other IMU defaults once the BNO085 is mounted and calibrated: see [car1-vs-car2.md](car1-vs-car2.md#imu).
> - **`max_steering_angle` is 0.349 rad**, car 2's limit from its 2026-10-16 circles.
>
> The table below was measured with car 1's values, so car 2's laps will differ.

### What changed

- **Top speed 3.5 → 5.0 m/s** (`max_speed`). A launch can still lower it, for example `max_speed:=1.0` for a first floor run. A launch cannot raise it above 5.0.
- **The speeds tied to the top speed, scaled by 5.0 / 3.5:** `min_speed` 1.0 → 1.429, `corner_speed` 1.5 → 2.143, `corner_speed_wide` 1.9 → 2.714, `narrow_corridor_speed` 2.5 → 3.571. A lower `max_speed:=` also lowers `min_speed`, `corner_speed` and `corner_speed_wide`.
- **Steering damping on** (`enable_steering_damping: true`), gain 0.03 → 0.10 s. It starts the IMU stack, the sensor that reads the car's turn rate. Without a healthy IMU it adds exactly zero.
- **Corner setup off** (`enable_corner_setup: false`). Its values are kept.
- **Speed ramp 3.0 → 5.0 m/s²** (`max_acceleration`). It limits how fast a speed command may rise. It never limits braking.

Left alone on purpose: the LB hold ([deadman](glossary.md#deadman)), the time-to-collision brake, and the emergency and forward clearance stops. Also unchanged are `max_braking_decel` (3.0 m/s²), `max_lateral_accel` (2.5 m/s²) and `max_steering_angle` (0.33 rad on car 1, 0.349 on car 2). Adaptive grip still ships off.

### The evidence

These are simulator runs with `tools/f1tenth_sim/eval_gap.py` on the indoor layouts and car 1's course (`car1_course_2026_10_26`). Each row is 24 runs: both directions, with seeds 12345 and 12346. Every run in every row finished one lap with no wall contact, and every row had 48 stop ticks in total.

| Setting | Mean lap, 24 runs | Car 1 course lap, 4 runs | Steering reversals per second, mean | Car 1 reversals per second |
|---|---|---|---|---|
| Before: the 2026-10-26 setup | 15.04 s | 14.00 s | 0.85 | 1.13 |
| Damping 0.10, corner setup off, speed ramp 5.0 | 13.58 s | 12.55 s | 0.60 | 0.54 |
| **Shipped now:** the same, plus top speed 5.0 and the scaled speeds | **12.26 s** | **12.40 s** | 0.65 | 0.49 |

The "before" row is the same protocol, with the old values passed to `eval_gap.py` as `--set` overrides:

- `max_speed=3.5`, `min_speed=1.0`, `corner_speed=1.5`, `corner_speed_wide=1.9`, `narrow_corridor_speed=2.5`
- `max_acceleration=3.0`, `enable_steering_damping=false`, `enable_corner_setup=true`

### What this evidence does not show

- It is the simulator only. The simulated car is close to this one, not the same. The first floor runs will show what the model misses.
- The simulator gives the controller the gym's true turn rate, with no sensor noise and no delay. The real gyro has both, so steering damping is less tested than the table suggests.
- Steering damping also resists a steady turn. For the same gap bearing, the steady steering is `1 / (1 + gain × speed / wheelbase)` of what it would be without the term, about two thirds at 2 m/s. Watch for the car running wide through a corner.
- The corner cap, where `corner_speed` and `corner_speed_wide` act, never set the speed in these runs. On the car it set the speed on about 80% of the driving ticks in the 2026-08-06 run, so the scaled corner speeds are the least tested part of this change.
- The car's speed loop already lags a 3.0 m/s² ramp (measured 2026-10-27), so on the car the 5.0 ramp mostly removes a software limit. VESC tuning is a team note in [the plan](superpowers/specs/2026-10-26-slam-and-gap-follow-plan.md).

### Going back

- **On the launch line, with no file edit:** `steering_damping:=false` turns the gyro term and the IMU stack off (on car 2 it is already off; `steering_damping:=true` turns it on). `max_speed:=<m/s>` lowers the top speed for one run, and also lowers `min_speed`, `corner_speed` and `corner_speed_wide` with it.
- **In the file:** in `src/gap_follow/config/gap_follow.yaml`, set these back, then restart the launch:
  - `enable_steering_damping: false`, `steering_damping_gain: 0.03` and `enable_corner_setup: true`.
  - `max_speed: 3.5`, `min_speed: 1.0`, `corner_speed: 1.5`, `corner_speed_wide: 1.9`, `narrow_corridor_speed: 2.5` and `max_acceleration: 3.0`.
- Only `max_speed`, `steering_damping`, `adaptive_grip` and `opponent_awareness` are launch arguments. The other settings need the file.

### Other launches that read the same file

- `map_course` and `auto_map_race` load this file too. Unless `mapping_max_speed:=` is given, their mapping laps now run at up to 5.0 m/s. On a course nobody has driven, pass `mapping_max_speed:=1.5`, as [operations](operations.md) already advises.
- On car 1, `gap_follow_launch.py` now starts the IMU stack by default. `map_course` already does there, so do not run the two together. On car 2 neither starts it by default yet (`steering_damping:=false`, `imu:=false`).
- The mapping launches (`map_course`, `auto_map_race` and `autonomous_mapping_launch.py`) start `gap_follow` from this same file, and turn the other-car layer off for their mapping laps. Decided 2026-10-28: mapping is always one car, so they pass `enable_opponent_awareness: false` after the YAML is read.
- Each of those launches takes an `opponent_awareness` argument that defaults to `false`. `opponent_awareness:=true` turns the other-car layer on for that run.

### Before the first run on the car

Follow [Validation](#validation) exactly: wheels off with the stack up, then the floor. On the floor, pass `max_speed:=1.0` for the first run. To test the speed change alone first, turn steering damping off with `steering_damping:=false` (car 2: already off until its IMU is set up).

## What changed, in plain words

Follow-the-gap looks at the LiDAR scan (one sweep of distance readings around the front of the car), finds the widest open gap, and points the wheels at it. It has no map. That is the whole idea, and it still is.

Three things went wrong with that in narrow indoor corridors, and each new feature fixes one:

| Problem | Feature | What it does |
|---|---|---|
| The car crawled down empty hallways. The speed limit used the closest thing in a ±30° cone ahead, and in a 1.5 m hallway that cone hits the side walls. | [Swept-path clearance](#swept-path-clearance) | Limits speed by how far the car's **body** can actually drive along the curve it is on before reaching something. |
| Turning toward a gap sometimes swept the rear of the car into an inside corner. | [Arc guard](#arc-guard) | Before steering, checks that the curve the body would follow is clear. If it isn't, it picks the nearest steering angle whose curve is. |
| The car started turning the moment a bend came into view. It reached every corner on the inside line, the tightest it could pick. | [Corner setup](#corner-setup) | On the approach to a turn, moves the car toward the **outside** wall so it can turn in late, the way a racing line does. |

A *curve* or *arc* here means the path the car follows if it holds its current steering. A car with fixed steering drives in a circle, and the arc is the piece of that circle ahead of it.

## Safety

**Hold LB on the F710 the whole time the car moves. Let go and it stops.** None of these changes touch the deadman or the emergency stop. The time-to-collision brake is still there and still stops the car. On 2026-10-20 its corridor was narrowed to the body plus 0.05 m, an approved and recorded threshold change ([below](#the-2026-10-19-run)).

If you test this with `enable_deadman: false`, a wrong steering decision has nothing between it and a wall except you catching the car. Never do that; it is workspace policy ([architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)).

The rule exists because a driving node with a bug does not know it has a bug. LB is the one stop that doesn't depend on the code being right.

**The 2026-10-20 changes have only run in the simulator. Bring them up wheels-off first, then at low speed** — the exact order is in [Validation](#validation).

The simulator's car is close to this one, but not identical. Its grip has been read on the cars in two runs, but no run has yet used the 2.5 m/s² cornering limit on the floor. Bring that limit up in the order in [Validation](#validation).

**The other-car layer is on by default, and it has never run on the floor.** It can only slow the car or choose a side of an opening. Its first-run order is in [Several cars at once](#several-cars-at-once). If it reads a wall as a car, the decision lines show `opponent mode=` with no other car moving. Let go of LB at that point.

## Validation

**Do these in order and do not skip ahead.** This is the workspace's standard order for any changed driving node ([writing-your-own-node.md](writing-your-own-node.md#testing-before-its-on-wheels)).

1. **Simulator.** Run the [checks below](#checking-it-in-the-simulator) and confirm every indoor run passes.
2. **Wheels off the ground.** Put the car on a stand, start the full stack, hold LB, and move your hand or a box around in front of the LiDAR.
   - **Working when:** the wheels turn away from your hand.
   - The node's decision log (one line a second in the `gap_follow_launch.py` terminal) mentions `swept clearance`, `corner setup` or `arc guard` when they act.
3. **Floor, low speed, open space, with the recorder on.** Lower the speed first: set `max_speed` to about 1.0 m/s and `max_lateral_accel` back to 1.0. Both can be changed live from the dashboard's tuning panel. Record every floor run (commands below), so the next change can be checked against what the car really did.
   - Since 2026-10-27 the top speed is 5.0 m/s unless a launch says otherwise, so this step is not optional ([race defaults](#race-defaults-2026-10-27)).
   - Drive a straight corridor, then a single corner, with someone ready to let go of LB.
4. **Raise speed in steps.** Raise `max_lateral_accel` back toward the shipped 2.5 only after the low-speed runs look right. No run has used 2.5 on the floor yet, so its first run follows the same order as any speed increase: wheels off, then low speed.
   - **If the car slides or runs wide in corners:** the floor has less grip than the simulator. Keep `max_lateral_accel` at the highest value that stayed planted.

If anything looks wrong, turn the suspect feature off (next section) and compare. That is what the separate switches are for.

The other-car layer has its own two-car order, in [Several cars at once](#several-cars-at-once).

### Recording a floor run

**Bringup records the run for you** (since 2026-10-26, [automatic recording](run-diagnostics.md#automatic-recording)). Start Terminals 2 and 3 below, without the `| tee`, and skip Terminal 1. Bringup's terminal prints the run folder when `gap_follow` starts.

The hand-started recorder still works and adds a `launch.log`. Four terminals, started in this order. Full detail is in [run-diagnostics.md](run-diagnostics.md#tldr).

**Terminal 1 — the recorder. Start it first, so nothing is missed:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch race_diagnostics record_run.py
```

**Working when:** it prints the run directory it created, `~/racerbot-ws/data/racerbot_runs/<ts>/`.

**Terminal 2 — hardware, plus the mux ([glossary](glossary.md#mux--multiplexer)) that picks which control layer's commands reach the motor:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** the VESC (the motor controller, [glossary](glossary.md#vesc)) and the LiDAR come up. This never moves the car by itself.

**Terminal 3 — `gap_follow`, logged into the run directory:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch gap_follow gap_follow_launch.py 2>&1 | tee ~/racerbot-ws/data/racerbot_runs/<ts>/launch.log
```

**Working when:** it prints a decision line once a second. The car only moves while you hold LB. Do **not** also run `teleop_launch.py`: its `/teleop` always wins the mux, so `gap_follow`'s commands would never reach the car ([architecture.md](architecture.md)).

**Afterwards, any terminal** — the report from [run-analysis.md](run-analysis.md):

```bash
python3 ~/racerbot-ws/tools/run_analysis/analyze_run.py ~/racerbot-ws/data/racerbot_runs/<ts>
```

**Working when:** it writes `~/racerbot-ws/data/racerbot_runs/<ts>/analysis/report.md`, explaining every stop.

## Turning a feature off

Each feature has a switch in `src/gap_follow/config/gap_follow.yaml`:

| Switch | Default | Off means |
|---|---|---|
| `enable_swept_clearance` | `true` | Speed is limited by the closest reading in the ±30° forward cone again, as before. |
| `enable_arc_guard` | `true` | The car steers exactly where the gap logic says. |
| `enable_corner_setup` | `false` (since 2026-10-27; was `true`) | Shipped off: no move to the outside before a turn. Corridor centering works as before. To turn it on, set it to `true` in the YAML and restart. |
| `narrow_corridor_speed` | `3.571` (since 2026-10-27; was `2.5`) | Set it to `max_speed` or above to turn the narrow-corridor slowdown off. |
| `ttc_path_margin` | `0.05` | Set to `0.18` (the old value, `safety_margin`) for the TTC corridor as it was on 2026-10-19. Never above `safety_margin`; the node refuses to start. |
| `arc_guard_blocked_fallback` | `request` | `freest` brings back the arc guard's old all-blocked behaviour. |
| `enable_steering_damping` | car 2: `false` until its BNO085 is set up ([car1-vs-car2.md](car1-vs-car2.md#imu)); car 1: `true` since 2026-10-27 | No gyro term: the steering is what it was before 2026-10-27. `steering_damping:=true` on the launch line turns it on and starts the IMU stack; `:=false` turns it off and skips the IMU stack. See [Steering damping (gyro)](#steering-damping-gyro). |
| `enable_adaptive_grip` | `false` | The corner speed caps use `max_lateral_accel` alone, as before 2026-10-27. Turn it on with `adaptive_grip:=true`, which also starts the IMU stack. See [Adaptive grip (IMU)](#adaptive-grip-imu). |
| `enable_opponent_awareness` (launch: `opponent_awareness`) | `true` (since 2026-10-28; was `false`) | The other-car layer is off, and the node's commands are what they were before the layer existed (2026-10-22). Launch with `opponent_awareness:=false`. See [Several cars at once](#several-cars-at-once). |
| `opponent_follow_memory_sec` | `1.0` s (since 2026-10-28; was none) | The follow cap is dropped the moment its car is lost, with the layer still on. Set it to `0.0` to get that. |

These switches are **not** live-tunable: edit the YAML and restart `gap_follow_launch.py`. The workspace is built with `--symlink-install`, so no rebuild is needed.

`max_lateral_accel` (shipped 2.5: car 1 since 2026-10-26, car 2 since 2026-10-29; 1.0 before 2026-10-19, then 2.0) is live-tunable from the dashboard.

## The 2026-10-19 run

The first floor run of this rework, recorded as `~/.ros/racerbot_runs/20261019-200703` (archived in `data/racerbot_runs/` since 2026-10-25) and analysed with the tools in [run-analysis.md](run-analysis.md).

**What happened.** One LB hold of 35.4 s: 78 m at 2.2 m/s on average, peak 3.75 m/s, no contact. The closest the padded body came to anything was 0.12 m. The track was a mostly-left-turning loop, 1.24 m wide at the median and 0.83 m at its narrowest 5%.

It also "went slower than expected at times". The car hard-stopped seven times: 97 control ticks commanded to zero speed, all of them by the time-to-collision (TTC) brake.

**Why.** The TTC brake asks, for each LiDAR beam, "at this speed, how soon would I reach that point?", and stops the car if the answer is under 0.30 s. It only counts points near the car's path. Until 2026-10-20, "near" meant within half the car's width plus `safety_margin`: 0.335 m of the path's centre line, which is 0.18 m beyond the body's side.

On this track that is the inside wall of every left bend. Each of the seven stops was that wall, or a step where the track narrowed to about 0.75 m. Every time, the wall was 0.12–0.18 m beyond the body's side, and the body's own swept path was clear for at least 1.5 m. The car was passing the wall, not approaching it.

**What changed on 2026-10-20.** These are the shipped values in `src/gap_follow/config/gap_follow.yaml`, each marked *simulator-validated only* there:

| Parameter | Was | Now | Why, and the evidence |
|---|---|---|---|
| `ttc_path_margin` | (the full `safety_margin`, 0.18 m) | 0.05 m | **A retuned safety threshold, approved on 2026-10-19 on condition of the next row.** The TTC brake now counts a point only within 0.05 m of the body's side. The brake itself, the deadman and the emergency stops are unchanged. Replaying the run's scans: 97 stop ticks become 0. |
| `narrow_corridor_speed`, `narrow_corridor_clearance`, `open_corridor_clearance` | (new) | 2.5 m/s, 0.05 m, 0.15 m | The condition for the row above. With the body within 0.15 m of a side wall, top speed falls linearly toward 2.5 m/s, reached at 0.05 m. The old corridor stopped the car dead for a side wall; this slows it only as far as the squeeze calls for. On the run's scans it would have dropped as low as 3.1 m/s, above every other limit, so it never bound. In the simulator it never bound either: there the car only gets that close to a wall in corners, where the curvature limit is already lower. |
| `braking_latency_sec` | (new; 0) | 0.25 s | The car keeps its speed for 0.275 s after a slower command (measured). The stopping-distance limit now allows for that. In the simulator this is what lets the 1.0 m and 1.2 m layouts pass: 18/18 runs, against 12–14 of 18 at 0.15 s or 0. |
| `max_steering_rate` | 1.0 rad/s | 2.0 rad/s | At 1.0 the steering needs 0.35 s to reach full lock, and square corners were lost to that alone in the simulator. The real servo's rate has not been measured. |
| `steering_gain_speed_ref` | (new; off) | 1.8 m/s | Above 1.8 m/s the steering response shrinks as 1.8/speed. With the measured 0.15 s steering lag, full gain at speed makes the car weave. |
| `arc_guard_blocked_fallback` | (new; `freest`) | `request` | When no arc within the arc guard's reach is clear, keep the gap's own choice. The old behaviour picked the straighter arc and, on a tight right-hander of `room_2026_07_27`, unwound a needed turn into the outer wall. |
| `speed_hold_sec` | (new) | 0 (off) | Holding the lowest speed of a 0.15 s window cut hard brakes by a quarter to two thirds in the simulator, but cost 4–9% of lap time. Left off, available. |
| `max_speed` | 3.5 m/s | 3.5 m/s (unchanged) | 4.5 m/s was tried. Laps were no faster, because corners and clearance limit the speed on these tracks, and more runs failed. |
| `max_lateral_accel` | 2.0 m/s² | 2.0 m/s² on 2026-10-20 (2.5 from 2026-10-26) | 2026-10-19 floor run: the car achieved 99% of the commanded turn up to 2.0 m/s², 91% at 2.0–2.5, 74% above 3.0, and started to slide above 2.0. Map run `20261026-194514` (2026-10-26, driven at a 2.0 ceiling): 99–108% from 0.3 to 2.5 m/s², 101% at 2.0–2.5, 93% at 2.5–3.0, 92% at 3.0–5.0. Its analysis report does not mention sliding. The runs disagree at 2.0–2.5 (91% against 101%) and above 3.0 (74% against 92%). The 2.5 setting rests on the newer run, so the first floor run at 2.5 tests the slide report. |

<details>
<summary><b>How these were chosen</b> — click to expand. Skip it unless you're changing them.</summary>

First the simulator was fitted to the run (`tools/run_analysis/fit_sim.py`, [run-analysis.md](run-analysis.md#fitting-the-simulator-to-the-real-car)). The fit found the real car accelerates harder than it brakes, the opposite of stock gym. Then each candidate was swept over the indoor, narrow and map tracks, 3–6 seeds, both directions. A setting was kept only if it lost no run the old config passed.

Two things were tried and dropped. Gap hysteresis ("stick to the previous aim") and a turn preview ("slow early for a bend seen far ahead") both made no difference or made things worse.

Narrowing the TTC corridor *alone* made the car wedge against walls in the simulator. The braking latency and the arc guard's fallback are what made it safe there. That is why they ship together.

</details>

### Simulated results, before and after

On the simulator fitted to the run, 6 seeds per track. "Before" is the 2026-10-19 config on the same simulator.

| Track | Width | Before: passed fwd / rev | Before: lap | After: passed fwd / rev | After: lap | Stop ticks, before → after |
|---|---|---|---|---|---|---|
| `indoor_oval` | — | 6/6 / 6/6 | 12.5 s | 6/6 / 6/6 | 12.2 s | 12 → 12 |
| `indoor_tight` | — | 6/6 / 6/6 | 11.3 s | 6/6 / 6/6 | 11.9 s | 27 → 12 |
| `indoor_wide` | — | 6/6 / 6/6 | 18.2 s | 6/6 / 6/6 | 16.5 s | 12 → 12 |
| `asb_10000` | — | 6/6 / 6/6 | 20.5 s | 6/6 / 6/6 | 19.7 s | 12 → 12 |
| `asb_sharp` | — | 6/6 / 6/6 | 21.3 s | 6/6 / 6/6 | 18.9 s | 12 → 12 |
| `narrow_1p2` | 1.2 m | 6/6 / 6/6 | 11.8 s | 6/6 / 6/6 | 12.7 s | 90 → 21 |
| `narrow_1p0` | 1.0 m | 3/6 / 2/6 | 14.9 s | 5/6 / 6/6 | 14.0 s | |
| `levine_blocked` | 1.6 m | 6/6 / 3/6 | 34.7 s | 6/6 / 6/6 | 23.6 s | 43 → 12 |
| `room_2026_07_27` | 1.4 m | 6/6 / 4/6 | 8.8 s | 6/6 / 5/6 | 8.3 s | 12 → 12 |
| `circular_obs` | obstacles | 0/6 / 0/6 | — | 2/6 / 0/6 | — | |
| `levine_obs` | obstacles | 0/6 / 0/6 | — | 0/6 / 0/6 | — | |
| **Total** | | **96/132** | | **108/132** | | |

Laps and stop ticks are forward-direction means; a standing start is 12 stop ticks. Every indoor run still passes, 60 of 60.

How to read it:

- **Hallways got faster.** `levine_blocked` is 11 s quicker a lap, `asb_sharp` 2.4 s. The old TTC corridor kept braking for walls the car was only passing.
- **The tightest layouts are a little slower, and much smoother.** `narrow_1p2` loses 0.9 s a lap but has a quarter of the stop ticks and half the hard brakes. That is the braking latency. It is what keeps the car out of the walls on the 1.0 m layout.
- **Failures that remain.** The two obstacle courses wedge the car against a box or a bar. `gap_follow` never reverses, so once its nose is against something it stays there. One `narrow_1p0` run reaches a 90° bend at 1.5 m/s on the outside line and runs out of steering lock; one `room` reverse run does the same.

### Several cars at once

> **Who this is for:** anyone running two or more cars on one track, or checking the other-car layer on the car.
> **Read first:** [Safety](#safety) and [Validation](#validation). The layer is on by default, so the same order applies.
> **You'll be able to:** say what the layer does, turn it off, and run the two-car floor order.
> **Time:** 15 minutes to read; each simulator check takes several minutes.

All cars run `gap_follow` on their own simulated LiDAR, which sees the other cars as obstacles ([run-analysis.md](run-analysis.md#testing-several-cars-at-once)). The leader was held to 1.8 m/s and the others ran at 3.5 m/s, so they catch it. The runs used 4 seeds per track, both directions, 2 laps each. A run passed only if no car touched anything and every car finished.

| Config | 2 cars (5 tracks) | 3 cars (3 tracks) |
|---|---|---|
| 2026-10-19 config | 23/40 | 4/24 |
| 2026-10-20 config, layer off (the shipped default until 2026-10-27) | 34/40 | 15/24 |
| 2026-10-22, layer on (before the follow hold, the memory and the outline) | 40/40, no contact | 24/24, no contact |

Without the layer, every remaining failure is one pattern. A fast car starts to pass a slower one with 0.15–0.2 m between them, because its own path is clear.

The slower car then turns into a bend across that path. At 3.3 m/s the fast car can't stop in the metre left. Follow-the-gap treats every LiDAR return as a fixed wall, so it doesn't know the obstacle beside it can move.

#### Opponent awareness: what it does

**The layer is on by default** (since 2026-10-28). With it on, a moving car is treated as a car. This car slows down behind it, and passes only with room on a straight. It is simulator-validated only, and it has never been driven on the floor.

**The off switch is `opponent_awareness:=false` on the launch line.** The same switch is `enable_opponent_awareness` in `config/gap_follow.yaml`. Anything other than `true` or `false` stops the launch.

**With it off, nothing below runs.** The node's commands are then what they were before the layer existed (2026-10-22). On 2026-10-22, with the layer off, the benchmark above reproduced the 2026-10-20 numbers exactly. A node test checks the same thing: in a world where nothing moves, the commands with the layer on match the commands with it off.

**Before the car, in this order.** Do not skip ahead. Each step is the cheapest place to find a problem the next one would find with a crash.

1. **Wheels off the ground, both cars.** Put each car on a stand. A person holds LB on each car's own F710 controller. Launch with `max_speed:=1.0`.
2. **Then a slow two-car floor run.** Use open space at least 2 m wide, not the 1.2 m track. One car sits about 2 m behind the other, with nobody standing on the track. Launch with `max_speed:=1.0`, and a person holds LB on each controller.
3. **If a wall shows up as a car, stop and report.** On the car, a wall that the layer takes for a moving car shows as `opponent mode=` in the decision lines while no other car is moving. If you see that, let go of LB, relaunch with `opponent_awareness:=false`, and report the run with its log.

Step 1 is the first time the layer runs on the car.

Leaving it on with only one car costs nothing measurable, as far as the simulator shows. On 2026-10-22 the single-car sweep (11 tracks, 6 seeds, both directions) passed 111 of 132 runs with the layer on, against 108 of 132 with it off, and every track did at least as well.

On 2026-10-28, with the 1 s memory added, single-car laps were unchanged (24 of 24).

#### Recognition measurements

Recognition means identifying the other visible car as a moving object in the scan. A false mover is an object identified as moving when only one car is on the track. [The metric definitions](run-analysis.md#testing-several-cars-at-once) explain how these are counted.

A simulator sweep tried `opponent_min_hits` of 3, 4, 6 and 8 with `opponent_moving_speed` of 0.4 and 0.6 m/s. No setting recognised cars faster without extra false movers. At 3–4 hits, car 1's course had up to 180 false-mover ticks. The defaults remain 8 hits and 0.6 m/s.

On 2026-10-28 a second sweep (R3: five solo tracks and two race tracks, both directions) chose the same pair. It had a recognised-tick share of 48.92%, against 48.85% for the next eligible pair. Each of the two had 9 false-mover ticks on car1_course and none on the other tracks.

Across the races, about 49% of ticks when the other car was visible were recognised; on car 1's course it was about 35%. The limit was the other car being close to walls, rather than `opponent_min_hits`. These are simulation measurements, not proof of recognition on the physical car.

#### Existing opponent-awareness procedure

**Terminal 2**, on **every** car on the track. The usual launch is enough, because the layer is on by default:

```bash
ros2 launch gap_follow gap_follow_launch.py
```

**Working when:** with another car moving ahead, the decision lines (one a second) include `opponent mode=follow` and `follow cap=...`. A pass that is allowed shows `opponent mode=clear`. Just after a followed car is lost, the line can show `opponent mode=follow_memory` while the cap is held.

With it on, every scan:

1. **Finds car-shaped things.** It groups the scan into clusters 0.15–0.7 m wide. A cluster counts only if the scan shows open space just past each of its ends. A stub of wall fails that test, because the beam past its end doesn't jump further away.
2. **Tracks them.** It matches clusters from scan to scan against this car's own motion, dead-reckoned from speed and steering, and filters each one into a position and a velocity. A cluster counts as **moving** after 8 scans in a row (0.2 s at 40 Hz) at 0.6 m/s or more over the ground. A parked car stays an obstacle, as before.

   The steering is modelled as the servo actually follows the command, 75 ms late. Using the command itself made parked clutter 0.9 m away read as a 1 m/s car in simulation.
3. **Follows.** Behind a moving car, this car's speed is capped at `(gap − 0.5 m) / 1.0 s`. The gap runs from this car's front bumper to the other car. At 1.8 m/s that allows a 2.3 m gap.

   A car already alongside the front bumper is not followed, because it is being passed. On 2026-10-22 the two car centres came as close as 0.82 m on the 1.2 m loop, and stayed over 2.1 m apart on every other track.
4. **Passes only with room, on a straight.** A pass needs all three of these:
   - this car steers less than 0.10 rad;
   - no car ahead heads more than 15° away from this car's direction, which would mean it is turning;
   - the room beside the other car is at least `car_width + 2 × opponent_passing_margin`, which is 0.61 m on the shipped settings.

   The pass must also be the same opening the car would take anyway. The check runs on every scan, so a pass under way is abandoned the moment the room narrows.

   If the other car is alongside when a pass is refused or abandoned, this car drops back behind it. Alongside means its rear face is at or behind this car's front bumper. This car's speed is then capped 0.5 m/s (`opponent_abort_speed_margin`) below the other car's speed. Two rules keep this from stopping both cars:
   - **Only the car that is behind yields:** the cap applies only when the other car's centre is ahead of this car's. When two of these cars ran side by side and each slowed below the other, both ended at zero (2026-10-29: 0 of 2 runs finished on car 1's course, reversed, with a 2.5 m/s leader).
   - **The cap lasts only while that car is seen.** The follow memory never holds it. A stretch of wall briefly tracked as a moving object beside the car would otherwise brake it for a whole second: in simulation, 3 of 36 two-car runs on car 1's course ended wedged that way.

**The other car's outline.** The layer does not use a 0.30 m disc for the other car. It uses the measured 0.31 × 0.58 m rectangle. The scan sees only the face of the other car that faces this car.

The layer works out the rectangle's centre from that visible face, using the car's length and width (the support-distance rule in `inflate_predicted`). It then widens the rectangle by 0.15 m on every side, and carries it 0.8 s ahead along the other car's motion.

`opponent_radius` (0.30 m) is now only the rear cutoff. A moving car more than 0.30 m behind this car's base link (the rear axle) is ignored.

**The follow hold.** Once a car has counted as moving, this car keeps following it for 1.0 s after the car drops below 0.3 m/s (`opponent_stopped_release_sec`). After that the car is an ordinary obstacle. The hold lowers the speed cap only. It never enables a pass, and its value must stay above zero.

**The follow memory.** If the car being followed is lost for a moment, the [follow cap](glossary.md#follow-cap) is held for 1.0 s (`opponent_follow_memory_sec`) instead of being dropped. A car is lost when its track is dropped, or when it leaves the band the layer watches.

The held cap can lower speed to zero. It never raises speed, overrides a safety stop or authorises motion by itself.

It does not steer. Set the memory to `0.0` to turn it off and keep the rest of the layer on.

**Narrow sections.** If the room beside the other car is narrower than the pass needs, the pass is refused, or abandoned if one is under way, and the car follows instead.

**What the layer cannot do.** It can only slow the car, or choose a side of an opening. It cannot switch off any safety check. The time-to-collision brake, the contact stop, the swept-path speed limit and the arc guard all still read the **real** scan. LB still gates everything: with LB let go, the car commands zero speed.

**It rarely passes.** On the 2026-10-22 benchmark the fast car followed the leader for the whole run on every simulated track, including the wide gym race tracks. The 2026-10-28 sweeps passed rarely too. In the shipped margin's 36-run sweep there was one overtake in total.

On the shipped settings a pass needs at least 0.61 m of room beside the other car. The real track is 1.0–1.3 m wide, where two 0.30 m cars can't pass each other safely anyway. Expect following, not racing.

<details>
<summary><b>The evidence, and the designs that were rejected</b> — click to expand. Skip it unless you're quoting a number or changing a setting. Skipping is safe: the rules above are the whole behaviour.</summary>

**Two cars, the 2026-10-28 matrix (R2b and R2c).** `car1_course_2026_10_26` and `indoor_wide`, both directions, 2 seeds, 3 laps. The car ahead ran at 1.0, 1.5, 2.0 and 2.5 m/s. That is 32 two-car runs per row, and 616 simulator runs in all.

A run passes when no car touches anything and every car finishes. A [stall](glossary.md#stall) is a car below 0.05 m/s for more than 1 s while another car moves at 0.05 m/s or more.

| Layer | Passed | Stalls | Contacts |
|---|---|---|---|
| Off (the shipped config before 2026-10-28) | 24/32 | 5 | 3 |
| On, follow memory 0.0 s | 30/32 | 4 | not given |
| **On, follow memory 1.0 s (shipped)** | **32/32** | **0** | **0** |

Single-car laps were unchanged with the memory on: 24 of 24. The accepted cost is that the faster car rarely passes on wide tracks.

The 8 in the YAML comment ("1 overtake against 8 before") is the awareness-on, no-memory configuration in the same 32-run matrix. All 8 of its overtakes were on indoor_wide, against a leader at 1.0–1.5 m/s. With the 1 s memory on, the same matrix gave 1 overtake.

**The margin sweep, 2026-10-28 (R4).** Leaders at 1.0, 1.5 and 2.0 m/s, on `car1_course_2026_10_26`, `indoor_wide` and `asb_10000`, both directions, 2 seeds, 3 laps. That is 36 runs per setting.

A margin of 0.15 m was the only one with 0 stalls at every steering limit tried (0.10, 0.20 and 0.33 rad). Each of those three settings gave 36 of 36 finished, 0 contacts and 1 overtake. Margins of 0.05 m and 0.10 m overtook up to 4 times, but stalled 1 to 5 times.

The rule for choosing: most overtakes with 0 contacts and 0 stalls, and ties go to the larger time gained by the faster car.

Time gained is the sum, over 18 cells (3 tracks × 3 leader speeds × 2 directions), of the faster car's mean time gained in each cell. Each cell has 2 seeds. eval_gap_multi prints a `mean time gained` figure for each car on the aggregate line under each track, and the faster car's figure is the one summed.

At 0.15 m, the shipped 0.10 rad gained +21.56 s. The 0.33 rad setting gained +21.34 s, and 0.20 rad gained +16.05 s. The raw sweep output is not checked into this repo.

**The follow hold (R2).** Holds of 1, 2 and 3 s each passed all 40 two-car races and 8 solo runs, with no contacts or stalls. The 1 s hold was shipped because the stuck follower in seed 12346 finished in 27.62 s at 1 s, against 29.18 s at 2 and 3 s.

**Rejected designs (R2b).**

- A wider following gap in corners. It regressed at 2.5 m/s and lost passes.
- A reachable-target filter. It gave 39 stalls.
- A deadlock escape. It almost never engaged, because the stuck car is held by `emergency_clearance`, which must never be overridden.
- Releasing the memory during a pass. It gave 13 stalls.

**History (2026-10-22).** The text then said a pass needed about 2.4 m of width with the margins in use, and that these tracks were narrower than that where the leader drives. The outline and the room rule changed on 2026-10-28. A pass now needs at least 0.61 m of room beside the other car.

**Why the follow hold and memory exist.** Behind a slower car on car 1's course, the shipped controller could get stuck for good. The car closed on the slow car in a tight corner, steered round it towards the outer wall, and stopped in a pose that no forward command allowed by its safety rules could leave. `gap_follow` never reverses.

R2b showed these were [controller deadlocks](glossary.md#deadlock), not physical traps. A scripted forward-only escape exists, but the stop rules (time-to-collision, no safe gap, emergency clearance and the creep reserve) hold the car at zero. With the layer on, the main route in was the leader's track being dropped after 7 missed scans in a row. `opponent_max_misses` is 6.

</details>

<details>
<summary><b>How each rule was found, in the simulator</b> — click to expand. Safe to skip.</summary>

Each rule below fixed a failure that was measured first, with the layer on:

| Rule | What went wrong without it |
|---|---|
| 8 detections **in a row** | With one car on track, wall fragments flickered in and out of the clusters. Each reappearance was slightly displaced, which the filter read as motion, so a car alone stalled at 0.1 m/s in the 1.2 m loop. With the rule there were no false movers in 16 one-car runs. |
| Ignore cars more than 1 m to the side, or driving the other way | A car in the other leg of a hairpin, seen through the opening, bent the follower's line into a dead-end corner it could not creep out of. |
| Only pick a side of the **same** opening, never a shallower gap | Same hairpin: the inflated search switched to a different opening. |
| No pass while steering > 0.10 rad, or while the car ahead turns > 15° | A third car passing on the inside of a bend, 0.6–0.8 m to the side, was squeezed into contact (3 cars, `indoor_wide`, reversed, seed 12348). |
| 1.0 s following gap, not 0.6 s | With 0.6 s, two cars stalled in 6 of 8 runs on the 1.2 m loop. With 1.0 s, none did. |
| Turn the dead reckoning with a modelled servo, not the steering command | Turning with the command, the car thought it had turned 75 ms early. That made parked clutter 0.9 m away in the scanned room map read as a 1 m/s car, and one car alone passed 1 of 6 reversed laps there, against 5 of 6 with the layer off. The servo model (`opponent_steer_*`, the delay and slew fitted to the car's 2026-10-19 run) brought it back to 5 of 6. |

The [odometry](glossary.md#odometry--odom) (wheel-speed position estimate) the dead reckoning uses over-reads speed by 5–11% on the real car, and its turning is computed from the steering *command*. In the simulator the car's own heading estimate drifted up to about 25°/s while the steering changed, because the servo lags the command.

The servo model, the 0.6 m/s "moving" threshold and the consecutive-detection rule are what absorb that. A 1.0 m/s threshold was also tried: it did no better with several cars and worse with one (108 of 132). The on-car check for a wall taken for a car is in the main text above.

Code: `src/gap_follow/gap_follow/opponent_tracking.py` (no ROS; 95 test functions in `test_opponent_tracking.py` on 2026-10-28, 71 when this was first written). It is wired in `gap_follow_node.py` and mirrored in `tools/f1tenth_sim/gap_mirror.py`.

Every `opponent_*` parameter is commented in `config/gap_follow.yaml`.

</details>

**Simulator-validated only.** Bring it up in the [usual order](#validation): two cars wheels-off first, then two cars at low speed (`max_speed` about 1.0) with a person holding LB on each controller.

**Checking it, any terminal** (several minutes on the Jetson):

```bash
python3 tools/f1tenth_sim/eval_gap_multi.py --track indoor_oval,indoor_wide,asb_10000,levine_blocked,narrow_1p2 \
    --cars 2 --speeds 1.8,3.5 --seeds 4 --set enable_opponent_awareness=true
```

The layer is on by default, so the `--set` only restates the default. To check the layer off, use `--set enable_opponent_awareness=false`.

**Working when:** every line reads `pass 4/4  wall 0  car-car 0`.

### Racing team A in the simulator

Four simulated races compared team A's shipped controller and safety parameters with car 1's `gap_follow.yaml`, with opponent awareness off. Each was set to 3 laps. These results are simulation only; they do not predict a physical race.

Team A's car touched a wall in every race, ending it before either car finished. These results were measured on 2026-10-28 with team A at commit `6bef23d`. Progress is rounded to three decimal places.

To repeat all four races, complete the [simulator and team A setup](ros-simulator.md#race-team-a) first. Run the commands one at a time; each race ends before the next starts.

**Terminal 1 — parameter copy**, from the workspace root: make a copy with awareness off to match the recorded races.

```bash
sed 's/^\([[:space:]]*enable_opponent_awareness:\) true/\1 false/' src/gap_follow/config/gap_follow.yaml > /tmp/gap-follow-team-a.yaml
```

**Working when:** `/tmp/gap-follow-team-a.yaml` exists and its `enable_opponent_awareness` entry reads `false`. If the source file is not found, check that the terminal is at the workspace root.

**Terminal 1 — races**, from the workspace root, with ROS and the workspace sourced and no real drivers running:

```bash
ros2 launch racerbot_sim sim_race_launch.py track:=car1_course_2026_10_26 opponent:=team_a laps:=3 swap_starts:=false timeout_s:=300 car1_params:=/tmp/gap-follow-team-a.yaml
ros2 launch racerbot_sim sim_race_launch.py track:=car1_course_2026_10_26 opponent:=team_a laps:=3 swap_starts:=true timeout_s:=300 car1_params:=/tmp/gap-follow-team-a.yaml
ros2 launch racerbot_sim sim_race_launch.py track:=indoor_wide opponent:=team_a laps:=3 swap_starts:=false timeout_s:=300 car1_params:=/tmp/gap-follow-team-a.yaml
ros2 launch racerbot_sim sim_race_launch.py track:=indoor_wide opponent:=team_a laps:=3 swap_starts:=true timeout_s:=300 car1_params:=/tmp/gap-follow-team-a.yaml
```

**Working when:** each race prints a standings table and writes a `race.json` under `~/racerbot-ws/data/racerbot_sim/races/`. A wall contact ends the race early. **If it doesn't:** check the terminal for missing team A packages or a hardware interlock message. Timing can vary between runs.

| Track | Start order | Contact time (simulated seconds) | Car 1 laps | Team A laps |
|---|---|---:|---:|---:|
| `car1_course_2026_10_26` | normal | 4.950 | 0.470 | 0.212 |
| `car1_course_2026_10_26` | swapped | 2.925 | 0.142 | 0.206 |
| `indoor_wide` | normal | 4.050 | 0.302 | 0.182 |
| `indoor_wide` | swapped | 2.850 | 0.146 | 0.162 |

**Car 1's awareness setting in these four races: off.** They ran before the default changed on 2026-10-28, so car 1's `gap_follow` had `enable_opponent_awareness` off in all four.

A rerun with today's defaults, which enable opponent awareness, may differ from this table.

Normal starts put car 1 first; swapped starts put team A first. In all four files, car 2 has wall contact, neither car has car contact, both finish times are `null`, and `winner` is `null`.

The measured cause was team A's disparity extender turning in late at 3.0 m/s, then asking for 0.56–0.67 rad of steering. The simulator limits steering to car 2's measured 0.509 rad left and 0.448 rad right (`STEERING_MAX_LEFT/RIGHT` in `sim_bridge.py`), so the car runs wide.

Team A's shipped settings are tuned for its own car: `wheelbase` 0.33 m, `max_speed` 3.0 m/s and `max_lateral_acc` 3.5 m/s². See [how to launch the race](ros-simulator.md#race-team-a).

## Checking it in the simulator

These use the no-ROS simulator ([simulator.md](simulator.md)) and run on the Jetson or a laptop. They never touch the car.

**Any terminal**, from `~/racerbot-ws`:

```bash
# every indoor layout, 6 seeds each, both directions
python3 tools/f1tenth_sim/sweep_gap.py --track indoor --seeds 6
python3 tools/f1tenth_sim/sweep_gap.py --track indoor --seeds 6 --reverse
# the same with one feature off
python3 tools/f1tenth_sim/sweep_gap.py --track indoor --seeds 6 --set enable_arc_guard=false
```

**Working when:** each line reads `pass 6/6 ... contact 0`, and the command exits 0. It takes about 30 seconds on the Jetson.

**If it doesn't:** a `contact` count above 0 means the body touched a wall (the simulator checks this itself; gym's own collision flag is not trusted). Look at that run with the trace tool:

```bash
python3 tools/f1tenth_sim/trace_gap.py --track indoor_tight --plot /tmp/trace.png
```

That writes one row per tick (every speed cap, the chosen gap, side distances, what each feature did) and a plot of the lap.

`--set name=value` overrides any `gap_follow.yaml` parameter for a run. Every other value comes from the YAML itself, so the simulator runs what the car would run.

## Swept-path clearance

**The old speed limit:** the car may drive at most as fast as it can stop in the distance to the closest LiDAR reading within ±30° of straight ahead.

In a 1.5 m hallway, that cone reaches the side walls about 1.25 m away. So on an empty straight the car was capped at about 2.4 m/s by walls it was never going to hit.

**The new limit** uses how far the car's body can drive along its current arc before any part of it touches something. On a straight, that is the distance to the end of the hallway.

It takes the shorter of two arcs: the one the car is steering toward, and the one the steering rack can actually reach this tick. So turning the wheels does not instantly unlock a speed the car cannot use yet.

It only changes the *speed limit*. Creeping, the escape logic and the TTC brake still use the forward cone, exactly as before.

<details>
<summary><b>How the swept distance is computed</b> — click to expand. Skip it unless you're changing <code>gap_logic.swept_path_clearance</code>.</summary>

The car's body is a rectangle `car_width` wide, extending `car_length` from the rear axle (`base_link`). Driving at steering δ, the rear axle follows a circle of radius R = L / tan δ (L = wheelbase).

The body then sweeps a ring around that circle's centre:

- inner edge: R − w/2 (the inside of the rear axle line),
- outer edge: hypot(front, R + w/2) (the outer front corner),

where w is the width plus `swept_clearance_margin` on each side.

Every LiDAR point inside that ring is an obstacle on the path. For each one, the free distance is how far the body's front can travel along the arc before reaching it. The clearance is the minimum over all such points, and `inf` beyond `swept_clearance_max_length` (3.0 m).

A straight path (δ = 0) is the limiting case: a band `w` wide straight ahead.

The function is `swept_path_clearance` in `src/gap_follow/gap_follow/gap_logic.py`. Its tests (`test_gap_logic_corners.py`) check it against hand-built geometry, including points just inside and just outside each edge.

</details>

## Arc guard

The gap logic decides where to point the wheels from the gap's direction alone. It does not check whether the *body* fits along the curve that steering produces.

Usually it does fit. Hugging a wall just before a square corner, it may not: turn toward the opening too early and the rear of the car clips the inner corner.

The arc guard checks the requested arc first. It looks `arc_guard_length` (1.0 m) ahead, or the stopping distance at the current speed if that is longer.

- **Arc clear:** nothing happens. On open track the guard never acts.
- **Arc blocked:** it tries steering angles across the full range (`arc_guard_candidates`, 29 of them) and takes the clear one *nearest* to what was asked.
- **Nothing clear within `arc_guard_max_correction`:** it takes the freest arc within that bound, so it can never swing the wheels further than 0.1745 rad (10°) from the request.

When it acts, the decision log says `arc guard <requested>-><used>rad (requested arc blocked)`. It is off while the car is creeping out of a tight spot.

## Corner setup

Racing drivers take a corner **outside → late apex → outside**: wide before the turn, clipping the inside late, wide again on the exit. That gives the biggest radius, so the most speed and the most room.

Follow-the-gap does the opposite on its own. As a bend comes into view, the gap's middle leans into the turn, so the car drifts to the inside long before the corner. In simulation that put the inner corner about 8 cm from the body on `indoor_tight`, and the TTC brake fired on the way into every corner.

Corner setup adds a small steering bias on the approach that moves the car toward the **outside** wall:

1. **Spot the turn.** On each side, it checks whether LiDAR beams reach well past where that side's wall would be if it carried on straight. If they do, the wall ends there: a turn, or an opening. The signal is the left fraction minus the right.
2. **Check this is a corridor approach.** Both side walls within 1.5–2.5 m, and an end wall ahead between 4.0 m and 1.1 m away. In an open room or on a plain straight it does nothing.
3. **Aim for a point toward the outside wall:** `corner_setup_fraction` (0.6) of the room between the corridor's middle and the outside wall (less half the car and a 10 cm margin). It steers toward that point with the same simple rule corridor centering uses, at most 0.10 rad.
4. **Let go at the turn-in point.** The bias fades out as the end wall gets close (1.6 m → 1.1 m), so the car is free to turn.

Corridor centering fades out while setup is active, so the two never fight. A doorway reads like a small turn while it passes. The other checks and the 0.10 rad bound keep that to a brief nudge.

The decision log shows `corner setup <bias>rad at weight <w>` while it acts.

<details>
<summary><b>Every corner-setup parameter</b> — click to expand. Skip it unless you're tuning the approach.</summary>

All in `config/gap_follow.yaml` under `# --- Corner setup`, with comments there:

| Parameter | Default | Meaning |
|---|---|---|
| `corner_setup_fraction` | 0.6 | how far toward the outside wall to aim, as a fraction of the available room |
| `corner_setup_wall_margin` | 0.10 m | room kept from the outside wall |
| `corner_setup_gain` | 0.6 rad/m | steering per metre of lateral error |
| `corner_setup_max_steering` | 0.10 rad | authority bound (startup refuses more than half of `max_steering_angle`) |
| `corner_setup_zero_signal` / `_full_signal` | 0.03 / 0.12 | turn-signal strength where setup starts / is fully on |
| `corner_setup_sector_lo_deg` / `_hi_deg` | 10 / 80 | the forward-diagonal sector each side is checked in |
| `corner_setup_excess_threshold` | 0.30 m | how far past the projected wall a beam must reach to count |
| `corner_setup_far_zero` / `_far_full` | 4.0 / 2.5 m | fade in as the end wall gets this close |
| `corner_setup_near_full` / `_near_zero` | 1.6 / 1.1 m | fade out by the turn-in point |
| `corner_setup_full_side_distance` / `_zero_side_distance` | 1.5 / 2.5 m | both side walls must be this close (a corridor) |

The node checks `near_zero < near_full <= far_full < far_zero` at startup and refuses to run otherwise.

</details>

## Measured results

Recorded 2026-10-19 on the Jetson, simulator fidelity profile `car`, seeds from 12345. Full numbers are in [gap-follow-sim-results.json](gap-follow-sim-results.json). "Old" is the same code with the three features off and `max_lateral_accel: 1.0`.

| Layout (forward) | Old: passed | Old: lap | New: passed | New: lap | Closest approach, old → new |
|---|---|---|---|---|---|
| `indoor_oval` | 6/6 | 15.9 s | 6/6 | 12.9 s | 0.20 → 0.33 m |
| `indoor_tight` | 0/6 (wall contact) | — | 6/6 | 11.8 s | 0.05 → 0.11 m |
| `indoor_wide` | 6/6 | 20.0 s | 6/6 | 16.3 s | 0.77 → 0.87 m |
| `asb_10000` | 6/6 | 24.6 s | 6/6 | 18.9 s | 0.07 → 0.17 m |
| `asb_sharp` | 0/6 (wall contact) | — | 6/6 | 19.3 s | 0.08 → 0.37 m |

Reverse direction: the same pass counts, laps within 0.1 s. On the three large gym circuits every run passed both before and after; laps went from 125 / 176 / 126 s to 107 / 144 / 110 s (Spielberg / Silverstone / BrandsHatch).

`asb_10000` is the real Applied Science Building 10000-level hallway. `asb_sharp` is the same building with its corners squared off, added for this work because square corners were where the old line failed.

<details>
<summary><b>What each feature contributes on its own</b> — click to expand. Skip it unless you're deciding whether to turn one off.</summary>

Each row turns **one** feature off (or `max_lateral_accel` back to 1.0) and keeps the rest. Forward direction, 6 seeds per layout, recorded 2026-10-19. Every one of these runs still passed with no contact. The old failures needed all three features missing at once.

| Configuration | `indoor_tight` lap / closest / stop ticks | `asb_10000` lap / closest | `asb_sharp` lap / closest |
|---|---|---|---|
| All on (shipped) | 11.8 s / 0.11 m / 19 | 18.9 s / 0.17 m | 19.3 s / 0.37 m |
| `enable_swept_clearance: false` | 13.2 s / 0.12 m / 34 | 22.3 s / 0.16 m | 22.6 s / 0.14 m |
| `enable_arc_guard: false` | 12.4 s / 0.12 m / 35 | 19.4 s / 0.15 m | 19.5 s / 0.18 m |
| `enable_corner_setup: false` | 12.1 s / 0.08 m / 80 | 18.8 s / 0.11 m | 19.4 s / 0.18 m |
| `max_lateral_accel: 1.0` | 15.1 s / 0.09 m / 12 | 23.0 s / 0.13 m | 23.0 s / 0.11 m |

How to read it:

- **Swept clearance is the speed.** Without it the hallway laps lose about 3.5 s.
- **Corner setup is the room.** Without it the closest approach on `indoor_tight` drops from 0.11 m to 0.08 m, and the safety stops fire four times as often.
- **The arc guard matters at square corners.** It halves the stops on `indoor_tight` and doubles the room on `asb_sharp`.
- **`max_lateral_accel` 2.0** (this table's value) is worth about 20% of lap time over 1.0. Its grip on the car has now been read in two runs, and they disagree above 2.0 m/s² (see [the 2026-10-19 run](#the-2026-10-19-run)). The shipped value is 2.5 since 2026-10-26.

A "stop tick" is one control tick (25 ms) commanded to zero by a safety layer. Most layouts show 12: that is the standing start.

</details>

## Ideas that were tried and dropped

Two simpler turn detectors for corner setup were built, measured and removed. They are written down so nobody rebuilds them:

- **The direction of the deepest beam.** About 2 m before a square corner, the deepest return in view is the far outer corner of the end wall, so this pointed the *wrong way*.
- **The direction the gap logic is aiming.** It wanders a few degrees either side of straight on a plain hallway. At a square inner corner it leans *away* from the turn, because the safety bubble around the corner clips the gap's inside edge.

The side-opening signal is measured against each side's own wall, so a car sitting off-centre on a straight reads exactly zero.

An earlier experiment, also reverted, is recorded in [gap-follow-improvement-plan.md](gap-follow-improvement-plan.md).

## Keeping the mirror honest

The simulator does not run the ROS node. It runs a copy of the node's per-scan pipeline, `GapController` in `tools/f1tenth_sim/gap_mirror.py`, because the no-ROS simulator has no ROS graph to put a node on.

A copy can drift. Before this work it had: `run_validation.py`'s old `gap_command` still had five stale constants and was missing three whole features, so its numbers described a controller the car didn't run.

Three things now stop that, all in `src/gap_follow/test/test_gap_mirror_parity.py`:

1. **One source of numbers.** The copy reads every parameter from `config/gap_follow.yaml`. A test fails if the YAML gains a parameter the copy neither uses nor lists as deliberately not modelled.
2. **It refuses what it doesn't model.** With `enable_cornering_anticipation: true` it raises instead of silently ignoring it.
3. **Same commands.** A test drives the real `GapFollowNode` and the copy through the same sequence of scans: straights, corners both ways, wall hugging, a blocked arc, a creep. It requires the same steering and speed every tick, to 1e-6.

**If you change `gap_follow_node._scan_step`, change `gap_mirror.GapController` the same way**, then run the test.

**Terminal 1** — with ROS sourced, on an isolated test domain (never the car's):

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest src/gap_follow/test/test_gap_mirror_parity.py -v
```

**Working when:** `5 passed`. A failure names the tick and whether steering or speed differed.

## Steering damping (gyro)

> **Who this is for:** anyone turning on steering damping, or checking what it does on the car.
> **Read first:** [Safety](#safety) and [Validation](#validation). This changes how the car steers, so the same order applies: wheels off first, then low speed.
> **You'll be able to:** turn it on, tell when it is working, and know what it falls back to.
> **Time:** 10 minutes to read, plus the car checks.

Steering damping is **off by default on car 2** until its BNO085 is mounted and calibrated ([car1-vs-car2.md](car1-vs-car2.md#imu)). Car 1 ships it on since 2026-10-27 ([race defaults](#race-defaults-2026-10-27)). It uses the gyro, the sensor that measures how fast the car is turning (its *yaw rate*), to take a little of the wobble out of the steering.

### What it does, in plain words

The steering wobbles. In two recorded runs from 2026-10-26 it swung back and forth about 0.6 times a second, and the gyro moved in step with it. The weave diagnosis that found this is `tools/run_analysis/weave.py`, run on 2026-10-27.

The correction pushes against the turn. A car turning left fast is asked for a little more right steering, and the other way round.

Its size is `steering_damping_gain` times the yaw rate. The gain is a time, in seconds. At 1 rad/s of turning (about 57 degrees a second) and the shipped gain of 0.10 s (0.03 s before 2026-10-27), the correction is 0.10 rad, about 5.7 degrees.

The term also resists a steady turn. For the same gap bearing, the steady steering is `1 / (1 + gain × speed / wheelbase)` of what it would be without the term: about two thirds at 2 m/s with the shipped gain.

The numbers behind the gain are in the comment above `enable_steering_damping` in `config/gap_follow.yaml`.

### What it never does

It never stops the car. It changes nothing about the LB deadman, the time-to-collision brake or the emergency stops. It only shifts the steering request, and the arc guard and the steering rate limit still check the result, as they check every other steering command.

Hold LB as always. Let go and the car stops, with or without this feature.

### Turning it on

**Terminal 1** — hardware, as in [Recording a floor run](#recording-a-floor-run):

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** the motor controller (VESC) and the LiDAR come up. This never moves the car by itself.

**Terminal 2** — `gap_follow` with its IMU stack. On car 2, ask for it (car 1's bare launch already does this). It needs `~/imu-calibration`, so it works only after the [IMU setup](car1-vs-car2.md#imu):

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch gap_follow gap_follow_launch.py steering_damping:=true
```

**Working when:** the decision lines name a `steering damping` value. In any sourced terminal, `ros2 topic echo /imu/status --once` prints `"state": "ok"`. Keep the car still for about 3 seconds after the launch starts, while the gyro settles.

To run without the gyro term, and without the IMU stack, leave the argument off on car 2, or add `steering_damping:=false` on car 1.

Do not also run `map_course`. It starts the same IMU stack, and two copies would fight over the sensor. Do not run `teleop_launch.py` beside it either: its `/teleop` always wins the mux, as in [Recording a floor run](#recording-a-floor-run).

### What it falls back to

Whenever the gyro cannot be trusted, the correction is exactly zero. The car then steers exactly as it does with the feature off, and it keeps driving. The reason appears in the decision log, and as `steering_damping_off: <reason>` on `/drive_intent`, which the web dashboard's decision panel reads.

| Reason | What it means | What to do |
|---|---|---|
| `imu waiting` | The IMU is still starting: its gyro bias is not removed yet, or the gravity check has not run. | Keep the car still for about 3 seconds. |
| `imu fault` | The IMU reports a fault: all six readings zero, a non-finite value, a failed gravity check, or no sample for too long. | Read `/imu/status`. See [the imu_bridge README](../src/imu_bridge/README.md). |
| `imu stale` | The newest gyro sample is older than `imu_timeout_sec` (0.1 s). | The IMU stream has stopped. Check the imu_bridge output. |
| `no imu` | The node has never received a sample or a status. | Check that the IMU stack is running. |
| `imu yaw not finite` | A NaN or infinite yaw rate arrived with status `ok`. The bridge should report a fault first, so this points at the bridge. | Stop, and look at `/imu/data`. |

### The car test order

Do these in order, as in [Validation](#validation). Each step needs the one before it.

1. **Sign check, wheels off the ground.** Put the car on a stand and start both terminals above. Hold LB, then turn the car by hand to the left, counterclockwise seen from above.
   - The yaw rate should be positive, and the decision line should show a *negative* steering damping value.
   - If the yaw rate goes negative when you turn left, let go of LB. The IMU mount is wrong. [The imu_bridge README](../src/imu_bridge/README.md) says how to check it.
2. **Fallback check, wheels off.** Start the launch while you turn the car. The decision lines should say `steering_damping_off: imu waiting` until the car has been still for about 3 seconds, and only then show a steering damping value.
3. **Floor, low speed.** Set `max_speed` to about 1.0 m/s, as in [Validation](#validation). Drive a straight corridor, then one corner, with LB held and someone ready to let go. If the damping value jumps while the car is going straight, let go of LB and look at `/imu/data` before going on.

### What the simulator says

`tools/f1tenth_sim/gap_mirror.py` has the same term, and `test_gap_mirror_parity.py` checks it against the node. `tools/f1tenth_sim/eval_gap.py --set enable_steering_damping=true` runs it on the simulated tracks. The simulator gives the controller the gym's true yaw rate, with no sensor noise and no delay, so a gain that helps there may help less on the car.

The gain of 0.10 that ships is in the [race table](#race-defaults-2026-10-27). TODO(verify): the sweep at 0.04 and 0.07 is still not written in this doc.

## Adaptive grip (IMU)

> **Who this is for:** anyone turning on the adaptive grip, or checking what it does on the car.
> **Read first:** [Safety](#safety) and [Validation](#validation). This changes how fast the car can take a corner, so the same order applies: wheels off first, then low speed.
> **You'll be able to:** turn it on, tell when it is working, and know what it falls back to.
> **Time:** 10 minutes to read, plus the car checks.

Adaptive grip is **off by default**. It lets the car take corners a little faster while the gyro says the tires are holding. It backs off when they start to slide.

### What it does, in plain words

Every corner has a sideways limit: how hard the tires can push the car before it slides. The node uses a number for that limit, `max_lateral_accel`, which is 2.5 m/s² (metres per second squared) in the shipped config. The speed cap for a corner comes from that number.

The gyro measures the car's *yaw rate*, how fast the car is turning. The node compares that with the turn the steering asks for. The result is the **grip ratio**. A ratio of 1.0 means the tires are making the turn the steering asks for. A ratio below 1.0 means the car is turning less than asked, which is what sliding looks like.

While the ratio stays at or above 0.95, the limit rises by 0.2 m/s² each second, up to 3.0 m/s². When the ratio falls below 0.90, the limit is cut to 85% of itself. It never goes below `max_lateral_accel`.

Three details stop it from reacting to noise:

- The ratio is smoothed over about 0.3 seconds, so one odd reading cannot cut the limit.
- The node judges the tires only when the steering asks for at least 80% of the current limit. On a gentle bend the car is not pushing the tires, so a slide would tell it nothing.
- On a straight, or below 1 m/s, nothing is judged. The limit holds where it is.
- The yaw trails the steering by about 0.15 seconds, so the node compares the yaw with the steering from 0.15 seconds earlier. The lag is the one in the weave diagnosis, noted above `enable_steering_damping` in `config/gap_follow.yaml`.
- It judges only while that steering has moved no more than 0.05 rad over those 0.15 seconds. During a quick change of steering, such as a corner entry, nothing is judged.

The lag compensation fixes one thing and leaves another. A steady turn is judged correctly once 0.15 seconds of steering has been recorded.

After a sudden change of steering, though, the yaw keeps catching up for a few tenths of a second. It reads `1 - exp(-t / 0.15 s)` of the way to its final value.

So it reaches 0.95 only about 0.45 seconds after the change. Until then the ratio reads low, and the limit can be cut. The steering check holds judging during the change itself, not during that catch-up.

### What it never does

It never stops the car. The LB deadman, the time-to-collision brake and the emergency stops work as before. Hold LB as always. Let go and the car stops, with or without this feature.

It can let the car take corners faster than `max_lateral_accel` allows. That is the whole point, and it is why the feature ships off and why the car test order below matters.

### Turning it on

**Terminal 1** — hardware, as in [Steering damping](#steering-damping-gyro):

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** the motor controller (VESC) and the LiDAR come up. This never moves the car by itself.

**Terminal 2** — `gap_follow` with its IMU stack:

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch gap_follow gap_follow_launch.py adaptive_grip:=true
```

**Working when:** the decision lines name an `adaptive grip limit` value. In any sourced terminal, `ros2 topic echo /imu/status --once` prints `"state": "ok"`. Keep the car still for about 3 seconds after the launch starts, while the gyro settles.

The two gyro features can run together: `steering_damping:=true adaptive_grip:=true` starts the IMU stack once. Do not also run `map_course`, and do not run `teleop_launch.py` beside it. Both are the same warnings as in [Steering damping](#steering-damping-gyro).

### What it falls back to

Whenever the gyro cannot be trusted, the limit goes back to `max_lateral_accel`, and the ratio smoothing starts again. The car then corners exactly as it does with the feature off, and it keeps driving. The reason appears in the decision log, and as `adaptive_grip_off: <reason>` on `/drive_intent`. The reasons are the same as for steering damping:

| Reason | What it means | What to do |
|---|---|---|
| `imu waiting` | The IMU is still starting: its gyro bias is not removed yet, or the gravity check has not run. | Keep the car still for about 3 seconds. |
| `imu fault` | The IMU reports a fault: all six readings zero, a non-finite value, a failed gravity check, or no sample for too long. | Read `/imu/status`. See [the imu_bridge README](../src/imu_bridge/README.md). |
| `imu stale` | The newest gyro sample is older than `imu_timeout_sec` (0.1 s). | The IMU stream has stopped. Check the imu_bridge output. |
| `no imu` | The node has never received a sample or a status. | Check that the IMU stack is running. |
| `imu yaw not finite` | A NaN or infinite yaw rate arrived with status `ok`. The bridge should report a fault first, so this points at the bridge. | Stop, and look at `/imu/data`. |

One more case gives no reason, because it is not a gyro fault. If the wheel speed on `/odom` goes stale, the limit holds where it is. Nothing is judged, and the limit is not reset. The time-to-collision brake still stops the car on stale odometry, when it is on.

### The car test order

Do these in order, as in [Validation](#validation). Each step needs the one before it.

1. **Fallback check, wheels off the ground.** Start the launch with the car on a stand, and keep it still. The decision lines should say `adaptive_grip_off: imu waiting` for about 3 seconds, and then `adaptive grip limit 2.50m/s^2`. That is the base, unchanged.
2. **Low speed does nothing, on purpose.** The node judges only when the sideways push the steering asks for reaches 80% of the limit. That push grows with speed squared, so with the shipped 2.5 m/s² (car 1, at its 0.33 rad limit: about 1.45 m/s):
   - at car 2's 0.349 rad steering limit, judging starts at about 1.41 m/s;
   - at 0.2 rad, it starts at about 1.89 m/s;
   - at 1 m/s, the limit never moves.

   A low-speed run therefore shows nothing. That is correct.
3. **Floor, about 2 m/s, open space.** Start with `max_speed:=2.0` added to the launch command, and drive one corner with LB held and someone ready to let go. The limit should stay near 2.5 while the car grips. It should drop if the car slides. If the car feels loose, or the limit keeps falling, let go of LB and restart with `adaptive_grip:=false`.

### What the simulator says

TODO(verify): the simulator's results for the adaptive grip are not written in this doc yet. The controller's solo and head-to-head runs will fill this in.

`tools/f1tenth_sim/gap_mirror.py` has the same rule, and `test_gap_mirror_parity.py` checks it against the node. The simulator gives the controller the gym's true yaw rate, with no sensor noise and no delay. So the car may do worse than the simulator says.

## See also

- [src/gap_follow/README.md](../src/gap_follow/README.md) — the whole follow-the-gap pipeline and every parameter.
- [simulator.md](simulator.md) — the no-ROS simulator these numbers come from.
- [gap-follow-improvement-plan.md](gap-follow-improvement-plan.md) — the original diagnosis that led here.
- [steering-laws.md](steering-laws.md) — the other October 2026 change, to the map-based `pure_pursuit` controller.
