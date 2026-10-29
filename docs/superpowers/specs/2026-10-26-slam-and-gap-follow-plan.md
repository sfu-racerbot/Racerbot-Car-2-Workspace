# SLAM and follow-the-gap: the 2026-10-26 plan

> **Who this is for:** the car 1 team, and whoever builds any part of this, human or agent. Implementers must also read `CLAUDE.md`, `TEST_QUALITY_STANDARDS.md`, [architecture.md](../../architecture.md) and [gap-follow.md](../../gap-follow.md) before starting.
> **Read first:** [architecture.md](../../architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car), the LB [deadman](../../glossary.md#deadman) rule. Nothing in this plan relaxes it.
> **You'll be able to:** see everything asked for on 2026-10-26 in one place, with the evidence behind each item. You'll also see what is built today, what waits, and how each piece is proven before the car drives on it.
> **Time:** 10 minutes for [the plan in one screen](#the-plan-in-one-screen). The rest is a work reference, read one item at a time.

**Status:** proposed 2026-10-26.
- Part A is being built today on branch `worktree-slam-gapfollow-2026-10-26`.
- Part B was designed and approved section by section in chat on the same day, and waits.

## Contents

- [The plan in one screen](#the-plan-in-one-screen)
- [What today's runs showed](#what-todays-runs-showed)
- [Decisions already made](#decisions-already-made)
- [Part A: today](#part-a-today)
- [Part B: later](#part-b-later)
- [How every change is proven](#how-every-change-is-proven)
- [Open questions for the team](#open-questions-for-the-team)

## The plan in one screen

Two problems were reported on 2026-10-26:
- `map_course` never saved a map. It relies on [SLAM](../../glossary.md#slam) (building the map while working out where the car is on it), which in turn relies on [odometry](../../glossary.md#odometry--odom) (the car's own estimate of how far it moved and turned).
- [Follow-the-gap](../../glossary.md#follow-the-gap) (`gap_follow`) is too slow and weaves through corners.

The table lists every item asked for that day.

| # | Item | Why (evidence) | When | Done when |
|---|---|---|---|---|
| A1 | SLAM profile for good odometry | False loop closures moved the car's pose 2.2-2.6 m in every gyro run | today | Replays of the three gyro runs have no jump onto a neighbouring stretch of track, and walls are single |
| A2 | Offline SLAM replay | SLAM settings can't be tuned safely by driving | today | A recorded run replays through SLAM on either odometry, and the tool refuses to run beside the drivers |
| A3 | Laps close at the closest approach | Every lap closed at about 29° of heading error against a 15° limit | today | Replaying today's poses, the laps pass the heading check |
| A4 | `max_lateral_accel` 2.0 → 2.5 | The car achieved 93-107% of its commanded turn up to 3.0 m/s² sideways | today | Config and tests changed together |
| A5 | Find why the steering weaves, then fix it behind a flag | Steering swings more than 0.2 rad about once a second; 22 speed dips in 48 s | today | Simulator: less weave, no new wall contacts |
| A6 | Corner setup on vs. off | It pulled against the turn on 95% of the ticks it was active | today | Numbers reported; whether to flip the default is the team's call |
| A7 | Race pace | Curve cap bound 54% of the time, acceleration ramp 31% | today: measure; build later | Each lever is costed in seconds per lap |
| B1 | SLAM observability | Diagnosing today took hand-written scripts | later | One command reports a run's SLAM health |
| B2 | IMU flags in `map_course` | Requested | later | `imu_lap_turns`, `imu_all` |
| B3 | Simulated IMU | IMU features need closed-loop testing before the car | later | `racerbot_sim` publishes `/imu/*` |
| B4 | IMU features in `gap_follow` | Requested | later | One flag per feature, plus `imu_all` |
| B5 | Record every `gap_follow` run | Today's races left no recording | later | Plain `gap_follow_launch.py` runs leave a bag |
| B6 | Overtaking | It braked to 0 instead of following | needs a decision | See [open questions](#open-questions-for-the-team) |
| B7 | Car 1's course in the simulator | Tuning happens on tracks the car doesn't race on | after A1 | A clean map of the course becomes an `eval_gap.py` track |

**The single most race-winning step is not in `gap_follow` at all.** On a course the car has mapped, it can race a [racing line](../../glossary.md#racing-line) with [pure pursuit](../../glossary.md#pure-pursuit) (`race_launch.py`). That needs a saved map, which A1 and A3 unblock. `gap_follow` only reacts to what the LiDAR sees right now; it is still worth making faster, but it is the fallback, not the ceiling.

## What today's runs showed

All runs are in `~/.ros/racerbot_runs/` on car 1.

**Without the gyro** (`20261026-174337`, `20261026-180228`), [SLAM](../../glossary.md#slam) used the steering-based [odometry](../../glossary.md#odometry--odom):
- Its heading drifted 116° from the gyro in 81 s.
- SLAM had to correct 29, 52 and 37 times per lap.

**With the gyro** (`map_course gyro_odom:=true`; runs `-194514` at 3.5 m/s, `-194714` at 1.5 m/s, `-194913` at 2.5 m/s):
- The odometry problem was gone. SLAM stayed within 8-17° of the gyro, and corrections fell to 7-14 per lap.
- But every correction over 0.3 m happened during a **loop closure**: the moment SLAM recognises a place it has already mapped and pulls the map into line.
- The worst of them moved the car's pose 2.2-2.6 m onto a neighbouring stretch of track. That doubled the walls in the map. This is what the driver saw as "the scan matched onto a part of the course the car wasn't on".

**Every lap was rejected at about 29° of heading error, and that is not SLAM's fault.** The lap recorder (`LapRecorder` in `auto_map_race_node.py`) works like this:
- It closes a lap the first moment the car is within 1.5 m and 30° of where the lap started.
- The start point is in a curve. So the car reaches the edge of the 1.5 m circle while it still has about 30° left to turn.
- The next lap then starts at that early point, in the curve again.

A replay of the real `LapRecorder` on the recorded poses reproduces the logged numbers exactly. The gyro confirms the car really was pointing 25-36° away at those moments.

**`gap_follow`**, measured on run `-194514` (48 s, 97 m, `max_speed` 3.5):
- **Speed:** average 2.0 m/s.
- **What limited it:** the curve speed cap 54% of the time, the acceleration ramp 31%, the clearance cap 14%.
- **Weave:** steering swings more than 0.2 rad about once a second and sits at full lock 18-27% of the time.
- **Speed dips:** each swing pulls the curve cap down, which gives 22 dips of more than 0.7 m/s.
- **Corner setup:** active 71% of the time, and against the turn on 95% of those ticks.
- **Time left on the table:** a minimum-time model on the path the car actually drove gives 47.8 s real, 41.9 s with smooth speed, and 38.8 s at 2.5 m/s² sideways.

**The races against team A** (opponent awareness on) were not recorded. Only the 1 Hz decision logs exist, and they show:
- The other car was recognised on about 5% of seconds.
- When it was, the car braked to 0 instead of following.
- There were 10 emergency stops in the 19:59 race.

<details>
<summary>Where each number comes from (safe to skip)</summary>

- **SLAM jumps:** the `map → odom_imu` transform in each run's bag, measured at the car's position at each change. Loop closures were identified by the Ceres solver lines (`num_threads`) in each run's `map_course-*.log`.
- **Lap closures:** `LapRecorder` replayed on `/slam_pose` with the run's own parameters. The car's true heading change comes from integrating `/odom/imu`.
- **Node spacing:** SLAM updates divided by distance driven. 0.30 m at 1.5 m/s, so the 20-scan evidence chain a loop closure needs covers only 5.9 m of track (9.5 m at 3.3 m/s).
- **`gap_follow` numbers:** a separate session's analysis of the same bags, recorded in memory as `gap-follow-runs-2026-10-26`.

</details>

## Decisions already made

Made in chat on 2026-10-26. Changing one is a new decision, not a tweak.

- **IMU failure:** if the IMU goes bad mid-run (`/imu/status` not `ok`, or data stops), each IMU feature switches itself off. `gap_follow` then drives exactly as it does without that feature, with a loud warning in the log and the reason on `/drive_intent`.
- **Flags:**
  - One flag per IMU feature, plus an `imu_all` shortcut, with the same names in `map_course` and `gap_follow_launch.py`.
  - Every new flag defaults to off.
  - An explicit `false` overrides `imu_all`.
- **Testing IMU features:** a simulated IMU plus replays of real bags, then the usual car order. SLAM must also be logged well enough to analyse a run in depth.
- **Order of the later work:** observability, then `map_course` wiring, then the simulated IMU, then `gap_follow`'s IMU features.
- **Observability style:** everything is analysed after the run, so it costs the car no CPU while mapping.
- **`imu_lap_turns`** needs `gyro_odom:=true`. It is a parameter change only, with no new code in the driving node.
- **Added to the `map_course` work:** loop-closure settings that apply only with the gyro, and closing laps at the closest approach.
- **Today:** SLAM and `gap_follow` improvements, with `gap_follow` "faster, better, more race-winning".

## Part A: today

### A1. A SLAM profile for good odometry

**What.** A new file, `src/racerbot_launch/config/slam_gyro.yaml`, layered on top of `slam_tracking.yaml`, and **only** when `gyro_odom:=true`. Runs without the gyro keep today's settings, because steering odometry drifts about 4 m a lap and needs the wide search.

**Why.** The team suspected that settings loosened in August, for odometry with no gyro, now hurt SLAM with good odometry. Comparing what ran today with slam_toolbox's own defaults bears that out for some settings, and finds two loose settings that were never ours:

| Setting | slam_toolbox default | Ran today | Origin |
|---|---|---|---|
| `loop_search_maximum_distance` | 3.0 m | 8.0 m | August, car 2: "odometry on this car has no gyro" |
| `minimum_travel_distance` / `minimum_travel_heading` / `minimum_time_interval` | 0.5 / 0.5 / 0.5 | 0.25 / 0.25 / 0.2 | August, car 2: correct dead reckoning more often |
| `loop_match_minimum_chain_size` | 10 | 20 | August, car 2: keeps about 5 m of evidence at 0.25 m spacing |
| `loop_search_space_dimension` | 8.0 (±4 m) | 8.0 | slam_toolbox stock, never tuned |
| `loop_match_minimum_response_coarse` / `_fine` | 0.35 / 0.45 | 0.35 / 0.45 | slam_toolbox stock, never tuned |
| `throttle_scans`, `ceres_loss_function`, `transform_timeout` | 1, None, 0.2 | 2, HuberLoss, 1.0 | CPU and robustness. Keep |

**Two things set the window.** Car 1's course is a loop of about 29 m, and its neighbouring stretch of track is 2.2-2.6 m away. With the gyro, odometry is about 1 m off after a lap. A ±4 m match window lets a loop closure accept the wrong stretch of track; about ±1.5 m cannot reach it.

**Starting candidates,** to be settled by A2 replays:
- `loop_search_maximum_distance` 3.0 and `loop_search_space_dimension` 3.0.
- Node spacing back toward 0.5 m.
- A chain covering at least 10 m of track.
- Optionally, stricter match responses.

**Done when** A2 replays of all three gyro runs show:
- no loop-closure correction larger than 1.5 m at the car;
- at least one loop closure per lap, which proves closures weren't simply switched off;
- single walls in the final map;
- fewer scan drops than the live run.

The pass/fail check never uses SLAM's own confidence numbers.

#### Results (2026-10-27)

Offline replays of the three gyro runs (`tools/run_analysis/slam_replay.py`, `--odom odom_imu`). Each cell is the largest correction at the car, the jumps over 1.5 m, and the loop closures (Ceres solves, the proxy `slam_metrics` uses):

| Run (speed) | Baseline (`slam_tracking.yaml` only) | P1 | P2 | P3 |
|---|---|---|---|---|
| `20261026-194514` (3.5 m/s) | 0.59 m / 0 / 6 | 0.71 m / 0 / 3 | 0.16 m / 0 / 0 | 0.17 m / 0 / 0 |
| `20261026-194714` (1.5 m/s) | 2.45 m / 2 / 4 | 0.47 m / 0 / 12 | 0.11 m / 0 / 0 | 0.11 m / 0 / 0 |
| `20261026-194913` (2.5 m/s) | 2.75 m / 4 / 9 | 0.47 m / 0 / 6 | 0.14 m / 0 / 0 | 0.13 m / 0 / 0 |

**Chosen: P1.** `loop_search_maximum_distance` and `loop_search_space_dimension` both at 3.0 m, in `src/racerbot_launch/config/slam_gyro.yaml`, loaded only with `gyro_odom:=true`. P2 adds 0.5 m node spacing, and P3 adds stricter loop-match responses on top of P2. Both switch loop closure off (no solves on any run), so neither is used.

### A2. Offline SLAM replay (minimal)

**What.** A tool that reruns a recorded run through slam_toolbox with the car parked. It is the first slice of B1's `slam_replay`.
- **What it plays:** `/scan`, `/tf_static` and the chosen odometry topic from the bag, on the bag's clock.
- **The odometry transform:** a small relay turns `/odom` or `/odom/imu` into the single `odom → base_link` or `odom_imu → base_link` [transform](../../glossary.md#tf--transform--frame).
- **What it never plays:** the recorded `/tf`, because it contains the live SLAM's own `map → odom` correction.
- **Settings:** slam_toolbox runs with the run's own parameter snapshot (`metadata/params/slam_toolbox.yaml`), plus any profile under test.
- **Output:** a new run folder with a small bag of SLAM's outputs, which the analysis scripts read.

**Safety.**
- **Isolated:** it forces its own `ROS_DOMAIN_ID` and localhost-only discovery in every child process.
- **Interlocked:** it refuses to start while `vesc_driver_node`, `urg_node` or `joy` run on the machine.
- **Never drives:** it publishes no drive topic.
- **Why the interlock:** a replay publishes a second `/scan`. On the car's own ROS graph, `gap_follow` would steer by it.

**Tests.**
- **Without ROS:** the environment builder, the interlock and the parameter merge are plain Python. The interlock must refuse when a fake driver process is present, and the child environment must always carry the isolated domain.
- **End to end:** replay run `-194913` with the unchanged profile. It must reproduce the live run's large jumps.

### A3. Laps close at the closest approach

**What.** In `LapRecorder`:
- When the car enters the closure circle with the other conditions met, the lap is not closed yet.
- The recorder follows the distance to the start and closes the lap at its **minimum**, the point where the car is nearest its start.
- The heading check is made at that point.
- The next lap therefore starts where the previous one started.

**Why.** Today's 29° failures (see [what today's runs showed](#what-todays-runs-showed)).

**Safety.** `auto_map_race_node` can drive the car, but this changes only when a lap is declared closed, not what is commanded. Its [deadman](../../glossary.md#deadman) tests run unchanged and must pass.

**Tests:**
- Synthetic laps with a known closest point: on a circle whose start lies on the arc, the heading error at the closest approach is close to 0 by geometry.
- The old behaviour, closing at first entry, must fail that test.
- Today's recorded `/slam_pose` must produce laps that pass the 15° check.

### A4. `max_lateral_accel` 2.0 → 2.5

A one-line change in `gap_follow.yaml`. Every test and doc that quotes 2.0 changes with it.

The evidence is the grip measurement on run `-194514`: the car achieved 99-107% of its commanded turn up to 2.5 m/s², and 93% between 2.5 and 3.0.

### A5. The steering weave: cause first, then a fix behind a flag

**Step 1, the cause.** From the bags, decide between two explanations:
- **The steering loop oscillates:** the car's heading, measured by the gyro on `/imu/data`, swings at the same 0.6 Hz as the steering, trailing it in phase.
- **The aim point jumps:** the chosen gap's bearing jumps while the car's heading stays smooth.

The other session ruled out scan noise (0.7° per scan, median) and corner setup.

**Step 2, the fix,** least invasive first:
1. **If it's a loop oscillation:** first the existing gain schedule (`steering_gain`, `steering_gain_speed_ref`, `max_steering_rate`), which changes no code.
2. **If that isn't enough:** damping from the gyro's measured yaw rate. A plain low-pass filter adds lag and can make a loop oscillate *more*, so it is not the fix for this case. With no IMU it falls back to today's behaviour.
3. **If the aim point jumps:** smoothing or hysteresis on the target.

**Step 3, the curve cap.** Only after the weave is fixed, measure whether the curve cap still dips.

If it does, add an option to base it on the steering the rack actually commands instead of the momentary request. That option stays off until measured. Braking early for a sharp request is deliberate today: see the comment above `curve_speed` in `gap_follow_node.py`.

**Every code change also lands in the simulator's copy** (`tools/f1tenth_sim/gap_mirror`, held in step by `test_gap_mirror_parity.py`).

**Done when:**
- `eval_gap.py` on the indoor tracks shows fewer steering reversals, no new wall contacts, no more stops, and equal or better lap times;
- replaying today's bags shows fewer big swings;
- the four deadman deny-path tests still pass.

#### Results (2026-10-27)

**The cause is the steering loop.** `tools/run_analysis/weave.py` was run on both gyro runs. The weave band was fixed at 0.45-1.5 Hz before the verdict, from the earlier analysis's 0.6 Hz peak, to keep out the slower rhythm of the corners.

| | Run `20261026-194514` | Run `20261026-194913` |
|---|---|---|
| Weave frequency | 0.625 Hz | 0.625 Hz |
| Steering-to-gyro coherence there | 0.97 | 0.95 |
| Measured steering lag τ | 0.150 s | 0.125 s |
| Loop model: damping ζ, frequency | 0.60, 0.71 Hz | 0.65, 0.74 Hz |
| Swings that start at an aim-point jump | 4% | 4% |

Searched over 0.2-5 Hz, the strongest steering rhythm is 0.31 Hz: the corners passing on a 31 m lap at about 2 m/s. On that band the verdict comes out "unclear". It's reported here so the band choice stays visible.

**The simulator reproduces the weave.** `eval_gap.py` counts 0.43-0.95 steering reversals a second on the indoor tracks and 1.34 on car 1's course (`car1_course_2026_10_26`). The car does about one a second.

**Gain schedule (Task 8): no change shipped.** Over 6 tracks, both directions and 2 seeds, no schedule met all four acceptance tests:

| `steering_gain`, `steering_gain_speed_ref` | Mean lap | Reversals/s | Car 1 course: lap, reversals/s |
|---|---|---|---|
| 1.0, 1.8 (today) | 15.06 s | 0.859 | 13.99 s, 1.14 |
| 1.0, 1.2 | +1.7% | −22% | 14.09 s, 0.92 |
| 0.8, 1.5 | +1.8% | −20% | 14.30 s, 1.02 |
| 1.0, 1.0 | +5.0% | −27% | 14.01 s, 0.75 |
| 0.8, 1.2 | +5.5% | −27% | **13.91 s, 0.76** |
| 0.6, 1.8 | +4.5% | −21% | 14.24 s, 1.02 |

Every run had 0 contacts and the same stops. Lowering the gain buys at most 27% less weave and costs lap time, as the loop model predicts, so the fix is gyro damping (Task 9).

**Worth trying on the floor:** 0.8 / 1.2 is the one exception. On car 1's own course it weaves 34% less and laps slightly faster. Both parameters are live-tunable.

### A6. Corner setup on vs. off

Compare `enable_corner_setup` true against false:
- with `eval_gap.py` across the indoor tracks;
- by replaying today's bags with `analyze_run.py --set`.

Report lap time, wall contacts, stops and which cap bound. Corner setup was designed on 2026-10-19 to stop the car hugging the inside wall in simulation ([gap-follow.md](../../gap-follow.md#corner-setup)), so turning it off could bring that back. The default stays until the team decides.

#### Results (2026-10-27)

This was run in `eval_gap.py` over the indoor tracks and car 1's course, both directions, 2 seeds, 24 runs per setting, with the shipped YAML (`max_lateral_accel` 2.5).

| Corner setup | Mean lap | Car 1 course lap | Reversals/s | Contacts | Stops |
|---|---|---|---|---|---|
| on (shipped) | 15.06 s | 13.99 s | 0.86 | 0 | 48 |
| off | 14.84 s (−1.5%) | 13.62 s (−2.7%) | 1.02 (+18%) | 0 | 48 |

**Off is slightly faster but weaves more.** Corner setup pulls against the turn, as the floor data showed, and that slows the car. It also damps the steering a little.

**The decision is better made after Task 9.** Gyro damping should cover the extra weave either way, so the off-vs-on choice is clearer once it lands.

**Replays of the real recordings add nothing here.** `analyze_run.py --set enable_corner_setup=false` on run `20261026-194514` reports 0 zero-speed ticks with and without it. That's all an open-loop replay of the same scans can show, so the simulator table above is the evidence.

### A7. Race pace: measure every lever

From today's bags and the existing minimum-time model, cost each lever in seconds per lap:
- **Weave:** about 6 s of the 48 s gap to smooth speed. This is A5.
- **Grip:** about 3 s more at 2.5 m/s². This is A4.
- **The acceleration ramp** (`max_acceleration` 3.0, binding 31% of the time): first check from `/odom` that the car actually reaches 3.0 m/s². Raise the ramp only if it can.
- **Corner setup's line:** A6.

Anything not covered by A4-A6 gets built later, each with its own simulator evidence.

### A8. Car test order (the team does this)

1. **Static check.** Nothing driving. Launch with the new flags and confirm the parameter snapshot shows them.
2. **Wheels off the ground.** Full stack, LB held. Steering and speed respond, and stop when LB is released.
3. **Floor.** Low speed in open space, then `map_course gyro_odom:=true` on the course.

Each test needs the previous step to pass. Never skip ahead: the simulator does not sign off a physical car.

### Shipped 2026-10-27: the race defaults

Decided 2026-10-27: the goal is a race-winning `gap_follow`, not a conservative one. The shipped defaults now carry the damping from A5 (on, gain 0.10 s), corner setup from A6 switched off, and a 5.0 m/s top speed with the speeds tied to it scaled by 5.0/3.5. A4 (`max_lateral_accel` 2.5) is unchanged, and adaptive grip still ships off.

A7 said to raise the ramp only if the car reaches 3.0 m/s². It does not (measured 2026-10-27), so the 5.0 ramp was shipped against that rule. On the car it mostly removes a software limit, because the lag is in the VESC speed loop, which is a team note.

The evidence, the numbers and how to go back are in [gap-follow.md](../../gap-follow.md#race-defaults-2026-10-27). Nothing here has run on the car yet, so the car order in A8 applies.

## Part B: later

Each of these needs its own written plan before it is built.

### B1. SLAM observability

**What it adds:**
- **CPU logging:** `record_run.py` also logs `tegrastats` (per-core load, clocks, temperatures, power; runs without sudo) and 1 Hz CPU per ROS node from `/proc`.
- **`slam_report.py`** (`tools/run_analysis/`, cached like `imu_report.py`), which reports:
  - which odometry SLAM used;
  - heading drift of each source;
  - every correction, sized at the car;
  - per-lap closure numbers and quality verdicts;
  - loop closures;
  - map consistency: the share of scan points that land on a wall of the final map;
  - scan drops against CPU;
  - wheel distance against scan-matched distance.
- **Compare mode:** `--compare` sets two runs side by side.
- **The full `slam_replay`.**
- **Automatic analysis:** the `map_course` wrapper runs the reports after the run; `analysis:=false` skips them.

### B2. IMU flags in `map_course`

- **`imu_lap_turns:=true`:** the lap counter counts turning from the gyro (`odom_frame: odom_imu`). It needs `gyro_odom:=true`.
- **`imu_all:=true`:** turns on every IMU flag.
- **Refusals:** any of them with `imu:=false` refuses to start with a clear message.
- **Known limitation:** if `vehicle_state_node` dies mid-run with `gyro_odom` on, SLAM loses its odometry and that run's map is lost. The car stays safe, because `gap_follow` steers from `/scan`.

### B3. Simulated IMU

- **What it publishes:** `gym_bridge_node` publishes `/imu/bno/data`, `/imu/vesc/data`, `/imu/data` and `/imu/status` from the gym's true yaw rate and accelerations.
  - Noise comes from a stationary stretch of a real bag.
  - The real `vehicle_state_node` runs on top (`vehicle_state_launch.py hardware:=false`).
- **Fault injection:** `imu_fault` can be set mid-run to `none`, `stale`, `status_fault`, `bias`, `scale` or `zero`.
- **Safety:** `imu_bridge_node` joins the simulator's interlock list.
- **Reproducing today's failure:** setting `odom_yaw_rate_scale` about 1.1 reproduces it.
- **Limit:** the gym's tyres never lose grip. Their force is linear in slip angle (`single_track.py`), so grip features can be checked for wiring and fall-back there, but not for their thresholds.

### B4. IMU features in `gap_follow`

One flag each, all off by default, each falling back as in [decisions](#decisions-already-made):
- **Fused speed:** `/odom/fused` for TTC braking and the acceleration ramp, now also in `gap_follow_launch.py`.
- **Corner grip cap:** slow when measured sideways acceleration nears a limit.
- **Understeer check:** slow when `/steering/measured` shows much less turn than commanded.
- **Steering damping:** gyro yaw-rate feedback. It may land earlier through A5.
- **Traction control:** hold acceleration when the wheels spin faster than the car moves.
- **Impact stop:** an accelerometer spike stops the car until LB is released and pressed again.

The design of each needs approval before building. Every feature keeps the four deadman deny-path tests.

### B5. Record every `gap_follow` run

Today's races left only 1 Hz decision logs. `gap_follow_launch.py` (or bringup) should start the run recorder the way `map_course` does, so every race can be analysed.

### B6. Overtaking

The [open questions](#open-questions-for-the-team) decide this. The options are:
- recognise the other car sooner (today it needs 8 detections in a row);
- follow at the other car's speed instead of braking to 0;
- choose how much side margin to give up for a pass.

### B7. Car 1's course in the simulator

Once A1 produces a clean map of the course, add it as an `eval_gap.py` track, so `gap_follow` is tuned on the course it races on.

## How every change is proven

These rules are from `CLAUDE.md` and `TEST_QUALITY_STANDARDS.md`.

**The deadman.**
- Every node that can move the car keeps its LB deadman.
- Its four deny-path tests (released, never received, stale, and held) must still pass on the published command.

**Tests that can fail.**
- Each new test is first run against deliberately broken code and must fail.
- A failing test is fixed in the code, never loosened.
- Tests are run sourced and in their own ROS domain:

```bash
# Terminal: any, with nothing driving. Success: the collected count matches
# `grep -c '^def test'` over the files, and everything passes.
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest src/gap_follow/test/ src/pure_pursuit/test/ src/racerbot_launch/test/ -q
```

**Evidence before the car.**
- Replays of real bags show what a change would have done on the scans the car actually saw.
- `eval_gap.py` shows the closed-loop result.
- Neither one signs off the car: [A8](#a8-car-test-order-the-team-does-this) does.

## Open questions for the team

1. **Overtaking (B6):** how much side margin may the car give up to pass? The track is 1.5-1.8 m wide and a safe pass needs about 2.4 m, so a pass here means accepting less than today's margin, or never passing.
2. **Defaults after the A/B runs:** should `gyro_odom` become the default for `map_course`, and should corner setup stay on?
3. **The 1.5 m/s observation:** slower mapping made SLAM worse, through denser nodes and shorter evidence chains. After A1 this should no longer hold; re-test before settling on a mapping speed.
