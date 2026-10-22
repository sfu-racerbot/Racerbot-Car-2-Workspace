# Follow-the-gap: swept clearance, arc guard and corner setup

> **Who this is for:** anyone about to run `gap_follow` (the reactive, no-map autonomy) after 2026-10-19, or tune it.
> **Read first:** [src/gap_follow/README.md](../src/gap_follow/README.md) — what follow-the-gap is and how the basic pipeline picks a gap.
> **You'll be able to:** say what the three newer features do and what changed after the first real run, turn any of them off, check them in the simulator, and bring them up on the car in the safe order with the recorder on.
> **Time:** 20 minutes to read; the simulator checks take a few minutes each.

In October 2026 `gap_follow` got three additions that change how it drives through corridors. In simulation the car went from hitting walls in 24 of 60 indoor runs to 0 of 60, and lapped about 20% faster.

They were first driven on the car on 2026-10-19. That run had no contact, but the car hard-stopped seven times. The [2026-10-19 run](#the-2026-10-19-run) section explains why, and what was changed on 2026-10-20 to stop it. **Those 2026-10-20 changes have only run in the simulator and in a replay of that run's own scans.**

## Contents

1. [Highlights](#highlights)
2. [What changed, in plain words](#what-changed-in-plain-words)
3. [Safety](#safety)
4. [Validation](#validation) — the order to bring it up on the car
5. [Turning a feature off](#turning-a-feature-off)
6. [The 2026-10-19 run](#the-2026-10-19-run) — why the car braked, and what changed
7. [Checking it in the simulator](#checking-it-in-the-simulator)
8. [Swept-path clearance](#swept-path-clearance)
9. [Arc guard](#arc-guard)
10. [Corner setup](#corner-setup)
11. [Measured results](#measured-results)
12. [Ideas that were tried and dropped](#ideas-that-were-tried-and-dropped)
13. [Keeping the mirror honest](#keeping-the-mirror-honest)

## Highlights

- **Every simulated indoor lap completes with no wall contact.** That is 5 layouts × 2 directions × 6 seeds. Before the change, `indoor_tight` and `asb_sharp` touched a wall on every run.
- **Faster where the old ceiling was wrong.** The `asb_10000` hallway lap dropped from 24.6 s to 18.9 s. The old speed limit read the hallway's *side* walls as if they were in front of the car.
- **More room, not less.** The closest the body came to anything went up on every layout. On `asb_10000` it went from 0.07 m to 0.17 m.
- **Each feature has its own off switch** in `config/gap_follow.yaml`, and the simulator measures what each one contributes.
- **Bounded authority.** Arc guard and corner setup can each move the steering by at most half of `max_steering_angle`. The node refuses to start if they are configured for more.
- **Cheap.** About 1.4 ms per LiDAR scan ([glossary](glossary.md#scan)) on average on the Jetson, and about 2.2 ms when the arc guard searches.
- **Tested against the real node.** A test drives the real ROS node (a running program on the ROS graph — [glossary](glossary.md#node)) and the simulator's copy through the same scans, and requires identical commands to 1e-6.

- **No more false hard stops in corridors (2026-10-20).** On the 2026-10-19 run, all 97 stop ticks were the TTC brake reacting to the inside wall of a bend 0.12–0.18 m *beside* the car. Replayed through the final config, the same scans produce none.
- **Safe with other cars on track, when you turn it on (2026-10-22).** With `opponent_awareness:=true`, two simulated cars pass 40 of 40 runs and three cars 24 of 24, with no contact at all. Without it: 34 of 40 and 15 of 24. It follows a slower car rather than overtaking it on every track tried. See [Several cars at once](#several-cars-at-once).
- **More of every track completes (2026-10-20).** Across 11 simulated tracks, 6 seeds each, both directions, 108 of 132 runs pass, against 96 for the 2026-10-19 config. The hall loop `levine_blocked` laps in 23.6 s instead of 34.7 s.

**Limits:** these numbers come from the simulator only. `max_lateral_accel` was raised from 1.0 to 2.0 m/s² for this work, and that value is simulator-only until it has been run on the floor.

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

The simulator's car is close to this one, but not identical. In particular, its grip and the 2.0 m/s² cornering limit have not been checked on this floor.

## Validation

**Do these in order and do not skip ahead.** This is the workspace's standard order for any changed driving node ([writing-your-own-node.md](writing-your-own-node.md#testing-before-its-on-wheels)).

1. **Simulator.** Run the [checks below](#checking-it-in-the-simulator) and confirm every indoor run passes.
2. **Wheels off the ground.** Put the car on a stand, start the full stack, hold LB, and move your hand or a box around in front of the LiDAR.
   - **Working when:** the wheels turn away from your hand.
   - The node's decision log (one line a second in the `gap_follow_launch.py` terminal) mentions `swept clearance`, `corner setup` or `arc guard` when they act.
3. **Floor, low speed, open space, with the recorder on.** Lower the speed first: set `max_speed` to about 1.0 m/s and `max_lateral_accel` back to 1.0. Both can be changed live from the dashboard's tuning panel. Record every floor run (commands below), so the next change can be checked against what the car really did.
   - Drive a straight corridor, then a single corner, with someone ready to let go of LB.
4. **Raise speed in steps.** Raise `max_lateral_accel` toward 2.0 only after the low-speed runs look right.
   - **If the car slides or runs wide in corners:** the floor has less grip than the simulator. Keep `max_lateral_accel` at the highest value that stayed planted.

If anything looks wrong, turn the suspect feature off (next section) and compare. That is what the separate switches are for.

### Recording a floor run

Four terminals, started in this order. Full detail is in [run-diagnostics.md](run-diagnostics.md#tldr).

**Terminal 1 — the recorder. Start it first, so nothing is missed:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch race_diagnostics record_run.py
```

**Working when:** it prints the run directory it created, `~/.ros/racerbot_runs/<ts>/`.

**Terminal 2 — hardware, plus the mux ([glossary](glossary.md#mux--multiplexer)) that picks which control layer's commands reach the motor:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** the VESC (the motor controller, [glossary](glossary.md#vesc)) and the LiDAR come up. This never moves the car by itself.

**Terminal 3 — `gap_follow`, logged into the run directory:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
ros2 launch gap_follow gap_follow_launch.py 2>&1 | tee ~/.ros/racerbot_runs/<ts>/launch.log
```

**Working when:** it prints a decision line once a second. The car only moves while you hold LB. Do **not** also run `teleop_launch.py`: its `/teleop` always wins the mux, so `gap_follow`'s commands would never reach the car ([architecture.md](architecture.md)).

**Afterwards, any terminal** — the report from [run-analysis.md](run-analysis.md):

```bash
python3 ~/racerbot-ws/tools/run_analysis/analyze_run.py ~/.ros/racerbot_runs/<ts>
```

**Working when:** it writes `~/.ros/racerbot_runs/<ts>/analysis/report.md`, explaining every stop.

## Turning a feature off

Each feature has a switch in `src/gap_follow/config/gap_follow.yaml`:

| Switch | Default | Off means |
|---|---|---|
| `enable_swept_clearance` | `true` | Speed is limited by the closest reading in the ±30° forward cone again, as before. |
| `enable_arc_guard` | `true` | The car steers exactly where the gap logic says. |
| `enable_corner_setup` | `true` | No move to the outside before a turn. Corridor centering works as before. |
| `narrow_corridor_speed` | `2.5` | Set it to `max_speed` or above to turn the narrow-corridor slowdown off. |
| `ttc_path_margin` | `0.05` | Set to `0.18` (the old value, `safety_margin`) for the TTC corridor as it was on 2026-10-19. Never above `safety_margin`; the node refuses to start. |
| `arc_guard_blocked_fallback` | `request` | `freest` brings back the arc guard's old all-blocked behaviour. |

These switches are **not** live-tunable: edit the YAML and restart `gap_follow_launch.py`. The workspace is built with `--symlink-install`, so no rebuild is needed.

`max_lateral_accel` (default 2.0, was 1.0) is live-tunable from the dashboard.

## The 2026-10-19 run

The first floor run of this rework, recorded as `~/.ros/racerbot_runs/20261019-200703` and analysed with the tools in [run-analysis.md](run-analysis.md).

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
| `max_lateral_accel` | 2.0 m/s² | 2.0 m/s² (unchanged) | Measured grip on that floor: the car achieved 99% of the commanded turn up to 2.0 m/s², 91% at 2.0–2.5, 74% above 3.0. Above 2.0 it starts to slide. |

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

All cars run `gap_follow` on their own simulated LiDAR, which sees the other cars as obstacles ([run-analysis.md](run-analysis.md#testing-several-cars-at-once)). The leader is held to 1.8 m/s and the others run at 3.5 m/s, so they catch it. 4 seeds per track, both directions, 2 laps each. A run passes only if no car touches anything and every car finishes.

| Config | 2 cars (5 tracks) | 3 cars (3 tracks) |
|---|---|---|
| 2026-10-19 config | 23/40 | 4/24 |
| 2026-10-20 config (shipped default) | 34/40 | 15/24 |
| **2026-10-22, `opponent_awareness:=true`** | **40/40, no contact** | **24/24, no contact** |

Without the layer, every remaining failure is one pattern. A fast car starts to pass a slower one with 0.15–0.2 m between them, because its own path is clear. The slower car then turns into a bend across that path, and at 3.3 m/s the fast car can't stop in the metre left. Follow-the-gap treats every LiDAR return as a fixed wall, so it doesn't know the obstacle beside it can move.

#### Opponent awareness: what it does

It's a switch, off by default. **With it off nothing below runs**, and the node's commands are exactly what they were before it existed.

Two checks back that up. With the layer off, the benchmark above reproduces the 2026-10-20 numbers exactly. And a node test runs the same scans with the layer on and off, in a world where nothing moves, and requires identical commands.

Leaving it on with only one car costs nothing measurable either. In the single-car sweep (11 tracks, 6 seeds, both directions) it passes 111 of 132 runs, against 108 of 132 with it off, and every track does at least as well.

**Terminal 2**, instead of the usual `gap_follow` launch, on **every** car on the track:

```bash
ros2 launch gap_follow gap_follow_launch.py opponent_awareness:=true
```

**Working when:** with another car moving ahead, the decision lines (one a second) include `opponent mode=follow` and `follow cap=...`.

With it on, every scan:

1. **Finds car-shaped things.** It splits the scan into clusters and keeps those 0.15–0.7 m across that stand free of anything behind them. A stub of wall is not free-standing, because the beam just past its end doesn't jump further away.
2. **Tracks them.** It matches clusters from scan to scan against this car's own motion, dead-reckoned from speed and steering (modelled as the servo actually follows the command, 75 ms late), and filters each into a position and a velocity. Something counts as **moving** only after 8 detections in a row at 0.6 m/s or more over the ground. A parked car is still just an obstacle, handled as before.
3. **Follows.** Behind a moving car, speed is capped at `(gap − 0.5 m) / 1.0 s`, where gap is from this car's bumper to the other car. At 1.8 m/s that allows a 2.3 m gap. In the benchmark the closest two car centres came was 0.82 m on the 1.2 m loop, and over 2.1 m on every other track.
4. **Passes only with room, on a straight.** The gap search sees where each moving car will be over the next 0.8 s, widened by 0.15 m, so it can only choose a gap with real room beside the other car. That choice is used only to pick a side of the opening the car would take anyway. It's refused while this car steers more than 0.10 rad, or while the car ahead is turning more than 15°.

The time-to-collision brake, the contact stop, the swept-path speed limit and the arc guard all still read the **real** scan. The layer can lower the speed and pick a side of an opening. It can't switch any safety check off.

**What it doesn't do: overtake on our tracks.** On every simulated track, including the wide gym race tracks, the fast car followed the leader for the whole run.

A pass needs about 2.4 m of width with these margins, and these tracks are narrower than that where the leader drives. The real track is 1.0–1.3 m wide, where two 0.30 m cars can't pass each other safely anyway. Expect following, not racing.

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

The servo model, the 0.6 m/s "moving" threshold and the consecutive-detection rule are what absorb that. A 1.0 m/s threshold was also tried: it did no better with several cars and worse with one (108 of 132). On the car, a wall that the layer takes for a moving car would show up as `opponent mode=` in a run where no other car is moving. If you see that, turn the layer off and report the run.

Code: `src/gap_follow/gap_follow/opponent_tracking.py` (no ROS; 71 unit tests), wired in `gap_follow_node.py` and mirrored in `tools/f1tenth_sim/gap_mirror.py`. Every `opponent_*` parameter is commented in `config/gap_follow.yaml`.

</details>

**Simulator-validated only.** Bring it up in the [usual order](#validation): two cars wheels-off first, then two cars at low speed (`max_speed` about 1.0) with a person holding LB on each controller.

**Checking it, any terminal** (several minutes on the Jetson):

```bash
python3 tools/f1tenth_sim/eval_gap_multi.py --track indoor_oval,indoor_wide,asb_10000,levine_blocked,narrow_1p2 \
    --cars 2 --speeds 1.8,3.5 --seeds 4 --set enable_opponent_awareness=true
```

**Working when:** every line reads `pass 4/4  wall 0  car-car 0`.

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
- **`max_lateral_accel` 2.0** is worth about 20% of lap time, and is the one value whose real-floor grip is unchecked.

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

**Working when:** `3 passed`. A failure names the tick and whether steering or speed differed.

## See also

- [src/gap_follow/README.md](../src/gap_follow/README.md) — the whole follow-the-gap pipeline and every parameter.
- [simulator.md](simulator.md) — the no-ROS simulator these numbers come from.
- [gap-follow-improvement-plan.md](gap-follow-improvement-plan.md) — the original diagnosis that led here.
- [steering-laws.md](steering-laws.md) — the other October 2026 change, to the map-based `pure_pursuit` controller.
