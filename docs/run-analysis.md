# Analysing a recorded run, and feeding it back into the simulator

> **Who this is for:** anyone who has just driven the car with the recorder on and wants to know what happened, tick by tick. Also anyone adding a new map to the simulator, or re-checking that the simulated car still behaves like the real one.
> **Read first:** [run-diagnostics.md](run-diagnostics.md) — how to *record* a run. This doc starts where that one ends, with a run directory on disk.
> **You'll be able to:** turn a run directory into a report explaining every stop, measure how the real car responded to its commands, replay the real LiDAR scans through the simulator's copy of `gap_follow`, fit the simulator's car model to the run, and turn a SLAM map into a simulator track.
> **Time:** about 2 minutes per run for the report; 10–20 minutes for a simulator fit.

The car can record everything it saw and did. These tools turn that recording into answers: why it braked, how fast it really went, and whether the simulator would have done the same.

## Contents

1. [Highlights](#highlights)
2. [What gets logged](#what-gets-logged)
3. [The one command: analyse a run](#the-one-command-analyse-a-run)
4. [Reading the report](#reading-the-report)
5. [Asking "what if": counterfactual replay](#asking-what-if-counterfactual-replay)
6. [Fitting the simulator to the real car](#fitting-the-simulator-to-the-real-car)
7. [Adding a map to the simulator](#adding-a-map-to-the-simulator)
8. [Testing several cars at once](#testing-several-cars-at-once)
9. [How the pieces work](#how-the-pieces-work)
10. [Troubleshooting](#troubleshooting)

## Highlights

- **One command explains a whole run.** `analyze_run.py` reads the bag and writes a report plus plots. On run `20261019-200703` it takes 7 seconds once the bag is cached. Every one of the 97 stop ticks is attributed to the layer that caused it.
- **It knows how the car really moved.** Odometry ([glossary](glossary.md#odometry--odom)) can't say that: its speed comes from the wheels, and its turning rate is *computed from the steering command*. The tools match each LiDAR scan ([glossary](glossary.md#scan)) against the one before it instead. On that run the median match error was 6 mm.
- **The replay is checked against the real car.** Feeding the run's scans to the simulator's copy of `gap_follow` reproduced 96 of the 97 recorded stop ticks exactly, with 0.0012 rad RMS steering error. So when it explains a stop, it is explaining the code that actually ran.
- **The simulator is fitted to real data.** `fit_sim.py` replays the recorded commands into the simulated car and searches for the settings that move it like the real one. On run `20261019-200703` that cut the speed error from 0.46 to 0.24 m/s and the turning-rate error from 0.25 to 0.15 rad/s, against the stock simulator.
- **Any occupancy map becomes a track.** `track_import.py` takes a SLAM map (one the car built while driving, [glossary](glossary.md#slam)) or another team's map, finds the loop's centreline itself, and registers it. Every simulator tool then accepts it as `--track <name>`.
- **Several cars at once.** `eval_gap_multi.py` runs two or more `gap_follow` cars on one track. Car-to-car contact is checked against the real chassis, never gym's own collision flag.

**Limits:** the replay is *open-loop*. It uses the scans the real car saw, so it can say which stops a change would remove, not where the changed car would then have driven. That question needs the closed-loop simulator. Scan matching under-reads speed in long featureless corridors (see [How the pieces work](#how-the-pieces-work)).

## What gets logged

[run-diagnostics.md](run-diagnostics.md#tldr) has the recording commands. A recorded run is one directory under `~/.ros/racerbot_runs/`, named for when it started:

| File | What's in it | Read by |
|---|---|---|
| `launch.log` | Everything the driving node printed: one decision line a second, plus every state change (`DRIVE [...]` / `STOP [...]`) | you; `summarize_run` |
| `events.jsonl` | The probe's machine-readable events: first messages, LB pressed/released, periodic status lines | `summarize_run`; `analyze_run.py` counts them |
| `bag/` | A rosbag (a recording of ROS messages, replayable later): every message on `/scan`, `/odom`, `/drive`, `/joy`, `/ackermann_cmd`, `/tf` and `/parameter_events`, with timestamps | `analyze_run.py` |
| `metadata/` | What the run was made with: git state and uncommitted diff of `racerbot-ws` and `imu-calibration`, copies of the IMU calibration files, every node's parameters (see [run-diagnostics.md](run-diagnostics.md#what-gets-captured-and-why-each-piece-earns-its-place)) | you, weeks later |
| `analysis/` | Written by the tools below: `run.npz` (the bag as arrays), motion caches, `report.md`, plots | you |

The 1 Hz lines in `launch.log` are a summary. Between them the node made about 39 more decisions, and only the bag has them. Only the bag also has the scans, so only it can answer "what did the car see when it braked".

## The one command: analyse a run

**Terminal 1** — any terminal on the car or a laptop with the workspace. The first run reads the bag, which needs ROS sourced:

```bash
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash
cd ~/racerbot-ws
python3 tools/run_analysis/analyze_run.py ~/.ros/racerbot_runs/20261019-200703
```

**Working when:** it prints a report starting `# Run analysis: 20261019-200703`, with one `## LB hold` section per time LB was held. The same text is saved as `~/.ros/racerbot_runs/<run>/analysis/report.md`, beside a `hold1.png` plot.

**If it doesn't:** `ModuleNotFoundError: rosbag2_py` means ROS isn't sourced in that terminal. That only matters the first time. After that, `analysis/run.npz` exists and the tool runs without ROS.

The first run of a long recording takes a minute or two, almost all of it scan matching. The result is cached, so later runs take seconds.

## How the car really turned: gyros vs odometry vs LiDAR

For a run recorded with the IMU side up (any `map_course` run since 2026-10-24), one more command compares every measure of turn rate the car has. These are `/odom`'s (computed from the steering *command*), the BNO085's and the VESC's gyros, the gyro odometry `/odom/imu`, and LiDAR scan matching.

**Terminal 1**, ROS sourced as above for the first run:

```bash
python3 tools/run_analysis/imu_report.py ~/.ros/racerbot_runs/<run>
```

**Working when:** it prints three sections, also saved as `analysis/imu_report.txt`:
- **Turn rate vs the bno gyro:** for each other source, the RMS and mean difference and the gain, while the car was moving. The `odom (steering model)` row is how wrong `/odom`'s turning is.
- **Steering response:** steering lag and turning gain, the same fit as above, measured once by the gyro and once by scan matching. The two should agree; the gyro's is the cleaner.
- **Heading correction SLAM applied** on top of `/odom` and on top of `/odom/imu`, over the run and per 100 m. Odometry that needs less correction is a better starting point for SLAM.

**If it says `not enough driving`:** the car was parked or below 0.5 m/s. That's expected for a stationary test. The whole report, scan matching included, took 13 s for a one-minute recording on the car (2026-10-24); `--no-icp` skips the scan matching.

## Reading the report

Each LB hold gets four parts.

**1. What the car did.** How long LB was held, distance, mean and peak speed, and how many ticks the node commanded zero speed.

**2. How the car responded.** Every number here compares the *command* with what the car *did*:

| Line | Meaning | On run 20261019-200703 |
|---|---|---|
| speed lag | How long after a speed command the car's speed follows it | 0.275 s |
| steering lag, curvature gain | How long the steering takes to act, and what fraction of the commanded turn the car actually makes | 0.15 s, 0.95 |
| grip | Fraction of the commanded turn achieved, grouped by how hard the turn was (lateral acceleration). Below ~0.9 the tyres are sliding | 0.99 up to 2.0 m/s², 0.91 at 2–2.5, 0.74 above 3 |
| odom speed scale | True speed ÷ what odometry reported | 0.95 on straights, 0.89 in tight turns |

**3. Replay.** How closely the simulator's copy of `gap_follow` reproduces what the car published. Expect a steering error of about 0.001 rad and a near-identical stop count. If it is far off, the config or code that ran is not the one in the tree: pass the run's config with `--yaml` (next section), or check out the commit the car ran.

**4. Stops.** One line per group of stop ticks, naming the layer that stopped the car. For the time-to-collision brake (TTC: the brake that fires when, at the current speed, the car would reach an obstacle within 0.30 s) it also gives:
- the beam that tripped it;
- how far that obstacle was from the car's path, and from the edge of the body;
- how far the body's own path was actually clear, against the stopping distance.

A TTC stop whose obstacle sits outside the body edge, with the path clear well past the stopping distance, is a false alarm. Those are what led to the [October 2026 change](gap-follow.md#the-2026-10-19-run).

## Asking "what if": counterfactual replay

Add `--set name=value` to replay the same scans with a `gap_follow` parameter changed. Add `--yaml` too whenever `gap_follow.yaml` has changed since the run, so the replay starts from the config the car actually ran. The first command below saves that config from git; `eec6405` is the commit the 2026-10-19 run used.

```bash
git show eec6405:src/gap_follow/config/gap_follow.yaml > /tmp/gap_follow_20261019.yaml
python3 tools/run_analysis/analyze_run.py ~/.ros/racerbot_runs/20261019-200703 \
  --yaml /tmp/gap_follow_20261019.yaml --set ttc_path_margin=0.05
```

**Working when:** the replay section shows a steering error near 0.001 rad, which means the replay matches the car. The report then ends with a `Counterfactual replay` section: the stop count with the change, next to the count without it.

**If it doesn't:** a steering error of 0.05 rad or more means the replay is not running the config the car ran. Check which commit was on the car at the time of the run.

This is how the TTC change was checked against the real run: the 2026-10-19 scans produce 97 stop ticks with that run's config, and none with `ttc_path_margin=0.05`.

A parameter the old file predates takes the value that reproduces the old behaviour (`gap_mirror.LEGACY_VALUES`), so an old YAML replays correctly against newer code.

**Only use it to count stops.** The changed car would have been somewhere else a second later, and the replay cannot know where. To judge whether a change is *safe*, run the closed-loop simulator ([gap-follow.md](gap-follow.md#checking-it-in-the-simulator)).

## Fitting the simulator to the real car

The simulator's car model is `tools/f1tenth_sim/sim_fidelity/calibration.py`. Every number there records where it came from. `fit_sim.py` re-checks the actuation numbers against a run. It plays the run's recorded commands into the simulated car on an empty floor, then compares the simulated motion with the real car's measured motion.

**Terminal 1** — after `analyze_run.py` has run once on the same directory:

```bash
cd ~/racerbot-ws
python3 tools/run_analysis/fit_sim.py ~/.ros/racerbot_runs/20261019-200703 \
  --grid speed_delay_ticks=1,2,4 --grid speed_loop_brake_gain=2,4,8 --grid steering_delay_ticks=1,3,5
```

**Working when:** it prints up to ten lines, best first. Each line is `speed RMS ... yaw-rate RMS ...` followed by the settings that produced it.

**Then what:** if the best settings differ clearly from `calibration.py`, change them there, together with their `PROVENANCE` entry (name the run). Then re-run the sweeps in [gap-follow.md](gap-follow.md#checking-it-in-the-simulator). A changed car model moves every simulator result, so treat it like a code change.

Each combination costs about 5 seconds. A three-by-three-by-three grid is about 2½ minutes.

<details>
<summary><b>What the fit found on 2026-10-19, and why the speed loop was rewritten</b> — click to expand. Skip it unless you're changing the plant.</summary>

Before the fit, the simulator used F1TENTH Gym's own speed controller. It drives the car toward the commanded speed with a gain of 3.4 /s when speeding up and **13.7 /s when slowing down**, both up to the friction limit. The real car is the other way round. With the command 1–4 m/s below its speed, it slowed at a median of only ~1.7 m/s². With the command 0.5–1 m/s above, it sped up at ~2.9 m/s².

`sim_fidelity/plant.py` now has an asymmetric speed loop (`speed_loop_*` in `calibration.py`). Feeding gym's own gains into it reproduces gym exactly (0.422 vs 0.420 m/s RMS), which is how the conversion was checked. Fitted values:

| Setting | Before | Fitted |
|---|---|---|
| `speed_delay_ticks` | 1 | 2 |
| `steering_delay_ticks` | 1 | 3 |
| `steering_servo_gain` | 25 | 15 |
| `speed_loop_accel_gain` / `_max` | 3.4 / 6.9 | 5.0 / 4.5 |
| `speed_loop_brake_gain` / `_max` | 13.7 / 6.9 | 4.0 / 4.0 |
| `surface_friction` | 0.70 (estimated) | 0.55 |

Speed error went from 0.46 to 0.24 m/s, and turning-rate error from 0.25 to 0.15 rad/s. One run on one floor: fit again whenever there's a run on a different surface.

</details>

## Adding a map to the simulator

Any ROS occupancy map ([glossary](glossary.md#occupancy-grid)) can be a simulator track. That covers a SLAM map from `auto_map_race`, one saved with `map_saver_cli`, or one from another team's repo. The importer:
- turns unexplored grey into wall, so the simulated car can't drive into space the map never saw;
- finds the loop's centreline as the line equally far from the inner island and the outer walls;
- picks the straightest stretch as the start.

**Terminal 1:**

```bash
cd ~/racerbot-ws
cp ~/.ros/racerbot_auto/<ts>/map.pgm  tools/f1tenth_sim/maps/sources/hall_<date>.pgm
sed 's/image: map.pgm/image: hall_<date>.pgm/' ~/.ros/racerbot_auto/<ts>/map.yaml > tools/f1tenth_sim/maps/sources/hall_<date>.yaml
python3 tools/f1tenth_sim/track_import.py tools/f1tenth_sim/maps/sources/hall_<date>.yaml \
  --name hall_<date> --source "slam_toolbox map, run <ts>" --plot /tmp/hall.png
```

**Working when:** it prints `hall_<date>: {'lap_length_m': ..., 'width_min_m': ..., ...}`, and `/tmp/hall.png` shows a red centreline running round the corridor, with a green dot at the start. Then `python3 tools/f1tenth_sim/sweep_gap.py --track hall_<date> --seeds 6` drives it, and `--track maps` drives every imported map.

**If it doesn't:**

| Message or picture | Cause | Fix |
|---|---|---|
| `free space is not a loop` | The free area has no island to drive round: a dead-end room, or a gap in the map's walls | Check the map has a closed loop; draw the missing wall into the source image |
| Red line cuts through a wall | The wall has a gap narrower than the car the importer couldn't see | Close the gap in the source image and re-import |
| `width_min_m` tiny | An obstacle sits on the centreline; the line bends round it | Fine for an obstacle course. For a lap on the plain corridor, import an obstacle-free copy with `--centerline-from` |

The import command is saved in `tools/f1tenth_sim/maps/catalog.yaml`, so anyone can re-run it. Commit both `sources/` and the generated files.

Current tracks:

| Track | Source | Lap | Width (median) | Shipped config |
|---|---|---|---|---|
| `levine_blocked` | racerbot_a `levine_blocked`: a hall loop with its side corridors closed | 63 m | 1.6 m | passes |
| `levine_obs` | racerbot_a `levine_obs`: the same loop with boxes in the corridor | 64 m | 1.6 m | **fails** all 12 runs: wedges against a box |
| `circular_obs` | racerbot_a `circularTrack`: an elliptical ring with bars and blocks | 22 m | 1.2 m | **fails** 10 of 12 runs: wedges against a bar |
| `room_2026_07_27` | this car's SLAM map, `~/.ros/racerbot_auto/20260727-200103` | 15 m | 1.4 m | passes; one seed in six fails in reverse |

Plain `levine` (side corridors open) was left out on purpose. Its dead-end branches look like open corridor straight ahead, so a no-map reactive driver goes down them, and this car can't reverse out.

Rebuilding the 2026-10-19 loop from that run's own scans was tried, and the laps didn't line up. Without a SLAM back end to close the loop, the same spot came out up to 2 m apart on different laps, so no map was made from it. The 1.0 m and 1.2 m `narrow_*` layouts stand in for it until a SLAM map of that track exists.

## Testing several cars at once

```bash
cd ~/racerbot-ws
python3 tools/f1tenth_sim/eval_gap_multi.py --track indoor_wide,asb_10000 --cars 3 --seeds 4
python3 tools/f1tenth_sim/eval_gap_multi.py --track levine_blocked --cars 2 --speeds 1.8,3.5
```

All cars run the shipped `gap_follow` on their own simulated LiDAR, which sees the other cars as obstacles. They start one behind another along the track, 3 m apart (`--spacing`). By default the leader runs at 0.7 × `max_speed` and the others at full speed, so they catch it. `--speeds` sets each car's top speed directly, leader first.

**Working when:** it prints one line per track and exits. A fully passing track reads `pass N/N wall 0 car-car 0 unfinished 0`; the exit code is 0 only if every track did. `closest` is the smallest centre-to-centre distance between any two cars during the run.

Results for the shipped config are in [gap-follow.md](gap-follow.md#several-cars-at-once): not every run passes yet.

A run fails if any car touches a wall or another car, or if any car fails to complete `--laps` (default 2) laps in time. A car that catches a slower one and follows it is fine. One that stops behind it for good shows up as `unfinished`.

## How the pieces work

Everything lives in two places and imports no ROS, except reading the bag:

| File | What it does |
|---|---|
| `tools/run_analysis/runlog.py` | Bag → arrays (`run.npz`, cached), LB holds from `/joy`, the odometry speed the node had at any moment |
| `tools/run_analysis/scan_motion.py` | Point-to-point ICP between each scan and the one 2 scans later: forward speed, sideways speed, turn rate |
| `tools/run_analysis/actuation.py` | Lags, curvature gain, grip and odometry scale from commands vs measured motion |
| `tools/run_analysis/replay.py` | Feeds scans to `gap_mirror.GapController`; parity check; stop clusters; TTC geometry |
| `tools/run_analysis/fit_sim.py` | Plays recorded commands into `FidelityPlant` on an empty floor; grid search over `calibration.py` fields |
| `tools/run_analysis/analyze_run.py` | All of the above, one report |
| `tools/f1tenth_sim/track_import.py` | Occupancy map → gym track triplet + catalog entry |
| `tools/f1tenth_sim/eval_gap_multi.py` | N `gap_follow` cars on one track |

<details>
<summary><b>Why scan matching, and where it goes wrong</b> — click to expand.</summary>

`vesc_to_odom` computes the car's turning rate as `speed × tan(steering command) / wheelbase`. So odometry's turn rate agrees with the command by construction. On run 20261019-200703 it matched the command to a gain of 0.998 with no lag, which is impossible for a real servo. Odometry can never show understeer or steering lag, so it can't be the reference.

Matching two LiDAR scans gives the rigid move between them, measured from the walls. Its weak case is a long straight corridor with nothing along it: the walls fix the sideways position and the heading well, but say little about how far the car moved along the corridor. Then ICP can under-read speed. Check the `residual` column and compare against odometry on curved sections before trusting a straight-line speed figure.

The speed figures here use a 2-scan baseline (50 ms). That is short enough to follow braking and long enough that 6 mm of match noise is about 0.12 m/s.

</details>

<details>
<summary><b>Why the replay matches the car to the tick</b> — click to expand.</summary>

`gap_mirror.GapController` is a copy of `gap_follow_node._scan_step` without ROS. `src/gap_follow/test/test_gap_mirror_parity.py` holds it to the node's own commands to 1e-6. The replay gives it each scan in the order it arrived, and with it the odometry speed the node had at that moment (the latest `/odom` received at or before the scan). It also uses the real interval between scans for the steering and speed slew.

What is left over is clock jitter: the node measures intervals with its own clock, the replay with the bag's receive times. That is why a stop that fired within a millisecond of the TTC threshold can land one tick apart.

</details>

The tests run without ROS:

```bash
python3 -m pytest tools/f1tenth_sim/tests/test_track_import.py tools/run_analysis/tests -q
```

**Working when:** every test passes; the count matches `grep -c '^def test'` over those files.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `bag needs /scan and /drive` | The run was recorded with a reduced `topics:=` list | Re-record with the default topics; the analysis needs the scans |
| Replay steering error far above 0.001 rad | The run used different code or YAML than the tree | Pass the run's config with `--yaml` (see [counterfactual replay](#asking-what-if-counterfactual-replay)), or check out the commit the car ran |
| `No LB hold in /joy` | `/joy` wasn't recorded, or LB was never pressed | Check `ros2 bag info <run>/bag` lists `/joy` |
| Scan-matched speed spikes on straights | ICP along a featureless corridor | Trust curved sections; see [How the pieces work](#how-the-pieces-work) |

## See also

- [run-diagnostics.md](run-diagnostics.md) — recording the run in the first place.
- [gap-follow.md](gap-follow.md) — what these tools found on 2026-10-19, and what changed because of it.
- [simulator.md](simulator.md) — the no-ROS simulator the replay and fits use.
