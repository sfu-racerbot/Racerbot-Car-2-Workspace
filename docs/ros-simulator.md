# ROS-level simulator (`racerbot_sim`)

> **Who this is for:** anyone who wants to run the real driving stack — launch files, SLAM, the dashboard — without the physical car.
> **Read first:** [concepts.md](concepts.md), and [simulator.md](simulator.md) for the other, simpler simulator and how the two differ.
> **You'll be able to:** run and validate whole launch files against simulated physics, and understand the interlock that stops this running beside real hardware.
> **Looking for the steps?** [sim-validation.md](sim-validation.md) is the step-by-step workflow — set up, run your own node, connect the dashboard, get a verdict. This doc is the reference behind it.

`tools/f1tenth_sim/run_validation.py` ([docs/simulator.md](simulator.md))
calls the controllers' *math* directly and skips ROS entirely. That is the
right shape for tuning a control law. It is the wrong shape for the way
`auto_map_race_launch.py` actually broke, which was in the wiring around
the math: SLAM, TF, a racing line recorded from a live pose, a runtime
parameter handover between two nodes, and a safety layer with no way out
of its own stop. None of that exists in a harness with no ROS in it.

`racerbot_sim` is the other half. It replaces **only** the hardware:

| `bringup_launch.py` | `sim_bringup_launch.py` |
|---|---|
| `joy_node` (physical F710) | `sim_joy_node` (synthetic LB) |
| `urg_node` (Hokuyo) | `gym_bridge_node` |
| `vesc_driver` + `ackermann_to_vesc` + `vesc_to_odom` | `gym_bridge_node` |
| `ackermann_mux` | `ackermann_mux` — the same node, same config |
| static `base_link->laser` | the same node, same 0.33/0/0.11 offset |

Everything above that line is the real thing: the real `ackermann_mux`
arbitrating `/teleop` over `/drive`, the real `slam_toolbox`, the real
`gap_follow_node`, `pure_pursuit_node` and `auto_map_race_node`, the real
`web_dashboard`. `sim_auto_map_race_launch.py` deliberately *includes*
`racerbot_launch`'s own `auto_map_race_launch.py` with
`include_bringup:=false` rather than reproducing it, so what is validated
is the launch file people actually run.

## It refuses to run next to the car

Two nodes here are dangerous beside real hardware, and the Jetson on the
car is exactly where this is most convenient to run:

- **`sim_joy_node` forges the LB deadman.** The whole workspace safety
  policy ([architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car))
  is that nothing moves unless a human is holding LB. This node is that
  human's hand, forged.
- **`gym_bridge_node` publishes an imaginary `/scan` and `/odom`.**
  Alongside `urg_node` and `vesc_to_odom_node` that is two publishers per
  topic, and the driving nodes would be steering on a blend of a real room
  and an invented one.

So neither publishes anything while any of `vesc_driver_node`,
`ackermann_to_vesc_node`, `vesc_to_odom_node`, `urg_node` or `joy` is on
the ROS graph. The check repeats, so bringing the car up *after* the
simulator silences the simulator too (`racerbot_sim/hardware_guard.py`).
It is a live-graph check rather than a flag because a flag is something
you can forget.

## Running it

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# The whole automatic composition, plus the dashboard on :8080
ros2 launch racerbot_sim sim_auto_map_race_launch.py dashboard:=true

# A tighter course, with two slower cars on it
ros2 launch racerbot_sim sim_auto_map_race_launch.py \
    track:=indoor_tight opponents:="6,0.7,0.0;14,0.6,0.0"

# Just the hardware layer, to put your own control layer on top
ros2 launch racerbot_sim sim_bringup_launch.py
ros2 launch gap_follow gap_follow_launch.py     # second terminal
```

Needs the F1TENTH Gym from `tools/f1tenth_sim/setup.sh` (one time).

It runs in real time and does not use `/clock`: every node above the
hardware layer is the one that runs on the car, with its own wall-clock
timers and its own `use_sim_time: false`. That costs test duration (a full
validation is about four minutes per scenario) and buys the thing this
exists for -- the timing the nodes actually see. It also means runs are not
bit-identical: the ROS graph is real, so scheduling varies, and a result
that depends on a thousandth of a radian will vary with it.

### Arguments

| Argument | Default | What it does |
|---|---|---|
| `track` | `indoor_oval` | `indoor_oval` (30m lap, 1.8m corridor), `indoor_tight` (27m, 1.4m), `indoor_wide` (42m, 2.6m — the one this car can actually *race*, see below) |
| `opponents` | *(none)* | `;`-separated `offset_m,speed_mps,lateral_m`. Speed `0` parks that car on the line as a static obstacle |
| `hold_deadman` | `true` | `false` proves the car stays stopped with LB released |
| `release_after_sec` | `-1` | Release the synthetic LB this far into the run |
| `odom_speed_scale` | `1.0` | Odometry scale error, as a miscalibrated `speed_to_erpm_gain` would give |
| `seed` | `12345` | LiDAR noise and odometry noise |
| `dashboard` | `false` | Also start `web_dashboard` |

### Why the tracks are small

The official F1TENTH tracks are 300m+ laps. `auto_map_race_launch.py` maps
at 1.0 m/s, so two mapping laps plus racing on one of those is over twenty
minutes of wall clock per run — long enough that nobody runs it, which is
how the automatic mode reached the state it was in. The generated layouts
are room-sized closed loops in the same size class as the space this car
is actually driven in: a ~30m lap, mapped in about thirty seconds, whole
validation under four minutes.

Their corner radii are all comfortably outside the car's own 1.22m minimum
turning circle, which the real course this car is driven on is *not* —
see [racing-autonomy.md](racing-autonomy.md#what-a-recorded-lap-actually-looks-like).

## Seeing what the dashboard sees

"The SLAM map looked really glitchy" is the way a mapping run is usually
judged, and it is hard to act on without the picture.

```bash
ros2 launch racerbot_sim sim_auto_map_race_launch.py dashboard:=true
tools/racerbot_sim/capture_dashboard.py --output /tmp/dashboard.png
```

`capture_dashboard.py` connects to the same WebSocket a browser does,
decodes the same messages `web_dashboard/protocol.py` sends, and draws them
the way `web/dashboard.js` does -- occupancy grid, LiDAR points, car arrow
-- into a PNG. It only listens, so it is safe against the real car too.

With `--seconds` and `--interval` it is the dashboard's test instrument
rather than a screenshot tool: it watches a whole run, writes a frame per
phase, checks every binary frame against the length its header declared,
and exits non-zero if any failed. Measured over a full auto-map race:

| | mapping -> handover -> racing |
|---|---|
| binary frames | 2516, **0 failed the length check** |
| pose updates | 11196 |
| map republished | 57 times, resizing 17 times |
| view disturbances | 28 under the old auto-fit, **2** under the current one |

It also replays both view-fitting policies over the recorded map sequence,
so "the map looks glitchy" can be attributed to the view moving, to SLAM
genuinely smearing, or to two stacks running at once -- see
[web-dashboard.md](web-dashboard.md#the-map-looks-glitchy).

## Automated validation

```bash
tools/racerbot_sim/run_auto_map_validation.py --scenario all
```

Runs three scenarios, each end to end, and exits non-zero if any fails:

| Scenario | What it puts on the track | What it proves |
|---|---|---|
| `solo` | nothing | mapping, cleanup, save, handover, and a clean racing stint |
| `obstacle` | one parked car on the racing line | the racing controller gets past a static obstacle rather than latching stopped in front of it |
| `traffic` | two slower cars | mapping with moving traffic, and racing among it without contact |

**All three scenarios run on `indoor_wide`**, overriding the launch files'
`indoor_oval` default, because what limits them is the *corridor* rather
than the controller:

- **Racing at all.** The line comes from `gap_follow`, which drives
  0.25-0.35m from a corner's wall, and pure pursuit's cross-track error
  through a corner near this car's turning circle measured 0.39-0.57m at
  2.5-3.0 m/s. On `indoor_oval`'s 1.8m corridor those do not both fit, and
  the car touches the wall -- a true statement about the course, not a
  defect to gate a regression suite on.
- **Getting past anything.** `gap_follow` inflates every obstacle by
  `car_width/2 + safety_margin`, which demands 0.67m of width, and a car in
  the middle of a 1.8m corridor leaves 0.6m either side and is simply a
  roadblock. Failing that proves nothing about the code.

Run `--track indoor_oval` or `--track indoor_tight` deliberately to ask the
narrower question: does the pipeline still map, clean up, and *refuse*
correctly. Note the consequence for a hand-run demo -- launching
`sim_auto_map_race_launch.py` on its `indoor_oval` default and watching the
car wedge itself into a corner during the racing phase is the expected
result, not a regression. Pass `track:=indoor_wide` to watch it race.

Each scenario is judged on:

- every phase reached — SLAM up → mapping lap → line cleaned → profile
  written → map saved → profile loaded → racing;
- no node crash, no unhandled exception, no refused profile;
- no wall contact for *any* car, and no car-to-car contact;
- **distance covered after the handover**, not just total distance. Total
  distance is dominated by the mapping laps and stays high even when the
  racing phase never moves. A run of the obstacle scenario passed on
  exactly that hole, having spent its entire racing phase stopped nose to
  nose with the parked car.

The generated racing line and the full launch log are kept for a failure.
The checked-in result is [docs/auto-map-sim-results.json](auto-map-sim-results.json)
(2026-08-08, seed 12345, all three scenarios passing):

| Scenario | Raced | Wall contact | Car contact | Line |
|---|---:|---:|---:|---|
| solo | 138.7 m / 81.6 s | 0 | 0 | 12.5° of the 14.9° rack, 0.83 m from the wall |
| obstacle | 29.7 m / 82.0 s | 0 | 0 | 0.0% of waypoints over the rack |
| traffic | 83.1 m / 80.7 s | 0 | 0 | map flagged as containing the other cars |

## What it does and does not model

**Real:** F1TENTH Gym single-track dynamics with RK4 at 5ms, multi-car
ray-cast LiDAR and collisions, 1081 beams over 270° matching the Hokuyo
UST-10LX, the 0.33m LiDAR offset, the padded 0.58 x 0.31m body, 40Hz
control, and the real ROS graph above the drivers.

**Dead-reckoned odometry, not ground truth.** `/odom` is integrated from
wheel speed and the *commanded* servo angle exactly as `vesc_to_odom` does
with `use_servo_cmd_to_calc_angular_velocity`, so it drifts in the same
shape the car's does. Publishing ground truth there would hide every
mapping problem this exists to find. Ground truth is published separately
on `/sim/ground_truth_pose` in its own unconnected `sim_world` frame, for
scoring only — nothing that drives the car may read it.

**Not modelled:** tyre grip against a real floor, VESC and servo dynamics,
LiDAR reflectivity and multi-echo, WiFi, and CPU contention on the Jetson.
[sim-fidelity-audit.md](sim-fidelity-audit.md) covers how far the shared
vehicle model is from this car; all of it applies here.

### One thing the older harness cannot see

Worth knowing if you rely on `run_validation.py`'s "no collision" result:
in the pinned Gym revision, wall collision is
`raw_scan - side_distances <= 0.005`, and with the vehicle parameters this
workspace uses (`collision_body_center_x = wheelbase/2` against the ST
model's own `-lr` offset) the LiDAR lands 0.04m *outside* the collision
rectangle, so `side_distances` comes out all zeros. The test reduces to
"did a beam return less than 5mm", which it cannot, because `range_min` is
0.05m. **Wall collisions never fire.** Measured directly: a car driven
straight into a wall at 1 m/s reports `collision=False` and keeps going,
right off the edge of the map.

`racerbot_sim` therefore samples the padded body against the occupancy
grid itself (`SimBridge.body_contact`) and treats off-map as contact. That
is what the validation criteria use.
