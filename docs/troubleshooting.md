# Troubleshooting

Real issues hit while bringing this car up, in the order you're likely to hit them, with how they were actually diagnosed — not just the fix, so you can apply the same method to whatever's different next time.

## Nothing happens when holding LB

Check the controller is in **XInput mode**:
```bash
lsusb | grep 046d
```
Should show `[XInput Mode]`. If it shows `[DirectInput Mode]`, flip the small switch on the back of the controller/receiver back to X, then restart `joy_node` — it holds onto the old (now-gone) device handle and won't pick up the new one on its own:
```bash
pkill -f joy_node
ros2 run joy joy_node --ros-args -r __node:=joy --params-file install/f1tenth_stack/share/f1tenth_stack/config/joy_teleop.yaml
```

If it's already in XInput mode, confirm LB is actually registering:
```bash
ros2 topic echo /joy
```
`buttons[4]` should read `1` while held. Holding a shoulder button while working the same-side stick is easy; holding LB while working the *opposite*-side stick (right stick, for steering) is easy to lose grip on without noticing — a genuinely steady hold is required.

## One axis doesn't do what you expect

Don't trust assumed Xbox-style axis numbering — verify empirically:
```bash
ros2 topic echo /joy
```
Watch which `axes[]` index moves as you work each stick/trigger. On this F710 in XInput mode: axes 2 and 5 (triggers) rest at `1.0` released; axis 1 is left-stick-Y; axis 3 is right-stick-X. This bit us once already — upstream `joy_teleop.yaml` shipped steering on axis 2 (left trigger), not the right stick. Already patched locally; if you ever regenerate this file from upstream, re-check it.

## ROS-side commands look right but the car doesn't respond

```bash
ros2 topic echo /commands/servo/position
ros2 topic echo /commands/motor/speed
```
If those show real, varying values in response to your input, the problem is downstream of ROS — servo/motor wiring to the VESC, VESC power, or VESC firmware config. If they *don't* vary, the problem is upstream — check `/teleop` and `/ackermann_cmd` to find where the chain breaks (see [architecture.md](architecture.md) for the full topic path).

## Steering servo does nothing, VESC connects fine (`fault_code: 0`)

This happened on first bring-up. Full diagnostic trail:

1. Confirmed the ROS→VESC link was healthy: `vesc_driver_node` connects (firmware version logged), `fault_code: 0` in `/sensors/core`, no errors.
2. Read `vesc_driver`'s source directly — confirmed it correctly builds a standard `COMM_SET_SERVO_POS` protocol packet and sends it over serial. Not a bug in this repo.
3. Multimeter on the servo header: **5V and GND present**, but the signal wire stayed flat **0V** regardless of commanded position, across multiple distinct test commands.
4. That combination — command accepted with no fault, power present, zero signal output — points at one thing: the VESC's servo/PPM output disabled in its own firmware app configuration. This is a per-VESC firmware setting, unrelated to anything in this ROS stack, and it's a common factory-default state since not every VESC build uses the servo header.
5. This ROS stack **cannot** read or write that setting — `vesc_driver` only implements motor/servo control commands, not the `COMM_GET_APPCONF`/`COMM_SET_APPCONF` config protocol VESC Tool uses.

**Fix:** stop the ROS bringup (frees the serial port — only one process can hold it), connect the official **VESC Tool** app over USB, enable servo output under App Settings, write the config, restart the bringup.

## Testing `/commands/servo/position` (or `/commands/motor/speed`) directly and seeing weird "twitching"

If you inject a raw `ros2 topic pub` command into `/commands/servo/position` while **`bringup_launch.py` and `teleop_launch.py` are both still running**, you'll likely see inconsistent, twitchy behavior that looks like a hardware fault but isn't.

Cause: `joy_teleop`'s `default` profile has no deadman-button restriction and continuously republishes a neutral command as a safety fail-safe, flowing through `ackermann_mux` → `ackermann_to_vesc_node` → the exact same `/commands/servo/position` topic you're injecting into. Two publishers end up racing on one topic, and whichever message arrived most recently wins — that interleaving is the "twitching," not a real fault.

This was diagnosed by systematically eliminating variables: single commands vs. repeated/continuous commands, checked power under load (multimeter, stayed rock-steady — ruled out a supply issue), and a wiggle test on the connector (inconclusive — twitching persisted independent of touching the cable, which argued against a loose connection). What actually resolved it was checking `/ackermann_cmd` and realizing it never reflected the injected value at all — it stayed at the joystick's neutral output the whole time.

**Fix / how to avoid it:** either stop `ackermann_to_vesc_node` before injecting raw test commands, or just trust a real controller test over raw topic injection — the actual controller worked the entire time this "issue" was being chased.

## Autonomy node publishes to `/drive`, car doesn't move, no errors anywhere

This is expected behavior, not a bug — see [architecture.md](architecture.md#the-safety-model-read-this-before-writing-autonomy-code). `/teleop` (joystick, priority 100) permanently masks `/drive` (navigation, priority 10) in `ackermann_mux` as long as `teleop_launch.py` is running, because the joystick's neutral output never times out. Confirm with:
```bash
ros2 topic echo /ackermann_cmd
```
If it's stuck at `0.0 / 0.0` regardless of what your node publishes to `/drive`, this is why. Fix: check whether `teleop_launch.py` is running in another terminal and `Ctrl+C` it. If you're following [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node) as documented, you won't have it running at all — `bringup_launch.py` no longer starts `joy_teleop` automatically, so this shouldn't come up unless you launched `teleop_launch.py` deliberately.

## Autonomy node publishes to `/drive`, `/ackermann_cmd` looks fine, but it's always `0.0 / 0.0` even with no `teleop_launch.py` running

Different root cause from the one above, easy to conflate. First, are you holding LB? Every autonomy node in this workspace requires it — this is current, mandatory workspace policy (see [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)), not a bug. If you are holding it and it's still stuck at zero, check whether `joy_node` is even still running:
```bash
ros2 node list | grep joy
```
If it isn't, this is why: `gap_follow_node` and `pure_pursuit_node` both have their **own** deadman-button check, separate from `ackermann_mux` — each subscribes to `/joy` directly and only publishes a non-zero command while LB is held on a *live* `/joy` stream (see `gap_follow_node.py`'s or `pure_pursuit_node.py`'s `joy_callback`/`_deadman_engaged`). `joy_node` lives in `bringup_launch.py`, so it should normally always be up while autonomy runs — if it isn't, something killed it (see the `pkill -f` gotcha below), or `bringup_launch.py`'s terminal itself died. Either way, no `/joy` means the node's own deadman check can never engage. The autonomy launch terminal now reports this explicitly as `STOP [waiting_for_joy]` or `STOP [joy_stale]`, including the configured timeout. A custom node should have the same check per [writing-your-own-node.md](writing-your-own-node.md#the-interface-contract) — if it doesn't, that's a bug in that node, not expected behavior.

**Fix:** restart `bringup_launch.py` so `joy_node` comes back, and hold LB while your autonomy node is active. See [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node).

## "slam_toolbox failed to save occupancy map (result code 255)"

Symptom: the run completes and races, `posegraph.*` and `raceline_*.csv` are
in the output directory, but `map.pgm`/`map.yaml` are not. The log has
`[map_saver]: Failed to spin map subscription` just before it.

`SaveMap` runs nav2's `map_saver` inline inside slam_toolbox, and
`map_saver` gives up after about two seconds if no `/map` message arrives
in that window. `/map` is only republished every `map_update_interval`
(5 s in `f1tenth_online_async.yaml`), so whether the save works is a race
against when the request happens to land — the same run succeeds or fails
with nothing else different.

`auto_map_race_node` now retries (`map_save_retries`, default 3, spaced
`map_save_retry_delay_sec`), which lands the request in a different part of
that window. If every attempt fails the run continues anyway and says so:
the racing line is already on disk and the pose graph usually saved fine,
and a map can be rebuilt from a pose graph with slam_toolbox's
`deserialize_map`. Shortening `map_update_interval` would also close the
race, at the cost of republishing a growing grid more often.

## `auto_map_race_launch.py` maps forever and never switches to pure pursuit

Symptom: the car drives cautious `gap_follow` laps indefinitely. The
supervisor keeps printing `lap 1/2: ...` and the racing phase never starts.

Read the numbers in that line; each gate says which one is unmet:

```
lap 1/2: samples=168, distance=27.2/5.0m, turn=341/300deg, elapsed=31.4/15.0s,
departed=yes, start distance=0.43/0.75m, heading error=6.2/30.0deg,
SLAM corrections absorbed=6
```

- **`start distance` never drops below its limit.** The car is not
  returning close enough to where the recorder started. That start point is
  wherever the car was when SLAM's `map->base_link` first appeared, which
  is a few seconds *into* the run, so it may be mid-corner. Raise
  `closure_distance`, or start the run with the car already sitting still
  and let SLAM come up before holding LB.
- **`heading error` too large at the moment it passes.** Same cause,
  different axis; `closure_heading_deg` is the knob.
- **`turn` climbing far past 300 without closing.** The car is going round
  and round without ever satisfying the proximity gates -- see above. It is
  not a `minimum_lap_turn_deg` problem, and past two laps of turning the log
  says so outright. Measured worst case: a car weaving around two slower
  cars drove 413m and ten laps' worth of turning, passing 6.5m wide of its
  start every time. `closure_distance` is the knob.
- **`SLAM corrections absorbed` in the dozens per lap.** Localisation is
  struggling. Look at the map in the dashboard before trusting anything
  downstream; a smeared map moves the start point out from under the
  closure test.

Before 2026-08-08 there was a fourth cause and it was the usual one:
`minimum_lap_distance` defaulted to `20.0`, longer than the ~15m loop this
car is driven on, so closure could not fire until the car had been round
*twice*. It is now `5.0`, with `minimum_lap_turn_deg` doing the real work.

## `auto_map_race_launch.py` refuses to start racing: "Refusing to hand it to pure pursuit"

Symptom: the mapping laps complete, then the supervisor logs an error and
stays stopped.

This is the racing line being rejected as unfollowable — deliberately, and
the message says by how much. A line the rack cannot steer does not degrade
gracefully; it saturates the steering, runs wide, and latches on the
emergency stop, which is what used to happen instead.

The message names which check failed. `passes closer to a wall than the car
is wide` is the geometric one — the cleaned line does not fit in the
corridor, usually because filtering rounded a corner inward on a course
whose corners are already near the car's turning circle. `asks for more
steering than the car has` is the kinematic one.

The two real causes:

1. **The course has a corner tighter than the car.** `tan(0.26)/0.324` is a
   1.22m minimum turning radius. Plot `raceline_raw.csv` (written even on a
   refusal) over the saved `map.pgm` and look at the tightest corner. If
   that is the answer, the course needs opening out — no amount of
   filtering fixes geometry.
2. **The map is smeared, so the recorded pose is not the path.** Check the
   map in the dashboard and the `SLAM corrections absorbed` count in the
   mapping log.

`profile_reject_ratio` and `profile_reject_fraction` in `auto_map_race.yaml`
control how strict the refusal is. Raising them races a line the car
partially cannot steer — do that only with the wheels off the ground first,
and read
[racing-autonomy.md](racing-autonomy.md#what-a-recorded-lap-actually-looks-like)
for what those numbers mean.

## `slam_toolbox` starts without errors but no map, no `map` frame, and mapping never finishes

Symptom: `slam_launch.py` (or anything including it — `autonomous_mapping_launch.py`, `auto_map_race_launch.py`) starts cleanly and `slam_toolbox` appears in `ros2 node list`, but `ros2 topic echo /map --once` never returns, `ros2 run tf2_ros tf2_echo map base_link` reports `"map" passed to lookupTransform argument target_frame does not exist`, and `auto_map_race_node` logs `lap 1/N: recorder waiting for a valid map->base_link pose` forever no matter how many laps you drive.

The giveaway is what `slam_toolbox` *didn't* print. Its full healthy startup is two lines:
```
[async_slam_toolbox_node-N] [INFO] [slam_toolbox]: Node using stack size 40000000
[async_slam_toolbox_node-N] [INFO] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
```
The first line comes from the constructor; the second from `on_configure`. If you only ever see the first, the node is parked in the `unconfigured` lifecycle state:
```bash
ros2 lifecycle get /slam_toolbox     # should say: active [3]
```
As of `slam_toolbox` 2.x (this car has 2.8.5 from `ros-jazzy-slam-toolbox`) `async_slam_toolbox_node` is an `rclcpp_lifecycle::LifecycleNode` and **does not configure itself** — no `autostart` parameter exists inside the node. Reading the parameter file, loading the solver, subscribing to `/scan`, publishing `/map`, publishing the `map`→`odom` TF, and advertising `/slam_toolbox/save_map` all happen in `on_configure`/`on_activate`, and the launch file has to emit those transitions. Started as a plain `launch_ros.actions.Node` it comes up and does nothing at all, silently, forever. Everything downstream then waits on it: the lap recorder never gets a pose, so no raceline is generated, so `pure_pursuit_node` stays on `STOP [waiting_for_profile]` and the run never leaves the cautious gap-follow mapping phase.

**Fix:** launch it as a `LifecycleNode` and emit `TRANSITION_CONFIGURE`, then `TRANSITION_ACTIVATE` on reaching `inactive` — that's what `racerbot_launch/launch/slam_launch.py` does now (mirroring upstream's `slam_toolbox/launch/online_async_launch.py`). If you hit this in a launch file of your own, copy that pattern rather than adding a plain `Node`.

## `pkill -f joy_teleop` takes down `joy_node` too (and cascades into killing the whole bringup)

You shouldn't need to `pkill` anything to switch between manual driving and autonomy anymore — `Ctrl+C` whichever control-layer terminal (`teleop_launch.py`, `gap_follow_launch.py`, `pure_pursuit_launch.py`) is running instead (see [operations.md](operations.md#the-two-layer-pattern-used-in-every-procedure-below)). But if you ever do reach for `pkill -f joy_teleop` out of habit (from before `teleop_launch.py` existed as its own file), know that it's still a trap: `joy_node` (started by `bringup_launch.py`) is launched with `--params-file .../joy_teleop.yaml` — the same config file `joy_teleop` uses, just a different top-level key inside it — so `joy_node`'s own command line contains the literal substring `joy_teleop` too. `pkill -f` matches against the whole command line, so it kills **both** processes, not just the one you meant to stop. Losing `joy_node` kills every autonomy node's deadman input (see the entry above) and, since `ros2 launch` treats any process it manages dying as fatal, tears down the *entire* bringup (VESC and LiDAR included) a few seconds later. If you ever do need to kill one specific node by pattern instead of `Ctrl+C`-ing its terminal, match something only that process's command line contains — e.g. `pkill -f "__node:=joy_teleop"` — or kill it by exact PID.

## New terminal, permission denied on `/dev/sensors/vesc` or `/dev/input/js0`

Group membership (`dialout` for the VESC, `input` for the joystick) only takes effect in login sessions started *after* the group was added. This isn't a udev or wiring problem — open a fresh terminal, or run `newgrp dialout && newgrp input` in the current one.

## General debugging approach that worked repeatedly here

1. **Check the actual topic data, not just node logs.** `ros2 topic echo` on the exact topic at each stage of the chain (see the topic table in [architecture.md](architecture.md)) narrows down which link is broken far faster than guessing.
2. **Isolate one variable at a time.** When the servo issue looked hardware-related, systematically ruling out power (multimeter under load), connection (wiggle test), and repeated-vs-single commands one at a time is what eventually surfaced the real cause (competing publishers), rather than jumping to "must be a bad servo."
3. **Verify empirically, don't trust assumed defaults** — axis numbering, topic names implied by a launch file's remap arguments (`ackermann_cmd_out` → `ackermann_drive` in `bringup_launch.py` looks like it should change the output topic name, but empirically the mux still publishes to `/ackermann_cmd` — the remap doesn't match the package's actual internal topic name, so it's a silent no-op). Read the launch file, then check `ros2 node info` / `ros2 topic list` against it before trusting what the source implies.
