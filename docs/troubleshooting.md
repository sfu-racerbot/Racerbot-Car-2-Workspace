# Troubleshooting

> **Who this is for:** anyone whose car, build, or launch isn't doing what it should.
> **Read first:** nothing — come straight here when something breaks.
> **What's in it:** real problems hit during bring-up, how each was diagnosed, and the fix.

Every entry here is a problem someone actually hit on this car, with the fix and — folded away — how it was diagnosed. The diagnostic trails are included on purpose: the next thing that breaks won't be on this list, and the *method* transfers even when the answer doesn't.

---

## Find your symptom

| What you're seeing | Go to |
|---|---|
| Holding LB does nothing | [Nothing happens when holding LB](#nothing-happens-when-holding-lb) |
| A stick or trigger controls the wrong thing | [One axis doesn't do what you expect](#one-axis-doesnt-do-what-you-expect) |
| Topics look right, car doesn't move | [ROS-side commands look right but the car doesn't respond](#ros-side-commands-look-right-but-the-car-doesnt-respond) |
| Steering servo dead, VESC otherwise fine | [Steering servo does nothing](#steering-servo-does-nothing-vesc-connects-fine-fault_code-0) |
| Servo twitches when you `ros2 topic pub` at it | [Weird twitching during direct injection](#testing-commandsservoposition-or-commandsmotorspeed-directly-and-seeing-weird-twitching) |
| Autonomy publishes `/drive`, nothing moves, no errors | [Autonomy publishes, car doesn't move](#autonomy-node-publishes-to-drive-car-doesnt-move-no-errors-anywhere) |
| Same, but `teleop_launch.py` definitely isn't running | [Stuck at 0.0/0.0 with no teleop](#autonomy-node-publishes-to-drive-ackermann_cmd-looks-fine-but-its-always-00--00-even-with-no-teleop_launchpy-running) |
| Map didn't save after a run | [slam_toolbox result code 255](#slam_toolbox-failed-to-save-occupancy-map-result-code-255) |
| Maps forever, never starts racing | [Never switches to pure pursuit](#auto_map_race_launchpy-maps-forever-and-never-switches-to-pure-pursuit) |
| "Refusing to hand it to pure pursuit" | [Racing line rejected](#auto_map_race_launchpy-refuses-to-start-racing-refusing-to-hand-it-to-pure-pursuit) |
| SLAM runs but produces no map at all | [slam_toolbox never configures](#slam_toolbox-starts-without-errors-but-no-map-no-map-frame-and-mapping-never-finishes) |
| Killing one node took down everything | [The pkill trap](#pkill--f-joy_teleop-takes-down-joy_node-too-and-cascades-into-killing-the-whole-bringup) |
| Permission denied on a device | [New terminal, permission denied](#new-terminal-permission-denied-on-devsensorsvesc-or-devinputjs0) |

**Nothing matching?** Skip to [the general debugging approach](#general-debugging-approach-that-worked-repeatedly-here) at the bottom. It's the method that found most of the entries above.

---

## Nothing happens when holding LB

**First, check the controller is in XInput mode.**

```bash
lsusb | grep 046d
```

**Working when:** the output says `[XInput Mode]`.

If it says `[DirectInput Mode]`, flip the small switch on the back of the controller/receiver back to **X**. Then restart `joy_node` — it holds onto the old, now-gone device handle and won't pick up the new one on its own.

**Terminal 3, or any spare one:**

```bash
pkill -f joy_node
ros2 run joy joy_node --ros-args -r __node:=joy --params-file install/f1tenth_stack/share/f1tenth_stack/config/joy_teleop.yaml
```

**Working when:** `ros2 topic echo /joy` starts producing messages again, and `buttons[4]` flips to `1` as you press LB.

**If it's already in XInput mode, confirm LB is actually registering.**

```bash
ros2 topic echo /joy
```

**Working when:** `buttons[4]` reads `1` while you hold LB.

> **This is more often the problem than people expect.** Holding a shoulder button while working the stick on the *same* side is easy. Holding **LB** while working the *opposite* stick — the right stick, for steering — is easy to lose grip on without noticing.
>
> The [deadman](glossary.md#deadman) is the button you must hold for the car to move at all. It needs a genuinely steady hold, and a momentary release reads as "stop".

---

## One axis doesn't do what you expect

**Don't trust assumed Xbox-style axis numbering.** Verify it empirically:

```bash
ros2 topic echo /joy
```

Watch which `axes[]` index moves as you work each stick and trigger.

On this F710 in XInput mode:

| Index | What it is |
|---|---|
| `axes[1]` | left stick, Y |
| `axes[2]`, `axes[5]` | triggers — note these **rest at `1.0`** when released, not `0.0` |
| `axes[3]` | right stick, X |

> **This bit us once already.** Upstream `joy_teleop.yaml` shipped steering on axis 2 — the left trigger — not the right stick. It's already patched locally, but if you ever regenerate that file from upstream, re-check it. See [git-setup.md](git-setup.md).

---

## ROS-side commands look right but the car doesn't respond

Check what's reaching the motor and servo:

```bash
ros2 topic echo /commands/servo/position
ros2 topic echo /commands/motor/speed
```

**If those show real, varying values** in response to your input, the problem is **downstream of ROS**: servo/motor wiring to the VESC, VESC power, or VESC firmware config.

**If they don't vary**, the problem is **upstream**. Check `/teleop` and `/ackermann_cmd` to find where the chain breaks — the full topic path is in [architecture.md](architecture.md).

---

## Steering servo does nothing, VESC connects fine (`fault_code: 0`)

This happened on first bring-up.

**Cause:** the VESC's servo/PPM output is disabled in its own firmware app configuration. This is a per-VESC firmware setting with nothing to do with this ROS stack, and it's a common factory-default state — not every VESC build uses the servo header.

**Fix:**

1. Stop the ROS bringup. This frees the serial port; only one process can hold it.
2. Connect the official **VESC Tool** app over USB.
3. Enable servo output under **App Settings**.
4. Write the config, then restart the bringup.

> **This ROS stack cannot fix it for you.** `vesc_driver` only implements motor and servo *control* commands — not the `COMM_GET_APPCONF` / `COMM_SET_APPCONF` config protocol that VESC Tool uses. There is no ROS-side workaround.

<details>
<summary><b>The full diagnostic trail</b> — how "servo is dead" was narrowed to a firmware setting. Read it for the method, which generalizes to any "command accepted, nothing happens" hardware problem.</summary>

1. **Confirmed the ROS→VESC link was healthy.** `vesc_driver_node` connects and logs its firmware version, `/sensors/core` reports `fault_code: 0`, no errors anywhere.
2. **Read `vesc_driver`'s source directly.** Confirmed it correctly builds a standard `COMM_SET_SERVO_POS` protocol packet and sends it over serial. So it wasn't a bug in this repo.
3. **Put a multimeter on the servo header.** **5V and GND present**, but the signal wire stayed flat at **0V** regardless of commanded position, across multiple distinct test commands.
4. **Read the combination.** Command accepted with no fault + power present + zero signal output points at exactly one thing: the output is disabled at the firmware level, below anything software can reach.
5. **Confirmed the stack couldn't address it.** `vesc_driver` doesn't implement the config protocol at all, so no amount of ROS-side work would have helped.

The generalizable step is 3. Everything up to that point was consistent with a dozen different faults; a multimeter on the actual signal line cut the search space in half in one measurement.

</details>

---

## Testing `/commands/servo/position` (or `/commands/motor/speed`) directly and seeing weird "twitching"

**Symptom:** you inject a raw `ros2 topic pub` command while **`bringup_launch.py` and `teleop_launch.py` are both still running**, and get inconsistent, twitchy behavior that looks like a hardware fault.

**Cause: it isn't a fault — it's two publishers racing on one topic.**

`joy_teleop`'s `default` profile has no deadman-button restriction and continuously republishes a neutral command as a safety fail-safe. That flows through `ackermann_mux` → `ackermann_to_vesc_node` → the exact same `/commands/servo/position` topic you're injecting into.

Two publishers, one topic, and whichever message arrived most recently wins. That interleaving *is* the twitching.

**Fix:** either stop `ackermann_to_vesc_node` before injecting raw test commands, or — better — just trust a real controller test over raw topic injection.

<details>
<summary><b>The diagnostic trail, including the dead ends</b> — three plausible hardware theories that were all wrong, and the check that actually settled it.</summary>

This one is worth reading because most of the effort went into the wrong places, and the wrong places were reasonable.

- **Single vs. repeated commands** — tried both, behavior differed, which *looked* like it implicated timing or a flaky connection.
- **Power under load** — multimeter, stayed rock-steady. Ruled out a supply issue.
- **Wiggle test on the connector** — inconclusive. Twitching persisted independent of touching the cable, which argued against a loose connection but didn't prove anything.

**What actually resolved it:** checking `/ackermann_cmd` and realizing it *never reflected the injected value at all*. It sat at the joystick's neutral output the entire time. The injected commands were being overwritten downstream, not mangled by hardware.

The lesson: the whole investigation was happening one layer below where the problem was. Checking the topic chain first — the thing recommended in [the general approach](#general-debugging-approach-that-worked-repeatedly-here) — would have found it in a minute.

And the punchline: **the actual controller worked the entire time this "issue" was being chased.**

</details>

---

## Autonomy node publishes to `/drive`, car doesn't move, no errors anywhere

**This is expected behavior, not a bug.** See [architecture.md](architecture.md#the-safety-model-read-this-before-writing-autonomy-code).

`/teleop` (joystick, priority 100) permanently masks `/drive` (navigation, priority 10) in `ackermann_mux` for as long as `teleop_launch.py` is running, because the joystick's neutral output never times out.

**Confirm it:**

```bash
ros2 topic echo /ackermann_cmd
```

**This is the problem when:** it's stuck at `0.0 / 0.0` regardless of what your node publishes to `/drive`.

**Fix:** check whether `teleop_launch.py` is running in another terminal, and `Ctrl+C` it.

If you're following [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node) as documented, you won't have it running at all.

`bringup_launch.py` no longer starts `joy_teleop` automatically, so this shouldn't come up unless you launched `teleop_launch.py` deliberately.

---

## Autonomy node publishes to `/drive`, `/ackermann_cmd` looks fine, but it's always `0.0 / 0.0` even with no `teleop_launch.py` running

Different root cause from the one above, and easy to conflate with it.

**First: are you holding LB?** Every autonomy node in this workspace requires it. That's current, mandatory workspace policy ([the rule](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)), not a bug.

**If you are holding it and it's still stuck at zero, check `joy_node` is running:**

```bash
ros2 node list | grep joy
```

**This is the problem when:** nothing comes back.

**Why it matters:** `gap_follow_node` and `pure_pursuit_node` each have their **own** deadman check, separate from `ackermann_mux`. Each subscribes to `/joy` directly and only publishes a non-zero command while LB is held on a *live* `/joy` stream — see `joy_callback` / `_deadman_engaged` in `gap_follow_node.py` or `pure_pursuit_node.py`.

No `/joy` means the node's own deadman can never engage, no matter how hard you hold the button.

`joy_node` lives in `bringup_launch.py`, so it should normally always be up while autonomy runs. If it isn't, either something killed it (see [the `pkill` trap](#pkill--f-joy_teleop-takes-down-joy_node-too-and-cascades-into-killing-the-whole-bringup)) or the bringup terminal itself died.

**The launch terminal tells you this directly.** It reports `STOP [waiting_for_joy]` or `STOP [joy_stale]`, including the configured timeout. Read the autonomy terminal before guessing.

**Fix:** restart `bringup_launch.py` so `joy_node` comes back, and hold LB while your autonomy node is active. See [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node).

> **If it's your own node:** it should have the same check, per [writing-your-own-node.md](writing-your-own-node.md#the-interface-contract). If it doesn't, that's a bug in your node — and a safety-relevant one — not expected behavior.

---

## "slam_toolbox failed to save occupancy map (result code 255)"

**Symptom:** the run completes and races, `posegraph.*` and `raceline_*.csv` are in the output directory, but `map.pgm` and `map.yaml` are not. The log has `[map_saver]: Failed to spin map subscription` just before it.

**Cause: a race, which is why the same run succeeds or fails with nothing else different.**

`SaveMap` runs nav2's `map_saver` inline inside slam_toolbox, and `map_saver` gives up after about two seconds if no `/map` message arrives in that window.

But `/map` is only republished every `map_update_interval` — 5 s in `f1tenth_online_async.yaml`. So whether the save works depends on where in that 5-second cycle the request happens to land.

**What already handles it:** `auto_map_race_node` retries (`map_save_retries`, default 3, spaced by `map_save_retry_delay_sec`), which lands the request in a different part of the window each time.

**If every attempt fails, the run continues anyway and says so.** That's deliberate and you have not lost the run: the racing line is already on disk, and the pose graph usually saved fine. A map can be rebuilt from a pose graph with slam_toolbox's `deserialize_map`.

Shortening `map_update_interval` would also close the race, at the cost of republishing a growing grid more often.

---

## `auto_map_race_launch.py` maps forever and never switches to pure pursuit

**Symptom:** the car drives cautious `gap_follow` laps indefinitely. The supervisor keeps printing `lap 1/2: ...` and the racing phase never starts.

**Read the numbers in that line — each gate says which one is unmet:**

```
lap 1/2: samples=168, distance=27.2/5.0m, turn=341/300deg, elapsed=31.4/15.0s,
departed=yes, start distance=0.43/0.75m, heading error=6.2/30.0deg,
SLAM corrections absorbed=6
```

Each `x/y` is *actual / required*. Find the one that isn't meeting its threshold:

**`start distance` never drops below its limit.** The car isn't returning close enough to where the recorder started.

> That start point is wherever the car was when SLAM's `map->base_link` first appeared — which is a few seconds *into* the run, so it may well be mid-corner.
>
> Fix: raise `closure_distance`, or start the run with the car already sitting still and let SLAM come up before you hold LB.

**`heading error` too large at the moment it passes.** Same cause, different axis. `closure_heading_deg` is the knob.

**`turn` climbing far past 300 without closing.** The car is going round and round without ever satisfying the proximity gates — see `start distance` above. This is *not* a `minimum_lap_turn_deg` problem, and past two laps of turning the log says so outright.

> Measured worst case: a car weaving around two slower cars drove 413 m and ten laps' worth of turning, passing 6.5 m wide of its start every time. `closure_distance` is the knob.

**`SLAM corrections absorbed` in the dozens per lap.** Localisation is struggling. Look at the map in the dashboard before trusting anything downstream — a smeared map moves the start point out from under the closure test.

<details>
<summary><b>The fourth cause, fixed 2026-08-08</b> — historical, but useful if you're on an older checkout or reading old logs.</summary>

Before 2026-08-08 there was a fourth cause, and it was the usual one.

`minimum_lap_distance` defaulted to `20.0` — longer than the ~15 m loop this car is actually driven on. Closure therefore could not fire until the car had been round **twice**, which looked exactly like a tuning problem in the other gates.

It is now `5.0`, with `minimum_lap_turn_deg` doing the real work.

</details>

---

## `auto_map_race_launch.py` refuses to start racing: "Refusing to hand it to pure pursuit"

**Symptom:** the mapping laps complete, then the supervisor logs an error and stays stopped.

**This is the racing line being rejected as unfollowable — deliberately.** The message says by how much.

> **Why refusing is better than trying.** A line the rack cannot steer does not degrade gracefully. It saturates the steering, runs wide, and latches on the emergency stop — which is exactly what used to happen instead.

The message names which check failed:

- **"passes closer to a wall than the car is wide"** — the geometric check. The cleaned line doesn't fit in the corridor, usually because filtering rounded a corner inward on a course whose corners are already near the car's turning circle.
- **"asks for more steering than the car has"** — the kinematic check.

**The two real causes:**

**1. The course has a corner tighter than the car can turn.** `tan(0.26)/0.324` gives a **1.22 m minimum turning radius**. Plot `raceline_raw.csv` (written even on a refusal) over the saved `map.pgm` and look at the tightest corner.

If that's the answer, **the course needs opening out.** No amount of filtering fixes geometry.

**2. The map is smeared, so the recorded pose isn't the path.** Check the map in the dashboard, and the `SLAM corrections absorbed` count in the mapping log.

**The knobs:** `profile_reject_ratio` and `profile_reject_fraction` in `auto_map_race.yaml` control how strict the refusal is.

> **Raising them races a line the car partially cannot steer.** Do that only with the wheels off the ground first, and read [racing-autonomy.md](racing-autonomy.md#what-a-recorded-lap-actually-looks-like) for what those numbers mean.

---

## `slam_toolbox` starts without errors but no map, no `map` frame, and mapping never finishes

A [frame](glossary.md#tf--transform--frame) is a named coordinate system — `map` is the one everything positional is measured against once SLAM is up.

**Symptom:** `slam_launch.py` — or anything including it, such as `autonomous_mapping_launch.py` or `auto_map_race_launch.py` — starts cleanly, and `slam_toolbox` appears in `ros2 node list`. But:

- `ros2 topic echo /map --once` never returns
- `ros2 run tf2_ros tf2_echo map base_link` reports `"map" passed to lookupTransform argument target_frame does not exist`
- `auto_map_race_node` logs `lap 1/N: recorder waiting for a valid map->base_link pose` forever, no matter how many laps you drive

**The giveaway is what `slam_toolbox` *didn't* print.** Its full healthy startup is two lines:

```
[async_slam_toolbox_node-N] [INFO] [slam_toolbox]: Node using stack size 40000000
[async_slam_toolbox_node-N] [INFO] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
```

The first comes from the constructor; the second from `on_configure`. **If you only ever see the first**, the node is parked in the `unconfigured` lifecycle state.

**Confirm it:**

```bash
ros2 lifecycle get /slam_toolbox
```

**Working when:** it says `active [3]`.

**Cause.** As of `slam_toolbox` 2.x, `async_slam_toolbox_node` is an `rclcpp_lifecycle::LifecycleNode` and **does not configure itself**. This car has 2.8.5, from `ros-jazzy-slam-toolbox`.

There is no `autostart` parameter inside the node — so nothing will do it for you.

Everything that matters happens in `on_configure` / `on_activate`:

- reading the parameter file
- loading the solver
- subscribing to `/scan`
- publishing `/map`
- publishing the `map`→`odom` [TF](glossary.md#tf--transform--frame), the coordinate relationship between those two frames
- advertising `/slam_toolbox/save_map`

The [launch file](glossary.md#launch-file) — the script that starts the node — has to emit those transitions itself.

Started as a plain `launch_ros.actions.Node`, it comes up and does nothing at all — silently, forever.

**Everything downstream then waits on it.** The lap recorder never gets a pose, so no raceline is generated, so `pure_pursuit_node` stays on `STOP [waiting_for_profile]` and the run never leaves the cautious gap-follow mapping phase. The visible symptom is several layers away from the cause.

**Fix:** launch it as a `LifecycleNode` and emit `TRANSITION_CONFIGURE`, then `TRANSITION_ACTIVATE` on reaching `inactive`.

That's what `racerbot_launch/launch/slam_launch.py` does now, mirroring upstream's `slam_toolbox/launch/online_async_launch.py`. **If you hit this in a launch file of your own, copy that pattern rather than adding a plain `Node`.**

---

## `pkill -f joy_teleop` takes down `joy_node` too (and cascades into killing the whole bringup)

**You shouldn't need to `pkill` anything** to switch between manual driving and autonomy any more. `Ctrl+C` whichever control-layer terminal is running instead — `teleop_launch.py`, `gap_follow_launch.py`, or `pure_pursuit_launch.py`. See [operations.md](operations.md#the-two-layer-pattern-used-in-every-procedure-below).

> **If that `Ctrl+C` didn't actually clear it, don't reach for `pkill`.** A node wedged in a callback can survive `Ctrl+C` and keep publishing to `/drive`, which is what makes the next run misbehave.
>
> The dashboard's [processes panel](web-dashboard.md#stopping-a-driving-process) lists what is really still running and ends it, escalating past `SIGINT` on its own. It refuses to touch `joy_node`, `joy_teleop` and the rest of the actuation path, so it cannot cause the cascade described below.

But if you reach for `pkill -f joy_teleop` out of habit — from before `teleop_launch.py` existed as its own file — know that it's a trap.

**Why it kills both.** `joy_node` is launched with `--params-file .../joy_teleop.yaml` — the same config file `joy_teleop` uses, just a different top-level key inside it. So **`joy_node`'s own command line contains the literal substring `joy_teleop`**.

`pkill -f` matches against the whole command line, so it kills both processes, not just the one you meant.

**And then it cascades.** Losing `joy_node` kills every autonomy node's deadman input ([see above](#autonomy-node-publishes-to-drive-ackermann_cmd-looks-fine-but-its-always-00--00-even-with-no-teleop_launchpy-running)). Worse, `ros2 launch` treats any process it manages dying as fatal, so it tears down the **entire bringup** — VESC and LiDAR included — a few seconds later.

**If you genuinely must kill one node by pattern**, match something only that process's command line contains:

```bash
pkill -f "__node:=joy_teleop"
```

Or kill it by exact PID.

---

## New terminal, permission denied on `/dev/sensors/vesc` or `/dev/input/js0`

**This isn't a udev or wiring problem.**

Group membership — `dialout` for the VESC, `input` for the joystick — only takes effect in login sessions started *after* the group was added.

**Fix:** open a fresh terminal. Or, in the current one:

```bash
newgrp dialout && newgrp input
```

---

## General debugging approach that worked repeatedly here

Three habits found most of the entries above. They're worth having before you need them.

**1. Check the actual topic data, not just node logs.**

`ros2 topic echo` on the exact topic at each stage of the chain (see the topic table in [architecture.md](architecture.md)) narrows down which link is broken far faster than guessing. A node can log happily while publishing into a void.

**2. Isolate one variable at a time.**

When the servo issue looked hardware-related, three things were ruled out one at a time: power (multimeter under load), connection (wiggle test), and repeated-vs-single commands.

That is what eventually surfaced the real cause. Jumping to "must be a bad servo" would have cost a part and not fixed it.

**3. Verify empirically; don't trust assumed defaults.**

Axis numbering is one example. Here's a sharper one:

> `bringup_launch.py` remaps `ackermann_cmd_out` → `ackermann_drive`, which looks like it should change the output topic name.
>
> Empirically, the [mux](glossary.md#mux--multiplexer) still publishes to `/ackermann_cmd`. The remap doesn't match the [package](glossary.md#package)'s actual internal topic name, so it's a **silent no-op**.

Read the launch file, then check `ros2 node info` and `ros2 topic list` against it before trusting what the source implies.
