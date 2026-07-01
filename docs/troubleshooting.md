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

If you inject a raw `ros2 topic pub` command into `/commands/servo/position` while the **full bringup is still running**, you'll likely see inconsistent, twitchy behavior that looks like a hardware fault but isn't.

Cause: `joy_teleop`'s `default` profile has no deadman-button restriction and continuously republishes a neutral command as a safety fail-safe, flowing through `ackermann_mux` → `ackermann_to_vesc_node` → the exact same `/commands/servo/position` topic you're injecting into. Two publishers end up racing on one topic, and whichever message arrived most recently wins — that interleaving is the "twitching," not a real fault.

This was diagnosed by systematically eliminating variables: single commands vs. repeated/continuous commands, checked power under load (multimeter, stayed rock-steady — ruled out a supply issue), and a wiggle test on the connector (inconclusive — twitching persisted independent of touching the cable, which argued against a loose connection). What actually resolved it was checking `/ackermann_cmd` and realizing it never reflected the injected value at all — it stayed at the joystick's neutral output the whole time.

**Fix / how to avoid it:** either stop `ackermann_to_vesc_node` before injecting raw test commands, or just trust a real controller test over raw topic injection — the actual controller worked the entire time this "issue" was being chased.

## Autonomy node publishes to `/drive`, car doesn't move, no errors anywhere

This is expected behavior, not a bug — see [architecture.md](architecture.md#the-safety-model-read-this-before-writing-autonomy-code). `/teleop` (joystick, priority 100) permanently masks `/drive` (navigation, priority 10) in `ackermann_mux` as long as `joy_node` + `joy_teleop` are both running, because the joystick's neutral output never times out. Confirm with:
```bash
ros2 topic echo /ackermann_cmd
```
If it's stuck at `0.0 / 0.0` regardless of what your node publishes to `/drive`, this is why. Fix: follow the procedure in [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node) — stop `joy_teleop` before launching autonomy (leave `joy_node` running — see the next entry), and restart `joy_teleop` afterward.

## Autonomy node publishes to `/drive`, `/ackermann_cmd` looks fine, but it's always `0.0 / 0.0` even with `joy_teleop` stopped

Different root cause from the one above, easy to conflate. First, are you holding LB? Every autonomy node in this workspace requires it — this is current, mandatory workspace policy (see [architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)), not a bug. If you are holding it and it's still stuck at zero, check whether `joy_node` is even still running:
```bash
ros2 node list | grep joy
```
If it isn't, this is why: `gap_follow_node` and `pure_pursuit_node` both have their **own** deadman-button check, separate from `ackermann_mux` — each subscribes to `/joy` directly and only publishes a non-zero command while LB is held on a *live* `/joy` stream (see `gap_follow_node.py`'s or `pure_pursuit_node.py`'s `joy_callback`/`_deadman_engaged`). Killing `joy_node` along with `joy_teleop` removes `/joy` entirely, so the node's own deadman check can never engage — it silently publishes `0.0/0.0` forever, with no error anywhere. A custom node should have the same check per [writing-your-own-node.md](writing-your-own-node.md#the-interface-contract) — if it doesn't, that's a bug in that node, not expected behavior.

**Fix:** only stop `joy_teleop`, leave `joy_node` running, and hold LB while your autonomy node is active. See [operations.md](operations.md#running-autonomy-gap_follow-pure_pursuit-or-your-own-node).

## New terminal, permission denied on `/dev/sensors/vesc` or `/dev/input/js0`

Group membership (`dialout` for the VESC, `input` for the joystick) only takes effect in login sessions started *after* the group was added. This isn't a udev or wiring problem — open a fresh terminal, or run `newgrp dialout && newgrp input` in the current one.

## General debugging approach that worked repeatedly here

1. **Check the actual topic data, not just node logs.** `ros2 topic echo` on the exact topic at each stage of the chain (see the topic table in [architecture.md](architecture.md)) narrows down which link is broken far faster than guessing.
2. **Isolate one variable at a time.** When the servo issue looked hardware-related, systematically ruling out power (multimeter under load), connection (wiggle test), and repeated-vs-single commands one at a time is what eventually surfaced the real cause (competing publishers), rather than jumping to "must be a bad servo."
3. **Verify empirically, don't trust assumed defaults** — axis numbering, topic names implied by a launch file's remap arguments (`ackermann_cmd_out` → `ackermann_drive` in `bringup_launch.py` looks like it should change the output topic name, but empirically the mux still publishes to `/ackermann_cmd` — the remap doesn't match the package's actual internal topic name, so it's a silent no-op). Read the launch file, then check `ros2 node info` / `ros2 topic list` against it before trusting what the source implies.
