# Wiimote driver

> **Who this is for:** someone reading or changing the Wiimote driver's code.
> **Read first:** [docs/wiimote.md](../../docs/wiimote.md) for connecting and driving with a Wiimote.
> **What's in it:** what each package and file does, and how to test it.

Vendored from [sfu-racerbot/ros2_wiimote_driver](https://github.com/sfu-racerbot/ros2_wiimote_driver)
(MIT, see `LICENSE`) on 2026-10-11 and fixed here. This copy is the one the
car builds; it is plain files, not a submodule.

## Highlights

- `racerbot_wiimote_msgs`: buttons, battery, accelerometer, LED and rumble messages.
- `racerbot_wiimote_driver`: `wiimote_driver_node` publishes `sensor_msgs/Joy` on `/wiimote/joy`, raw `/wiimote/buttons`, `/wiimote/accel`, `/wiimote/battery`, and takes `/wiimote/led` and `/wiimote/rumble`.
- `wiimote_teleop_launch.py`: the Wiimote as a control layer (driver + `joy_teleop` → `/teleop`).
- `wiimote_connect`: Bluetooth pair/trust/connect, status check, and an auto-reconnect service.
- It never publishes a drive command itself, and never on `/joy`.

## What was fixed in this copy

| Problem in the original | Effect | Fix |
|---|---|---|
| `CMakeLists.txt` used `ament_target_dependencies()` after keyword `target_link_libraries()` on ament_cmake < 2.7.3 | **never built on Jazzy** (CMake error) | keyword form only |
| one exception (LED permission `-13`, battery read) aborted the whole 200 Hz tick | no `/joy` at all | each output isolated; LED written only on change; battery every 5 s |
| no Wiimote at start | node exited | waits and reconnects every second |
| Wiimote disconnect not detected | last buttons, including the deadman, stayed "held" | disconnect, link stall (0.5 s silence) and no-Wiimote all publish "nothing held" |
| A/B passed through with 2 released | `joy_teleop` went silent instead of commanding a stop | every button and axis zero unless 2 is held |
| `axes.resize(2)` then indexed the configured axis | crash for an axis index above 1 | sized from the largest index |
| published on `/joy` | a Wiimote could act as the F710's LB deadman for autonomy | `/wiimote/joy` |
| one device opened twice, leaked on error paths | | one `xwii_iface`, RAII |

## Files

| File | What it does |
|---|---|
| `racerbot_wiimote_driver/src/joy_mapping.cpp` | Wiimote state → Joy (deadman masking, throttle, tilt filter). No ROS, no xwiimote. |
| `racerbot_wiimote_driver/src/wiimote_device.cpp` | xwiimote wrapper: open, read events, detect disconnect, LEDs, battery, rumble. |
| `racerbot_wiimote_driver/src/wiimote_driver_node.cpp` | The ROS node: connect/reconnect, publish, stall detection. |
| `racerbot_wiimote_driver/config/wiimote_driver.yaml` | Joy layout, tilt calibration, timeouts. |
| `racerbot_wiimote_driver/config/wiimote_teleop.yaml` | `joy_teleop` profile: speed and steering scale. |
| `racerbot_wiimote_driver/scripts/wiimote_connect` | Bluetooth helper (bash). |

## Tests

```bash
colcon test --packages-select racerbot_wiimote_driver && colcon test-result --verbose
```

That runs two tests:

- `test_joy_mapping` (gtest): the deadman masking, throttle, tilt filter and layout checks.
- `test_wiimote_connect` (pytest): the Bluetooth helper against a fake `bluetoothctl`. It also runs unsourced with `python3 -m pytest src/racerbot_wiimote/racerbot_wiimote_driver/test/`.

`tools/wiimote_e2e/run_wiimote_e2e.py` runs the real node with no Wiimote and
the real `joy_teleop` with `wiimote_teleop.yaml`, and checks what reaches
`/teleop`. None of these need a Wiimote; driving one still needs the
wheels-up check in [docs/wiimote.md](../../docs/wiimote.md).
