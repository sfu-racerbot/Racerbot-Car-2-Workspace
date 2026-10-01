# racerbot_doctor: one command that tells you what's broken

> **Who this is for:** anyone about to drive the car, or whose car isn't working and who doesn't know where to start.
> **Read first:** [operations.md — Every session, before anything else](../../docs/operations.md#every-session-before-anything-else) (sourcing ROS).
> **You'll be able to:** check the VESC, LiDAR, joystick and camera in one go, and get told which exact thing is wrong and how to fix it.
> **Time:** 5 seconds to run.

`racerbot_doctor` is a self-check for the car. Run it before a session, or the moment something doesn't work. It answers "is it plugged in?", "did the driver load?" and "is data actually coming out?" for every device, in that order.

## Highlights

- **One command, every device:** VESC (motor controller), Hokuyo LiDAR, F710 gamepad, RealSense camera, the bringup nodes, and your terminal's ROS setup.
- **Tells "not plugged in" apart from "driver didn't load".** Each device is checked layer by layer: USB or Ethernet link → Linux kernel driver → device file and permissions → ROS driver process → message rate → message content. The report stops at the **first** broken layer, so you know where to look.
- **Read-only, so it's safe to run any time, even with bringup up.** It never publishes on a topic, never opens the VESC's serial port or the camera, and doesn't connect to the LiDAR while `urg_node` is using it. A test scans the package source and fails if any of that is ever added.
- **Every failure comes with a fix line**: the actual command or physical action, e.g. `nmcli connection up hokuyo` or "flip the X/D switch on the back to X".
- **Reads the battery** (3S LiPo) from the VESC's own voltage measurement, and tells "battery off" (VESC alive on USB power only) apart from "battery low".
- **Works before bringup, too.** With nothing running it checks all the hardware layers and reports the ROS layers as "not running", which is not a failure.
- **Exit code** 0 = nothing failed, 1 = something failed, 2 = the doctor itself crashed, so scripts can use it.
- **136 tests**, written failing-first and checked by deliberately breaking each verdict (flipped comparisons, deleted checks) to confirm a test catches it, per [TEST_QUALITY_STANDARDS.md](../../TEST_QUALITY_STANDARDS.md).

**Why it exists:** most entries in [troubleshooting.md](../../docs/troubleshooting.md) are an hour spent finding something a ten-line check would have caught. A gamepad switched to the wrong mode, a missing device link, a VESC with its battery off. This tool runs all of those checks at once.

## Running it

First [source](../../docs/glossary.md#sourcing) ROS and this workspace (the two `source` lines every terminal needs; see [operations.md](../../docs/operations.md#every-session-before-anything-else)).

**Terminal 1** — any terminal will do; the doctor runs and exits.

```bash
ros2 run racerbot_doctor doctor
```

**Working when:** the last line reads `0 FAIL, ...`. A `WARN` is worth reading but won't stop you driving.

**If it doesn't:** read the `FAIL` lines. Each has a `fix:` line under it. The summary at the bottom names the first broken layer of each device.

To check the ROS side as well (topic rates, the battery, VESC faults), start bringup first in another terminal, then run the doctor:

**Terminal 1** — the hardware drivers. Leave it running.

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** log lines stop scrolling after a few seconds. A `vesc_driver_node` `[FATAL]` here means the VESC isn't reachable, and the doctor will say why.

**Terminal 2** — the doctor.

```bash
ros2 run racerbot_doctor doctor
```

**Working when:** the VESC, LiDAR and Joystick sections end in `PASS` lines for their `rate` layers, and the VESC shows a battery voltage.

Options:

| Flag | What it does |
|---|---|
| `--window 5` | Sample each topic for 5 s instead of 3 (more stable rate numbers) |
| `--no-live` | Hardware and OS checks only; never joins the ROS graph |
| `--no-color` | Plain text, e.g. for pasting into a chat |

## Reading the report

Each line has a status:

| Status | Meaning |
|---|---|
| `PASS` | This layer is fine. |
| `FAIL` | This layer is broken. Everything above it is `SKIP`ped, because checking `/scan` when the cable is out tells you nothing new. |
| `WARN` | Works, but not as it should (low battery, camera on USB 2). Doesn't fail the run. |
| `INFO` | Nothing wrong, just worth knowing: usually "bringup isn't running, so I stopped here". |
| `SKIP` | Not checked, and the line says why. |

Here's an example. It was captured on 2026-10-01 with bringup running and the VESC unplugged:

```
VESC
  FAIL  USB device                           not plugged in (no 0483:5740 on USB)
                                             fix: plug in the VESC USB cable; `lsusb` should list 0483:5740
  SKIP  kernel driver .. battery (3S)  (7 layers) blocked by: USB device

LiDAR
  PASS  Ethernet link                        enP8p1s0 carrier up
  PASS  host IP                              192.168.0.15/24
  PASS  ping                                 192.168.0.10 answers
  SKIP  TCP 10940                            not probed: urg_node owns the sensor
  PASS  ROS driver urg_node_driver           running (pid 554263)
  PASS  /scan rate                           40.0 Hz
  PASS  /scan content                        1081 beams, 100% with a return
```

The camera never fails the run (team decision, 2026-10-01): the car drives without it, so camera problems show as `WARN`.

## What it checks, device by device

<details>
<summary>The full layer list and thresholds. Skip this unless you're changing the doctor or a threshold.</summary>

All numbers live in [`racerbot_doctor/car.py`](racerbot_doctor/car.py), each one next to a note saying where it comes from.

| Device | Layers, bottom → top |
|---|---|
| **VESC** | USB `0483:5740` present → `cdc_acm` kernel driver made a `/dev/ttyACM*` → `/dev/sensors/vesc` points at it and you can read/write it (`dialout` group) → no other program has the port open → `vesc_driver_node` running → `/sensors/core` ≥ 25 Hz → `fault_code` 0 → battery |
| **LiDAR** (Ethernet) | `enP8p1s0` has a link → it has `192.168.0.15/24` → `192.168.0.10` answers ping → TCP port 10940 accepts (only probed while `urg_node` is *not* running) → `urg_node_driver` running → `/scan` ≥ 30 Hz → at least one beam has a return (`WARN` under 50 %) |
| **Joystick** | F710 receiver present **in XInput mode** (`c21f`; DirectInput `c219` fails with "flip the switch") → `xpad` made `/dev/input/js*` → readable (`input` group) → `joy_node` running → `/joy` ≥ 5 Hz |
| **Camera** | D435i `8086:0b3a` present → USB 3 speed (`WARN` at USB 2) → `uvcvideo` made `/dev/video*` → `realsense2_camera_node` running (`INFO` if not: it's optional) → color image ≥ 8 Hz |
| **Bringup** | `ackermann_mux`, `ackermann_to_vesc_node`, `vesc_to_odom_node` running → `/odom` ≥ 25 Hz → `base_link→laser` transform is `0.26 0 0.11` (±1 mm) → `INFO` if something publishes `/teleop` (that blocks autonomy's `/drive`) |
| **Environment** | `ROS_DISTRO=jazzy` → workspace sourced → `ROS_DOMAIN_ID` isn't the test domain 79 → `racerbot_sim` not running (`FAIL` if it's running beside the real drivers) |

**"Bringup not running" vs "a driver died".** If *no* bringup driver is running, the ROS-driver layer says `INFO`: nothing has been started, so nothing is wrong. If *some* are running and one is missing, that one is a `FAIL` ("it crashed or failed to start"). That's how a driver that didn't load is told apart from bringup simply not being up.

**Battery thresholds (3S LiPo).** The VESC measures its own input voltage, which is the pack voltage. Its driver publishes that as `voltage_input` on `/sensors/core`.

| Reading | Verdict |
|---|---|
| under 5.0 V | `FAIL`: no battery. The VESC is running on USB power only |
| under 10.2 V (3.4 V/cell) | `FAIL`: critically low |
| under 10.8 V (3.6 V/cell) | `WARN`: low |
| over 12.75 V (4.25 V/cell) | `WARN`: more than a full 3S pack. Is a 4S pack fitted? |

Limits: the per-cell figure is the total divided by 3. The VESC can't see individual cells, so one bad cell can hide behind a healthy total. The reading hasn't yet been checked against a multimeter on this car.

**Known blind spots.**
- It can't tell whether the F710 *controller* is switched on: the USB receiver looks the same either way. Press a button and watch `ros2 topic echo /joy` if in doubt.
- It can't see other users' processes holding the VESC port without root.

</details>

## How it's built

<details>
<summary>For anyone changing the code. Skip otherwise.</summary>

| File | Job |
|---|---|
| `racerbot_doctor/system.py` | The only code that touches the OS: walks `/sys/bus/usb/devices`, `/proc`, `/dev`, runs `ip` and `ping`. Every root path is a parameter so tests use a fake tree. |
| `racerbot_doctor/live.py` | The only code that imports `rclpy`. Subscribes for a few seconds and counts messages. Creates no publishers. |
| `racerbot_doctor/checks.py` | All the verdicts. Pure functions, no I/O. |
| `racerbot_doctor/snapshot.py` | The plain data passed between them, and the layer ladder. |
| `racerbot_doctor/report.py`, `cli.py` | Printing and the command line. |
| `racerbot_doctor/car.py` | Every threshold and hardware ID, each with its source. |

Running the tests: everything except `test_live.py` runs with no ROS at all. `test_live.py` needs ROS sourced and the isolated test domain:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m pytest src/racerbot_doctor/test/ -v
```

**Working when:** `136 passed`. Unsourced, `test_live.py` fails to collect. That's deliberate and loud, so don't add `--continue-on-collection-errors`.

The plan this was built from: [docs/superpowers/plans/2026-10-01-racerbot-doctor.md](../../docs/superpowers/plans/2026-10-01-racerbot-doctor.md).

</details>

## See also

- [troubleshooting.md](../../docs/troubleshooting.md): the long-form fixes the doctor's `fix:` lines point towards.
- [hardware-reference.md](../../docs/hardware-reference.md): where the LiDAR offset and the other measured numbers come from.
