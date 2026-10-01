# racerbot_doctor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One read-only command, `ros2 run racerbot_doctor doctor`, that says whether the hardware and the bringup layer are healthy, and for each device names the *first* layer that is broken: not plugged in, vs. kernel driver not bound, vs. permissions, vs. ROS driver not running, vs. ROS driver running but producing no/bad data.

**Architecture:** Three layers, split so the logic is testable without ROS:
1. `system.py` — the only code that touches the OS (`/sys`, `/dev`, `/proc`, `ip`, `ping`, a TCP connect). It fills a plain `Snapshot` dataclass. Every path root is a parameter so tests point it at a fake tree in `tmp_path`.
2. `live.py` — the only code that imports `rclpy`. It subscribes (never publishes) for a short window and fills a plain `LiveSample` dataclass.
3. `checks.py` — pure functions `Snapshot, LiveSample|None -> list[DeviceReport]`. All verdicts live here; all tests target here.

`report.py` renders; `cli.py` wires it together and sets the exit code.

**Tech Stack:** Python 3.12, `ament_python`, rclpy (live stage only), pytest.

**Spec:** this document (the design was agreed in chat on 2026-10-01; summarized under "Design" below).

## Design

### The layer ladder (the core idea)

Each device is a ladder of layers checked bottom-up. A layer is only evaluated if every layer below it passed; above the first failure every layer is reported `SKIP (blocked by: <layer>)`. That is what makes "the USB isn't plugged in" and "the driver didn't load" print differently.

| Device | Layers, bottom → top |
|---|---|
| **VESC** (USB 0483:5740) | USB enumerated → kernel driver `cdc_acm` bound, tty node exists → `/dev/sensors/vesc` symlink points at that tty and is R/W for this user → no *other* process holds the tty → `vesc_driver_node` process running → `/sensors/core` ≥ 25 Hz → `fault_code == 0` → battery voltage (3S) |
| **LiDAR** (Hokuyo UST-10LX, Ethernet) | `enP8p1s0` carrier up → has `192.168.0.15/24` → `192.168.0.10` answers ping → TCP 10940 accepts (**only probed when `urg_node` is NOT running** — never open a second session to a sensor the driver owns) → `urg_node_driver` running → `/scan` ≥ 30 Hz → scan sane (beam count > 0, ≥ 50 % finite) |
| **Joystick** (F710, 046d) | USB present → in XInput mode (pid `c21f`; `c219` = DirectInput → FAIL with "flip the X/D switch to X") → `xpad` bound, `js` node exists and readable → `joy_node` running → `/joy` ≥ 5 Hz |
| **Camera** (RealSense D435i, 8086:0b3a) — **every FAIL capped to WARN** (user decision 2026-10-01: not needed to drive) | USB present → link speed ≥ 5000 Mb/s (else WARN "on USB 2 — move it to a blue USB 3 port / off the USB 2 hub") → `uvcvideo` bound, video nodes exist → `realsense2_camera_node` running (not running = INFO, not part of bringup) → `/camera/camera/color/image_raw` ≥ 8 Hz |
| **Bringup graph** | `ackermann_mux`, `ackermann_to_vesc_node`, `vesc_to_odom_node` running → `/odom` ≥ 25 Hz → static TF `base_link→laser` = `0.26 0.0 0.11` (± 1 mm) → `/teleop` has no publisher (WARN if it does: autonomy's `/drive` is masked) |
| **Environment** | `ROS_DISTRO == jazzy` → workspace overlay sourced (`AMENT_PREFIX_PATH` has `racerbot-ws/install`) → `ROS_DOMAIN_ID` is not `79` (test domain; WARN) → no `racerbot_sim` process (`gym_bridge_node`/`sim_joy_node`) running beside real drivers (FAIL) |

**Statuses:** `PASS`, `WARN`, `FAIL`, `INFO` (fine, but a thing you should know — e.g. "bringup not running"), `SKIP` (blocked by a lower layer, or live stage not run).

**"Bringup not running" is not a failure.** If *none* of the bringup driver processes exist, the process layers are `INFO "not running — start bringup_launch.py to check this layer"` and the live layers `SKIP`. If *some* exist and others don't, the missing ones are `FAIL "bringup is running but <node> is not — it crashed or failed to start; check the bringup terminal"`. That partial case is exactly "driver not loading properly".

**Battery (3S LiPo, user decision 2026-10-01).** `voltage_input` from `/sensors/core`. `< 5.0 V` → FAIL "no battery voltage — VESC on USB only; battery unplugged or switched off". `< 10.2 V` (3.4 V/cell) → FAIL. `< 10.8 V` (3.6 V/cell) → WARN. `> 12.75 V` (4.25 V/cell) → WARN "above 3S full — is a 4S pack fitted?". Always print the raw volts and per-cell average. Only meaningful at rest (sag under load); the doctor never commands motion, so it always is. Not yet cross-checked against a multimeter on this car.

**Exit code:** 0 = no FAIL; 1 = at least one FAIL; 2 = the doctor itself crashed.

### Hard safety constraints (Global)

- Never publishes on any topic, never calls a service, never sets a parameter. Guarded by a static test.
- Never `open()`s a tty, a `/dev/video*` node, or `/dev/input/js*` — access is checked with `os.access` only (opening a tty toggles DTR on some adapters; opening a video node can steal the camera from its driver).
- Never opens a TCP session to the Hokuyo while `urg_node` is running.
- Live stage runs on whatever `ROS_DOMAIN_ID` the shell has (that's the point — it inspects the car's graph). Its *tests* run on domain 79, localhost.

## Global Constraints

- Thresholds: `/sensors/core` ≥ 25 Hz (driver timer is 50 Hz, `vesc_driver.cpp:107`); `/scan` ≥ 30 Hz (UST-10LX = 40 Hz); `/odom` ≥ 25 Hz; `/joy` ≥ 5 Hz (`autorepeat_rate: 20.0`); camera ≥ 8 Hz (profile is 15 fps).
- 3S thresholds: FAIL < 10.2 V, WARN < 10.8 V, WARN > 12.75 V, "no battery" < 5.0 V.
- Laser TF expected `0.26 0.0 0.11`, tolerance 0.001 m (source: `bringup_launch.py`, `docs/hardware-reference.md`).
- Default sample window 3.0 s (`--window`).
- `checks.py`, `system.py`, `report.py`, `snapshot.py` must import without rclpy.

## Review Focus

1. Running the doctor while bringup is up must not disturb it (no tty open, no Hokuyo TCP). → static test in Task 5 + `test_lidar_tcp_not_probed_while_urg_node_runs` in Task 3.
2. The F710 in DirectInput mode must say "flip the switch", not "not plugged in". → `test_joystick_directinput_mode` in Task 3.
3. A VESC with USB but battery off must say "battery", not "driver broken". → `test_vesc_usb_present_no_telemetry_*` and `test_battery_no_voltage` in Task 3.
4. `/dev/sensors/vesc` pointing at a *different* tty than the VESC (stale symlink / another ACM device) must FAIL. → `test_vesc_symlink_points_elsewhere` in Task 3.
5. Unsourced shell must still produce the hardware report, with live layers SKIP and a "source ROS" hint, not a traceback. → `test_cli_without_rclpy` in Task 5.

---

### Task 1: Package skeleton + result types

**Files:**
- Create: `src/racerbot_doctor/{package.xml,setup.py,setup.cfg,resource/racerbot_doctor,racerbot_doctor/__init__.py}`
- Create: `src/racerbot_doctor/racerbot_doctor/snapshot.py`

**Interfaces — Produces:**
```python
class Status(enum.Enum): PASS, WARN, FAIL, INFO, SKIP
@dataclass class Layer: name: str; status: Status; detail: str; fix: str = ''
@dataclass class DeviceReport: device: str; layers: list[Layer]
    def worst(self) -> Status
@dataclass class UsbDevice: sysname: str; vid: str; pid: str; product: str; speed_mbps: float;
    drivers: list[str]; ttys: list[str]; videos: list[str]; joysticks: list[str]
@dataclass class Process: pid: int; argv: list[str]
@dataclass class NetIface: name: str; carrier: bool | None; ipv4: list[str]  # 'a.b.c.d/nn'
@dataclass class Snapshot: usb: list[UsbDevice]; processes: list[Process];
    ifaces: dict[str, NetIface]; vesc_symlink: str | None  # resolved target or None
    vesc_symlink_rw: bool; tty_holders: dict[str, list[int]]  # '/dev/ttyACM0' -> pids
    js_readable: dict[str, bool]; lidar_ping: bool | None; lidar_tcp: bool | None  # None = not probed
    env: dict[str, str]
@dataclass class TopicStats: count: int; window_s: float; last: object | None
    @property hz -> float
@dataclass class LiveSample: topics: dict[str, TopicStats]; teleop_publishers: int;
    laser_tf: tuple[float, float, float] | None
```
Plus a `ladder(device, steps)` helper: `steps` is a list of `(name, fn)` where `fn() -> Layer`; evaluates in order and turns everything after the first FAIL into `SKIP(blocked by …)`. (`INFO` at a process layer also blocks the live layers above it, as SKIP.)

- [ ] Write `test_snapshot.py`: `ladder` stops at the first FAIL and marks the rest SKIP naming the blocker; WARN does not block; INFO blocks; `worst()` ordering FAIL > WARN > INFO/SKIP > PASS; `TopicStats.hz` with window 0 → 0.0 (no ZeroDivisionError).
- [ ] Run, see fail (module missing). Implement. Run, pass. Mutation: make `ladder` not stop on FAIL → a test fails.
- [ ] Commit.

### Task 2: OS probe (`system.py`)

**Files:** Create `racerbot_doctor/system.py`, `test/test_system.py`

**Interfaces — Produces:** `gather(sys_root='/sys', dev_root='/dev', proc_root='/proc', run=subprocess.run, connect=socket.create_connection, probe_lidar_tcp: bool) -> Snapshot`, and the pure parsers it uses: `read_usb_devices(sys_root) -> list[UsbDevice]`, `read_processes(proc_root) -> list[Process]`, `tty_holders(proc_root, ttys) -> dict`, `parse_ip_addr(text) -> dict[str, list[str]]`.

USB walk: for each `/sys/bus/usb/devices/<X>` with `idVendor`: read `idVendor`, `idProduct`, `product`, `speed`; for each interface dir `<X>:<cfg>.<if>`, read the `driver` symlink basename, `tty/*`, `video4linux/*`, `input/*/js*`.

- [ ] Tests build a fake sysfs in `tmp_path` that mirrors the real layout captured on this car on 2026-10-01 (`lsusb -t`: D435i on bus 1 at 480M behind a USB 2 hub, F710 `c21f` with `xpad`), and assert the parsed `UsbDevice` fields; a device with no `driver` link → `drivers == []`; missing `speed` file → `0.0`; fake `/proc/<pid>/cmdline` with NUL-separated argv, an unreadable pid (race: process exited) skipped; `tty_holders` via fake `/proc/<pid>/fd/N -> /dev/ttyACM0` symlinks; `parse_ip_addr` on real `ip -o -4 addr` output.
- [ ] Fail first, implement, pass, mutation-check one parser (e.g. swap vid/pid). Commit.

### Task 3: Device checks (`checks.py`)

**Files:** Create `racerbot_doctor/checks.py`, `test/test_checks.py`

**Interfaces — Produces:** `check_vesc(snap, live) -> DeviceReport`, `check_lidar`, `check_joystick`, `check_camera`, `check_bringup`, `check_environment`, `battery_layer(volts) -> Layer`, `run_all(snap, live) -> list[DeviceReport]`. `live is None` means the live stage didn't run (unsourced or no bringup).

Tests (each pins a deny path; constants trace to the Global Constraints, which cite their source):
- `test_vesc_unplugged` → layer 1 FAIL "not plugged in", rest SKIP.
- `test_vesc_no_kernel_driver` → USB present, `drivers == []` → FAIL at layer 2 mentioning `cdc_acm`.
- `test_vesc_symlink_missing` / `test_vesc_symlink_points_elsewhere` / `test_vesc_symlink_not_rw` (fix hint names `dialout`).
- `test_vesc_tty_held_by_other_process` → FAIL naming the pid; held by `vesc_driver_node` is fine.
- `test_vesc_driver_not_running_bringup_down` → INFO; `..._bringup_partially_up` → FAIL "crashed".
- `test_vesc_usb_present_no_telemetry` → driver running, `/sensors/core` count 0 → FAIL mentioning battery/power.
- `/sensors/core` at 24 Hz FAIL, 25 Hz PASS (boundary); fault_code 2 → FAIL naming `UNDER_VOLTAGE`.
- `battery_layer`: 0.0, 4.99, 5.0, 10.19, 10.2, 10.79, 10.8, 12.75, 12.76, NaN → statuses per thresholds; NaN → FAIL "invalid reading".
- `test_lidar_no_carrier`, `test_lidar_wrong_ip`, `test_lidar_no_ping`, `test_lidar_tcp_refused_driver_down`, `test_lidar_tcp_not_probed_while_urg_node_runs` (snapshot `lidar_tcp=None` + urg running → that layer is INFO "skipped: driver owns the sensor", not FAIL), `/scan` 29 vs 30 Hz, scan all-inf → FAIL, empty ranges → FAIL.
- `test_joystick_missing`, `test_joystick_directinput_mode` (pid c219 → FAIL with "X/D switch"), `test_joystick_no_js_node`, `/joy` rate.
- `test_camera_missing_is_warn_not_fail`, `test_camera_usb2_warns`, `test_camera_not_launched_is_info`, `test_camera_no_frames_is_warn`.
- `test_bringup_laser_tf_wrong` (0.33 → FAIL), `test_bringup_tf_within_1mm`, `test_teleop_publisher_warns`.
- `test_env_not_sourced`, `test_env_test_domain_warns`, `test_env_sim_running_fails`.
- [ ] Each test seen failing against a stubbed check (return all-PASS) before implementation; boundary tests catch a flipped `<`/`<=`.
- [ ] Commit.

### Task 4: Live sampler (`live.py`)

**Files:** Create `racerbot_doctor/live.py`, `test/test_live.py` (rclpy; run sourced on domain 79)

**Interfaces — Produces:** `sample(window_s: float) -> LiveSample`. Subscribes with sensor-data QoS (best effort, compatible with reliable publishers) to `/sensors/core`, `/scan`, `/odom`, `/joy`, `/camera/camera/color/image_raw`; `/tf_static` with transient-local; counts messages, keeps last; `teleop_publishers = len(node.get_publishers_info_by_topic('/teleop'))`; laser TF from `/tf_static` where `header.frame_id=='base_link'` and `child_frame_id=='laser'`. Waits ~1 s for discovery before the window starts. Creates **no publishers**.

- [ ] Test (domain 79): a helper node publishes `/scan` at 20 Hz and a `base_link→laser` static TF; `sample(2.0)` reports 15–25 Hz on `/scan`, 0 on `/sensors/core`, the TF triple, and `teleop_publishers == 0`; then with a `/teleop` publisher → 1. Seen failing first.
- [ ] Commit.

### Task 5: Report, CLI, docs

**Files:** Create `racerbot_doctor/report.py`, `racerbot_doctor/cli.py`, `test/test_cli.py`, `README.md`; modify `docs/troubleshooting.md` (top: "Run the doctor first"), `docs/operations.md` (pre-drive check), `README.md` + `CLAUDE.md` package table.

`cli.main(argv)`: `--window`, `--no-live`, `--no-color`. Gathers snapshot (probing Hokuyo TCP only if no `urg_node_driver` process); runs live stage only if rclpy imports **and** at least one bringup/camera driver process exists; prints; exits 0/1/2.

- [ ] `test_exit_code_fail` / `test_exit_code_pass` with an injected snapshot; `test_cli_without_rclpy` (monkeypatch import to fail → report printed, live layers SKIP with "source ROS", exit not 2); `test_no_publishers_in_package` — static scan of the package source asserting no `create_publisher`, `create_client`, `call_async`, `set_parameters`, and no `open(` on `/dev` paths.
- [ ] Build, run for real on the car (bringup down: expect VESC FAIL "not plugged in" as of 2026-10-01, camera WARN USB 2, joystick PASS up to the process layer, LiDAR PASS through TCP).
- [ ] Commit.
