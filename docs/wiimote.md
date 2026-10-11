# Driving with a Wiimote

> **Who this is for:** anyone who wants to drive the car by hand with a Nintendo Wiimote (optionally in a Wii Wheel) instead of the F710 gamepad.
> **Read first:** [operations.md](operations.md#manual-driving-teleop) — you need to be able to bring the car up and drive it with the F710 first.
> **You'll be able to:** connect a Wiimote over Bluetooth once, have it reconnect by itself, and drive the car with it.
> **Time:** about 20 minutes the first time, then seconds.

## Highlights

- Hold **2** (the deadman, see [glossary.md](glossary.md#deadman)), hold **A** to drive forward at 1.0 m/s or **B** to reverse, and tilt to steer. Let go of **2** and the car is told to stop.
- Pair it once with the red **SYNC** button. After that, pressing any button on the Wiimote reconnects it; nothing to type.
- `ros2 run racerbot_wiimote_driver wiimote_connect` checks every layer that has to work (Bluetooth service, adapter, kernel driver, xwiimote, permissions) and says exactly what to fix.
- The driver waits for the Wiimote instead of crashing, and reconnects by itself.
- If the Wiimote disconnects, goes out of range, or sends nothing for 0.5 s, the driver reports "nothing held", so the car is told to stop.
- It publishes on `/wiimote/joy`, never `/joy`, so a Wiimote **cannot** stand in for the F710's LB in `gap_follow` or `pure_pursuit`.
- Limits: tilt steering is coarser than a stick, and forward speed is one fixed value (1.0 m/s; change it in `wiimote_teleop.yaml`).

## Why it exists

The F710 is the car's only remote. A Wiimote in a Wii Wheel is a cheap,
familiar second controller for demos and for people learning to drive the
car. It is a manual-driving controller only; autonomy still needs the F710.

## Before you start

- [ ] The car's normal bringup works (see [operations.md](operations.md)).
- [ ] The Wiimote has fresh AA batteries.
- [ ] You can open the Wiimote's battery cover (the red SYNC button is inside).

## One-time setup on the car

**Terminal 1** — install the Wiimote libraries, then build.

```bash
sudo apt update
sudo apt install -y bluez xwiimote libxwiimote-dev pkg-config
cd ~/racerbot-ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select racerbot_wiimote_msgs racerbot_wiimote_driver
source install/setup.bash
```

**Working when:** `colcon` ends with `2 packages finished` and no `Failed`.

**If it doesn't:**

- `Package 'libxwiimote' not found`: the `apt install` line did not finish;
  run it again.
- `CMake Error: The source ... does not match the source ... used to generate
  cache`: an old build of the driver from its previous folder
  (`src/racerbot_wiimote_driver`) is in the way. Move that old folder out of
  `src/` if it is still there, delete the stale build, and build again:

  ```bash
  mv ~/racerbot-ws/src/racerbot_wiimote_driver ~/old_wiimote_clone 2>/dev/null
  rm -rf ~/racerbot-ws/build/racerbot_wiimote_* ~/racerbot-ws/install/racerbot_wiimote_*
  ```

Let your account read the Wiimote's input device. Log out and back in
afterwards; group changes only apply to new logins.

```bash
sudo usermod -aG input $USER
```

Then let your account set the Wiimote's LEDs (without this the driver still
drives, but logs LED error `-13`):

```bash
sudo tee /etc/udev/rules.d/99-wiimote-leds.rules <<'EOF'
ACTION=="add", SUBSYSTEM=="leds", KERNEL=="*:blue:p[0-3]", RUN+="/bin/chmod 666 /sys/class/leds/%k/brightness"
EOF
sudo udevadm control --reload-rules
```

**Working when:** after the Wiimote next connects, `ls -l /sys/class/leds/*:blue:p0/brightness`
shows `-rw-rw-rw-`.

## Connect the Wiimote (first time)

**Terminal 1**:

```bash
source ~/racerbot-ws/install/setup.bash
ros2 run racerbot_wiimote_driver wiimote_connect
```

It checks the computer first and prints `[ ok ]` or `[FAIL]` for each layer:

```text
  [ ok ] Bluetooth service running
  [ ok ] Bluetooth adapter powered on
  [ ok ] hid-wiimote kernel driver loaded
  [ ok ] xwiimote installed
  [ ok ] user racerbot is in the input group
```

Fix every `[FAIL]` line first; each one says how. Then follow its prompt:
open the battery cover and press the small red **SYNC** button once. The four
blue LEDs flash while it searches.

**Working when:** it prints `Wiimote <address> connected`. The LEDs keep
flashing until the driver starts; that is normal.

**If it doesn't:**

- `No Wiimote found`: the LEDs had stopped flashing before the search
  ended. Press SYNC again and rerun.
- `hid-wiimote kernel driver missing`: the Jetson kernel leaves this driver
  out. Build it once; see [Build the kernel driver](#build-the-kernel-driver).

Use SYNC, not 1+2. A Wiimote paired with 1+2 works, but it forgets the car
when it turns off and has to be paired again every time.

## Every time after that: auto-connect

A SYNC-paired, trusted Wiimote reconnects by itself: **press any button** on
it. The car cannot wake a sleeping Wiimote, so the press is always needed.

To also keep reconnecting a Wiimote that was woken with 1+2, install the
background service once.

**Terminal 1**:

```bash
ros2 run racerbot_wiimote_driver wiimote_connect --install-service
```

**Working when:** `systemctl status wiimote-autoconnect` shows
`active (running)`. It only ever connects Wiimotes this car has trusted,
never another team's.

`wiimote_connect --check` prints the status of every layer and every known
Wiimote without changing anything.

## Drive

Driving takes two terminals, exactly like F710 driving with
`teleop_launch.py`. Terminal 1 is bringup (the car's hardware). Terminal 2
is the Wiimote's control layer: the driver plus `joy_teleop`. That turns
button presses into drive commands on the `/teleop` topic. Teleop means
hand-driving; a topic is a named message stream, and sending on one is
called publishing (see [glossary.md](glossary.md)).

Run it **instead of** `teleop_launch.py`, never beside it. Both send
(publish) commands on `/teleop`, and the F710's idle zeros would fight the
Wiimote's commands.

**Terminal 1** — the car's hardware. Leave it running.

```bash
source ~/racerbot-ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

**Working when:** it keeps running with no red `[ERROR]` lines (see
[operations.md](operations.md#manual-driving-teleop)).

**Terminal 2** — the Wiimote and its teleop profile.

```bash
source ~/racerbot-ws/install/setup.bash
ros2 launch racerbot_wiimote_driver wiimote_teleop_launch.py
```

**Working when:** terminal 2 prints `Wiimote connected` and the Wiimote's
first LED goes solid. If it prints `No Wiimote connected`, press a button on
the Wiimote; the driver picks it up within a second.

| Wiimote (held sideways, buttons up) | Car |
|---|---|
| hold **2** | deadman; LED 2 lights while held |
| **2** + **A** | forward at 1.0 m/s |
| **2** + **B** | reverse at 1.0 m/s |
| **2** + tilt | steer, up to 0.34 rad (the F710's full stick) |
| let go of **2** | commanded stop |

Check the steering direction **with the wheels off the ground first**: hold
2 and tilt left. If the wheels turn right, set
`invert_accelerometer_y: true` in
`src/racerbot_wiimote/racerbot_wiimote_driver/config/wiimote_driver.yaml`
and restart terminal 2.

## Tuning the tilt

`ros2 topic echo /wiimote/accel` shows the raw accelerometer. Hold the
Wiimote level, then fully left, then fully right, and put the three `y`
values into `accelerometer_y_center`, `accelerometer_y_min` and
`accelerometer_y_max` in `wiimote_driver.yaml`. The forward speed and the
steering range are `scale` under `human_control` in `wiimote_teleop.yaml`.

<details>
<summary>How the stop is guaranteed (safe to skip)</summary>

`joy_teleop` only publishes for a profile whose buttons are held. Its
`default` profile, which publishes a zero command, is active only when no
button at all is held. If the driver passed "A held, 2 released" straight
through, no profile would match and `joy_teleop` would go silent, leaving the
last drive command in place until `ackermann_mux` times out.

So the driver zeroes every button and axis unless 2 is held (`build_joy()` in
`joy_mapping.cpp`). Letting go of 2 always lands on the `default` profile
and an active zero.

The same zero frame is published whenever no Wiimote is
connected, the link drops, or no event arrives for `input_timeout_s` (0.5 s).
A Wiimote in a hand reports its accelerometer continuously, so silence means
the radio link has stalled. Linux can take seconds to notice that itself.

`tools/wiimote_e2e/run_wiimote_e2e.py` checks all of this against the real
`joy_teleop` with this profile.

</details>

## Build the kernel driver

> **Skip this unless** `wiimote_connect --check` says `hid-wiimote kernel driver missing`.

NVIDIA's Jetson kernel does not include `hid-wiimote`. The Wiimote then
connects over Bluetooth but its LEDs never stop flashing and the driver never
finds it. Build the module from the matching upstream kernel source:

```bash
sudo apt install -y build-essential linux-headers-$(uname -r) wget
mkdir -p ~/hid-wiimote-build && cd ~/hid-wiimote-build
KERNEL_TAG="v$(uname -r | cut -d'.' -f1,2)"
for f in hid-wiimote.h hid-wiimote-core.c hid-wiimote-debug.c hid-wiimote-modules.c hid-ids.h; do
  wget "https://raw.githubusercontent.com/torvalds/linux/${KERNEL_TAG}/drivers/hid/$f"
done
```

Open each downloaded file and check it is C source, not a 404 page. Then
create the Makefile, build, and load it at every boot:

```bash
cat > Makefile << 'EOF'
obj-m := hid-wiimote.o
hid-wiimote-y := hid-wiimote-core.o hid-wiimote-debug.o hid-wiimote-modules.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
ccflags-y := -I$(PWD)
default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
EOF
make
sudo mkdir -p /lib/modules/$(uname -r)/updates
sudo cp hid-wiimote.ko /lib/modules/$(uname -r)/updates/
sudo depmod -a
echo hid-wiimote | sudo tee /etc/modules-load.d/hid-wiimote.conf
sudo modprobe hid-wiimote
```

**Working when:** `lsmod | grep wiimote` lists `hid_wiimote`. A kernel
update needs this repeated.

## Safety

- Every powered movement needs **2** held and a spotter watching the car.
- Check steering direction with the wheels off the ground before the first
  drive, and after changing any tilt setting.
- The Wiimote is a manual controller only. It publishes `/wiimote/joy`, which
  no autonomy node reads, so it can never enable `gap_follow` or
  `pure_pursuit`. Those still need LB held on the F710
  ([architecture.md](architecture.md)).
- One control layer at a time: `wiimote_teleop_launch.py` or
  `teleop_launch.py`, never both.
