# Concepts: ROS2, colcon, and this workspace

> **Who this is for:** anyone new to ROS2, colcon, or this workspace. If `source`, `colcon build` or `ros2 launch` are unfamiliar, start here.
> **Read first:** [glossary.md](glossary.md) — skim it, then come back.
> **You'll be able to:** explain what each command actually does, why every terminal needs sourcing, and what each top-level folder holds.
> **Time:** about 20 minutes.

This doc answers the questions that are embarrassing to ask out loud: what actually *is* a workspace, why do I have to type the same two commands every single time, and what is all this stuff doing.

It's the difference between copying commands out of the README and knowing what they did. Everything else in `docs/` assumes you know what's here. If you've used ROS2 before, skip to [What each top-level folder is for](#what-each-top-level-folder-is-for).

---

## First: what is ROS2?

ROS2 is not an operating system, despite the name (Robot Operating System). It's a **way for many small programs to talk to each other**, plus the tooling to build and start them.

The idea is that a robot is too complicated to be one program. So instead you write a lot of little programs, each doing one job:

- one that reads the laser scanner and reports distances
- one that decides where to steer
- one that talks to the motor controller

Each of those little programs is called a **node**. Nodes don't call each other's functions. They don't share memory. They send **messages** to each other over named channels called **topics**.

### The one mental model you need

A topic works like a radio channel:

- A node that wants to share information **publishes** to a topic. It doesn't know or care who's listening — possibly nobody.
- A node that wants that information **subscribes** to the topic. It doesn't know or care who's sending.

For example, take the [LiDAR](glossary.md#lidar): the spinning laser scanner on the front of the car, which measures how far away everything around it is.

Its driver publishes those readings to a topic named `/scan`. The `gap_follow` node subscribes to `/scan`, decides where to steer, and publishes a steering-and-speed message to a topic named `/drive`. Something else subscribes to `/drive` and moves the motors.

```
  urg_node  ──publishes──▶  /scan  ──▶  gap_follow  ──publishes──▶  /drive  ──▶  ...
 (the LiDAR)                              (the brain)
```

Nothing in that chain knows what the others are. `gap_follow` just needs *some* node to put laser data on `/scan`. That's the whole point, and it buys three real things:

1. **You can replace one piece without touching the others.** This is why the simulator works: `racerbot_sim` publishes fake `/scan` and `/odom` messages, and the driving code above it can't tell the difference. See [ros-simulator.md](ros-simulator.md).
2. **You can watch anything from outside.** Since messages go over named channels, you can point a tool at any topic and print what's flowing through it, live, without modifying a single line of code. That's how most debugging on this car happens.
3. **A crash stays local.** One node dying doesn't take the rest down with it.

The price is that the system's behavior isn't visible in any one file. To understand what's running you have to know the graph of nodes and topics — which is exactly what [architecture.md](architecture.md) documents, and why it's required reading before you write driving code.

### The other three words you'll hit immediately

**Message** — the actual data sent on a topic, in a fixed format both sides agree on ahead of time. `/scan` carries a `LaserScan`, which has fields like `ranges` (an array of distances) and `angle_min`. You don't invent these formats; you use standard ones (`sensor_msgs`, `nav_msgs`, `ackermann_msgs`) so unrelated packages interoperate.

**Parameter** — a named setting a node reads when it starts, so you can change behavior without editing code. `gap_follow` has a `max_speed` parameter.

Parameters live in YAML files under each [package](glossary.md#package)'s `config/` folder. A package is one self-contained unit of ROS2 code — the thing you build and install. There's a full breakdown of one [further down this page](#anatomy-of-a-package).

> **In this workspace, tuning happens in YAML, not in Python.** If you find yourself editing a number in a `.py` file, it probably should have been a parameter.

**Distro** — a numbered ROS2 release, each with its own package set. This car runs **Jazzy** (Ubuntu 24.04). This matters constantly, because much of the F1TENTH ecosystem still targets the older **Humble** release, and mixing them breaks in confusing ways. See the note in [git-setup.md](git-setup.md).

<details>
<summary><b>How nodes actually find each other</b> — the discovery mechanism underneath topics. Skip it: nothing in this workspace requires knowing it, but it explains a class of weird networking bug.</summary>

ROS2 has no central broker. Underneath, it uses **DDS** (Data Distribution Service), where each node broadcasts its existence on the local network and discovers peers automatically. There's no master process to start, and no fixed order — you can start nodes in any sequence and they'll find each other.

Two consequences worth knowing:

- **Nodes on the same network can see each other by accident.** If two people run ROS2 on the same LAN, their nodes may join one graph. ROS2 partitions this with the `ROS_DOMAIN_ID` environment variable — machines with different domain IDs ignore each other. If topics from someone else's laptop appear in `ros2 topic list`, this is why.
- **Discovery is not instant.** A subscriber that starts after a publisher may miss the first messages. This is why some topics are "latched" (technically, `transient_local` durability): a late subscriber still receives the most recent message. The map topic works this way, which is why a dashboard opened halfway through a run still gets the map.

</details>

---

## What `ros2 launch` actually does

You *can* start a single node by hand:

```bash
ros2 run <package> <executable>
```

That's fine for one node — say `vesc_driver_node`, the program that talks to the motor controller. But a real driving stack is a dozen nodes, each needing its own parameters, some needing their topics renamed. Starting those by hand, in the right order, in a dozen terminals, correctly, every time, is not realistic.

So instead there's a **launch file**: a Python script that declares which nodes to start, which YAML config to load for each, and any topic renaming. [`bringup_launch.py`](../src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py) is the one you'll use most.

```bash
ros2 launch <package> <file>.py
```

runs that script, starts everything it declares in one shot, and — importantly — a single `Ctrl+C` cleanly shuts all of it down together.

### What you'd actually launch

What you launch depends on what you're doing. The full explanation of why driving needs *two* launch commands is in [architecture.md](architecture.md#the-node-graph); the short version is that one gives you the hardware and one gives you control over it.

| Command | What it starts |
|---|---|
| `ros2 launch f1tenth_stack bringup_launch.py` | The shared foundation: joystick input, VESC, LiDAR, mux. **Never a control layer by itself** — nothing publishes to `/teleop` or `/drive` until something else is launched on top. Nearly everything else assumes this is already running, in its own terminal. |
| `ros2 launch f1tenth_stack teleop_launch.py` | The manual-driving control layer (just `joy_teleop`), launched on top of the bringup, in a second terminal. |
| `ros2 launch racerbot_launch slam_launch.py` | Mapping. |
| `ros2 launch particle_filter localize_launch.py` | Localizing against a saved map. |
| `ros2 launch gap_follow gap_follow_launch.py` | Reactive autonomy — another control layer, on top of the bringup. |
| `ros2 launch pure_pursuit pure_pursuit_launch.py` | Map-based racing — another control layer, on top of the bringup. |

**Run at most one control layer at a time.** Two things fighting over the steering is exactly as bad as it sounds — see [architecture.md](architecture.md#control-layers-exactly-one-at-a-time-in-a-second-terminal).

Step-by-step procedures for each of these, with what success looks like, are in [operations.md](operations.md).

---

## What `colcon build` actually does

`colcon` is the ROS2 build tool. Run from the workspace root, it finds every package under `src/` — it recognizes them by their `package.xml` file.

It then works out which packages depend on which, so it can build them in a valid order. For each one, it compiles and installs.

**Compiles** anything that needs compiling. C++ packages like `ackermann_mux` get run through CMake and make. Python packages don't need compiling, but still get processed. This is where the `build/` folder comes from: intermediate per-package artifacts like the CMake cache and object files. You never read or edit anything in `build/` by hand.

**Installs** the result into `install/`. That means compiled executables, plus every package's launch files, config YAML, and other declared resources. They're laid out where ROS2 expects to find them at runtime, under `install/<package>/share/<package>/`.

> **`install/` is what you actually run.** Not `src/`. `source install/setup.bash` points your shell at `install/`, and that's where `ros2 launch` and `ros2 run` read from. This surprises people: editing a file in `src/` may change nothing until it gets into `install/`.

### The flag that saves you the most time

**Terminal 1, from `~/racerbot-ws`:**

```bash
colcon build --symlink-install
```

**Working when:** the last line reads `Summary: N packages finished [time]` with no `failed` count. Warnings during the build are normal and can be ignored; `failed` cannot.

`--symlink-install` makes Python, launch, and config files get **symlinked** into `install/` rather than copied. A symlink is a pointer, so `install/` and `src/` are now the same file under two names.

The practical effect: **editing a `.py`, launch, or YAML file takes effect on the next launch, with no rebuild.** Change a parameter, relaunch, see the difference. Without this flag you'd rebuild after every one-line tweak.

You still have to rebuild after:

- changing `package.xml` or `setup.py`
- adding or removing files
- touching any C++

Always use `--symlink-install`, always. Everything in these docs assumes it.

### The other flags

**Terminal 1, from `~/racerbot-ws`:**

```bash
colcon build --symlink-install --packages-select gap_follow
```

**Working when:** `Summary: 1 package finished`. If it says more than one, you typed the package name wrong and colcon fell back to building everything.

`--packages-select <name>` builds one package instead of all of them. Use it constantly while iterating — it turns a multi-minute rebuild into a few seconds.

Drop it to rebuild everything. On this Jetson's 8GB of RAM, a full rebuild should add `--parallel-workers 1`, which builds one package at a time instead of many:

**Terminal 1, from `~/racerbot-ws`:**

```bash
colcon build --symlink-install --parallel-workers 1
```

**Working when:** packages tick past one at a time and it ends with `Summary: N packages finished`. Expect this to take several minutes — that's the trade you're making.

Without it, a full rebuild can exhaust memory and get killed by the OS — an **OOM** (out-of-memory) kill. If a build dies with no useful error, or the machine locks up, this is the first thing to try.

`log/` holds a timestamped folder per build and per run, with the full compiler and launch output. This is genuinely useful: when a build fails, the real error is often thousands of lines above where your terminal stopped. `log/latest_build` always symlinks to the most recent one.

<details>
<summary><b>When the build is in a broken state</b> — the nuclear option and why it's safe.</summary>

If the build gets into a state that makes no sense — stale artifacts, a package that won't rebuild, errors referencing files you deleted — delete the three generated directories and rebuild:

```bash
rm -rf build install log && colcon build --symlink-install --parallel-workers 1
```

This is safe. All three are pure build output, all three are gitignored, and none contain anything you wrote. The only cost is time.

It's worth trying this before spending an hour on a confusing build error, because stale `colcon` state produces some genuinely misleading messages.

</details>

---

## Why you have to `source` things, and what that means

**Sourcing** a script means running it *inside your current shell*, with `source script.sh` rather than `./script.sh`.

The difference matters. `./script.sh` starts a new shell, runs the script there, and throws that shell away — so any environment variables it set are gone.

`source script.sh` runs the commands in the shell you're sitting in. The variables it sets (`PATH`, `PYTHONPATH`, `AMENT_PREFIX_PATH`, and others) **stay** after it finishes.

Two sourcing steps, always in this order:

```bash
source /opt/ros/jazzy/setup.bash        # 1. base ROS2
source ~/racerbot-ws/install/setup.bash  # 2. this workspace, on top
```

1. **`/opt/ros/jazzy/setup.bash`** makes the base ROS2 install visible: the `ros2` and `colcon` commands themselves, plus anything installed via `apt` such as `slam_toolbox` and `urg_node`.
2. **`~/racerbot-ws/install/setup.bash`** layers this workspace's own built packages (`gap_follow`, `f1tenth_stack`, `pure_pursuit`, and the rest) on top.

Skip them and things fail in ways that look like the code is broken when it isn't.

Without the first, `ros2` and `colcon` may not exist in your `PATH` at all. Without the second, `ros2 launch` and `ros2 run` can't find this workspace's packages — the shell simply has no record they exist.

The error you get is usually "package not found", which reads like a missing install rather than a missing `source`. That's why this catches people repeatedly.

> **You have to do this in every new terminal.** Environment variables don't survive across separate shell sessions, so a new terminal, a new SSH connection, or a new tmux pane each start clean. There's no way around it short of putting the two lines in your `~/.bashrc`. That is deliberately **not** done on this machine, so it's manual every time.

**If a `ros2` command behaves strangely, check your sourcing before anything else.** A surprising share of "the code is broken" turns out to be an unsourced terminal.

<details>
<summary><b>Why the order matters, and what "overlay" means</b> — the underlay/overlay model. Skip unless a package is resolving to the wrong version.</summary>

ROS2 stacks environments. `/opt/ros/jazzy` is the **underlay**; your workspace's `install/` is the **overlay** on top of it. Sourcing in that order means the workspace's version of a package wins over the system's.

That layering is what makes vendored packages work. This workspace ships its own `ackermann_mux` and `vesc` under `src/`, and because the workspace is sourced second, those are the ones that get used even if a system version exists.

Source them the wrong way round and the system version silently wins — you'd be running code you aren't editing, with no error message to tell you.

To see which copy is actually in play:

```bash
ros2 pkg prefix ackermann_mux
```

If that prints a path under `/opt/ros/`, you're running the system copy. If it prints one under `~/racerbot-ws/install/`, you're running this workspace's.

</details>

---

## What each top-level folder is for

| Folder | What it is |
|---|---|
| `src/` | Source code. What you read, edit, and add new packages into. See the package table in the [README](../README.md#layout-src) and [architecture.md](architecture.md#package-reference) for what's in each one. |
| `build/` | Intermediate `colcon build` artifacts, one subfolder per package. Not human-facing; safe to delete. |
| `install/` | The actual runtime output of the build — what you `source`, and what `ros2 launch` and `ros2 run` use. |
| `log/` | Timestamped `colcon build` and `ros2 launch` logs, for debugging a failed build or launch after the fact. |
| `docs/` | This documentation set. Start at [docs/README.md](README.md). |

Only `src/` and `docs/` contain anything a human wrote. The other three are generated, and all three are gitignored.

---

## How the car actually runs the code

There's **no autostart**. No `systemd` service, no boot script, nothing that runs when the car powers on. It is entirely manual:

1. Someone opens a terminal — directly on the Jetson, or over SSH.
2. They run the two `source` commands.
3. They run a `ros2 launch` command by hand.

That's it. If nobody typed a command, nothing is running. This is a deliberate safety property, not an oversight: a car that starts driving code on boot is a car that can move while nobody is paying attention.

The nodes that launch starts then talk to the physical hardware directly:

| Hardware | How it connects |
|---|---|
| VESC (motor controller) | serial, at `/dev/sensors/vesc` |
| Hokuyo LiDAR | Ethernet |
| F710 gamepad | USB |

...and to each other purely over ROS2 topics. No shared memory, no direct function calls between packages. The topic graph is in [architecture.md](architecture.md).

Shutting down is the same story in reverse: `Ctrl+C` in the launch terminal. If something is stuck, [operations.md](operations.md#shutting-down-cleanly) has the `pkill` commands.

---

## Anatomy of a package

Every local Python (`ament_python`) package in this workspace follows the same shape. Here's `gap_follow`, which is the simplest one and the intended template:

```
src/gap_follow/
├── package.xml            # manifest: name, dependencies (rclpy, sensor_msgs, ...). colcon/rosdep read this.
├── setup.py               # registers console_script entry points (why `ros2 run gap_follow gap_follow_node` works)
│                          # and lists which files (launch/, config/) get installed into install/<pkg>/share/<pkg>/
├── setup.cfg              # boilerplate pointing the script installer at the right output dir
├── resource/gap_follow    # empty marker file — required so ROS2's package index knows this package exists
├── gap_follow/            # the actual importable Python module
│   ├── __init__.py
│   └── gap_follow_node.py # the node: subscribes/publishes topics, contains the algorithm
├── launch/
│   └── gap_follow_launch.py  # declares the Node action + which config YAML to load as parameters
└── config/
    └── gap_follow.yaml    # the actual parameter values — tune behavior here, not in the Python
```

Two things about this trip people up:

**The doubled folder name is not a mistake.** `src/gap_follow/gap_follow/` is correct. The outer one is the package; the inner one is the importable Python module. They're required to have the same name.

**`resource/gap_follow` is an empty file, and it has to exist.** It's how ROS2's package index discovers the package. Delete it and the package vanishes from `ros2 pkg list` with no error explaining why.

### Where to go next

- [writing-your-own-node.md](writing-your-own-node.md) — using this shape as a template for your own package. **Required reading if your code can move the car.**
- [adding-your-own-code.md](adding-your-own-code.md) — deciding where new code belongs in the first place.
- [racing-autonomy.md](racing-autonomy.md) — `pure_pursuit`'s variant of this shape, which splits the pure math into its own dependency-light, unit-testable file. Worth copying for anything more complex than `gap_follow`, because it lets you test the algorithm without a robot.
