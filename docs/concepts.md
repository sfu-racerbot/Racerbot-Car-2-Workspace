# Concepts: ROS2, colcon, and this workspace

This doc is for questions like "what actually *is* a workspace" and "why do I have to do this every time" — the stuff that's easy to cargo-cult from the README's quick-start without understanding. If you already know ROS2/colcon, skip this; everything else in `docs/` assumes the concepts here.

## What `ros2 launch` actually does

A single node (one running program, e.g. `vesc_driver_node`) can be started directly with `ros2 run <package> <executable>`. A real driving stack needs many nodes running together with specific parameters, so instead there's a **launch file** — a Python script (e.g. [`bringup_launch.py`](../src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py)) that declares a list of nodes to start, which YAML config/parameters to load for each, and any topic remappings. `ros2 launch <package> <file>.py` runs that script and starts everything it declares, in one shot, and a single `Ctrl+C` cleanly shuts all of it down together.

What you actually launch depends on what you're doing — see the table in [operations.md](operations.md):
- `ros2 launch f1tenth_stack bringup_launch.py` — the core stack (joystick, VESC, LiDAR, mux). Nearly everything else assumes this is already running.
- `ros2 launch racerbot_launch slam_launch.py` — mapping.
- `ros2 launch particle_filter localize_launch.py` — localizing against a saved map.
- `ros2 launch gap_follow gap_follow_launch.py` / `ros2 launch pure_pursuit pure_pursuit_launch.py` — autonomy, on top of the bringup.

## What `colcon build` actually does

`colcon` is the ROS2 build tool. Run from the workspace root, it looks at every package under `src/` (it finds them by their `package.xml`), works out the dependency order between them, and for each one:
- **Compiles** anything that needs compiling — C++ packages like `ackermann_mux` get run through CMake/make. This is where `build/` comes from: intermediate, per-package build artifacts (CMake cache, object files). You never read or edit anything in `build/` directly.
- **Installs** the result into `install/` — compiled executables, plus every package's launch files, config YAML, and any other declared resources, laid out the way ROS2 expects to find them at runtime (`install/<package>/share/<package>/...`). **`install/` is what you actually run** — it's what `source install/setup.bash` points your shell at, and what `ros2 launch`/`ros2 run` actually read from.

`--symlink-install` makes Python files and config/launch files get *symlinked* into `install/` instead of copied. That means editing a `.py` file takes effect on the next launch with no rebuild — you only need to `colcon build` again after changing `package.xml`, `setup.py`, adding/removing files, or touching any C++.

`--packages-select <name>` builds just one package (fast, use this while iterating). Drop it to rebuild everything (slow, and this Jetson's 8GB RAM means a full rebuild should add `--parallel-workers 1` to avoid OOM — see the README).

`log/` holds a timestamped folder per build/run with the full compiler/launch output — useful when a build fails and the terminal output scrolled past the actual error. `log/latest_build` symlinks to the most recent one.

## Why you have to `source` things, and what that means

"Sourcing" a script (`source some_script.sh`, not just running it as `./some_script.sh`) executes it *inside your current shell* so any environment variables it sets (`PATH`, `PYTHONPATH`, `AMENT_PREFIX_PATH`, etc.) persist in your terminal afterward, instead of vanishing when a subshell exits.

Two sourcing steps, always in this order:
1. `source /opt/ros/jazzy/setup.bash` — makes the base ROS2 install visible: the `ros2`/`colcon` commands themselves, plus anything installed via `apt` (`slam_toolbox`, `urg_node`).
2. `source ~/racerbot-ws/install/setup.bash` — layers this workspace's *own* built packages (`gap_follow`, `f1tenth_stack`, `pure_pursuit`, etc.) on top.

Without both, `ros2`/`colcon` may not exist in your `PATH` at all, or `ros2 launch`/`ros2 run` won't be able to find this workspace's packages — the shell simply has no record they exist. This has to be repeated in **every new terminal** because environment variables don't persist across separate shell sessions; there's no way around it short of adding the two lines to your `~/.bashrc` (not currently done on this machine, so it's manual every time).

## What each top-level folder is for

| Folder | What it is |
|---|---|
| `src/` | Source code. What you read, edit, and add new packages into. See the package table in the [README](../README.md#layout-src) and [architecture.md](architecture.md#package-reference) for what's in each one. |
| `build/` | Intermediate `colcon build` artifacts, one subfolder per package. Not human-facing; safe to `rm -rf build install log && colcon build` if something's in a broken state. |
| `install/` | The actual runtime output of the build — what you `source` and what `ros2 launch`/`ros2 run` use. |
| `log/` | Timestamped `colcon build`/`ros2 launch` logs, for debugging a failed build or launch after the fact. |
| `docs/` | This documentation set. |

## How the car actually runs the code

There's no autostart — no `systemd` service, no boot script. It's fully manual: someone opens a terminal (directly on the Jetson, or over SSH), runs the two `source` commands, then runs a `ros2 launch` command by hand. The nodes that launch starts then talk to the physical hardware directly — the VESC over serial (`/dev/sensors/vesc`), the Hokuyo LiDAR over Ethernet, the gamepad over USB — and to each other purely over ROS2 topics (no shared memory, no direct function calls between packages; see the topic graph in [architecture.md](architecture.md)). Shutdown is the same: `Ctrl+C` in the launch terminal, or the `pkill` commands in [operations.md](operations.md#shutting-down-cleanly) if something's stuck.

## Anatomy of a package (using `gap_follow` as the example)

Every local Python (`ament_python`) package in this workspace — `gap_follow`, `pure_pursuit`, `racerbot_launch`'s Python side — follows the same shape:

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

See [writing-your-own-node.md](writing-your-own-node.md) for using this as a template for your own package, and [racing-autonomy.md](racing-autonomy.md) for `pure_pursuit`'s variant of this shape (which splits pure math out into its own dependency-light, unit-testable file — a pattern worth copying for anything more complex than `gap_follow`).
