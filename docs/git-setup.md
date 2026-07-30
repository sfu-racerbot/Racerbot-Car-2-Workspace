# Git setup / version control

How this workspace is versioned, what's a real git submodule vs. a plain vendored copy, and what to check before pulling upstream updates. Read this before touching `.gitmodules`, running any `git submodule` command, or updating any package listed below.

## Remote

- GitHub, private: `https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace.git`
- Default/only branch: `main`
- `build/`, `install/`, `log/` (colcon artifacts), `__pycache__/`/`*.pyc`, and `.claude/` (local Claude Code settings, machine-specific) are gitignored — never expected in a commit.

## Submodules vs. vendored code — know which is which

| `src/` package | Tracking | Upstream | Branch |
|---|---|---|---|
| `racerbot_a` | **team-developed git submodule** | `sfu-racerbot/racerbot_a` | `main` |
| `racerbot_b` | **team-developed git submodule** | `sfu-racerbot/racerbot_b` | `main` |
| `particle_filter` | **real git submodule** | `f1tenth/particle_filter` | `humble-devel` |
| `range_libc` | **real git submodule** | `f1tenth/range_libc` | `humble-devel` |
| `transport_drivers` | **real git submodule** | `ros-drivers/transport_drivers` | `humble` |
| `realsense-ros` | **real git submodule** | `realsenseai/realsense-ros` | `ros2-master` |
| `f1tenth_system` | **vendored (plain tracked files, NOT a submodule)** | was `f1tenth/f1tenth_system` | was `humble-devel` |

### Team-developed code vs. vibe-coded workspace code

`racerbot_a` and `racerbot_b` are the codebases the Racerbot teams are actively developing. The workspace-specific integration code and documentation outside those two repositories were produced through vibe coding (AI-assisted development). The remaining submodules and vendored packages are third-party upstream code, not team-authored workspace code.

`f1tenth_system` used to be a submodule too. It was **deliberately disconnected from upstream** and converted to a normal tracked directory (its `.git` gitlink and `.gitmodules`/`.git/config` entries were removed; the files themselves were kept and `git add`-ed like any other package) because it carries local fixes/modifications that have to be committed to this repo:

- `src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml` — the `human_control` profile's `drive-steering_angle` axis was changed from upstream's `axis: 2` to `axis: 3` (this F710's right stick in XInput mode; axis 2 is the left trigger — see [hardware-reference.md](hardware-reference.md#joystick--logitech-f710)).
- `src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py` / `launch/teleop_launch.py` — upstream bundles `joy_teleop` into `bringup_launch.py` itself; here it's split out into its own `teleop_launch.py` so manual driving and autonomy are independent control layers you switch between with `Ctrl+C`, not a `pkill` dance (see [architecture.md](architecture.md#the-node-graph)). `bringup_launch.py` still starts `joy_node` (every control layer's deadman check needs it), just not `joy_teleop` anymore.
- `src/f1tenth_system/f1tenth_stack/config/vesc.yaml` — `vesc_to_odom_node` overrides `speed_to_erpm_gain` to **`-4614.0`**, against the `+4614.0` in the shared `/**` block. `vesc_ackermann`'s `vesc_to_odom.cpp` hardcodes `speed = (-state.speed - offset) / gain`, negating the eRPM the VESC reports; this car's VESC already reports eRPM with the same sign as the commanded eRPM, so without the override `/odom` publishes **negative** `twist.linear.x` while the car drives forward, and integrates the `odom`→`base_link` TF backwards along with it. The override cancels the hardcoded negation for odometry only — `ackermann_to_vesc_node`, which sets the motor command, must keep `+4614.0` or a forward command would drive the car backwards. Added 2026-07-27 after a `gap_follow` collision (see [hardware-reference.md](hardware-reference.md)); verify both nodes after any upstream sync:
  ```bash
  ros2 param get /vesc_to_odom_node speed_to_erpm_gain      # must be -4614.0
  ros2 param get /ackermann_to_vesc_node speed_to_erpm_gain # must be +4614.0
  ```

A git submodule can only ever point at a commit in *someone else's* repo — there's no way to carry an uncommitted local edit through it into this repo's history. Vendoring was the simplest way to keep these changes without also standing up a fork. **Practical effect: `f1tenth_system` will never move on its own.** There's no `git submodule update --remote` for it anymore — updating it means manually pulling upstream changes and re-applying/re-checking the local modifications (see below).

## Cloning this repo fresh

```bash
git clone --recurse-submodules https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace.git
```
Forgot `--recurse-submodules`, or the clone predates a submodule being added? The submodule paths—including `racerbot_a/`, `racerbot_b/`, `particle_filter/`, `range_libc/`, `transport_drivers/`, and `realsense-ros/`—will exist but be empty:
```bash
git submodule update --init --recursive
```
`f1tenth_system` needs no such step — it's a plain part of the repo and comes with a normal clone.

## Working on Racerbot A or Racerbot B

Changes to the team code must be committed and pushed from inside the appropriate submodule first. Then commit the updated submodule pointer in this workspace:

```bash
cd src/racerbot_b                 # or src/racerbot_a
git add .
git commit -m "Describe the team code change"
git push origin main

cd ../..
git add src/racerbot_b            # or src/racerbot_a
git commit -m "Bump Racerbot B submodule"
git push origin main
```

The workspace repository records only the referenced commit ID, not the submodule's files. Never leave a team-code commit only on one machine: push the submodule commit before pushing the workspace pointer.

## Checking for upstream updates — do this periodically, not just on breakage

None of the official `f1tenth`/roboracer/ros-drivers repos below have a `jazzy` branch yet (per the main [README.md](../README.md#notes)); everything here is `humble`-branch source built against ROS2 Jazzy. Before any dependency bump or if a `rosdep`/build error looks upstream-related, check each repo for a newer ROS2-distro branch rather than patching locally first:

- `particle_filter` / `range_libc` — https://github.com/f1tenth/particle_filter, https://github.com/f1tenth/range_libc
- `transport_drivers` — https://github.com/ros-drivers/transport_drivers
- `f1tenth_system` (now vendored, no longer linked) — https://github.com/f1tenth/f1tenth_system
- `realsense-ros` — https://github.com/realsenseai/realsense-ros — **the one exception to the paragraph above**: its `ros2-master` branch already supports Jazzy natively (along with Humble/Iron/Kilted/Rolling from the same branch), so there's no "no jazzy branch yet" caveat here, and no need to hunt for a different branch on a bump.

**Updating an actual submodule** (`particle_filter`, `range_libc`, `transport_drivers`, `realsense-ros`):
```bash
cd src/<package>
git fetch origin
git checkout origin/<branch>   # e.g. humble-devel, or a newer distro branch if one now exists
cd ../..
git add src/<package>
git commit -m "Bump <package> submodule"
```
Rebuild just that package afterwards (`colcon build --symlink-install --packages-select <package>`) and re-test before trusting it.

**"Updating" `f1tenth_system`** (no submodule machinery to help you — do this manually):
1. Diff your vendored copy against a fresh clone/checkout of upstream `f1tenth/f1tenth_system` (whatever branch/commit you want to pull in) to see what actually changed upstream.
2. Apply the parts you want into `src/f1tenth_system/` by hand (copy files over, or `git diff`/`git apply` between the two trees).
3. **Before committing, re-check all three local modifications** — `f1tenth_stack/config/joy_teleop.yaml`'s `drive-steering_angle` under `human_control` should still be `axis: 3`, not reverted to upstream's `axis: 2`; `f1tenth_stack/launch/bringup_launch.py` should still *not* start a `joy_teleop` node, with `teleop_launch.py` still present as the separate control layer (upstream will have them bundled back together); and `f1tenth_stack/config/vesc.yaml` should still carry the `speed_to_erpm_gain: -4614.0` override under `vesc_to_odom_node`. All three are guaranteed to get silently clobbered by a naive overwrite, and the third one fails *silently and unsafely* — nothing errors, odometry just reports the wrong sign and every speed-aware safety layer degrades with it.
4. `git add src/f1tenth_system && git commit` as normal — there's no submodule pointer to bump, the files themselves are the commit.

## Other repo docs
- [hardware-reference.md](hardware-reference.md) — the axis-3 fix from the hardware/joystick side, and other exact config values for this car.
- [troubleshooting.md](troubleshooting.md) — the axis mixup as a symptom ("one axis doesn't do what you expect").
- [README.md](../README.md) — doc index and `src/` package table.
