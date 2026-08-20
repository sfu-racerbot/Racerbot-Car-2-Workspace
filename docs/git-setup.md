# Git setup / version control

> **Who this is for:** anyone cloning this repo fresh, or about to pull upstream changes into a vendored package.
> **Read first:** nothing, though [concepts.md](concepts.md) explains what the `src/` packages are.
> **You'll be able to:** tell submodules from vendored code, update either safely, and avoid clobbering this workspace's local fixes.

How this [workspace](glossary.md#workspace) — the whole `~/racerbot-ws` tree that colcon builds — is versioned. Which [packages](glossary.md#package) are real git submodules, which are plain vendored copies, and what to check before pulling upstream updates.

**Read this before touching `.gitmodules`, running any `git submodule` command, or updating any [package](glossary.md#package) listed below.**

---

## Why this page exists

Most of `src/` is other people's code. Some of it is linked to its original repository; one package deliberately isn't, because this workspace has patched it in ways a link can't carry.

Get that distinction wrong and you can silently revert a fix.

One of those fixes, if reverted, makes the car's [odometry](glossary.md#odometry--odom) — its own estimate of how far it has travelled — report the wrong sign. With no error message. Every speed-aware safety layer degrades along with it.

That's the whole reason this page is written down.

---

## First, a word on submodules

If you haven't used them: a **git submodule** is a pointer from this repository to a specific commit in *another* repository. This repo stores the URL and the commit ID, not the files.

That's useful: you get someone else's code at a known version without copying it in.

But it has one hard limitation, and that limitation is the reason for everything below.

> **A submodule can only ever point at a commit that exists in the other repo.** There is no way to carry a local edit through a submodule into this repository's history.

So if you need to patch third-party code and keep the patch, a submodule can't help you. You either fork upstream, or you **vendor** it — copy the files in and track them like your own.

This workspace vendors exactly one package, for exactly that reason.

---

## Remote

- GitHub, private: `https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace.git`
- Default and only branch: `main`

**Gitignored, never expected in a commit:** `build/`, `install/`, `log/` (colcon artifacts), `__pycache__/` and `*.pyc`, `.sim`, `.venv-f1tenth/`, `.pytest_cache/`.

`.claude/` is ignored **except** `.claude/skills/`, which is tracked deliberately — it holds the [documentation standard](../.claude/skills/novice-docs/SKILL.md) and its checker, which belong to the team rather than to one machine.

---

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

**Team-developed vs. workspace code.** `racerbot_a` and `racerbot_b` are the codebases the Racerbot teams are actively developing.

The workspace-specific integration code and documentation outside those two repositories were produced through vibe coding (AI-assisted development). The remaining submodules and vendored packages are third-party upstream code, not team-authored.

---

## The one vendored package, and the three fixes it carries

`f1tenth_system` used to be a submodule. It was **deliberately disconnected from upstream** and converted to a normal tracked directory.

Concretely: its `.git` gitlink and its `.gitmodules` / `.git/config` entries were removed, then the files were kept and `git add`-ed like any other package.

It carries three local fixes that have to live in this repo's history.

> **All three are guaranteed to be silently clobbered by a naive upstream overwrite.** Check every one of them after any sync.

### Fix 1 — the steering axis

`src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml`

The `human_control` profile's `drive-steering_angle` axis was changed from upstream's `axis: 2` to **`axis: 3`**.

Upstream's axis 2 is this F710's *left trigger*. Axis 3 is the right stick, in XInput mode — see [hardware-reference.md](hardware-reference.md#joystick--logitech-f710).

**How it fails if reverted:** obviously and harmlessly. Steering moves to the trigger, you notice immediately.

### Fix 2 — teleop split into its own launch file

[Teleop](glossary.md#teleop) means driving the car by hand with the gamepad. A [launch file](glossary.md#launch-file) is the script that starts a set of nodes together.

`src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py` and `launch/teleop_launch.py`

Upstream bundles `joy_teleop` into `bringup_launch.py` itself. Here it's split out into its own [launch file](glossary.md#launch-file), `teleop_launch.py`.

That makes manual driving ([teleop](glossary.md#teleop)) and autonomy independent control layers you switch between with `Ctrl+C` — not a `pkill` dance. See [architecture.md](architecture.md#the-node-graph).

`bringup_launch.py` still starts `joy_node`, because every control layer's deadman check needs it. It just doesn't start `joy_teleop` any more.

**How it fails if reverted:** confusingly. Autonomy silently stops working, because a bundled `joy_teleop` masks `/drive` permanently — see [troubleshooting.md](troubleshooting.md#autonomy-node-publishes-to-drive-car-doesnt-move-no-errors-anywhere).

### Fix 3 — the odometry sign, which fails silently

`src/f1tenth_system/f1tenth_stack/config/vesc.yaml`

`vesc_to_odom_node` overrides `speed_to_erpm_gain` to **`-4614.0`**, against the `+4614.0` in the shared `/**` block. (The [VESC](glossary.md#vesc) is the motor controller.)

> **This is the dangerous one.** Nothing errors if it's lost. Odometry just reports the wrong sign, and every speed-aware safety layer degrades along with it.

**Verify both nodes after any upstream sync:**

```bash
ros2 param get /vesc_to_odom_node speed_to_erpm_gain      # must be -4614.0
ros2 param get /ackermann_to_vesc_node speed_to_erpm_gain # must be +4614.0
```

**Working when:** the two values are opposite in sign, exactly as shown. If they match, Fix 3 has been lost.

### Not a fix: the SLAM tuning lives outside this package

[SLAM](glossary.md#slam) is the software that builds a map and locates the car in it at the same time.

`src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml` is **unmodified**, and should stay that way.

This workspace does change how `slam_toolbox` is tuned: how often it corrects the car's position, and how far it looks when trying to close a loop. Those changes live in a file this repo owns:

`src/racerbot_launch/config/slam_tracking.yaml`

`slam_launch.py` loads the vendored config first and then layers ours on top, so later values win. The vendored file stays clean, an upstream sync cannot take our tuning with it, and there is one place to read what this workspace changed about SLAM.

**Follow the same pattern for anything else you want to change about a vendored node.** A fourth entry on the list above is a fourth thing to check after every sync; a layered override file is none.

Why those particular values moved is in [localization.md](localization.md).

<details>
<summary><b>Why the two nodes need opposite signs</b> — the upstream hardcoded negation this cancels. Read before "tidying up" what looks like an inconsistency.</summary>

It looks like a typo. It isn't, and the asymmetry is load-bearing.

`vesc_ackermann`'s `vesc_to_odom.cpp` hardcodes:

```
speed = (-state.speed - offset) / gain
```

That negates the eRPM the VESC reports. Upstream assumes a VESC that reports eRPM with the *opposite* sign to the commanded eRPM.

**This car's VESC doesn't.** It reports eRPM with the same sign as the command. So without the override, `/odom` publishes **negative** `twist.linear.x` while the car drives forward — and integrates the `odom`→`base_link` TF backwards along with it.

The `-4614.0` override cancels that hardcoded negation, for odometry only. [TF](glossary.md#tf--transform--frame) — ROS2's record of where things sit relative to each other — is built from that odometry, which is why a sign error there propagates into everything positional.

`ackermann_to_vesc_node` — which sets the actual motor command — **must keep `+4614.0`**. Flip that one and a forward command drives the car backwards.

So: same parameter name, two nodes, opposite signs, both correct.

Added 2026-07-27, after a `gap_follow` collision. See [hardware-reference.md](hardware-reference.md).

</details>

### What vendoring costs you

**`f1tenth_system` will never move on its own.** There's no `git submodule update --remote` for it any more.

Updating it means manually pulling upstream changes and re-applying or re-checking the local modifications — [the procedure is below](#updating-f1tenth_system--manual-no-submodule-machinery).

---

## Cloning this repo fresh

```bash
git clone --recurse-submodules https://github.com/sfu-racerbot/Racerbot-Car-2-Workspace.git
```

**Working when:** `src/particle_filter/` and the other submodule directories contain files, not just empty folders.

**Forgot `--recurse-submodules`?** Or does your clone predate a submodule being added? The submodule paths — `racerbot_a/`, `racerbot_b/`, `particle_filter/`, `range_libc/`, `transport_drivers/`, `realsense-ros/` — will exist but be empty. Fix it with:

```bash
git submodule update --init --recursive
```

`f1tenth_system` needs no such step. It's a plain part of the repo and arrives with a normal clone — which is the upside of vendoring.

---

## Working on Racerbot A or Racerbot B

**Order matters here.** Team code must be committed and pushed from inside the submodule *first*, then the pointer committed in this workspace.

**Terminal 1, from `~/racerbot-ws`:**

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

> **Never leave a team-code commit only on one machine.** The workspace repository records only the referenced commit ID, not the submodule's files.
>
> Push the submodule commit before pushing the workspace pointer. Do it the other way round and everyone else gets a pointer to a commit that doesn't exist anywhere they can reach — their clone breaks, and the fix has to come from your laptop.

---

## Checking for upstream updates

Do this periodically, not just when something breaks.

**The context:** none of the official `f1tenth` / roboracer / ros-drivers repos below have a `jazzy` branch yet (per the main [README.md](../README.md#notes)). Everything here is `humble`-branch source built against ROS2 Jazzy.

So before any dependency bump — or if a `rosdep` or build error looks upstream-related — check each repo for a newer ROS2-distro branch *before* patching locally.

| Package | Upstream |
|---|---|
| `particle_filter` | https://github.com/f1tenth/particle_filter |
| `range_libc` | https://github.com/f1tenth/range_libc |
| `transport_drivers` | https://github.com/ros-drivers/transport_drivers |
| `f1tenth_system` (vendored, no longer linked) | https://github.com/f1tenth/f1tenth_system |
| `realsense-ros` | https://github.com/realsenseai/realsense-ros |

> **`realsense-ros` is the exception** to the "no jazzy branch yet" caveat above. Its `ros2-master` branch already supports Jazzy natively — along with Humble, Iron, Kilted and Rolling from the same branch — so there's no need to hunt for a different branch on a bump.

### Updating an actual submodule

For `particle_filter`, `range_libc`, `transport_drivers`, `realsense-ros`:

**Terminal 1, from `~/racerbot-ws`:**

```bash
cd src/<package>
git fetch origin
git checkout origin/<branch>   # e.g. humble-devel, or a newer distro branch if one now exists
cd ../..
git add src/<package>
git commit -m "Bump <package> submodule"
```

Then rebuild just that package and re-test before trusting it:

```bash
colcon build --symlink-install --packages-select <package>
```

**Working when:** `Summary: 1 package finished`, and whatever that package does still works on the car.

### Updating `f1tenth_system` — manual, no submodule machinery

1. **Diff your vendored copy against upstream.** Take a fresh clone or checkout of `f1tenth/f1tenth_system` at whatever branch or commit you want to pull in, and see what actually changed.
2. **Apply the parts you want by hand** into `src/f1tenth_system/` — copy files over, or use `git diff` / `git apply` between the two trees.
3. **Before committing, re-check all three local modifications.** Every one of them:

   | Check | Should still be |
   |---|---|
   | `f1tenth_stack/config/joy_teleop.yaml` → `drive-steering_angle` under `human_control` | `axis: 3`, **not** upstream's `axis: 2` |
   | `f1tenth_stack/launch/bringup_launch.py` | still does **not** start a `joy_teleop` node, with `teleop_launch.py` still present separately (upstream will have bundled them back together) |
   | `f1tenth_stack/config/vesc.yaml` → `vesc_to_odom_node` | still carries `speed_to_erpm_gain: -4614.0` |

4. **Commit normally:** `git add src/f1tenth_system && git commit`. There's no submodule pointer to bump — the files themselves are the commit.

> **Do step 3 even when you're confident.** All three fixes are silently reverted by a naive overwrite, and the third one fails silently *and* unsafely.

---

## Related docs

- [hardware-reference.md](hardware-reference.md) — the axis-3 fix from the hardware and joystick side, plus other exact config values for this car.
- [troubleshooting.md](troubleshooting.md) — the axis mixup as a symptom you'd actually notice ("one axis doesn't do what you expect").
- [README.md](../README.md) — doc index and `src/` package table.
