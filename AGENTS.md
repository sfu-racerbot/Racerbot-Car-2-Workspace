# Repository Guidelines

## Project Structure & Module Organization

This is a ROS 2 Jazzy `colcon` workspace. First-party Python packages live directly under `src/`; `racerbot_launch` contains shared orchestration. Team code is maintained in the `src/racerbot_a` and `src/racerbot_b` submodules. Other submodules and vendored trees provide drivers and localization; check `docs/git-setup.md` before modifying them. Tests belong in `src/<package>/test/`, launch files in `launch/`, parameters in `config/`, and system documentation in `docs/`. Never edit generated `build/`, `install/`, or `log/` content.

## Project Context & Claude History

Read `CLAUDE.md` before non-trivial work; it captures architecture, workflow, and physical-car safety decisions from the repository’s initial Claude-assisted development. Persistent Claude memory is in `~/.claude/projects/-home-racerbotcar-2-racerbot-ws/memory/`; sibling `*.jsonl` files contain session context. These files are machine-local, supplemental history: verify details against tracked code and docs, and never commit or expose their contents blindly.

## Build, Test, and Development Commands

Run commands from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Use `colcon build --symlink-install --packages-select <pkg>` while iterating; add `--parallel-workers 1` on the Jetson if memory is tight. Run ROS tests with `colcon test --packages-select <pkg>` followed by `colcon test-result --verbose`. Framework-independent tests can run directly, for example `python3 -m pytest src/pure_pursuit/test/ -v`. Launch installed nodes with `ros2 launch <pkg> <file>_launch.py`.

## Coding Style & Naming Conventions

Use four-space indentation and PEP 8 for Python: `snake_case` functions/modules, `PascalCase` classes, and `UPPER_CASE` constants. Keep ROS wiring in node modules and extract reusable logic into ROS-free modules. C++ should match adjacent ROS 2 style and compile with `-Wall -Wextra -Wpedantic`. Use `<feature>_node.py`, `<feature>_launch.py`, and matching YAML names. No repository-wide autoformatter is configured; preserve local style.

## Testing & Safety

Use pytest and ament lint; name Python tests `test_*.py`. Add tests for non-trivial control math, parsing, and regressions. There is no numeric coverage threshold, but changed behavior should be exercised. Drive publishers must retain the LB deadman check. Follow `docs/writing-your-own-node.md`: static checks, wheels off the ground, then low-speed floor testing. Never disable `enable_deadman` unilaterally.

## Commit & Pull Request Guidelines

Write short, imperative commit subjects; history uses both plain subjects (`Add ...`) and scoped prefixes (`feat:`, `docs:`, `chore:`). Keep each commit cohesive. Pull requests should describe affected packages and safety impact, link relevant issues, list exact test commands/results, and include screenshots for dashboard changes. For submodule work, commit inside the submodule first, then update the parent pointer explicitly.
