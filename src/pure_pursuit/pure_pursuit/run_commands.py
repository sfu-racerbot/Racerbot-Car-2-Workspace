"""What to run next, for a map saved by a mapping-only run.

auto_map_race_node in ``finish_after_mapping`` mode (``ros2 run
racerbot_launch map_course``) writes this text to ``commands.txt`` in the run
directory and prints it, and map_course prints it again after everything has
shut down, so the last thing on screen is a set of commands to copy.

Pure Python, no rclpy: the exact text is unit-tested
(test/test_run_commands.py), because a copy-paste command that is subtly
wrong is worse than none.
"""

import math
from pathlib import Path

START_POSE_FILE = 'start_pose.yaml'

# steering_law values pure_pursuit_node accepts (pure_pursuit.yaml), in the
# order they are offered, with a one-line description each.
STEERING_LAWS = (
    ('pure_pursuit', 'pure pursuit (the default racing controller)'),
    ('stanley', 'Stanley'),
    ('lqr', 'LQR'),
    ('mpc', 'MPC'),
)

RULE = '=' * 72


def write_start_pose(run_dir, pose) -> None:
    """Save the car's map-frame race start pose (x, y, yaw) for race_launch.py."""
    x, y, yaw = (float(v) for v in pose)
    (Path(run_dir) / START_POSE_FILE).write_text(
        '# Where the car stopped at the end of mapping (map frame, base_link).\n'
        '# race_launch.py run_dir:=... seeds the particle filter here.\n'
        f'x: {x:.4f}\ny: {y:.4f}\nyaw: {yaw:.4f}\n')


def read_start_pose(run_dir):
    """(x, y, yaw) from run_dir/start_pose.yaml, or None if there is no
    usable one (a run from before the file existed, or a damaged file) --
    the caller then falls back to the map origin."""
    path = Path(run_dir) / START_POSE_FILE
    try:
        fields = {}
        for line in path.read_text().splitlines():
            line = line.split('#', 1)[0].strip()
            if ':' in line:
                key, value = line.split(':', 1)
                fields[key.strip()] = float(value)
        pose = (fields['x'], fields['y'], fields['yaw'])
    except (OSError, KeyError, ValueError):
        return None
    return pose if all(math.isfinite(v) for v in pose) else None


def race_command(run_dir, steering_law: str = 'pure_pursuit') -> str:
    """The one-line race command for ``run_dir`` and one steering law."""
    command = f'ros2 launch racerbot_launch race_launch.py run_dir:={Path(run_dir)}'
    if steering_law != 'pure_pursuit':
        command += f' steering_law:={steering_law}'
    return command


def commands_text(run_dir, raceline_ok: bool = True, map_ok: bool = True,
                  start_ok: bool = True) -> str:
    """The full block printed at the end of a mapping-only run."""
    run_dir = Path(run_dir)
    lines = [RULE, f'MAPPING FINISHED.  Saved in: {run_dir}', RULE]
    saved = []
    if map_ok:
        saved.append('map.yaml + map.pgm (the map), posegraph (SLAM pose graph)')
    if raceline_ok:
        saved.append('raceline_profiled.csv (racing line with speeds)')
    if start_ok:
        saved.append('start_pose.yaml (where the car is stopped now; racing starts there)')
    for item in saved:
        lines.append(f'  - {item}')
    if not map_ok:
        lines += [
            '',
            '  !! The map did NOT save (see the errors above). The race',
            '  !! commands below will not work for this run. Map again.',
        ]
    if not raceline_ok:
        lines += [
            '',
            '  !! No racing line was saved (see the errors above). Record one',
            '  !! by hand -- docs/operations.md, "Manual/reusable saved-map',
            '  !! workflow" -- or map again.',
        ]
    if not start_ok:
        lines += [
            '',
            '  !! The start pose was not recorded; racing will assume the car',
            '  !! starts at the map origin, which may not be where it is.',
        ]
    lines += [
        '',
        'BEFORE RACING: leave the car exactly where it stopped. If you move',
        'it, put it back on that spot, facing the same way. The race launch',
        'tells the localizer the car starts there (there is no RViz on the',
        'car to click a start pose), and it only searches about 0.5 m and',
        '20 deg around it. Mark the spot (tape on the floor) before you move',
        'the car, and you can race this map again on any later day.',
        '',
        'Terminal 1 -- hardware: the bringup you mapped with, still running.',
        'If you closed it, start it again:',
        '  ros2 launch f1tenth_stack bringup_launch.py',
        '',
        'Terminal 2 -- pick ONE, then hold LB to drive:',
    ]
    for law, description in STEERING_LAWS:
        lines.append(f'  # {description}')
        lines.append(f'  {race_command(run_dir, law)}')
    lines += [
        '',
        f'These commands are also in {run_dir / "commands.txt"}',
        RULE,
    ]
    return '\n'.join(lines) + '\n'
