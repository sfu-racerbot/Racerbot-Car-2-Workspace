"""
run_activity.py

What actually happened during a recorded run, read from its bag and added to
its auto_record.yaml under `activity:`. auto_record_node.py starts this,
detached and at low priority, after each recording has closed:

    python3 -m race_diagnostics.run_activity <run directory>

  - started_late: the car was already moving at the first /odom, so the bag
    does not hold the start of the run (it also goes in auto_record.log).
  - moved, max_speed_mps, lb_held_fraction: tools/save_runs.sh lists runs that
    never moved so a person can decide whether to commit them.

It only reads the bag. An unknown stays None rather than becoming "did not
move": a bag with no /odom says nothing about whether the car moved.
"""

from datetime import datetime
import math
import os
from pathlib import Path
import sys

import yaml

# gap_follow.yaml's deadman_button, the F710's LB in XInput mode.
LB_BUTTON = 4
# Below this a stopped car's wheel odometry is noise, not driving.
MOVING_MPS = 0.1


def lb_state(buttons):
    """1 if this /joy message holds LB. Too short to hold it counts as not held."""
    return int(len(buttons) > LB_BUTTON and bool(buttons[LB_BUTTON]))


def summarize(odom_speeds, lb_states):
    """odom_speeds: /odom forward speeds in bag order. lb_states: 0/1, likewise."""
    speeds = [abs(speed) for speed in odom_speeds if math.isfinite(speed)]
    states = list(lb_states)
    return {
        'first_odom_speed_mps': round(speeds[0], 2) if speeds else None,
        'max_speed_mps': round(max(speeds), 2) if speeds else None,
        'moved': max(speeds) >= MOVING_MPS if speeds else None,
        'started_late': speeds[0] >= MOVING_MPS if speeds else None,
        'lb_held_fraction': round(sum(states) / len(states), 3) if states else None,
    }


def read_bag(run_dir):
    """The inputs to summarize(), from <run_dir>/bag. Needs ROS sourced."""
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from sensor_msgs.msg import Joy

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(run_dir) / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/odom', '/joy']))
    speeds, states = [], []
    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        if topic == '/odom':
            speeds.append(deserialize_message(data, Odometry).twist.twist.linear.x)
        else:
            states.append(lb_state(deserialize_message(data, Joy).buttons))
    return speeds, states


def add_to_info(run_dir, activity):
    """Add `activity:` to auto_record.yaml, replacing the file in one step."""
    path = Path(run_dir) / 'auto_record.yaml'
    info = yaml.safe_load(path.read_text()) or {}
    info['activity'] = activity
    temporary = path.with_suffix('.yaml.tmp')
    temporary.write_text(yaml.safe_dump(info, sort_keys=False))
    os.replace(temporary, path)


def report(run_dir, activity):
    """A lost start is worth a line in the log next to the runs."""
    if not activity.get('started_late'):
        return
    run_dir = Path(run_dir)
    now = datetime.now().astimezone().isoformat(timespec='seconds')
    with open(run_dir.parent / 'auto_record.log', 'a') as log:
        log.write(f"{now} WARN {run_dir.name} started late: the car was already at "
                  f"{activity['first_odom_speed_mps']} m/s at its first /odom\n")


def main(argv=None, reader=read_bag):
    argv = sys.argv[1:] if argv is None else argv
    run_dir = argv[0]
    # Detached from everything: there is nobody to raise to, and a bag a killed
    # recorder left behind is expected to be unreadable.
    try:
        activity = summarize(*reader(run_dir))
    except Exception as e:  # noqa: BLE001
        activity = {'error': repr(e)}
    add_to_info(run_dir, activity)
    report(run_dir, activity)
    return 0


if __name__ == '__main__':
    sys.exit(main())
