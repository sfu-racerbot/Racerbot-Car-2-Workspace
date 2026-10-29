"""
Every tool that saves a run writes it into the workspace's data/ folder.

The requirement (data/README.md): runs are committed to git, so they must
land inside ~/racerbot-ws/data/ rather than ~/.ros/, which git never sees.
The default is spelled out separately in each file below -- a node
parameter, its YAML, and every launch file that forwards it -- and a launch
default left at the old path silently wins over the node's, so each one is
checked, not just one.

Plain text, no ROS: runs unsourced.

    python3 -m pytest src/racerbot_launch/test/test_run_output_paths.py -v
"""
import re
from pathlib import Path

import pytest

WS = Path(__file__).resolve().parents[3]

# (file, the default it must contain) -- oracle: spec, data/README.md's table.
EXPECTED = [
    ('src/race_diagnostics/launch/record_run.py', "'~/racerbot-ws/data/racerbot_runs'"),
    ('src/pure_pursuit/pure_pursuit/auto_map_race_node.py',
     "'output_directory', '~/racerbot-ws/data/racerbot_auto'"),
    ('src/pure_pursuit/config/auto_map_race.yaml',
     'output_directory: ~/racerbot-ws/data/racerbot_auto'),
    ('src/racerbot_launch/launch/auto_map_race_launch.py', "'~/racerbot-ws/data/racerbot_auto'"),
    ('src/racerbot_launch/launch/auto_map_race_launch.py',
     "'output_directory': '~/racerbot-ws/data/racerbot_runs'"),
    ('src/racerbot_launch/launch/map_course_launch.py', "'~/racerbot-ws/data/racerbot_auto'"),
    ('src/racerbot_launch/launch/map_course_launch.py',
     "'output_directory': '~/racerbot-ws/data/racerbot_runs'"),
    ('src/racerbot_launch/scripts/map_course',
     "DEFAULT_OUTPUT = '~/racerbot-ws/data/racerbot_auto'"),
    ('src/racerbot_launch/scripts/map_course',
     "LOG_DIRECTORY = '~/racerbot-ws/data/racerbot_runs'"),
    ('src/racerbot_sim/launch/sim_auto_map_race_launch.py',
     "'~/racerbot-ws/data/racerbot_sim/auto'"),
    # bringup's automatic recorder (race_diagnostics auto_record, from car 1)
    ('src/race_diagnostics/race_diagnostics/auto_record_node.py',
     "'output_directory', '~/racerbot-ws/data/racerbot_runs'"),
    ('src/race_diagnostics/config/auto_record.yaml',
     'output_directory: ~/racerbot-ws/data/racerbot_runs'),
    ('src/race_diagnostics/launch/auto_record_launch.py',
     "'record_output_directory', default_value='~/racerbot-ws/data/racerbot_runs'"),
    ('src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py',
     "'~/racerbot-ws/data/racerbot_runs/<time>/"),
    ('src/f1tenth_system/f1tenth_stack/launch/no_lidar_bringup_launch.py',
     "'~/racerbot-ws/data/racerbot_runs/<time>/"),
    ('src/f1tenth_system/f1tenth_stack/launch/sick_bringup_launch.py',
     "'~/racerbot-ws/data/racerbot_runs/<time>/"),
]

OLD_PATH = re.compile(r'\.ros/racerbot_(runs|auto|sim/auto)\b')


@pytest.mark.parametrize('relpath,needle', EXPECTED)
def test_default_is_the_data_folder(relpath, needle):
    assert needle in (WS / relpath).read_text(), relpath


@pytest.mark.parametrize('relpath', sorted({p for p, _ in EXPECTED}))
def test_no_writer_still_names_the_old_folder(relpath):
    # A second default left behind (an included launch file, a help string a
    # user copies) would put some runs back where git cannot see them.
    hits = [line for line in (WS / relpath).read_text().splitlines() if OLD_PATH.search(line)]
    assert hits == [], f'{relpath}: {hits}'
