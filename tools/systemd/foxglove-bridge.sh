#!/bin/bash
# Start foxglove_bridge the way tools/systemd/foxglove-bridge.service needs:
# a login shell's ROS environment, rebuilt from scratch, because systemd
# starts services with an almost empty environment.
#
# ROS_DOMAIN_ID and RMW_IMPLEMENTATION are deliberately NOT set here. The
# car's stack runs with both unset (checked 2026-09-26: not in ~/.bashrc,
# ~/.profile or /etc/environment), which means domain 0 and Fast DDS
# (rmw_fastrtps_cpp, the only RMW installed). A bridge on a different
# domain would start fine and show an empty topic list. If the team ever
# sets either for the stack, put the same values in
# /etc/default/foxglove-bridge (see the unit file) -- never only here.
#
# See docs/foxglove-bridge.md.

# No `set -u`: ROS's setup.bash reads unset variables on purpose.
set -e

WORKSPACE="${RACERBOT_WS:-/home/racerbotcar-2/racerbot-ws}"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE}/install/setup.bash"

exec ros2 launch racerbot_launch foxglove_bridge_launch.py
