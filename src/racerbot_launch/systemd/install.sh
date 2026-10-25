#!/usr/bin/env bash
# Start car 2's web dashboard server at boot.
#
#   sudo src/racerbot_launch/systemd/install.sh <user> <workspace>
#   e.g. sudo src/racerbot_launch/systemd/install.sh racerbotcar-2 /home/racerbotcar-2/racerbot-ws
#
# <user> runs the service (the account that owns the ROS 2 workspace);
# <workspace> is the colcon workspace with racerbot_launch and web_dashboard
# built into it.
#
# Installs /etc/systemd/system/racerbot-dashboard.service, enables it, and
# (re)starts it so a changed unit takes effect now. The unit runs the
# workspace's install space, not this checkout, so it keeps working from
# wherever this script was run.
#
# The Lichtblick bridge has its own unit in the submodule
# (src/web_dashboards/car/systemd/install.sh); the camera is still started
# by hand (docs/web-dashboard.md).
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then echo "run with sudo" >&2; exit 1; fi
user="${1:-}"; workspace="${2:-}"
if [ -z "$user" ] || [ -z "$workspace" ]; then
  echo "usage: sudo $0 <user> <workspace>" >&2; exit 1
fi
for pkg in racerbot_launch web_dashboard; do
  if [ ! -d "$workspace/install/$pkg" ]; then
    echo "$workspace/install/$pkg not found -- build the workspace first (colcon build --symlink-install)" >&2
    exit 1
  fi
done
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sed -e "s|@USER@|$user|g" -e "s|@WORKSPACE@|$workspace|g" \
  "$here/racerbot-dashboard.service" > /etc/systemd/system/racerbot-dashboard.service
systemctl daemon-reload
systemctl enable racerbot-dashboard
systemctl restart racerbot-dashboard
systemctl --no-pager --lines=5 status racerbot-dashboard || true
