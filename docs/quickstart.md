# Quickstart command sheet

> **Who this is for:** someone who has driven the car before and wants the commands without the explanations.
> **Read first:** [operations.md](operations.md), which explains every command below and how to tell it worked.
> **You'll be able to:** copy the usual start-up commands, one terminal each.
> **Time:** a minute to read.

Every terminal starts with the first line. Each later line goes in its own
terminal.

```bash
# every terminal, first
source /opt/ros/jazzy/setup.bash && source ~/racerbot-ws/install/setup.bash

ros2 launch f1tenth_stack bringup_launch.py
# the dashboard already runs at boot (systemctl status racerbot-dashboard)
ros2 launch racerbot_launch realsense_camera_launch.py
ros2 launch slam_toolbox online_async_launch.py params_file:=$HOME/racerbot-ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
ros2 launch racerbot_launch slam_launch.py
ros2 launch particle_filter localize_launch.py
```

It worked when each terminal keeps running without a red `[ERROR]` line.
What each one should print is in [operations.md](operations.md).

`range_libc` (the GPU ray caster that `particle_filter` uses) is not built
by `colcon`. It is installed once, by hand, with `WITH_CUDA=ON`; the exact
commands are in
[gpu-acceleration.md](gpu-acceleration.md#what-changed-on-the-car).
