# Guided odometry calibration

The workspace includes a local, read-only Web wizard for tape-measure
calibration of VESC speed odometry and optional steering conversion.

Start it after the normal car bringup:

```bash
cd ~/racerbot-ws
source install/setup.bash
ros2 launch odom_calibration odom_calibration_launch.py
```

Open `http://<car-ip>:8090/`. The wizard never publishes a ROS command; the
operator drives with the physical remote and LB.

The complete procedure, calibration equations, failure handling, and report
interpretation are documented in
[`src/odom_calibration/README.md`](../src/odom_calibration/README.md).
