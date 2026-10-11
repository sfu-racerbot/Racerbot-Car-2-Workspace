# Calibration wizard end-to-end test

> **Who this is for:** someone changing `odom_calibration`, `vesc.yaml` handling, or the VESC/teleop chain the wizard watches.
> **Read first:** [docs/odom-calibration.md](../../docs/odom-calibration.md).
> **What's in it:** how to run the whole wizard against the car's real software with only the hardware simulated.

`run_e2e.py` starts the car's real `ackermann_to_vesc_node`,
`vesc_to_odom_node`, `ackermann_mux`, `joy_teleop` (with the real config
files) and the wizard's real launch file. It fakes only the hardware: a
scripted F710, and a VESC that clips the servo like `vesc_driver` and drives a
simulated car whose true steering linkage and wheel scale differ from
`vesc.yaml`. It then runs every wizard test over the HTTP API, "tape-measures"
each run from the simulated car, and checks:

- the live values are read from all three nodes and every topic is healthy;
- the range check finds the `servo_max` clamp and names software, hardware or both correctly;
- drive-straight runs converge on the true centre offset, through a real write to a copy of `vesc.yaml` and a bringup restart;
- the steering fit recovers the simulated linkage (within 3%) and the wheel gain the simulated scale (within 1%);
- writes, undo, the WebSocket, the page and the JSON-only API all work.

With `--browser`, it also clicks through a steering session in Chromium and
fails on any JavaScript error.

**Terminal 1**, from a built workspace. The domain settings keep it off the
car's ROS graph; never run it with the real driver stack up.

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 tools/odom_calibration_e2e/run_e2e.py
# optional: pip install playwright, then
python3 tools/odom_calibration_e2e/run_e2e.py --browser /path/to/chromium
```

**Working when:** it ends with `N passed, 0 failed` (37 with `--browser`) and
exits 0. It takes about four minutes. Any `FAIL` line names the check; `--keep`
keeps the logs of every node.
