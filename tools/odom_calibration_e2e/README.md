# Calibration wizard end-to-end test

> **Who this is for:** someone changing `odom_calibration`, `vesc.yaml` handling, or the VESC/teleop chain the wizard watches.
> **Read first:** [docs/odom-calibration.md](../../docs/odom-calibration.md).
> **What's in it:** how to run the whole wizard against the car's real software with only the hardware simulated.

`run_e2e.py` starts the car's real `ackermann_to_vesc_node`,
`vesc_to_odom_node`, `ackermann_mux`, `joy_teleop` (with the real config
files, as temporary copies) and the wizard's real launch file.

It fakes only the hardware: a scripted F710, and a VESC (the motor and servo
controller) that clips the servo like `vesc_driver` and drives a simulated
car. That car's true steering linkage and wheel scale differ from
`vesc.yaml`, and its right steering has a hard stop at 24.6°.

The harness runs every wizard test over the HTTP API, "tape-measures"
each run from the simulated car, and checks:

- the live values are read from all four nodes (including `joy_teleop`'s full-stick angle) and every topic is healthy;
- the servo clamp check finds the 2026-10-10 `servo_max: 0.85` problem from settings alone;
- drive-straight runs converge on the true centre offset, through a real write to a copy of `vesc.yaml` and a bringup restart;
- right circles at `servo_max` 0.85, 0.90, 0.95 and 1.00, measured inner tire only, find the stop at 0.95. The report suggests `servo_max` 0.95 and a full-stick angle both sides reach. Each step is the page's own **Write servo_max** action, which raises full stick only when it would fall short, and refuses to go past 1.00;
- the steering fit recovers the simulated linkage (within 3%) and the wheel gain the simulated scale (within 1%);
- one write updates both `vesc.yaml` and `joy_teleop.yaml`, both restarts are noticed, check circles then confirm the calibration, and undo restores both files exactly;
- overrides: a run kept despite warnings, a rear track change (and a refused one), a typed `servo_max` and a refused wrong-sign gain. **Undo everything** returns both files exactly to the start of the session;
- the WebSocket, the page and the JSON-only API all work.

With `--browser`, it also clicks through a steering session in Chromium and
fails on any JavaScript error. That part was updated for the 2026-10-16
rework but has not been run since: Playwright is not installed on car 2.

**Terminal 1**, from a built workspace. The domain settings keep it off the
car's ROS graph; never run it with the real driver stack up.

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=79 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 tools/odom_calibration_e2e/run_e2e.py
# optional: pip install playwright, then
python3 tools/odom_calibration_e2e/run_e2e.py --browser /path/to/chromium
```

**Working when:** it ends with `59 passed, 0 failed` and exits 0. It takes about four minutes. Any `FAIL` line names the check; `--keep`
keeps the logs of every node.
