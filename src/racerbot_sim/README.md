# racerbot_sim

> **Who this is for:** someone reading or changing this package's code.
> **Read first:** [docs/ros-simulator.md](../../docs/ros-simulator.md) for how to run it and what it's for.
> **What's in it:** the gym bridge, the forged sensor/joystick nodes, and the interlock that refuses to run beside real hardware.

F1TENTH Gym behind the car's own ROS topics, so the real driving stack can
be run and validated without the car.

Full documentation: **[docs/ros-simulator.md](../../docs/ros-simulator.md)**.

```bash
ros2 launch racerbot_sim sim_auto_map_race_launch.py dashboard:=true
tools/racerbot_sim/run_auto_map_validation.py --scenario all
tools/racerbot_sim/run_gap_follow_validation.py --track indoor_wide
```

## Safety

`sim_joy_node` publishes a synthetic `/joy` with LB held. The entire
workspace safety policy is that nothing moves unless a human is holding LB
on the physical controller; this node forges that hand. `gym_bridge_node`
publishes an imaginary `/scan` and `/odom`.

Both refuse to publish while any of `vesc_driver_node`,
`ackermann_to_vesc_node`, `vesc_to_odom_node`, `urg_node` or `joy` is on
the ROS graph, and re-check before every publish. If the graph query itself
fails they also refuse, and stay refused until a query succeeds. **Do not
defeat that check.**

Know what that check is *not*: it matches those five exact node names, so a
renamed or namespaced driver, or one DDS hasn't discovered yet, reads as
"no hardware". It is a tripwire, not an isolation boundary. On a machine
that can also run the real car, the actual isolation is a separate ROS
domain for the simulator, set in every simulator terminal:

```bash
export ROS_DOMAIN_ID=42   # anything other than the car's domain (default 0)
```

## Layout

| File | What it is |
|---|---|
| `racerbot_sim/tracks.py` | Procedural room-sized closed loops (png + map yaml + centerline csv). No ROS, no Gym |
| `racerbot_sim/sim_bridge.py` | The Gym wrapper, scripted opponents, and dead-reckoned odometry. No rclpy |
| `racerbot_sim/hardware_guard.py` | The real-hardware interlock |
| `racerbot_sim/gym_bridge_node.py` | The ROS node: `/ackermann_cmd` in, `/scan` + `/odom` + TF out |
| `racerbot_sim/sim_joy_node.py` | The synthetic deadman |
| `launch/sim_bringup_launch.py` | Stands in for `f1tenth_stack/bringup_launch.py` |
| `launch/sim_auto_map_race_launch.py` | Includes the real `auto_map_race_launch.py` on top of it |

```bash
python3 -m pytest src/racerbot_sim/test/ -v      # no ROS, no Gym needed
```
