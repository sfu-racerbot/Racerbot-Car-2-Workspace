# racerbot_sim

F1TENTH Gym behind the car's own ROS topics, so the real driving stack can
be run and validated without the car.

Full documentation: **[docs/ros-simulator.md](../../docs/ros-simulator.md)**.

```bash
ros2 launch racerbot_sim sim_auto_map_race_launch.py dashboard:=true
tools/racerbot_sim/run_auto_map_validation.py --scenario all
```

## Safety

`sim_joy_node` publishes a synthetic `/joy` with LB held. The entire
workspace safety policy is that nothing moves unless a human is holding LB
on the physical controller; this node forges that hand. `gym_bridge_node`
publishes an imaginary `/scan` and `/odom`.

Both refuse to publish while any of `vesc_driver_node`,
`ackermann_to_vesc_node`, `vesc_to_odom_node`, `urg_node` or `joy` is on
the ROS graph, and re-check continuously. **Do not defeat that check.**

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
