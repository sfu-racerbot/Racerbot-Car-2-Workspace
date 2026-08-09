# Glossary seed

Starting definitions for `docs/glossary.md`. Written for someone who has never used ROS2 or worked on a robot.

Use these as a base, adjust to how the terms are actually used in this workspace, and add anything a doc needed to gloss. The glossary is the definition of record — individual docs gloss briefly at first use and link here for the full version.

Keep entries short. A glossary someone reads mid-task is a glossary with three-sentence entries.

---

## ROS2 basics

**Node** — one running program that does one job. `urg_node` reads the LiDAR; `gap_follow_node` decides where to steer. A robot is many small nodes running at once rather than one big program.

**Topic** — a named channel that nodes use to send messages to each other, like `/scan` or `/drive`. Nodes never call each other directly in this workspace; everything goes over topics.

**Publish / subscribe** — publishing means putting messages onto a topic; subscribing means asking to receive them. A publisher doesn't know or care who's listening, which is why you can add a dashboard without touching the driving code.

**Message** — one piece of data on a topic, with a fixed shape. A `LaserScan` message holds one sweep of LiDAR distances; an `AckermannDriveStamped` holds one steering angle and speed.

**Package** — one folder of related code that ROS2 builds and runs as a unit. Everything in `src/` is a package.

**Workspace** — the whole folder you build in (`~/racerbot-ws`), containing many packages plus the build output.

**Launch file** — a Python script that starts several nodes at once with the right settings, so you don't type ten commands. `ros2 launch <package> <file>.py` runs one.

**Parameter** — a setting a node reads at startup, usually from a YAML file in `config/`. Tuning happens here rather than in the code.

**`colcon build`** — the command that compiles the workspace and lays the result out in `install/` where ROS2 can find it.

**Sourcing** — running `source <script>` to load ROS2's environment settings into your terminal. Needed in every new terminal; without it, `ros2` either doesn't exist or can't find this workspace's packages.

**TF / transform / frame** — the system that tracks where things are relative to each other. A "frame" is a coordinate system attached to something (the map, the car's base, the LiDAR). TF answers "where is the LiDAR's view in map coordinates?"

## This car

**LiDAR** — the spinning sensor on the front. It measures distance to whatever's around it, many times per revolution, and publishes the result on `/scan`.

**Scan** — one full sweep of LiDAR distance readings.

**VESC** — the motor controller board. It drives the motor and the steering servo, and reports back how far the wheels have turned.

**Odometry / odom** — the car's estimate of how far it has travelled and turned, worked out from wheel rotation. Accurate over seconds, drifts over minutes, which is why localization exists.

**Ackermann steering** — steering like a car (front wheels turn) rather than like a tank (wheels spin at different speeds). It means the car has a turning-circle limit and cannot turn in place.

**Teleop** — driving by hand with the gamepad. Short for teleoperation.

**Mux / multiplexer** — `ackermann_mux`, the referee. Several nodes may send drive commands at once; the mux picks which one actually reaches the motor, by priority. Manual driving on `/teleop` outranks autonomy on `/drive`.

**Deadman** — the LB button on the gamepad, which must be held for the car to move. Let go and it stops. It's the stop that works even when the driving code is wrong. See the safety policy in [architecture.md](architecture.md).

**Watchdog** — a check that stops the car if something it depends on goes quiet — no new pose, no new scan, no new command.

**Latch** — a state the code enters and then stays in until something specific clears it. Worth knowing because a latched safety state that never clears is a real failure mode in this repo's history.

## Maps and driving

**SLAM** — Simultaneous Localization and Mapping. Driving the car around while it builds a map of the track and works out where it is on that map at the same time.

**Occupancy grid** — the map format: a grid of cells, each marked free, occupied, or unknown. What SLAM produces and what localization matches against.

**Localization** — working out where the car is on a map it already has. Different from SLAM, which builds the map as it goes.

**Particle filter** — the localization method here. It keeps thousands of guesses about where the car might be, scores each against what the LiDAR sees, and keeps the ones that match. Also called Monte Carlo Localization (MCL).

**Racing line** — the path around the track the car tries to follow, recorded by driving a good lap, plus a target speed for every point on it.

**Velocity profile** — the target speed at each point of the racing line: slow for corners, fast for straights, with braking that starts early enough to work.

**Curvature** — how sharply the path bends at a point. High curvature means a tight corner, which means a lower safe speed.

**Pure pursuit** — the steering method. Pick a point on the racing line a short distance ahead, steer along the arc that reaches it, repeat. The "short distance ahead" is the lookahead.

**Lookahead** — how far ahead on the path the controller aims. Short lookahead follows the line tightly but wobbles; long lookahead is smooth but cuts corners.

**Follow-the-gap** — the reactive method `gap_follow` uses: look at the LiDAR scan, find the widest open gap, steer at it. Needs no map, which is why it's the starting point for new drivers and new code.
