# Glossary

> **Who this is for:** anyone reading these docs who hits a word they don't know. Written assuming you have never used ROS2 or worked on a robot.
> **Read first:** nothing — this is the bottom of the stack.
> **What's in it:** short definitions of the vocabulary the rest of the docs use.

Skim this once now, then come back whenever a word bites. Definitions are deliberately short; the doc that uses a term in anger will explain it properly in context.

---

## ROS2 basics

### Node

One running program that does one job. `urg_node` reads the LiDAR. `gap_follow_node` decides where to steer. A robot is many small nodes running at once rather than one big program, which is why you'll often have three or four terminals open.

### Topic

A named channel that nodes use to send messages to each other, like `/scan` or `/drive`. Nodes in this workspace never call each other directly — everything goes over topics.

### Publish / subscribe

Publishing means putting messages onto a topic. Subscribing means asking to receive them. A publisher doesn't know or care who is listening, which is why you can add a dashboard without touching the driving code.

### Message

One piece of data on a topic, with a fixed shape. A `LaserScan` message holds one sweep of LiDAR distances. An `AckermannDriveStamped` holds one steering angle and one speed.

### Package

One folder of related code that ROS2 builds and runs as a unit. Everything in `src/` is a package. See [concepts.md](concepts.md#anatomy-of-a-package) for what's inside one.

### Workspace

The whole folder you build in (`~/racerbot-ws`), containing many packages plus the build output.

### Launch file

A Python script that starts several nodes at once with the right settings, so you don't type ten commands. `ros2 launch <package> <file>.py` runs one, and a single `Ctrl+C` stops everything it started.

### Parameter

A setting a node reads at startup, usually from a YAML file in the package's `config/` folder. Tuning happens there rather than in the code.

### `colcon build`

The command that compiles the workspace and lays the result out in `install/`, where ROS2 can find it.

### Sourcing

Running `source <script>` to load settings into your current terminal. Needed in every new terminal — without it, `ros2` either doesn't exist or can't find this workspace's packages. See [concepts.md](concepts.md#why-you-have-to-source-things-and-what-that-means).

### TF / transform / frame

The system that tracks where things are relative to each other. A "frame" is a coordinate system attached to something — the map, the car's body, the LiDAR. TF answers questions like "where is this LiDAR reading in map coordinates?"

---

## This car

### LiDAR

The spinning sensor on the front. It measures the distance to whatever is around it, many times per revolution, and publishes the result on `/scan`.

### Scan

One full sweep of LiDAR distance readings.

### VESC

The motor controller board. It drives the motor and the steering servo, and reports back how far the wheels have turned.

### Odometry / odom

The car's estimate of how far it has travelled and turned, worked out from wheel rotation. Accurate over seconds, drifts over minutes — which is why localization exists.

### Ackermann steering

Steering like a car, where the front wheels turn, rather than like a tank, where wheels spin at different speeds. It means the car has a minimum turning circle (1.22 m on this car) and cannot turn on the spot.

### Mux / multiplexer

`ackermann_mux`, the referee. Several nodes may send drive commands at once; the mux decides which one actually reaches the motor, by priority. Manual driving on `/teleop` outranks autonomy on `/drive`. Full detail in [architecture.md](architecture.md#the-safety-model-read-this-before-writing-autonomy-code).

### Deadman

The **LB** button on the gamepad, which must be held for the car to move. Let go and it stops.

It is the stop that still works when the driving code is wrong, which is why every node in this workspace that can move the car enforces it independently, in its own code, on top of the mux. See the [standing policy](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car).

### Watchdog

A check that stops the car when something it depends on goes quiet — no new pose, no new scan, no new command for too long.

### Latch

A state the code enters and then stays in until something specific clears it. Worth knowing because a safety latch that never clears is a real failure mode in this repo's history — see the postmortems in [racing-autonomy.md](racing-autonomy.md).

### Teleop

Driving by hand with the gamepad. Short for teleoperation.

---

## Maps and driving

### SLAM

Simultaneous Localization and Mapping. Driving the car around while it builds a map of the track *and* works out where it is on that map, at the same time.

### Occupancy grid

The map format: a grid of cells, each marked free, occupied, or unknown. It's what SLAM produces and what localization matches against.

### Localization

Working out where the car is on a map it already has. Different from SLAM, which builds the map as it goes.

### Particle filter

The localization method used here. It keeps thousands of guesses about where the car might be, scores each guess against what the LiDAR currently sees, and keeps the ones that match. Also called Monte Carlo Localization (MCL).

### Racing line

The path around the track the car tries to follow, recorded by driving a good lap, plus a target speed for every point along it.

### Velocity profile

The target speed at each point of the racing line: slow for corners, fast for straights, with braking that starts early enough to actually work. See [racing-autonomy.md](racing-autonomy.md#phase-4-generate-the-velocity-profile).

### Curvature

How sharply the path bends at a point. High curvature means a tight corner, which means a lower safe speed.

### Pure pursuit

The steering method used by the race controller. Pick a point on the racing line a short distance ahead, steer along the arc that reaches it, repeat.

### Lookahead

How far ahead on the path the controller aims. A short lookahead follows the line tightly but wobbles; a long one is smooth but cuts corners.

### Follow-the-gap

The reactive method `gap_follow` uses: look at the LiDAR scan, find the widest open gap, steer at it. It needs no map, which is why it's the starting point both for new drivers and for new code.
