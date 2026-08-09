# RealSense–LiDAR Perception and Autonomy Research Report

> **Who this is for:** anyone considering combining camera and LiDAR perception on this car. This is background reading, not a procedure.
> **Read first:** [realsense-camera.md](realsense-camera.md) and [architecture.md](architecture.md).
> **What's in it:** a design and research report. It changes no driving behavior — nothing here is running on the car.

**Platform:** Racerbot F1TENTH / ROS 2 Jazzy / NVIDIA Jetson Orin Nano Super  
**Sensors:** Intel RealSense D435i RGB-D camera, Hokuyo UST-10LX 2D LiDAR, VESC odometry and IMU  
**Status:** Design and research report; no driving behavior is changed by this document  
**Date:** 2026-07-26

This report answers two questions:

1. How can the RealSense camera augment the algorithms already in this
   workspace without discarding their tested structure?
2. What new camera–LiDAR methods become practical because the car has an
   NVIDIA Jetson with CUDA, TensorRT, VPI, and enough compute for real-time
   perception?

The central recommendation is to treat the sensors as complementary:

- The Hokuyo remains the primary source for 360-degree planar geometry,
  SLAM/localization, and the independent hard collision layer.
- The RealSense adds height, appearance, local 3D structure, visual motion,
  and semantic identity inside its forward field of view.
- Fusion should happen in explicit perception products—obstacle grids,
  tracked objects, confidence estimates, and local track boundaries—not by
  silently replacing `/scan`.
- Learned perception should initially propose or confirm information. A
  deterministic collision guard and the current LB deadman remain authoritative.

The most useful near-term project is a **camera-derived 2.5D obstacle layer
plus speed-aware swept-path collision check**. The best medium-term racing
project is **RGB-D/LiDAR opponent tracking in track-relative coordinates**.
The strongest longer-term research direction is a **semantic local occupancy
map feeding a GPU-parallel MPPI controller**, with pure pursuit retained as a
fallback.

## 1. Improving the Current Algorithms

### 1.1 Current baseline and constraints

The workspace already has a coherent autonomy architecture:

- `gap_follow` consumes `/scan` and `/odom`, performs footprint-aware
  clearance checks, LiDAR TTC, disparity extension, safety bubbling, and gap
  selection, then publishes `/drive`.
- `pure_pursuit` follows a saved `(x, y, speed)` racing line at 40 Hz, with
  LiDAR-based emergency stopping, reactive avoidance, map-subtraction
  opponent detection, track-relative opponent tracking, and lateral target
  offsets for overtaking.
- `slam_toolbox` and `particle_filter` use the 2D LiDAR for mapping and
  localization.
- Every node capable of moving the car independently enforces the LB
  deadman, and `ackermann_mux` remains the final command arbiter.

Relevant local references are:

- [architecture.md](architecture.md)
- [racing-autonomy.md](racing-autonomy.md)
- [gap-follow implementation](../src/gap_follow/gap_follow/gap_follow_node.py)
- [pure-pursuit implementation](../src/pure_pursuit/pure_pursuit/pure_pursuit_node.py)
- [RealSense status](realsense-camera.md)

The camera is already usable as a ROS sensor:

| Capability | Current state |
|---|---|
| RGB | `/camera/camera/color/image_raw`, verified at 424×240 @ 15 fps |
| Depth | `/camera/camera/depth/image_rect_raw`, verified at 424×240 @ 15 fps |
| Point cloud | Disabled to save CPU and ROS serialization overhead |
| RGB-D synchronization/alignment | Supported by the vendored driver but not enabled in the car launch |
| D435i IMU | Unavailable with the current RSUSB backend |
| Camera extrinsic | `base_link -> camera_link` is still a zero-translation placeholder |

The [D435i specification](https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html)
describes active-stereo depth, an approximately 87°×58° depth field of view,
and a nominal approximately 0.3–3 m operating range. Those figures are a useful
design envelope, not a guarantee: range quality depends on distance, texture,
lighting, reflectivity, exposure, vibration, and calibration.

The live Jetson audit for this report found:

| Resource | Live system |
|---|---|
| Module | Jetson Orin Nano Super, 8 GB shared memory |
| Power mode | `MAXN_SUPER` |
| CUDA | 13.2 |
| TensorRT | 10.16.2 |
| VPI | 4.1.3 |
| Python OpenCV | CPU/TBB build; `cv2.cuda` reports no CUDA device |
| PyTorch / CuPy / ONNX Runtime | Not currently installed |
| Storage | NVMe with ample free space |

NVIDIA lists the Orin Nano Super at up to 67 sparse INT8 TOPS, 33 dense INT8
TOPS, 17 FP16 TFLOPS, and 102 GB/s memory bandwidth, but it has no DLA
([NVIDIA platform description](https://developer.nvidia.com/blog/nvidia-jetson-orin-nano-developer-kit-gets-a-super-boost/)).
That leads to four deployment conclusions:

1. TensorRT inference belongs on the GPU; there is no DLA to offload it to.
2. The Nano lacks the PVA available on Orin NX/AGX. VPI CUDA, VIC, and OFA
   backends are relevant; PVA-based designs are not.
3. Because CPU OpenCV is not CUDA-enabled, production GPU pipelines should
   use VPI, CUDA C++, or TensorRT directly instead of assuming `cv2.cuda`.
4. Training should happen off-car. Export an ONNX model and build/benchmark
   its TensorRT engine on the Jetson.

The current 15 fps camera period is 66.7 ms, whereas pure pursuit runs every
25 ms. At 4 m/s the car travels about 0.267 m between camera frames before
processing delay is included. Camera information therefore needs timestamps,
motion compensation, prediction, and explicit staleness handling. It must not
be treated as a synchronous 40 Hz safety sensor.

### 1.2 Recommended integration architecture

Do not put computer vision directly into `gap_follow_node.py` or
`pure_pursuit_node.py`. Add a sensor-facing package, tentatively
`racerbot_perception`, with ROS-free math separated from ROS wiring in the same
style as the current controllers.

```text
RealSense depth + CameraInfo ──► depth_geometry_node
                                      ├── /perception/camera_scan
                                      ├── /perception/depth_obstacles
                                      ├── /perception/depth_health
                                      └── /perception/local_height_grid

RealSense color + aligned depth ─► semantic_perception_node
                                      ├── /perception/visual_detections
                                      ├── /perception/track_boundaries
                                      └── /perception/debug_image

Hokuyo /scan ────────────────────► fusion/tracking_node
camera products ─────────────────►      ├── /perception/fused_obstacles
map + vehicle pose ──────────────►      ├── /perception/tracked_objects
                                           └── /perception/local_costmap

raw /scan ───────────────────────► existing independent LiDAR safety
fused products ──────────────────► gap_follow / pure_pursuit enhancements
```

Design rules:

- Keep the raw LiDAR topic named `/scan`. Publish camera and fused scans under
  new names. This prevents camera failures from contaminating SLAM and
  particle-filter localization.
- Perception nodes publish observations and diagnostics, never `/drive`.
  Driving nodes retain the LB deadman and final decision logic.
- Use standard messages where they fit: `sensor_msgs/LaserScan`,
  `sensor_msgs/PointCloud2`, `nav_msgs/OccupancyGrid`,
  `diagnostic_msgs/DiagnosticArray`, and visualization markers. A small custom
  tracked-object message is justified if covariance, identity, dimensions, and
  track-relative state cannot be represented cleanly.
- Use sensor-data QoS, queue depth one, and “newest frame wins.” Backlogged
  images are worse than dropped images on a racing vehicle.
- Carry the original measurement timestamp through every output. Record both
  acquisition age and processing latency.
- Make camera dependence configurable. During rollout, camera data is
  supplemental: stale camera data disables camera augmentation but does not
  invalidate healthy LiDAR. If a future behavior genuinely depends on vision,
  its stale-data policy must slow or stop explicitly.

ROS Jazzy already provides useful building blocks. `depth_image_proc` can
produce point clouds from depth images, and `pointcloud_to_laserscan` can crop
by height and project a cloud into `LaserScan`
([depth_image_proc](https://docs.ros.org/en/ros2_packages/jazzy/api/depth_image_proc/doc/index.html),
[pointcloud_to_laserscan](https://docs.ros.org/en/ros2_packages/jazzy/api/pointcloud_to_laserscan/)).
They are good baselines. A custom direct depth-to-polar-grid component will
eventually be more efficient because it can avoid generating and serializing a
full point cloud.

### 1.3 Calibration and synchronization are the first algorithm

No fused method is trustworthy until the camera pose is known. The current
static transform is a placeholder in
[realsense_camera_launch.py](../src/racerbot_launch/launch/realsense_camera_launch.py).
The required calibration is six-dimensional:

```text
T_base_camera = [x, y, z, roll, pitch, yaw]
```

Measuring only translation is insufficient. A one-degree yaw or pitch error
produces a growing lateral or vertical error with distance and can move an
obstacle into the wrong side of a pass corridor.

Recommended calibration procedure:

1. Measure camera position relative to the rear-axle `base_link` origin.
2. Estimate roll and pitch from a flat ground plane observed in depth.
3. Estimate yaw and refine translation with a large vertical target visible
   in both LiDAR and RGB-D at several distances and bearings.
4. Optimize the transform to minimize camera–LiDAR point-to-plane or
   point-to-line residuals over all target poses.
5. Validate on objects not used for fitting. Report median and 95th-percentile
   lateral reprojection error at 0.5, 1, 2, and 3 m.
6. Save the transform as parameters and document the physical mount so it can
   be rechecked after impacts.

A 2D LiDAR constrains calibration less strongly than a 3D LiDAR. Multiple
target poses, a known ground plane, and physical measurements are therefore
important. Do not rely on one checkerboard pose.

Time offset also matters. Move a board laterally through both sensor fields,
extract the nearest-range trace from each sensor, and cross-correlate the
traces to estimate a systematic timestamp offset. During driving, use odometry
to transform older camera points from their measurement-time base frame to the
controller's current base frame.

For color-guided depth, enable the driver's `enable_sync` and
`align_depth.enable` options and consume the aligned-depth topic. The vendored
driver also supports decimation, spatial, temporal, and hole-filling filters.
Intel describes these post-processing tools in its
[D400 post-processing guide](https://www.intel.com/content/www/us/en/content-details/842031/depth-post-processing-for-intel-realsense-d400-depth-cameras.html).

Use two depth products rather than one aggressively smoothed product:

- **Collision depth:** current or minimally filtered depth, no indefinite
  persistence and no assumption that filled pixels are measured. It may be
  noisy, but it does not preserve a disappeared object or invent free space.
- **Planning/visualization depth:** decimated, edge-preserving spatially
  filtered, and lightly temporally filtered for a stable local map.

Never interpret zero/invalid depth as “free.” It means unknown. Hole filling
may improve display quality, but a filled value must carry lower confidence
than an actual stereo measurement.

Stereo uncertainty should also affect fusion. With focal length \(f\), baseline
\(B\), disparity \(d\), and disparity error \(\sigma_d\):

\[
Z = \frac{fB}{d}, \qquad
\sigma_Z \approx \frac{Z^2}{fB}\sigma_d
\]

Depth uncertainty therefore grows approximately quadratically with range.
Near camera observations can strongly constrain a local obstacle; distant
depth should receive less weight than a valid Hokuyo return.

### 1.4 Improve `gap_follow`

#### 1.4.1 Add a height-aware virtual scan

The Hokuyo sees one horizontal slice. It can miss:

- an obstacle above the scan plane;
- a low object below the plane;
- the body of another RC car when the beam passes through an opening;
- sloped or irregular geometry that produces a weak planar return.

Deproject the depth image using `CameraInfo`, transform points to `base_link`,
remove the ground, retain points inside a measured vehicle-relevant height
band, and reduce them into angular bins:

\[
r_\text{camera}(\theta)
  = \min_{\mathbf p \in \text{valid height bin at }\theta} \|\mathbf p_{xy}\|
\]

Require a spatial cluster, not a single pixel, before generating a range.
Publish both the range and support count/confidence.

Fuse only where the camera has a valid observation:

\[
r_\text{fused}(\theta) =
\begin{cases}
\min(r_\text{lidar}(\theta), r_\text{camera}(\theta)),
  & \text{both valid}\\
r_\text{lidar}(\theta), & \text{camera unknown}\\
r_\text{camera}(\theta), & \text{LiDAR unknown and camera sufficiently confident}\\
\text{unknown}, & \text{neither valid}
\end{cases}
\]

Initially, the final case where only the camera is valid should be allowed to
add an obstacle but not certify open space. This is conservative and matches
the intended role of the camera.

The virtual scan can enter `gap_follow` immediately before disparity extension,
allowing the existing footprint inflation, safety bubble, and gap-width checks
to remain useful. Keep the original LiDAR hard-clearance and TTC checks
independent so a camera bug cannot weaken them.

#### 1.4.2 Replace fixed forward thresholds with a swept-path speed governor

The current gap follower already has footprint-aware TTC. Camera depth permits
a true 3D version along the path the car is predicted to take.

For steering angle \(\delta\) and wheelbase \(L\):

\[
\kappa = \frac{\tan\delta}{L}
\]

The nominal bicycle-model centerline at arc length \(s\) is:

\[
x(s)=\frac{\sin(\kappa s)}{\kappa},\qquad
y(s)=\frac{1-\cos(\kappa s)}{\kappa}
\]

with the straight-line limit used as \(\kappa \to 0\). Inflate this curve by
half the vehicle width, front/rear extent, localization uncertainty, and a
speed-dependent margin. Test camera and LiDAR obstacle clusters against this
swept volume instead of a fixed rectangular image region.

The stopping criterion should use measured speed and measured braking
performance:

\[
d_\text{stop}(v)
  = v\tau_\text{total}
  + \frac{v^2}{2a_\text{brake}}
  + m
\]

where \(\tau_\text{total}\) includes sensor period, processing, ROS delivery,
controller scheduling, command transport, and actuator response. The
corresponding speed ceiling for available distance \(d\) is:

\[
v_\text{cap}
  = -a_\text{brake}\tau_\text{total}
    + \sqrt{
      (a_\text{brake}\tau_\text{total})^2
      + 2a_\text{brake}\max(0,d-m)
    }
\]

Measure \(a_\text{brake}\) on the actual car at low speed; do not select it
from a generic RC-car estimate.

Use a two-tier action:

- warning distance: smoothly cap speed using \(v_\text{cap}\);
- unavoidable collision distance: publish zero speed through the existing
  deadman-gated controller.

This avoids the oscillation created by a binary stop/release threshold.

#### 1.4.3 Improve gap scoring with stability and semantics

The current best-gap score is mainly gap width multiplied by average depth.
It can be extended without changing the overall algorithm:

\[
J_\text{gap} =
 w_d \bar r
 w_w W_\text{physical}
 w_h H_\text{clearance}
 w_p \cos(\theta_\text{gap}-\theta_\text{preferred})
-w_s |\theta_\text{gap}-\theta_\text{previous}|
-w_u q_\text{unknown}
-w_m q_\text{moving}
\]

Terms contributed by the camera are:

- vertical clearance \(H_\text{clearance}\);
- fraction of unknown depth \(q_\text{unknown}\);
- moving-object risk \(q_\text{moving}\), estimated from depth change or
  tracked detections;
- a semantic penalty for opponent/car pixels, people, or track-exterior
  regions.

The steering-change penalty reduces left/right gap switching. It should be
small enough that a newly detected obstacle still dominates immediately.

Start with geometric terms only. Add semantics after collecting data proving
that a classifier improves decisions rather than merely looking impressive in
the dashboard.

#### 1.4.4 Improve moving-obstacle TTC

Current LiDAR TTC assumes static obstacles and ego closing speed. RGB-D can
estimate radial object motion:

\[
\text{TTC}
  = \frac{r-r_\text{body}}
         {\max(\epsilon, v_\text{ego}\cos\theta-v_{\text{object,radial}})}
\]

Estimate object velocity from a filtered 3D track, not frame-to-frame minimum
depth. This distinguishes a stationary wall from an opponent moving in the
same direction and prevents unnecessary hard braking behind a moving car while
still enforcing a safe following time.

At 15 fps, use a constant-velocity Kalman filter and report covariance. If the
velocity estimate is uncertain, fall back to the static-obstacle assumption.

### 1.5 Improve `pure_pursuit`

#### 1.5.1 Bring the stronger gap-follow collision model into pure pursuit

Pure pursuit currently has a fixed forward `emergency_stop_distance`, while
gap follow already has footprint-aware clearance and TTC. Before adding
complex vision, reuse the tested ROS-free gap safety math in pure pursuit:

- subscribe to `/odom` for measured speed;
- convert LiDAR range to body clearance using the sensor offset and vehicle
  rectangle;
- use speed-dependent TTC and stopping distance;
- use the camera swept-path result as an additional obstacle source;
- keep raw-LiDAR staleness fail-safe behavior.

This also improves opponent closing-rate estimation. The current overtake
logic compares opponent progress rate with the **profiled** ego speed, not
measured ego speed. Measured velocity is the correct quantity for “are we
actually catching them?”

The final speed command should become:

\[
v_\text{command} =
\min(
v_\text{profile},
v_\text{obstacle},
v_\text{visibility},
v_\text{localization},
v_\text{controller limit}
)
\]

Every term should expose a diagnostic explaining which constraint is active.

#### 1.5.2 Fuse camera and LiDAR for opponent recognition

The current map-subtraction detector already supplies an excellent geometric
proposal: a cluster that is closer than the static map predicts. The camera
should first **confirm and refine that proposal**, not run an expensive
full-frame detector unconditionally.

Recommended staged detector:

**Stage A — training-free RGB-D geometry**

1. Detect a dynamic LiDAR cluster using the existing map subtraction.
2. Project the cluster bearing and range into the calibrated depth/color image.
3. Find the connected depth component near the projected range.
4. Estimate its 3D centroid, width, height, and visible footprint.
5. Reject components inconsistent with an RC car or the drivable region.

This directly reduces wall-corner and thin-post false positives.

**Stage B — lightweight semantic confirmation**

1. Crop an image ROI around the projected cluster.
2. Run a small binary RC-car/background classifier or instance segmenter.
3. Combine semantic probability with geometric likelihood.

ROI inference is much cheaper than full-frame detection and naturally uses
LiDAR to focus the GPU. For recovery when LiDAR temporarily misses the car,
run a full-frame detector at a lower rate.

A practical starting detector is YOLOX-Nano: the published model has about
0.91 million parameters and 1.08 GFLOPs, and its authors provide ONNX and
TensorRT deployment paths
([YOLOX paper](https://arxiv.org/abs/2107.08430)). It would still require a
custom RC-car dataset; generic “car” weights should not be assumed to
recognize 1/10-scale vehicles from unusual viewpoints.

Each fused observation should contain:

```text
timestamp
position and covariance in base_link
width / height / visible extent
LiDAR geometric score
depth support and invalid fraction
visual class probability
source mask: lidar | depth | RGB
```

Associate observations with predicted tracks using position/bearing
Mahalanobis distance and dimension consistency. A simple Kalman filter is
enough for one or a few opponents; a ByteTrack-style association strategy is
an option when visual detections become frequent or partially occluded
([ByteTrack paper](https://www.ecva.net/papers/eccv_2022/papers_ECCV/papers/136820001.pdf)).

#### 1.5.3 Track opponents in Frenet coordinates with uncertainty

The current opponent tracker already reasons in track arc length. Extend its
state from scalar progress to:

\[
\mathbf x_\text{opp}
  = [s,\ d,\ \dot s,\ \dot d]^\top
\]

where \(s\) is distance along the raceline and \(d\) is lateral displacement.
This provides:

- true measured closing rate using ego odometry;
- lateral intent—staying on line, moving left, or moving right;
- covariance that grows during occlusion;
- predicted occupancy over the overtake horizon.

Keep a track through short camera/LiDAR dropouts, but expand its collision
envelope with covariance. Abandon a pass when predicted uncertainty makes both
candidate corridors unsafe.

The D435i's forward field of view cannot solve side/rear blindness. During the
alongside phase, motion prediction remains necessary. Camera semantics improve
the entry to the pass and the estimate of lateral extent; they do not create a
360-degree tracker.

#### 1.5.4 Replace one-time pass-side choice with corridor evaluation

The current controller compares small LiDAR windows on each side of the
opponent and commits to one lateral offset. Use the fused local grid to score
complete left and right candidate corridors:

\[
J_\text{pass} =
w_c C_\text{clearance}
w_p C_\text{progress}
w_r C_\text{rejoin}
-w_k C_\text{curvature}
-w_u C_\text{uncertainty}
-w_b C_\text{boundary violation}
\]

Evaluate:

- minimum clearance along the predicted ego path;
- predicted opponent occupancy;
- track boundary or wall clearance;
- curvature and steering feasibility;
- the ability to return to the raceline after the pass.

Re-evaluate safety every tick, but add hysteresis so minor noise does not flip
the chosen side. The hard collision layer always overrides the strategic pass.

#### 1.5.5 Add camera-based local raceline correction

If the course has visible tape, cones, painted edges, or visually consistent
barriers, train or hand-design a boundary detector:

1. Segment left boundary, right boundary, and drivable surface in RGB.
2. Use aligned depth and the camera pose to project pixels to the ground plane.
3. Reject points inconsistent with LiDAR wall geometry.
4. Fit local robust splines or polynomials.
5. Estimate a camera-derived centerline and confidence.
6. Apply only a bounded lateral correction to the map raceline:

\[
\mathbf p_\text{target}^\ast
 = \mathbf p_\text{map}
 + \operatorname{clip}(w(q)\Delta d,\,-d_\max,d_\max)\mathbf n
\]

where \(q\) includes segmentation confidence, depth support, LiDAR agreement,
and temporal consistency.

This is useful for:

- small map/localization bias;
- barriers shifted after mapping;
- recovering toward the visible track after a minor deviation;
- determining real track width for overtaking.

It must not pull the car away from a well-localized raceline merely because a
shadow resembles tape. Start in shadow mode and log corrections without using
them.

A lightweight embedded segmentation network such as Fast-SCNN is a reasonable
research baseline; it was explicitly designed for low-memory, real-time
embedded segmentation
([Fast-SCNN paper](https://arxiv.org/abs/1902.04502)). Classical HSV/color
thresholding should be benchmarked first when the course markings are
controlled—it may be more reliable and far easier to validate.

#### 1.5.6 Make speed depend on perception horizon and confidence

The offline velocity profile knows curvature but not current visibility,
traffic, depth quality, localization uncertainty, or track changes.

Define a safe visibility horizon \(d_\text{visible}\) as the farthest distance
along the swept path for which:

- LiDAR/camera coverage is valid;
- the path is not dominated by unknown cells;
- a track boundary or map corridor is available;
- sensor age is below its limit.

Compute a visibility speed ceiling using the same braking equation as the
collision guard. Reduce speed smoothly when:

- depth invalid fraction rises;
- camera exposure saturates;
- map and visual boundaries disagree;
- localization covariance or cross-track residual rises;
- opponent prediction uncertainty grows.

The camera should initially only lower speed, never raise it above the offline
profile. That makes perception errors conservative.

### 1.6 Improve mapping, localization, and automatic map-to-race

#### 1.6.1 RGB-D odometry as a watchdog

Run RGB-D or stereo visual odometry in evaluation mode and compare its relative
motion over short windows with:

- VESC wheel odometry;
- LiDAR/localization motion;
- commanded steering.

Use disagreement to flag:

- particle-filter jumps;
- wheel slip;
- a camera mount that has moved;
- stalled or stale sensors.

Do not immediately replace particle-filter pose with visual pose. Visual
odometry can fail under blur, low texture, repeated patterns, abrupt exposure,
and dynamic opponents. At the present 15 fps, it should first be considered a
diagnostic signal. A 30 fps camera profile should be benchmarked before relying
on it for fast motion.

ORB-SLAM3 supports RGB-D and stereo modes
([ORB-SLAM3 paper](https://doi.org/10.1109/TRO.2021.3075644)), while RTAB-Map
supports visual and LiDAR SLAM configurations
([RTAB-Map paper](https://arxiv.org/abs/2403.06341)). Neither is currently
installed in this workspace, so both should be evaluated offline from bags
before adding a race-day dependency.

#### 1.6.2 Fuse wheel odometry, VESC IMU, and visual motion

The RealSense IMU is unavailable on this backend, but the VESC publishes IMU
topics. After calibrating axis orientation, bias, noise, and timestamp quality,
an EKF/UKF can fuse:

- wheel-derived forward velocity;
- VESC yaw rate and acceleration;
- RGB-D relative pose/velocity;
- optionally LiDAR scan-matching odometry.

This fused local odometry would improve motion compensation and TTC even if the
global particle-filter architecture remains unchanged. `robot_localization`
provides a conventional ROS EKF baseline; it is not currently installed and
must not be inserted without covariance calibration.

#### 1.6.3 Add visual loop-closure evidence to automatic mapping

The automatic map-to-race supervisor currently decides that a lap closed from
SLAM geometry. Appearance matching can provide a second vote:

- extract compact visual descriptors at intervals;
- compare the current view with the start region;
- require spatial proximity plus sustained descriptor similarity;
- use the result to confirm—not unilaterally declare—loop closure.

This reduces false closures in geometrically repetitive tracks. It also gives
operators a visual keyframe for reviewing the generated lap.

#### 1.6.4 Do not feed camera scans into the static LiDAR map by default

A camera-derived scan can improve local collision avoidance while degrading
2D mapping: it may contain movable objects at many heights, ground artifacts,
and a narrow forward field of view. Keep these separate:

- `/scan`: raw Hokuyo for SLAM and particle filter;
- `/perception/camera_scan`: camera obstacle evidence;
- `/perception/fused_obstacles`: local planning and safety;
- semantic/dynamic objects: excluded from the static map.

Only consider map fusion after recording evidence that it improves localization
on the car's real courses.

### 1.7 Jetson implementation strategy

#### 1.7.1 Assign work to the right hardware

| Workload | Recommended implementation |
|---|---|
| 424×240 depth validation, ROI, deprojection prototype | Python/NumPy is adequate initially |
| Production depth-to-polar grid and clustering | C++ component; SIMD first, custom CUDA only if profiling justifies it |
| Resize, color conversion, warps | VPI/VIC where available |
| Optical flow / feature tracking | VPI OFA or CUDA backend; benchmark quality and latency |
| RC-car detection or segmentation | TensorRT FP16; consider INT8 after calibration |
| Local occupancy/ESDF and trajectory rollout | CUDA C++ or standalone `nvblox` experiment |
| ROS image/point-cloud transport | Composed C++ components and intra-process communication |

NVIDIA VPI supports asynchronous streams and zero-copy where buffer layout
permits, with CUDA on Jetson, VIC on all Jetsons, and OFA on Orin
([VPI architecture](https://docs.nvidia.com/vpi/basic_concepts.html)). PVA is
not available on the Orin Nano and should not appear in the design.

TensorRT supports mixed and quantized precision
([TensorRT documentation](https://docs.nvidia.com/deeplearning/tensorrt/latest/index.html)).
Use FP16 first because it is straightforward to validate. Move to explicit
INT8 quantization only after assembling a calibration set covering real
lighting, tracks, motion blur, opponents, and camera exposure. Quantization
reduces memory and can improve speed, but accuracy must be re-measured
([TensorRT quantization guidance](https://docs.nvidia.com/deeplearning/tensorrt/latest/inference-library/work-with-quantized-types.html)).

#### 1.7.2 Prefer multi-rate perception

Not every task needs to run at the same rate:

- depth collision geometry: every depth frame;
- object tracker prediction: every controller tick;
- LiDAR/depth measurement updates: whenever data arrives;
- ROI classifier: only when LiDAR/depth proposes an object;
- full-frame detector: lower-rate recovery;
- boundary segmentation: 10–15 Hz with temporal tracking;
- diagnostics/debug images: 2–5 Hz.

This preserves GPU margin and reduces latency. A tracker can predict at 40 Hz
between 15 Hz camera measurements, but its covariance must grow with age.

#### 1.7.3 Eliminate avoidable race-day load

The camera launch currently always starts the MJPEG browser bridge. Make the
stream optional for race launches. JPEG encoding and dashboard delivery are
valuable during development but do not improve control. During a race:

- disable the MJPEG bridge unless a human needs it;
- do not publish full point clouds unless a consumer exists;
- throttle debug overlays and markers;
- avoid Python copies of full images in multiple nodes;
- use bounded queues.

#### 1.7.4 Measure instead of assuming

For every pipeline report:

- camera acquisition-to-output latency, p50/p95/p99;
- data age at the controller;
- dropped frames and stale events;
- CPU per core, GPU utilization, memory, swap;
- TensorRT engine memory and inference latency;
- thermal state and clocks over a full race-length run;
- controller deadline misses.

Use `tegrastats`, TensorRT `trtexec`, and Nsight Systems. Benchmark with
LiDAR, localization, controller, recording, and dashboard in the actual
combination intended for testing—not perception in isolation.

The official Isaac ROS 4.4 platform matrix supports ROS 2 Jazzy on
Jetson Thor/JetPack 7.1, not Orin Nano
([Isaac ROS supported platforms](https://nvidia-isaac-ros.github.io/v/release-4.4/repositories_and_packages/isaac_ros_common/index.html)).
The older 3.2 release supports Orin but targets ROS 2 Humble and JetPack
6.1/6.2. Therefore, do not design the first implementation around Isaac ROS
packages. Native TensorRT/VPI/CUDA is installed and compatible. Standalone
libraries can be tested individually.

### 1.8 Recommended current-stack backlog

| Priority | Improvement | Racing value | Effort | Main risk |
|---|---|---:|---:|---|
| P0 | Measure 6-DoF camera extrinsic and time offset | Foundational | Low–medium | Bad calibration invalidates all fusion |
| P0 | Record synchronized RGB, depth, LiDAR, odom, pose, and commands | Foundational | Low | Storage/data management |
| P1 | Camera health metrics and shadow-mode depth obstacles | High safety insight | Low | Treating unknown as free |
| P1 | Port footprint-aware TTC and measured speed into pure pursuit | High | Low–medium | Brake model must be measured |
| P1 | Height-aware virtual scan for gap follow | High | Medium | Ground/edge artifacts |
| P1 | Swept-path camera speed governor | High | Medium | Latency and false stops |
| P2 | Training-free RGB-D confirmation of LiDAR opponents | High | Medium | Partial views and calibration |
| P2 | Frenet opponent tracker with covariance and measured ego speed | High | Medium | Incorrect association |
| P2 | Left/right pass corridor scoring from fused local grid | High | Medium–high | Track-boundary confidence |
| P2 | Temporal/semantic gap scoring | Medium | Medium | Over-tuning |
| P3 | Camera-derived raceline correction | Track-dependent | Medium–high | Shadows/appearance shift |
| P3 | RGB-D odometry watchdog and wheel-slip detector | Medium | Medium–high | Blur/low texture |
| P3 | Visual loop-closure confirmation | Medium | Medium | Repetitive scenes |

The suggested first deliverable is intentionally narrow:

```text
depth image
  -> calibrated ground removal
  -> clustered 2.5D obstacles
  -> camera LaserScan + health
  -> shadow-mode comparison with LiDAR decisions
  -> optional speed cap after validation
```

It exercises calibration, depth quality, timing, ROS interfaces, logging, and
safety without requiring a training dataset.

## 2. New Camera–LiDAR Approaches

This section moves beyond adding observations to the current controllers. Each
proposal is designed to preserve a deterministic safety envelope and to fit an
8 GB Orin Nano, but each requires a separate experimental control layer and
must earn trust through bag replay, simulation, wheels-off testing, and
low-speed physical validation.

### 2.1 Semantic 2.5D local map plus GPU-parallel MPPI

#### Research question

Can a rolling camera–LiDAR map and sampling-based predictive controller drive
faster and more smoothly than “one lookahead point plus reactive override,”
especially around dynamic obstacles?

#### Representation

Maintain an egocentric rolling grid around the car. A practical initial grid
is roughly 8–10 m square with 0.05–0.10 m cells. Each cell stores:

```text
occupancy log-odds
minimum and maximum observed height
ground/traversability estimate
semantic class probabilities
dynamic probability or velocity
last observation time
sensor-source mask
uncertainty
```

Update the grid with:

- 2D LiDAR rays: strong planar free/occupied evidence;
- RGB-D points: height and near-field surface evidence;
- track segmentation: drivable/non-drivable semantic evidence;
- tracked opponents: predicted dynamic occupancy;
- the static occupancy map: prior wall evidence.

Ray clearing must be sensor-specific. A missing depth pixel cannot clear a
cell. Dynamic objects should decay or move rather than be burned into the
static layer.

#### Planner

Use the kinematic bicycle model to roll out thousands of candidate steering
and acceleration sequences on the GPU over a 1–2 s horizon. Model Predictive
Path Integral control is a natural match because its sampling workload is
parallel; the original MPPI work explicitly uses GPU-parallel sampling
([MPPI paper](https://doi.org/10.2514/1.G001921)).

Candidate cost:

\[
J =
w_e e_\text{contour}^2
+w_l e_\text{lag}^2
+w_v(v-v_\text{target})^2
+w_o\phi(d_\text{obstacle},\sigma)
+w_b\phi(d_\text{boundary})
+w_u\|\Delta u\|^2
+w_j\|\Delta^2 u\|^2
+w_p\phi(\text{opponent prediction})
\]

Important terms:

- progress along the track;
- contour error from the raceline;
- collision cost using the full footprint;
- chance constraint using occupancy/opponent covariance;
- track boundary violation;
- steering rate, acceleration, and jerk;
- terminal ability to return to the raceline.

The controller applies only the first action, receives new observations, and
replans.

#### Why it is feasible

The local grid is small, and rollout dynamics/costs are highly parallel.
CUDA 13.2 is installed. The GPU is better used for many simple trajectory
evaluations than for a giant end-to-end network.

`nvblox` is an NVIDIA open-source CUDA library for RGB-D TSDF/ESDF mapping
([project](https://github.com/nvidia-isaac/nvblox),
[paper](https://arxiv.org/abs/2311.00626)). Its standalone library is worth an
isolated compatibility experiment, but its Isaac ROS integration must not be
assumed compatible with this Orin Nano/JetPack 7.2 system. A simple custom
2.5D occupancy grid is the lower-risk first version.

#### Safety architecture

- MPPI publishes a candidate command, not directly to the VESC.
- An independent deadman-gated safety supervisor checks raw LiDAR, depth
  swept-path clearance, command age, and steering/speed limits.
- Pure pursuit remains a selectable fallback.
- If GPU planning misses a deadline, reuse no stale aggressive action:
  command a controlled slowdown/stop.

#### Feasibility

**Medium–high effort, high potential.** This is the best flagship research
project after the perception foundation is reliable.

### 2.2 Object-centric multimodal world model and multi-hypothesis overtaking

#### Research question

Can the car plan safer, earlier overtakes by explicitly predicting multiple
opponents and evaluating maneuver alternatives rather than applying one fixed
lateral offset?

#### World model

Track each object in Frenet coordinates:

\[
\mathbf x_i =
[s_i,d_i,\dot s_i,\dot d_i,\text{length},\text{width}]^\top
\]

with covariance and class probability. Fuse:

- LiDAR map-subtraction clusters;
- RGB-D 3D components;
- semantic RC-car detections;
- optical/depth motion;
- track-boundary constraints.

Use a constant-velocity model initially. Add constant-acceleration or
interaction models only if logs show systematic prediction error.

#### Behavior planner

Generate a small maneuver set:

1. remain on line and follow;
2. pass left;
3. pass right;
4. abort and return behind;
5. emergency slow/stop.

For each maneuver, generate a smooth lateral trajectory in \(s,d\), predict
ego and opponent occupancy with uncertainty, and score:

- expected progress;
- collision probability;
- minimum clearance;
- boundary feasibility;
- curvature/steering limits;
- rejoin quality;
- robustness if the opponent accelerates or changes lane.

This discrete layer can feed either pure pursuit targets or MPPI. It is more
explainable than a direct neural policy and handles a multi-car field better
than “closest cluster wins.”

#### Creative extension: intent hypotheses

Maintain several opponent hypotheses—constant speed, braking, or moving toward
the same gap—and evaluate the pass against all of them. Use the worst credible
case or a risk-sensitive weighted score. Do not attempt adversarial blocking;
the workspace's conservative “drive your own line” policy should remain the
default when intent is uncertain.

#### Feasibility

**Medium–high effort, high race value.** It depends on reliable calibration
and tracking but not on a large neural network.

### 2.3 Vision-guided cone or boundary racing without a pre-recorded line

#### Research question

Can the car build and race a course online when it is defined by visible
cones, tape, or barriers rather than a previously mapped occupancy grid?

#### Algorithm

1. Detect left/right boundary classes in RGB.
2. Use depth to obtain 3D boundary points.
3. Use LiDAR to recover geometry where depth is invalid and to reject visual
   false positives.
4. Track landmarks across frames using ego motion.
5. Pair or order left/right landmarks.
6. Construct a local centerline using robust splines or Delaunay-style
   midpoint selection.
7. Estimate corridor width and uncertainty.
8. Feed the local path to pure pursuit or MPPI.
9. Accumulate landmarks into a map and optimize a global raceline after the
   first lap.

This provides a genuinely different autonomy mode: online track discovery
with semantic boundary identity. LiDAR supplies metric structure; vision
supplies “which side/type of boundary is this?”

#### Failure modes

- no distinctive markings;
- color shift, shadows, glare, or visually similar background objects;
- depth holes at cone edges;
- ambiguous left/right association;
- short visual horizon around corners.

The method is compelling only for courses with controlled visual structure.
It should not replace the current map pipeline on plain-wall tracks.

#### Feasibility

**Medium effort for controlled tape/cones; high effort for general tracks.**
Classical color segmentation may beat deep learning on controlled courses.

### 2.4 Bounded residual learning on top of classical control

#### Research question

Can a small learned model correct repeatable pure-pursuit errors without
giving a neural network unrestricted control authority?

#### Safe residual design

Keep pure pursuit as the baseline:

\[
u_\text{final}
=u_\text{classical}
+g(q)\operatorname{clip}(\Delta u_\theta,-\Delta u_\max,\Delta u_\max)
\]

The network predicts a small steering correction, speed multiplier, or lateral
target offset from:

- local LiDAR/camera occupancy;
- raceline curvature and cross-track error;
- measured speed and yaw rate;
- recent steering history;
- visual boundary offset;
- perception confidence.

The gate \(g(q)\) approaches zero when the input is out of distribution,
sensor confidence is low, or localization disagrees with vision.

Residual policies have been studied specifically for F1TENTH-style racing and
can improve over a classical baseline while retaining its stable behavior
([residual racing controller paper](https://arxiv.org/abs/2302.07035)).

#### Training strategies

- **Supervised residual:** fit corrections from expert/human laps or an
  offline optimizer.
- **MPPI distillation:** train the network to imitate the expensive MPPI
  action, then keep MPPI as evaluator/fallback.
- **Simulation reinforcement learning:** randomize textures, lighting, depth
  dropout, friction, and opponents; deploy only bounded residuals.
- **Offline reinforcement learning:** learn from logged bags without unsafe
  on-car exploration.

Train off-car, export ONNX, deploy as a small TensorRT FP16 engine. A compact
CNN over a 2.5D BEV grid plus a small state MLP is preferable to processing
full-resolution RGB in the control network.

#### Feasibility

**Medium research effort, medium–high potential.** It becomes appropriate only
after the baseline and evaluation dataset are strong.

### 2.5 Cross-modal self-supervision for an RC-car detector

#### Research question

Can the existing LiDAR/map algorithm automatically generate most of the labels
needed to train a camera detector?

This is one of the most practical research ideas because custom RC-car labels
are otherwise expensive.

#### Pseudo-label pipeline

1. Run the existing map-subtraction detector over recorded laps.
2. Retain high-margin dynamic LiDAR clusters.
3. Project each cluster into the calibrated RGB/depth image.
4. Use depth connectivity around the projected range to generate a foreground
   mask and bounding box.
5. Track the proposal temporally; discard labels that jump or contradict
   object dimensions.
6. Sample negatives from wall corners and static map residuals.
7. Present uncertain examples in a lightweight human-review tool.
8. Train a small RC-car detector/segmenter offboard.
9. Deploy with TensorRT and use new visual predictions to recover LiDAR misses.
10. Repeat: camera and LiDAR disagreement becomes the next annotation queue.

This is cross-modal teacher–student learning:

- LiDAR/map subtraction supplies precise geometric pseudo-labels;
- RGB learns appearance and can generalize to cases where the LiDAR slice is
  incomplete;
- depth refines masks and rejects projection errors.

Dataset splitting must be by track/session/day, not random adjacent frames,
or temporal similarity will make validation results misleading.

#### Feasibility

**Medium effort, high practical value.** It directly builds on code already in
the repository and avoids unsafe online learning.

### 2.6 Online traction and slip estimation from visual motion

#### Research question

Can camera/LiDAR motion reveal when the tires are sliding so the velocity
profile adapts to the actual surface?

#### Algorithm

1. Track ground/static-scene features using VPI optical flow.
2. Use depth to turn image flow into metric ego-motion where possible.
3. Estimate independent yaw/translation from LiDAR scan matching.
4. Compare visual/LiDAR motion with wheel odometry and the bicycle-model
   prediction from steering.
5. Compute a slip indicator:

\[
q_\text{slip}
=\alpha|\dot\psi_\text{measured}-\dot\psi_\text{model}|
+\beta|v_\text{visual}-v_\text{wheel}|
\]

6. Filter it by track position \(s\) and repeated observations.
7. Lower local lateral-acceleration and braking assumptions when persistent
   slip is detected.
8. Save a per-track friction/confidence map for later laps.

This could make the offline speed profile adaptive to dust, tire temperature,
battery state, or surface changes. It is also useful as a safety diagnostic.

The method needs texture and careful rejection of moving objects. Camera
motion estimates at 15 fps may be too noisy at high speed; benchmark a faster
profile and use LiDAR as an independent reference.

#### Feasibility

**Medium–high effort, medium value, high research interest.** Start as a
logger/diagnostic, never as an automatic speed increase.

### 2.7 Learned depth completion with LiDAR anchors and uncertainty

#### Research question

Can a network fill RealSense depth holes in sunlight or on difficult surfaces
without losing metric consistency?

Input:

- RGB;
- raw RealSense depth plus validity mask;
- projected 2D LiDAR points;
- optionally previous depth warped by ego motion.

Output:

- dense depth;
- per-pixel aleatoric uncertainty;
- optional moving-object mask.

Sparse metric depth has been shown to substantially improve monocular dense
depth prediction
([Sparse-to-Dense paper](https://arxiv.org/abs/1709.07492)). Here, RealSense
already supplies dense-but-imperfect depth and LiDAR provides highly trusted
sparse anchors, so the task is depth refinement/completion rather than
monocular scale recovery.

Important restriction: learned depth is suitable for soft planning costs and
semantic mapping. It must not be the only source for a hard emergency stop.
The network can hallucinate plausible surfaces, especially outside its
training distribution.

Training data is the hard part because a 2D LiDAR is not dense ground truth.
Possible supervision:

- multi-frame RGB-D reconstruction in static scenes;
- carefully placed calibration objects with known geometry;
- synthetic domain-randomized stereo data;
- temporal photometric/geometric consistency;
- high-confidence RealSense pixels plus sparse LiDAR anchors.

#### Feasibility

**High effort, uncertain near-term payoff.** This is a research extension, not
an early milestone.

### 2.8 Full hybrid factor-graph localization

#### Research question

Can one estimator combine visual, depth, LiDAR, wheel, and inertial
constraints more robustly than the current particle-filter pipeline?

Potential graph factors:

- wheel odometry between poses;
- VESC IMU preintegration or yaw-rate constraints;
- RGB-D feature reprojection;
- depth ICP or visual odometry increments;
- 2D LiDAR scan-to-map constraints;
- loop closures from visual appearance and LiDAR geometry;
- racetrack boundary landmark observations.

A fixed-lag smoother could provide local high-rate state while a slower global
graph handles loop closures. The current particle filter remains a useful
baseline and recovery source.

This approach is only justified if data demonstrates localization is the
lap-time or reliability bottleneck. It introduces difficult calibration,
covariance, solver, and failure-recovery problems. RTAB-Map is the most
practical first comparison because it already supports visual and LiDAR
configurations; a custom GTSAM graph is a later research path.

#### Feasibility

**High effort.** Evaluate offline and in parallel before considering a
replacement.

### 2.9 Reliability-aware compute scheduling

The camera pipeline itself can adapt to the situation:

- At high speed, prioritize depth geometry, obstacle tracking, and low latency.
- During mapping or low-speed exploration, enable richer segmentation and
  dense local mapping.
- Run full-frame detection only when LiDAR/depth disagreement or track
  uncertainty is high.
- Disable dashboard JPEG and high-rate debug outputs during timed runs.
- If depth invalid fraction rises, spend GPU budget on RGB semantics or reduce
  speed rather than repeatedly processing unusable depth.
- If thermals or GPU latency exceed a limit, shed non-safety workloads first.

This is an “anytime” perception stack: it always produces a small,
deterministic geometric safety product, then adds semantic richness when
compute and confidence permit.

### 2.10 Recommended research portfolio

The proposals should not all be built at once. A coherent sequence is:

**Phase A — measurement and deterministic geometry**

1. Camera extrinsic and time calibration.
2. Synchronized data recording and offline replay.
3. Depth health, ground removal, clustered obstacles.
4. Virtual scan and swept-path speed governor in shadow mode.
5. Stronger measured-speed TTC in pure pursuit.

**Phase B — object-level fusion**

1. RGB-D confirmation of LiDAR/map opponent proposals.
2. Frenet Kalman tracking with covariance.
3. Fused pass-corridor evaluation.
4. Cross-modal pseudo-label dataset.
5. TensorRT RC-car detector.

**Phase C — local world model**

1. Rolling 2.5D occupancy/height/semantic grid.
2. Camera-derived track boundaries.
3. Visibility and uncertainty speed governor.
4. GPU MPPI experimental controller with pure-pursuit fallback.

**Phase D — research extensions**

1. Residual policy distilled from MPPI.
2. Online slip/friction estimation.
3. Learned depth completion.
4. Hybrid factor-graph localization.

### 2.11 Experimental protocol and acceptance criteria

Every idea should be evaluated against the same baseline, not judged from a
good-looking video.

#### Dataset

Record:

- raw/rectified color and depth plus `CameraInfo`;
- `/scan`, `/odom`, VESC IMU, localization pose;
- `/drive`, `/ackermann_cmd`, steering/servo feedback;
- `/tf` and `/tf_static`;
- map and raceline;
- all perception outputs and health diagnostics;
- Jetson utilization/thermal logs.

Include:

- empty tracks;
- boxes at different heights and reflectivities;
- wall corners and posts as hard negatives;
- stationary and moving opponent cars;
- direct/oblique lighting and shadows;
- slow and fast laps;
- partial occlusion;
- sensor dropout and stale-message tests.

Split evaluation by entire session/track/day. Adjacent frames from one lap
must not appear in both training and validation.

#### Perception metrics

| Task | Metrics |
|---|---|
| Depth obstacles | range/lateral error, cluster recall, false clusters, invalid fraction |
| Virtual scan | disagreement with LiDAR by angle/range, added-obstacle precision |
| Opponent detection | precision/recall, false positives per lap, range/bearing/width error |
| Tracking | ID switches, position/velocity error, track continuity, covariance calibration |
| Boundaries | lateral error, visible horizon, continuity, false boundary rate |
| Localization check | relative-pose drift, jump-detection recall, false alarm rate |

#### Control and safety metrics

- collision-free laps;
- human LB releases/interventions;
- false emergency stops per lap;
- missed-obstacle events;
- minimum observed clearance;
- lap time and sector time;
- RMS/max cross-track error;
- steering rate and jerk;
- speed lost to perception conservatism;
- command and perception deadline misses.

#### Compute metrics

- p50/p95/p99 acquisition-to-decision latency;
- camera observation age at command time;
- CPU/GPU/memory/swap;
- dropped frames and queue backlog;
- thermal throttling over a full run;
- power-mode sensitivity;
- impact of MJPEG/debug recording.

#### Ablations

At minimum compare:

```text
L                  LiDAR baseline
L + D              LiDAR plus geometric depth
L + D + RGB        LiDAR/depth plus semantic vision
L + D + RGB + pred object prediction / uncertainty
```

For learned models compare FP32/FP16 and, only after calibration, INT8.
Measure model accuracy and end-to-end control outcomes; faster inference is
not useful if it increases false braking or missed opponents.

#### Rollout gates

1. Offline bag replay with no ROS drive publisher.
2. Live sensor shadow mode; publish diagnostics only.
3. Static obstacle tests with drive stack off.
4. Wheels off the ground, LB held.
5. Low-speed floor tests in open space.
6. One new capability at a time.
7. Only then increase speed or enable strategic behavior.

The independent raw-LiDAR guard, mux, sensor watchdogs, and LB deadman remain
active at every physical stage.

### 2.12 Final recommendation

The camera should not be introduced as “another image topic” or as an
end-to-end steering network. Its most valuable first role is to build a
calibrated, timestamped, uncertainty-aware near-field representation that
describes obstacles the planar LiDAR cannot fully see.

The recommended concrete architecture is:

```text
raw LiDAR ───────────────► independent hard safety + localization
        └───────────────► local fusion grid

RealSense depth ─────────► 2.5D obstacles + swept-path speed ceiling
RealSense RGB + depth ───► opponent identity + track boundaries

local fusion grid ───────► enhanced gap follow / enhanced pure pursuit
                     └──► later: GPU MPPI experimental controller
```

This sequence is feasible on the installed Orin Nano because it uses:

- deterministic geometry for the high-rate core;
- TensorRT only for small semantic models;
- VPI/OFA/VIC where those accelerators actually exist;
- CUDA for workloads that genuinely parallelize;
- multi-rate scheduling to stay within 8 GB;
- the current controllers as trusted fallbacks.

The combination offers more than either sensor alone: LiDAR supplies robust
metric geometry and global map consistency, while RGB-D supplies vertical
structure, appearance, object identity, visual motion, and track semantics.
The research opportunity is to fuse those strengths while preserving explicit
uncertainty and a simple, independently testable safety path.

### 2.13 Primary and official references

- NVIDIA, [Jetson Orin Nano Super performance and hardware configuration](https://developer.nvidia.com/blog/nvidia-jetson-orin-nano-developer-kit-gets-a-super-boost/)
- NVIDIA, [TensorRT documentation](https://docs.nvidia.com/deeplearning/tensorrt/latest/index.html)
- NVIDIA, [TensorRT quantization](https://docs.nvidia.com/deeplearning/tensorrt/latest/inference-library/work-with-quantized-types.html)
- NVIDIA, [VPI backends and memory model](https://docs.nvidia.com/vpi/basic_concepts.html)
- NVIDIA, [Isaac ROS 4.4 supported platforms](https://nvidia-isaac-ros.github.io/v/release-4.4/repositories_and_packages/isaac_ros_common/index.html)
- Intel, [RealSense D435i specifications](https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html)
- Intel, [D400 depth post-processing](https://www.intel.com/content/www/us/en/content-details/842031/depth-post-processing-for-intel-realsense-d400-depth-cameras.html)
- ROS 2 Jazzy, [depth_image_proc](https://docs.ros.org/en/ros2_packages/jazzy/api/depth_image_proc/doc/index.html)
- ROS 2 Jazzy, [pointcloud_to_laserscan](https://docs.ros.org/en/ros2_packages/jazzy/api/pointcloud_to_laserscan/)
- Williams et al., [Model Predictive Path Integral Control](https://doi.org/10.2514/1.G001921)
- Millane et al., [nvblox: GPU-Accelerated Incremental Signed Distance Field Mapping](https://arxiv.org/abs/2311.00626)
- Campos et al., [ORB-SLAM3](https://doi.org/10.1109/TRO.2021.3075644)
- Labbé and Michaud, [RTAB-Map as an Open-Source LiDAR and Visual SLAM Library](https://arxiv.org/abs/2403.06341)
- Ge et al., [YOLOX](https://arxiv.org/abs/2107.08430)
- Zhang et al., [ByteTrack](https://www.ecva.net/papers/eccv_2022/papers_ECCV/papers/136820001.pdf)
- Poudel et al., [Fast-SCNN](https://arxiv.org/abs/1902.04502)
- Ma and Karaman, [Sparse-to-Dense Depth Prediction](https://arxiv.org/abs/1709.07492)
- Trumpp et al., [Residual Policy Learning for Autonomous Racing](https://arxiv.org/abs/2302.07035)
