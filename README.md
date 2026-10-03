# jgb_rover: Gazebo simulation with IMU + encoder fusion and camera-based mapping

Simulation of a differential-drive rover (2 driven wheels, 2 swivel casters, MPU6050 IMU, one USB
webcam) in Gazebo, with:

- `ros2_control` diff drive (`/cmd_vel` → wheels → `/wheel/odom`)
- startup gyro-bias calibration and a `robot_localization` EKF (wheel odometry + gyro → `odom → base_footprint`)
- a **visual floor scan**: the monocular camera turned into a `LaserScan` using the flat-floor assumption
- `slam_toolbox` building a 2D map from that scan, plus an ArUco marker check of the map
- evaluation scripts with measured results for every stage

Every interface uses the topic names and message types the real drivers will use, so the
simulation nodes can be replaced by hardware drivers without changing anything downstream
(see [Sim to real](#sim-to-real)).

```
        Gazebo (sim)                       same on the real robot
 ┌──────────────────────────┐   ┌──────────────────────────────────────────────────────┐
 │ wheels  (gz_ros2_control)│──▶│ diff_drive_controller ─▶ /wheel/odom ─┐               │
 │ MPU6050 (IMU sensor)     │──▶│ /imu/data_raw ─▶ imu_bias_calibration ─▶ /imu/data ─┐ │
 │ webcam  (camera sensor)  │──▶│ /camera/image_raw ─▶ visual_floor_scan ─▶ /visual_scan│
 └──────────────────────────┘   │                         │            EKF ◀──────────┘│
          ▲ /cmd_vel            │                         ▼             │ odom→base     │
          └─────────────────────│                    slam_toolbox ◀─────┘               │
                                │                         │ map→odom, /map              │
                                └──────────────────────────────────────────────────────┘
```

## Contents

1. [Environment](#environment)
2. [Install](#install)
3. [Build](#build)
4. [Run](#run)
5. [Topics and frames](#topics-and-frames)
6. [Configuration and tuning](#configuration-and-tuning)
7. [Measured results](#measured-results)
8. [Limits of the visual floor scan](#limits-of-the-visual-floor-scan)
9. [Sim to real](#sim-to-real)
10. [Repository layout](#repository-layout)

## Environment

The task targeted ROS 2 Jazzy + Gazebo Harmonic. This machine runs **Ubuntu 22.04, ROS 2 Humble and
Gazebo Fortress** (`ign gazebo`, Gazebo Sim 6), so everything is written for that combination:

| Jazzy + Harmonic                         | used here (Humble + Fortress)                                   |
|------------------------------------------|-----------------------------------------------------------------|
| `gz sim`                                 | `ign gazebo` (started through `ros_gz_sim gz_sim.launch.py`)    |
| `gz-sim-*-system` plugins, `gz.msgs.*`   | `ignition-gazebo-*-system`, `ignition.msgs.*` in `bridge.yaml`  |
| `<gz_frame_id>`                          | `<ignition_frame_id>`                                           |
| `gz_ros2_control`                        | `gz_ros2_control` 0.7 (`gz_ros2_control/GazeboSimSystem`)        |
| diff drive takes `TwistStamped`          | same: `use_stamped_vel: true`, so the interface matches Jazzy   |

A Docker image with the same stack is provided (see [Docker](#docker)).

## Install

```bash
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-robot-localization ros-humble-slam-toolbox \
  ros-humble-nav2-map-server ros-humble-teleop-twist-keyboard ros-humble-xacro \
  ros-humble-joint-state-publisher-gui ros-humble-tf2-tools ros-humble-cv-bridge \
  python3-opencv python3-matplotlib python3-pytest
```

**Python packages in `~/.local`.** pip-installed numpy 2.x / OpenCV 5.x in the user site directory
shadow the system packages and make `cv_bridge` crash (`KeyError: 16`). All launch files set
`PYTHONNOUSERSITE=1`; do the same in any shell where you run the scripts:

```bash
export PYTHONNOUSERSITE=1
```

### Docker

```bash
xhost +local:docker                            # once per login: let the container open windows
docker compose -f docker/compose.yaml build    # ROS 2 Humble + Fortress + all dependencies
docker compose -f docker/compose.yaml run --rm dev
# inside the container:
build_ws                                       # builds into a volume, not into ros2_ws/build
source /home/dev/colcon/install/setup.bash
ros2 launch jgb_rover_bringup bringup_sim.launch.py
```

The repository is mounted at `/home/dev/mobile`; edits on the host are live (symlink install). The
container uses the host network (ROS 2 discovery with the host and the robot) and the NVIDIA GPU
(needs `nvidia-container-toolkit`). The image's user gets UID/GID 1000; for another ID build with
`export UID; export GID=$(id -g)` first (in bash `UID=...` cannot be assigned, but it can be exported).

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`robot_spec.yaml` in the repository root is installed into `jgb_rover_description/config` at build
time; rebuild after editing it.

## Run

### Everything

```bash
ros2 launch jgb_rover_bringup bringup_sim.launch.py world:=apartment camera_pitch:=0.26 \
    slam:=true rviz:=true inject_errors:=false
```

Hold the robot still for the first 3 s (gyro-bias calibration); `/imu/data` and the EKF's use of
the gyro start after that.

| argument             | default     | meaning |
|----------------------|-------------|---------|
| `world`              | `apartment` | `apartment`, `apartment_slip` (low-friction floor patches) or a path to an `.sdf` |
| `camera_pitch`       | spec (0.26) | camera tilt down [rad]; empty = `camera.pitch_down_default` |
| `inject_errors`      | `false`     | left wheel 1.5 % smaller (radius mismatch) |
| `slam`               | `true`      | `slam_toolbox` on `/visual_scan` |
| `slam_scan_matching` | `false`     | let `slam_toolbox` correct the EKF pose by scan matching ([why off](#slam)) |
| `slam_params_file`   | (package)   | alternative `slam_toolbox` parameter file |
| `perception`         | `true`      | visual floor scan |
| `aruco`              | `false`     | ArUco detector and map-quality check |
| `ekf_imu_accel`      | `false`     | also fuse IMU forward acceleration ([tested](#measured-results), no gain) |
| `rviz`               | `true`      | RViz: robot, TF, map, scan, EKF vs ground-truth path, debug image |
| `headless`           | `false`     | Gazebo server only, no Gazebo GUI |
| `headless_rendering` | `false`     | with `headless`: render cameras with EGL (no X display needed) |
| `x`, `y`, `yaw`      | 0           | spawn pose (= map origin) |

Simulation only (no EKF, no perception): `ros2 launch jgb_rover_gazebo sim.launch.py`.
Model only, with joint sliders: `ros2 launch jgb_rover_description display.launch.py gui:=true`.

### Driving

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p use_sim_time:=true
```

`stamped:=true` makes teleop publish `geometry_msgs/TwistStamped` on `/cmd_vel`. Its default speed
(0.5 m/s) is limited to the spec's 0.40 m/s by the controller.

### Saving a map

```bash
ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p use_sim_time:=true -p save_map_timeout:=20.0
```

The default 2 s timeout is too short once the map has a few hundred scans.

### Test and evaluation scripts

All need the simulation running (except the first two) and print their result; most save a plot.

| command | what it checks |
|---------|----------------|
| `ros2 run jgb_rover_description check_alignment.py` | visual meshes vs collision primitives (bounding boxes) |
| `PYTHONNOUSERSITE=1 python3 -m pytest ros2_ws/src/jgb_rover_perception/test` | floor-scan geometry unit tests |
| `ros2 run jgb_rover_gazebo check_spawn.py` | settling time, jitter, 4 contacts after spawn |
| `ros2 run jgb_rover_bringup drive_test.py` | 1 m straight and 360° spin: wheel odometry vs ground truth |
| `ros2 run jgb_rover_bringup eval_odometry.py --check injected` (or `nominal`) | 3 squares + figure-8: wheel-only vs EKF, PASS/FAIL |
| `ros2 run jgb_rover_bringup check_visual_scan.py` | `/visual_scan` vs ray-cast ground truth while spinning |
| `ros2 run jgb_rover_bringup waypoint_tour.py` | drives through both rooms with 360° look-arounds |
| `ros2 run jgb_rover_bringup eval_map.py map.yaml` | saved map vs world SDF: wall-alignment error, overlay PNG |
| `PYTHONNOUSERSITE=1 python3 ros2_ws/src/jgb_rover_gazebo/scripts/gen_world.py` | regenerates the world, markers and posters from `apartment_layout.yaml` |

`waypoint_tour.py` and the motion scripts navigate on the ground truth (they are test drivers);
the estimators only see the sensors.

## Topics and frames

| topic | type | from (sim) | from (real robot) |
|-------|------|------------|-------------------|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | teleop / scripts | teleop / navigation |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | same |
| `/wheel/odom` | `nav_msgs/Odometry` | `diff_drive_controller` (remapped) | same |
| `/imu/data_raw` | `sensor_msgs/Imu` | Gazebo IMU via `ros_gz_bridge` | microcontroller (MPU6050) |
| `/imu/data` | `sensor_msgs/Imu` | `imu_bias_calibration` | same |
| `/odometry/filtered` | `nav_msgs/Odometry` | `robot_localization` EKF | same |
| `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` | Gazebo camera via bridge | `v4l2_camera` |
| `/visual_scan` | `sensor_msgs/LaserScan` | `visual_floor_scan` | same |
| `/visual_scan/debug` | `sensor_msgs/Image` | `visual_floor_scan` (floor mask, boundary, horizon) | same |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | same |
| `/aruco/poses` | `geometry_msgs/PoseArray` | `aruco_detector` (frame `camera_optical_frame`) | same |
| `/path/ekf`, `/path/ground_truth` | `nav_msgs/Path` | `path_recorder.py` (RViz) | `/path/ekf` only |
| `/ground_truth/odom` | `nav_msgs/Odometry` | Gazebo `OdometryPublisher` (frame `world`) | (none) |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo | (none, `use_sim_time:=false`) |

TF tree (who publishes in brackets):

```
map ─[slam_toolbox]─▶ odom ─[EKF, 50 Hz]─▶ base_footprint
base_footprint ─▶ base_link (z = 0.0435)           [robot_state_publisher, fixed]
base_link ─▶ left_wheel_link, right_wheel_link     [robot_state_publisher, from /joint_states]
base_link ─▶ caster_front_link, caster_rear_link, imu_link
base_link ─▶ camera_link (pitched by camera_pitch) ─▶ camera_optical_frame
base_footprint ─▶ visual_scan_link                 (ground point under the lens: scan origin)
camera_optical_frame ─[aruco_detector]─▶ aruco_<id>
```

The ground truth is published only as a topic, never on `/tf`, so it cannot interfere with the EKF
or SLAM. `diff_drive_controller` has `enable_odom_tf: false`; the EKF owns `odom → base_footprint`.

## Configuration and tuning

### Where the numbers come from

- **`robot_spec.yaml`** (repository root) is the single source of truth for dimensions, masses,
  sensor poses and noise. The xacro reads it with `xacro.load_yaml`. Parameter files that need spec
  values (`controllers.yaml`, `imu_bias_calibration.yaml`) contain placeholders like
  `$(spec drive.wheel_separation)` that `jgb_rover_control.spec_params.render()` fills in at launch.
- **`jgb_rover_description/config/sim_assumptions.yaml`** holds every value the simulation needs
  that is *not* in the spec, each with its reason:

| value | why |
|-------|-----|
| `wheel.collision: sphere` | a 40 mm wide cylinder touches the floor at its inner edges in DART: track width 0.230 m instead of 0.270 m, in-place turns 15 % off |
| `wheel.joint_damping: 0.005` | gearbox loss (guess) |
| `camera.mesh_clearance: 0.005` | the lens centre in the spec is 2 mm inside `webcam.stl`; a camera inside its housing renders empty frames at some headings. Only the visual is moved |
| `error_injection.left_wheel_radius_scale: 0.985` | the radius mismatch for `inject_errors:=true`; the axle is lowered by the difference so the smaller wheel still touches the floor |
| caster links without inertia | their mass is already part of `mass_properties` |

### Drive (`jgb_rover_control/config/controllers.yaml`)

Separation, radius and the velocity/acceleration limits come from the spec.
`velocity_rolling_window_size: 2`: with the default 10 the published twist lagged ~50 ms and the EKF
(which integrates the twist) trailed the robot by 1 cm while moving. Odometry covariances are set so
the EKF trusts the gyro for yaw rate (wheel vyaw σ = 0.1 rad/s) and the wheels for speed.

### IMU bias (`jgb_rover_localization/config/imu_bias_calibration.yaml`)

The node averages the gyro for `calibration_duration` (3 s) while `/cmd_vel` is zero and the wheels
are still, then subtracts the bias. Afterwards it keeps refining the bias whenever the robot stands
still (zero-velocity update, `zupt_*`), which also follows thermal drift on the real MPU6050.
`~/recalibrate` (`std_srvs/Trigger`) restarts the calibration. Orientation is always published as
unknown (`orientation_covariance[0] = -1`): the MPU6050 has no magnetometer.

### EKF (`jgb_rover_localization/config/ekf.yaml`)

`two_d_mode`, 50 Hz. Wheel odometry: vx, vy (= 0, non-holonomic constraint), vyaw. IMU: vyaw only.
Forward acceleration was tested (`ekf_imu_accel:=true`) and gave no consistent improvement, while on
the real robot it adds accelerometer bias and tilt errors, so it is off.

### Visual floor scan (`jgb_rover_perception/config/visual_floor_scan.yaml`)

Per frame (downscaled to 320 × 240): Lab floor-colour model learned from a seed patch at the bottom
centre (running mean + covariance, slow updates, only from patches that still look like floor),
Mahalanobis threshold + morphology, Canny edges cut out of the floor mask; for each column the first
non-floor pixel from the bottom is the obstacle base; it is mapped to (range, bearing) through a
lookup table built from `camera_info` (including distortion) and the camera pose from **tf2**. The
table is rebuilt only when the camera pose or intrinsics change, so a frame costs ~8–10 ms here.

| parameter | effect |
|-----------|--------|
| `mahalanobis_threshold` (4.0) | higher = more tolerant floor (fewer false obstacles, more missed low-contrast ones) |
| `min_stddev_l` / `min_stddev_ab` (8 / 3) | minimum spread of the floor model: how much brightness/colour change across the floor is still floor. The floor gets darker with distance; too small and the far floor becomes "obstacle" |
| `luminance_weight` (1.0) | < 1 tolerates shadows better, but walls and floor are told apart mostly by brightness |
| `canny_low/high` (60/150), `use_canny_edges` | edges stop the column walk (wall bases, floor-coloured boxes) |
| `boundary_offset_px` (1.0) | blur + edge shift the detected boundary ~1 px towards the robot; ranges were 5 % short without it |
| `min_obstacle_px` (3) | non-floor run length needed to count as an obstacle (noise rejection) |
| `range_min/max` (0.15 / 2.5 m), `num_beams` (160) | scan geometry; one image row covers ~2 cm of floor at 0.9 m but ~18 cm at 2.5 m |
| `no_obstacle_mode` (`max_plus`) | "looked, nothing there" is sent as `range_max + 0.05` (with `msg.range_max = range_max + 0.1`), which `slam_toolbox` ray-traces as free space; `inf` gives REP-117 behaviour |
| `max_rate` (15 Hz) | frames above this are dropped, never queued |

Watch `/visual_scan/debug` while tuning: green = floor, red dots = obstacle bases within range,
orange = beyond range, magenta = horizon, cyan box = seed patch.

### SLAM

`jgb_rover_bringup/config/slam_toolbox.yaml`, `online_async`, resolution 0.03 m, max range 2.5 m.

**Scan matching is off by default.** `slam_toolbox` builds the map and publishes `map → odom`, but it
places the scans at the EKF poses instead of correcting them. On the apartment tour, with matching
on, the SLAM pose drifted up to 47° / 1.9 m (false matches and loop closures on a 100° × 2.5 m scan
that often sees one short piece of wall) and the map failed the target; without it the map is within
3.5 cm ([results](#measured-results)). The matching parameters in the file are tuned for when it is
switched on (`slam_scan_matching:=true`), e.g. to try on the real robot where the odometry is worse.

Two `slam_toolbox` behaviours that matter for this robot (both handled in the config):
- it only accepts a new scan after the robot **translated** `minimum_travel_distance`, whatever
  `minimum_travel_heading` says; in-place spins (the camera's way of looking around) added nothing
  until that was set to 0 (scan rate is then limited by `minimum_time_interval`);
- `use_response_expansion: true` widens the rotation search on weak matches, which produced 44°
  jumps with this scan.

## Measured results

Headless runs in Gazebo (RTX 3060). Plots and maps are in `results/`.

**Phase 1, description and world.** `check_urdf` passes. A 20 mm drop settles in 0.06 s, then 0.0 µm
jitter, roll/pitch 0.000°, all four contact points at 0.00 mm. Visual and collision bounding boxes
agree (wheels to 0.0 mm).

**Phase 2, drive** (`drive_test.py`). 1.0 m straight at 0.2 m/s: wheel odometry 1.0020 m vs ground
truth 1.0020 m (0.00 %, limit 3 %). 360° spin: wheel-odometry yaw error +0.00°. With
`inject_errors:=true`: straight 0.77 %, robot really turns +3.22° per metre (theory 3.18°), spin error
+2.72°. The simulated wheels do not slip, so without injection wheel odometry is exact.

**Phase 3, fusion** (`eval_odometry.py`, 3 × 1 m squares + figure-8, ~18 m, 130 s):

| run | wheel-only RMSE / final yaw | EKF RMSE / final yaw | check |
|-----|-----------------------------|----------------------|-------|
| left wheel 1.5 % small | 35.99 cm / −50.74° | **0.66 cm / −0.15°** | PASS (EKF yaw ≤ ¼ of wheel-only) |
| no injection | 0.02 cm / −0.00° | 0.43 cm / −0.47° | PASS within tolerance (+1 cm, +1°) |
| slip patches (`apartment_slip`) | 10.82 cm / +4.16° | 8.26 cm / +0.53° | gyro fixes heading, not slip distance |
| radius mismatch + IMU ax | 35.99 cm / −50.74° | 0.88 cm / +0.53° | ax not kept |
| no injection + IMU ax | 0.02 cm / −0.00° | 0.36 cm / −0.24° | |

Without injection the simulated wheel odometry is exact, so no fused estimate can be strictly better;
the EKF's remaining error is the gyro bias left after calibration (expected 0.0009/√300 rad/s, i.e.
~0.4° over two minutes) and a one-IMU-sample lag in turns. Results vary by a few tenths of a degree
between runs because the simulated gyro bias is drawn anew each run, as on a real MPU6050 at power-up.

**Phase 4, camera and mapping.**

- `/visual_scan` vs ground truth (`check_visual_scan.py`, spinning at three spots): median range
  error 1.8 / 2.6 / 4.4 cm, false obstacles 0.3 / 0.2 / 1.2 % of beams, missed obstacles
  0.3 / 0.0 / 2.1 %. 12–13 Hz, 8–10 ms per frame on this PC (not yet measured on a Pi 4).
- Map after the waypoint tour (`eval_map.py`): **wall-alignment error 3.5 cm RMS** (median 3.0 cm,
  p90 5.0 cm; 4.1 cm without any alignment), 0.03 % false walls, 96 % of the explored walls covered
  (`results/phase4/map_overlay.png`). With scan matching on: 10.6 cm, 30 % false, 54 % covered.
- ArUco markers (all 8 seen): map position 3.7–13 cm from the truth, height within 0.7 cm, facing
  direction within ±10°.

## Limits of the visual floor scan

Measured or observed in simulation:

- **Shadows / lighting.** With shadows switched on in the world, the median range error grows from
  ~2 cm to 13 cm and false obstacles to 6–12 %: the boundary lands on the shadow edge in front of
  the wall. The apartment world has shadows off for this reason. Strong lighting changes across the
  floor have the same effect (the floor model's spread, `min_stddev_l`, is the knob).
- **Floor-coloured obstacles.** The near-floor-coloured grey box was only partly mapped, through its
  edges; a smooth obstacle of exactly the floor colour would be invisible.
- **Overhangs.** Table tops and chair seats are invisible; only legs are seen. Anything overhanging
  below the robot's height (0.276 m), such as a low shelf, is a collision risk.
- **Close range.** Nothing closer than ~0.17 m is visible (below the image).
- **Flat-floor assumption.** A 1° body pitch (caster bump, carpet edge, ramp, door threshold) moves a
  reading at 2 m by ~27 cm. Ranges beyond ~2 m are coarse anyway (one row ≈ 18 cm at 2.5 m).
- **Thin obstacles.** Legs of 2.5–3 cm are mapped but with larger errors and may vanish far away.
- **Textured floors / rugs / floor-coloured walls** break the single-Gaussian floor model.
- **No absolute correction.** With scan matching off, odometry errors (wheel slip) go straight into
  the map; ArUco landmarks are the planned absolute correction (detection works, pose correction is
  not implemented yet).

## Sim to real

The microcontroller (ESP32 or Teensy) reads the encoders and the MPU6050; the Raspberry Pi 4 runs ROS 2.
What changes and what does not:

| simulation | real robot | notes |
|------------|------------|-------|
| Gazebo world, `ros_gz_bridge`, `/clock` | the real world | `use_sim_time:=false` everywhere |
| `gz_ros2_control/GazeboSimSystem` | custom `ros2_control` hardware interface (`hardware_plugin` xacro arg, default `jgb_rover_hardware/JgbRoverSystem`, not written yet), run with `sim_gazebo:=false` | serial/USB link to the MCU: write wheel velocity commands [rad/s], read wheel positions [rad] (x4 decoding: 3960 counts/rev, 1.587 mrad/count) and velocities. `diff_drive_controller`, `/cmd_vel`, `/wheel/odom` stay identical |
| (alternative) | micro-ROS on the MCU | then the MCU must provide the same interfaces: either a `ros2_control` hardware interface on top of micro-ROS topics, or publish `/wheel/odom` (same covariances) itself and subscribe `/cmd_vel` (`TwistStamped`) |
| Gazebo IMU → `/imu/data_raw` | MCU publishes `sensor_msgs/Imu` on `/imu/data_raw`, frame `imu_link`, 100 Hz | gyro in rad/s, acceleration in m/s² including gravity, `orientation_covariance[0] = -1`. Board mounted X forward, Z up as in the spec |
| Gazebo camera | `v4l2_camera` | see below |
| `/ground_truth/odom`, `path_recorder` ground-truth path | (none) | evaluation only |

Unchanged: `robot_state_publisher` + URDF (with `sim_gazebo:=false`), `imu_bias_calibration`, the EKF,
`visual_floor_scan`, `slam_toolbox`, `aruco_detector`, all topic and frame names.

**Camera.**

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 -p image_size:="[640,480]" \
  -p camera_frame_id:=camera_optical_frame \
  -p camera_info_url:=file://$HOME/.ros/camera_info/webcam.yaml \
  -r image_raw:=/camera/image_raw -r camera_info:=/camera/camera_info
```

Calibrate it once (the WEB007 is a wide-angle lens, so distortion matters; the floor scan uses the
`camera_info` distortion coefficients):

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image_raw -p camera:=/camera
```

Use manual focus and fixed exposure / white balance if the driver allows it: auto exposure changes
the floor colour and the floor model has to re-learn. Measure the real camera pitch and pass it as
`camera_pitch`; the floor scan reads the camera pose from TF, so nothing else changes.

**Values to re-measure on the real robot** (in `robot_spec.yaml`): effective wheel radius (the spec
notes ~0.042 m vs 0.0435 m nominal; drive a measured distance), effective wheel separation (spin
10 turns), IMU noise (record still data), camera pose and pitch, wheel mass. Then re-check the EKF
covariances (`controllers.yaml` twist covariance, `imu_bias_calibration.yaml` noise).

**Raspberry Pi 4.** The floor scan takes ~8–10 ms per frame on the desktop; expect several times
that on the Pi. If it cannot keep 10 Hz, lower `proc_width/height` or `max_rate`. Start without
`rviz` and `aruco` on the Pi and visualise from a laptop on the same network.

There is no real-robot bringup launch yet; it will be `bringup_sim.launch.py` minus Gazebo plus the
hardware interface / micro-ROS agent and `v4l2_camera`.

## Repository layout

```
robot_spec.yaml                  single source of truth (from CAD)
meshes/                          original meshes
docker/                          Dockerfile, compose.yaml, entrypoint, build_ws helper
results/phase3, results/phase4   evaluation plots, maps, overlays (JSON next to each)
ros2_ws/src/
  jgb_rover_description/   urdf/*.xacro, meshes/, rviz/, config/sim_assumptions.yaml, display.launch.py
  jgb_rover_gazebo/        worlds/ (generated), models/ (markers, posters), config/bridge.yaml,
                           config/apartment_layout.yaml, scripts/gen_world.py, launch/sim.launch.py
  jgb_rover_control/       config/controllers.yaml (spec placeholders), spec_params.py
  jgb_rover_localization/  imu_bias_calibration node, config/ekf.yaml
  jgb_rover_perception/    visual_floor_scan (+ floor_scan_core.py, tests), aruco_detector
  jgb_rover_bringup/       launch/bringup_sim.launch.py, config/slam_toolbox.yaml, rviz/sim.rviz,
                           scripts/ (tests, evaluation, tour, path recorder)
```
