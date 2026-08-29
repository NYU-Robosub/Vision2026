# RoboSub ZED Vision Pipeline

A focused ROS 2 project for evaluating a front-mounted ZED 2i as the visual-inertial SLAM foundation of a RoboSub AUV perception system.

## Current Goal

Phase 1 answers one question:

> Can the real ZED 2i localize reliably, close loops, relocalize, and produce useful room geometry before it is trusted underwater or used by motion and semantic perception?

The active repository performs only ZED SLAM bringup and validation. Object detection, semantic world modeling, motion interfaces, bottom-camera perception, and deployment optimization are future phases and are not represented by placeholder nodes.

## Active System

```text
ZED 2i
  |
  v
ZED SDK + ZED ROS 2 wrapper
  |-- RGB and registered depth
  |-- registered point cloud
  |-- IMU-fused visual odometry
  |-- loop-closure-corrected map pose
  |-- Area Memory and relocalization
  `-- map and odom TF
  |
  +--> robot_state_publisher --> fixed AUV/camera frames
  `--> RViz2                --> images, point cloud, paths, and TF
```

There are currently no custom runtime nodes. The canonical launch starts:

1. the Stereolabs ZED wrapper;
2. `robot_state_publisher`;
3. RViz2 by default.

This is the smallest useful system for evaluating ZED SLAM without adding unrelated failure modes.

## Technology Baseline

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Stereolabs ZED SDK 5.2
- Stereolabs `zed-ros2-wrapper` tag `v5.2.2`
- CUDA 12.8 on the Ubuntu development computer
- ZED 2i stereo camera
- RViz2 for ROS data and TF inspection
- ZED SVO for repeatable sensor-data replay
- JetPack 6.2.2 only when future Jetson Orin deployment begins

Change the SDK, wrapper, and CUDA baseline together and retest it rather than following the wrapper's `master` branch.

## Build

On the Ubuntu 22.04 ROS 2 computer:

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Run launches from the repository root so `maps/area_memory/front.area` resolves correctly.

## Run the SLAM Test

Normal lightweight positional-tracking test:

```bash
ros2 launch vision_bringup slam_live.launch.py
```

Dense room-mapping test:

```bash
ros2 launch vision_bringup slam_live.launch.py \
  zed_config:=$(pwd)/config/zed_front_mapping_test.yaml
```

The dense test enables the registered point cloud and ZED spatial mapping. These are disabled in the normal configuration because they consume additional GPU and memory.

Replay a recorded SVO:

```bash
ros2 launch vision_bringup slam_replay.launch.py \
  svo_path:=/absolute/path/to/recording.svo2
```

Replay with dense mapping:

```bash
ros2 launch vision_bringup slam_replay.launch.py \
  svo_path:=/absolute/path/to/recording.svo2 \
  zed_config:=$(pwd)/config/zed_front_mapping_test.yaml
```

RViz starts by default. Disable it for unattended or performance testing with `start_rviz:=false`.

## What to Validate

The Phase 1 test covers:

- RGB and registered depth quality;
- registered point-cloud geometry;
- pose and odometry publication rate;
- a connected, conflict-free TF tree;
- stationary drift;
- closed-loop drift;
- loop-closure correction;
- tracking loss and recovery;
- Area Memory saving;
- relocalization after restart;
- CPU, GPU, and memory use;
- repeatability using the same SVO recording.

The detailed route, commands, measurements, and initial pass criteria are in [`docs/replay_testing.md`](docs/replay_testing.md).

## RViz, SVO, Gazebo, and Isaac Sim

- **RViz is required** to inspect the real or replayed images, depth, point cloud, paths, and TF.
- **SVO replay is required** for repeatable regression tests against identical ZED stereo and IMU data.
- **Gazebo is deferred.** It is useful for future AUV physics, controllers, and generic sensor interfaces, but it does not validate real ZED stereo matching, Area Memory, camera-housing refraction, or underwater optics.
- **Isaac Sim is optional future work.** Stereolabs provides a supported simulated-ZED integration for it, but it is not needed to answer the Phase 1 hardware question.

The strongest validation order is:

1. real ZED in a room;
2. RViz inspection and quantitative measurements;
3. SVO replay for repeatability;
4. real ZED underwater through the final housing;
5. simulation later, only for a clearly defined purpose.

## TF Ownership

While ZED owns localization, the official camera-root layout is:

```text
map
└── odom
    └── zed_front_camera_link
        ├── base_link
        └── ZED optical frames
```

- ZED publishes dynamic `map -> odom` and `odom -> zed_front_camera_link`.
- `robot_state_publisher` publishes fixed camera/AUV joints from xacro.
- No custom node publishes TF.

The camera-to-`base_link` transform currently defaults to zero. Replace it with measured mount translation and rotation before vehicle or underwater validation.

## Repository Layout

```text
config/
  zed_front.yaml                 Lightweight positional tracking
  zed_front_mapping_test.yaml    Point-cloud and spatial-mapping validation
data/svo/front/                  Local SVO recordings; large files stay out of Git
docs/
  architecture.md                Active and future system architecture
  replay_testing.md              Room, replay, and underwater test procedure
maps/area_memory/                Saved ZED Area Memory
ros_ws/src/vision_bringup/
  launch/                        Live and SVO replay launches
  rviz/                          Phase 1 visualization
  urdf/                          ZED 2i and AUV mounting model
scripts/run_front.sh             Convenience live-launch script
```

## Phase 1 Definition of Done

Phase 1 is complete when:

1. one documented command starts the ZED, robot description, and RViz;
2. images, depth, point cloud, pose, odometry, paths, and TF are inspectable;
3. no TF edge has competing publishers;
4. stationary and closed-loop drift are measured rather than judged visually;
5. tracking-loss recovery and Area Memory relocalization are demonstrated;
6. the same run can be replayed from SVO;
7. results and hardware utilization are recorded;
8. the critical tests are repeated underwater through the real housing.

## Future Integration Plan

Only add the next phase after the ZED baseline is measured.

1. Motion-facing odometry and localization-health adapter using standard ROS 2 messages.
2. Python detector for gates and selected competition objects.
3. Timestamp-synchronized depth-based 3D object localization.
4. Minimal semantic observation interfaces.
5. C++ persistent world-model node publishing map-frame objects.
6. Bottom-camera perception as a separate node if a chosen task requires it.
7. Jetson profiling followed by ONNX/TensorRT only if needed.
8. RTAB-Map comparison only if ZED Area Memory is inadequate.
9. Gazebo for AUV dynamics/interface testing or Isaac Sim for simulated ZED testing when either has a specific test objective.

PID control, thruster output, vehicle stabilization, mission planning, and autonomy decisions remain outside this repository.
