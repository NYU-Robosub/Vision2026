# ZED SLAM Validation

The best initial test uses the real ZED 2i in a real room, visualizes the run in RViz, and records an SVO so the exact sensor sequence can be replayed. Gazebo does not reproduce ZED optics, stereo matching, IMU fusion, housing refraction, or Area Memory and is not the acceptance environment for Phase 1.

## 1. Build

On the Ubuntu 22.04 ROS 2 Humble machine with the pinned ZED SDK and wrapper installed:

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Run launch commands from the repository root so `maps/area_memory/front.area` resolves correctly.

## 2. Smoke test

```bash
ros2 launch vision_bringup slam_live.launch.py
```

Confirm in RViz:

- RGB and registered depth update;
- TF contains `map`, `odom`, `zed_front_camera_link`, and `base_link`;
- map and odometry paths update while the camera moves;
- no duplicate-parent or repeated-transform warnings appear.

Confirm from another terminal:

```bash
ros2 topic hz /zed_front/zed_node/odom
ros2 topic echo /zed_front/zed_node/pose_with_covariance --once
ros2 run tf2_ros tf2_echo map zed_front_camera_link
ros2 run tf2_tools view_frames
```

## 3. Dense room-mapping test

Use the heavier test configuration only while evaluating geometry:

```bash
ros2 launch vision_bringup slam_live.launch.py \
  zed_config:=$(pwd)/config/zed_front_mapping_test.yaml
```

This enables a reduced registered point cloud and ZED spatial mapping. It is deliberately disabled in the normal runtime configuration to save GPU and memory.

Walk the camera slowly through a textured room. Avoid rapid rotation and featureless walls. In RViz, inspect the registered cloud, trajectory, TF, and whether revisited geometry aligns with earlier observations.

## 4. Repeatable trajectory

Use tape marks or surveyed points:

1. Hold the camera stationary for 60 seconds.
2. Move in a rectangle of known dimensions.
3. Return to the starting pose and wait 10 seconds.
4. Cover the lenses briefly to force tracking degradation.
5. Uncover the camera and revisit a previously seen area.
6. Shut down cleanly so Area Memory is saved.
7. Restart at a known mapped location and measure relocalization time.

Record:

- stationary translation and rotation drift;
- final closed-loop position error;
- size and timing of loop-closure corrections;
- odometry frequency;
- number and duration of tracking losses;
- relocalization success and time;
- CPU, GPU, and GPU-memory usage;
- visible point-cloud artifacts or duplicated surfaces.

Initial engineering targets for a small indoor room:

- odometry near the configured 30 Hz;
- no NaN or infinite values;
- no duplicate TF publishers;
- stationary translation drift below 5 cm over 60 seconds;
- closed-loop final position error below 20 cm;
- recovery after a short occlusion;
- successful relocalization using saved Area Memory.

These are project targets, not ZED guarantees. Underwater acceptance thresholds must be set after collecting pool data.

## 5. SVO recording and replay

Record synchronized stereo video and IMU using ZED Explorer or the wrapper's SVO recording service. Store recordings under `data/svo/front/` and do not commit large recordings to Git.

Replay:

```bash
ros2 launch vision_bringup slam_replay.launch.py \
  svo_path:=/absolute/path/to/recording.svo2
```

For dense mapping during replay:

```bash
ros2 launch vision_bringup slam_replay.launch.py \
  svo_path:=/absolute/path/to/recording.svo2 \
  zed_config:=$(pwd)/config/zed_front_mapping_test.yaml
```

Replay the same SVO after every parameter change. Compare trajectory, drift, tracking-loss events, and resource usage rather than judging only by how the RViz picture looks.

## 6. Simulation decision

- RViz: required for inspecting real or replayed ROS data.
- SVO replay: required for repeatable ZED regression testing.
- Gazebo: deferred; useful later for generic AUV dynamics and interface tests, but not for validating the real ZED SDK pipeline.
- Isaac Sim: optional future work if full simulated ZED integration becomes worth its installation and GPU cost.

After room testing passes, repeat depth, drift, loop closure, and relocalization tests underwater through the actual camera housing. That is the decisive validation environment for RoboSub.
