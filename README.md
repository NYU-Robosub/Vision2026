# RoboSub ZED + RTAB-Map

Bedroom-scale, above-water SLAM using a ZED 2i and ROS 2. ZED supplies RGB,
registered depth, calibration, and visual-inertial odometry. RTAB-Map builds the
accumulated room map, corrects its graph, and stores a reusable database.

The current test environment is a laptop Linux VM. Jetson Orin deployment comes
later. There is no detector, custom SLAM algorithm, external IMU fusion, or motion
control in this update. Requirements and implementation status are in [scope.md](scope.md).

## Build in Linux

Repository baseline: Ubuntu 22.04, ROS 2 Humble, ZED SDK 5.2 and the checked-in
ZED wrapper. Verify camera USB and NVIDIA GPU/CUDA access inside the VM before
testing the camera. Source/configuration checks cannot establish live SLAM quality.

From the repository root, with ROS and the compatible ZED SDK installed:

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-rtabmap-slam
cd ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The RTAB-Map configuration was checked against Humble packages version 0.23.7.
See [validation results](docs/test_results/implementation_checks.md) for what was
actually tested. The full ZED/camera demonstration is still pending.

## Map the bedroom

After sourcing the workspace in each terminal:

```bash
ros2 launch vision_bringup zed_rtabmap.launch.py
```

This starts the ZED wrapper, `robot_state_publisher`, **one RTAB-Map node**, and
RViz. No second odometry algorithm, custom accumulator, or PLY saver runs in this
profile. `bash scripts/run_front.sh` is a convenience entry point from the repository.

In RViz, **Accumulated Room Map** displays `/rtabmap/cloud_map` in `map`.
The current ZED observation is a separate, initially disabled cyan display.
Slowly scan a textured wall, turn away, and confirm that the wall remains visible.
RViz Decay Time is zero: accumulation must come from RTAB-Map.

The initial map resolution is 5 cm with a 5 m depth range and a 2 Hz SLAM
processing target. These are starting settings for the room trial, not measured
performance guarantees. Return to the starting view to test loop closure.

## Save, reopen, and relocalize

The default database is `~/.ros/robosub/maps/bedroom.db`, independent of the shell's
working directory. Normal startup preserves it. Stop with **Ctrl+C and allow clean
shutdown to finish** so RTAB-Map can finish writing its database.

Restart in localization mode to recognize the saved room without adding new map nodes:

```bash
ros2 launch vision_bringup zed_rtabmap.launch.py mode:=localization
```

Localization requires an existing nonempty file. RTAB-Map validates the database
contents when opening it. A visible saved cloud does not by itself prove that the
current camera pose has relocalized.

The default `mode:=mapping` reopens the database and permits extending it. For a
fresh experiment, choose a **different** absolute filename; no launch deletes maps:

```bash
ros2 launch vision_bringup zed_rtabmap.launch.py \
  database_path:="$HOME/.ros/robosub/maps/bedroom_trial_02.db"
```

Use `database_path` in both mapping and localization when choosing a custom file.
An absolute path beneath `maps/rtabmap/` is also supported and ignored by Git.
The database stores SLAM observations and graph data; a PLY is only a geometry export.

## Repeat from an SVO

```bash
ros2 launch vision_bringup zed_rtabmap.launch.py \
  svo_path:=/absolute/path/to/bedroom.svo2 \
  database_path:="$HOME/.ros/robosub/maps/replay_trial_01.db" \
  start_rviz:=false
```

Omit `start_rviz:=false` to inspect replay visually. Use a fresh database filename
for each comparison. ZED publishes recorded `/clock`; RTAB-Map, RViz and the robot
description publisher use simulated time. The ZED clock producer uses its own
clock to avoid waiting on itself. Recordings stay under `data/svo/front/`, outside Git.

## ZED-only diagnostics

```bash
ros2 launch vision_bringup zed_front.launch.py
```

This mode runs ZED's original localization profile and displays the current cloud.
The compatibility names `slam_live.launch.py` and `slam_replay.launch.py` still
work for ZED-only diagnostics; the latter requires `svo_path`.

For the previous ZED fused-mapping/PLY comparison, run from the repository root:

```bash
ros2 launch vision_bringup zed_front.launch.py \
  zed_config:="$(pwd)/config/zed_front_mapping_test.yaml" export_fused_map:=true
```

Enable **Fused Room Point Cloud** in the diagnostic RViz display. The retained
exporter saves `maps/spatial_mapping/front_room.ply` every five seconds when new
valid fused data arrives; `/save_fused_map` requests an immediate save. ZED's
`maps/area_memory/front.area` is separate localization memory. Neither is used
by the RTAB-Map profile. Run one camera stack at a time.

## Configuration and tests

- `config/zed_front_rtabmap.yaml`: sensor and smooth local odometry settings.
- `config/rtabmap.yaml`: SLAM, synchronization and cloud settings.
- `ros_ws/src/vision_bringup/launch/zed_front.launch.py`: shared camera, fixed TF,
  optional diagnostic exporter and RViz startup.
- `ros_ws/src/vision_bringup/launch/zed_rtabmap.launch.py`: database mode and RTAB-Map.
- [Architecture](docs/architecture.md): TF ownership and source-derived topic table.
- [Testing procedure](docs/replay_testing.md): bedroom, restart, replay and diagnostics.

Topics can be overridden with `rgb_topic`, `depth_topic`, `camera_info_topic`, and
`odom_topic`; confirm them against the actual running wrapper. `serial_number`,
`start_rviz`, and `camera_to_base_{x,y,z,roll,pitch,yaw}` pass through to camera
bringup. Mount transforms still default to zero until measured.

After building, run the tests from `ros_ws/`:

```bash
source install/setup.bash
colcon test --packages-select vision_bringup --event-handlers console_direct+
colcon test-result --verbose
```

The suite checks launch contracts, installed assets, database path handling, and
real RTAB-Map startup with the project parameters. It does not certify camera
tracking, accumulated geometry, loop closure, or relocalization without sensor data.
