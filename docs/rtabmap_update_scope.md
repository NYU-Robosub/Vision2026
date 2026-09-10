# ZED + RTAB-Map update scope

Historical planning note. The implementation specification and current completion
status now live in [scope.md](../scope.md). Follow that file for implementation.

Status: proposed implementation scope, based on the supplied update and repository inspection on 2026-09-08. No runtime changes or hardware validation have been performed for this scope.

## Outcome and boundary

Use ZED 2i RGB, registered depth, calibration, and visual-inertial odometry as inputs to RTAB-Map. RTAB-Map owns global SLAM, map correction, accumulated geometry, and the persistent database.

The first acceptance gate is a room cloud that retains previously observed surfaces as the camera moves away. The complete milestone adds demonstrated loop closure, database reopening, and relocalization. This is an above-water test; model deployment, semantic perception, controls, external IMU fusion, and underwater tuning are excluded.

Changing the mapping backend does not establish the cause of the previous ZED threshold failure. Invalid depth or odometry must still be identified before evaluating RTAB-Map.

## Architecture decisions

- Keep the existing `vision_bringup` package and directory structure. A repository rename is unnecessary.
- Use RGB-D input with ZED odometry. Do not launch RTAB-Map visual or ICP odometry. The upstream ZED example supports using ZED odometry, but its older topic names, IMU options, and database deletion flag must not be copied unchanged. [Upstream ZED example](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/zed.launch.py)
- Keep ZED internal IMU fusion enabled; RTAB-Map does not need a separate IMU subscription for this initial integration.
- Disable ZED spatial mapping in the RTAB-Map profile from the start. Retain the existing ZED mapping profile as an optional diagnostic comparison.
- Keep the existing exporter out of the RTAB-Map launch. Its PLY snapshots are not the new persistence mechanism. Preserve the existing uncommitted exporter work during implementation.
- Use six-degree-of-freedom SLAM; do not introduce ground-robot planar motion constraints.

### TF ownership

Retain the working camera-root layout for the first implementation:

```text
map                              RTAB-Map publishes map -> odom
  odom                           ZED publishes odom -> zed_front_camera_link
    zed_front_camera_link
      base_link                  robot_state_publisher publishes fixed mount
      camera/optical frames      robot_state_publisher publishes fixed frames
```

Set RTAB-Map's tracking frame to `zed_front_camera_link` initially and validate it against the actual odometry child frame. `base_link` remains transformable into `map`. The proposal's base-root diagram is conceptual; changing URDF parentage is not required for the room demo. A later vehicle-root conversion would need consistent odometry and mount-transform changes, not merely swapping joint labels.

The RTAB-Map-specific ZED profile must retain positional tracking and local TF but disable ZED map TF, Area Memory loading/saving, and `reset_odom_with_loop_closure`. Both existing launch files explicitly pass `publish_map_tf: true`; editing YAML alone is insufficient. The checked-in wrapper resets its odometry transform on ZED loop closure when that reset option is enabled.

RTAB-Map alone publishes `map -> odom`. Use the ZED odometry message, including its covariance, rather than accidentally selecting TF-only odometry. The upstream launch uses `odom_frame_id` to select TF odometry and enables visual odometry by default, so both choices must be explicit. Database and localization arguments are also already supported upstream. [RTAB-Map launch implementation](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_launch/launch/rtabmap.launch.py)

## Work packages

| Work | Proposed files | Deliverable |
| --- | --- | --- |
| Dependencies and baseline | `ros_ws/src/vision_bringup/package.xml`, `CMakeLists.txt`, README | Declare required RTAB-Map packages, install new configuration files, record tested Humble/SDK/wrapper/RTAB-Map versions |
| ZED input profile | `config/zed_front_rtabmap.yaml` | RGB, registered depth, calibration, smooth ZED odometry, internal IMU fusion, no competing global mapping |
| Launch composition | Existing live/replay launches and new `launch/zed_rtabmap.launch.py` | Reuse camera/URDF bringup without starting the legacy saver or duplicate RViz; one SLAM command, ZED-only diagnostics, optional SVO input |
| SLAM configuration | `config/rtabmap.yaml` | Existing odometry input, synchronization/QoS settings, 3D map output, graph optimization, conservative processing settings |
| Database lifecycle | SLAM launch and `maps/rtabmap/` ignore rules | Explicit database path, mapping/resume and localization modes, documented creation of a separate fresh map |
| Visualization | New `rviz/rtabmap_slam.rviz` | Accumulated cloud, separately toggled current cloud, TF, map pose and optimized trajectory, local odometry trajectory |
| Validation and documentation | README, `docs/architecture.md`, `docs/replay_testing.md` | Exact launch commands, verified topic table, tests and results template, troubleshooting |

Prefer directly launching `rtabmap_slam/rtabmap` with project YAML and topic remappings. Add the standard `rtabmap_sync/rgbd_sync` node only if needed to separate RGB-D synchronization from odometry synchronization. No custom SLAM, cloud accumulator, relay, or loop-closure node is needed.

### Input contract to verify on the ROS computer

Source inspection suggests the following topics; these are not a live topic inventory:

| Input | Expected topic |
| --- | --- |
| Rectified RGB | `/zed_front/zed_node/rgb/color/rect/image` |
| Registered depth | `/zed_front/zed_node/depth/depth_registered` |
| RGB calibration | `/zed_front/zed_node/rgb/color/rect/camera_info`; the wrapper also creates an image-suffixed calibration topic |
| Local odometry | `/zed_front/zed_node/odom` |
| Current cloud, for comparison | `/zed_front/zed_node/point_cloud/cloud_registered` |

Verify topic names, message types, matching image dimensions/calibration, depth encoding and units, timestamps, rates, QoS, and frame IDs on the installed wrapper. Decide exact versus bounded approximate synchronization from those measurements. A visible RGB image alone does not prove synchronized SLAM input is arriving.

### Accumulated cloud and persistence

Use RTAB-Map's assembled cloud output (expected `/rtabmap/cloud_map` under the proposed namespace), with configuration verified against the installed release. Ensure RGB-D/keyframe data needed to regenerate geometry is retained, and test cloud reconstruction after reopening a database. RTAB-Map provides assembled cloud publication through its map manager. [Map manager implementation](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_util/src/MapsManager.cpp)

RViz uses `map` as its fixed frame. The accumulated map is the primary enabled display; the current ZED cloud is available as a separate comparison display. Avoid using display history to satisfy the accumulation test. Replace the ZED global path with RTAB-Map's optimized path or graph representation; ZED odometry remains the local trajectory.

Resolve the database to an explicit writable path independent of shell working directory. Preserve it by default: no automatic `-d` or `--delete_db_on_start`. Use a separate database filename for a fresh mapping session. Localization mode must require an existing database and should not silently create an empty replacement. Test saving/reopening before testing relocalization; loading geometry alone does not prove the camera has localized within it.

## Acceptance sequence

| Gate | Test | Evidence |
| --- | --- | --- |
| 1. Sensor baseline | Stationary hold, 2-3 m translation, rotation | Valid RGB/depth/calibration, connected TF, local odometry rate and measured drift |
| 2. SLAM ingestion | Slowly explore a textured room | Nodes added with movement, input synchronization working, no competing TF publishers |
| 3. Live accumulation | Observe a wall, turn away, move to another view | Earlier surfaces remain in the assembled cloud; current-cloud display is disabled for the demonstration |
| 4. Loop closure | Return to the starting area and orientation | Recorded loop constraint, optimized graph, measured endpoint error and surface alignment; local odometry remains smooth |
| 5. Database persistence | Clean shutdown and reopen same database | Existing graph and room geometry are recoverable without remapping |
| 6. Relocalization | Restart in localization mode at a previously observed location | Successful observation-to-map association and measured recovery time |
| 7. Reproducibility and Orin check | Repeat an SVO and a live room run on target hardware | Recorded software versions, configuration, CPU/GPU/RAM use, processing latency, cloud rate, and database growth |

Carry forward existing indoor targets of less than 5 cm stationary translation drift over 60 seconds and less than 20 cm closed-loop endpoint error as provisional project criteria. Record orientation error, tracking losses, relocalization time, loop events, and map node count as well. Do not require nodes to be added while stationary or expect dense map updates at the 30 Hz sensor rate.

Automated checks should cover launch/configuration validity, installation, TF-owner settings, and database mode/path behavior. Hardware/SVO checks establish mapping quality; static checks cannot certify it. No SVO recording or live ROS session was available during scoping.

## Risks, dependencies, and sequencing

The main integration risks are synchronization/QoS, incorrect TF ownership, odometry resets, inadequate depth, and loading a database without republishing its full geometry. RTAB-Map also adds CPU and memory work; limit processing rate and cloud density based on measurements, with optional remote RViz for the Orin trial.

Confirm the Orin variant/RAM, installed JetPack/SDK/ROS versions, and available storage before choosing resource budgets. The repository documents Ubuntu 22.04, ROS 2 Humble, ZED SDK 5.2, and wrapper v5.2.2; confirm the running installation matches. Pin the tested RTAB-Map version rather than assume the current upstream ROS 2 branch matches the installed Humble packages.

Implement in three reviewable increments:

1. Input/TF profile plus RTAB-Map ingestion and accumulated RViz cloud.
2. Database lifecycle, cloud reopening, loop-closure and relocalization validation.
3. SVO regression workflow, Orin profiling, and final operating documentation.

This is a moderate integration change across roughly 10-15 configuration, launch, build, and documentation files. No new mapping algorithm is required. Camera/runtime validation is a separate completion gate from implementing the files; the room demonstration cannot be promised from repository inspection alone.
