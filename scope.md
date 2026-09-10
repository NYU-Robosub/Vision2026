# ZED + RTAB-Map update scope

Status: scoped launch/configuration implementation delivered; package build and 17 automated tests pass in Ubuntu 22.04/ROS 2 Humble, including actual RTAB-Map startup. Bedroom camera tests and SVO acceptance remain pending. See [implementation checks](docs/test_results/implementation_checks.md).

This is the canonical scope for this update. Track implementation here and link test evidence below. The earlier `docs/rtabmap_update_scope.md` is a historical scoping note; maintain future requirements in this file.

## Outcome and boundary

Use ZED 2i RGB, registered depth, calibration, and visual-inertial odometry as inputs to RTAB-Map. RTAB-Map owns global SLAM, map correction, accumulated geometry, and the persistent database.

The first acceptance gate is a room cloud that retains previously observed surfaces as the camera moves away. The complete milestone adds demonstrated loop closure, database reopening, and relocalization. This is an above-water test; model deployment, semantic perception, controls, external IMU fusion, and underwater tuning are excluded.

Changing the mapping backend does not establish the cause of the previous ZED threshold failure. Invalid depth or odometry must still be identified before evaluating RTAB-Map.

### Current test environment

The user is currently testing on a laptop using a Linux VM. Use that environment for initial camera bringup, room mapping, persistence, relocalization, and SVO regression tests. Jetson Orin remains the eventual deployment target; Orin access and performance validation are not prerequisites for completing the laptop demonstration.

Record the VM platform, guest Linux version, available GPU/CUDA access, camera USB access, and installed ROS/ZED versions when validating the running stack. These details have not yet been confirmed. Report laptop/VM measurements as such; do not treat them as Jetson performance results.

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
| 7. Laptop/VM reproducibility | Repeat an SVO and a live room run in the current Linux VM | Recorded software versions, configuration, CPU/GPU/RAM use, processing latency, cloud rate, and database growth |

Carry forward existing indoor targets of less than 5 cm stationary translation drift over 60 seconds and less than 20 cm closed-loop endpoint error as provisional project criteria. Record orientation error, tracking losses, relocalization time, loop events, and map node count as well. Do not require nodes to be added while stationary or expect dense map updates at the 30 Hz sensor rate.

Automated checks should cover launch/configuration validity, installation, TF-owner settings, and database mode/path behavior. Hardware/SVO checks establish mapping quality; static checks cannot certify it. No SVO recording or live ROS session was available during scoping.

## Risks, dependencies, and sequencing

The main integration risks are synchronization/QoS, incorrect TF ownership, odometry resets, inadequate depth, and loading a database without republishing its full geometry. RTAB-Map also adds CPU and memory work; limit processing rate and cloud density based on measurements, with optional remote RViz for the Orin trial.

Confirm the laptop/VM resources and installed Linux/ROS/SDK versions before choosing initial resource budgets. The repository documents Ubuntu 22.04, ROS 2 Humble, ZED SDK 5.2, and wrapper v5.2.2; confirm the running installation matches. Pin the tested RTAB-Map version rather than assume the current upstream ROS 2 branch matches the installed Humble packages. Confirm the Orin variant/RAM, JetPack compatibility, and storage separately when deployment work begins.

Implement in three reviewable increments:

1. Input/TF profile plus RTAB-Map ingestion and accumulated RViz cloud.
2. Database lifecycle, cloud reopening, loop-closure and relocalization validation.
3. SVO regression workflow, laptop/VM profiling, and final operating documentation. Repeat validation on Orin later as a deployment gate.

This is a moderate integration change across roughly 10-15 configuration, launch, build, and documentation files. No new mapping algorithm is required. Camera/runtime validation is a separate completion gate from implementing the files; the room demonstration cannot be promised from repository inspection alone.

## Implementation checklist

Checkboxes below track code and documentation delivery. The acceptance table that follows tracks demonstrated behavior separately.

### Increment 1: live room accumulation

- [ ] Record installed software versions and verify ZED topics, calibration, QoS, timestamps, and odometry frames on the ROS computer.
- [x] Add the RTAB-Map ZED profile and resolve TF ownership in launch arguments as well as YAML.
- [x] Add RTAB-Map package dependencies and install its configuration/launch/RViz files.
- [x] Compose diagnostic and SLAM modes without duplicate camera, odometry, TF, or visualization nodes.
- [x] Wire RGB-D and ZED odometry with exact synchronization and acquisition-time stamping; verify actual input alignment in gate 1.
- [x] Configure the assembled room cloud and separate current-cloud display in RViz.
- [x] Add and run the small automated checks described below.
- [x] Document the exact launch command.
- [ ] Conduct acceptance gates 1-3 with the camera.

### Increment 2: persistent SLAM

- [x] Implement explicit database paths, resume mapping, and localization against an existing database.
- [x] Ensure normal startup preserves existing maps and missing localization databases produce a clear failure.
- [ ] Confirm reopening a database reconstructs the previous graph and cloud.
- [ ] Record loop closure, graph correction, and restart/relocalization results for gates 4-6.
- [x] Document clean shutdown, reopening, and starting a separate new map.

### Increment 3: repeatability in the laptop Linux VM

- [x] Support the same SLAM configuration in SVO replay with a consistent clock across nodes; clock settings tested, actual playback pending.
- [ ] Record a reusable room sequence and its metadata outside Git for large recordings.
- [ ] Run the replay regression and a live test in the laptop Linux VM.
- [ ] Record processing latency, sensor/cloud rates, CPU/GPU/RAM usage, and database growth.
- [x] Update architecture, setup, launch, troubleshooting, and test instructions to match the delivered stack.

### Later deployment gate: Jetson Orin

- [ ] Confirm the target Orin hardware and compatible software baseline.
- [ ] Repeat the accepted room and replay tests on Orin and record its resource usage separately.

These deployment checks do not block acceptance of the current laptop/VM milestone. Model deployment remains outside this update.

## Minimum test suite

Build a small suite alongside implementation. Do not delay the first integration to build a large testing framework, and do not retest RTAB-Map or ZED algorithms with mocks. Tests should catch project-level failures and protect saved data.

| Layer | Minimum coverage | Environment and timing |
| --- | --- | --- |
| Static/build checks | Python launch syntax; YAML/XML parsing; package build; new assets discoverable from the installed package | Syntax/parsing where dependencies exist; package/build checks on Ubuntu ROS 2, after affected edits |
| Launch/configuration tests | Resolved SLAM launch retains ZED local TF, disables ZED global TF, starts only RTAB-Map global SLAM, selects odometry topic input, and excludes duplicate odometry and the legacy saver | Small pytest tests with ROS launch dependencies; evaluate effective launch parameters, not source-string matching |
| Database behavior tests | Existing database preserved on normal startup; explicit paths resolve consistently; missing localization database rejected; separate new-map path supported | Temporary test directories and disposable fixtures; never use the team's real room database |
| ROS integration/replay | Synchronized input creates graph nodes; map contains finite geometry; earlier geometry remains after viewpoint changes; database reload recovers the graph/cloud | Ubuntu with ZED SDK/GPU and a known SVO or suitable recorded ROS inputs; run after SLAM-affecting changes |
| Physical acceptance | Translation, rotation, loop closure, camera tracking loss/recovery, restart/relocalization, and laptop/VM resource usage | Real camera and textured room using the Linux VM; repeat for milestone releases and material sensor/configuration changes, then on Orin during deployment |

Implemented tests live in `ros_ws/src/vision_bringup/test/test_bringup.py`, registered through ament/pytest. They evaluate launch contracts, installed assets, database preservation/path handling, and real RTAB-Map startup with a disposable database. Physical/replay procedures are in `docs/replay_testing.md`; their acceptance results remain separate from the automated suite.

On the ROS computer, register package tests so the standard workflow applies from `ros_ws/` after sourcing ROS and installing dependencies:

```bash
colcon build --symlink-install --packages-select vision_bringup
source install/setup.bash
colcon test --packages-select vision_bringup --event-handlers console_direct+
colcon test-result --verbose
```

A green test command is meaningful only when tests were discovered and executed. Report discovered/executed test counts and failures. Hardware-dependent checks must explicitly report `not run` when the device, GPU runtime, or recording is unavailable; they must not pass using empty data or be treated as acceptance evidence because they were skipped.

For the accumulation regression, inspect retained geometry in a previously observed region, allowing for loop-closure correction and voxel filtering. Do not assert that raw point count must always increase. For relocalization, require a successful match/localization signal and a plausible map-frame pose; loading a database or publishing a cloud alone is insufficient.

### Acceptance evidence register

Allowed statuses: `not run`, `pass`, `fail`, or `blocked` with a concrete missing dependency. Keep implementation status separate from these results.

| Gate | Status | Evidence / measured result |
| --- | --- | --- |
| 1. Sensor baseline | not run | Pending camera/ROS validation |
| 2. SLAM ingestion | not run | Launch and real node startup checked; no sensor sequence available |
| 3. Live accumulation | not run | Awaiting bedroom camera test |
| 4. Loop closure | not run | Awaiting bedroom camera test |
| 5. Database persistence | not run | Database handling/startup tested; reopening actual mapped geometry still pending |
| 6. Relocalization | not run | Awaiting saved room map and camera test |
| 7. Laptop/VM replay | not run | Pending recording and Linux VM validation |

For each run, store a short result in `docs/test_results/` with date, commit plus any uncommitted changes, device/OS/software versions, launch command, effective configuration, recording identifier, measured values, pass/fail rationale, and log/artifact locations. Keep large recordings and databases outside Git. Check all acceptance gates before declaring the full milestone complete.

## Working with an AI coding agent

Use a small set of documents with distinct jobs:

- `scope.md`: the agreed outcome, boundaries, architecture, implementation checklist, and acceptance status.
- `AGENTS.md` (recommended future addition): short, durable repository instructions, such as build/test commands, TF ownership rules, preservation of existing work/maps, and a pointer to this scope. Do not duplicate the entire specification. Codex reads applicable `AGENTS.md` instructions automatically; an arbitrary scope file should be explicitly referenced in the task or those instructions. [Official Codex guidance](https://developers.openai.com/codex/guides/agents-md)
- `README.md`: commands a teammate needs to build and operate the delivered system.
- `docs/replay_testing.md` and `docs/test_results/`: repeatable procedures and actual evidence.

Implement one increment at a time. Have the agent inspect current files, make the bounded change, run relevant checks, inspect the diff, and update status with evidence. Add regression tests when a concrete defect is fixed. Keep tests tied to requirements rather than implementation details, and do not weaken an acceptance criterion just to make a run pass.

Example implementation request:

> Read scope.md and the applicable repository instructions. Implement Increment 1, including its automated checks. Preserve existing unrelated changes and map files. Run available validation and report commands, test counts, failures, and hardware checks that could not run. Update the implementation checklist and acceptance evidence without marking untested behavior as passed. Do not implement later increments.

Implementation decisions discovered during validation: use exact synchronization because the tested RTAB-Map 0.23.7 core node does not expose `approx_sync_max_interval`; preserve ZED acquisition timestamps explicitly. No extra synchronization node was introduced. The earlier draft remains historical context; this file and the delivered configuration record the current decisions.
