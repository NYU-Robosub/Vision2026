# Bedroom SLAM validation

These are acceptance procedures, not completed results. Keep each outcome and
its evidence in `docs/test_results/`. The current platform is the laptop Linux VM;
repeat on Orin later. Use the build and launch instructions in the root README.

## 1. Record the environment and inputs

Record the commit and uncommitted changes, VM platform, guest Linux version, ROS,
ZED SDK, wrapper version/commit, RTAB-Map version, GPU driver, camera serial,
configuration files, and database/recording filenames. Check `nvidia-smi` and
camera access inside the VM. A missing camera or inaccessible GPU is an
environment problem before it is a mapping problem.

Start the ZED-only diagnostic launch and inspect RGB, depth and current cloud.
From a second sourced terminal:

```bash
ros2 topic list -t
ros2 topic info -v /zed_front/zed_node/rgb/color/rect/image
ros2 topic info -v /zed_front/zed_node/depth/depth_registered
ros2 topic echo /zed_front/zed_node/rgb/color/rect/camera_info --once
ros2 topic echo /zed_front/zed_node/odom --once
ros2 topic hz /zed_front/zed_node/odom
ros2 run tf2_ros tf2_echo odom zed_front_camera_link
```

Confirm the actual calibration/image dimensions, optical frames, depth units and
encoding, timestamp alignment, QoS, finite odometry and covariance. The expected
topic table is in `architecture.md`; override launch topic arguments if the
installed wrapper differs. Record actual values rather than assuming the source
checkout matches the installed binary.

## 2. Start a separate mapping trial

Stop diagnostics first. Choose a new absolute database filename for the trial:

```bash
ros2 launch vision_bringup zed_rtabmap.launch.py \
  database_path:="$HOME/.ros/robosub/maps/bedroom_trial_01.db"
```

Keep **Accumulated Room Map** enabled and the current ZED cloud disabled in RViz.
Fixed Frame is `map` and Decay Time is `0`.

Confirm the active ZED settings and inputs:

```bash
ros2 param get /zed_front/zed_node pos_tracking.publish_map_tf
ros2 param get /zed_front/zed_node pos_tracking.reset_odom_with_loop_closure
ros2 param get /zed_front/zed_node mapping.mapping_enabled
ros2 node info /rtabmap/rtabmap
ros2 topic echo /rtabmap/info --once
ros2 topic hz /rtabmap/cloud_map
ros2 run tf2_ros tf2_echo map odom
```

All three queried ZED parameters should be false. RTAB-Map alone owns global TF.
The `/rtabmap/rtabmap` node should subscribe to the documented RGB, depth,
calibration and odometry topics. No `rgbd_odometry`, `stereo_odometry`, or exporter
should run in this profile. Record graph node count from `/rtabmap/mapGraph`.
Cloud rate is measured under an active subscriber and need not match camera rate.

## 3. Perform the room route

| Test | Action | Evidence to record |
| --- | --- | --- |
| Stationary | Hold camera still for 60 seconds | Translation/rotation drift, tracking status, no spurious room expansion |
| Translation | Move 2-3 m along a measured route | Odometry displacement and newly accumulated geometry |
| Retained geometry | Scan a distinctive wall or desk, then turn away and translate | That region remains in the assembled cloud with current-cloud display disabled |
| Loop closure | Walk a loop and return to the starting pose/view | RTAB-Map loop-closure event/constraint, map correction, endpoint error and alignment |
| Tracking recovery | Briefly obscure the lenses, then revisit the mapped area | Loss duration and recovery behavior; do not treat invalid odometry as valid motion |

Initial indoor targets inherited from the project are less than 5 cm translation
drift over the stationary minute and less than 20 cm closed-loop endpoint error.
Record orientation errors too. These are provisional engineering targets, not
manufacturer guarantees. Do not require node count to grow while stationary or
point count to increase monotonically: filtering and graph correction change it.

For every test record CPU/GPU/RAM usage, RTAB-Map processing rate/latency, odometry
rate, cloud rate, node count, tracking losses, loop events, and database growth.
Use `top`/`htop` and `nvidia-smi` on the laptop; label those results as VM results.
If claiming drift reduction, compare measured alignment/error before and after
the correction, rather than only reporting a loop event.

## 4. Reopen and relocalize

Stop with Ctrl+C and wait for RTAB-Map to close. Preserve the database and logs.
Restart with the same filename in localization mode:

```bash
ros2 launch vision_bringup zed_rtabmap.launch.py mode:=localization \
  database_path:="$HOME/.ros/robosub/maps/bedroom_trial_01.db"
```

Look at a distinctive previously mapped view. Check that the saved graph and room
geometry return and that an observation matches the map. Record the time to
successful localization and the resulting camera pose at a known location.
Simply seeing an old cloud or a TF is not proof of relocalization.

With RViz subscribed, explicitly request the saved global map if it has not yet
been displayed:

```bash
ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap \
  "{global_map: true, optimized: true, graph_only: false}"
```

Confirm the service name/type with `ros2 service list -t` for the installed
release. This requests publication; it does not establish camera localization.
Reopening in `mode:=mapping` permits extending the same graph. Use a different
database filename for an independent map; do not delete the previous trial.

## 5. SVO regression

Record the same stationary/translation/loop route with ZED Explorer or the
wrapper's SVO recording service. Keep stereo and IMU data, a recording identifier,
and a checksum. Large files belong under `data/svo/front/` outside Git.

```bash
ros2 launch vision_bringup zed_rtabmap.launch.py \
  svo_path:=/absolute/path/to/bedroom.svo2 \
  database_path:="$HOME/.ros/robosub/maps/replay_trial_01.db"
```

Use the same recording, route segments and startup conditions for parameter
comparisons, with a fresh database filename each time. Verify `/clock` advances,
and that RTAB-Map, RViz and robot_state_publisher use simulated time. ZED itself
must not wait for its own published clock. SVO playback is not looped.

Compare retained geometry, endpoint error, graph size, loop events, tracking-loss
duration, processing latency and resources. Replay requires the ZED SDK/GPU even
without a physically attached camera. If no recording is available, mark this
test **not run**.

## Troubleshooting by symptom

| Symptom | First checks |
| --- | --- |
| Current cloud only; old geometry disappears | Enable `/rtabmap/cloud_map`, disable current-cloud display, check graph node creation and valid depth |
| RTAB-Map receives no usable data | Actual topic names, camera info, encodings, timestamps, QoS, sync warnings and TF availability |
| Map jumps or doubles unexpectedly | Duplicate TF publishers, ZED odometry reset setting, tracking quality, loop constraints |
| Map absent after restart | Correct database path, clean previous shutdown, graph contents, map subscriber/publication; then check localization separately |
| Sparse or empty depth | Inspect ZED confidence/texture settings and real scene; do not assume a database fault |
| Delayed map or rising memory | Record processing latency, lower workload based on measurements; retain complete-map acceptance while tuning |

## Automated checks and result record

Run the package's tests using the README commands. The launch tests evaluate ROS
substitutions and effective settings. The runtime smoke test starts the actual
RTAB-Map executable on a separate ROS domain with a disposable database; it tests
parameter acceptance and clean startup/shutdown, not room reconstruction.

For each physical/replay run, copy `docs/test_results/template.md` to a new result
file. Fill measurements and link logs, recordings and databases. Update the
acceptance register in `scope.md` only with supporting evidence. Unavailable
hardware and recordings are **not run**, not passing tests.
