# Lucy + Sydney integration testing

Branch: `integrate-lucy-sydney`. Combines Lucy's startup summary (PR #1)
and Sydney's runtime monitor (PR #2). Monitor defaults match this repository's
`/zed_front/zed_node` topics. The monitor is started separately.

## Build and automated tests

Run in Ubuntu 22.04 with ROS 2 Humble, from the repository root. The existing
ZED wrapper, SDK and RTAB-Map must be installed/built as described in README.md.

```bash
source /opt/ros/humble/setup.bash
cd ros_ws
colcon build --symlink-install --packages-select vision_bringup vision_status
source install/setup.bash
colcon test --packages-select vision_bringup vision_status --event-handlers console_direct+
colcon test-result --verbose
```

## Monitor without a camera

In each terminal, source ROS and `ros_ws/install/setup.bash`.
Start the monitor in terminal 1:

```bash
ros2 run vision_status vision_status
```

In terminal 2, publish synthetic RGB messages (do not run the camera concurrently):

```bash
ros2 topic pub -r 10 /zed_front/zed_node/rgb/color/rect/image sensor_msgs/msg/Image '{}'
```

RGB should change from WAITING to OK and report roughly 10 Hz. Stop the publisher
with Ctrl+C: RGB should become NO DATA after three seconds plus up to two seconds
until the next report. Restart it and look for `rgb: data recovered`. Other streams
remain WAITING because this test only publishes RGB. This checks message receipt,
not image validity. Stop both processes before the live test.

## Combined live test

In terminal 1 (with a connected ZED camera and working GPU/SDK):

```bash
ros2 launch vision_bringup zed_rtabmap.launch.py \
  database_path:="$HOME/.ros/robosub/maps/combined_$(date +%Y%m%d_%H%M%S).db"
```

Confirm Lucy's `Vision bring-up summary` shows LIVE ZED, mapping mode, the new
database path, topics, clock and TF ownership. In terminal 2:

```bash
ros2 run vision_status vision_status
```

Confirm RGB, depth, camera_info and odometry receive data. Move the camera slowly
through a textured scene and check cloud_map receives data and RViz retains mapped
geometry. The cloud timeout is 15 seconds; a lack of new map output can cause
NO DATA without implying the camera failed. Stop the launch cleanly with Ctrl+C
while leaving the monitor running to observe timeouts. Restart the launch to check
recovery. Use the same explicit database filename to test the existing-map warning.

Custom launch topic overrides must also be given to the standalone monitor using
`--ros-args -p rgb_topic:=... -p depth_topic:=... -p camera_info_topic:=...` and
`-p odometry_topic:=...` (the launch calls the last argument `odom_topic`).

For SVO, localization and mapping acceptance checks, see [replay_testing.md](replay_testing.md).
Automated tests do not establish live camera, replay or SLAM quality.
