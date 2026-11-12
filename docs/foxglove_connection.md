# Foxglove connection over Tailscale

Use Foxglove on a laptop as the primary live visualization interface for the
headless Jetson AGX Orin (Ubuntu 22.04, ROS 2 Humble, ZED SDK 5.2).
The bridge exposes existing ROS data; it does not open the camera, run YOLO,
build a map, or replace RTAB-Map's database persistence. Keep health monitoring
and ROS logging. Live visualization does not require recording images or bags.

This guide documents the setup procedure, not a completed Jetson/laptop test.
No installation or remote configuration was performed when writing it.

## 1. Connect and inspect the Jetson

Connect Tailscale on the laptop, then SSH:

```bash
ssh robosub@100.71.125.7
```

In each Jetson terminal used below, source the existing workspaces in order:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/Vision2026/ros_ws/install/local_setup.bash

ros2 pkg prefix zed_wrapper
ros2 pkg prefix rtabmap_slam
ros2 pkg prefix vision_bringup
tailscale ip -4
ros2 node list
ros2 topic list -t
```

The expected Tailscale address is `100.71.125.7`; use the reported address if it
has changed. The ZED wrapper is normally installed in `~/ros2_ws`. Using
`local_setup.bash` for the project overlay preserves the explicitly sourced
underlay. Use the same `ROS_DOMAIN_ID` and compatible ROS middleware settings
as the camera/mapping terminals.

## 2. Install only if missing

Check the bridge first:

```bash
ros2 pkg prefix foxglove_bridge
```

If it is not installed:

```bash
sudo apt install ros-humble-foxglove-bridge
```

If apt cannot locate the package, refresh its package index with `sudo apt update`
and retry. Do not run a system upgrade or reinstall the ZED SDK, ROS, or existing
workspaces for this setup.

## 3. Start the existing processing stack only if needed

Existing launch files:

- `vision_bringup/launch/zed_front.launch.py`: camera diagnostics.
- `vision_bringup/launch/zed_rtabmap.launch.py`: camera plus RTAB-Map.
- `scripts/run_front.sh`: entry point for the mapping launch and, where present,
  the persistent logging integration.

If the camera/mapping nodes already run, reuse them and go to step 4. Coordinate
with the team before starting anything; never run both camera launches together.

For a new headless mapping test, in a dedicated Jetson terminal:

```bash
mkdir -p ~/maps_test
cd ~/Vision2026
bash scripts/run_front.sh start_rviz:=false mode:=mapping \
  database_path:=/home/robosub/maps_test/foxglove_trial_01.db
```

Choose a new database filename for each fresh test. Reusing one in mapping mode
extends the existing map. This starts mapping, so it intentionally saves a
database. Foxglove only displays its output.

If only camera diagnostics are needed, use this instead, never alongside the
mapping launch:

```bash
ros2 launch vision_bringup zed_front.launch.py start_rviz:=false
```

The diagnostic camera profile can enable ZED positional tracking and save Area
Memory on shutdown. It is not a strict RGB-only/no-persistence profile, and it
does not produce `/rtabmap/cloud_map`.

## 4. Start Foxglove Bridge independently

In another sourced Jetson terminal, inspect the installed launch arguments:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml --show-args
```

Then launch with the Tailscale address and port:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=$(tailscale ip -4) port:=8765
```

Ensure `tailscale ip -4` returns an address before launching. Binding to the
Tailscale address exposes the bridge on that interface rather than every network
interface. Keep this terminal running. Do not start a second bridge on the same
port. This uses the upstream bridge launch as an independent monitoring component;
the repository's camera and mapping launch files need no changes.

## 5. Connect from the laptop

Open Foxglove, choose **Open connection**, select **Foxglove WebSocket**, and enter:

```text
ws://100.71.125.7:8765
```

Use the current Jetson Tailscale address if different. This direct connection
does not need VNC, X forwarding, an SSH port tunnel, or ROS installed on the laptop.
The laptop and Jetson must be connected to the tailnet, with access to TCP 8765
allowed by its policies and the Jetson firewall. Current Foxglove documentation
lists a developer seat requirement for direct connections; check your account's
access if the option is unavailable.

## 6. Select the actual topics and panels

The checked-in launch uses camera name `zed_front`, and previous live output
confirmed that namespace. A separately configured wrapper may use `zed`.
Do not rename publishers just to match a saved layout; select the running topics.

| Data | Repository default | Alternate camera namespace |
| --- | --- | --- |
| RGB | `/zed_front/zed_node/rgb/color/rect/image` | `/zed/zed_node/rgb/color/rect/image` |
| Live cloud | `/zed_front/zed_node/point_cloud/cloud_registered` | `/zed/zed_node/point_cloud/cloud_registered` |
| Odometry | `/zed_front/zed_node/odom` | `/zed/zed_node/odom` |
| Accumulated map | `/rtabmap/cloud_map` (verify) | Depends on RTAB-Map namespace |
| Transforms | `/tf`, `/tf_static` | Same unless remapped |
| Chair count | `/vision/chair_count` when the detector runs | Independent of camera namespace |
| Chair image | `/vision/chair_image` when the detector runs | Annotated processed RGB image |

- **Image panel:** select the live RGB topic. If bandwidth is limiting, select its
  existing `/compressed` topic if advertised and supported. Do not select depth
  compression as an RGB stream.
- **3D panel:** use `map` as the display frame when RTAB-Map is running. Enable
  `/rtabmap/cloud_map` and the live cloud separately, plus TF frames. For a
  camera-only check, use a frame actually available in TF or the cloud's
  `header.frame_id`. A live cloud is one observation; it is not the accumulated map.
- **Raw Messages / Plot:** inspect odometry, diagnostics (if published), and
  `/vision/chair_count`. Use the ROS graph view where supported to inspect nodes
  and connections. Select `/vision/chair_image` in another Image panel to see
  chair bounding boxes and confidence labels when the detector runs.

Select the accumulated cloud in the panel before judging whether it is publishing:
some map output is generated only when subscribed. Start with RGB and one cloud
to limit bandwidth and Jetson load, then enable additional displays as needed.

## 7. Verify and troubleshoot

On the Jetson, use actual names from `ros2 topic list -t`:

```bash
ss -ltnp | grep ':8765'
ros2 node list
ros2 topic info -v /zed_front/zed_node/rgb/color/rect/image
ros2 topic info -v /rtabmap/cloud_map
ros2 topic hz /zed_front/zed_node/rgb/color/rect/image
```

Stop the rate check with Ctrl+C. Inspect message types and offered/requested QoS
in `topic info -v`; a listed topic does not guarantee that messages are arriving.
For one RGB header without printing the entire image:

```bash
ros2 topic echo /zed_front/zed_node/rgb/color/rect/image \
  --once --field header --qos-reliability best_effort
```

| Symptom | Check |
| --- | --- |
| Laptop cannot connect | Tailscale reachability, correct address, listening TCP port, firewall/tailnet policy, bridge startup errors, and Foxglove account access. |
| Connected but no expected topics | Source all workspaces before starting the bridge; compare ROS domain/middleware settings and inspect actual namespaces. Source custom message packages for future detector topics. |
| Topic listed but no data | Check publisher count, source-node health, and QoS compatibility; inspect bridge warnings before changing publisher settings. |
| Image works but live cloud is absent | Verify depth/cloud publication on the camera; Foxglove does not generate ZED depth. |
| Live cloud works but accumulated map is absent | Verify RTAB-Map is running, receiving synchronized inputs, and publishing the map when subscribed. Camera-only launch has no RTAB-Map map. |
| Cloud has a transform error | Inspect its header frame and timestamp and the TF chain to the display frame. Do not invent a static `map` transform to hide missing localization. |
| Frozen image, USB errors, corrupted frames | Check ZED logs and the USB cable, connection, power, and competing camera processes. If ROS data is already missing/corrupt, it is upstream of Foxglove. |
| High latency or sluggish display | Disable unneeded clouds/panels, try existing compressed RGB, and inspect network and GPU load before changing the processing pipeline. |

In this repository's mapping profile, RTAB-Map owns `map -> odom`, ZED owns
`odom -> zed_front_camera_link`, and the robot description supplies the static
camera frames. Keep exactly one owner for each transform. Previous USB/frame
failures must be diagnosed separately from bridge/network failures.

Acceptance checklist (complete on the actual Jetson and laptop):

- [ ] Laptop connects through Tailscale to the bridge.
- [ ] RGB image updates live in the Image panel.
- [ ] Live cloud updates in the 3D panel.
- [ ] Accumulated RTAB-Map cloud retains previously observed surfaces.
- [ ] TF, odometry, available diagnostics, and message inspection work.
- [ ] Camera/USB faults can be distinguished from bridge or network faults.

These hardware checks have not been performed as part of this documentation change.

## Shutdown and data handling

Ctrl+C in the bridge terminal stops visualization only. To stop mapping, press
Ctrl+C once in its own terminal and wait for clean database saving. Closing
Foxglove does not stop the robot processes. Keep necessary health monitoring and
ROS logs; do not enable bag/image recording unless a test explicitly calls for it.
Store logs and exports outside Git; in-repository run output belongs under an
ignored `logs/` directory. Existing RTAB-Map databases remain separate from logs.

## References

- [Foxglove ROS 2 setup](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)
- [Direct live connections and account requirements](https://docs.foxglove.dev/docs/visualization/connecting/live)
- [Image panel](https://docs.foxglove.dev/docs/visualization/panels/image)
- [3D panel](https://docs.foxglove.dev/docs/visualization/panels/3d)
- [SSH/VNC fallback](remote_connection.md)
