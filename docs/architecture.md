# Architecture

## Active stack

```text
ZED 2i -> ZED wrapper -> RGB + registered depth + calibration + local odometry
                                    |
                                    v
                             RTAB-Map SLAM
                         graph + database + cloud
                                    |
                                    v
                                   RViz
```

`vision_bringup` composes upstream nodes. Its only custom runtime node is the
existing optional ZED fused-cloud exporter, retained for diagnostics and excluded
from the RTAB-Map launch. There are no custom perception or SLAM nodes.

`zed_front.launch.py` owns shared camera and robot-description startup. Live and
replay diagnostic entry points include it, as does `zed_rtabmap.launch.py`.
Replay is selected by `svo_path`, so the mapping configuration is shared with live
operation. `rtabmap_sync/rgbd_sync` is not needed initially: the RTAB-Map node
synchronizes the four subscribed inputs directly.

## TF ownership in the RTAB-Map profile

```text
map
  odom                         RTAB-Map: map -> odom
    zed_front_camera_link      ZED: odom -> camera
      base_link                robot_state_publisher: fixed mounting transform
      ZED optical frames       robot_state_publisher: camera model
```

RTAB-Map uses `zed_front_camera_link` as its tracking frame. The camera-root tree
is intentional; the AUV frame remains connected without an odometry adapter.
ZED internal IMU fusion remains enabled. ZED Area Memory, map TF, loop-closure
odometry reset and spatial mapping are disabled for this profile. Global map
corrections belong to RTAB-Map; local odometry must remain smooth.

The ZED-only diagnostic profile retains ZED map TF and Area Memory. It must not
run alongside the RTAB-Map camera launch.

## Input and output contract

These names were checked against the source checkout and RTAB-Map startup.
Actual ZED publications, calibration, rates, and timestamp alignment still need
verification with the attached camera on the Linux VM.

| Input | Expected ZED topic | RTAB-Map subscription |
| --- | --- | --- |
| Rectified RGB | `/zed_front/zed_node/rgb/color/rect/image` | `rgb/image` |
| Registered depth | `/zed_front/zed_node/depth/depth_registered` | `depth/image` |
| RGB camera calibration | `/zed_front/zed_node/rgb/color/rect/camera_info` | `rgb/camera_info` |
| Local odometry | `/zed_front/zed_node/odom` | `odom` |

The wrapper also publishes RGB calibration under the image-suffixed path
`/zed_front/zed_node/rgb/color/rect/image/camera_info`. Use the calibration that
matches the rectified RGB dimensions and optical frame. Depth must be registered
to that image. The current registered point cloud is only a comparison display.

The initial subscription policy is best effort with exact synchronization and
queues of 10. ZED acquisition timestamps must match across RGB, depth, calibration
and odometry; publication-time stamping is explicitly disabled. Verify this on
the real stream. RTAB-Map 0.23.7's core subscriber does not expose
`approx_sync_max_interval`; setting that YAML key would silently leave the intended
bound unapplied. If actual stamps do not match, investigate them before selecting
a different synchronization arrangement.

| Output | Purpose |
| --- | --- |
| `/rtabmap/cloud_map` | Assembled colored geometry from map nodes; main RViz display |
| `/rtabmap/mapPath` | Optimized map trajectory |
| `/rtabmap/mapGraph` | Pose graph and constraints |
| `/rtabmap/info` | Processing and loop-closure diagnostics |
| `/rtabmap/localization_pose` | Map-frame pose estimate; verify a successful match for relocalization |
| `/zed_front/zed_node/path_odom` | ZED local odometry trajectory |

## Persistence and resource choices

The database retains graph and sensor data; all stored nodes initialize working
memory when reopened for this small-room phase. Mapping can extend it;
localization sets `Mem/IncrementalMemory` false. Neither mode deletes the database.
The launch rejects relative database paths and missing/empty localization files.
Database contents are validated by RTAB-Map itself.

The assembled map has no nearby-node count limit, uses 5 cm voxels and depth up to
5 m. A 2 Hz SLAM target keeps the initial workload below camera rate. These choices
prioritize a complete bedroom cloud; larger environments and Orin deployment
require measured resource budgets. A subscriber must request the map output for
its visualization work to occur; RViz provides that subscriber during the demo.

## Future work

After the laptop bedroom tests pass, repeat them on Jetson Orin. Object detection,
depth-based object localization, and a semantic map can follow. Underwater optics,
external sensor fusion, vehicle control, and detector deployment are separate
milestones. No placeholders for them are included here.
