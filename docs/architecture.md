# Architecture

## Active Phase: ZED SLAM Validation

The active repository intentionally contains one ROS 2 package and no custom runtime nodes:

```text
ZED 2i
  |
  v
ZED ROS 2 wrapper
  |-- RGB and registered depth
  |-- registered point cloud
  |-- visual-inertial odometry
  |-- loop-closure-corrected pose
  |-- Area Memory
  `-- map and odom TF
  |
  +--> robot_state_publisher --> fixed AUV/camera frames
  `--> RViz2                --> visualization and inspection
```

`vision_bringup` provides the live and SVO-replay launch files, the ZED parameter overrides, the camera/AUV xacro, and the RViz configuration.

## Why this is the minimum

Phase 1 evaluates the ZED SDK rather than custom perception code. Adding an odometry adapter, detector, semantic messages, or a world model would add failure modes without improving the answer to the current question: can the ZED 2i localize, close loops, relocalize, and reconstruct useful geometry in the target environment?

## Future Architecture

After Phase 1 passes, add capabilities in this order:

1. A motion-facing odometry/health adapter.
2. A Python competition-object detector.
3. Synchronized depth-based 3D object localization.
4. Semantic observation messages.
5. A C++ persistent world-model node.
6. Bottom-camera perception as a separate node, if required by chosen tasks.
7. Jetson/TensorRT deployment after profiling.

The future semantic flow is:

```text
ZED RGB + depth --> detector/localizer --> object observations
ZED SLAM + TF ------------------------------------|
                                                  v
                                         semantic world model
                                                  |
                                                  v
                                         motion and autonomy
```

Motion control, PID, thrusters, and mission logic remain outside this repository.
