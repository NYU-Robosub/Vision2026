# Architecture

## Overview

The stack is organized as a staged pipeline:

Front ZED 2 VSLAM + Bottom ZED Mini perception -> YOLO-OBB + depth-based 3D target estimation -> persistent object map -> autonomy

At a system level, localization owns the global pose reference, perception produces camera-level detections and 3D estimates, mapping promotes those detections into persistent world-frame objects, and autonomy consumes that mapped world state for task execution.

## Package Responsibilities

### `localization/`

- Front ZED 2 pose owner.
- Owns the TF tree.
- Keeps `map`, `odom`, and `base_link` consistent.

### `perception/`

- Runs the front and bottom detectors.
- Uses YOLO-OBB for oriented detections.
- Produces depth-based 3D target estimates.

### `mapping/`

- Fuses detections from both cameras.
- Maintains persistent world-frame objects.

### `autonomy/`

- Hosts the task FSM / behavior logic.
- Consumes the persistent object map instead of raw detections.

### `bringup/`

- Provides laptop launch flows.
- Provides playback launch flows.
- Provides Jetson launch flows.

### `robosub_msgs/`

- Defines shared custom ROS messages.
- Provides the common interfaces used across localization, perception, mapping, autonomy, and bringup.

## Suggested Data Flow

1. `localization/` publishes the vehicle pose and camera transforms.
2. `perception/` runs the front and bottom detection pipeline, then publishes detections and depth-derived 3D target estimates.
3. `mapping/` fuses those observations into persistent world-frame objects.
4. `autonomy/` consumes the persistent object map to drive higher-level behavior.
