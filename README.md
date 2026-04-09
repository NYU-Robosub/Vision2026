# robosub-zed-pipeline

RoboSub ZED-based localization, perception, mapping, and autonomy stack.

## Pipeline

Front ZED 2 VSLAM + Bottom ZED Mini perception -> YOLO-OBB + depth-based 3D target estimation -> persistent object map -> autonomy

## Structure

```text
robosub-zed-pipeline/
|-- README.md
|-- docker/
|   |-- Dockerfile.dev
|   |-- Dockerfile.jetson
|   `-- docker-compose.yml
|-- config/
|   |-- zed_front.yaml
|   |-- zed_bottom.yaml
|   |-- localization.yaml
|   |-- detector_front.yaml
|   |-- detector_bottom.yaml
|   |-- mapping.yaml
|   `-- autonomy.yaml
|-- models/
|   |-- weights/
|   |-- onnx/
|   `-- tensorrt/
|-- data/
|   |-- svo/
|   |   |-- front/
|   |   `-- bottom/
|   |-- rosbags/
|   `-- calibration/
|-- maps/
|   `-- area_memory/
|-- scripts/
|   |-- build.sh
|   |-- run_dev.sh
|   |-- run_jetson.sh
|   |-- export_onnx.sh
|   `-- convert_trt.sh
|-- ros_ws/
|   `-- src/
|       |-- robosub_msgs/
|       |-- localization/
|       |-- perception/
|       |-- mapping/
|       |-- autonomy/
|       `-- bringup/
`-- docs/
    `-- architecture.md
```

## Package Responsibilities

`localization/`
Front ZED 2 pose owner, TF tree, and `map` / `odom` / `base_link`.

`perception/`
Front + bottom detectors with depth-based 3D target estimates.

`mapping/`
Fuses detections into persistent world-frame objects.

`autonomy/`
Task FSM / behavior logic.

`bringup/`
Laptop, playback, and Jetson launch files.

`robosub_msgs/`
Shared custom ROS messages.

`maps/area_memory/`
Stores persistent map artifacts, saved area state, and other reusable mission memory outputs.

`data/rosbags/`
Holds ROS bag recordings for replay, debugging, and offline pipeline validation.

More detail lives in `docs/architecture.md`.
