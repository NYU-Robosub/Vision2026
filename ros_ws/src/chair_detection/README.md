# Configurable object detection

Headless ROS 2 Humble node for Jetson AGX Orin. `DetectionNode` loads a compatible
Ultralytics YOLO detection checkpoint once and filters by `object_classes` using
the selected model's label table. Unknown class names fail startup with the
available names. Parameters are startup-only; restart to change weights/classes.
The package, executable, launch file, and ROS node name remain `chair_detection`.

## Outputs

- `/vision/detections` (`std_msgs/msg/String`): JSON for each successfully processed
  image, with its input `header` (`frame_id`, `stamp.sec`, `stamp.nanosec`) and
  `classes`, keyed by model class name. Each class contains `class_id`, `count`,
  and `instances`. Each instance contains `bbox_xyxy` (four input-image pixel
  coordinates: left, top, right, bottom) and `confidence` (0 to 1).
- `/vision/detection_image` (`sensor_msgs/msg/Image`): selected-class boxes and
  confidence labels, rendered only when subscribed, preserving the input header.
- Console: latest per-class counts and completed inference FPS about once a second.

Example `classes` entry:

```json
{"chair":{"class_id":56,"count":1,"instances":[{"bbox_xyxy":[10,20,100,200],"confidence":0.91}]},"dining table":{"class_id":60,"count":0,"instances":[]}}
```

Counts are detected instances in one frame, not unique tracked identities across
frames. Every selected class is present, including zero counts and empty instance
lists. Inference failures publish no result; consumers must time out stale data.
The old `/vision/chair_count` and `/vision/chair_image` topics are replaced by the
outputs above. Update Foxglove Image panels to `/vision/detection_image`.

`detect(frame)` accepts a BGR image and returns `(yolo_result, classes)`; the ROS
callback adds the input header and publishes the structured results. This method
uses the already-loaded model. It does not load weights on each call.

## Setup and run

Use Ubuntu 22.04 / ROS 2 Humble with Python 3.10 and JetPack-compatible CUDA PyTorch
and torchvision installed. CUDA device 0 is required; CPU fallback is not provided.
Install ROS `cv_bridge`, colcon, and pytest, run rosdep for this package, and install
`requirements.txt` while retaining the Jetson's installed torch/torchvision pair.
See the [Jetson installation guide](https://docs.ultralytics.com/guides/nvidia-jetson/).
Do not start another camera if the existing ZED feed is active.

From the repository root on the Jetson, after installing the compatible CUDA
PyTorch/torchvision pair:

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-cv-bridge python3-colcon-common-extensions python3-pytest
rosdep install --from-paths ros_ws/src/chair_detection --ignore-src -r -y

# Confirm CUDA and retain the installed JetPack-compatible torch/torchvision pair.
python3 -c 'import torch, torchvision; assert torch.cuda.is_available(); print(torch.__version__, torchvision.__version__, torch.cuda.get_device_name(0))'
python3 -m pip install -r ros_ws/src/chair_detection/requirements.txt \
  --constraint <(python3 -c 'from importlib.metadata import version; print("torch==" + version("torch")); print("torchvision==" + version("torchvision"))')
python3 -c 'import torch, torchvision, cv_bridge; from ultralytics import YOLO; assert torch.cuda.is_available()'
```

Use the same Python environment for building and running. A virtual environment
must expose ROS/system packages and the Jetson CUDA PyTorch installation. Colcon
and rosdep do not install the pinned Python requirements or choose CUDA wheels.

```bash
source /opt/ros/humble/setup.bash
cd ros_ws
colcon build --packages-select chair_detection --symlink-install
source install/setup.bash

ros2 run chair_detection chair_detection --ros-args \
  -p model_path:=/absolute/path/to/yolov8n.pt \
  -p 'object_classes:=[chair,dining table]' \
  -p confidence_threshold:=0.5 \
  -p inference_fps:=10.0
```

Use exact names from your checkpoint: standard COCO weights use `dining table`,
not `table`. Custom weights may use different names. Weights must exist on the
machine running the node. The default `yolov8n.pt` must already exist in the
working directory; startup never downloads missing weights. Provision them first.

For reusable configuration, copy `config/chair_detection.yaml`, edit its parameters
under `chair_detection.ros__parameters`, and pass its absolute path:

```bash
ros2 launch chair_detection chair_detection.launch.py params_file:=/absolute/path/to/detector.yaml
ros2 topic echo /vision/detections
```

Default input: `/zed_front/zed_node/rgb/color/rect/image`. The latest pending frame
replaces older ones; inference never overlaps and runs at a configured maximum
rate. No new frames means no results. No images, videos, or detection files are
saved. No depth localization, tracking, or map writes are implemented.

## Jetson testing workflow

### 1. Provision and inspect the checkpoint

Download or copy the trained checkpoint onto the Jetson, for example to
`$HOME/robosub_models/gate/v3/best.pt`. Use versioned directories so different
files named `best.pt` remain distinguishable. Supply an absolute path on the
Jetson, not a path on the laptop. Ultralytics is the inference library; custom
weights are loaded as the selected model, not layered over a default model.
No training occurs when starting this node.

Inspect the actual checkpoint in the detector's Python environment:

```bash
test -f "$HOME/robosub_models/gate/v3/best.pt"
python3 -c 'from pathlib import Path; from ultralytics import YOLO; m = YOLO(str(Path.home() / "robosub_models/gate/v3/best.pt")); print(m.task); print(m.names)'
```

Use a compatible Ultralytics object detection checkpoint (`detect`). A `.pt`
extension alone does not establish compatibility. Choose `object_classes` from
the printed labels. Supplying classes does not teach the model new objects.

### 2. Start or reuse the camera

In each terminal, source `/opt/ros/humble/setup.bash` and the workspace's
`ros_ws/install/setup.bash`. Start one existing camera profile if it is not
already running. From the repository root, for camera diagnostics:

```bash
ros2 launch vision_bringup zed_front.launch.py
```

Alternatively, use `zed_rtabmap.launch.py` when mapping is wanted; it already
starts the camera. Do not launch both camera profiles together. These packages
and the ZED dependencies must be built separately from a detector-only build.
Detection itself does not require RTAB-Map.

In another sourced terminal, verify the input before starting the detector:

```bash
ros2 topic info /zed_front/zed_node/rgb/color/rect/image
ros2 topic hz /zed_front/zed_node/rgb/color/rect/image
```

The image topic must have an active publisher and deliver frames. Use the same
ROS domain and compatible networking for camera, detector, and viewers.

### 3. Start the detector with the local weights path

```bash
ros2 run chair_detection chair_detection --ros-args \
  -p model_path:="$HOME/robosub_models/gate/v3/best.pt" \
  -p 'object_classes:=[gate]' \
  -p confidence_threshold:=0.25 \
  -p inference_fps:=10.0
```

Startup requires an existing, nonempty local `.pt` file, loads the checkpoint
once, checks that its task is `detect`, moves it to CUDA device 0, validates
requested class names, and only then creates detection publishers and subscribes
to the image topic. The startup log identifies the resolved weights path, input
topic, and selected classes. Retain the command or YAML for the experiment.

Missing/empty files, unsupported model tasks, checkpoint loading failures, missing
CUDA, invalid confidence/FPS, and mismatched labels abort detector startup with
a fatal error and a nonzero process exit. For example, requesting `chair` from a
gate-only model raises `Classes not found in model: ['chair']. Available: ['gate']`.
Fix the configuration and restart; it does not silently switch models or classes.
These checks stop the detector, not separately running camera/mapping processes.
They do not guarantee camera availability or model accuracy: missing camera frames
are reported by the runtime stale-count warning rather than a startup failure.

For repeatable tests, store the same settings in a YAML file on the Jetson:

```yaml
chair_detection:
  ros__parameters:
    model_path: /home/YOUR_USER/robosub_models/gate/v3/best.pt
    object_classes: [gate]
    image_topic: /zed_front/zed_node/rgb/color/rect/image
    confidence_threshold: 0.25
    inference_fps: 10.0
```

Replace `YOUR_USER` with the real account name; YAML does not expand `$HOME`.
Pass this file using the `params_file` launch argument shown above.

### 4. Check expected results

```bash
ros2 topic echo /vision/detections
ros2 topic hz /vision/detections
```

- A successfully processed image produces one JSON message with the source
  timestamp and frame ID, each selected class's count, and its instances.
- A detected gate produces a `gate` entry with a positive count and one bounding
  box and confidence per detection. Multiple selected classes have separate entries.
- A frame without a detected gate produces `count: 0` and `instances: []`.
  Zero means the model did not detect a gate, not proof that no gate is present.
- Console output resembles `gate: 1 | FPS: 9.5`. These are illustrative numbers,
  not promised performance. Confidence is a model score, not measured accuracy.
- With the Foxglove bridge running, select `/vision/detection_image` in an Image
  panel to see boxes, class labels, and confidence. No image subscriber means
  rendering is skipped, while structured detections continue.
- Stopping the camera stops result updates. Failed inference logs an error and
  publishes no result, rather than a misleading zero. No completed inference
  triggers a stale-count warning. Receiving no frames does not prevent startup.

Check new images/scenes with targets, without targets, and at different distances,
angles, lighting, and occlusion. Inspect misses, false positives, and box fit.
Compare model versions on the same footage/settings. Measure FPS with mapping
and viewers enabled if those will run together on the robot. Counts are not
accumulated across frames, and boxes are 2D pixels, not map coordinates.

### 5. Swap models without restarting the camera

Stop only the detector with Ctrl+C. Restart it with a different `model_path` and
matching `object_classes`, or another saved YAML configuration. No rebuild is
needed to change weights or parameters. Run only one detector publishing these
output topics at a time, so results from different models are not mixed.

### Bringup status and guidance for agents

The desired normal-operation interface is one bringup that starts the camera,
detector, and optional mapping/viewer components, forwarding the weights path and
target classes to the detector. **This integration is not implemented yet.**
Currently `zed_front.launch.py` and `zed_rtabmap.launch.py` do not start detection.
The detector launch accepts `params_file`; it does not expose `model_path` or
`object_classes` as launch arguments. Use `ros2 run ... --ros-args -p ...` for direct
parameter overrides, or the detector launch with YAML. Do not describe proposed
combined-bringup commands as working commands. Preserve the individual detector
launch for model swapping while the camera/mapping remain running.

When helping someone run this on a Jetson, first establish their JetPack/CUDA
PyTorch environment, local checkpoint path, actual model labels, and active RGB
topic. Do not claim live hardware validation from the fake-model tests.

### Troubleshooting

| Symptom | Check |
| --- | --- |
| CUDA unavailable / torch import fails | JetPack-compatible torch and torchvision in the Python environment used by ROS |
| Model fails to load | File exists on Jetson; checkpoint format/task and library version are compatible |
| Requested class not found | Use the label names printed by the checkpoint; COCO uses `dining table` |
| Startup succeeds but no results | Input frames, configured topic, ROS domain/network, and inference errors |
| Counts work but image is absent | Subscribe to `/vision/detection_image`; verify the Foxglove bridge exposes it |
| Low FPS | Model size, GPU memory/load, rendering, camera rate, configured inference cap, and competing mapping workload |

## Validation

The ROS tests use fake YOLO results and synthetic images with real ROS transport
and CvBridge. They cover multiple class counts, boxes and confidence, zero counts,
class filtering, headers, latest-frame reception, failure recovery, and optional
annotation. They do not validate real YOLO/CUDA inference or Jetson performance.
The suite also checks local checkpoint validation, model-task rejection, CUDA
and checkpoint loading errors, and label-mismatch startup failure before any
detection publisher is created. Jetson camera/CUDA acceptance remains pending.

```bash
colcon test --packages-select chair_detection --event-handlers console_direct+
colcon test-result --test-result-base build/chair_detection --verbose
```
