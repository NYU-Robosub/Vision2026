# Live chair detection

Headless ROS 2 Humble node for Jetson AGX Orin. Subscribes to the existing ZED
rectified RGB topic; does not start or open a camera. By default uses pretrained
COCO `yolov8n.pt`, CUDA device 0, and the `chair` class. The mapping stack is independent.

Publishes `/vision/chair_count` (`std_msgs/msg/Int32`) once per successfully
processed image, including zero when no chairs are detected. This is the count
in the latest processed image, not unique chairs over time. If `chair` is not in
the selected classes, the chair count is zero. Other selected classes are drawn
but are not included in this count. Prints
`Chairs detected: N | FPS: X` about once per second; FPS is completed inferences
per elapsed wall-clock second, including the configured rate limit.

The subscription uses best-effort QoS with depth 1. A single pending image is
replaced by newer arrivals. Separate callback groups and two executor threads
allow image reception during inference, with only one inference at a time.
The requested FPS is a ceiling/target, not a performance guarantee. With no new
images there is no repeated inference or count publication. Failed inference
does not publish a misleading zero; consumers should time out stale counts.

When an image subscriber is present, publishes `/vision/chair_image`
(`sensor_msgs/msg/Image`) with YOLO's chair boxes and confidence labels drawn on
the processed frame. Its timestamp and frame ID come from the input image. The
annotated stream updates at the inference rate and is independent of RTAB-Map;
the original camera image remains available at full camera rate. The node does
not annotate frames when there are no subscribers.

No snapshots, video, detection files, tracking, depth
localization, or map writes are implemented. No GUI is opened. Weights and
dependency caches are installation assets, not recorded camera data.

## Install and build on Jetson

Run from the repository root on Ubuntu 22.04 / ROS 2 Humble. Use a Python 3.10
environment visible to ROS with CUDA-enabled PyTorch and matching torchvision
already installed for the Jetson's **actual JetPack version**. Generic desktop
PyTorch wheels are not a substitute. See the official
[Jetson installation guide](https://docs.ultralytics.com/guides/nvidia-jetson/).
Do not run a second camera launch if the existing ZED feed is already active.

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-cv-bridge python3-colcon-common-extensions python3-pytest
rosdep install --from-paths ros_ws/src/chair_detection --ignore-src -r -y

# Confirm the preinstalled Jetson PyTorch pair; constrain pip to retain it.
python3 -c 'import torch, torchvision; assert torch.cuda.is_available(), "CUDA PyTorch required"; print(torch.__version__, torchvision.__version__, torch.cuda.get_device_name(0))'
python3 -m pip install -r ros_ws/src/chair_detection/requirements.txt \
  --constraint <(python3 -c 'from importlib.metadata import version; print("torch==" + version("torch")); print("torchvision==" + version("torchvision"))')
python3 -c 'import torch, torchvision, cv_bridge; from ultralytics import YOLO; assert torch.cuda.is_available(); print(torch.cuda.get_device_name(0))'

cd ros_ws
colcon build --packages-select chair_detection --symlink-install
source install/setup.bash
colcon test --packages-select chair_detection --event-handlers console_direct+
colcon test-result --test-result-base build/chair_detection --verbose
```

Ultralytics is pinned to the existing detector baseline version. NumPy stays
below 2 for Humble's binary `cv_bridge`; the OpenCV bound avoids pulling NumPy 2
back in. Install Python requirements explicitly: colcon/rosdep do not install
Ultralytics or select JetPack-specific PyTorch wheels for you. If using a virtual
environment, it must expose ROS/system packages and the Jetson CUDA PyTorch pair;
build and run with that same environment active.

## Run against the existing live camera

From `ros_ws/`, with the environment above sourced and the ZED publisher running:

```bash
ros2 topic info /zed_front/zed_node/rgb/color/rect/image
ros2 launch chair_detection chair_detection.launch.py
```

Keep the existing mapping launch and Foxglove bridge running. In Foxglove, add
an Image panel and select `/vision/chair_image`; keep `/rtabmap/cloud_map`
enabled in a 3D panel with display frame `map`. The box positions are 2D image
detections, not chair positions in the map. GPU load and network bandwidth may
reduce the rate of detection or mapping; monitor both on the Jetson.

In another sourced terminal:

```bash
ros2 topic echo /vision/chair_count
ros2 topic hz /vision/chair_count
```

`yolov8n.pt` is downloaded by Ultralytics on first use if missing (internet needed).
For an offline Jetson, provision the pretrained weights first and use an absolute
path. CUDA absence or a requested class absent from the chosen model fails startup.

All parameters are startup-only. Override them directly:

```bash
ros2 run chair_detection chair_detection --ros-args \
  -p model_path:=/absolute/path/to/yolov8n.pt \
  -p 'object_classes:=[chair,person]' \
  -p image_topic:=/zed_front/zed_node/rgb/color/rect/image \
  -p confidence_threshold:=0.5 \
  -p inference_fps:=10.0
```

Or copy `src/chair_detection/config/chair_detection.yaml`, edit it, and launch:

```bash
ros2 launch chair_detection chair_detection.launch.py params_file:=/absolute/path/to/chair_detection.yaml
```

Keep the same ROS domain and compatible networking as the existing camera.
Use Ctrl+C to stop only the detector. Verify a chair produces a positive count,
removing all chairs produces zero, and stopping the image stream stops updates.

## Validation performed

Built this package in isolation on Ubuntu 22.04 WSL / ROS 2 Humble, Python 3.10.
All 15 pytest cases passed using real ROS messages, executors, topic transport,
and CvBridge, with a fake YOLO model and synthetic in-memory images. Checked the
installed launch file with `ros2 launch ... --show-args`. No camera or mapping
nodes were started. The test environment selected Ubuntu's NumPy 1.x through
`PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH` because its unrelated
system-wide NumPy 2 installation is incompatible with Humble's binary CvBridge.

**Live detection, actual YOLO/CUDA inference, and Jetson performance are not
tested.** The tests do not require or download weights, torch, or Ultralytics.
The updated suite covers class selection, chair-only counting, annotated image
headers, rendering only for viewers, and continued counting after rendering
failure. The annotation test uses a fake model render; real YOLO drawing still
needs a live check. Run the commands above on the Jetson to validate the real
model and camera.
