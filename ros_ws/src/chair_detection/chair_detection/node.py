"""Headless, latest-frame chair detection. No images or detections are saved."""

import math
import threading
import time

from cv_bridge import CvBridge
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


def load_model(path):
    # Lazy imports let ROS contract tests run without downloading weights or CUDA.
    import torch
    from ultralytics import YOLO

    if not torch.cuda.is_available():
        raise RuntimeError('CUDA is unavailable. Install JetPack-compatible CUDA PyTorch.')
    model = YOLO(path)
    names = model.names
    if len(names) <= 56 or names[56] != 'chair':
        raise ValueError('Model must use COCO class 56 = chair (use pretrained yolov8n.pt).')
    model.to('cuda:0')
    return model


class ChairDetection(Node):
    def __init__(self, **kwargs):
        super().__init__('chair_detection', **kwargs)
        try:
            descriptor = ParameterDescriptor(read_only=True)
            self.declare_parameter('model_path', 'yolov8n.pt', descriptor)
            self.declare_parameter(
                'image_topic', '/zed_front/zed_node/rgb/color/rect/image', descriptor)
            self.declare_parameter('confidence_threshold', 0.5, descriptor)
            self.declare_parameter('inference_fps', 10.0, descriptor)
            model_path = self.get_parameter('model_path').value
            image_topic = self.get_parameter('image_topic').value
            self.confidence = self.get_parameter('confidence_threshold').value
            fps = self.get_parameter('inference_fps').value
            if not model_path.strip() or not image_topic.strip():
                raise ValueError('model_path and image_topic must be nonempty.')
            if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
                raise ValueError('confidence_threshold must be finite and between 0 and 1.')
            if not math.isfinite(fps) or fps <= 0.0:
                raise ValueError('inference_fps must be finite and greater than zero.')

            self.model = load_model(model_path)
            self.bridge = CvBridge()
            self.lock = threading.Lock()
            self.latest = None
            self.count = 0
            self.completed = 0
            self.report_time = time.monotonic()
            self.publisher = self.create_publisher(Int32, '/vision/chair_count', 1)
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.subscription = self.create_subscription(Image, image_topic, self.receive, qos)
            # A separate group keeps receiving images while the GPU is busy, but
            # prevents overlapping inference calls on the same model.
            self.inference_group = MutuallyExclusiveCallbackGroup()
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.inference_timer = self.create_timer(
                1.0 / fps, self.infer, callback_group=self.inference_group,
                clock=self.wall_clock)
            self.report_timer = self.create_timer(1.0, self.report, clock=self.wall_clock)
            self.get_logger().info(f'CUDA chair detection subscribed to {image_topic}')
        except Exception:
            self.destroy_node()
            raise

    def receive(self, message):
        with self.lock:
            self.latest = message

    def infer(self):
        with self.lock:
            message, self.latest = self.latest, None
        if message is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            result = self.model.predict(
                source=frame, device=0, classes=[56], conf=self.confidence,
                verbose=False, save=False, save_txt=False, save_crop=False,
                save_conf=False, show=False, visualize=False)[0]
            count = len(result.boxes) if result.boxes is not None else 0
        except Exception as error:
            self.get_logger().error(f'Chair inference failed: {error}', throttle_duration_sec=5.0)
            return  # Failure is not evidence of zero chairs.
        self.publisher.publish(Int32(data=count))
        with self.lock:
            self.count = count
            self.completed += 1

    def report(self):
        now = time.monotonic()
        with self.lock:
            count, completed = self.count, self.completed
            self.completed = 0
        elapsed = now - self.report_time
        self.report_time = now
        if completed:
            # Console only: do not persist detection counts in ROS log files.
            print(f'Chairs detected: {count} | FPS: {completed / elapsed:.1f}', flush=True)
        else:
            self.get_logger().warning(
                'No completed image inference in the last second; chair count is not fresh.',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = ChairDetection()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
