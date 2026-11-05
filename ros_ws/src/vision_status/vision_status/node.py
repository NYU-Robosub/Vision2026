import time
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from nav_msgs.msg import Odometry

from .monitor import StreamMonitor


class VisionStatus(Node):
    def __init__(self):
        super().__init__('vision_status')
        defaults = {
            'rgb': ('/zed_front/zed_node/rgb/color/rect/image', Image),
            'depth': ('/zed_front/zed_node/depth/depth_registered', Image),
            'camera_info': ('/zed_front/zed_node/rgb/color/rect/camera_info', CameraInfo),
            'odometry': ('/zed_front/zed_node/odom', Odometry),
            'cloud_map': ('/rtabmap/cloud_map', PointCloud2),
        }
        self.declare_parameter('timeout_sec', 3.0)
        self.declare_parameter('cloud_timeout_sec', 15.0)
        self.declare_parameter('report_interval_sec', 2.0)
        self.declare_parameter('rate_window_sec', 10.0)
        self.declare_parameter('startup_grace_sec', 10.0)
        self.start_time = time.monotonic()
        self.monitors = {}
        self.last_states = {}
        self.subscriptions_ = []
        timeout = float(self.get_parameter('timeout_sec').value)
        cloud_timeout = float(self.get_parameter('cloud_timeout_sec').value)
        window = float(self.get_parameter('rate_window_sec').value)
        self.grace = float(self.get_parameter('startup_grace_sec').value)
        if min(timeout, cloud_timeout, window, self.grace) < 0 or timeout == 0 or cloud_timeout == 0 or window == 0:
            raise ValueError('Timeouts and window must be positive; grace must be nonnegative')
        for name, (default_topic, msg_type) in defaults.items():
            self.declare_parameter(name + '_topic', default_topic)
            topic = self.get_parameter(name + '_topic').value
            self.monitors[name] = StreamMonitor(cloud_timeout if name == 'cloud_map' else timeout, window)
            self.last_states[name] = 'WAITING'
            # Sensor-data QoS works with typical best-effort ZED streams.
            qos = qos_profile_sensor_data
            self.subscriptions_.append(self.create_subscription(msg_type, topic, partial(self.on_message, name), qos))
            self.get_logger().info(f'Monitoring {name}: {topic}')
        interval = float(self.get_parameter('report_interval_sec').value)
        if interval <= 0:
            raise ValueError('report_interval_sec must be positive')
        self.timer = self.create_timer(interval, self.report)

    def on_message(self, name, _msg):
        self.monitors[name].record(time.monotonic())

    def report(self):
        now = time.monotonic()
        lines = ['Vision pipeline status:']
        for name, monitor in self.monitors.items():
            state = monitor.status(now)
            age = monitor.age(now)
            lines.append(f'  {name:12} {state:8} {monitor.rate(now):6.2f} Hz  age={"never" if age is None else f"{age:.1f}s"}')
            previous = self.last_states[name]
            if state == 'NO DATA' and previous != 'NO DATA':
                self.get_logger().warning(f'{name}: data timed out')
            elif state == 'OK' and previous == 'NO DATA':
                self.get_logger().info(f'{name}: data recovered')
            elif state == 'OK' and previous == 'WAITING':
                self.get_logger().info(f'{name}: first data received')
            elif state == 'WAITING' and now - self.start_time >= self.grace and previous == 'WAITING':
                # Warn once, but preserve WAITING until first message.
                self.get_logger().warning(f'{name}: still waiting for first message')
                self.last_states[name] = 'WAITING_WARNED'
            if state != 'WAITING' or previous != 'WAITING_WARNED':
                self.last_states[name] = state
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionStatus()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
