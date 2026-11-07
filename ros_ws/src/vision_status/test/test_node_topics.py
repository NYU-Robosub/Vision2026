"""Check actual subscriptions against the project's camera topic contract."""

import rclpy

from vision_status.node import VisionStatus


def test_default_subscriptions_match_front_camera():
    rclpy.init()
    node = None
    try:
        node = VisionStatus()
        assert {subscription.topic_name for subscription in node.subscriptions_} == {
            '/zed_front/zed_node/rgb/color/rect/image',
            '/zed_front/zed_node/depth/depth_registered',
            '/zed_front/zed_node/rgb/color/rect/camera_info',
            '/zed_front/zed_node/odom',
            '/rtabmap/cloud_map',
        }
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
