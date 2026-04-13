#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


@dataclass(frozen=True)
class RigidTransform:
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)


def _normalize_quaternion(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def _quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        (lw * rx) + (lx * rw) + (ly * rz) - (lz * ry),
        (lw * ry) - (lx * rz) + (ly * rw) + (lz * rx),
        (lw * rz) + (lx * ry) - (ly * rx) + (lz * rw),
        (lw * rw) - (lx * rx) - (ly * ry) - (lz * rz),
    )


def _quaternion_conjugate(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def _rotate_vector(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    pure = (vector[0], vector[1], vector[2], 0.0)
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, pure),
        _quaternion_conjugate(quaternion),
    )
    return (rotated[0], rotated[1], rotated[2])


def _compose(left: RigidTransform, right: RigidTransform) -> RigidTransform:
    rotated = _rotate_vector(left.rotation, right.translation)
    return RigidTransform(
        (
            left.translation[0] + rotated[0],
            left.translation[1] + rotated[1],
            left.translation[2] + rotated[2],
        ),
        _normalize_quaternion(_quaternion_multiply(left.rotation, right.rotation)),
    )


def _inverse(transform: RigidTransform) -> RigidTransform:
    rotation = _quaternion_conjugate(_normalize_quaternion(transform.rotation))
    translation = _rotate_vector(
        rotation,
        (
            -transform.translation[0],
            -transform.translation[1],
            -transform.translation[2],
        ),
    )
    return RigidTransform(translation, rotation)


def _transform_from_pose(pose) -> RigidTransform:
    return RigidTransform(
        (pose.position.x, pose.position.y, pose.position.z),
        _normalize_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        ),
    )


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return _normalize_quaternion(
        (
            (sr * cp * cy) - (cr * sp * sy),
            (cr * sp * cy) + (sr * cp * sy),
            (cr * cp * sy) - (sr * sp * cy),
            (cr * cp * cy) + (sr * sp * sy),
        )
    )


def _dot(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> float:
    return sum(a * b for a, b in zip(left, right))


def _slerp(
    start: tuple[float, float, float, float],
    end: tuple[float, float, float, float],
    alpha: float,
) -> tuple[float, float, float, float]:
    dot = _dot(start, end)
    if dot < 0.0:
        end = (-end[0], -end[1], -end[2], -end[3])
        dot = -dot

    if dot > 0.9995:
        return _normalize_quaternion(
            tuple(((1.0 - alpha) * a) + (alpha * b) for a, b in zip(start, end))
        )

    theta_0 = math.acos(max(min(dot, 1.0), -1.0))
    theta = theta_0 * alpha
    sin_theta_0 = math.sin(theta_0)
    scale_start = math.sin(theta_0 - theta) / sin_theta_0
    scale_end = math.sin(theta) / sin_theta_0

    return _normalize_quaternion(
        tuple((scale_start * a) + (scale_end * b) for a, b in zip(start, end))
    )


def _low_pass(previous: RigidTransform, current: RigidTransform, alpha: float) -> RigidTransform:
    return RigidTransform(
        tuple(((1.0 - alpha) * a) + (alpha * b) for a, b in zip(previous.translation, current.translation)),
        _slerp(previous.rotation, current.rotation, alpha),
    )


def _float_list(node: Node, name: str) -> list[float]:
    values = list(node.get_parameter(name).value)
    if len(values) != 3:
        raise ValueError(f'Parameter "{name}" must contain exactly three values.')
    return [float(value) for value in values]


class FrontPoseNode(Node):
    """Promote front-camera motion into the robot pose and TF chain."""

    def __init__(self) -> None:
        super().__init__('front_pose_node')
        self._declare_parameters()
        self._load_parameters()
        self._create_interfaces()

        self.last_map_to_base: Optional[RigidTransform] = None
        self.last_map_stamp = None
        self.last_odom_to_base: Optional[RigidTransform] = None
        self.last_odom_stamp = None
        self.last_filtered_pose: Optional[RigidTransform] = None
        self.warned_sync_skew = False

        self.create_subscription(PoseStamped, self.input_pose_topic, self._handle_pose, 10)
        self.create_subscription(Odometry, self.input_odom_topic, self._handle_odom, 10)

        self._publish_startup_tf()
        self.get_logger().info(
            (
                f'Front pose node active on "{self.input_pose_topic}" and "{self.input_odom_topic}". '
                f'Publishing {self.map_frame} -> {self.odom_frame} -> {self.base_frame} -> {self.camera_frame}.'
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameters(
            '',
            [
                ('input_pose_topic', '/front/zed_node/pose'),
                ('input_odom_topic', '/front/zed_node/odom'),
                ('output_pose_topic', '/localization/front/pose'),
                ('output_odom_topic', '/localization/front/odom'),
                ('pose_source', 'pose'),
                ('publish_pose', True),
                ('publish_odom', True),
                ('publish_tf', True),
                ('publish_map_to_odom', True),
                ('publish_odom_to_base', True),
                ('publish_camera_mount_tf', True),
                ('publish_filtered_pose', False),
                ('filtered_pose_topic', '/localization/front/pose_filtered'),
                ('map_frame', 'map'),
                ('odom_frame', 'odom'),
                ('base_frame', 'base_link'),
                ('camera_frame', 'front_camera_link'),
                ('base_to_camera_xyz', [0.0, 0.0, 0.0]),
                ('base_to_camera_rpy', [0.0, 0.0, 0.0]),
                ('max_sync_dt_sec', 0.2),
                ('publish_identity_map_to_odom_on_startup', True),
                ('pose_filter_alpha', 0.2),
            ],
        )

    def _load_parameters(self) -> None:
        self.input_pose_topic = str(self.get_parameter('input_pose_topic').value)
        self.input_odom_topic = str(self.get_parameter('input_odom_topic').value)
        self.output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        self.output_odom_topic = str(self.get_parameter('output_odom_topic').value)
        self.filtered_pose_topic = str(self.get_parameter('filtered_pose_topic').value)

        self.pose_source = str(self.get_parameter('pose_source').value).strip().lower()
        if self.pose_source not in ('pose', 'odom'):
            raise ValueError('Parameter "pose_source" must be "pose" or "odom".')

        self.publish_pose = bool(self.get_parameter('publish_pose').value)
        self.publish_odom = bool(self.get_parameter('publish_odom').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_map_to_odom = bool(self.get_parameter('publish_map_to_odom').value)
        self.publish_odom_to_base = bool(self.get_parameter('publish_odom_to_base').value)
        self.publish_camera_mount_tf = bool(self.get_parameter('publish_camera_mount_tf').value)
        self.publish_filtered_pose = bool(self.get_parameter('publish_filtered_pose').value)
        self.publish_identity_map_to_odom_on_startup = bool(
            self.get_parameter('publish_identity_map_to_odom_on_startup').value
        )

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)

        self.max_sync_dt_sec = float(self.get_parameter('max_sync_dt_sec').value)
        self.pose_filter_alpha = float(self.get_parameter('pose_filter_alpha').value)
        if not 0.0 < self.pose_filter_alpha <= 1.0:
            raise ValueError('Parameter "pose_filter_alpha" must be in (0.0, 1.0].')

        xyz = _float_list(self, 'base_to_camera_xyz')
        rpy = _float_list(self, 'base_to_camera_rpy')
        self.base_to_camera = RigidTransform(tuple(xyz), _quaternion_from_rpy(*rpy))
        self.camera_to_base = _inverse(self.base_to_camera)

    def _create_interfaces(self) -> None:
        self.pose_publisher = (
            self.create_publisher(PoseStamped, self.output_pose_topic, 10)
            if self.publish_pose
            else None
        )
        self.odom_publisher = (
            self.create_publisher(Odometry, self.output_odom_topic, 10)
            if self.publish_odom
            else None
        )
        self.filtered_pose_publisher = (
            self.create_publisher(PoseStamped, self.filtered_pose_topic, 10)
            if self.publish_filtered_pose
            else None
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.static_tf_broadcaster = (
            StaticTransformBroadcaster(self) if self.publish_camera_mount_tf else None
        )

    def _publish_startup_tf(self) -> None:
        if self.static_tf_broadcaster is not None:
            self.static_tf_broadcaster.sendTransform(
                self._transform_msg(
                    parent=self.base_frame,
                    child=self.camera_frame,
                    transform=self.base_to_camera,
                    stamp=self.get_clock().now().to_msg(),
                )
            )

        if (
            self.tf_broadcaster is not None
            and self.publish_map_to_odom
            and self.publish_identity_map_to_odom_on_startup
        ):
            self.tf_broadcaster.sendTransform(
                self._transform_msg(
                    parent=self.map_frame,
                    child=self.odom_frame,
                    transform=RigidTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                    stamp=self.get_clock().now().to_msg(),
                )
            )

    def _pose_msg(self, frame_id: str, transform: RigidTransform, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = transform.translation[0]
        msg.pose.position.y = transform.translation[1]
        msg.pose.position.z = transform.translation[2]
        msg.pose.orientation.x = transform.rotation[0]
        msg.pose.orientation.y = transform.rotation[1]
        msg.pose.orientation.z = transform.rotation[2]
        msg.pose.orientation.w = transform.rotation[3]
        return msg

    def _transform_msg(self, parent: str, child: str, transform: RigidTransform, stamp) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = transform.translation[0]
        msg.transform.translation.y = transform.translation[1]
        msg.transform.translation.z = transform.translation[2]
        msg.transform.rotation.x = transform.rotation[0]
        msg.transform.rotation.y = transform.rotation[1]
        msg.transform.rotation.z = transform.rotation[2]
        msg.transform.rotation.w = transform.rotation[3]
        return msg

    def _odom_msg(self, source: Odometry, transform: RigidTransform) -> Odometry:
        msg = Odometry()
        msg.header.stamp = source.header.stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = transform.translation[0]
        msg.pose.pose.position.y = transform.translation[1]
        msg.pose.pose.position.z = transform.translation[2]
        msg.pose.pose.orientation.x = transform.rotation[0]
        msg.pose.pose.orientation.y = transform.rotation[1]
        msg.pose.pose.orientation.z = transform.rotation[2]
        msg.pose.pose.orientation.w = transform.rotation[3]
        msg.pose.covariance = source.pose.covariance
        msg.twist = source.twist
        return msg

    def _handle_pose(self, msg: PoseStamped) -> None:
        self.last_map_to_base = _compose(_transform_from_pose(msg.pose), self.camera_to_base)
        self.last_map_stamp = msg.header.stamp

        if self.pose_publisher is not None and self.pose_source == 'pose':
            self.pose_publisher.publish(self._pose_msg(self.map_frame, self.last_map_to_base, msg.header.stamp))

        self._publish_filtered_pose(msg.header.stamp)
        self._publish_map_to_odom(msg.header.stamp)

    def _handle_odom(self, msg: Odometry) -> None:
        self.last_odom_to_base = _compose(_transform_from_pose(msg.pose.pose), self.camera_to_base)
        self.last_odom_stamp = msg.header.stamp

        if self.odom_publisher is not None:
            self.odom_publisher.publish(self._odom_msg(msg, self.last_odom_to_base))

        if self.pose_publisher is not None and self.pose_source == 'odom':
            self.pose_publisher.publish(self._pose_msg(self.odom_frame, self.last_odom_to_base, msg.header.stamp))

        if self.tf_broadcaster is not None and self.publish_odom_to_base:
            self.tf_broadcaster.sendTransform(
                self._transform_msg(self.odom_frame, self.base_frame, self.last_odom_to_base, msg.header.stamp)
            )

        self._publish_map_to_odom(msg.header.stamp)

    def _publish_filtered_pose(self, stamp) -> None:
        if self.filtered_pose_publisher is None or self.last_map_to_base is None:
            return

        self.last_filtered_pose = (
            self.last_map_to_base
            if self.last_filtered_pose is None
            else _low_pass(self.last_filtered_pose, self.last_map_to_base, self.pose_filter_alpha)
        )
        self.filtered_pose_publisher.publish(
            self._pose_msg(self.map_frame, self.last_filtered_pose, stamp)
        )

    def _publish_map_to_odom(self, stamp) -> None:
        if self.tf_broadcaster is None or not self.publish_map_to_odom:
            return
        if self.last_map_to_base is None or self.last_odom_to_base is None:
            return

        skew = abs(_stamp_seconds(self.last_map_stamp) - _stamp_seconds(self.last_odom_stamp))
        if skew > self.max_sync_dt_sec:
            if not self.warned_sync_skew:
                self.get_logger().warning(
                    (
                        f'Pose and odom differ by {skew:.3f}s; '
                        f'skipping map -> odom while max_sync_dt_sec={self.max_sync_dt_sec:.3f}.'
                    )
                )
                self.warned_sync_skew = True
            return

        self.warned_sync_skew = False
        map_to_odom = _compose(self.last_map_to_base, _inverse(self.last_odom_to_base))
        self.tf_broadcaster.sendTransform(
            self._transform_msg(self.map_frame, self.odom_frame, map_to_odom, stamp)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontPoseNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
