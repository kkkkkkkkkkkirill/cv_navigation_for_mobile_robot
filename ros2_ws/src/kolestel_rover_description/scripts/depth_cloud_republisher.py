#!/usr/bin/env python3
"""
Republish /camera/depth/points (camera optical frame) into base_footprint frame.

Why: in the optical frame X is lateral and Z is forward, which makes RViz's
AxisColor look counter-intuitive. After republishing to base_footprint
(X-forward, Y-left, Z-up), AxisColor X visualises depth/forward distance
exactly as a person familiar with robot body conventions expects.

Subscribes:
  /camera/depth/points          sensor_msgs/PointCloud2 (frame: camera_optical)

Publishes:
  /camera/depth/points_base     sensor_msgs/PointCloud2 (frame: base_footprint)
"""
from __future__ import annotations

import struct
from typing import Optional

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener


def _quat_to_mat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (qy * qy + qz * qz),  s * (qx * qy - qz * qw),      s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw),      1 - s * (qx * qx + qz * qz),  s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw),      s * (qy * qz + qx * qw),      1 - s * (qx * qx + qy * qy)],
    ], dtype=np.float32)


class DepthCloudRepublisher(Node):
    def __init__(self) -> None:
        super().__init__('depth_cloud_republisher')
        self.declare_parameter('input_topic', '/camera/depth/points')
        self.declare_parameter('output_topic', '/camera/depth/points_base')
        self.declare_parameter('target_frame', 'base_footprint')

        in_topic = str(self.get_parameter('input_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(PointCloud2, in_topic, self._cb, 5)
        self._pub = self.create_publisher(PointCloud2, out_topic, 5)

        self._tf_warned = False
        self._first_pub = False
        self.get_logger().info(
            f'depth_cloud_republisher ready: {in_topic} -> {out_topic} (frame={self.target_frame})'
        )

    def _lookup_tf(self, source_frame: str) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            if not self._tf_warned:
                self.get_logger().warn(
                    f'TF {self.target_frame} <- {source_frame} not ready: {exc}'
                )
                self._tf_warned = True
            return None

    def _cb(self, msg: PointCloud2) -> None:
        ts = self._lookup_tf(msg.header.frame_id)
        if ts is None:
            return

        q = ts.transform.rotation
        R = _quat_to_mat(q.x, q.y, q.z, q.w)
        tx = float(ts.transform.translation.x)
        ty = float(ts.transform.translation.y)
        tz = float(ts.transform.translation.z)

        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return
        pts = pts.astype(np.float32, copy=False).reshape(-1, 3)

        # belt-and-suspenders nan filter (skip_nans is unreliable on organized clouds)
        finite_mask = np.isfinite(pts).all(axis=1)
        pts = pts[finite_mask]
        if pts.size == 0:
            return

        # Transform: p_target = R * p_source + t
        transformed = pts @ R.T
        transformed[:, 0] += tx
        transformed[:, 1] += ty
        transformed[:, 2] += tz

        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame
        out.height = 1
        out.width = transformed.shape[0]
        out.is_bigendian = False
        out.is_dense = True
        out.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        out.point_step = 12
        out.row_step = out.point_step * out.width
        out.data = transformed.tobytes()

        self._pub.publish(out)

        if not self._first_pub:
            self._first_pub = True
            self.get_logger().info(
                f'first cloud republished: {transformed.shape[0]} pts in {self.target_frame}'
            )


def main() -> None:
    rclpy.init()
    node = DepthCloudRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
