#!/usr/bin/env python3
"""
Lidar cloud republisher in the map frame.

Why: when subscribers buffer many TF entries with mixed stamps + rapid robot
rotation, RViz can show the lidar cloud as if rotating with the robot. The
fix is to transform the cloud to the static `map` frame ONCE on publication —
RViz then renders points that already are in map coordinates and never needs
a TF lookup for them.

Subscribes:
  /lidar          sensor_msgs/PointCloud2  (frame: lidar body frame)

Publishes:
  /lidar_map      sensor_msgs/PointCloud2  (frame: map; same point count, transformed)
"""
from __future__ import annotations

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
        return np.eye(3, dtype=np.float32)
    s = 2.0 / n
    return np.array([
        [1 - s * (qy * qy + qz * qz),  s * (qx * qy - qz * qw),      s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw),      1 - s * (qx * qx + qz * qz),  s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw),      s * (qy * qz + qx * qw),      1 - s * (qx * qx + qy * qy)],
    ], dtype=np.float32)


class LidarMapRepublisher(Node):
    def __init__(self) -> None:
        super().__init__('lidar_map_republisher')
        self.declare_parameter('input_topic', '/lidar')
        self.declare_parameter('output_topic', '/lidar_map')
        self.declare_parameter('target_frame', 'map')

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
            f'lidar_map_republisher ready: {in_topic} -> {out_topic} (frame={self.target_frame})'
        )

    def _lookup_tf(self, source_frame: str, stamp: Time) -> Optional[TransformStamped]:
        # Try the source-stamped lookup first (best alignment with the cloud capture
        # time), fall back to the latest available transform if the buffer is short.
        try:
            return self._tf_buffer.lookup_transform(
                self.target_frame, source_frame, stamp,
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            try:
                return self._tf_buffer.lookup_transform(
                    self.target_frame, source_frame, Time(),
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
        ts = self._lookup_tf(msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if ts is None:
            return

        q = ts.transform.rotation
        R = _quat_to_mat(q.x, q.y, q.z, q.w)
        t = np.array([ts.transform.translation.x,
                      ts.transform.translation.y,
                      ts.transform.translation.z], dtype=np.float32)

        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return
        pts = pts.astype(np.float32, copy=False).reshape(-1, 3)

        finite_mask = np.isfinite(pts).all(axis=1)
        pts = pts[finite_mask]
        if pts.size == 0:
            return

        transformed = pts @ R.T + t

        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame
        out.height = 1
        out.width = transformed.shape[0]
        out.is_bigendian = False
        out.is_dense = True
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
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
    node = LidarMapRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
