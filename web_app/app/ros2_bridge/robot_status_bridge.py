#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import urllib.error
import urllib.request
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

urllib.request.install_opener(
    urllib.request.build_opener(urllib.request.ProxyHandler({}))
)

BACKEND_URL = os.environ.get('AMR_BACKEND_URL', 'http://127.0.0.1:8010')
ROBOT_NAME = os.environ.get('AMR_ROBOT_NAME', 'amr-1')

# Fallback if TF map->base_footprint is not yet available:
# add this offset to the raw /odom pose. Match the static map->odom shift
# from gazebo.launch.py for backwards compatibility.
MAP_ORIGIN_X = float(os.environ.get('AMR_MAP_ORIGIN_X', '10.5'))
MAP_ORIGIN_Y = float(os.environ.get('AMR_MAP_ORIGIN_Y', '-8.0'))

MAP_FRAME = os.environ.get('AMR_MAP_FRAME', 'map')
BASE_FRAME = os.environ.get('AMR_BASE_FRAME', 'base_footprint')


class RobotStatusBridge(Node):
    def __init__(self) -> None:
        super().__init__('robot_status_bridge')
        self._last_odom: Optional[Odometry] = None
        self._ok_count = 0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_warned = False
        self._used_tf_once = False
        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.create_timer(1.0, self._flush)
        self.get_logger().info(
            f'posting robot status to {BACKEND_URL}/robot/status. '
            f'prefer TF {MAP_FRAME}->{BASE_FRAME}; fallback: /odom + '
            f'origin ({MAP_ORIGIN_X}, {MAP_ORIGIN_Y})'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _pose_from_tf(self) -> Optional[tuple[float, float, float]]:
        try:
            ts = self._tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            if not self._tf_warned:
                self.get_logger().warn(
                    f'TF {MAP_FRAME}->{BASE_FRAME} not ready, falling back to /odom + offset ({exc})'
                )
                self._tf_warned = True
            return None
        q = ts.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        if not self._used_tf_once:
            self._used_tf_once = True
            self.get_logger().info(
                f'using TF {MAP_FRAME}->{BASE_FRAME} for robot pose '
                f'(x={ts.transform.translation.x:.2f}, y={ts.transform.translation.y:.2f}, '
                f'yaw={yaw:.2f})'
            )
        return (
            float(ts.transform.translation.x),
            float(ts.transform.translation.y),
            float(yaw),
        )

    def _pose_from_odom(self) -> Optional[tuple[float, float, float]]:
        if self._last_odom is None:
            return None
        pose = self._last_odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (
            MAP_ORIGIN_X + float(pose.position.x),
            MAP_ORIGIN_Y + float(pose.position.y),
            float(yaw),
        )

    def _flush(self) -> None:
        pose = self._pose_from_tf()
        if pose is None:
            pose = self._pose_from_odom()
        if pose is None:
            self.get_logger().warn('waiting for /odom or TF ...')
            return
        x, y, yaw = pose

        payload = {
            'name': ROBOT_NAME,
            'pos_x': x,
            'pos_y': y,
            'heading_rad': yaw,
            'battery_pct': 100.0,
            'motion_mode': os.environ.get('AMR_MOTION_MODE', 'odom_tracking'),
        }

        req = urllib.request.Request(
            BACKEND_URL.rstrip('/') + '/robot/status',
            data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )

        try:
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                resp.read()
            self._ok_count += 1
            if self._ok_count == 1 or self._ok_count % 10 == 0:
                self.get_logger().info(
                    f'status posted ok x={payload["pos_x"]:.2f} y={payload["pos_y"]:.2f} yaw={payload["heading_rad"]:.2f}'
                )
        except urllib.error.HTTPError as exc:
            body = exc.read().decode('utf-8', errors='ignore')
            self.get_logger().error(
                f'backend post failed: HTTP {exc.code}; body={body}; payload={payload}'
            )
        except Exception as exc:
            self.get_logger().error(
                f'backend post failed: {exc}; payload={payload}'
            )


def main() -> None:
    rclpy.init()
    node = RobotStatusBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
