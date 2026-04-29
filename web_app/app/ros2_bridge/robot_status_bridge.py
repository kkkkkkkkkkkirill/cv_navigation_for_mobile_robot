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
from rclpy.node import Node

urllib.request.install_opener(
    urllib.request.build_opener(urllib.request.ProxyHandler({}))
)

BACKEND_URL = os.environ.get('AMR_BACKEND_URL', 'http://127.0.0.1:8010')
ROBOT_NAME = os.environ.get('AMR_ROBOT_NAME', 'amr-1')
MAP_ORIGIN_X = float(os.environ.get('AMR_MAP_ORIGIN_X', '10.5'))
MAP_ORIGIN_Y = float(os.environ.get('AMR_MAP_ORIGIN_Y', '-8.0'))


class RobotStatusBridge(Node):
    def __init__(self) -> None:
        super().__init__('robot_status_bridge')
        self._last_odom: Optional[Odometry] = None
        self._ok_count = 0
        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.create_timer(1.0, self._flush)
        self.get_logger().info(
            f'posting robot status to {BACKEND_URL}/robot/status using odom + map origin ({MAP_ORIGIN_X}, {MAP_ORIGIN_Y})'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _flush(self) -> None:
        if self._last_odom is None:
            self.get_logger().warn('waiting for /odom ...')
            return

        pose = self._last_odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        payload = {
            'name': ROBOT_NAME,
            'pos_x': float(MAP_ORIGIN_X + pose.position.x),
            'pos_y': float(MAP_ORIGIN_Y + pose.position.y),
            'heading_rad': float(yaw),
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
