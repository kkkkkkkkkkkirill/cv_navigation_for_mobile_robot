#!/usr/bin/env python3
"""
Continuous map -> odom TF publisher.

The default tf2_ros static_transform_publisher uses TRANSIENT_LOCAL QoS and
publishes once at startup. In Gazebo+RViz with many late-arriving subscribers
the latched message is sometimes missed, RViz's TF buffer ends up without the
map -> odom chain, and any cloud whose frame_id is below `odom` (lidar,
cameras, ...) renders incorrectly because the lookup falls back to identity
and the cloud appears to rotate with the robot.

This node mirrors the static transform but publishes on the regular /tf
topic at 30 Hz, which is robust to subscriber arrival order and bridges any
DDS hiccups.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class MapToOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__('map_to_odom_publisher')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'odom')
        self.declare_parameter('rate_hz', 30.0)

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        yaw = float(self.get_parameter('yaw').value)
        self.parent = str(self.get_parameter('parent_frame').value)
        self.child = str(self.get_parameter('child_frame').value)
        rate = float(self.get_parameter('rate_hz').value)

        self.qz = math.sin(yaw / 2.0)
        self.qw = math.cos(yaw / 2.0)

        self.br = TransformBroadcaster(self)
        self.create_timer(1.0 / max(rate, 1.0), self._publish)

        self.get_logger().info(
            f'map_to_odom_publisher: {self.parent}->{self.child} '
            f'xyz=({self.x:.2f},{self.y:.2f},{self.z:.2f}) yaw={yaw:.2f} '
            f'rate={rate:.0f}Hz'
        )

    def _publish(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation.z = self.qz
        t.transform.rotation.w = self.qw
        self.br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = MapToOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
