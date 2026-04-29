#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('cmd_vel_topic', '/line_follower/cmd_vel')
        self.declare_parameter('linear_speed', 0.4)
        self.declare_parameter('kp', 0.006)
        self.declare_parameter('scan_y1', 380)
        self.declare_parameter('scan_y2', 430)
        self.declare_parameter('intersection_threshold', 250)
        self.declare_parameter('line_lost_threshold', 25)

        image_topic = self.get_parameter('image_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.kp = float(self.get_parameter('kp').value)
        self.scan_y1 = int(self.get_parameter('scan_y1').value)
        self.scan_y2 = int(self.get_parameter('scan_y2').value)
        self.intersection_thresh = int(self.get_parameter('intersection_threshold').value)
        self.line_lost_thresh = int(self.get_parameter('line_lost_threshold').value)

        self.enabled = False
        self.width = 640
        self.center_x = 320

        self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.create_subscription(Bool, '/line_follower/enable', self.enable_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.error_pub = self.create_publisher(Float64, '/line_follower/error', 10)
        self.active_pub = self.create_publisher(Bool, '/line_follower/active', 10)
        self.intersection_pub = self.create_publisher(Bool, '/line_follower/intersection', 10)
        self.debug_pub = self.create_publisher(Image, '/line_follower/debug_image', 10)

        self.get_logger().info('line follower ready')

    def enable_cb(self, msg):
        was_enabled = self.enabled
        self.enabled = msg.data
        if was_enabled and not self.enabled:
            self.cmd_pub.publish(Twist())
        self.get_logger().info(f'line follower {"enabled" if self.enabled else "disabled"}')

    def image_cb(self, msg):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        except ValueError:
            return

        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
        else:
            frame = buf.copy()

        self.width = msg.width
        self.center_x = self.width // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 80, 80])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        scan_y1 = min(self.scan_y1, msg.height - 1)
        scan_y2 = min(self.scan_y2, msg.height - 1)

        scan1 = mask[scan_y1, :]
        scan2 = mask[scan_y2, :]

        pixels1 = np.where(scan1 > 0)[0]
        pixels2 = np.where(scan2 > 0)[0]

        line_detected = len(pixels1) > self.line_lost_thresh or len(pixels2) > self.line_lost_thresh
        line_center = self.center_x
        error = 0
        is_intersection = False

        if line_detected:
            total = len(pixels1) + len(pixels2)
            if total > self.intersection_thresh:
                is_intersection = True

            cx1 = int(np.mean(pixels1)) if len(pixels1) > 0 else self.center_x
            cx2 = int(np.mean(pixels2)) if len(pixels2) > 0 else self.center_x
            line_center = (cx1 + cx2) // 2
            error = line_center - self.center_x

        self.active_pub.publish(Bool(data=line_detected))
        self.intersection_pub.publish(Bool(data=is_intersection))
        err_msg = Float64()
        err_msg.data = float(error)
        self.error_pub.publish(err_msg)

        if self.enabled and line_detected:
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = -self.kp * error
            self.cmd_pub.publish(twist)
        elif self.enabled and not line_detected:
            self.cmd_pub.publish(Twist())

        debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cv2.line(debug, (0, scan_y1), (self.width, scan_y1), (0, 0, 255), 2)
        cv2.line(debug, (0, scan_y2), (self.width, scan_y2), (0, 0, 255), 2)
        cv2.line(debug, (self.center_x, 0), (self.center_x, msg.height), (255, 0, 0), 1)

        if line_detected:
            cv2.circle(debug, (line_center, (scan_y1 + scan_y2) // 2), 10, (0, 255, 0), -1)
            cv2.line(debug, (line_center, scan_y1), (line_center, scan_y2), (0, 255, 0), 2)
            status = "INTERSECTION" if is_intersection else "TRACKING"
            color = (0, 255, 255) if is_intersection else (0, 255, 0)
        else:
            status = "LINE LOST"
            color = (0, 0, 255)

        cv2.putText(debug, f"{status} err:{error}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(debug, "ENABLED" if self.enabled else "DISABLED", (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0) if self.enabled else (128, 128, 128), 2)

        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height = debug.shape[0]
        debug_msg.width = debug.shape[1]
        debug_msg.encoding = 'bgr8'
        debug_msg.step = debug.shape[1] * 3
        debug_msg.data = debug.tobytes()
        self.debug_pub.publish(debug_msg)


def main():
    rclpy.init()
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
