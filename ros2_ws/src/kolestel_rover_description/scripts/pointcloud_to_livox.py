#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from std_msgs.msg import Header

class PointCloudToLivox(Node):
    def __init__(self):
        super().__init__('pointcloud_to_livox')
        self.declare_parameter('input_topic', '/kolestel_rover/scan')
        self.declare_parameter('output_topic', '/livox/lidar')
        self.declare_parameter('scan_rate', 10.0)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.scan_rate = self.get_parameter('scan_rate').value
        self.scan_period_ns = int(1e9 / self.scan_rate)
        self.pub = self.create_publisher(CustomMsg, output_topic, 10)
        self.sub = self.create_subscription(PointCloud2, input_topic, self.callback, 10)
        self.get_logger().info(f'Конвертер запущен: {input_topic} -> {output_topic}')

    def callback(self, msg: PointCloud2):
        field_offsets = {f.name: f.offset for f in msg.fields}
        has_intensity = 'intensity' in field_offsets
        timebase_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        point_step = msg.point_step
        data = msg.data
        num_points = msg.width * msg.height
        custom_points = []
        for i in range(num_points):
            off = i * point_step
            x = struct.unpack_from('f', data, off + field_offsets['x'])[0]
            y = struct.unpack_from('f', data, off + field_offsets['y'])[0]
            z = struct.unpack_from('f', data, off + field_offsets['z'])[0]
            if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                continue
            offset_time_ns = int(i * self.scan_period_ns / max(num_points - 1, 1))
            reflectivity = 0
            if has_intensity:
                intensity = struct.unpack_from('f', data, off + field_offsets['intensity'])[0]
                reflectivity = min(255, max(0, int(intensity)))
            pt = CustomPoint()
            pt.offset_time = offset_time_ns
            pt.x = x
            pt.y = y
            pt.z = z
            pt.reflectivity = reflectivity
            pt.tag = 0
            pt.line = 0
            custom_points.append(pt)
        if not custom_points:
            return
        out = CustomMsg()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.timebase = timebase_ns
        out.point_num = len(custom_points)
        out.lidar_id = 1
        out.points = custom_points
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLivox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
