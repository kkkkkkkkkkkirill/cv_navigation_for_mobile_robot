#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class WarehouseMapVisualizer(Node):
    def __init__(self):
        super().__init__('warehouse_map_visualizer')
        self.publisher = self.create_publisher(MarkerArray, '/warehouse_map', 10)
        self.timer = self.create_timer(2.0, self.publish_map)

        pkg = get_package_share_directory('kolestel_rover_description')
        env_map_file = os.environ.get('AMR_WAREHOUSE_MAP_PATH')
        if env_map_file and os.path.isfile(env_map_file):
            map_file = env_map_file
        else:
            map_file = os.path.join(pkg, 'config', 'warehouse_map.yaml')
            if env_map_file:
                self.get_logger().warn(
                    f'AMR_WAREHOUSE_MAP_PATH does not exist: {env_map_file}; '
                    f'using installed package map: {map_file}'
                )

        with open(map_file, encoding='utf-8') as f:
            self.config = yaml.safe_load(f)['warehouse']

        self.get_logger().info(
            f"loaded {len(self.config['stations'])} stations, "
            f"{len(self.config['nodes'])} nodes, {len(self.config['lanes'])} lanes "
            f"from {map_file}"
        )

    def publish_map(self):
        markers = MarkerArray()
        mid = 0

        for name, data in self.config['nodes'].items():
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'nodes'
            m.id = mid
            mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(data['x'])
            m.pose.position.y = float(data['y'])
            m.pose.position.z = 0.08
            m.scale.x = 0.22
            m.scale.y = 0.22
            m.scale.z = 0.22
            m.color.r = 0.15
            m.color.g = 0.85
            m.color.b = 0.25
            m.color.a = 0.8
            markers.markers.append(m)

        for name, data in self.config['stations'].items():
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'stations'
            m.id = mid
            mid += 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(data['x'])
            m.pose.position.y = float(data['y'])
            m.pose.position.z = 0.12
            m.scale.x = 0.45
            m.scale.y = 0.45
            m.scale.z = 0.24
            if name.lower() == 'depot':
                m.color.r = 1.0
                m.color.g = 0.85
                m.color.b = 0.1
            else:
                m.color.r = 0.95
                m.color.g = 0.2
                m.color.b = 0.2
            m.color.a = 0.9
            markers.markers.append(m)

            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = self.get_clock().now().to_msg()
            t.ns = 'labels'
            t.id = mid
            mid += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(data['x'])
            t.pose.position.y = float(data['y'])
            t.pose.position.z = 0.85
            t.scale.z = 0.35
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 1.0
            t.text = data.get('label', name)
            markers.markers.append(t)

        nodes = self.config['nodes']
        for n1, n2 in self.config['lanes']:
            if n1 not in nodes or n2 not in nodes:
                continue
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'lanes'
            m.id = mid
            mid += 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.06
            p1 = Point(x=float(nodes[n1]['x']), y=float(nodes[n1]['y']), z=0.04)
            p2 = Point(x=float(nodes[n2]['x']), y=float(nodes[n2]['y']), z=0.04)
            m.points = [p1, p2]
            m.color.r = 0.3
            m.color.g = 0.7
            m.color.b = 1.0
            m.color.a = 0.65
            markers.markers.append(m)

        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = WarehouseMapVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
