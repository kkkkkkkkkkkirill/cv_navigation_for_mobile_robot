#!/usr/bin/env python3
import json
import math
import os
import threading
import urllib.error
import urllib.request
from collections import defaultdict

import yaml
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory


urllib.request.install_opener(
    urllib.request.build_opener(urllib.request.ProxyHandler({}))
)

BACKEND_URL = os.environ.get('AMR_BACKEND_URL', 'http://127.0.0.1:8010')

ARRIVAL_TOLERANCE = 0.8
YAW_TOLERANCE = 0.12
TURN_SPEED = 0.6
TURN_CREEP = 0.03


def _find_map():
    env = os.environ.get('AMR_WAREHOUSE_MAP_PATH')
    if env and os.path.isfile(env):
        return env
    try:
        pkg = get_package_share_directory('kolestel_rover_description')
        for cand in ('config/warehouse_map.yaml',):
            path = os.path.join(pkg, cand)
            if os.path.isfile(path):
                return path
    except Exception:
        pass
    home = os.path.expanduser('~')
    fallback = os.path.join(home, 'Desktop', 'amr_stage4_cv_nav', 'shared', 'warehouse_map.yaml')
    return fallback


MAP_PATH = _find_map()


class CVNavigator(Node):
    def __init__(self):
        super().__init__('cv_navigator')

        self.get_logger().info(f'loading map: {MAP_PATH}')
        with open(MAP_PATH) as f:
            wh = yaml.safe_load(f)['warehouse']

        self.nodes = wh['nodes']
        self.stations = wh['stations']
        self.lanes = wh['lanes']

        self.graph = defaultdict(list)
        for lane in self.lanes:
            a, b = lane[0], lane[1]
            self.graph[a].append(b)
            self.graph[b].append(a)

        self.spawn_x = float(self.stations['depot']['x'])
        self.spawn_y = float(self.stations['depot']['y'])
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.have_odom = False

        self.line_active = False
        self.last_line_cmd = Twist()

        self.cancel_requested = False
        self.active_goal_handle = None
        self.lock = threading.Lock()

        cb_group = ReentrantCallbackGroup()

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10, callback_group=cb_group)
        self.create_subscription(Bool, '/line_follower/active', self.line_active_cb, 10, callback_group=cb_group)
        self.create_subscription(Twist, '/line_follower/cmd_vel', self.line_cmd_cb, 10, callback_group=cb_group)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, '/line_follower/enable', 10)

        self.create_timer(0.05, self.forward_line_cmd, callback_group=cb_group)

        self.action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_navigate,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f'cv_navigator ready: {len(self.nodes)} nodes, '
            f'{len(self.stations)} stations, {len(self.lanes)} lanes'
        )
        self.get_logger().info(f'action server: /navigate_to_pose')

    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x + self.spawn_x
        self.odom_y = msg.pose.pose.position.y + self.spawn_y
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.have_odom = True

    def line_active_cb(self, msg):
        self.line_active = bool(msg.data)

    def line_cmd_cb(self, msg):
        self.last_line_cmd = msg

    def forward_line_cmd(self):
        pass

    def world_pos(self):
        return self.odom_x, self.odom_y

    def distance_between(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def distance_to_node(self, node_name):
        n = self.nodes[node_name]
        wx, wy = self.world_pos()
        return math.sqrt((wx - n['x']) ** 2 + (wy - n['y']) ** 2)

    def nearest_node_to(self, x, y):
        best = None
        best_d = float('inf')
        for name, pos in self.nodes.items():
            d = math.sqrt((pos['x'] - x) ** 2 + (pos['y'] - y) ** 2)
            if d < best_d:
                best_d = d
                best = name
        return best, best_d

    def _post_route(self, node_names, phase):
        points = []
        for name in node_names:
            if name in self.nodes:
                n = self.nodes[name]
                points.append({'x': float(n['x']), 'y': float(n['y'])})

        payload = {'points': points, 'phase': phase}

        def _send():
            try:
                req = urllib.request.Request(
                    BACKEND_URL.rstrip('/') + '/robot/route',
                    data=json.dumps(payload).encode('utf-8'),
                    headers={'Content-Type': 'application/json'},
                    method='POST',
                )
                with urllib.request.urlopen(req, timeout=2.0) as resp:
                    resp.read()
            except Exception as exc:
                self.get_logger().warn(f'route post failed: {exc}')

        threading.Thread(target=_send, daemon=True).start()

    def bfs(self, start, goal):
        if start == goal:
            return [start]
        visited = {start}
        queue = [(start, [start])]
        while queue:
            current, path = queue.pop(0)
            for neighbor in self.graph[current]:
                if neighbor not in visited:
                    new_path = path + [neighbor]
                    if neighbor == goal:
                        return new_path
                    visited.add(neighbor)
                    queue.append((neighbor, new_path))
        return []

    def compute_heading(self, from_xy, to_xy):
        return math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])

    def angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def enable_line(self, enabled):
        self.enable_pub.publish(Bool(data=bool(enabled)))

    def stop(self):
        self.cmd_pub.publish(Twist())

    def handle_goal(self, goal_request):
        self.get_logger().info('navigate_to_pose goal received')
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        self.get_logger().warn('cancel requested')
        self.cancel_requested = True
        return CancelResponse.ACCEPT

    def sleep_rate(self, rate_hz):
        rate = self.create_rate(rate_hz)
        rate.sleep()

    def execute_navigate(self, goal_handle):
        self.cancel_requested = False
        self.active_goal_handle = goal_handle

        pose = goal_handle.request.pose.pose
        gx = pose.position.x
        gy = pose.position.y
        self.get_logger().info(f'navigate to ({gx:.2f}, {gy:.2f})')

        result = NavigateToPose.Result()

        wait_start = self.get_clock().now().nanoseconds / 1e9
        while not self.have_odom:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - wait_start > 5.0:
                self.get_logger().error('no odometry, aborting')
                goal_handle.abort()
                return result
            self._sleep(0.1)

        start_node, sd = self.nearest_node_to(*self.world_pos())
        goal_node, gd = self.nearest_node_to(gx, gy)

        self.get_logger().info(f'start={start_node} (d={sd:.2f}), goal={goal_node} (d={gd:.2f})')

        if not start_node or not goal_node:
            self.get_logger().error('cannot find nodes')
            goal_handle.abort()
            return result

        route = self.bfs(start_node, goal_node)
        if not route:
            self.get_logger().error(f'no route from {start_node} to {goal_node}')
            goal_handle.abort()
            return result

        self.get_logger().info(f'route: {" -> ".join(route)}')
        self._post_route(route, phase='active')

        for idx in range(len(route) - 1):
            if self.cancel_requested:
                self.stop()
                self.enable_line(False)
                self._post_route([], phase='idle')
                goal_handle.canceled()
                return result

            next_node = route[idx + 1]
            next_pos = (self.nodes[next_node]['x'], self.nodes[next_node]['y'])

            heading = self.compute_heading(self.world_pos(), next_pos)
            self.align_to_heading(heading, goal_handle)
            if self.cancel_requested:
                self.stop()
                self.enable_line(False)
                self._post_route([], phase='idle')
                goal_handle.canceled()
                return result

            self.enable_line(True)
            self.follow_until_near(next_node, goal_handle)

            self.enable_line(False)
            self.stop()
            if self.cancel_requested:
                self._post_route([], phase='idle')
                goal_handle.canceled()
                return result

            self.get_logger().info(f'reached node {next_node}')

            fb = NavigateToPose.Feedback()
            fb.current_pose.pose.position.x = float(self.world_pos()[0])
            fb.current_pose.pose.position.y = float(self.world_pos()[1])
            goal_handle.publish_feedback(fb)

        self.final_approach(gx, gy, goal_handle)
        self.stop()
        self.enable_line(False)
        self._post_route([], phase='idle')

        self.get_logger().info('goal reached')
        goal_handle.succeed()
        return result

    def align_to_heading(self, target_yaw, goal_handle):
        self.enable_line(False)
        self.stop()
        self._sleep(0.2)

        while rclpy.ok() and not self.cancel_requested:
            diff = self.angle_diff(target_yaw, self.odom_yaw)
            if abs(diff) < YAW_TOLERANCE:
                break
            twist = Twist()
            twist.linear.x = TURN_CREEP
            twist.angular.z = TURN_SPEED if diff > 0 else -TURN_SPEED
            if abs(diff) < 0.4:
                twist.angular.z *= 0.4
            self.cmd_pub.publish(twist)
            self._sleep(0.05)

        self.stop()
        self._sleep(0.15)

    def follow_until_near(self, target_node, goal_handle):
        stall_counter = 0
        last_dist = None

        while rclpy.ok() and not self.cancel_requested:
            d = self.distance_to_node(target_node)
            if d < ARRIVAL_TOLERANCE:
                return

            if self.line_active:
                self.cmd_pub.publish(self.last_line_cmd)
            else:
                twist = Twist()
                twist.linear.x = 0.15
                target = (self.nodes[target_node]['x'], self.nodes[target_node]['y'])
                desired = self.compute_heading(self.world_pos(), target)
                twist.angular.z = 0.6 * self.angle_diff(desired, self.odom_yaw)
                self.cmd_pub.publish(twist)

            if last_dist is not None:
                if abs(last_dist - d) < 0.01:
                    stall_counter += 1
                else:
                    stall_counter = 0
            last_dist = d

            if stall_counter > 200:
                self.get_logger().warn(f'stalled near {target_node}, giving up')
                return

            fb = NavigateToPose.Feedback()
            fb.current_pose.pose.position.x = float(self.world_pos()[0])
            fb.current_pose.pose.position.y = float(self.world_pos()[1])
            goal_handle.publish_feedback(fb)

            self._sleep(0.05)

    def final_approach(self, gx, gy, goal_handle):
        while rclpy.ok() and not self.cancel_requested:
            wx, wy = self.world_pos()
            d = math.sqrt((gx - wx) ** 2 + (gy - wy) ** 2)
            if d < 0.5:
                return
            desired = self.compute_heading((wx, wy), (gx, gy))
            diff = self.angle_diff(desired, self.odom_yaw)
            twist = Twist()
            twist.linear.x = 0.15
            twist.angular.z = 0.6 * diff
            self.cmd_pub.publish(twist)
            self._sleep(0.05)

    def _sleep(self, seconds):
        end = self.get_clock().now().nanoseconds / 1e9 + seconds
        while rclpy.ok() and self.get_clock().now().nanoseconds / 1e9 < end:
            pass


def main():
    rclpy.init()
    node = CVNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
