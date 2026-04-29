#!/usr/bin/env python3
from __future__ import annotations

import json
import heapq
import math
import os
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

urllib.request.install_opener(
    urllib.request.build_opener(urllib.request.ProxyHandler({}))
)

BACKEND = os.environ.get('AMR_BACKEND_URL', 'http://127.0.0.1:8010').rstrip('/')
ROBOT_NAME = os.environ.get('AMR_ROBOT_NAME', 'amr-1')
POLL_INTERVAL_S = float(os.environ.get('AMR_DISPATCH_POLL_S', '1.0'))
ACTION_NAME = os.environ.get('AMR_NAV2_ACTION', 'navigate_to_pose')
DEFAULT_WAREHOUSE_MAP = str((Path(__file__).resolve().parents[3] / 'shared' / 'warehouse_map.yaml').resolve())
WAREHOUSE_MAP_PATH = os.environ.get('AMR_WAREHOUSE_MAP_PATH', DEFAULT_WAREHOUSE_MAP)
USE_GRAPH_WAYPOINTS = os.environ.get('AMR_NAV2_USE_GRAPH_WAYPOINTS', 'false').lower() in {'1', 'true', 'yes', 'on'}


def get_json(path: str) -> dict[str, Any]:
    try:
        with urllib.request.urlopen(BACKEND + path, timeout=2.0) as resp:
            return json.loads(resp.read().decode('utf-8'))
    except urllib.error.HTTPError as exc:
        if exc.code == 503:
            return {'task_id': None, 'phase': 'idle', 'target_station': None, 'source': None, 'destination': None, 'should_return_to_depot': False, 'robot_name': ROBOT_NAME, 'mode': 'external_bridge'}
        raise


def post_json(path: str, payload: dict[str, Any]) -> dict[str, Any]:
    req = urllib.request.Request(
        BACKEND + path,
        data=json.dumps(payload).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urllib.request.urlopen(req, timeout=5.0) as resp:
        return json.loads(resp.read().decode('utf-8'))


def yaw_to_quat(yaw: float) -> tuple[float, float]:
    from math import sin, cos
    return sin(yaw / 2.0), cos(yaw / 2.0)


class TaskNav2Bridge(Node):
    def __init__(self) -> None:
        super().__init__('task_nav2_bridge')
        self._client = ActionClient(self, NavigateToPose, ACTION_NAME)
        self._stations, self._nodes, self._graph, self._station_links = self._load_map()
        self._active_key = None
        self._goal_future = None
        self._result_future = None
        self._active_phase = None
        self._active_task_id = None
        self._waypoints: list[dict[str, float | str]] = []
        self._waypoint_index = 0
        self.create_timer(POLL_INTERVAL_S, self._tick)

        self.get_logger().info(f'nav2 bridge started against {BACKEND}')
        self.get_logger().info(f'using warehouse map {WAREHOUSE_MAP_PATH}')
        self.get_logger().info(f'waiting for Nav2 action [{ACTION_NAME}] ...')
        self._client.wait_for_server()
        self.get_logger().info('Nav2 action server is available.')

    def _load_map(self) -> tuple[
        dict[str, dict[str, float]],
        dict[str, dict[str, float]],
        dict[str, list[tuple[str, float]]],
        dict[str, str],
    ]:
        with open(WAREHOUSE_MAP_PATH, encoding='utf-8') as f:
            data = yaml.safe_load(f)['warehouse']

        stations = {k.upper(): v for k, v in data['stations'].items()}
        nodes = {k: {'x': float(v['x']), 'y': float(v['y'])} for k, v in data['nodes'].items()}
        graph: dict[str, list[tuple[str, float]]] = {name: [] for name in nodes}
        for a, b in data['lanes']:
            if a not in nodes or b not in nodes:
                continue
            dist = math.hypot(nodes[a]['x'] - nodes[b]['x'], nodes[a]['y'] - nodes[b]['y'])
            graph[a].append((b, dist))
            graph[b].append((a, dist))

        explicit_links = data.get('station_links', {})
        station_links: dict[str, str] = {}
        for name, station in stations.items():
            explicit = explicit_links.get(name) or explicit_links.get(name.lower())
            if explicit in nodes:
                station_links[name] = explicit
            else:
                station_links[name] = self._nearest_node(float(station['x']), float(station['y']), nodes)

        return stations, nodes, graph, station_links

    @staticmethod
    def _nearest_node(x: float, y: float, nodes: dict[str, dict[str, float]]) -> str:
        return min(nodes, key=lambda name: math.hypot(nodes[name]['x'] - x, nodes[name]['y'] - y))

    def _current_robot_pose(self) -> tuple[float, float] | None:
        try:
            with urllib.request.urlopen(BACKEND + '/robot/status', timeout=1.0) as resp:
                robots = json.loads(resp.read().decode('utf-8'))
        except Exception:
            return None

        if not isinstance(robots, list):
            return None
        for robot in robots:
            if robot.get('name') != ROBOT_NAME:
                continue
            x = robot.get('pos_x')
            y = robot.get('pos_y')
            if x is not None and y is not None:
                return float(x), float(y)
        return None

    def _shortest_node_path(self, start: str, goal: str) -> list[str]:
        queue: list[tuple[float, str]] = [(0.0, start)]
        distances = {start: 0.0}
        parents: dict[str, str | None] = {start: None}

        while queue:
            cost, node = heapq.heappop(queue)
            if node == goal:
                break
            if cost > distances.get(node, float('inf')):
                continue
            for nxt, edge_cost in self._graph.get(node, []):
                new_cost = cost + edge_cost
                if new_cost < distances.get(nxt, float('inf')):
                    distances[nxt] = new_cost
                    parents[nxt] = node
                    heapq.heappush(queue, (new_cost, nxt))

        if goal not in parents:
            return [start, goal]

        path = []
        node: str | None = goal
        while node is not None:
            path.append(node)
            node = parents[node]
        return list(reversed(path))


    def _build_waypoints(self, station_name: str) -> list[dict[str, float | str]]:
        station = self._stations[station_name]
        station_x = float(station['x'])
        station_y = float(station['y'])
        station_yaw = float(station.get('yaw', 0.0))

        if not USE_GRAPH_WAYPOINTS:
            return [{
                'name': station_name,
                'x': station_x,
                'y': station_y,
                'yaw': station_yaw,
            }]

        current_pose = self._current_robot_pose()
        if current_pose is None:
            depot = self._stations.get('DEPOT', {'x': 10.5, 'y': -8.0})
            current_pose = (float(depot['x']), float(depot['y']))

        start_node = self._nearest_node(current_pose[0], current_pose[1], self._nodes)
        target_node = self._station_links[station_name]
        node_path = self._shortest_node_path(start_node, target_node)

        raw_points: list[dict[str, float | str]] = []
        if math.hypot(self._nodes[start_node]['x'] - current_pose[0], self._nodes[start_node]['y'] - current_pose[1]) > 0.35:
            raw_points.append({'name': start_node, 'x': self._nodes[start_node]['x'], 'y': self._nodes[start_node]['y']})

        for node in node_path[1:]:
            raw_points.append({'name': node, 'x': self._nodes[node]['x'], 'y': self._nodes[node]['y']})

        raw_points.append({
            'name': station_name,
            'x': station_x,
            'y': station_y,
            'yaw': station_yaw,
        })

        waypoints: list[dict[str, float | str]] = []
        previous = {'x': current_pose[0], 'y': current_pose[1]}
        for i, point in enumerate(raw_points):
            is_final_station = i == len(raw_points) - 1 and point['name'] == station_name
            if is_final_station:
                yaw = float(point.get('yaw', station_yaw))
            else:
                yaw = math.atan2(float(point['y']) - float(previous['y']), float(point['x']) - float(previous['x']))
            waypoints.append({**point, 'yaw': yaw})
            previous = point
        return waypoints

    def _tick(self) -> None:
        if self._goal_future is not None or self._result_future is not None:
            return

        dispatch = get_json('/robot/dispatch')
        key = (dispatch.get('task_id'), dispatch.get('phase'), dispatch.get('target_station'))

        if key != self._active_key:
            self.get_logger().info(f'dispatch={dispatch}')
            self._active_key = key

        phase = dispatch.get('phase')
        task_id = dispatch.get('task_id')
        station_name = (dispatch.get('target_station') or '').upper() if dispatch.get('target_station') else None

        if phase not in {'pickup', 'dropoff', 'return'} or not station_name:
            return

        if station_name not in self._stations:
            self.get_logger().error(f'unknown station [{station_name}]')
            post_json('/robot/event', {'robot_name': ROBOT_NAME, 'event': 'nav_failed', 'task_id': task_id, 'message': f'unknown_station:{station_name}'})
            return

        self._active_phase = phase
        self._active_task_id = task_id
        self._waypoints = self._build_waypoints(station_name)
        self._waypoint_index = 0

        post_json('/robot/event', {'robot_name': ROBOT_NAME, 'event': 'nav_started', 'task_id': task_id})

        route = ' -> '.join(str(wp['name']) for wp in self._waypoints)
        self.get_logger().info(f'route phase={phase} station={station_name}: {route}')
        self._send_current_waypoint()

    def _send_current_waypoint(self) -> None:
        if self._waypoint_index >= len(self._waypoints):
            self._finish_route()
            return

        waypoint = self._waypoints[self._waypoint_index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(waypoint['x'])
        goal.pose.pose.position.y = float(waypoint['y'])
        qz, qw = yaw_to_quat(float(waypoint['yaw']))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'sending waypoint {self._waypoint_index + 1}/{len(self._waypoints)} '
            f'name={waypoint["name"]} x={float(waypoint["x"]):.2f} y={float(waypoint["y"]):.2f} '
            f'yaw={float(waypoint["yaw"]):.2f}'
        )
        self._goal_future = self._client.send_goal_async(goal)
        self._goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        self._goal_future = None
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('NavigateToPose goal rejected.')
            post_json('/robot/event', {'robot_name': ROBOT_NAME, 'event': 'nav_failed', 'task_id': self._active_task_id, 'message': 'goal_rejected'})
            self._active_phase = None
            self._active_task_id = None
            return

        self.get_logger().info('NavigateToPose goal accepted.')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        self._result_future = None
        result = future.result()
        status = int(result.status)

        if status == 4:
            self._waypoint_index += 1
            if self._waypoint_index < len(self._waypoints):
                self._send_current_waypoint()
                return
            self._finish_route()
        else:
            self.get_logger().error(f'NavigateToPose failed with status={status}')
            post_json('/robot/event', {'robot_name': ROBOT_NAME, 'event': 'nav_failed', 'task_id': self._active_task_id, 'message': f'status_{status}'})
            self._clear_route()

    def _finish_route(self) -> None:
        if self._active_phase == 'pickup':
            event = 'arrived_pickup'
        elif self._active_phase == 'dropoff':
            event = 'arrived_dropoff'
        else:
            event = 'returned_to_depot'
        self.get_logger().info(f'event={event} task_id={self._active_task_id}')
        post_json('/robot/event', {'robot_name': ROBOT_NAME, 'event': event, 'task_id': self._active_task_id})
        self._clear_route()

    def _clear_route(self) -> None:
        self._active_phase = None
        self._active_task_id = None
        self._waypoints = []
        self._waypoint_index = 0


def main() -> None:
    rclpy.init()
    node = TaskNav2Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
