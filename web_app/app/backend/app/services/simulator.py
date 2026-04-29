from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..config import settings
from ..db import SessionLocal
from ..models import Robot, RobotState, Task, TaskStatus
from ..schemas import RobotStatusOut, TaskOut
from .task_queue import get_task, next_pending, set_status
from .warehouse_map import map_config
from .ws_manager import manager

CONFIG = map_config()
STATIONS = CONFIG['stations']
NODES = {name: (node['world_x'], node['world_y']) for name, node in CONFIG['nodes'].items()}
GRAPH: dict[str, list[str]] = {}
for road in CONFIG['roads']:
    GRAPH.setdefault(road['a'], []).append(road['b'])
    GRAPH.setdefault(road['b'], []).append(road['a'])
STATION_LINKS = CONFIG['station_links']
DEPOT_ID = 'DEPOT'


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _nearest_node(x: float, y: float) -> str:
    return min(NODES, key=lambda n: _dist((x, y), NODES[n]))


def _normalize_angle(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle


def _angle_delta(current: float, target: float) -> float:
    return _normalize_angle(target - current)


def _segment_heading(from_xy: tuple[float, float], to_xy: tuple[float, float]) -> float:
    dx = to_xy[0] - from_xy[0]
    dy = to_xy[1] - from_xy[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return math.atan2(dy, dx)


def _step_turn(current: float, target: float, max_step: float) -> tuple[float, bool]:
    delta = _angle_delta(current, target)
    if abs(delta) <= max_step:
        return target, True
    current += max_step if delta > 0 else -max_step
    return _normalize_angle(current), False


def _astar(start: str, goal: str) -> list[str]:
    open_set = {start}
    came: dict[str, str] = {}
    g = {start: 0.0}
    f = {start: _dist(NODES[start], NODES[goal])}
    while open_set:
        current = min(open_set, key=lambda n: f.get(n, 1e18))
        if current == goal:
            path = [current]
            while current in came:
                current = came[current]
                path.append(current)
            return list(reversed(path))
        open_set.remove(current)
        for neigh in GRAPH.get(current, []):
            cand = g[current] + _dist(NODES[current], NODES[neigh])
            if cand < g.get(neigh, 1e18):
                came[neigh] = current
                g[neigh] = cand
                f[neigh] = cand + _dist(NODES[neigh], NODES[goal])
                open_set.add(neigh)
    return [start, goal]


def build_path(start_xy: tuple[float, float], station_id: str) -> list[dict[str, float]]:
    station_id = station_id.upper()
    start_node = _nearest_node(*start_xy)
    goal_node = STATION_LINKS[station_id]
    node_path = _astar(start_node, goal_node)

    points = [{'x': start_xy[0], 'y': start_xy[1]}]
    for node_name in node_path:
        x, y = NODES[node_name]
        if _dist((points[-1]['x'], points[-1]['y']), (x, y)) > 0.01:
            points.append({'x': x, 'y': y})

    gx, gy = STATIONS[station_id]['world_x'], STATIONS[station_id]['world_y']
    if _dist((points[-1]['x'], points[-1]['y']), (gx, gy)) > 0.01:
        points.append({'x': gx, 'y': gy})
    return points


@dataclass
class SimRuntime:
    route_points: list[dict[str, float]] = field(default_factory=list)
    route_task_id: int | None = None
    phase: str = 'idle'
    target_station: str | None = None
    motion_mode: str = 'idle'


runtime = SimRuntime()


async def ensure_robot(session: AsyncSession) -> Robot:
    result = await session.execute(select(Robot).where(Robot.name == 'amr-1'))
    robot = result.scalar_one_or_none()
    if robot is None:
        depot = STATIONS[DEPOT_ID]
        robot = Robot(
            name='amr-1',
            state=RobotState.IDLE.value,
            battery_pct=100.0,
            pos_x=depot['world_x'],
            pos_y=depot['world_y'],
            heading_rad=0.0,
            has_cargo=0,
            last_seen_at=_utcnow(),
        )
        session.add(robot)
        await session.commit()
        await session.refresh(robot)
    elif robot.heading_rad is None:
        robot.heading_rad = 0.0
        await session.commit()
        await session.refresh(robot)
    return robot


async def _broadcast_robot(robot: Robot) -> None:
    payload = RobotStatusOut.model_validate({
        'id': robot.id,
        'name': robot.name,
        'state': robot.state,
        'battery_pct': robot.battery_pct,
        'pos_x': robot.pos_x,
        'pos_y': robot.pos_y,
        'heading_rad': robot.heading_rad,
        'has_cargo': bool(robot.has_cargo),
        'current_task_id': robot.current_task_id,
        'last_seen_at': robot.last_seen_at,
        'motion_mode': runtime.motion_mode,
    }).model_dump(mode='json')
    await manager.broadcast({'type': 'robot.status', 'payload': payload})


async def _broadcast_task(task: Task) -> None:
    await manager.broadcast({'type': 'task.updated', 'payload': TaskOut.model_validate(task).model_dump(mode='json')})


async def _broadcast_route(task_id: int | None, points: list[dict[str, float]], target: str | None, phase: str) -> None:
    await manager.broadcast({'type': 'robot.route', 'payload': {'task_id': task_id, 'points': points, 'target': target, 'phase': phase, 'motion_mode': runtime.motion_mode}})


async def plan_task(session: AsyncSession, task: Task, robot: Robot, phase: str, target_station: str) -> None:
    task = await set_status(session, task.id, TaskStatus.HEADING_TO_PICKUP if phase == 'pickup' else TaskStatus.HEADING_TO_DROPOFF, assigned_robot_id=robot.id)
    robot.current_task_id = task.id
    robot.state = (RobotState.MOVING_TO_PICKUP if phase == 'pickup' else RobotState.MOVING_TO_DROPOFF).value
    robot.last_seen_at = _utcnow()
    await session.commit()
    await session.refresh(robot)

    points = build_path((robot.pos_x or 0.0, robot.pos_y or 0.0), target_station)
    runtime.route_points = points
    runtime.route_task_id = task.id
    runtime.phase = phase
    runtime.target_station = target_station.upper()
    runtime.motion_mode = 'rotating' if len(points) >= 2 else 'idle'

    await _broadcast_task(task)
    await _broadcast_robot(robot)
    await _broadcast_route(task.id, points, runtime.target_station, phase)


async def confirm_load(task_id: int) -> Task | None:
    async with SessionLocal() as session:
        task = await get_task(session, task_id)
        if task is None or task.status != TaskStatus.AWAITING_LOAD.value:
            return None
        robot = await ensure_robot(session)
        robot.has_cargo = 1
        await session.commit()
        await session.refresh(robot)
        await manager.broadcast({'type': 'toast', 'payload': {'message': 'Погрузка подтверждена'}})
        if settings.use_internal_simulator:
            await plan_task(session, task, robot, 'dropoff', task.destination)
            return await get_task(session, task_id)
        task = await set_status(session, task.id, TaskStatus.HEADING_TO_DROPOFF, assigned_robot_id=robot.id)
        robot.state = RobotState.MOVING_TO_DROPOFF.value
        robot.current_task_id = task.id
        robot.last_seen_at = _utcnow()
        runtime.phase = 'dropoff'
        runtime.target_station = task.destination
        runtime.route_points = []
        runtime.route_task_id = task.id
        runtime.motion_mode = 'idle'
        await session.commit(); await session.refresh(robot)
        await _broadcast_task(task); await _broadcast_robot(robot); await _broadcast_route(task.id, [], task.destination, 'dropoff')
        return task


async def confirm_unload(task_id: int) -> Task | None:
    async with SessionLocal() as session:
        task = await get_task(session, task_id)
        if task is None or task.status != TaskStatus.AWAITING_UNLOAD.value:
            return None
        robot = await ensure_robot(session)
        task = await set_status(session, task.id, TaskStatus.DONE)
        robot.has_cargo = 0
        robot.state = RobotState.IDLE.value
        robot.current_task_id = None
        robot.last_seen_at = _utcnow()
        await session.commit()
        await session.refresh(robot)

        if settings.use_internal_simulator:
            charge_path = build_path((robot.pos_x or 0.0, robot.pos_y or 0.0), DEPOT_ID)
            runtime.route_points = charge_path
            runtime.route_task_id = None
            runtime.phase = 'return'
            runtime.target_station = DEPOT_ID
            runtime.motion_mode = 'rotating' if len(charge_path) >= 2 else 'idle'
            await _broadcast_task(task); await _broadcast_robot(robot); await _broadcast_route(None, charge_path, DEPOT_ID, 'return')
        else:
            runtime.route_points = []
            runtime.route_task_id = None
            runtime.phase = 'idle'
            runtime.target_station = None
            runtime.motion_mode = 'idle'
            await _broadcast_task(task); await _broadcast_robot(robot); await _broadcast_route(None, [], DEPOT_ID, 'idle')

        await manager.broadcast({'type': 'toast', 'payload': {'message': 'Разгрузка подтверждена'}})
        return task


async def simulator_loop(stop_event: asyncio.Event) -> None:
    speed_mps = 0.65
    turn_rate_rad_s = math.radians(120.0)
    tick = 0.1
    turn_tolerance = math.radians(3.0)

    while not stop_event.is_set():
        async with SessionLocal() as session:
            robot = await ensure_robot(session)

            if runtime.phase == 'idle':
                runtime.motion_mode = 'idle'
                task = await next_pending(session)
                if task is not None:
                    await plan_task(session, task, robot, 'pickup', task.source)
            else:
                if len(runtime.route_points) >= 2:
                    cx, cy = robot.pos_x or 0.0, robot.pos_y or 0.0
                    nx, ny = runtime.route_points[1]['x'], runtime.route_points[1]['y']

                    target_heading = _segment_heading((cx, cy), (nx, ny))
                    current_heading = _normalize_angle(robot.heading_rad or 0.0)
                    delta = _angle_delta(current_heading, target_heading)
                    max_turn_step = turn_rate_rad_s * tick

                    if abs(delta) > turn_tolerance:
                        runtime.motion_mode = 'rotating'
                        robot.heading_rad, _ = _step_turn(current_heading, target_heading, max_turn_step)
                        robot.last_seen_at = _utcnow()
                        robot.battery_pct = max(0.0, (robot.battery_pct or 100.0) - 0.001)
                        await session.commit(); await session.refresh(robot)
                        await _broadcast_robot(robot)
                        await _broadcast_route(runtime.route_task_id, runtime.route_points, runtime.target_station, runtime.phase)
                    else:
                        runtime.motion_mode = 'driving'
                        robot.heading_rad = target_heading
                        dx, dy = nx - cx, ny - cy
                        dist = math.hypot(dx, dy)
                        step = speed_mps * tick

                        if dist <= step:
                            robot.pos_x, robot.pos_y = nx, ny
                            runtime.route_points.pop(0)
                            if len(runtime.route_points) >= 2:
                                runtime.motion_mode = 'rotating'
                        elif dist > 0:
                            robot.pos_x = cx + dx / dist * step
                            robot.pos_y = cy + dy / dist * step

                        robot.last_seen_at = _utcnow()
                        robot.battery_pct = max(0.0, (robot.battery_pct or 100.0) - 0.002)
                        await session.commit(); await session.refresh(robot)
                        await _broadcast_robot(robot)
                        await _broadcast_route(runtime.route_task_id, runtime.route_points, runtime.target_station, runtime.phase)
                else:
                    runtime.motion_mode = 'waiting'
                    if runtime.phase == 'pickup' and runtime.route_task_id:
                        task = await set_status(session, runtime.route_task_id, TaskStatus.AWAITING_LOAD)
                        robot.state = RobotState.WAITING_FOR_LOAD.value
                        robot.last_seen_at = _utcnow()
                        await session.commit(); await session.refresh(robot)
                        await _broadcast_task(task); await _broadcast_robot(robot)
                        await _broadcast_route(task.id, [], task.source, 'awaiting_load')
                        runtime.phase = 'awaiting_load'
                    elif runtime.phase == 'dropoff' and runtime.route_task_id:
                        task = await set_status(session, runtime.route_task_id, TaskStatus.AWAITING_UNLOAD)
                        robot.state = RobotState.WAITING_FOR_UNLOAD.value
                        robot.last_seen_at = _utcnow()
                        await session.commit(); await session.refresh(robot)
                        await _broadcast_task(task); await _broadcast_robot(robot)
                        await _broadcast_route(task.id, [], task.destination, 'awaiting_unload')
                        runtime.phase = 'awaiting_unload'
                    elif runtime.phase == 'return':
                        robot.state = RobotState.IDLE.value
                        robot.last_seen_at = _utcnow()
                        await session.commit(); await session.refresh(robot)
                        runtime.phase = 'idle'
                        runtime.motion_mode = 'idle'
                        runtime.route_points = []
                        runtime.target_station = None
                        await _broadcast_robot(robot)
                        await _broadcast_route(None, [], DEPOT_ID, 'idle')

            if runtime.phase in {'awaiting_load', 'awaiting_unload'}:
                runtime.motion_mode = 'waiting'

        await asyncio.sleep(tick)
