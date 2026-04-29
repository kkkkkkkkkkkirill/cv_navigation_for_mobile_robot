from __future__ import annotations

import math
from typing import Any

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..models import Robot, RobotState, Task, TaskStatus
from ..schemas import RobotDispatchOut, RobotEventIn
from .task_queue import next_pending, get_task, set_status
from .simulator import (
    DEPOT_ID,
    STATIONS,
    build_path,
    ensure_robot,
    runtime,
    _broadcast_robot,
    _broadcast_route,
    _broadcast_task,
)
from .ws_manager import manager

ACTIVE_TASK_STATUSES = {
    TaskStatus.HEADING_TO_PICKUP.value,
    TaskStatus.AWAITING_LOAD.value,
    TaskStatus.HEADING_TO_DROPOFF.value,
    TaskStatus.AWAITING_UNLOAD.value,
}


def _dist(a: tuple[float | None, float | None], b: tuple[float | None, float | None]) -> float:
    ax = float(a[0] or 0.0)
    ay = float(a[1] or 0.0)
    bx = float(b[0] or 0.0)
    by = float(b[1] or 0.0)
    return math.hypot(ax - bx, ay - by)


async def _current_task(session: AsyncSession, robot: Robot) -> Task | None:
    stmt = (
        select(Task)
        .where(Task.assigned_robot_id == robot.id)
        .where(Task.status.in_(list(ACTIVE_TASK_STATUSES)))
        .order_by(Task.created_at.asc())
        .limit(1)
    )
    result = await session.execute(stmt)
    return result.scalar_one_or_none()


async def _compute_route(task: Task | None, robot: Robot, phase: str, target_station: str | None) -> list[dict[str, float]]:
    if phase in {'pickup', 'dropoff', 'return'} and target_station:
        return build_path((robot.pos_x or 0.0, robot.pos_y or 0.0), target_station)
    return []


async def _set_runtime_and_broadcast(task: Task | None, robot: Robot, phase: str, target_station: str | None) -> None:
    points = await _compute_route(task, robot, phase, target_station)
    runtime.route_points = points
    runtime.route_task_id = task.id if task else None
    runtime.phase = phase
    runtime.target_station = target_station.upper() if target_station else None
    runtime.motion_mode = 'path_planned' if points else 'idle'
    if task is not None:
        await _broadcast_task(task)
    await _broadcast_robot(robot)
    await _broadcast_route(runtime.route_task_id, points, runtime.target_station, phase)


async def get_dispatch(session: AsyncSession, *, robot_name: str = 'amr-1') -> RobotDispatchOut:
    robot = await ensure_robot(session)
    task = await _current_task(session, robot)

    if task is None:
        pending = await next_pending(session)
        if pending is not None:
            task = await set_status(session, pending.id, TaskStatus.HEADING_TO_PICKUP, assigned_robot_id=robot.id)
            robot.current_task_id = task.id
            robot.state = RobotState.MOVING_TO_PICKUP.value
            await session.commit()
            await session.refresh(robot)
            await _set_runtime_and_broadcast(task, robot, 'pickup', task.source)
            return RobotDispatchOut(
                task_id=task.id,
                phase='pickup',
                target_station=task.source,
                source=task.source,
                destination=task.destination,
                should_return_to_depot=True,
                robot_name=robot.name,
                mode='external_bridge',
            )

        depot = STATIONS[DEPOT_ID]
        if _dist((robot.pos_x, robot.pos_y), (depot['world_x'], depot['world_y'])) > 1.0:
            await _set_runtime_and_broadcast(None, robot, 'return', DEPOT_ID)
            return RobotDispatchOut(
                task_id=None,
                phase='return',
                target_station=DEPOT_ID.lower(),
                source=None,
                destination=None,
                should_return_to_depot=True,
                robot_name=robot.name,
                mode='external_bridge',
            )

        await _set_runtime_and_broadcast(None, robot, 'idle', None)
        return RobotDispatchOut(
            task_id=None,
            phase='idle',
            target_station=None,
            source=None,
            destination=None,
            should_return_to_depot=False,
            robot_name=robot.name,
            mode='external_bridge',
        )

    phase_map = {
        TaskStatus.HEADING_TO_PICKUP.value: ('pickup', task.source),
        TaskStatus.AWAITING_LOAD.value: ('awaiting_load', task.source),
        TaskStatus.HEADING_TO_DROPOFF.value: ('dropoff', task.destination),
        TaskStatus.AWAITING_UNLOAD.value: ('awaiting_unload', task.destination),
    }
    phase, target_station = phase_map.get(task.status, ('idle', None))
    await _set_runtime_and_broadcast(task, robot, phase, target_station)
    return RobotDispatchOut(
        task_id=task.id,
        phase=phase,
        target_station=target_station,
        source=task.source,
        destination=task.destination,
        should_return_to_depot=True,
        robot_name=robot.name,
        mode='external_bridge',
    )


async def handle_event(session: AsyncSession, body: RobotEventIn) -> dict[str, Any]:
    robot = await ensure_robot(session)

    task: Task | None = None
    if body.task_id is not None:
        task = await get_task(session, body.task_id)
    if task is None and robot.current_task_id is not None:
        task = await get_task(session, robot.current_task_id)

    if body.event == 'nav_started':
        if task is not None:
            if task.status == TaskStatus.HEADING_TO_PICKUP.value:
                robot.state = RobotState.MOVING_TO_PICKUP.value
                robot.current_task_id = task.id
                await session.commit()
                await session.refresh(robot)
                await _set_runtime_and_broadcast(task, robot, 'pickup', task.source)
            elif task.status == TaskStatus.HEADING_TO_DROPOFF.value:
                robot.state = RobotState.MOVING_TO_DROPOFF.value
                robot.current_task_id = task.id
                await session.commit()
                await session.refresh(robot)
                await _set_runtime_and_broadcast(task, robot, 'dropoff', task.destination)
        else:
            robot.state = RobotState.IDLE.value
            await session.commit()
            await session.refresh(robot)
            await _set_runtime_and_broadcast(None, robot, 'return', DEPOT_ID)
        return {'ok': True, 'event': body.event, 'task_id': getattr(task, 'id', None)}

    if body.event == 'arrived_pickup':
        if task is None or task.status != TaskStatus.HEADING_TO_PICKUP.value:
            return {'ok': False, 'reason': 'task_not_heading_to_pickup'}
        task = await set_status(session, task.id, TaskStatus.AWAITING_LOAD, assigned_robot_id=robot.id)
        robot.state = RobotState.WAITING_FOR_LOAD.value
        robot.current_task_id = task.id
        await session.commit()
        await session.refresh(robot)
        runtime.phase = 'awaiting_load'
        runtime.target_station = task.source.upper()
        runtime.route_points = []
        runtime.route_task_id = task.id
        runtime.motion_mode = 'idle'
        await _broadcast_task(task)
        await _broadcast_robot(robot)
        await _broadcast_route(task.id, [], task.source.upper(), 'awaiting_load')
        await manager.broadcast({'type': 'toast', 'payload': {'message': f'Robot arrived at pickup {task.source.upper()}.'}})
        return {'ok': True, 'event': body.event, 'task_id': task.id}

    if body.event == 'arrived_dropoff':
        if task is None or task.status != TaskStatus.HEADING_TO_DROPOFF.value:
            return {'ok': False, 'reason': 'task_not_heading_to_dropoff'}
        task = await set_status(session, task.id, TaskStatus.AWAITING_UNLOAD, assigned_robot_id=robot.id)
        robot.state = RobotState.WAITING_FOR_UNLOAD.value
        robot.current_task_id = task.id
        await session.commit()
        await session.refresh(robot)
        runtime.phase = 'awaiting_unload'
        runtime.target_station = task.destination.upper()
        runtime.route_points = []
        runtime.route_task_id = task.id
        runtime.motion_mode = 'idle'
        await _broadcast_task(task)
        await _broadcast_robot(robot)
        await _broadcast_route(task.id, [], task.destination.upper(), 'awaiting_unload')
        await manager.broadcast({'type': 'toast', 'payload': {'message': f'Robot arrived at dropoff {task.destination.upper()}.'}})
        return {'ok': True, 'event': body.event, 'task_id': task.id}

    if body.event == 'nav_failed':
        if task is not None:
            task = await set_status(session, task.id, TaskStatus.FAILED, error_message=body.message or 'navigation_failed', assigned_robot_id=robot.id)
        robot.state = RobotState.ERROR.value
        robot.current_task_id = None
        await session.commit()
        await session.refresh(robot)
        runtime.phase = 'idle'
        runtime.target_station = None
        runtime.route_points = []
        runtime.route_task_id = None
        runtime.motion_mode = 'idle'
        if task is not None:
            await _broadcast_task(task)
        await _broadcast_robot(robot)
        await _broadcast_route(None, [], None, 'idle')
        await manager.broadcast({'type': 'toast', 'payload': {'message': body.message or 'Navigation failed'}})
        return {'ok': True, 'event': body.event, 'task_id': getattr(task, 'id', None)}

    if body.event == 'returned_to_depot':
        robot.state = RobotState.IDLE.value
        robot.current_task_id = None
        await session.commit()
        await session.refresh(robot)
        runtime.phase = 'idle'
        runtime.target_station = None
        runtime.route_points = []
        runtime.route_task_id = None
        runtime.motion_mode = 'idle'
        await _broadcast_robot(robot)
        await _broadcast_route(None, [], DEPOT_ID, 'idle')
        return {'ok': True, 'event': body.event, 'task_id': None}

    return {'ok': False, 'reason': 'unknown_event'}
