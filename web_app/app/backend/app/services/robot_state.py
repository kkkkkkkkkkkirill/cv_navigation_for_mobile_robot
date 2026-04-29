from __future__ import annotations

from datetime import datetime, timedelta, timezone

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..config import settings
from ..models import Robot, RobotState
from ..schemas import RobotStatusIn

# Временное in-memory хранилище motion_mode по имени робота
ROBOT_MOTION_MODES: dict[str, str | None] = {}


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


def _as_aware_utc(value: datetime | None) -> datetime | None:
    if value is None:
        return None
    if value.tzinfo is None:
        return value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc)


async def upsert_status(session: AsyncSession, report: RobotStatusIn) -> Robot:
    result = await session.execute(select(Robot).where(Robot.name == report.name))
    robot = result.scalar_one_or_none()
    if robot is None:
        robot = Robot(name=report.name)
        session.add(robot)

    fields_set = report.model_fields_set

    if 'state' in fields_set and report.state is not None:
        robot.state = report.state.value
    elif not robot.state:
        robot.state = RobotState.IDLE.value

    if 'battery_pct' in fields_set and report.battery_pct is not None:
        robot.battery_pct = report.battery_pct
    elif robot.battery_pct is None:
        robot.battery_pct = 100.0

    if 'pos_x' in fields_set and report.pos_x is not None:
        robot.pos_x = report.pos_x
    if 'pos_y' in fields_set and report.pos_y is not None:
        robot.pos_y = report.pos_y
    if 'heading_rad' in fields_set and report.heading_rad is not None:
        robot.heading_rad = report.heading_rad

    if 'has_cargo' in fields_set and report.has_cargo is not None:
        robot.has_cargo = 1 if report.has_cargo else 0

    if 'current_task_id' in fields_set:
        robot.current_task_id = report.current_task_id

    robot.last_seen_at = _utcnow()

    if report.motion_mode is not None:
        ROBOT_MOTION_MODES[report.name] = report.motion_mode
    else:
        ROBOT_MOTION_MODES.setdefault(report.name, None)

    await session.commit()
    await session.refresh(robot)
    return robot


async def list_robots(session: AsyncSession) -> list[Robot]:
    result = await session.execute(select(Robot).order_by(Robot.id.asc()))
    robots = list(result.scalars().all())
    cutoff = _utcnow() - timedelta(seconds=settings.robot_offline_after_s)

    for robot in robots:
        last_seen_at = _as_aware_utc(robot.last_seen_at)
        if last_seen_at is None or last_seen_at < cutoff:
            robot.state = RobotState.OFFLINE.value

    return robots


def get_motion_mode(robot_name: str) -> str | None:
    return ROBOT_MOTION_MODES.get(robot_name)
