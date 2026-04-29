
from __future__ import annotations

from datetime import datetime, timezone
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from ..models import Task, TaskStatus
from ..schemas import TaskCreate


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


async def enqueue(session: AsyncSession, payload: TaskCreate) -> Task:
    task = Task(
        source=payload.source,
        destination=payload.destination,
        payload=payload.payload,
        priority=payload.priority,
        requested_by=payload.requested_by,
        status=TaskStatus.PENDING.value,
    )
    session.add(task)
    await session.commit()
    await session.refresh(task)
    return task


async def list_tasks(session: AsyncSession, *, status: TaskStatus | None = None, limit: int = 100) -> list[Task]:
    stmt = select(Task).order_by(Task.created_at.desc()).limit(limit)
    if status is not None:
        stmt = stmt.where(Task.status == status.value)
    result = await session.execute(stmt)
    return list(result.scalars().all())


async def get_task(session: AsyncSession, task_id: int) -> Task | None:
    return await session.get(Task, task_id)


async def next_pending(session: AsyncSession) -> Task | None:
    stmt = select(Task).where(Task.status == TaskStatus.PENDING.value).order_by(Task.priority.asc(), Task.created_at.asc()).limit(1)
    result = await session.execute(stmt)
    return result.scalar_one_or_none()


async def set_status(session: AsyncSession, task_id: int, status: TaskStatus, *, error_message: str | None = None, assigned_robot_id: int | None = None) -> Task | None:
    task = await session.get(Task, task_id)
    if task is None:
        return None
    task.status = status.value
    task.updated_at = _utcnow()
    if assigned_robot_id is not None:
        task.assigned_robot_id = assigned_robot_id
    if status in {TaskStatus.HEADING_TO_PICKUP, TaskStatus.HEADING_TO_DROPOFF}:
        task.started_at = task.started_at or _utcnow()
    if status in {TaskStatus.DONE, TaskStatus.FAILED, TaskStatus.CANCELLED}:
        task.finished_at = _utcnow()
    if error_message is not None:
        task.error_message = error_message
    await session.commit()
    await session.refresh(task)
    return task
