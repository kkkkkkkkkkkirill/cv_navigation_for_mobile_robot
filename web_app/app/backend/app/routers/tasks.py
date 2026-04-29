from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy.ext.asyncio import AsyncSession

from ..config import settings
from ..db import get_session
from ..models import TaskStatus
from ..schemas import TaskCreate, TaskOut
from ..services import task_queue
from ..services.simulator import confirm_load, confirm_unload
from ..services.ws_manager import manager

router = APIRouter(prefix='/tasks', tags=['tasks'])


def _event(type_: str, task) -> dict:
    return {'type': type_, 'payload': TaskOut.model_validate(task).model_dump(mode='json')}


@router.post('', response_model=TaskOut, status_code=status.HTTP_201_CREATED)
async def create_task(body: TaskCreate, session: AsyncSession = Depends(get_session)) -> TaskOut:
    task = await task_queue.enqueue(session, body)
    await manager.broadcast(_event('task.created', task))
    return TaskOut.model_validate(task)


@router.get('', response_model=list[TaskOut])
async def list_tasks(status_: TaskStatus | None = Query(default=None, alias='status'), limit: int = Query(default=100, ge=1, le=500), session: AsyncSession = Depends(get_session)) -> list[TaskOut]:
    tasks = await task_queue.list_tasks(session, status=status_, limit=limit)
    return [TaskOut.model_validate(t) for t in tasks]


@router.get('/{task_id}', response_model=TaskOut)
async def get_task(task_id: int, session: AsyncSession = Depends(get_session)) -> TaskOut:
    task = await task_queue.get_task(session, task_id)
    if task is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, 'Task not found')
    return TaskOut.model_validate(task)


@router.post('/{task_id}/confirm-load', response_model=TaskOut)
async def post_confirm_load(task_id: int) -> TaskOut:
    task = await confirm_load(task_id)
    if task is None:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, 'Task is not waiting for load confirmation')
    if not settings.use_internal_simulator:
        await manager.broadcast({'type': 'toast', 'payload': {'message': 'Погрузка подтверждена. Внешний робот может ехать к точке доставки.'}})
    return TaskOut.model_validate(task)


@router.post('/{task_id}/confirm-unload', response_model=TaskOut)
async def post_confirm_unload(task_id: int) -> TaskOut:
    task = await confirm_unload(task_id)
    if task is None:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, 'Task is not waiting for unload confirmation')
    if not settings.use_internal_simulator:
        await manager.broadcast({'type': 'toast', 'payload': {'message': 'Разгрузка подтверждена. Робот может вернуться в DEPOT.'}})
    return TaskOut.model_validate(task)
