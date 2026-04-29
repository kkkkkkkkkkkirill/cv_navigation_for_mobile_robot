from __future__ import annotations

from datetime import datetime
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from .models import RobotState, TaskStatus


class TaskCreate(BaseModel):
    source: str = Field(..., min_length=1, max_length=64)
    destination: str = Field(..., min_length=1, max_length=64)
    payload: str | None = Field(None, max_length=512)
    priority: int = Field(5, ge=0, le=9)
    requested_by: str | None = Field(None, max_length=64)


class TaskOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int
    source: str
    destination: str
    payload: str | None
    priority: int
    status: TaskStatus
    requested_by: str | None
    assigned_robot_id: int | None
    created_at: datetime
    updated_at: datetime
    started_at: datetime | None
    finished_at: datetime | None
    error_message: str | None


class TaskUpdateStatus(BaseModel):
    status: TaskStatus
    error_message: str | None = None


class RobotStatusIn(BaseModel):
    name: str = Field(..., min_length=1, max_length=64)
    state: RobotState | None = None
    battery_pct: float | None = Field(None, ge=0, le=100)
    pos_x: float | None = None
    pos_y: float | None = None
    heading_rad: float | None = None
    has_cargo: bool | None = None
    current_task_id: int | None = None
    motion_mode: str | None = None


class RobotStatusOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int
    name: str
    state: RobotState
    battery_pct: float | None
    pos_x: float | None
    pos_y: float | None
    heading_rad: float | None
    has_cargo: bool = False
    current_task_id: int | None = None
    last_seen_at: datetime | None
    motion_mode: str | None = None


class RobotDispatchOut(BaseModel):
    task_id: int | None = None
    phase: Literal['idle', 'pickup', 'awaiting_load', 'dropoff', 'awaiting_unload', 'return']
    target_station: str | None = None
    source: str | None = None
    destination: str | None = None
    should_return_to_depot: bool = False
    robot_name: str = 'amr-1'
    mode: Literal['internal_simulator', 'external_bridge']


class RobotEventIn(BaseModel):
    robot_name: str = Field(default='amr-1', min_length=1, max_length=64)
    event: Literal['nav_started', 'arrived_pickup', 'arrived_dropoff', 'nav_failed', 'returned_to_depot']
    task_id: int | None = None
    message: str | None = Field(default=None, max_length=512)


class WSEvent(BaseModel):
    type: Literal['task.created', 'task.updated', 'robot.status', 'robot.route', 'toast']
    payload: dict
