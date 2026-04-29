
from __future__ import annotations

import enum
from datetime import datetime, timezone
from sqlalchemy import DateTime, Float, ForeignKey, Integer, String, Text
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, relationship


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


class Base(DeclarativeBase):
    pass


class TaskStatus(str, enum.Enum):
    PENDING = "pending"
    HEADING_TO_PICKUP = "heading_to_pickup"
    AWAITING_LOAD = "awaiting_load"
    HEADING_TO_DROPOFF = "heading_to_dropoff"
    AWAITING_UNLOAD = "awaiting_unload"
    DONE = "done"
    FAILED = "failed"
    CANCELLED = "cancelled"


class RobotState(str, enum.Enum):
    IDLE = "idle"
    MOVING_TO_PICKUP = "moving_to_pickup"
    WAITING_FOR_LOAD = "waiting_for_load"
    MOVING_TO_DROPOFF = "moving_to_dropoff"
    WAITING_FOR_UNLOAD = "waiting_for_unload"
    ERROR = "error"
    OFFLINE = "offline"


class Task(Base):
    __tablename__ = "tasks"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    source: Mapped[str] = mapped_column(String(64))
    destination: Mapped[str] = mapped_column(String(64))
    payload: Mapped[str | None] = mapped_column(Text, nullable=True)
    priority: Mapped[int] = mapped_column(Integer, default=5)
    status: Mapped[str] = mapped_column(String(32), default=TaskStatus.PENDING.value, index=True)
    requested_by: Mapped[str | None] = mapped_column(String(64), nullable=True)
    assigned_robot_id: Mapped[int | None] = mapped_column(ForeignKey("robots.id"), nullable=True)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=_utcnow)
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=_utcnow, onupdate=_utcnow)
    started_at: Mapped[datetime | None] = mapped_column(DateTime, nullable=True)
    finished_at: Mapped[datetime | None] = mapped_column(DateTime, nullable=True)
    error_message: Mapped[str | None] = mapped_column(Text, nullable=True)
    robot: Mapped["Robot | None"] = relationship(back_populates="tasks")


class Robot(Base):
    __tablename__ = "robots"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    name: Mapped[str] = mapped_column(String(64), unique=True)
    state: Mapped[str] = mapped_column(String(32), default=RobotState.OFFLINE.value)
    battery_pct: Mapped[float | None] = mapped_column(Float, nullable=True)
    pos_x: Mapped[float | None] = mapped_column(Float, nullable=True)
    pos_y: Mapped[float | None] = mapped_column(Float, nullable=True)
    heading_rad: Mapped[float | None] = mapped_column(Float, nullable=True)
    last_seen_at: Mapped[datetime | None] = mapped_column(DateTime, nullable=True)
    has_cargo: Mapped[int] = mapped_column(Integer, default=0)
    current_task_id: Mapped[int | None] = mapped_column(Integer, nullable=True)
    tasks: Mapped[list[Task]] = relationship(back_populates="robot")
