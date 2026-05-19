from __future__ import annotations

import asyncio

from fastapi import APIRouter, Depends, status
from pydantic import BaseModel
from sqlalchemy.ext.asyncio import AsyncSession

from ..db import get_session
from ..schemas import RobotDispatchOut, RobotEventIn, RobotStatusIn, RobotStatusOut
from ..services import robot_state
from ..services.external_bridge import get_dispatch, handle_event
from ..services.simulator import map_config, runtime, _broadcast_route
from ..services.ws_manager import manager


class ObstacleEventIn(BaseModel):
    robot_name: str = 'amr-1'
    blocked: bool
    source: str | None = None
    distance_m: float | None = None
    detected_class: str | None = None


router = APIRouter(prefix="/robot", tags=["robot"])


@router.get("/status", response_model=list[RobotStatusOut])
async def get_status(session: AsyncSession = Depends(get_session)) -> list[RobotStatusOut]:
    robots = await robot_state.list_robots(session)

    result: list[RobotStatusOut] = []
    for r in robots:
        result.append(
            RobotStatusOut.model_validate(
                {
                    "id": r.id,
                    "name": r.name,
                    "state": r.state,
                    "battery_pct": r.battery_pct,
                    "pos_x": r.pos_x,
                    "pos_y": r.pos_y,
                    "heading_rad": r.heading_rad,
                    "has_cargo": bool(r.has_cargo),
                    "current_task_id": r.current_task_id,
                    "last_seen_at": r.last_seen_at,
                    "motion_mode": robot_state.get_motion_mode(r.name),
                }
            )
        )
    return result


@router.post("/status", response_model=RobotStatusOut, status_code=status.HTTP_200_OK)
async def post_status(body: RobotStatusIn, session: AsyncSession = Depends(get_session)) -> RobotStatusOut:
    robot = await robot_state.upsert_status(session, body)
    out = RobotStatusOut.model_validate(
        {
            "id": robot.id,
            "name": robot.name,
            "state": robot.state,
            "battery_pct": robot.battery_pct,
            "pos_x": robot.pos_x,
            "pos_y": robot.pos_y,
            "heading_rad": robot.heading_rad,
            "has_cargo": bool(robot.has_cargo),
            "current_task_id": robot.current_task_id,
            "last_seen_at": robot.last_seen_at,
            "motion_mode": robot_state.get_motion_mode(robot.name),
        }
    )
    try:
        asyncio.create_task(manager.broadcast({"type": "robot.status", "payload": out.model_dump(mode="json")}))
    except Exception:
        pass
    return out


@router.get("/dispatch", response_model=RobotDispatchOut)
async def get_robot_dispatch(session: AsyncSession = Depends(get_session)) -> RobotDispatchOut:
    return await get_dispatch(session)


@router.post("/event")
async def post_robot_event(body: RobotEventIn, session: AsyncSession = Depends(get_session)) -> dict:
    return await handle_event(session, body)


@router.post("/obstacle")
async def post_obstacle(body: ObstacleEventIn) -> dict:
    payload = {
        "robot_name": body.robot_name,
        "blocked": body.blocked,
        "source": body.source,
        "distance_m": body.distance_m,
        "detected_class": body.detected_class,
    }
    try:
        await manager.broadcast({"type": "obstacle", "payload": payload})
    except Exception:
        pass
    return {"ok": True}


@router.get("/map", tags=["robot"])
async def get_map() -> dict:
    data = map_config()
    data["active_route"] = {
        "task_id": runtime.route_task_id,
        "points": runtime.route_points,
        "phase": runtime.phase,
        "motion_mode": runtime.motion_mode,
    }
    return data


@router.post("/route", status_code=status.HTTP_200_OK)
async def post_route(body: dict) -> dict:
    """Receive BFS-built route from cv_navigator and broadcast to all clients."""
    points = body.get("points") or []
    phase = body.get("phase") or "idle"

    runtime.route_points = points
    runtime.phase = phase
    if phase == "idle":
        runtime.route_task_id = None
        runtime.motion_mode = "idle"
    else:
        runtime.motion_mode = "driving"

    try:
        await _broadcast_route(
            runtime.route_task_id,
            runtime.route_points,
            getattr(runtime, "target_station", None),
            runtime.phase,
        )
    except Exception:
        pass

    return {
        "task_id": runtime.route_task_id,
        "points": runtime.route_points,
        "phase": runtime.phase,
        "motion_mode": runtime.motion_mode,
    }


@router.get("/map_source", tags=["robot"])
async def get_map_source() -> dict:
    from ..config import settings
    import os
    return {
        "env": os.environ.get("AMR_WAREHOUSE_MAP_PATH"),
        "configured": settings.warehouse_map_path,
    }
