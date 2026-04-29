from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
from typing import AsyncIterator

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .config import settings
from .db import init_db
from .routers import robot, tasks, ws
from .services.simulator import simulator_loop


@asynccontextmanager
async def lifespan(_: FastAPI) -> AsyncIterator[None]:
    await init_db()
    stop = asyncio.Event()
    task: asyncio.Task | None = None
    if settings.use_internal_simulator:
        task = asyncio.create_task(simulator_loop(stop))
    try:
        yield
    finally:
        stop.set()
        if task is not None:
            task.cancel()
            try:
                await task
            except Exception:
                pass


app = FastAPI(title=settings.app_name, version='0.3.0', lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=['*'], allow_credentials=False, allow_methods=['*'], allow_headers=['*'])
app.include_router(tasks.router)
app.include_router(robot.router)
app.include_router(ws.router)


@app.get('/healthz', tags=['meta'])
async def healthz() -> dict[str, str | bool]:
    return {'status': 'ok', 'use_internal_simulator': settings.use_internal_simulator}


_PWA_DIR = Path(__file__).resolve().parents[2] / 'pwa'
if _PWA_DIR.is_dir():
    assets = _PWA_DIR / 'assets'
    if assets.is_dir():
        app.mount('/assets', StaticFiles(directory=assets), name='assets')
    app.mount('/', StaticFiles(directory=_PWA_DIR, html=True), name='pwa')
