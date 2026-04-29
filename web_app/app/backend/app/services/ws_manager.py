"""Very small WebSocket broadcast manager.

Holds a set of active connections and fans out JSON-serialisable events to
all of them. Failures are silently dropped — a disconnected client is simply
pruned. This keeps publishers (REST handlers, the robot telemetry endpoint)
decoupled from the transport.
"""
from __future__ import annotations

import asyncio
from typing import Any

from fastapi import WebSocket


class ConnectionManager:
    def __init__(self) -> None:
        self._connections: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self._connections.add(ws)

    async def disconnect(self, ws: WebSocket) -> None:
        async with self._lock:
            self._connections.discard(ws)

    async def broadcast(self, event: dict[str, Any]) -> None:
        # Copy the set before iterating so a disconnect during send does not
        # mutate us mid-loop.
        async with self._lock:
            targets = list(self._connections)

        dead: list[WebSocket] = []
        for ws in targets:
            try:
                await ws.send_json(event)
            except Exception:
                dead.append(ws)

        if dead:
            async with self._lock:
                for ws in dead:
                    self._connections.discard(ws)


# Module-level singleton: FastAPI handlers import this directly.
manager = ConnectionManager()
