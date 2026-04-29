"""WebSocket bus.

The PWA opens one connection to ``/ws`` and receives every domain event
(``task.created``, ``task.updated``, ``robot.status``). The server ignores
incoming messages — the channel is broadcast-only for now.
"""
from __future__ import annotations

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from ..services.ws_manager import manager

router = APIRouter(tags=["ws"])


@router.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    await manager.connect(ws)
    try:
        while True:
            # Drain the socket so that keepalive/pong frames flow. We use
            # ``receive()`` rather than ``receive_text()`` so that a client
            # sending binary frames does not blow up the whole connection.
            msg = await ws.receive()
            if msg.get("type") == "websocket.disconnect":
                break
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        await manager.disconnect(ws)
