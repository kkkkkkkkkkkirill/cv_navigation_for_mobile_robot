"""Smoke tests for the robot telemetry endpoints."""
from __future__ import annotations

import pytest
from httpx import AsyncClient


@pytest.mark.asyncio
async def test_post_and_get_status(client: AsyncClient) -> None:
    body = {
        "name": "R1",
        "state": "moving",
        "battery_pct": 82.5,
        "pos_x": 1.2,
        "pos_y": 3.4,
        "heading_rad": 0.78,
    }
    r = await client.post("/robot/status", json=body)
    assert r.status_code == 200, r.text
    # State was just reported so it shouldn't flip to "offline".
    assert r.json()["state"] == "moving"

    r = await client.get("/robot/status")
    robots = r.json()
    assert len(robots) == 1
    assert robots[0]["name"] == "R1"
    assert robots[0]["battery_pct"] == 82.5


@pytest.mark.asyncio
async def test_upsert_is_idempotent(client: AsyncClient) -> None:
    for pct in (100.0, 90.0, 85.0):
        r = await client.post(
            "/robot/status", json={"name": "R1", "state": "idle", "battery_pct": pct}
        )
        assert r.status_code == 200

    r = await client.get("/robot/status")
    robots = r.json()
    assert len(robots) == 1
    assert robots[0]["battery_pct"] == 85.0
