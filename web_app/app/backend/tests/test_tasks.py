"""Smoke tests for the task queue REST surface."""
from __future__ import annotations

import pytest
from httpx import AsyncClient


@pytest.mark.asyncio
async def test_healthz(client: AsyncClient) -> None:
    r = await client.get("/healthz")
    assert r.status_code == 200
    assert r.json() == {"status": "ok"}


@pytest.mark.asyncio
async def test_enqueue_and_list(client: AsyncClient) -> None:
    payload = {
        "source": "STATION_A",
        "destination": "STATION_B",
        "payload": "pallet-42",
        "priority": 3,
        "requested_by": "op-7",
    }
    r = await client.post("/tasks", json=payload)
    assert r.status_code == 201, r.text
    created = r.json()
    assert created["status"] == "pending"
    assert created["priority"] == 3

    r = await client.get("/tasks")
    assert r.status_code == 200
    tasks = r.json()
    assert len(tasks) == 1
    assert tasks[0]["id"] == created["id"]


@pytest.mark.asyncio
async def test_priority_ordering(client: AsyncClient) -> None:
    # Insert tasks with varying priorities — lower value = more urgent.
    priorities = [5, 1, 9, 3]
    ids = []
    for p in priorities:
        r = await client.post(
            "/tasks",
            json={"source": "A", "destination": "B", "priority": p},
        )
        ids.append(r.json()["id"])

    r = await client.get("/tasks")
    returned_priorities = [t["priority"] for t in r.json()]
    assert returned_priorities == sorted(priorities)


@pytest.mark.asyncio
async def test_claim_advances_status(client: AsyncClient) -> None:
    # First we need a robot to claim tasks on behalf of.
    await client.post(
        "/robot/status",
        json={"name": "R1", "state": "idle", "battery_pct": 99.0},
    )
    r = await client.get("/robot/status")
    robot_id = r.json()[0]["id"]

    await client.post("/tasks", json={"source": "A", "destination": "B"})

    r = await client.post(f"/tasks/next/claim", params={"robot_id": robot_id})
    assert r.status_code == 200
    claimed = r.json()
    assert claimed["status"] == "assigned"
    assert claimed["assigned_robot_id"] == robot_id


@pytest.mark.asyncio
async def test_lifecycle_transitions(client: AsyncClient) -> None:
    r = await client.post("/tasks", json={"source": "A", "destination": "B"})
    task_id = r.json()["id"]

    for target in ("in_progress", "done"):
        r = await client.patch(
            f"/tasks/{task_id}/status", json={"status": target}
        )
        assert r.status_code == 200, r.text
        assert r.json()["status"] == target

    r = await client.get(f"/tasks/{task_id}")
    body = r.json()
    assert body["status"] == "done"
    assert body["started_at"] is not None
    assert body["finished_at"] is not None


@pytest.mark.asyncio
async def test_cancel_task(client: AsyncClient) -> None:
    r = await client.post("/tasks", json={"source": "A", "destination": "B"})
    task_id = r.json()["id"]

    r = await client.post(f"/tasks/{task_id}/cancel")
    assert r.status_code == 200
    assert r.json()["status"] == "cancelled"


@pytest.mark.asyncio
async def test_404_for_missing_task(client: AsyncClient) -> None:
    r = await client.get("/tasks/9999")
    assert r.status_code == 404
