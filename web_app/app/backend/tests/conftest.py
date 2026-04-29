"""Shared pytest fixtures for the backend test suite.

Each test runs against an ephemeral on-disk SQLite DB so schema state is
shared across connections (``:memory:`` under ``NullPool`` would allocate a
fresh DB per connection and drop our tables). The file is created in a
temp directory and deleted at session end.
"""
from __future__ import annotations

import os
import tempfile
from typing import AsyncIterator

import pytest
import pytest_asyncio

# --- Point the app at a temp sqlite file BEFORE importing the app --------
_tmpdir = tempfile.mkdtemp(prefix="amr-tests-")
_db_path = os.path.join(_tmpdir, "amr_test.db")
os.environ["AMR_DATABASE_URL"] = f"sqlite+aiosqlite:///{_db_path}"

from httpx import ASGITransport, AsyncClient  # noqa: E402

from app import db as db_module  # noqa: E402
from app.main import app  # noqa: E402
from app.models import Base  # noqa: E402


@pytest_asyncio.fixture
async def client() -> AsyncIterator[AsyncClient]:
    # Rebuild the schema before every test to keep them independent.
    async with db_module.engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)
        await conn.run_sync(Base.metadata.create_all)

    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as c:
        yield c


def pytest_sessionfinish(session, exitstatus) -> None:  # noqa: ARG001
    """Remove the temp database file once the test run is over."""
    try:
        os.remove(_db_path)
    except OSError:
        pass
    try:
        os.rmdir(_tmpdir)
    except OSError:
        pass
