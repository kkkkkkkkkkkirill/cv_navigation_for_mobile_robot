"""Application settings loaded from environment variables or .env."""
from __future__ import annotations

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Runtime configuration for the backend.

    Values can be overridden by environment variables (e.g. ``AMR_DATABASE_URL``)
    or a local ``.env`` file next to the backend root.
    """

    app_name: str = "AMR Navigation Backend"
    debug: bool = False

    # SQLite lives alongside the backend process by default.
    database_url: str = "sqlite+aiosqlite:///./amr.db"

    # Heartbeat threshold after which the robot is marked offline (seconds).
    robot_offline_after_s: float = 15.0

    # Backend can either run its demo simulator or wait for an external bridge.
    use_internal_simulator: bool = True

    # Optional explicit path to the shared warehouse_map.yaml.
    warehouse_map_path: str | None = None

    # Host/port are read by uvicorn via CLI; kept here for docs/scripts.
    host: str = "0.0.0.0"
    port: int = 8000

    model_config = SettingsConfigDict(
        env_file=".env",
        env_prefix="AMR_",
        extra="ignore",
    )


settings = Settings()
