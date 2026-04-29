#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

pkill -f uvicorn 2>/dev/null || true
pkill -f robot_status_bridge.py 2>/dev/null || true
pkill -f task_nav2_bridge.py 2>/dev/null || true
pkill -f task_command_bridge.py 2>/dev/null || true

rm -f "$ROOT_DIR/web_app/app/backend/amr.db"
rm -f "$ROOT_DIR/web_app/app/backend/amr.db-shm"
rm -f "$ROOT_DIR/web_app/app/backend/amr.db-wal"

echo "Runtime state cleared."
