#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
source "$SCRIPT_DIR/.venv/bin/activate"
export AMR_USE_INTERNAL_SIMULATOR=false
export AMR_WAREHOUSE_MAP_PATH="$ROOT_DIR/shared/warehouse_map.yaml"
exec uvicorn app.main:app --host 0.0.0.0 --port 8010
