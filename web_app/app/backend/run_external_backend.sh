#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
cd "$SCRIPT_DIR"

if [ ! -d .venv ]; then
  python3 -m venv .venv
fi

source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt

export AMR_USE_INTERNAL_SIMULATOR=false
export AMR_WAREHOUSE_MAP_PATH="${AMR_WAREHOUSE_MAP_PATH:-$ROOT_DIR/shared/warehouse_map.yaml}"
uvicorn app.main:app --host 0.0.0.0 --port 8010
