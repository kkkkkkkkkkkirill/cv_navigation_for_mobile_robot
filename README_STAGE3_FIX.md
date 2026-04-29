# Stage 3 Fixes

This patch fixes two issues observed during stage 3 bringup:

1. `run_task_command_bridge.sh` crashed with `AMENT_TRACE_SETUP_FILES: unbound variable`.
2. `robot_status_bridge.py` caused repeated `503 Service Unavailable` responses.

## What changed

- shell wrappers now source ROS with `set +u`
- SQLite now uses WAL mode and a longer busy timeout
- robot status bridge no longer performs a GET before every POST
- `/robot/status` supports sparse updates and merges omitted fields

## Recommended restart

```bash
pkill -f uvicorn || true
pkill -f robot_status_bridge.py || true
pkill -f task_command_bridge.py || true
rm -f ~/Desktop/amr_stage3_dispatch_bridge_fix/web_app/app/backend/amr.db
```
