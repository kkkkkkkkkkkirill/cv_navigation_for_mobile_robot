from __future__ import annotations

from pathlib import Path
from typing import Any
import math
import os
import yaml

from ..config import settings

MAP_WIDTH = 1000
MAP_HEIGHT = 640
PADDING_X = 80
PADDING_Y = 50


def _candidate_paths() -> list[Path]:
    here = Path(__file__).resolve()
    app_root = here.parents[3]
    candidates: list[Path] = []
    if settings.warehouse_map_path:
        candidates.append(Path(settings.warehouse_map_path).expanduser())
    env_path = os.environ.get('AMR_WAREHOUSE_MAP_PATH')
    if env_path:
        candidates.append(Path(env_path).expanduser())
    candidates.extend([
        app_root / 'ros2_bridge' / 'config' / 'warehouse_map.yaml',
        app_root / 'backend' / 'app' / 'warehouse_map.yaml',
    ])
    return candidates


def _load_raw() -> dict[str, Any]:
    for path in _candidate_paths():
        if path.is_file():
            with path.open('r', encoding='utf-8') as f:
                return yaml.safe_load(f)['warehouse']
    raise FileNotFoundError('warehouse_map.yaml not found in configured search paths')


RAW = _load_raw()
BOUNDS = RAW['bounds']
WORLD_W = float(BOUNDS['x_max']) - float(BOUNDS['x_min'])
WORLD_H = float(BOUNDS['y_max']) - float(BOUNDS['y_min'])
SCALE = min((MAP_WIDTH - 2 * PADDING_X) / WORLD_W, (MAP_HEIGHT - 2 * PADDING_Y) / WORLD_H)
DRAW_W = WORLD_W * SCALE
DRAW_H = WORLD_H * SCALE
OFFSET_X = (MAP_WIDTH - DRAW_W) / 2.0
OFFSET_Y = (MAP_HEIGHT - DRAW_H) / 2.0


def world_to_ui(x: float, y: float) -> dict[str, float]:
    px = OFFSET_X + (x - float(BOUNDS['x_min'])) * SCALE
    py = OFFSET_Y + (float(BOUNDS['y_max']) - y) * SCALE
    return {'x': round(px, 2), 'y': round(py, 2)}


def ui_to_world(x: float, y: float) -> dict[str, float]:
    wx = float(BOUNDS['x_min']) + (x - OFFSET_X) / SCALE
    wy = float(BOUNDS['y_max']) - (y - OFFSET_Y) / SCALE
    return {'x': wx, 'y': wy}


def world_rect_to_ui(x: float, y: float, w: float, h: float) -> dict[str, float]:
    p0 = world_to_ui(x, y)
    p1 = world_to_ui(x + w, y + h)
    return {
        'x': min(p0['x'], p1['x']),
        'y': min(p0['y'], p1['y']),
        'w': abs(p1['x'] - p0['x']),
        'h': abs(p1['y'] - p0['y']),
    }


def _nearest_node_world(station_x: float, station_y: float, nodes: dict[str, dict[str, float]]) -> str:
    return min(nodes, key=lambda name: math.hypot(nodes[name]['world_x'] - station_x, nodes[name]['world_y'] - station_y))


def _explicit_rects(items: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for item in items:
        x = float(item['x'])
        y = float(item['y'])
        w = float(item['w'])
        h = float(item['h'])
        ui = world_rect_to_ui(x, y, w, h)
        out.append({
            'id': item.get('id'),
            'kind': item.get('kind', 'box'),
            'source_model': item.get('source_model'),
            'world_x': x,
            'world_y': y,
            'world_w': w,
            'world_h': h,
            **ui,
        })
    return out


def _fallback_obstacles() -> list[dict[str, Any]]:
    aisle_x = sorted({float(v['x']) for v in RAW['aisles'].values()})
    row_y = sorted({float(v['y']) for k, v in RAW['nodes'].items() if '_row' in k})
    obstacles: list[dict[str, Any]] = []
    if len(aisle_x) >= 2 and len(row_y) >= 2:
        xmids = [(aisle_x[i] + aisle_x[i + 1]) / 2.0 for i in range(len(aisle_x) - 1)]
        ymids = [(row_y[i] + row_y[i + 1]) / 2.0 for i in range(len(row_y) - 1)]
        cell_w = (aisle_x[1] - aisle_x[0]) * 0.42
        cell_h = (row_y[1] - row_y[0]) * 0.55
        for i, xm in enumerate(xmids):
            for j, ym in enumerate(ymids):
                ui = world_rect_to_ui(xm - cell_w / 2, ym - cell_h / 2, cell_w, cell_h)
                obstacles.append({
                    'id': f'fallback_{i}_{j}',
                    'kind': 'rack',
                    'world_x': xm - cell_w / 2,
                    'world_y': ym - cell_h / 2,
                    'world_w': cell_w,
                    'world_h': cell_h,
                    **ui,
                })
    return obstacles


def map_config() -> dict[str, Any]:
    stations = {}
    for key, station in RAW['stations'].items():
        ui = world_to_ui(float(station['x']), float(station['y']))
        stations[key.upper()] = {
            'label': station.get('label', key),
            'world_x': float(station['x']),
            'world_y': float(station['y']),
            'yaw': float(station.get('yaw', 0.0)),
            'x': ui['x'],
            'y': ui['y'],
        }

    nodes = {}
    for key, node in RAW['nodes'].items():
        ui = world_to_ui(float(node['x']), float(node['y']))
        nodes[key] = {
            'world_x': float(node['x']),
            'world_y': float(node['y']),
            'x': ui['x'],
            'y': ui['y'],
        }

    roads = [{'a': a, 'b': b} for a, b in RAW['lanes']]

    explicit_links = RAW.get('station_links', {})
    station_links = {
        name.upper(): explicit_links.get(name) or explicit_links.get(name.lower()) or _nearest_node_world(s['world_x'], s['world_y'], nodes)
        for name, s in stations.items()
    }

    floor_regions = _explicit_rects(RAW.get('floor_regions', []))
    obstacles = _explicit_rects(RAW.get('obstacles', [])) if RAW.get('obstacles') else _fallback_obstacles()

    return {
        'width': MAP_WIDTH,
        'height': MAP_HEIGHT,
        'transform': {
            'x_min': float(BOUNDS['x_min']),
            'x_max': float(BOUNDS['x_max']),
            'y_min': float(BOUNDS['y_min']),
            'y_max': float(BOUNDS['y_max']),
            'scale': SCALE,
            'offset_x': OFFSET_X,
            'offset_y': OFFSET_Y,
        },
        'bounds': RAW['bounds'],
        'stations': stations,
        'nodes': nodes,
        'roads': roads,
        'station_links': station_links,
        'floor_regions': floor_regions,
        'obstacles': obstacles,
        'control_mode': 'internal_simulator' if settings.use_internal_simulator else 'external_bridge',
    }
