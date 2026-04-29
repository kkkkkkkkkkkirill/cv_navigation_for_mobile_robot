#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path
import yaml


def fill_rect(img, x, y, w, h, xmin, ymin, resolution, value):
    height = len(img)
    width = len(img[0])
    x0 = int(math.floor((x - xmin) / resolution))
    x1 = int(math.ceil((x + w - xmin) / resolution))
    y0 = int(math.floor((y - ymin) / resolution))
    y1 = int(math.ceil((y + h - ymin) / resolution))
    r0 = height - y1
    r1 = height - y0
    r0 = max(0, min(height, r0))
    r1 = max(0, min(height, r1))
    c0 = max(0, min(width, x0))
    c1 = max(0, min(width, x1))
    if r1 > r0 and c1 > c0:
        for r in range(r0, r1):
            row = img[r]
            for c in range(c0, c1):
                row[c] = value


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--warehouse-map', required=True)
    ap.add_argument('--output-dir', required=True)
    ap.add_argument('--resolution', type=float, default=0.05)
    args = ap.parse_args()

    wh = yaml.safe_load(Path(args.warehouse_map).read_text(encoding='utf-8'))['warehouse']
    bounds = wh['bounds']
    xmin, xmax = float(bounds['x_min']), float(bounds['x_max'])
    ymin, ymax = float(bounds['y_min']), float(bounds['y_max'])
    resolution = args.resolution
    width = int(round((xmax - xmin) / resolution))
    height = int(round((ymax - ymin) / resolution))
    img = [[205 for _ in range(width)] for _ in range(height)]

    for fr in wh.get('floor_regions', []):
        fill_rect(img, float(fr['x']), float(fr['y']), float(fr['w']), float(fr['h']), xmin, ymin, resolution, 254)
    for ob in wh.get('obstacles', []):
        fill_rect(img, float(ob['x']), float(ob['y']), float(ob['w']), float(ob['h']), xmin, ymin, resolution, 0)

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = out_dir / 'warehouse_nav2_map.pgm'
    yaml_path = out_dir / 'warehouse_nav2_map.yaml'

    with pgm_path.open('wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(bytes(v for row in img for v in row))

    yaml_path.write_text(yaml.safe_dump({
        'image': 'warehouse_nav2_map.pgm',
        'mode': 'trinary',
        'resolution': resolution,
        'origin': [xmin, ymin, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }, sort_keys=False), encoding='utf-8')

    print(f'Wrote {pgm_path}')
    print(f'Wrote {yaml_path}')


if __name__ == '__main__':
    main()
