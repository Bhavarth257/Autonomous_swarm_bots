"""Generate a collapsed-building world plus its ground truth.

Writes, for ``name = rescue_<seed>``:
  <name>.sdf          Gazebo world
  <name>.json         ground truth: victims, decoys, spawns, entrance, explorable area
  <name>_map.pgm/yaml ground-truth occupancy map (nav2 map_server format; blocking = black,
                      climbable = grey, which map_server reads as unknown)
  <name>_preview.png  top-down picture: blocking dark, climbable tan, victims red (visible) /
                      orange (buried), decoys blue, spawns green; hard mode: smoke rooms
                      blue-grey, dark (roofed) rooms grey
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import zlib

import numpy as np

from rescue_world.layout import GRID_RES, generate_layout
from rescue_world.sdf_writer import to_sdf

DEFAULT_OUT = os.path.expanduser('~/.ros/rescue_worlds')


def _write_png(path, rgb):
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[j].tobytes() for j in range(h))

    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data))

    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b'IDAT', zlib.compress(raw, 6)))
        f.write(chunk(b'IEND', b''))


def _preview(layout):
    grid, reach = layout.grid, layout.reachable
    img = np.full(grid.shape + (3,), 255, np.uint8)
    img[reach] = (225, 240, 225)
    img[layout.climbable] = (205, 180, 135)
    img[grid] = (60, 60, 60)
    ox, oy = layout.grid_origin

    def dot(x, y, color, r=0.3):
        i, j = int((x - ox) / GRID_RES), int((y - oy) / GRID_RES)
        k = int(r / GRID_RES)
        yy, xx = np.ogrid[-k:k + 1, -k:k + 1]
        mask = xx * xx + yy * yy <= k * k
        sub = img[max(0, j - k):j + k + 1, max(0, i - k):i + k + 1]
        m = mask[:sub.shape[0], :sub.shape[1]]
        sub[m] = color

    # Hard-mode rooms: smoke tinted grey-blue, dark (roofed) rooms tinted dark.
    cw, rh = layout.width / layout.cols, layout.height / layout.rows
    for e in layout.extras:
        if e['type'] == 'smoke':
            x0, x1, y0, y1 = e['x'] - e['sx'] / 2, e['x'] + e['sx'] / 2, e['y'] - e['sy'] / 2, e['y'] + e['sy'] / 2
            tint = np.array((170, 180, 200))
        else:
            i, j = e['room']
            x0, x1, y0, y1 = i * cw, (i + 1) * cw, j * rh, (j + 1) * rh
            tint = np.array((120, 120, 140))
        sl = (slice(int((y0 - oy) / GRID_RES), int((y1 - oy) / GRID_RES)),
              slice(int((x0 - ox) / GRID_RES), int((x1 - ox) / GRID_RES)))
        region = img[sl]
        free = ~grid[sl]
        region[free] = (region[free] * 0.5 + tint * 0.5).astype(np.uint8)
    for v in layout.victims:
        dot(v.x, v.y, (220, 30, 30) if v.kind == 'visible' else (245, 140, 0))
    for d in layout.decoys:
        dot(d.x, d.y, (40, 90, 230))
    for sx, sy, _ in layout.spawns:
        dot(sx, sy, (30, 150, 30), 0.15)
    return img[::-1]            # image rows go top-down, map y goes up


def generate(seed=0, out_dir=DEFAULT_OUT, name=None, n_robots=3, **kw):
    """Generate (or reuse) the world for ``seed``; returns a dict of output paths."""
    os.makedirs(out_dir, exist_ok=True)
    name = name or f'rescue_{seed}'
    layout = generate_layout(seed=seed, n_robots=n_robots, **kw)
    base = os.path.join(out_dir, name)
    paths = {'sdf': base + '.sdf', 'json': base + '.json', 'map_pgm': base + '_map.pgm',
             'map_yaml': base + '_map.yaml', 'preview': base + '_preview.png'}

    with open(paths['sdf'], 'w') as f:
        f.write(to_sdf(layout))

    ox, oy = layout.grid_origin
    occ = np.where(layout.grid, 0, np.where(layout.climbable, 140, 254)).astype(np.uint8)[::-1]
    with open(paths['map_pgm'], 'wb') as f:
        f.write(f'P5\n{occ.shape[1]} {occ.shape[0]}\n255\n'.encode())
        f.write(occ.tobytes())
    with open(paths['map_yaml'], 'w') as f:
        f.write(f'image: {os.path.basename(paths["map_pgm"])}\nresolution: {GRID_RES}\n'
                f'origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    _write_png(paths['preview'], _preview(layout))

    # Explorable area = free space inside the building the robot can actually reach.
    i0, i1 = int(-ox / GRID_RES), int((layout.width - ox) / GRID_RES)
    j0, j1 = int(-oy / GRID_RES), int((layout.height - oy) / GRID_RES)
    explorable = float(layout.reachable[j0:j1, i0:i1].sum() * GRID_RES ** 2)

    gt = {
        'seed': seed,
        'attempt': layout.attempt,
        'building': {'width': layout.width, 'height': layout.height,
                     'cols': layout.cols, 'rows': layout.rows},
        'entrance': {'x': layout.entrance[0], 'y': layout.entrance[1], 'width': layout.entrance[2]},
        'spawns': [{'name': f'r{k + 1}', 'x': x, 'y': y, 'yaw': yaw}
                   for k, (x, y, yaw) in enumerate(layout.spawns)],
        'victims': [{'id': v.id, 'kind': v.kind, 'posture': v.posture,
                     'x': round(v.x, 3), 'y': round(v.y, 3), 'yaw': round(v.yaw, 3),
                     'room': list(v.room)} for v in layout.victims],
        'decoys': [{'id': d.id, 'x': round(d.x, 3), 'y': round(d.y, 3),
                    'temperature': round(d.temperature, 1)} for d in layout.decoys],
        'doors': [{'x': round(x, 3), 'y': round(y, 3), 'blocked': blocked}
                  for x, y, blocked in layout.doors],
        'explorable_area_m2': round(explorable, 2),
        'difficulty': layout.difficulty,
        'extras': layout.extras,
        'files': paths,
    }
    with open(paths['json'], 'w') as f:
        json.dump(gt, f, indent=2)
    return paths


def generate_from_launch(seed, n_robots, grid=4, difficulty='normal', victims_file=''):
    """What the launch files call: same seed and options -> same world files."""
    return generate(seed=seed, out_dir=DEFAULT_OUT, n_robots=max(n_robots, 3), cols=grid, rows=grid,
                    difficulty=difficulty, custom_victims=load_victims(victims_file))


def load_victims(path):
    """User victim file: a JSON list, or {"victims": [...]} as the dashboard saves it."""
    if not path:
        return None
    with open(os.path.expanduser(path)) as f:
        data = json.load(f)
    return data['victims'] if isinstance(data, dict) else data


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--out-dir', default=DEFAULT_OUT)
    p.add_argument('--name')
    p.add_argument('--cols', type=int, default=4)
    p.add_argument('--rows', type=int, default=4)
    p.add_argument('--room-w', type=float, default=6.0)
    p.add_argument('--room-h', type=float, default=5.0)
    p.add_argument('--visible', type=int, default=4, help='victims in the open')
    p.add_argument('--buried', type=int, default=2, help='victims under debris (thermal only)')
    p.add_argument('--decoys', type=int, default=1, help='hot non-human debris')
    p.add_argument('--robots', type=int, default=3)
    p.add_argument('--difficulty', choices=['normal', 'hard'], default='normal')
    p.add_argument('--victims-file', help='JSON list of {x, y, posture, buried, yaw}: place victims yourself')
    p.add_argument('--victims', choices=['auto', 'randy', 'primitive'], default='auto',
                   help='victim models: Rescue Randy meshes (needs the Fuel download) or primitives')
    a = p.parse_args(argv)
    paths = generate(seed=a.seed, out_dir=a.out_dir, name=a.name, n_robots=a.robots,
                     cols=a.cols, rows=a.rows, room_w=a.room_w, room_h=a.room_h,
                     n_visible=a.visible, n_buried=a.buried, n_decoys=a.decoys,
                     victim_style=a.victims, difficulty=a.difficulty,
                     custom_victims=load_victims(a.victims_file))
    with open(paths['json']) as f:
        gt = json.load(f)
    print(f"world:   {paths['sdf']}")
    print(f"preview: {paths['preview']}")
    print(f"victims: {sum(v['kind'] == 'visible' for v in gt['victims'])} visible, "
          f"{sum(v['kind'] == 'buried' for v in gt['victims'])} buried, {len(gt['decoys'])} decoys; "
          f"explorable {gt['explorable_area_m2']} m^2 (layout attempt {gt['attempt']})")


if __name__ == '__main__':
    main()
