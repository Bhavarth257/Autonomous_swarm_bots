"""Procedural collapsed-building layout.

Pure geometry, no ROS or SDF: ``generate_layout`` returns a ``Layout`` that
``sdf_writer`` turns into a Gazebo world and ``maps`` turns into ground truth.

Frame: the building spans [0, width] x [0, height] metres, the entrance is a
gap in the south wall (y = 0) and the robots spawn just outside it.

Difficulty "hard" adds narrower doors, more rubble, stepped debris, smoke (blinds the
camera, not the thermal sensor) and rooms under a collapsed roof (dark).
Victims can be given by the user (custom_victims) instead of being placed at random.

Obstacles come in two classes for the walking hexapods:
  climbable  low blocks, thin beams, gravel, ramped rubble mounds (<= CLIMB_MAX_H steps)
  blocking   walls, tall rubble piles, leaning slabs, tall beams, victims, buried-victim piles
"""
from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from rescue_world import humans
from rescue_world.primitives import Footprint, Part

WALL_T = 0.2           # wall thickness
WALL_H = 2.2           # intact wall height
DOOR_W = 1.0
DOOR_W_HARD = 0.85
ENTRANCE_W = 1.4
ROBOT_RADIUS = 0.30    # used to inflate blocking obstacles for the reachability check
GRID_RES = 0.05
CLIMB_MAX_H = 0.12     # tallest step generated as climbable (default; climb_max per robot)
BLOCK_MIN_H = 0.28     # shortest obstacle generated as blocking

CONCRETE = (0.58, 0.57, 0.55)
RUBBLE_COLORS = [(0.50, 0.47, 0.43), (0.62, 0.58, 0.52), (0.45, 0.40, 0.36),
                 (0.66, 0.64, 0.60), (0.55, 0.42, 0.33)]
BEAM_COLOR = (0.40, 0.40, 0.42)
AMBIENT_K = 293.0


@dataclass
class Victim:
    id: str
    kind: str                  # visible | buried
    posture: str               # lying | sitting
    x: float
    y: float
    yaw: float
    room: tuple
    parts: list = field(default_factory=list)   # in the victim's model frame


@dataclass
class Decoy:
    """Hot debris (smouldering material): thermal false-positive bait."""
    id: str
    x: float
    y: float
    temperature: float


@dataclass
class Layout:
    seed: int
    attempt: int
    width: float
    height: float
    cols: int
    rows: int
    parts: list
    footprints: list
    victims: list
    decoys: list
    entrance: tuple            # x, y, width
    spawns: list               # (x, y, yaw) per robot
    doors: list                # (x, y, blocked)
    difficulty: str = 'normal'
    extras: list = field(default_factory=list)   # smoke volumes, dark rooms (for the SDF writer)
    grid: np.ndarray | None = None        # blocking obstacles
    climbable: np.ndarray | None = None   # climbable obstacles (not blocking)
    reachable: np.ndarray | None = None   # inflated free space reachable from spawn
    grid_origin: tuple = (0.0, 0.0)


class LayoutError(RuntimeError):
    pass


def _seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class _Builder:
    def __init__(self, rng: random.Random, cols, rows, room_w, room_h, victim_style='primitive',
                 hard=False, climb_max=CLIMB_MAX_H):
        self.rng = rng
        self.climb_max = climb_max
        self.victim_style = victim_style
        self.hard = hard
        self.extras = []
        self.cols, self.rows = cols, rows
        self.cw, self.rh = room_w, room_h
        self.width, self.height = cols * room_w, rows * room_h
        self.parts: list[Part] = []
        self.footprints: list[Footprint] = []
        self.walls: list[tuple] = []     # solid wall segments, for placement clearance
        self.blobs: list[tuple] = []     # (x, y, r) occupied discs, for placement clearance
        self.passages: list[tuple] = []  # (x, y) door/breach/entrance centres to keep clear
        self.doors: list[list] = []      # [x, y, blocked]
        self._n = 0

    def _name(self, prefix):
        self._n += 1
        return f'{prefix}_{self._n}'

    # ---- primitives -------------------------------------------------------
    def box(self, prefix, size, pose, color, temperature=None, footprint=True, climbable=False):
        sx, sy, sz = size
        x, y, z, roll, pitch, yaw = pose
        self.parts.append(Part(self._name(prefix), 'box', size, pose, color, temperature))
        if footprint:
            # Grow the floor rectangle to cover small roll/pitch tilts.
            lx = sx + abs(math.sin(pitch)) * sz
            ly = sy + abs(math.sin(roll)) * sz
            self.footprints.append(Footprint(x, y, yaw, lx, ly, climbable))

    def rubble_color(self):
        base = self.rng.choice(RUBBLE_COLORS)
        k = self.rng.uniform(0.85, 1.12)
        return tuple(min(1.0, c * k) for c in base)

    # ---- placement --------------------------------------------------------
    def room_bounds(self, i, j, margin=0.0):
        h = WALL_T / 2 + margin
        return (i * self.cw + h, (i + 1) * self.cw - h, j * self.rh + h, (j + 1) * self.rh - h)

    def is_free(self, x, y, r, clear=0.15, passage_clear=1.0):
        for bx, by, br in self.blobs:
            if math.hypot(x - bx, y - by) < r + br + clear:
                return False
        for (ax, ay, bx, by) in self.walls:
            if _seg_dist(x, y, ax, ay, bx, by) < r + WALL_T / 2 + clear:
                return False
        for px, py in self.passages:
            if math.hypot(x - px, y - py) < r + passage_clear:
                return False
        return True

    def sample_free(self, room, r, tries=60, **kw):
        x0, x1, y0, y1 = self.room_bounds(*room, margin=r)
        if x0 >= x1 or y0 >= y1:
            return None
        for _ in range(tries):
            x, y = self.rng.uniform(x0, x1), self.rng.uniform(y0, y1)
            if self.is_free(x, y, r, **kw):
                return x, y
        return None

    # ---- structure --------------------------------------------------------
    def wall(self, ax, ay, bx, by, gaps, damaged):
        """Wall from a to b with gaps [(t_centre, width)], built from chunks of varying height."""
        L = math.hypot(bx - ax, by - ay)
        ux, uy = (bx - ax) / L, (by - ay) / L
        yaw = math.atan2(uy, ux)
        solid, t = [], 0.0
        for c, w in sorted(gaps):
            if c - w / 2 > t:
                solid.append((t, c - w / 2))
            t = c + w / 2
        if t < L:
            solid.append((t, L))
        for s0, s1 in solid:
            self.walls.append((ax + ux * s0, ay + uy * s0, ax + ux * s1, ay + uy * s1))
            s = s0
            while s < s1 - 1e-6:
                e = min(s1, s + self.rng.uniform(0.8, 2.0))
                if s1 - e < 0.3:
                    e = s1
                h = WALL_H if not damaged or self.rng.random() < 0.4 else self.rng.uniform(0.6, 1.8)
                # Only the building's corners get extended, so door gaps keep their width.
                s_ext = s - (WALL_T / 2 if s == 0.0 else 0.0)
                e_ext = e + (WALL_T / 2 if e == L else 0.0)
                m = (s_ext + e_ext) / 2
                shade = self.rng.uniform(0.9, 1.05) * (0.9 if h < WALL_H else 1.0)
                self.box('wall', (e_ext - s_ext, WALL_T, h),
                         (ax + ux * m, ay + uy * m, h / 2, 0, 0, yaw),
                         tuple(c * shade for c in CONCRETE))
                s = e

    def rubble_pile(self, x, y, radius, n, max_h=0.6):
        for _ in range(n):
            a, d = self.rng.uniform(0, 2 * math.pi), self.rng.uniform(0, radius)
            sx, sy = self.rng.uniform(0.25, 0.6), self.rng.uniform(0.25, 0.6)
            sz = self.rng.uniform(BLOCK_MIN_H, max_h)
            self.box('rubble', (sx, sy, sz),
                     (x + d * math.cos(a), y + d * math.sin(a), sz / 2,
                      self.rng.uniform(-0.25, 0.25), self.rng.uniform(-0.25, 0.25),
                      self.rng.uniform(-math.pi, math.pi)),
                     self.rubble_color())
        self.blobs.append((x, y, radius + 0.35))

    def leaning_slab(self, room):
        """Slab fallen against one wall of the room, bottom edge out in the room."""
        x0, x1, y0, y1 = self.room_bounds(*room)
        side = self.rng.choice('nsew')
        length, h, t = self.rng.uniform(1.4, 2.4), self.rng.uniform(1.4, 1.9), 0.08
        lean = self.rng.uniform(math.radians(18), math.radians(32))
        out = h * math.sin(lean)                 # how far the bottom edge sits from the wall
        if side in 'ns':
            span = (x0 + length / 2 + 0.3, x1 - length / 2 - 0.3)
            if span[0] >= span[1]:
                return
            along = self.rng.uniform(*span)
            wy, ny = (y1, -1.0) if side == 'n' else (y0, 1.0)
            cx, cy, nx = along, wy + ny * out / 2, 0.0
        else:
            span = (y0 + length / 2 + 0.3, y1 - length / 2 - 0.3)
            if span[0] >= span[1]:
                return
            along = self.rng.uniform(*span)
            wx, nx = (x1, -1.0) if side == 'e' else (x0, 1.0)
            cx, cy, ny = wx + nx * out / 2, along, 0.0
        if not self.is_free(cx, cy, max(length, out) / 2, clear=0.0, passage_clear=0.9):
            return
        # Local +y must point into the room, then roll tips the top back onto the wall.
        yaw = math.atan2(-nx, ny)
        z = (h / 2) * math.cos(lean)
        self.parts.append(Part(self._name('slab'), 'box', (length, t, h),
                               (cx, cy, z, lean, 0.0, yaw), CONCRETE))
        self.footprints.append(Footprint(cx, cy, yaw, length, out + t))
        self.blobs.append((cx, cy, max(length, out) / 2))

    def fallen_beam(self, room, thin=False):
        """Thick beams block; thin ones (a fallen joist) can be climbed."""
        length = self.rng.uniform(1.2, 2.4) if thin else self.rng.uniform(1.5, 3.0)
        # Thin beams may lie across a doorway: the robot climbs over them.
        p = self.sample_free(room, length / 2, passage_clear=0.0 if thin else 0.9)
        if p is None:
            return
        yaw = self.rng.uniform(-math.pi, math.pi)
        if thin:
            h = self.rng.uniform(min(0.09, self.climb_max - 0.01), self.climb_max)
            self.box('joist', (length, 0.15, h), (p[0], p[1], h / 2, 0, 0, yaw),
                     (0.45, 0.36, 0.26), climbable=True)
        else:
            self.box('beam', (length, 0.3, 0.3), (p[0], p[1], 0.15, 0, 0, yaw), BEAM_COLOR)
        self.blobs.append((p[0], p[1], length / 2))

    def low_debris(self, room):
        """Loose low blocks: climbable."""
        p = self.sample_free(room, 0.35, clear=0.25, passage_clear=0.6)
        if p is None:
            return
        h = self.rng.uniform(0.05, self.climb_max)
        self.box('block', (self.rng.uniform(0.3, 0.6), self.rng.uniform(0.3, 0.6), h),
                 (p[0], p[1], h / 2, 0, 0, self.rng.uniform(-math.pi, math.pi)),
                 self.rubble_color(), climbable=True)
        self.blobs.append((p[0], p[1], 0.35))

    def gravel(self, room):
        """A field of small rubble: rough, climbable ground."""
        r = self.rng.uniform(0.6, 1.0)
        p = self.sample_free(room, r, clear=0.1, passage_clear=0.3)
        if p is None:
            return
        for _ in range(self.rng.randint(20, 35)):
            a, d = self.rng.uniform(0, 2 * math.pi), r * math.sqrt(self.rng.random())
            size, h = self.rng.uniform(0.06, 0.15), self.rng.uniform(0.03, 0.07)
            self.box('gravel', (size, size * self.rng.uniform(0.6, 1.0), h),
                     (p[0] + d * math.cos(a), p[1] + d * math.sin(a), h / 2, 0, 0,
                      self.rng.uniform(-math.pi, math.pi)),
                     self.rubble_color(), climbable=True)
        self.blobs.append((p[0], p[1], r))

    def mound(self, room):
        """Rubble mound with a flat top and gentle ramps on two sides: climbable, but its
        un-ramped sides are drops a traversability map has to notice."""
        h = self.rng.uniform(0.18, 0.24)
        top, width = self.rng.uniform(0.8, 1.2), self.rng.uniform(1.0, 1.3)
        run = h / math.tan(math.radians(self.rng.uniform(13, 17)))
        half = top / 2 + run
        p = self.sample_free(room, half, passage_clear=0.9)
        if p is None:
            return
        yaw = self.rng.uniform(-math.pi, math.pi)
        c, s = math.cos(yaw), math.sin(yaw)
        color = self.rubble_color()
        self.box('mound', (top, width, h), (p[0], p[1], h / 2, 0, 0, yaw), color, climbable=True)
        slope = math.atan2(h, run)
        t = 0.04
        for side in (1, -1):
            d = side * (top / 2 + run / 2)
            self.box('ramp', (math.hypot(run, h) + 0.04, width, t),
                     (p[0] + d * c, p[1] + d * s, h / 2 - t / 2, 0, side * slope, yaw),
                     color, footprint=False)
        self.footprints.append(Footprint(p[0], p[1], yaw, top + 2 * run, width, True))
        self.blobs.append((p[0], p[1], half))

    # ---- victims ----------------------------------------------------------
    def place_victim(self, vid, kind, room):
        rng = self.rng
        posture = 'lying' if kind == 'buried' or rng.random() < 0.6 else 'sitting'
        if posture == 'sitting':
            spot = self._wall_spot(room)
            if spot is None:
                posture = 'lying'
        if posture == 'lying':
            p = self.sample_free(room, 1.25 if kind == 'buried' else 0.95, passage_clear=0.8)
            if p is None:
                return None
            x, y, yaw = p[0], p[1], rng.uniform(-math.pi, math.pi)
        else:
            x, y, yaw = spot
        v = Victim(vid, kind, posture, x, y, yaw, room,
                   humans.build(posture, rng, warm=True, style=self.victim_style))
        c, s = math.cos(yaw), math.sin(yaw)
        if posture == 'lying':
            # Body axis is local x: head reaches +0.66, feet -1.1.
            width = 1.4 if self.victim_style == 'randy' else 0.6   # Randy lies with arms out
            self.footprints.append(Footprint(x - 0.22 * c, y - 0.22 * s, yaw, 1.8, width))
            self.blobs.append((x - 0.22 * c, y - 0.22 * s, 0.95))
        else:
            # Back against the wall, legs along local +x to ~1.0.
            self.footprints.append(Footprint(x + 0.43 * c, y + 0.43 * s, yaw, 1.2, 0.6))
            self.blobs.append((x + 0.43 * c, y + 0.43 * s, 0.65))
        if kind == 'buried':
            self._bury(v)
        return v

    def place_victim_at(self, vid, kind, posture, x, y, yaw):
        """A user-placed victim (no free-space search: the user decides)."""
        room = (min(self.cols - 1, max(0, int(x // self.cw))), min(self.rows - 1, max(0, int(y // self.rh))))
        v = Victim(vid, kind, posture, x, y, yaw, room,
                   humans.build(posture, self.rng, warm=True, style=self.victim_style))
        c, s = math.cos(yaw), math.sin(yaw)
        if posture == 'lying':
            width = 1.4 if self.victim_style == 'randy' else 0.6
            self.footprints.append(Footprint(x - 0.22 * c, y - 0.22 * s, yaw, 1.8, width))
            self.blobs.append((x - 0.22 * c, y - 0.22 * s, 0.95))
        else:
            self.footprints.append(Footprint(x + 0.43 * c, y + 0.43 * s, yaw, 1.2, 0.6))
            self.blobs.append((x + 0.43 * c, y + 0.43 * s, 0.65))
        if kind == 'buried':
            self._bury(v)
        return v

    def stairs(self, room):
        """Stepped debris: three 8 cm steps up to a 24 cm landing (climbable one step at a time)."""
        p = self.sample_free(room, 1.0, passage_clear=0.9)
        if p is None:
            return
        yaw = self.rng.uniform(-math.pi, math.pi)
        c, s = math.cos(yaw), math.sin(yaw)
        color = self.rubble_color()
        for k, (depth, h) in enumerate(((1.5, 0.08), (1.05, 0.16), (0.6, 0.24))):
            off = (1.5 - depth) / 2                 # steps share the back edge
            self.box('step', (depth, 1.0, h), (p[0] + off * c, p[1] + off * s, h / 2, 0, 0, yaw),
                     color, footprint=False)
        self.footprints.append(Footprint(p[0], p[1], yaw, 1.5, 1.0, True))
        self.blobs.append((p[0], p[1], 1.0))

    def smoke(self, room):
        x0, x1, y0, y1 = self.room_bounds(*room, margin=0.4)
        self.extras.append({'type': 'smoke', 'x': (x0 + x1) / 2, 'y': (y0 + y1) / 2,
                            'sx': x1 - x0, 'sy': y1 - y0})

    def roof(self, room):
        """Collapsed roof slab over a room: no daylight inside (lidar is unaffected)."""
        x0, x1, y0, y1 = self.room_bounds(*room, margin=-0.1)
        self.parts.append(Part(self._name('roof'), 'box', (x1 - x0, y1 - y0, 0.15),
                               ((x0 + x1) / 2, (y0 + y1) / 2, 2.35, self.rng.uniform(-0.03, 0.03),
                                self.rng.uniform(-0.03, 0.03), 0), (0.35, 0.34, 0.33)))
        self.extras.append({'type': 'dark', 'room': list(room)})

    def _wall_spot(self, room):
        x0, x1, y0, y1 = self.room_bounds(*room)
        for _ in range(30):
            side = self.rng.choice('nsew')
            off = 0.14
            if side == 'n':
                x, y, yaw = self.rng.uniform(x0 + 0.6, x1 - 0.6), y1 - off, -math.pi / 2
            elif side == 's':
                x, y, yaw = self.rng.uniform(x0 + 0.6, x1 - 0.6), y0 + off, math.pi / 2
            elif side == 'e':
                x, y, yaw = x1 - off, self.rng.uniform(y0 + 0.6, y1 - 0.6), math.pi
            else:
                x, y, yaw = x0 + off, self.rng.uniform(y0 + 0.6, y1 - 0.6), 0.0
            cx, cy = x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw)
            # The victim touches the wall on purpose, so only check other obstacles.
            if all(math.hypot(cx - bx, cy - by) > 0.6 + br + 0.15 for bx, by, br in self.blobs) \
                    and all(math.hypot(cx - px, cy - py) > 1.6 for px, py in self.passages):
                return x, y, yaw
        return None

    def _bury(self, v: Victim):
        """Cover a lying victim with debris; the top slabs are warmed by the body underneath."""
        c, s = math.cos(v.yaw), math.sin(v.yaw)
        for t in (-0.85, -0.4, 0.05, 0.45):
            bx, by = v.x + t * c, v.y + t * s
            sz = self.rng.uniform(0.25, 0.4)
            self.box('debris', (self.rng.uniform(0.5, 0.65), self.rng.uniform(0.6, 0.8), sz),
                     (bx, by, 0.2 + sz / 2, self.rng.uniform(-0.15, 0.15),
                      self.rng.uniform(-0.15, 0.15), v.yaw + self.rng.uniform(-0.3, 0.3)),
                     self.rubble_color(), temperature=self.rng.uniform(302.0, 305.0))
        for _ in range(self.rng.randint(5, 8)):
            a = self.rng.uniform(0, 2 * math.pi)
            t = self.rng.uniform(-0.9, 0.5)
            bx = v.x + t * c + 0.55 * math.cos(a)
            by = v.y + t * s + 0.55 * math.sin(a)
            sz = self.rng.uniform(0.3, 0.55)
            self.box('debris', (self.rng.uniform(0.3, 0.55), self.rng.uniform(0.3, 0.55), sz),
                     (bx, by, sz / 2, self.rng.uniform(-0.2, 0.2), self.rng.uniform(-0.2, 0.2),
                      self.rng.uniform(-math.pi, math.pi)),
                     self.rubble_color())
        self.blobs.append((v.x - 0.22 * c, v.y - 0.22 * s, 1.35))

    def decoy(self, did, room):
        p = self.sample_free(room, 0.5, passage_clear=0.9)
        if p is None:
            return None
        temp = self.rng.uniform(330.0, 360.0)
        for _ in range(3):
            sz = self.rng.uniform(0.25, 0.4)
            self.box('smoulder', (self.rng.uniform(0.3, 0.5), self.rng.uniform(0.3, 0.5), sz),
                     (p[0] + self.rng.uniform(-0.15, 0.15), p[1] + self.rng.uniform(-0.15, 0.15),
                      sz / 2, 0, 0, self.rng.uniform(-math.pi, math.pi)),
                     (0.22, 0.2, 0.19), temperature=temp)
        self.blobs.append((p[0], p[1], 0.5))
        return Decoy(did, p[0], p[1], temp)


def _rasterize(footprints, origin, shape, grow=0.0):
    grid = np.zeros(shape, dtype=bool)
    ox, oy = origin
    for f in footprints:
        hx, hy = f.lx / 2 + grow, f.ly / 2 + grow
        r = math.hypot(hx, hy)
        i0 = max(0, int((f.x - r - ox) / GRID_RES))
        i1 = min(shape[1], int((f.x + r - ox) / GRID_RES) + 2)
        j0 = max(0, int((f.y - r - oy) / GRID_RES))
        j1 = min(shape[0], int((f.y + r - oy) / GRID_RES) + 2)
        if i0 >= i1 or j0 >= j1:
            continue
        xs = ox + (np.arange(i0, i1) + 0.5) * GRID_RES - f.x
        ys = oy + (np.arange(j0, j1) + 0.5) * GRID_RES - f.y
        X, Y = np.meshgrid(xs, ys)
        c, s = math.cos(f.yaw), math.sin(f.yaw)
        inside = (np.abs(X * c + Y * s) <= hx) & (np.abs(-X * s + Y * c) <= hy)
        grid[j0:j1, i0:i1] |= inside
    return grid


def _flood(blocked, start):
    reach = np.zeros_like(blocked)
    if blocked[start]:
        return reach
    h, w = blocked.shape
    q = deque([start])
    reach[start] = True
    while q:
        j, i = q.popleft()
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nj, ni = j + dj, i + di
            if 0 <= nj < h and 0 <= ni < w and not reach[nj, ni] and not blocked[nj, ni]:
                reach[nj, ni] = True
                q.append((nj, ni))
    return reach


def _check(b: _Builder, layout: Layout, check_victims=True):
    """Every room and every victim must be reachable (or, if buried, observable) from the
    spawn, climbing over climbable debris where needed."""
    ox, oy = -2.0, -3.5
    shape = (int((b.height + 5.5) / GRID_RES), int((b.width + 4.0) / GRID_RES))
    blocking = [f for f in b.footprints if not f.climbable]
    grid = _rasterize(blocking, (ox, oy), shape)
    climbable = _rasterize([f for f in b.footprints if f.climbable], (ox, oy), shape) & ~grid
    inflated = _rasterize(blocking, (ox, oy), shape, grow=ROBOT_RADIUS)
    sx, sy, _ = layout.spawns[0]
    start = (int((sy - oy) / GRID_RES), int((sx - ox) / GRID_RES))
    reach = _flood(inflated, start)
    layout.grid, layout.climbable, layout.reachable = grid, climbable, reach
    layout.grid_origin = (ox, oy)

    def cells_near(x, y, r):
        i0, i1 = int((x - r - ox) / GRID_RES), int((x + r - ox) / GRID_RES)
        j0, j1 = int((y - r - oy) / GRID_RES), int((y + r - oy) / GRID_RES)
        return reach[max(0, j0):j1, max(0, i0):i1].sum()

    for i in range(b.cols):
        for j in range(b.rows):
            x0, x1, y0, y1 = b.room_bounds(i, j)
            sub = reach[int((y0 - oy) / GRID_RES):int((y1 - oy) / GRID_RES),
                        int((x0 - ox) / GRID_RES):int((x1 - ox) / GRID_RES)]
            if sub.sum() * GRID_RES ** 2 < 1.0:
                raise LayoutError(f'room {(i, j)} unreachable')
    for v in layout.victims if check_victims else []:      # user-placed victims: user's call
        if cells_near(v.x, v.y, 1.8 if v.kind == 'visible' else 3.0) == 0:
            raise LayoutError(f'{v.id} unreachable')


def generate_layout(seed=0, cols=4, rows=4, room_w=6.0, room_h=5.0,
                    n_visible=4, n_buried=2, n_decoys=1, n_robots=3, max_attempts=50,
                    victim_style='auto', difficulty='normal', custom_victims=None,
                    climb_max=CLIMB_MAX_H) -> Layout:
    """victim_style: 'randy' (Fuel mannequin meshes), 'primitive', or 'auto' (randy if cached).
    climb_max: tallest climbable step for the robot that will search it (others are walls).
    difficulty: 'normal' or 'hard'.
    custom_victims: list of {x, y, posture: lying|sitting, buried: bool, yaw (deg, optional)}
    replacing the random victims."""
    if victim_style == 'auto':
        victim_style = 'randy' if humans.randy_available() else 'primitive'
    last = None
    for attempt in range(max_attempts):
        try:
            return _try(seed, attempt, cols, rows, room_w, room_h,
                        n_visible, n_buried, n_decoys, n_robots, victim_style, difficulty, custom_victims,
                        climb_max)
        except LayoutError as e:
            last = e
    raise LayoutError(f'seed {seed}: no valid layout in {max_attempts} attempts ({last})')


def _try(seed, attempt, cols, rows, room_w, room_h, n_visible, n_buried, n_decoys, n_robots,
         victim_style, difficulty='normal', custom_victims=None, climb_max=CLIMB_MAX_H):
    rng = random.Random(seed * 1000 + attempt)
    hard = difficulty == 'hard'
    b = _Builder(rng, cols, rows, room_w, room_h, victim_style, hard, climb_max)
    W, H = b.width, b.height
    door_w = DOOR_W_HARD if hard else DOOR_W

    # Outer shell: entrance in the middle of a south room (never where an interior wall
    # meets the outer one, which happens at W/2 with an even number of columns).
    ex = (cols // 2 + 0.5) * room_w
    b.passages.append((ex, 0.0))
    b.wall(0, 0, W, 0, [(ex, ENTRANCE_W)], damaged=True)
    b.wall(W, 0, W, H, [], damaged=True)
    b.wall(W, H, 0, H, [], damaged=True)
    b.wall(0, H, 0, 0, [], damaged=True)

    # Interior walls: one door per shared edge, some breaches, some doors buried.
    edges = [((i * room_w, j * room_h), (i * room_w, (j + 1) * room_h))
             for i in range(1, cols) for j in range(rows)]
    edges += [((i * room_w, j * room_h), ((i + 1) * room_w, j * room_h))
              for j in range(1, rows) for i in range(cols)]
    blocked_doors = []
    for (ax, ay), (bx, by) in edges:
        L = math.hypot(bx - ax, by - ay)
        ux, uy = (bx - ax) / L, (by - ay) / L
        td = rng.uniform(0.9, L - 0.9)
        gaps = [(td, door_w)]
        door = [ax + ux * td, ay + uy * td, False]
        b.doors.append(door)
        if rng.random() < (0.2 if hard else 0.3):
            wb = rng.uniform(0.8, 1.3)
            for _ in range(10):
                tb = rng.uniform(wb / 2 + 0.4, L - wb / 2 - 0.4)
                if abs(tb - td) > (wb + door_w) / 2 + 0.6:
                    gaps.append((tb, wb))
                    b.passages.append((ax + ux * tb, ay + uy * tb))
                    break
        if rng.random() < (0.25 if hard else 0.2):
            door[2] = True
            blocked_doors.append(door)
        else:
            b.passages.append((door[0], door[1]))
        b.wall(ax, ay, bx, by, gaps, damaged=rng.random() < 0.5)

    for d in blocked_doors:
        b.rubble_pile(d[0], d[1], 0.45, rng.randint(6, 9))

    rooms = [(i, j) for i in range(cols) for j in range(rows)]
    for room in rooms:
        if rng.random() < 0.3:
            b.leaning_slab(room)
        if rng.random() < 0.35:
            b.fallen_beam(room)

    # Victims before generic rubble so they get first pick of floor space.
    victims = []
    order = rooms[:]
    rng.shuffle(order)
    if custom_victims:
        kinds = []
        for k, cv in enumerate(custom_victims):
            kind = 'buried' if cv.get('buried') else 'visible'
            posture = 'lying' if kind == 'buried' else cv.get('posture', 'lying')
            yaw = math.radians(cv['yaw']) if 'yaw' in cv else rng.uniform(-math.pi, math.pi)
            victims.append(b.place_victim_at(f'victim_{k}', kind, posture, float(cv['x']), float(cv['y']), yaw))
    else:
        kinds = ['visible'] * n_visible + ['buried'] * n_buried
        rng.shuffle(kinds)
        for k, kind in enumerate(kinds):
            v = b.place_victim(f'victim_{k}', kind, order[k % len(order)])
            if v is None:
                raise LayoutError(f'could not place victim_{k}')
            victims.append(v)
    decoys = []
    for k in range(n_decoys):
        d = b.decoy(f'decoy_{k}', order[(len(kinds) + k) % len(order)])
        if d is not None:
            decoys.append(d)

    if hard:
        # Dark rooms and smoke away from the entrance room, so the robots get started.
        entrance_room = (cols // 2, 0)
        others = [r for r in rooms if r != entrance_room]
        rng.shuffle(others)
        n_dark, n_smoke = max(1, len(rooms) // 6), max(1, len(rooms) // 4)
        for room in others[:n_dark]:
            b.roof(room)
        for room in others[n_dark:n_dark + n_smoke]:
            b.smoke(room)
    for room in rooms:
        if hard and rng.random() < 0.35:
            b.stairs(room)
        for _ in range(rng.choice([1, 1, 2, 2] if hard else [0, 1, 1, 2])):
            p = b.sample_free(room, 0.9, passage_clear=1.0)
            if p:
                b.rubble_pile(p[0], p[1], rng.uniform(0.4, 0.8), rng.randint(5, 10))
        if rng.random() < 0.35:
            b.mound(room)
        if rng.random() < 0.4:
            b.fallen_beam(room, thin=True)
        if rng.random() < 0.4:
            b.gravel(room)
        for _ in range(rng.randint(1, 4)):
            b.low_debris(room)

    spawns = [(ex + (k - (n_robots - 1) / 2) * 1.0, -2.0, math.pi / 2) for k in range(n_robots)]
    layout = Layout(seed, attempt, W, H, cols, rows, b.parts, b.footprints, victims, decoys,
                    (ex, 0.0, ENTRANCE_W), spawns, [tuple(d) for d in b.doors], difficulty, b.extras)
    _check(b, layout, check_victims=not custom_victims)
    return layout
