"""Grid maths shared by the traversability and explorer nodes. Pure numpy, no ROS.

Conventions follow nav_msgs/OccupancyGrid: row-major, row = y, -1 unknown,
0..100 cost, 100 lethal.
"""
from __future__ import annotations

import math
import warnings

import numpy as np

with warnings.catch_warnings():
    # Ubuntu's SciPy warns about the newer NumPy pip installs; ndimage works fine.
    warnings.simplefilter('ignore')
    from scipy import ndimage

UNKNOWN = -1
LETHAL = 100
FREE_MAX = 60          # cells below this count as walkable for exploration


def step_map(height: np.ndarray) -> np.ndarray:
    """How far a cell rises above its lowest known 8-neighbour (nan where unknown).

    Only the high side of an edge counts: the floor next to a wall or a victim is
    still floor, so obstacles aren't drawn a cell thicker than they are.
    """
    h = np.pad(height, 1, constant_values=np.nan)
    out = np.full(height.shape, np.nan, dtype=np.float32)
    for dj in (-1, 0, 1):
        for di in (-1, 0, 1):
            if dj == 0 and di == 0:
                continue
            nb = h[1 + dj:1 + dj + height.shape[0], 1 + di:1 + di + height.shape[1]]
            d = height - nb
            out = np.fmax(out, d)        # fmax ignores nan
    out[np.isnan(height)] = np.nan
    return out


def traversability(height: np.ndarray, step_max: float, obstacle_height: float,
                   flat_step: float = 0.04) -> np.ndarray:
    """Height map -> int8 cost grid.

    lethal    the top of a step taller than the robot can climb, or anything taller than
              `obstacle_height` above the floor (walls, rubble piles)
    20..70    climbable steps, costlier the taller they are, so paths prefer flat floor
    0         flat
    -1        never observed
    """
    step = step_map(height)
    known = ~np.isnan(height)
    out = np.full(height.shape, UNKNOWN, dtype=np.int8)
    s = np.maximum(np.nan_to_num(step, nan=0.0), 0.0)
    cost = np.where(s < flat_step, 0, 20 + 50 * np.minimum(s / step_max, 1.0)).astype(np.int8)
    out[known] = cost[known]
    lethal = known & ((np.nan_to_num(height, nan=0.0) > obstacle_height) | (s > step_max))
    out[lethal] = LETHAL
    return out


def frontier_mask(grid: np.ndarray, region: np.ndarray | None = None) -> np.ndarray:
    """Walkable cells with an unknown 4-neighbour, optionally restricted to `region`."""
    free = (grid >= 0) & (grid < FREE_MAX)
    unk = grid == UNKNOWN
    u = np.pad(unk, 1, constant_values=False)
    near_unknown = u[:-2, 1:-1] | u[2:, 1:-1] | u[1:-1, :-2] | u[1:-1, 2:]
    mask = free & near_unknown
    if region is not None:
        mask &= region
    return mask


def clusters(mask: np.ndarray, min_size: int) -> list[np.ndarray]:
    """8-connected components of a boolean mask, as (N, 2) arrays of (row, col)."""
    labels, n = ndimage.label(mask, structure=np.ones((3, 3), dtype=bool))
    if n == 0:
        return []
    rows, cols = np.nonzero(labels)
    lab = labels[rows, cols]
    order = np.argsort(lab, kind='stable')
    rows, cols, lab = rows[order], cols[order], lab[order]
    splits = np.flatnonzero(np.diff(lab)) + 1
    out = []
    for r, c in zip(np.split(rows, splits), np.split(cols, splits)):
        if len(r) >= min_size:
            out.append(np.column_stack([r, c]))
    return out


def disk(radius_cells: int) -> np.ndarray:
    r = int(radius_cells)
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    return xx * xx + yy * yy <= r * r


def reachable(grid: np.ndarray, start: tuple, radius_cells: int, search_cells: int = 10) -> np.ndarray:
    """Cells the robot centre can reach from `start` (row, col).

    Lethal cells are grown by the robot radius; unknown cells count as passable (unseen
    floor may well be open). If the start itself is inside the grown obstacles (robot
    brushing a wall), the nearest reachable cell within `search_cells` is used.
    """
    blocked = ndimage.binary_dilation(grid >= LETHAL, structure=disk(radius_cells))
    labels, _ = ndimage.label(~blocked)
    j, i = start
    h, w = grid.shape
    j0, j1 = max(0, j - search_cells), min(h, j + search_cells + 1)
    i0, i1 = max(0, i - search_cells), min(w, i + search_cells + 1)
    window = labels[j0:j1, i0:i1]
    if window.size == 0 or not window.any():
        return np.zeros_like(blocked)
    if 0 <= j < h and 0 <= i < w and labels[j, i]:
        lab = labels[j, i]
    else:
        jj, ii = np.nonzero(window)
        k = np.argmin((jj + j0 - j) ** 2 + (ii + i0 - i) ** 2)
        lab = window[jj[k], ii[k]]
    return labels == lab


def nearest_true(mask: np.ndarray, cell: tuple, radius: int):
    """(row, col) of the True cell nearest to `cell` within `radius` cells, else (None, None)."""
    j, i = cell
    h, w = mask.shape
    j0, j1 = max(0, j - radius), min(h, j + radius + 1)
    i0, i1 = max(0, i - radius), min(w, i + radius + 1)
    jj, ii = np.nonzero(mask[j0:j1, i0:i1])
    if len(jj) == 0:
        return None, None
    k = np.argmin((jj + j0 - j) ** 2 + (ii + i0 - i) ** 2)
    return int(jj[k] + j0), int(ii[k] + i0)


def cell_centers(info_shape, resolution, origin_xy):
    """World (map-frame) x, y of every cell centre."""
    h, w = info_shape
    xs = origin_xy[0] + (np.arange(w) + 0.5) * resolution
    ys = origin_xy[1] + (np.arange(h) + 0.5) * resolution
    return np.meshgrid(xs, ys)


def rect_region(shape, resolution, origin_xy, rect_world, map_in_world, margin=0.0):
    """Mask of cells whose centre lies inside an axis-aligned world rectangle.

    rect_world    (x0, y0, x1, y1) in the world frame
    map_in_world  (x, y, yaw) of the map frame origin in the world frame
    """
    X, Y = cell_centers(shape, resolution, origin_xy)
    mx, my, myaw = map_in_world
    c, s = math.cos(myaw), math.sin(myaw)
    wx = mx + c * X - s * Y
    wy = my + s * X + c * Y
    x0, y0, x1, y1 = rect_world
    return (wx > x0 + margin) & (wx < x1 - margin) & (wy > y0 + margin) & (wy < y1 - margin)
