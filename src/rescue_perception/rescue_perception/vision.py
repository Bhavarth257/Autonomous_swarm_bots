"""Image-side victim cues and their 3D position. Pure numpy/OpenCV, no ROS.

Thermal images are 16-bit temperatures in centi-kelvin (Gazebo's L16 thermal camera).
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

BODY_K = (301.0, 318.0)     # a body, or debris warmed by one buried underneath
FIRE_K = 325.0              # hotter than any person: smouldering debris, fire


@dataclass
class Cue:
    source: str              # 'rgb' (YOLO person) | 'thermal' (body-temperature blob) | 'hot'
    box: tuple               # x0, y0, x1, y1 in pixels of the source image
    score: float             # YOLO confidence, or blob area fraction for thermal
    temp_k: float = 0.0      # max temperature inside the box (thermal cues)


def thermal_cues(centi_kelvin: np.ndarray, min_area: int = 25) -> list[Cue]:
    """Body-temperature blobs and too-hot blobs in a thermal frame."""
    t = centi_kelvin.astype(np.float32) / 100.0
    out = []
    for source, mask in (('thermal', (t >= BODY_K[0]) & (t <= BODY_K[1])), ('hot', t > FIRE_K)):
        mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for k in range(1, n):
            x, y, w, h, area = stats[k]
            if area < min_area:
                continue
            temps = t[labels == k]
            out.append(Cue(source, (int(x), int(y), int(x + w), int(y + h)),
                           float(area) / t.size, float(temps.max())))
    return out


def scale_box(box, from_shape, to_shape):
    """Map a box between two images of the same camera at different resolutions."""
    sy, sx = to_shape[0] / from_shape[0], to_shape[1] / from_shape[1]
    x0, y0, x1, y1 = box
    return (x0 * sx, y0 * sy, x1 * sx, y1 * sy)


def touches_border(box, shape, margin=2):
    x0, y0, x1, y1 = box
    return x0 <= margin or y0 <= margin or x1 >= shape[1] - margin or y1 >= shape[0] - margin


def box_covers(K: np.ndarray, R_cam: np.ndarray, p_cam: np.ndarray, point, box, radius: float) -> bool:
    """Does the image box overlap a ball of `radius` around `point` (map frame)?

    Used to drop "person" boxes that are really a teammate: YOLO sometimes takes a
    distant dark robot for a person, and a floor-plane position estimate that far
    away is too rough to tell it apart from the teammate's position.
    """
    pc = R_cam.T @ (np.asarray(point, dtype=float) - p_cam)
    if pc[2] < 0.2:                  # behind or right at the camera
        return False
    u = K[0, 0] * pc[0] / pc[2] + K[0, 2]
    v = K[1, 1] * pc[1] / pc[2] + K[1, 2]
    m = K[0, 0] * radius / pc[2]
    x0, y0, x1, y1 = box
    return x0 - m <= u <= x1 + m and y0 - m <= v <= y1 + m


def floor_point(K: np.ndarray, R_cam: np.ndarray, p_cam: np.ndarray, pixel, floor_z: float):
    """Where the ray through `pixel` meets the plane z = floor_z (map frame), or None.

    R_cam, p_cam: camera optical frame -> map. Victims lie or sit on the floor, so the
    bottom of their image box is on the floor: this gives their distance even when the
    sparse lidar rings pass above a lying body and only hit the wall behind it.
    """
    d = np.linalg.inv(K) @ np.array([pixel[0], pixel[1], 1.0])
    d = R_cam @ d
    if d[2] > -1e-3:                 # ray doesn't go down
        return None
    s = (floor_z - p_cam[2]) / d[2]
    return p_cam + s * d if s > 0 else None


def locate(points_cam: np.ndarray, K: np.ndarray, box, shrink: float = 0.15,
           min_points: int = 3, depth_band: float = 0.5, expected_depth: float | None = None,
           depth_tolerance: float = 1.0):
    """3D position (camera optical frame) of what is inside `box`, from lidar points.

    points_cam      (N, 3) lidar points already in the camera optical frame (z forward)
    K               3x3 intrinsics of the image the box comes from
    expected_depth  if given (from floor_point), only points within `depth_tolerance` of it
                    count, so a wall behind the object can't be mistaken for it
    The nearest surface inside the (slightly shrunk) box is taken as the object: points
    within `depth_band` of the 20th-percentile depth, averaged. None if too few points.
    """
    front = points_cam[points_cam[:, 2] > 0.2]
    if expected_depth is not None:
        front = front[np.abs(front[:, 2] - expected_depth) < depth_tolerance]
    if len(front) == 0:
        return None
    uv = (K @ front.T).T
    u, v = uv[:, 0] / uv[:, 2], uv[:, 1] / uv[:, 2]
    x0, y0, x1, y1 = box
    dx, dy = (x1 - x0) * shrink / 2, (y1 - y0) * shrink / 2
    inside = (u >= x0 + dx) & (u <= x1 - dx) & (v >= y0 + dy) & (v <= y1 - dy)
    pts = front[inside]
    if len(pts) < min_points:
        return None
    near = np.percentile(pts[:, 2], 20)
    sel = pts[pts[:, 2] < near + depth_band]
    return sel.mean(axis=0)


def annotate(rgb: np.ndarray, cues: list[Cue], thermal_shape=None) -> np.ndarray:
    """Draw cue boxes on a copy of the RGB image (thermal boxes rescaled)."""
    img = np.ascontiguousarray(rgb.copy())
    colors = {'rgb': (0, 255, 0), 'thermal': (255, 140, 0), 'hot': (255, 0, 0)}
    for c in cues:
        box = c.box if c.source == 'rgb' or thermal_shape is None else \
            scale_box(c.box, thermal_shape, img.shape)
        x0, y0, x1, y1 = (int(v) for v in box)
        cv2.rectangle(img, (x0, y0), (x1, y1), colors[c.source], 2)
        label = f'person {c.score:.2f}' if c.source == 'rgb' else f'{c.temp_k - 273.15:.0f}C'
        cv2.putText(img, label, (x0, max(12, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colors[c.source], 1)
    return img
