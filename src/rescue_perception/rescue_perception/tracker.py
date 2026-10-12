"""Fuse victim sightings over time into a list of victims. Pure Python, no ROS.

Each sighting is a map-frame position from one cue (YOLO person / body-temperature
blob / too-hot blob). Sightings within `radius` of a track join it. A track becomes:

  visible   YOLO saw a person there at least twice (thermal agreement adds confidence)
  buried    only thermal: a body-temperature signature, seen repeatedly, no person
            visible (debris warmed by someone underneath)
  hazard    hotter than a person (fire, smouldering debris): reported, never a victim
  candidate not enough evidence yet

Confidence grows with sightings and with the number of distinct viewpoints, so a
single bad frame can't confirm a victim. When reporting, a weaker track close to a
stronger one is folded into it: partial views (a body cut by the image edge or an
obstacle) produce "ghost" sightings a metre or two off, and a rescue team needs one
marker per person.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class Track:
    id: int
    x: float
    y: float
    n_rgb: int = 0
    n_thermal: int = 0
    n_hot: int = 0
    max_temp_k: float = 0.0
    first_seen: float = 0.0
    last_seen: float = 0.0
    viewpoints: list = field(default_factory=list)   # robot positions it was seen from
    extra_views: int = 0                              # viewpoints reported by teammates
    weight: float = 0.0                               # sum of sighting weights (position mean)
    found_by: str = ''

    @property
    def views(self):
        return len(self.viewpoints) + self.extra_views

    @property
    def kind(self):
        if self.n_hot >= 2 and self.n_hot >= self.n_thermal:
            return 'hazard'
        if self.n_rgb >= 2:
            return 'visible'
        if self.n_thermal >= 3 and self.n_rgb == 0 and self.views >= 2:
            return 'buried'
        return 'candidate'

    @property
    def confidence(self):
        views = min(self.views, 4)
        if self.kind == 'visible':
            e = 0.5 * self.n_rgb + 0.25 * min(self.n_thermal, 6) + 0.3 * views
        elif self.kind == 'buried':
            e = 0.15 * self.n_thermal + 0.3 * views
        else:
            return 0.0
        return round(1.0 - math.exp(-e), 3)

    def to_dict(self):
        return {'id': self.id, 'kind': self.kind, 'confidence': self.confidence,
                'x': round(self.x, 2), 'y': round(self.y, 2),
                'sightings': {'rgb': self.n_rgb, 'thermal': self.n_thermal, 'hot': self.n_hot},
                'views': self.views,
                'max_temp_c': round(self.max_temp_k - 273.15, 1) if self.max_temp_k else None,
                'first_seen': round(self.first_seen, 1), 'last_seen': round(self.last_seen, 1),
                'found_by': self.found_by}


class Tracker:
    def __init__(self, radius=1.6, viewpoint_spacing=0.8, confirm=0.6, merge_victims=3.0, merge_hazards=3.0):
        self.radius = radius
        self.merge_victims, self.merge_hazards = merge_victims, merge_hazards
        self.viewpoint_spacing = viewpoint_spacing
        self.confirm = confirm
        self.tracks: list[Track] = []
        self._next = 1

    def add(self, source, x, y, t, robot_xy, temp_k=0.0, found_by='', distance=None):
        """`distance`: how far the robot was. Far sightings are less precise (the camera is
        low, a pixel is a lot of floor), so they get a wider match radius and less weight."""
        radius = self.radius if distance is None else max(self.radius, 0.3 * distance)
        w = 1.0 if distance is None else 1.0 / max(distance, 1.0) ** 2
        best, best_d = None, radius
        for tr in self.tracks:
            d = math.hypot(tr.x - x, tr.y - y)
            if d < best_d:
                best, best_d = tr, d
        if best is None:
            best = Track(self._next, x, y, first_seen=t, found_by=found_by)
            self._next += 1
            self.tracks.append(best)
        best.x = (best.x * best.weight + x * w) / (best.weight + w)   # weighted running mean
        best.y = (best.y * best.weight + y * w) / (best.weight + w)
        best.weight += w
        if source == 'rgb':
            best.n_rgb += 1
        elif source == 'thermal':
            best.n_thermal += 1
        else:
            best.n_hot += 1
        best.max_temp_k = max(best.max_temp_k, temp_k)
        best.last_seen = t
        new_view = robot_xy is not None and all(
            math.hypot(robot_xy[0] - vx, robot_xy[1] - vy) > self.viewpoint_spacing for vx, vy in best.viewpoints)
        if new_view:
            best.viewpoints.append(tuple(robot_xy))
        return best

    def merge_external(self, victim, t):
        """Add a victim reported by another robot (dict from to_dict, positions in the
        same frame). Its evidence counts, so a teammate's confirmation carries over."""
        tr = self.add('rgb' if victim['kind'] == 'visible' else
                      'hot' if victim['kind'] == 'hazard' else 'thermal',
                      victim['x'], victim['y'], t, None, found_by=victim.get('found_by', ''))
        tr.extra_views = max(tr.extra_views, victim.get('views', 1) - len(tr.viewpoints))
        s = victim.get('sightings', {})
        tr.n_rgb = max(tr.n_rgb, s.get('rgb', 0))
        tr.n_thermal = max(tr.n_thermal, s.get('thermal', 0))
        tr.n_hot = max(tr.n_hot, s.get('hot', 0))
        if victim.get('max_temp_c') is not None:
            tr.max_temp_k = max(tr.max_temp_k, victim['max_temp_c'] + 273.15)
        if not tr.found_by:
            tr.found_by = victim.get('found_by', '')
        return tr

    @staticmethod
    def _suppress(tracks, radius):
        """Strongest first; drop tracks within `radius` of an already kept one."""
        kept = []
        for t in sorted(tracks, key=lambda t: -(t.n_rgb * 3 + t.n_thermal + t.n_hot)):
            if all(math.hypot(t.x - k.x, t.y - k.y) >= radius for k in kept):
                kept.append(t)
        return kept

    def confirmed(self):
        tracks = [t for t in self.tracks if t.kind in ('visible', 'buried') and t.confidence >= self.confirm]
        return self._suppress(tracks, self.merge_victims)

    def hazards(self):
        return self._suppress([t for t in self.tracks if t.kind == 'hazard'], self.merge_hazards)
