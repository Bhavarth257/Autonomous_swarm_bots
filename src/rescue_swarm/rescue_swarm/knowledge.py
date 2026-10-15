"""What one robot knows about its team, and how two such views merge. Pure Python/numpy.

Everything is in the world frame (the building plan the rescuers share), so robots
whose SLAM maps have different origins can still combine knowledge. Gossip style:
each robot broadcasts its whole view; a receiver keeps the newest entry per robot and
ORs the explored maps, so news travels through teammates that are in range of both.
"""
from __future__ import annotations

import base64
import math

import numpy as np

from rescue_perception.tracker import Tracker


class ExploredGrid:
    """Coarse world-frame map of which parts of the building someone has seen."""

    def __init__(self, rect, res=0.5):
        self.x0, self.y0, x1, y1 = rect
        self.res = res
        self.w = int(math.ceil((x1 - self.x0) / res))
        self.h = int(math.ceil((y1 - self.y0) / res))
        self.mask = np.zeros((self.h, self.w), dtype=bool)

    def cell(self, x, y):
        i, j = int((x - self.x0) / self.res), int((y - self.y0) / self.res)
        return (j, i) if 0 <= i < self.w and 0 <= j < self.h else None

    def explored(self, x, y):
        c = self.cell(x, y)
        return c is not None and bool(self.mask[c])

    def update_from_points(self, wx, wy, known, per_cell):
        """Coarse cell is explored if >= 60 % of its fine cells (`per_cell` of them) are known."""
        i = ((wx - self.x0) / self.res).astype(np.int64)
        j = ((wy - self.y0) / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.w) & (j >= 0) & (j < self.h) & known
        counts = np.zeros(self.h * self.w)
        np.add.at(counts, j[ok] * self.w + i[ok], 1)
        self.mask |= (counts.reshape(self.h, self.w) >= 0.6 * per_cell)

    def encode(self):
        return base64.b64encode(np.packbits(self.mask).tobytes()).decode()

    def merge_encoded(self, text):
        bits = np.unpackbits(np.frombuffer(base64.b64decode(text), np.uint8))[:self.h * self.w]
        self.mask |= bits.reshape(self.h, self.w).astype(bool)

    def fraction(self):
        return float(self.mask.mean())

    def fraction_in(self, x0, y0, x1, y1):
        """Explored fraction of a world rectangle (e.g. a room)."""
        i0, i1 = max(0, int((x0 - self.x0) / self.res)), min(self.w, int(math.ceil((x1 - self.x0) / self.res)))
        j0, j1 = max(0, int((y0 - self.y0) / self.res)), min(self.h, int(math.ceil((y1 - self.y0) / self.res)))
        sub = self.mask[j0:j1, i0:i1]
        return float(sub.mean()) if sub.size else 0.0


class TeamView:
    """One robot's picture of the team."""

    def __init__(self, me, rect, res=0.5):
        self.me = me
        self.poses = {}       # robot -> [x, y, yaw, t]      (world frame)
        self.goals = {}       # robot -> [x, y, t]   (x, y None: no goal)
        self.states = {}      # robot -> [state, t]
        self.health = {}      # robot -> [health, t]    (health.py: ok / stuck / down)
        self.heard = {}       # robot -> local time we last heard it directly
        self.rect, self.res = rect, res
        self.explored = {me: ExploredGrid(rect, res)}   # per robot, so "others" stays separable
        self.victims = Tracker(radius=1.5)
        self.hazards = Tracker(radius=1.5)

    # ---- own data ---------------------------------------------------------------------
    def set_self(self, pose, goal, state, t):
        self.poses[self.me] = [*pose, t]
        self.goals[self.me] = [*(goal or (None, None)), t]
        self.states[self.me] = [state, t]

    def set_health(self, health, t):
        self.health[self.me] = [health, t]

    def add_own_victims(self, report, t):
        """`report` is the detector's /rN/victims JSON (world_x/world_y used)."""
        for key, tracker in (('victims', self.victims), ('hazards', self.hazards)):
            for v in report.get(key, []):
                tracker.merge_external({**v, 'x': v['world_x'], 'y': v['world_y'],
                                        'found_by': v.get('found_by') or self.me}, t)

    # ---- radio --------------------------------------------------------------------------
    def message(self, t):
        return {
            'from': self.me, 't': t,
            'poses': self.poses, 'goals': self.goals, 'states': self.states, 'health': self.health,
            'explored': {r: g.encode() for r, g in self.explored.items()},
            'victims': [v.to_dict() for v in self.victims.confirmed()],
            'hazards': [v.to_dict() for v in self.hazards.hazards()],
        }

    def receive(self, msg, t):
        self.heard[msg['from']] = t
        for table, incoming in ((self.poses, msg['poses']), (self.goals, msg['goals']),
                                (self.states, msg['states']), (self.health, msg.get('health', {}))):
            for r, v in incoming.items():
                if r != self.me and (r not in table or v[-1] > table[r][-1]):
                    table[r] = v
        for r, text in msg['explored'].items():
            if r != self.me:
                self.explored.setdefault(r, ExploredGrid(self.rect, self.res)).merge_encoded(text)
        for v in msg['victims']:
            self.victims.merge_external(v, t)
        for v in msg['hazards']:
            self.hazards.merge_external(v, t)

    # ---- for the explorer ----------------------------------------------------------------
    def mine(self):
        return self.explored[self.me]

    def explored_by_others(self):
        g = ExploredGrid(self.rect, self.res)
        for r, e in self.explored.items():
            if r != self.me:
                g.mask |= e.mask
        return g

    def explored_by_team(self):
        g = ExploredGrid(self.rect, self.res)
        for e in self.explored.values():
            g.mask |= e.mask
        return g

    def others(self):
        """World positions to keep away from: teammates and the goals they claimed."""
        pts = []
        for r, p in self.poses.items():
            if r != self.me:
                pts.append((p[0], p[1]))
        for r, g in self.goals.items():
            if r != self.me and g[0] is not None:
                pts.append((g[0], g[1]))
        return pts
