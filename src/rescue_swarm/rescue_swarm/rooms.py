"""Who searches which room: an initial split of the building's rooms between the robots,
and taking rooms over from the busiest teammate once one's own are done. Pure Python.

The rescuers' plan gives the building outline and its grid of rooms. Every robot computes
the same initial split from it, so they agree before saying a word: rooms are ordered by
their bearing from the entrance (left to right as you walk in) and cut into one
contiguous, equal-sized group per robot, r1 taking the leftmost (it starts leftmost).

Each entry of the book is [owner, t]; books merge entry by entry, newest wins, so a
take-over spreads by gossip like everything else. A room is done once enough of it is
explored by anyone. A robot whose own rooms are all done takes the half of the busiest
teammate's unfinished rooms nearest to itself (never the room that teammate is in or
heading to), stamping them with the current time. It only takes from a teammate with at
least two unfinished rooms, and never a room that changed hands in the last minute:
otherwise two idle robots hand the last room back and forth.
"""
from __future__ import annotations

import math

DONE_FRACTION = 0.85


def room_key(i, j):
    return f'{i},{j}'


def room_rects(rect, cols, rows):
    """{key: (x0, y0, x1, y1)} for a cols x rows grid over the building rectangle."""
    x0, y0, x1, y1 = rect
    w, h = (x1 - x0) / cols, (y1 - y0) / rows
    return {room_key(i, j): (x0 + i * w, y0 + j * h, x0 + (i + 1) * w, y0 + (j + 1) * h)
            for i in range(cols) for j in range(rows)}


def room_at(rooms, x, y):
    for k, (x0, y0, x1, y1) in rooms.items():
        if x0 <= x < x1 and y0 <= y < y1:
            return k
    return None


def centre(r):
    return (r[0] + r[2]) / 2, (r[1] + r[3]) / 2


def initial_split(rooms, robots, entrance):
    """{room key: robot}: rooms by bearing from the entrance, left to right, cut evenly."""
    ex, ey = entrance

    def bearing(k):
        cx, cy = centre(rooms[k])
        # angle measured from "straight in" (+y) towards the left (-x): left rooms first
        return math.atan2(-(cx - ex), cy - ey), math.hypot(cx - ex, cy - ey)

    order = sorted(rooms, key=bearing, reverse=True)
    n = len(robots)
    return {k: robots[min(n - 1, idx * n // len(order))] for idx, k in enumerate(order)}


class RoomBook:
    def __init__(self, rooms, robots, entrance):
        self.rooms = rooms
        self.robots = list(robots)
        self.book = {k: [r, 0.0] for k, r in initial_split(rooms, robots, entrance).items()}

    def owner(self, key):
        e = self.book.get(key)
        return e[0] if e else None

    def owner_at(self, x, y):
        return self.owner(room_at(self.rooms, x, y))

    def message(self):
        return {k: list(v) for k, v in self.book.items()}

    def merge(self, other):
        for k, (r, t) in other.items():
            if k in self.book and t > self.book[k][1]:
                self.book[k] = [r, t]

    def done(self, explored):
        """{key: bool}; explored(x0, y0, x1, y1) -> fraction of that rectangle explored."""
        return {k: explored(*r) >= DONE_FRACTION for k, r in self.rooms.items()}

    def open_rooms(self, robot, done):
        return [k for k, (r, _) in self.book.items() if r == robot and not done[k]]

    def rebalance(self, me, my_pos, busy_points, done, t, settle=60.0):
        """Own rooms all done: take the nearest half (at least one) of the busiest teammate's
        unfinished rooms, except rooms holding one of `busy_points` (teammates' positions
        and goals). Returns the keys taken."""
        if self.open_rooms(me, done):
            return []
        loads = {r: self.open_rooms(r, done) for r in self.robots if r != me}
        if not loads:
            return []
        busiest = max(loads, key=lambda r: (len(loads[r]), r))
        if len(loads[busiest]) < 2:
            return []                     # one room left: let its owner finish it
        busy = {room_at(self.rooms, x, y) for x, y in busy_points}
        candidates = [k for k in loads[busiest]
                      if k not in busy and (self.book[k][1] == 0.0 or t - self.book[k][1] > settle)]
        if not candidates:
            return []
        n = max(1, len(loads[busiest]) // 2)
        candidates.sort(key=lambda k: math.hypot(centre(self.rooms[k])[0] - my_pos[0],
                                                 centre(self.rooms[k])[1] - my_pos[1]))
        taken = candidates[:n]
        for k in taken:
            self.book[k] = [me, t]
        return taken
