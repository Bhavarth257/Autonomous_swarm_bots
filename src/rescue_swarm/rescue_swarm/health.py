"""Is this robot still able to search? Each robot judges itself, from its own sensors, and
tells the team over the radio (knowledge.py); teammates then take over the rooms of one
that is not (rooms.py RoomBook.adopt). Pure Python.

  down   tipped over: roll or pitch past `tip_angle` for `tip_time` s
  stuck  trying to move (exploring, returning, operator goal) but no further than
         `stuck_move` m from where it was `stuck_window` s ago; ok again once it has
         got `recover_move` m away from where it got stuck
  ok     otherwise

Silence is not taken as a failure: a robot out of radio range is usually just far away.
"""
from __future__ import annotations

import math

MOVING_STATES = ('exploring', 'returning', 'operator goal')
OK, STUCK, DOWN = 'ok', 'stuck', 'down'


class HealthWatch:
    def __init__(self, tip_angle=math.radians(60), tip_time=5.0, stuck_window=120.0,
                 stuck_move=0.5, recover_move=1.0):
        self.tip_angle, self.tip_time = tip_angle, tip_time
        self.stuck_window, self.stuck_move, self.recover_move = stuck_window, stuck_move, recover_move
        self.tipped_since = None
        self.anchor = None          # (t, x, y): where it was when it last made progress
        self.stuck_at = None        # (x, y) where it got stuck
        self.health = OK
        self.reason = ''

    def update(self, t, xy, state, tilt=None):
        """xy: own position (any fixed frame) or None; tilt: (roll, pitch) rad or None.
        Returns 'ok', 'stuck' or 'down'."""
        # tipped over
        if tilt is not None and max(abs(tilt[0]), abs(tilt[1])) > self.tip_angle:
            if self.tipped_since is None:
                self.tipped_since = t
            if t - self.tipped_since >= self.tip_time:
                return self._set(DOWN, f'tipped over ({math.degrees(max(map(abs, tilt))):.0f} deg)')
        else:
            self.tipped_since = None

        # stuck
        if xy is None or state not in MOVING_STATES:
            self.anchor, self.stuck_at = None, None
            return self._set(OK, '')
        if self.stuck_at is not None:
            if math.hypot(xy[0] - self.stuck_at[0], xy[1] - self.stuck_at[1]) < self.recover_move:
                return self._set(STUCK, self.reason)
            self.stuck_at, self.anchor = None, (t, *xy)
        if self.anchor is None or math.hypot(xy[0] - self.anchor[1], xy[1] - self.anchor[2]) > self.stuck_move:
            self.anchor = (t, *xy)
        elif t - self.anchor[0] >= self.stuck_window:
            self.stuck_at = xy
            return self._set(STUCK, f'moved under {self.stuck_move} m in {self.stuck_window:.0f} s')
        return self._set(OK, '')

    def _set(self, health, reason):
        self.health, self.reason = health, reason
        return health


def unavailable(health, states):
    """Robots that can't search: not ok, or stopped by the operator.
    health: {robot: [health, t]}, states: {robot: [state, t]}."""
    out = {r for r, (h, _) in health.items() if h != OK}
    out |= {r for r, (s, _) in states.items() if s == 'stopped'}
    return out
