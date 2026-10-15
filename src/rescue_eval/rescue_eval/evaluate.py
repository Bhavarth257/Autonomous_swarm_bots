"""Score a running mission against the world's ground truth.

  ros2 run rescue_eval evaluate --ros-args -p seed:=4 -p robots:="['r1','r2','r3']" \
      -p duration:=900.0 -p out:=~/rescue_reports/seed4

Listens (it never influences the mission) to every robot's team/state, mission/status
and ground-truth pose, and when the mission ends (all robots done/stopped) or
`duration` passes, writes <out>.json and <out>.md:

  victims    found / missed per type (visible, buried); a report within `match_radius`
             (1.5 m) of a true victim is a hit, anything else a false positive;
             position error; sim time of first report
  hazards    hot decoys reported as hazards, and decoys wrongly reported as victims
  coverage   share of the building's free floor (ground-truth map) inside the 0.5 m
             cells some robot reports explored
  robots     distance walked (ground truth), final state, whether back at the start
  time       mission time (explorer clocks)
"""
import base64
import json
import math
import os

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def reachable_floor(gt_base):
    """World x, y of free cells from the generator's ground-truth map."""
    with open(gt_base + '_map.yaml') as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(os.path.dirname(gt_base), meta['image']), 'rb') as f:
        parts = f.read().split(b'\n', 3)
    w, h = map(int, parts[1].split())
    img = np.frombuffer(parts[3], np.uint8)[:w * h].reshape(h, w)[::-1]
    free = img == 254
    ox, oy = meta['origin'][:2]
    res = meta['resolution']
    jj, ii = np.nonzero(free)
    return ox + (ii + 0.5) * res, oy + (jj + 0.5) * res


def score_victims(reported, truth, match_radius=1.5):
    hits, false_pos, matched = [], [], {}
    for v in reported:
        best = min(truth, key=lambda g: math.hypot(g['x'] - v['x'], g['y'] - v['y']), default=None)
        d = math.hypot(best['x'] - v['x'], best['y'] - v['y']) if best else math.inf
        if d <= match_radius and best['id'] not in matched:
            matched[best['id']] = (v, d)
            hits.append({'truth': best['id'], 'kind_true': best['kind'], 'kind_reported': v['kind'],
                         'error_m': round(d, 2), 'confidence': v['confidence'],
                         'found_by': v.get('found_by'), 'first_seen_s': v.get('first_seen')})
        else:
            false_pos.append({'x': v['x'], 'y': v['y'], 'kind': v['kind'], 'confidence': v['confidence']})
    missed = [{'id': g['id'], 'kind': g['kind'], 'x': g['x'], 'y': g['y']}
              for g in truth if g['id'] not in matched]
    return hits, false_pos, missed


class Evaluator(Node):
    def __init__(self):
        super().__init__('mission_evaluator')
        self.seed = self.declare_parameter('seed', 0).value
        self.robots = list(self.declare_parameter('robots', ['r1', 'r2', 'r3']).value)
        self.duration = self.declare_parameter('duration', 1200.0).value
        self.out = os.path.expanduser(self.declare_parameter('out', '~/rescue_reports/eval').value)
        world_json = self.declare_parameter('world_json', '').value
        base = world_json[:-5] if world_json else os.path.expanduser(f'~/.ros/rescue_worlds/rescue_{self.seed}')
        with open(base + '.json') as f:
            self.gt = json.load(f)
        self.floor = reachable_floor(base)
        self.team, self.status, self.dist, self.last = {}, {}, {r: 0.0 for r in self.robots}, {}
        for r in self.robots:
            self.create_subscription(String, f'/{r}/team/state', lambda m, r=r: self.on_team(r, m), LATCHED)
            self.create_subscription(String, f'/{r}/mission/status',
                                     lambda m, r=r: self.status.__setitem__(r, json.loads(m.data)), 10)
            self.create_subscription(Odometry, f'/{r}/ground_truth/odom', lambda m, r=r: self.on_gt(r, m), 10)
        # Victims the operator places during the run (dashboard) count too.
        self.create_subscription(String, '/scenario/ground_truth', self.on_truth, LATCHED)
        # worst health each robot reported about itself (swarm health.py), and when
        self.health = {r: ['ok', None] for r in self.robots}
        self.tipped = {}
        self.started = None
        self.finished = False
        self.create_timer(2.0, self.check)

    def on_team(self, r, msg):
        st = json.loads(msg.data)
        self.team[r] = st
        h = st.get('team', {}).get(r, {}).get('health', 'ok')
        rank = {'ok': 0, 'stuck': 1, 'down': 2}
        if rank.get(h, 0) > rank.get(self.health[r][0], 0):
            elapsed = self.status.get(r, {}).get('elapsed_s')
            self.health[r] = [h, elapsed]

    def health_now(self, r):
        return self.team.get(r, {}).get('team', {}).get(r, {}).get('health', 'ok')

    def on_truth(self, msg):
        self.gt['victims'] = json.loads(msg.data)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, r, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        q = m.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        if r not in self.tipped and max(abs(roll), abs(pitch)) > math.radians(60):
            # where and when it went over (ground truth), for finding what tips robots
            self.tipped[r] = {'x': round(p[0], 2), 'y': round(p[1], 2), 'roll_deg': round(math.degrees(roll)),
                              'pitch_deg': round(math.degrees(pitch)),
                              'at_s': self.status.get(r, {}).get('elapsed_s')}
            self.get_logger().warn(f'{r} tipped over at ({p[0]:.1f}, {p[1]:.1f})')
        if r in self.last:
            self.dist[r] += math.hypot(p[0] - self.last[r][0], p[1] - self.last[r][1])
        self.last[r] = p

    def check(self):
        if self.finished:
            return
        if self.started is None and self.status:
            self.started = self.now()
        # a robot that tipped over will never finish: the others decide when it's over
        over = all(self.status.get(r, {}).get('state') in ('done', 'stopped') or self.health_now(r) == 'down'
                   for r in self.robots) and any(self.health_now(r) != 'down' for r in self.robots)
        timeout = self.started is not None and self.now() - self.started > self.duration
        if over or timeout:
            self.finished = True
            self.write(reason='all robots finished' if over else 'time limit')

    def team_view(self):
        """The richest team view any robot holds (they converge when in contact)."""
        if not self.team:
            return {'victims': [], 'hazards': []}
        return max(self.team.values(), key=lambda t: (len(t['victims']), t['explored_fraction']['team']))

    def coverage(self):
        """Free floor inside the building that lies in a cell some robot has explored."""
        cov, g = None, None
        for t in self.team.values():
            g = t['grid']
            bits = np.unpackbits(np.frombuffer(base64.b64decode(t['mine_explored']), np.uint8))
            mask = bits[:g['w'] * g['h']].reshape(g['h'], g['w']).astype(bool)
            cov = mask if cov is None else cov | mask
        if cov is None:
            return 0.0
        i = ((self.floor[0] - g['x0']) / g['res']).astype(int)
        j = ((self.floor[1] - g['y0']) / g['res']).astype(int)
        ok = (i >= 0) & (i < g['w']) & (j >= 0) & (j < g['h'])
        inside = np.zeros(len(i), bool)
        inside[ok] = cov[j[ok], i[ok]]
        building = (self.floor[0] > 0) & (self.floor[0] < self.gt['building']['width']) & \
                   (self.floor[1] > 0) & (self.floor[1] < self.gt['building']['height'])
        return float(inside[building].mean()) if building.any() else 0.0

    def write(self, reason):
        view = self.team_view()
        hits, fps, missed = score_victims(view['victims'], self.gt['victims'])
        decoys = self.gt['decoys']
        hazard_hits = [h for h in view['hazards']
                       if any(math.hypot(h['x'] - d['x'], h['y'] - d['y']) < 1.5 for d in decoys)]
        decoy_as_victim = [f for f in fps if any(math.hypot(f['x'] - d['x'], f['y'] - d['y']) < 1.5
                                                 for d in decoys)]
        by_kind = {}
        for kind in ('visible', 'buried'):
            total = sum(1 for g in self.gt['victims'] if g['kind'] == kind)
            found = sum(1 for h in hits if h['kind_true'] == kind)
            by_kind[kind] = {'found': found, 'total': total}
        robots = {}
        for r in self.robots:
            st = self.status.get(r, {})
            sp = next(s for s in self.gt['spawns'] if s['name'] == r)
            home = self.last.get(r)
            robots[r] = {'state': st.get('state'), 'distance_m': round(self.dist[r], 1),
                         'explored_m2_own': st.get('explored_m2'),
                         'worst_health': self.health[r][0], 'worst_health_at_s': self.health[r][1],
                         'tipped_over': self.tipped.get(r),
                         'rooms_owned_at_end': sum(1 for room in self.team_view().get('rooms', [])
                                                   if room.get('owner') == r),
                         'back_at_start': bool(home and math.hypot(home[0] - sp['x'], home[1] - sp['y']) < 1.5)}
        result = {
            'seed': self.seed, 'end': reason,
            'mission_time_s': max([s.get('elapsed_s', 0) for s in self.status.values()] or [0]),
            'victims': {'found': len(hits), 'total': len(self.gt['victims']), 'by_kind': by_kind,
                        'false_positives': len(fps), 'hits': hits, 'missed': missed,
                        'false_positive_list': fps,
                        'mean_error_m': round(float(np.mean([h['error_m'] for h in hits])), 2) if hits else None},
            'hazards': {'decoys': len(decoys), 'reported_as_hazard': len(hazard_hits),
                        'reported_as_victim': len(decoy_as_victim)},
            'coverage_of_floor': round(self.coverage(), 3),
            'robots': robots,
        }
        os.makedirs(os.path.dirname(self.out) or '.', exist_ok=True)
        with open(self.out + '.json', 'w') as f:
            json.dump(result, f, indent=2)
        with open(self.out + '.md', 'w') as f:
            f.write(markdown(result))
        self.get_logger().info(f'evaluation written to {self.out}.md ({reason})')
        print(markdown(result), flush=True)
        raise SystemExit


def markdown(r):
    v, h = r['victims'], r['hazards']
    lines = [f"# Mission evaluation, seed {r['seed']}", '',
             f"Ended: {r['end']}; mission time {r['mission_time_s'] / 60:.1f} min; "
             f"coverage of the building's floor {100 * r['coverage_of_floor']:.0f} %.", '',
             '| | found | total |', '|---|---|---|']
    for kind, k in v['by_kind'].items():
        lines.append(f"| {kind} victims | {k['found']} | {k['total']} |")
    lines += [f"| **all victims** | **{v['found']}** | **{v['total']}** |", '',
              f"False positives: {v['false_positives']}; mean position error "
              f"{v['mean_error_m']} m. Hot decoys: {h['reported_as_hazard']}/{h['decoys']} reported as "
              f"hazard, {h['reported_as_victim']} mistaken for a victim.", '',
              '| victim | true kind | reported as | error (m) | confidence | found by | first seen (s) |',
              '|---|---|---|---|---|---|---|']
    for x in v['hits']:
        lines.append(f"| {x['truth']} | {x['kind_true']} | {x['kind_reported']} | {x['error_m']} | "
                     f"{x['confidence']:.2f} | {x['found_by']} | {x['first_seen_s']} |")
    for m in v['missed']:
        lines.append(f"| {m['id']} | {m['kind']} | **missed** | | | | |")
    lines += ['', '| robot | final state | walked (m) | back at start | worst health | rooms at end |',
              '|---|---|---|---|---|---|']
    for name, rb in r['robots'].items():
        hl = rb.get('worst_health', 'ok')
        if hl != 'ok' and rb.get('worst_health_at_s') is not None:
            hl += f" (from {rb['worst_health_at_s']:.0f} s)"
        lines.append(f"| {name} | {rb['state']} | {rb['distance_m']} | {'yes' if rb['back_at_start'] else 'no'} | "
                     f"{hl} | {rb.get('rooms_owned_at_end', '')} |")
    for name, rb in r['robots'].items():
        tp = rb.get('tipped_over')
        if tp:
            lines.append(f"\n{name} tipped over at ({tp['x']}, {tp['y']}) after {tp['at_s']} s "
                         f"(roll {tp['roll_deg']} deg, pitch {tp['pitch_deg']} deg).")
    return '\n'.join(lines) + '\n'


def main():
    rclpy.init()
    node = Evaluator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        if not node.finished:                 # stopped early: still report what we have
            node.finished = True
            try:
                node.write(reason='interrupted')
            except SystemExit:
                pass


if __name__ == '__main__':
    main()
