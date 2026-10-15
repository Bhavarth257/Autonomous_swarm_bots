"""Swarm agent: one per robot, the only thing that talks over the radio.

Once a second it broadcasts the robot's view of the team (TeamView: poses, claimed
goals, mission states, explored maps, victims) and merges whatever arrives. No robot
is in charge; information spreads by gossip through whoever is in range.

  in   mission/status   own explorer (pose and goal in the map frame)
       odom/tilt        own roll/pitch (tipped over? health.py)
       nav_map          own map: what this robot has seen (-> coarse explored grid)
       victims          own detector
       radio/in         teammates' broadcasts
  out  radio/out        broadcast
       team/state       std_msgs/String JSON, 1 Hz: teammates (pose, goal, state, age of
                        the news, whether in direct radio contact), explored fractions,
                        team victim list (world frame), who searches which room
                        (rooms.py: owner, done, rectangle) and each robot's health
                        (health.py: ok / stuck / down); read by the explorer and dashboard
       team/explored    nav_msgs/OccupancyGrid (frame `world`): explored by others (100)
                        and by this robot (50), for RViz
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from rescue_navigation import grid_utils as gu
from rescue_swarm.health import OK, HealthWatch, unavailable
from rescue_swarm.knowledge import TeamView
from rescue_swarm.rooms import RoomBook, room_rects

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class Agent(Node):
    def __init__(self):
        super().__init__('swarm_agent')
        self.me = self.get_namespace().strip('/') or 'r1'
        rect = list(self.declare_parameter('building_rect', [0.0, 0.0, 18.0, 15.0]).value)
        self.map_in_world = list(self.declare_parameter('map_in_world', [0.0, 0.0, 0.0]).value)
        rate = self.declare_parameter('broadcast_rate', 1.0).value
        self.contact_timeout = self.declare_parameter('contact_timeout', 3.0).value
        self.team = TeamView(self.me, rect)
        # Rooms: the building's grid of rooms, split between the team (rooms.py).
        grid = list(self.declare_parameter('rooms_grid', [4, 4]).value)
        entrance = list(self.declare_parameter('entrance', [(rect[0] + rect[2]) / 2, rect[1]]).value)
        robots = list(self.declare_parameter('team', [self.me]).value)
        self.rooms = RoomBook(room_rects(rect, int(grid[0]), int(grid[1])), robots, entrance)
        self.health = HealthWatch()
        self.tilt = None
        self.status = None
        self.grid_msg = None
        self.sent = self.received = 0

        self.create_subscription(String, 'mission/status', self.on_status, 10)
        self.create_subscription(OccupancyGrid, 'nav_map', lambda m: setattr(self, 'grid_msg', m), LATCHED)
        self.create_subscription(String, 'victims', self.on_victims, LATCHED)
        self.create_subscription(String, 'radio/in', self.on_radio, 10)
        self.create_subscription(QuaternionStamped, 'odom/tilt', self.on_tilt, 10)
        self.pub_radio = self.create_publisher(String, 'radio/out', 10)
        self.pub_state = self.create_publisher(String, 'team/state', LATCHED)
        self.pub_explored = self.create_publisher(OccupancyGrid, 'team/explored', LATCHED)
        self.create_timer(1.0 / rate, self.broadcast)
        self.create_timer(3.0, self.update_explored)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def to_world(self, x, y):
        mx, my, myaw = self.map_in_world
        c, s = math.cos(myaw), math.sin(myaw)
        return mx + c * x - s * y, my + s * x + c * y

    # ---- inputs ----------------------------------------------------------------------
    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def on_tilt(self, msg):
        q = msg.quaternion
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.tilt = (roll, pitch)

    def on_victims(self, msg):
        self.team.add_own_victims(json.loads(msg.data), self.now())

    def on_radio(self, msg):
        data = json.loads(msg.data)
        if data.get('from') != self.me:
            self.team.receive(data, self.now())
            if 'rooms' in data:
                self.rooms.merge(data['rooms'])
            self.received += 1

    def update_explored(self):
        m = self.grid_msg
        if m is None:
            return
        g = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        X, Y = gu.cell_centers(g.shape, m.info.resolution,
                               (m.info.origin.position.x, m.info.origin.position.y))
        mx, my, myaw = self.map_in_world
        c, s = math.cos(myaw), math.sin(myaw)
        wx, wy = mx + c * X - s * Y, my + s * X + c * Y
        per_cell = (self.team.res / m.info.resolution) ** 2
        self.team.mine().update_from_points(wx.ravel(), wy.ravel(), (g != gu.UNKNOWN).ravel(), per_cell)

    # ---- outputs ---------------------------------------------------------------------
    def broadcast(self):
        t = self.now()
        xy = None
        if self.status and self.status.get('pose'):
            x, y, yaw = self.status['pose']
            wx, wy = xy = self.to_world(x, y)
            goal = self.status.get('goal')
            wgoal = self.to_world(*goal) if goal else None
            self.team.set_self((wx, wy, yaw + self.map_in_world[2]), wgoal, self.status['state'], t)
        self.update_health(t, xy)
        self.update_rooms(t)
        msg = self.team.message(t)
        msg['rooms'] = self.rooms.message()
        self.pub_radio.publish(String(data=json.dumps(msg)))
        self.sent += 1
        self.publish_state(t)

    def update_health(self, t, xy):
        before = self.health.health
        state = self.status.get('state') if self.status else None
        h = self.health.update(t, xy, state, self.tilt)
        if h != before:
            self.get_logger().info(f'health: {h}' + (f' ({self.health.reason})' if self.health.reason else ''))
        self.team.set_health(h, t)
        # teammates' health as I hear it, logged once per change
        seen = getattr(self, 'seen_health', {})
        for r, (hr, _) in self.team.health.items():
            if r != self.me and seen.get(r, OK) != hr:
                self.get_logger().info(f'{r} reports health: {hr}')
            seen[r] = hr
        self.seen_health = seen

    def update_rooms(self, t):
        """A teammate that can't search loses its rooms to the nearest healthy robot; own
        rooms all done: take some from the busiest teammate (rooms.py)."""
        explored = self.team.explored_by_team()
        self.room_done = self.rooms.done(explored.fraction_in)
        me = self.team.poses.get(self.me)
        if me is None or self.status is None or self.status.get('state') != 'exploring':
            return
        unav = unavailable(self.team.health, self.team.states)
        positions = {r: p[:2] for r, p in self.team.poses.items()}
        adopted = self.rooms.adopt(self.me, positions, unav, self.room_done, t)
        if adopted:
            self.get_logger().info(f'teammate(s) {", ".join(sorted(unav - {self.me}))} can\'t search: '
                                   f'took over {", ".join(adopted)}')
        busy = [p[:2] for r, p in self.team.poses.items() if r != self.me and r not in unav]
        busy += [g[:2] for r, g in self.team.goals.items() if r != self.me and r not in unav and g[0] is not None]
        taken = self.rooms.rebalance(self.me, me[:2], busy, self.room_done, t, unavailable=unav)
        if taken:
            self.get_logger().info(f'own rooms done: took over {", ".join(taken)}')

    def publish_state(self, t):
        team = {}
        for r, p in self.team.poses.items():
            g = self.team.goals.get(r)
            team[r] = {
                'pose': [round(v, 2) for v in p[:3]],
                'goal': [round(v, 2) for v in g[:2]] if g and g[0] is not None else None,
                'state': self.team.states.get(r, ['unknown'])[0],
                'news_age_s': round(t - p[3], 1),
                'in_contact': r == self.me or (t - self.team.heard.get(r, -1e9)) < self.contact_timeout,
                'health': self.team.health.get(r, [OK])[0],
            }
        others = self.team.explored_by_others()
        state = {
            'robot': self.me, 't': round(t, 1), 'team': team,
            'explored_fraction': {'mine': round(self.team.mine().fraction(), 3),
                                  'team': round(self.team.explored_by_team().fraction(), 3)},
            'others_explored': others.encode(), 'mine_explored': self.team.mine().encode(),
            'team_explored': self.team.explored_by_team().encode(), 'grid': {
                'x0': others.x0, 'y0': others.y0, 'res': others.res, 'w': others.w, 'h': others.h},
            'victims': [v.to_dict() for v in self.team.victims.confirmed()],
            'hazards': [v.to_dict() for v in self.team.hazards.hazards()],
            'radio': {'sent': self.sent, 'received': self.received},
            'health_reason': self.health.reason,
            'rooms': [{'key': k, 'owner': self.rooms.owner(k), 't': round(self.rooms.book[k][1], 1),
                       'done': bool(getattr(self, 'room_done', {}).get(k)),
                       'rect': [round(v, 2) for v in r]} for k, r in self.rooms.rooms.items()],
        }
        self.pub_state.publish(String(data=json.dumps(state)))

        msg = OccupancyGrid()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = others.res
        msg.info.width, msg.info.height = others.w, others.h
        msg.info.origin.position.x, msg.info.origin.position.y = others.x0, others.y0
        msg.info.origin.orientation.w = 1.0
        grid = np.zeros((others.h, others.w), dtype=np.int8)
        grid[self.team.mine().mask] = 50
        grid[others.mask] = 100
        msg.data = grid.ravel().tolist()
        self.pub_explored.publish(msg)


def main():
    rclpy.init()
    node = Agent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
