"""Swarm agent: one per robot, the only thing that talks over the radio.

Once a second it broadcasts the robot's view of the team (TeamView: poses, claimed
goals, mission states, explored maps, victims) and merges whatever arrives. No robot
is in charge; information spreads by gossip through whoever is in range.

  in   mission/status   own explorer (pose and goal in the map frame)
       nav_map          own map: what this robot has seen (-> coarse explored grid)
       victims          own detector
       radio/in         teammates' broadcasts
  out  radio/out        broadcast
       team/state       std_msgs/String JSON, 1 Hz: teammates (pose, goal, state, age of
                        the news, whether in direct radio contact), explored fractions,
                        team victim list (world frame); read by the explorer and dashboard
       team/explored    nav_msgs/OccupancyGrid (frame `world`): explored by others (100)
                        and by this robot (50), for RViz
"""
import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from rescue_navigation import grid_utils as gu
from rescue_swarm.knowledge import TeamView

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
        self.status = None
        self.grid_msg = None
        self.sent = self.received = 0

        self.create_subscription(String, 'mission/status', self.on_status, 10)
        self.create_subscription(OccupancyGrid, 'nav_map', lambda m: setattr(self, 'grid_msg', m), LATCHED)
        self.create_subscription(String, 'victims', self.on_victims, LATCHED)
        self.create_subscription(String, 'radio/in', self.on_radio, 10)
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

    def on_victims(self, msg):
        self.team.add_own_victims(json.loads(msg.data), self.now())

    def on_radio(self, msg):
        data = json.loads(msg.data)
        if data.get('from') != self.me:
            self.team.receive(data, self.now())
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
        if self.status and self.status.get('pose'):
            x, y, yaw = self.status['pose']
            wx, wy = self.to_world(x, y)
            goal = self.status.get('goal')
            wgoal = self.to_world(*goal) if goal else None
            self.team.set_self((wx, wy, yaw + self.map_in_world[2]), wgoal, self.status['state'], t)
        self.pub_radio.publish(String(data=json.dumps(self.team.message(t))))
        self.sent += 1
        self.publish_state(t)

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
