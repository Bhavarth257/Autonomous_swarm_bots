"""Simulated radio channel between the robots inside the collapsed building.

This is the physics of the medium, not a coordinator: it never reads or changes the
messages, it only decides who can hear whom. A message from one robot reaches another if

  - they have line of sight (no wall or rubble between them, from the ground-truth
    occupancy map) and are within `los_range` (12 m), or
  - they are within `wall_range` (4 m) even through obstacles.

  in   /rN/radio/out          std_msgs/String (any payload)
       /rN/ground_truth/odom  true positions (the channel is physics, it may use them)
  out  /rM/radio/in           for every robot M in range of N
       /radio/links           std_msgs/String JSON, 1 Hz: links up now, robot positions
"""
import json
import os

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def load_map(yaml_path):
    """Ground-truth map -> (blocked bool grid with row 0 at the bottom, resolution, origin)."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(os.path.dirname(yaml_path), meta['image']), 'rb') as f:
        data = f.read()
    parts = data.split(b'\n', 3)                 # P5, size, maxval, pixels
    w, h = map(int, parts[1].split())
    img = np.frombuffer(parts[3], np.uint8)[:w * h].reshape(h, w)[::-1]
    return img == 0, meta['resolution'], tuple(meta['origin'][:2])


def link_ok(a, b, grid, los_range=12.0, wall_range=4.0):
    blocked, res, (ox, oy) = grid
    d = float(np.hypot(b[0] - a[0], b[1] - a[1]))
    if d > los_range:
        return False
    if d <= wall_range:
        return True
    n = max(2, int(d / (res / 2)))
    xs = np.linspace(a[0], b[0], n)
    ys = np.linspace(a[1], b[1], n)
    i = ((xs - ox) / res).astype(int)
    j = ((ys - oy) / res).astype(int)
    ok = (i >= 0) & (i < blocked.shape[1]) & (j >= 0) & (j < blocked.shape[0])
    return not blocked[j[ok], i[ok]].any()


class Radio(Node):
    def __init__(self):
        super().__init__('radio')
        self.robots = list(self.declare_parameter('robots', ['r1', 'r2', 'r3']).value)
        map_yaml = self.declare_parameter('map_yaml', '').value
        self.los_range = self.declare_parameter('los_range', 12.0).value
        self.wall_range = self.declare_parameter('wall_range', 4.0).value
        self.grid = load_map(map_yaml) if map_yaml else (np.zeros((1, 1), bool), 1.0, (0.0, 0.0))
        self.pos = {}
        self.delivered = {r: 0 for r in self.robots}
        self.pubs = {r: self.create_publisher(String, f'/{r}/radio/in', 10) for r in self.robots}
        for r in self.robots:
            self.create_subscription(String, f'/{r}/radio/out', lambda m, r=r: self.on_out(r, m), 10)
            self.create_subscription(Odometry, f'/{r}/ground_truth/odom',
                                     lambda m, r=r: self.pos.__setitem__(
                                         r, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.pub_links = self.create_publisher(String, '/radio/links', 10)
        self.create_timer(1.0, self.publish_links)

    def in_range(self, a, b):
        return a in self.pos and b in self.pos and \
            link_ok(self.pos[a], self.pos[b], self.grid, self.los_range, self.wall_range)

    def on_out(self, sender, msg):
        for r in self.robots:
            if r != sender and self.in_range(sender, r):
                self.pubs[r].publish(msg)
                self.delivered[r] += 1

    def publish_links(self):
        links = [[a, b] for k, a in enumerate(self.robots) for b in self.robots[k + 1:]
                 if self.in_range(a, b)]
        self.pub_links.publish(String(data=json.dumps({
            'links': links, 'positions': {r: [round(v, 2) for v in p] for r, p in self.pos.items()},
            'los_range': self.los_range, 'wall_range': self.wall_range})))


def main():
    rclpy.init()
    node = Radio()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
