"""Wall guard: the last check between Nav2 and the wheels.

The planner treats the robot as a point kept half a body width from walls, and pure
pursuit cuts corners, so the outline sometimes brushes a wall or a door jamb and the
robot pushes against it until the progress checker gives up. This node looks at the 3D
lidar every cycle and corrects Nav2's command:

  - something within `margin` of a side: steer away from it (towards the middle when
    both sides are close, as in a doorway) and slow down;
  - something just ahead of the front edge: stop driving forward (turning still works);
  - turning in place with something inside the circle the corners sweep: creep away
    from it (forward if it is behind, back if it is in front);
  - pushing forward without moving while touching something: back off and turn away
    for a moment, then hand back to Nav2.

Only Nav2's path following passes through here; the explorer's escape moves, Nav2's
recoveries and the operator's teleop drive cmd_vel directly.

  in   cmd_vel_raw    geometry_msgs/Twist (velocity smoother output)
       lidar/points   sensor_msgs/PointCloud2 (lidar frame, body-fixed, roughly level)
       odom           nav_msgs/Odometry (to tell pushing from moving)
  out  cmd_vel

The lidar ignores returns closer than 0.4 m, so a wall right beside the body is only
seen a little ahead and behind; those points still give its distance from the side.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

INF = float('inf')


@dataclass
class Clearance:
    left: float = INF       # m from the left side of the outline to the nearest point beside it
    right: float = INF
    front: float = INF      # m from the front edge to the nearest point ahead, within the width
    sweep_front: bool = False   # something the corners would hit turning in place, front half
    sweep_rear: bool = False


def obstacle_points(xyz, lidar_height, min_height, max_height, reach=1.0):
    """Nx3 lidar-frame points -> Nx2 (x, y) of things between min_height and max_height
    above the ground, within `reach` m of the body centre."""
    if len(xyz) == 0:
        return np.zeros((0, 2))
    z = xyz[:, 2] + lidar_height
    keep = (z > min_height) & (z < max_height) & (np.abs(xyz[:, 0]) < reach) & (np.abs(xyz[:, 1]) < reach)
    return xyz[keep, :2]


def clearances(pts, half_l, half_w, sweep_margin=0.03):
    """Distances from the rectangular outline (centred on the origin, x forward) to the
    points around it. Points inside the outline are the robot itself and ignored."""
    c = Clearance()
    if len(pts) == 0:
        return c
    x, y = pts[:, 0], pts[:, 1]
    outside = (np.abs(x) > half_l) | (np.abs(y) > half_w)
    beside = outside & (np.abs(x) <= half_l + 0.1)
    left = beside & (y > half_w)
    right = beside & (y < -half_w)
    ahead = outside & (x > half_l) & (np.abs(y) <= half_w)
    if left.any():
        c.left = float((y[left] - half_w).min())
    if right.any():
        c.right = float((-y[right] - half_w).min())
    if ahead.any():
        c.front = float((x[ahead] - half_l).min())
    swept = outside & (np.hypot(x, y) < math.hypot(half_l, half_w) + sweep_margin)
    c.sweep_front = bool((swept & (x > 0)).any())
    c.sweep_rear = bool((swept & (x <= 0)).any())
    return c


def guard(vx, wz, c, margin=0.12, gain=5.0, max_bias=0.8, slow=0.15, stop=0.06, creep=0.08):
    """Nav2's (vx, wz) -> corrected (vx, wz)."""
    if vx > 0.02:
        near = min(c.left, c.right)
        if near < margin:
            # left closer -> turn right (negative); both close -> towards the middle
            bias = gain * (min(c.left, margin) - min(c.right, margin))
            wz += max(-max_bias, min(max_bias, bias))
            vx = min(vx, max(slow, vx * max(near, 0.0) / margin))
        if c.front < stop:
            vx = 0.0
        return vx, wz
    if abs(wz) > 0.2 and abs(vx) <= 0.02 and c.sweep_front != c.sweep_rear:
        return (-creep if c.sweep_front else creep), wz
    return vx, wz


class StallWatch:
    """Pushing forward for `window` s without moving `min_move` m while something is
    within `touch` of the outline: back off for `backoff` s, turning away from it."""

    def __init__(self, window=2.5, min_move=0.05, touch=0.08, backoff=1.2, cooldown=3.0,
                 back_speed=0.15, back_turn=0.5):
        self.window, self.min_move, self.touch = window, min_move, touch
        self.backoff, self.cooldown = backoff, cooldown
        self.back_speed, self.back_turn = back_speed, back_turn
        self.since = None           # (t, x, y) when the current forward push started
        self.until = -INF           # backing off until this time
        self.turn = 0.0

    def update(self, t, vx, xy, c):
        """Returns (vx, wz) to send instead while backing off, else None."""
        if t < self.until:
            return -self.back_speed, self.turn
        if vx <= 0.1 or xy is None:
            self.since = None
            return None
        if self.since is None or math.hypot(xy[0] - self.since[1], xy[1] - self.since[2]) > self.min_move:
            self.since = (t, *xy)
            return None
        touching = min(c.left, c.right, c.front) < self.touch
        if touching and t - self.since[0] > self.window and t > self.until + self.cooldown:
            # backing up while turning right swings the front to the right: away from a left wall
            self.turn = -self.back_turn if c.left <= c.right else self.back_turn
            self.until = t + self.backoff
            self.since = None
            return -self.back_speed, self.turn
        return None


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    from rescue_navigation.traversability import cloud_xyz

    class WallGuard(Node):
        def __init__(self):
            super().__init__('wall_guard')
            p = lambda n, v: self.declare_parameter(n, v).value
            self.half_l, self.half_w = p('half_length', 0.40), p('half_width', 0.30)
            self.lidar_height = p('lidar_height', 0.54)
            self.min_h, self.max_h = p('min_height', 0.15), p('max_height', 1.2)
            self.margin = p('margin', 0.12)
            self.c, self.cloud_t, self.xy = Clearance(), -INF, None
            self.stall = StallWatch()
            self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
            self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)
            self.create_subscription(PointCloud2, 'lidar/points', self.on_cloud, qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odom', self.on_odom, 10)
            self.create_timer(0.05, self.tick)
            self.backing = False

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_cloud(self, msg):
            pts = obstacle_points(cloud_xyz(msg), self.lidar_height, self.min_h, self.max_h)
            self.c, self.cloud_t = clearances(pts, self.half_l, self.half_w), self.now()

        def on_odom(self, msg):
            self.xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        def send(self, vx, wz):
            out = Twist()
            out.linear.x, out.angular.z = float(vx), float(wz)
            self.pub.publish(out)

        def on_cmd(self, msg):
            t = self.now()
            if t - self.cloud_t > 0.6:                 # no fresh lidar: pass Nav2 through
                self.pub.publish(msg)
                return
            back = self.stall.update(t, msg.linear.x, self.xy, self.c)
            if back is not None:
                if not self.backing:
                    self.get_logger().info('pushing against something: backing off')
                self.backing = True
                self.send(*back)
                return
            self.backing = False
            self.send(*guard(msg.linear.x, msg.angular.z, self.c, margin=self.margin))

        def tick(self):
            # keep backing off even if Nav2 has stopped sending
            t = self.now()
            if t < self.stall.until:
                self.send(-self.stall.back_speed, self.stall.turn)
            elif self.backing:
                self.backing = False
                self.send(0.0, 0.0)

    rclpy.init()
    node = WallGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
