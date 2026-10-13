"""Traversability mapping for a walking hexapod.

Builds a height map (max point height per 10 cm cell) from the 3D lidar in the
robot's SLAM map frame, turns it into costs (see grid_utils.traversability) and
fuses it with the SLAM occupancy grid into the map Nav2 and the explorer use:

  in   lidar/points    3D lidar cloud
       map             SLAM occupancy grid (walls / tall obstacles from the 2D scan)
       traversability/forget_near  std_msgs/Empty: the explorer is stuck here; forget
                       what the height map holds within the robot's blind radius
  out  nav_map         fused OccupancyGrid (5 cm) over a fixed area: the building
                       outline plus the robot's start, with a margin. Fixed rather
                       than following SLAM's map, which only grows where the scan
                       hits something and so can end right under a robot standing
                       in the open; Nav2 can't plan from the edge of a costmap.
                       Outside the building, except near the start, everything is
                       lethal (geofence). Inside, cells are:
                         lethal          where SLAM or the height map says so
                         height cost     where the lidar has seen the floor
                         5, probably free  where SLAM's scan (0.4-1.5 m above the
                                         floor) saw open space but no lidar ring has
                                         touched the floor yet
                         unknown         everywhere else
       traversability  the 10 cm cost grid itself, for debugging
       terrain_ahead   std_msgs/Float32: tallest climbable step 0.25-0.9 m in front of the
                       robot, relative to the floor it stands on; the gait plugin uses it
                       to lift its feet higher and slow down before reaching debris

The cells under the robot are marked as walked-on floor: the lidar can't see
within ~0.6 m of the robot, but the robot is standing there. For the same reason an
obstacle recorded there (typically a teammate squeezing past) can never be seen to go
away; when the explorer reports being stuck, those cells are forgotten and refill as
floor until the lidar sees them again from further off.
"""
import json
import math
from array import array

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, Float32, String

from rescue_navigation import grid_utils as gu
from rescue_navigation.body_pose import BodyPose, FrameWatch, stamp_seconds

PROBABLY_FREE = 5

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def cloud_xyz(msg: PointCloud2) -> np.ndarray:
    fields = {f.name: f.offset for f in msg.fields}
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                      'offsets': [fields['x'], fields['y'], fields['z']],
                      'itemsize': msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


class Traversability(Node):
    def __init__(self):
        super().__init__('traversability')
        ns = self.get_namespace().strip('/')
        prefix = f'{ns}/' if ns else ''
        self.map_frame = self.declare_parameter('map_frame', f'{prefix}map').value
        self.base_frame = self.declare_parameter('base_frame', f'{prefix}base_footprint').value
        self.body_height = self.declare_parameter('body_height', 0.15).value
        self.res = self.declare_parameter('resolution', 0.1).value
        size = self.declare_parameter('size', 60.0).value
        self.step_max = self.declare_parameter('step_max', 0.14).value
        self.obstacle_height = self.declare_parameter('obstacle_height', 0.40).value
        self.max_range = self.declare_parameter('max_range', 8.0).value
        self.self_radius = self.declare_parameter('self_radius', 0.35).value
        # The lidar can't see the floor within ~0.6 m: unknown cells this close to where
        # the robot stands are taken as floor at its own level.
        self.blind_radius = self.declare_parameter('blind_radius', 0.7).value
        # Points this close (horizontally) to the robot's centre are its own legs: a
        # foot swinging forward reaches the lidar's lowest ring at ~0.4 m.
        self.body_radius = self.declare_parameter('body_radius', 0.55).value
        self.ceiling_cut = self.declare_parameter('ceiling_cut', 1.2).value
        # Robots are not part of the building: the team's start positions and teammates'
        # current positions never become map obstacles (live robots are still seen by Nav2's
        # obstacle layer). Without this a robot that started between two others stays boxed
        # in by their ghosts: it isn't moving, so it never re-scans the floor they stood on.
        sp = list(self.declare_parameter('robot_starts', [0.0, 0.0]).value)
        self.robot_starts = [(sp[k], sp[k + 1]) for k in range(0, len(sp) - 1, 2)]
        self.robot_clear = self.declare_parameter('robot_clearance', 0.6).value
        self.teammates = []           # world (x, y) from the swarm agent
        rate = self.declare_parameter('publish_rate', 1.0).value
        rect = list(self.declare_parameter('building_rect', [0.0, 0.0, 18.0, 15.0]).value)
        map_in_world = list(self.declare_parameter('map_in_world', [0.0, 0.0, 0.0]).value)
        self.map_in_world = map_in_world
        margin = self.declare_parameter('nav_map_margin', 3.0).value
        # Geofence: the mission area is the building. Outside it only the area around the
        # start (to reach the entrance and come back) is allowed; otherwise the planner
        # happily routes through unknown space around the outside of the building.
        self.start_zone = self.declare_parameter('start_zone_radius', 3.0).value
        self.nav_res = self.declare_parameter('nav_map_resolution', 0.05).value

        self.n = int(size / self.res)
        self.origin = (-size / 2, -size / 2)
        self.height = np.full((self.n, self.n), np.nan, dtype=np.float32)
        self._init_nav_geometry(rect, map_in_world, margin)
        self.slam = None
        self.stats = {'clouds': 0, 'used': 0, 'points': 0, 'tf_error': ''}

        self.pub_nav = self.create_publisher(OccupancyGrid, 'nav_map', LATCHED)
        self.pub_trav = self.create_publisher(OccupancyGrid, 'traversability', LATCHED)
        self.pub_ahead = self.create_publisher(Float32, 'terrain_ahead', 10)
        self.frames = FrameWatch(self, [(self.map_frame, f'{prefix}odom')])
        self.pose = BodyPose(self, self.frames, prefix, self.body_height)
        self.create_subscription(PointCloud2, 'lidar/points', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, LATCHED)
        self.create_subscription(String, 'team/state', self.on_team, LATCHED)
        self.create_subscription(Empty, 'traversability/forget_near', self.forget_near, 10)
        self.body = None              # last body position (map frame), from the lidar's pose
        self.create_timer(1.0 / rate, self.publish)
        self.create_timer(10.0, self.report)

    def on_map(self, msg):
        self.slam = msg

    def forget_near(self, _msg):
        if self.body is None:
            return
        bx, by = self.body
        k = int(self.blind_radius / self.res) + 1
        ci, cj = int((bx - self.origin[0]) / self.res), int((by - self.origin[1]) / self.res)
        j0, j1, i0, i1 = max(0, cj - k), min(self.n, cj + k + 1), max(0, ci - k), min(self.n, ci + k + 1)
        if j0 >= j1 or i0 >= i1:
            return
        jj, ii = np.ogrid[j0:j1, i0:i1]
        sub = self.height[j0:j1, i0:i1]
        sub[((ii - ci) ** 2 + (jj - cj) ** 2) * self.res ** 2 <= self.blind_radius ** 2] = np.nan
        self.get_logger().info(f'stuck: forgot the height map within {self.blind_radius} m')

    def on_team(self, msg):
        st = json.loads(msg.data)
        self.teammates = [tuple(v['pose'][:2]) for r, v in st.get('team', {}).items()
                          if r != st.get('robot') and v.get('pose')]

    def robot_spots(self):
        """Start positions and teammates, in this robot's map frame."""
        mx, my, myaw = self.map_in_world
        c, s = math.cos(-myaw), math.sin(-myaw)
        return [(c * (x - mx) - s * (y - my), s * (x - mx) + c * (y - my))
                for x, y in self.robot_starts + self.teammates]

    def _init_nav_geometry(self, rect, map_in_world, margin):
        mx, my, myaw = map_in_world
        c, s = math.cos(-myaw), math.sin(-myaw)
        pts = [(0.0, 0.0)]                                  # the start, in map frame
        for wx in (rect[0], rect[2]):
            for wy in (rect[1], rect[3]):
                dx, dy = wx - mx, wy - my
                pts.append((c * dx - s * dy, s * dx + c * dy))
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        x0, y0 = min(xs) - margin, min(ys) - margin
        self.nav_w = int(math.ceil((max(xs) + margin - x0) / self.nav_res))
        self.nav_h = int(math.ceil((max(ys) + margin - y0) / self.nav_res))
        self.nav_origin = (x0, y0)
        X, Y = gu.cell_centers((self.nav_h, self.nav_w), self.nav_res, self.nav_origin)
        ti = ((X - self.origin[0]) / self.res).astype(np.int64)
        tj = ((Y - self.origin[1]) / self.res).astype(np.int64)
        self.nav_inside = (ti >= 0) & (ti < self.n) & (tj >= 0) & (tj < self.n)
        self.nav_ti, self.nav_tj = ti, tj
        self.nav_X, self.nav_Y = X, Y
        in_building = gu.rect_region((self.nav_h, self.nav_w), self.nav_res, self.nav_origin,
                                     rect, map_in_world)
        near_start = np.hypot(X, Y) < self.start_zone
        self.fence = ~in_building & ~near_start

    def report(self):
        st = self.stats
        self.get_logger().info(
            f"clouds {st['clouds']} used {st['used']} last points {st['points']} "
            f"known cells {int((~np.isnan(self.height)).sum())} slam map {'yes' if self.slam else 'no'}"
            + (f" | last TF error: {st['tf_error']}" if st['tf_error'] else ''))

    def on_cloud(self, msg: PointCloud2):
        self.stats['clouds'] += 1
        try:
            m_b, m_c = self.pose.at(stamp_seconds(msg.header.stamp), msg.header.frame_id)
        except LookupError as e:
            self.stats['tf_error'] = str(e)
            return
        self.stats['used'] += 1
        pts = cloud_xyz(msg)
        self.stats['points'] = len(pts)
        rng = np.linalg.norm(pts, axis=1)
        pts = pts[rng < self.max_range]
        R, p = m_c
        world = pts @ R.T + p
        bx, by, bz = m_b[1]
        self.body = (bx, by)
        world = world[np.hypot(world[:, 0] - bx, world[:, 1] - by) > self.body_radius]
        # Collapsed roof slabs and ceilings are far above the robot: not obstacles.
        world = world[world[:, 2] < bz + self.ceiling_cut]
        for rx, ry in self.robot_spots():
            world = world[np.hypot(world[:, 0] - rx, world[:, 1] - ry) > self.robot_clear]
        i = ((world[:, 0] - self.origin[0]) / self.res).astype(np.int64)
        j = ((world[:, 1] - self.origin[1]) / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        i, j, z = i[ok], j[ok], world[ok, 2]

        # Per-cell max of this scan replaces what we had: obstacles that moved away
        # (other robots) clear as soon as their cell is seen again.
        scan = np.full(self.n * self.n, -np.inf)
        np.maximum.at(scan, j * self.n + i, z)
        scan = scan.reshape(self.n, self.n)
        hit = np.isfinite(scan)
        self.height[hit] = scan[hit]

        # The robot stands on the floor it occupies.
        self.publish_ahead(m_b)
        k = int(self.blind_radius / self.res)
        ci = int((bx - self.origin[0]) / self.res)
        cj = int((by - self.origin[1]) / self.res)
        j0, j1 = max(0, cj - k), min(self.n, cj + k + 1)
        i0, i1 = max(0, ci - k), min(self.n, ci + k + 1)
        if j0 < j1 and i0 < i1:
            jj, ii = np.ogrid[j0:j1, i0:i1]
            d2 = ((ii - ci) ** 2 + (jj - cj) ** 2) * self.res ** 2
            sub = self.height[j0:j1, i0:i1]       # view: edits land in self.height
            sub[(d2 <= self.blind_radius ** 2) & np.isnan(sub)] = bz
            # Within the body: anything tall is this robot's own legs.
            sub[(d2 <= self.self_radius ** 2) & (sub > bz + self.step_max)] = bz

    def publish_ahead(self, m_b):
        """Tallest step up in a corridor ahead (body frame x 0.25-0.9 m, |y| < 0.3 m)."""
        R, p = m_b
        xs, ys = np.meshgrid(np.arange(0.25, 0.9, self.res), np.arange(-0.3, 0.31, self.res))
        wx = p[0] + R[0, 0] * xs + R[0, 1] * ys
        wy = p[1] + R[1, 0] * xs + R[1, 1] * ys
        i = ((wx - self.origin[0]) / self.res).astype(np.int64)
        j = ((wy - self.origin[1]) / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        h = self.height[j[ok], i[ok]] - p[2]
        h = h[np.isfinite(h) & (h < self.step_max + 0.05)]       # climbable steps only
        self.pub_ahead.publish(Float32(data=float(max(0.0, h.max())) if len(h) else 0.0))

    def publish(self):
        trav = gu.traversability(self.height, self.step_max, self.obstacle_height)
        now = self.get_clock().now().to_msg()
        msg = OccupancyGrid()
        msg.header.stamp = now
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.res
        msg.info.width = msg.info.height = self.n
        msg.info.origin.position.x, msg.info.origin.position.y = self.origin
        msg.info.origin.orientation.w = 1.0
        msg.data = array('b', trav.tobytes())
        self.pub_trav.publish(msg)

        inside = self.nav_inside
        fused = np.full((self.nav_h, self.nav_w), gu.UNKNOWN, dtype=np.int8)
        fused[inside] = trav[self.nav_tj[inside], self.nav_ti[inside]]
        if self.slam is not None:
            info = self.slam.info
            slam = np.array(self.slam.data, dtype=np.int8).reshape(info.height, info.width)
            si = ((self.nav_X - info.origin.position.x) / info.resolution).astype(np.int64)
            sj = ((self.nav_Y - info.origin.position.y) / info.resolution).astype(np.int64)
            ok = (si >= 0) & (si < info.width) & (sj >= 0) & (sj < info.height)
            sval = np.full(ok.shape, -1, dtype=np.int16)
            sval[ok] = slam[sj[ok], si[ok]]
            # The 16 lidar rings hit the floor in circles, leaving unseen gaps between
            # them; SLAM's scan sweeps the space continuously. Floor SLAM saw as free but
            # no ring has touched yet is "probably free": walkable, not a frontier. Low
            # debris there is still caught when the robot gets within lidar view of it.
            fused[(fused == gu.UNKNOWN) & (sval >= 0) & (sval < 25)] = PROBABLY_FREE
            fused[sval >= 65] = gu.LETHAL
        for rx, ry in self.robot_spots():             # robots are not walls
            near = np.hypot(self.nav_X - rx, self.nav_Y - ry) < self.robot_clear
            fused[near & (fused == gu.LETHAL)] = PROBABLY_FREE
        fused[self.fence] = gu.LETHAL
        out = OccupancyGrid()
        out.header.stamp = now
        out.header.frame_id = self.map_frame
        out.info.resolution = self.nav_res
        out.info.width, out.info.height = self.nav_w, self.nav_h
        out.info.origin.position.x, out.info.origin.position.y = self.nav_origin
        out.info.origin.orientation.w = 1.0
        out.data = array('b', fused.tobytes())
        self.pub_nav.publish(out)


def main():
    rclpy.init()
    node = Traversability()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
