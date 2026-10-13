"""Frontier exploration mission for one robot.

States: IDLE -> (start) -> EXPLORING -> (no frontiers left / time limit / return_home)
        -> RETURNING -> DONE.   stop -> STOPPED (start resumes exploring).
Operator intervention (from the dashboard):
        takeover -> MANUAL      autonomy paused; the operator drives cmd_vel (teleop)
        goto (point, world frame) -> OPERATOR_GOAL -> MANUAL when reached
        resume -> EXPLORING

Frontiers are walkable cells of nav_map next to unknown cells, inside the building
outline (the outline is known to the rescuers; the robot starts outside it). The
robot drives to the frontier cluster with the best size-versus-distance score.
Frontiers it can't reach, or that stay unresolved after it got there, are
blacklisted so the mission always ends.

  in   nav_map                   fused map from the traversability node
       team/state                swarm agent (optional): teammates' positions and claimed
                                 goals (avoided) and areas they explored (skipped)
  out  mission/status            std_msgs/String, JSON, 1 Hz (for the dashboard)
       explore/markers           frontier and goal markers for RViz
  srv  mission/start, mission/stop, mission/return_home, mission/takeover,
       mission/resume   (std_srvs/Trigger)
  in   mission/goto      geometry_msgs/PointStamped, world frame: walk there (operator)
  act  navigate_to_pose          (Nav2)
"""
import base64
import json
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from rescue_navigation import grid_utils as gu
from rescue_navigation.body_pose import FrameWatch, compose, quat_rpy, rpy_matrix

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

IDLE, EXPLORING, RETURNING, DONE, STOPPED = 'idle', 'exploring', 'returning', 'done', 'stopped'
MANUAL, OPERATOR_GOAL = 'manual', 'operator goal'


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        ns = self.get_namespace().strip('/')
        self.robot = ns or 'robot'
        prefix = f'{ns}/' if ns else ''
        self.map_frame = self.declare_parameter('map_frame', f'{prefix}map').value
        self.rect = list(self.declare_parameter('building_rect', [0.0, 0.0, 18.0, 15.0]).value)
        self.map_in_world = list(self.declare_parameter('map_in_world', [0.0, 0.0, 0.0]).value)
        self.margin = self.declare_parameter('region_margin', 0.15).value
        self.min_cells = self.declare_parameter('min_frontier_cells', 6).value
        # Slightly optimistic: Nav2 makes the final call on tight gaps.
        self.robot_radius = self.declare_parameter('robot_radius', 0.20).value
        self.blacklist_radius = self.declare_parameter('blacklist_radius', 0.6).value
        # Blacklisted spots get another chance later: what blocked them may have been a
        # teammate walking past, or a costmap glitch.
        self.blacklist_ttl = self.declare_parameter('blacklist_ttl', 90.0).value
        self.stuck_time = self.declare_parameter('stuck_time', 20.0).value
        self.time_limit = self.declare_parameter('time_limit', 1200.0).value
        # Be back by the time limit: head home once the walk back (straight line x detour,
        # at the average speed through rubble, plus a margin) would take what's left.
        self.return_speed = self.declare_parameter('return_speed', 0.15).value
        self.return_detour = self.declare_parameter('return_detour', 1.5).value
        self.return_margin = self.declare_parameter('return_margin', 30.0).value
        self.done_checks = self.declare_parameter('empty_checks_to_finish', 10).value
        # Never call the building explored before having mapped at least this much of it.
        self.min_explored = self.declare_parameter('min_explored_m2', 10.0).value
        # With a team: don't call it done until the team has seen this much of the building.
        self.finish_team_coverage = self.declare_parameter('finish_team_coverage', 0.85).value
        auto_start = self.declare_parameter('auto_start', False).value

        self.grid = None
        self.info = None
        self._region_key = None
        self.region = None
        self.state = IDLE
        self.goal = None              # (x, y) in map frame
        self.goal_handle = None
        self.pending = False          # goal sent, Nav2 hasn't accepted it yet
        self.deferred = None          # (x, y, kind) to send once Nav2's action server is up
        self.goal_sent_at = 0.0
        self.goal_sent_pose = None
        self.blacklist = []
        self.empty_checks = 0
        self.started_at = None
        self.elapsed_before = 0.0
        self.distance = 0.0
        self.pose = None
        self.last_progress = (None, 0.0)   # (pose, time) of the last real movement
        self.n_frontiers = 0
        self.frontier_points = []
        self.message = 'waiting for start'

        self.odom_frame = f'{prefix}odom'
        self.frames = FrameWatch(self, [(self.map_frame, self.odom_frame)])
        self.odom = None
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'odom', m), 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # If Nav2's bringup failed (goals get rejected), ask its lifecycle manager to retry.
        self.nav2_manager = self.create_client(ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes')
        self.rejections = 0
        self.nav2_restart_at = -1e9
        self.nav2_missing_since = None   # when Nav2's action server was first found missing
        self.create_subscription(OccupancyGrid, 'nav_map', self.on_map, LATCHED)
        self.team_others = []          # world points: teammates and their goals
        self.team_grid = None          # (mask, x0, y0, res): explored by teammates
        self.team_union = None         # (mask, x0, y0, res): explored by anyone
        self.spread = self.declare_parameter('teammate_spread', 3.0).value
        self.create_subscription(String, 'team/state', self.on_team, LATCHED)
        self.pub_status = self.create_publisher(String, 'mission/status', 10)
        # Unstick: after repeated quick goal failures (typically the robot brushing a
        # door jamb, inside the costmap's inscribed zone so nothing can be planned) it
        # crab-walks away from the nearest obstacles for a moment.
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_forget = self.create_publisher(Empty, 'traversability/forget_near', 10)
        self.fail_streak = 0
        self.escapes = []              # (x, y) of recent escapes: vary the direction if stuck again
        self.escape_cmd = None
        self.escape_until = 0.0
        self.create_timer(0.1, self.escape_tick)
        self.pub_markers = self.create_publisher(MarkerArray, 'explore/markers', 10)
        self.create_service(Trigger, 'mission/start', self.srv_start)
        self.create_service(Trigger, 'mission/stop', self.srv_stop)
        self.create_service(Trigger, 'mission/return_home', self.srv_return)
        self.create_service(Trigger, 'mission/takeover', self.srv_takeover)
        self.create_service(Trigger, 'mission/resume', self.srv_resume)
        self.create_subscription(PointStamped, 'mission/goto', self.on_goto, 10)
        self.create_timer(1.0, self.step)
        if auto_start:
            self.start()

    def now(self):
        """Simulation time in seconds (wall time when not simulated)."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- services ----------------------------------------------------------
    def start(self):
        if self.state in (EXPLORING, RETURNING):
            return False, f'already {self.state}'
        self.state = EXPLORING
        self.started_at = self.now()
        self.empty_checks = 0
        self.message = 'exploring'
        self.get_logger().info('mission started')
        return True, 'exploring'

    def srv_start(self, _req, res):
        res.success, res.message = self.start()
        return res

    def srv_stop(self, _req, res):
        self.cancel_goal()
        self._pause_clock()
        self.state = STOPPED
        self.message = 'stopped by operator'
        res.success, res.message = True, 'stopped'
        return res

    def srv_return(self, _req, res):
        self.go_home('return requested by operator')
        res.success, res.message = True, 'returning'
        return res

    def srv_takeover(self, _req, res):
        self.cancel_goal()
        self.escape_cmd = None
        if self.started_at is None and self.state in (IDLE, DONE, STOPPED):
            self.started_at = self.now()
        self.state = MANUAL
        self.message = 'operator in control (teleop)'
        self.get_logger().info('operator took over')
        res.success, res.message = True, 'manual'
        return res

    def srv_resume(self, _req, res):
        self.cancel_goal()
        self.state = EXPLORING
        self.empty_checks = 0
        if self.started_at is None:
            self.started_at = self.now()
        self.message = 'autonomy resumed'
        self.get_logger().info('autonomy resumed by operator')
        res.success, res.message = True, 'exploring'
        return res

    def on_goto(self, msg: PointStamped):
        """Operator goal in the world (building) frame -> this robot's map frame."""
        mx, my, myaw = self.map_in_world
        c, s = math.cos(-myaw), math.sin(-myaw)
        dx, dy = msg.point.x - mx, msg.point.y - my
        if self.started_at is None:
            self.started_at = self.now()
        self.state = OPERATOR_GOAL
        self.message = f'going to operator point ({msg.point.x:.1f}, {msg.point.y:.1f})'
        self.get_logger().info(self.message)
        self.send_goal(c * dx - s * dy, s * dx + c * dy, 'operator')

    def _pause_clock(self):
        if self.started_at is not None:
            self.elapsed_before += self.now() - self.started_at
            self.started_at = None

    # ---- inputs ------------------------------------------------------------
    def on_map(self, msg: OccupancyGrid):
        self.info = msg.info
        self.grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        key = (msg.info.width, msg.info.height, msg.info.resolution,
               msg.info.origin.position.x, msg.info.origin.position.y)
        if key != self._region_key:
            self._region_key = key
            self.region = gu.rect_region(self.grid.shape, msg.info.resolution,
                                         (msg.info.origin.position.x, msg.info.origin.position.y),
                                         self.rect, self.map_in_world, self.margin)

    def on_team(self, msg):
        st = json.loads(msg.data)
        self.team_others = []
        for r, v in st.get('team', {}).items():
            if r == st.get('robot'):
                continue
            self.team_others.append(tuple(v['pose'][:2]))
            if v.get('goal'):
                self.team_others.append(tuple(v['goal']))
        g = st.get('grid')
        if g:
            def decode(key):
                bits = np.unpackbits(np.frombuffer(base64.b64decode(st[key]), np.uint8))
                return bits[:g['w'] * g['h']].reshape(g['h'], g['w']).astype(bool)
            self.team_grid = (decode('others_explored'), g['x0'], g['y0'], g['res'])
            if 'team_explored' in st:
                self.team_union = (decode('team_explored'), g['x0'], g['y0'], g['res'])

    def to_world(self, x, y):
        mx, my, myaw = self.map_in_world
        c, s = math.cos(myaw), math.sin(myaw)
        return mx + c * x - s * y, my + s * x + c * y

    def explored_by_team(self, wx, wy):
        if self.team_grid is None:
            return False
        mask, x0, y0, res = self.team_grid
        i, j = int((wx - x0) / res), int((wy - y0) / res)
        return 0 <= j < mask.shape[0] and 0 <= i < mask.shape[1] and bool(mask[j, i])

    def update_pose(self):
        if self.odom is None:
            return False
        try:
            m_o = self.frames.get(self.map_frame, self.odom_frame)
        except LookupError:
            return False
        p = self.odom.pose.pose.position
        o_yaw = quat_rpy(self.odom.pose.pose.orientation)[2]
        R, t = compose(m_o, (rpy_matrix(0.0, 0.0, o_yaw), np.array([p.x, p.y, p.z])))
        new = (float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0]))
        if self.pose is not None:
            self.distance += math.hypot(new[0] - self.pose[0], new[1] - self.pose[1])
        self.pose = new
        return True

    def time_to_home(self):
        walk = math.hypot(self.pose[0], self.pose[1]) * self.return_detour / self.return_speed
        return walk + self.return_margin

    # ---- main loop -----------------------------------------------------------
    def step(self):
        have_pose = self.update_pose()
        if self.deferred and self.nav.server_is_ready():
            self.send_goal(*self.deferred)
        if self.state in (EXPLORING, RETURNING) and have_pose:
            if self.state == EXPLORING and self.elapsed() + self.time_to_home() > self.time_limit:
                self.go_home('time limit reached' if self.elapsed() > self.time_limit
                             else 'time to head back to be out by the time limit')
            elif self.state == EXPLORING and self.grid is not None:
                self.explore()
            self.check_stuck()
        self.publish_status()

    def explore(self):
        if self.escape_cmd is not None:
            return
        res = self.info.resolution
        ox, oy = self.info.origin.position.x, self.info.origin.position.y
        rx, ry = self.pose[0], self.pose[1]
        # Only frontiers the robot can actually get to: behind a rubble-blocked door they
        # would cost a minute of failed planning each.
        reach = gu.reachable(self.grid, (int((ry - oy) / res), int((rx - ox) / res)),
                             int(round(self.robot_radius / res)))
        near = gu.ndimage.binary_dilation(reach, structure=gu.disk(int(0.3 / res)))
        mask = gu.frontier_mask(self.grid, self.region) & near
        found = []
        for cl in gu.clusters(mask, self.min_cells):
            keep = np.array([not self.blacklisted(ox + (i + 0.5) * res, oy + (j + 0.5) * res)
                             for j, i in cl], dtype=bool)
            cl = cl[keep]
            if len(cl) < self.min_cells:
                continue
            # Goal: the reachable cell closest to the middle of the frontier.
            cj, ci = cl.mean(axis=0)
            on = cl[reach[cl[:, 0], cl[:, 1]]]
            if len(on):
                j, i = on[np.argmin((on[:, 0] - cj) ** 2 + (on[:, 1] - ci) ** 2)]
            else:
                j, i = gu.nearest_true(reach, (int(cj), int(ci)), int(0.6 / res))
                if j is None:
                    continue
            target = np.array([ox + (i + 0.5) * res, oy + (j + 0.5) * res])
            if self.explored_by_team(*self.to_world(*target)):
                continue                  # a teammate has already seen what lies beyond
            found.append((target, len(cl) * res))
        if not found:
            # Own map has no open edges left that a teammate hasn't covered: go to the edge of
            # what *nobody* has seen yet. The planner can cross space this robot never mapped.
            found = self.team_frontiers(reach, res, ox, oy)
        self.n_frontiers = len(found)
        self.frontier_points = [f[0] for f in found]

        if not found:
            if self.explored_m2() < self.min_explored:
                self.message = 'waiting for the first map of the building'
                return
            team = float(self.team_union[0].mean()) if self.team_union is not None else None
            if team is not None and team < self.finish_team_coverage:
                self.message = f'no open frontier right now; team has explored {100 * team:.0f} %, waiting'
                return
            self.empty_checks += 1
            if self.empty_checks >= self.done_checks and self.goal_handle is None:
                why = 'no frontiers left: building explored'
                if team is not None:
                    why += f' (team coverage {100 * team:.0f} %)'
                self.go_home(why)
            return
        self.empty_checks = 0
        self.message = f'exploring: {len(found)} frontiers'

        if self.pending and self.now() - self.goal_sent_at < 10.0:
            return
        self.pending = False
        # Keep the current goal while its frontier still exists.
        if self.goal is not None and self.goal_handle is not None:
            if any(math.hypot(p[0] - self.goal[0], p[1] - self.goal[1]) < 0.4 for p in self.frontier_points):
                return

        def score(f):
            (x, y), length = f
            s = min(length, 4.0) - math.hypot(x - rx, y - ry)
            wx, wy = self.to_world(x, y)
            # Spread out: frontiers near a teammate or its claimed goal are worth less.
            for ox_, oy_ in self.team_others:
                d = math.hypot(wx - ox_, wy - oy_)
                if d < self.spread:
                    s -= 2.0 * (self.spread - d)
            return s
        target, _ = max(found, key=score)
        self.send_goal(float(target[0]), float(target[1]), 'frontier')

    def team_frontiers(self, reach, res, ox, oy):
        """Coarse cells inside the building nobody explored, next to cells someone did,
        as (map-frame target, size) like own frontiers; targets snapped to reachable cells."""
        if self.team_union is None:
            return []
        mask, x0, y0, cres = self.team_union
        rx0, ry0, rx1, ry1 = self.rect
        h, w = mask.shape
        jj, ii = np.mgrid[0:h, 0:w]
        cx, cy = x0 + (ii + 0.5) * cres, y0 + (jj + 0.5) * cres
        inside = (cx > rx0 + 0.3) & (cx < rx1 - 0.3) & (cy > ry0 + 0.3) & (cy < ry1 - 0.3)
        m = np.pad(mask, 1)
        next_to_known = m[:-2, 1:-1] | m[2:, 1:-1] | m[1:-1, :-2] | m[1:-1, 2:]
        cand = inside & ~mask & next_to_known
        mx, my, myaw = self.map_in_world
        c, s = math.cos(-myaw), math.sin(-myaw)
        rx, ry = (self.pose[0], self.pose[1]) if self.pose else (0.0, 0.0)
        out, unverified = [], []
        for cl in gu.clusters(cand, 2):
            # Aim at the nearest point of this stretch of unexplored edge: the edge often
            # rings the explored area, and the middle of a ring is already explored.
            wx, wy = cx[cl[:, 0], cl[:, 1]], cy[cl[:, 0], cl[:, 1]]
            px, py = c * (wx - mx) - s * (wy - my), s * (wx - mx) + c * (wy - my)   # world -> map
            ok = np.array([not self.blacklisted(a, b) for a, b in zip(px, py)], dtype=bool)
            if not ok.any():
                continue
            px, py = px[ok], py[ok]
            k = int(np.argmin(np.hypot(px - rx, py - ry)))
            j, i = gu.nearest_true(reach, (int((py[k] - oy) / res), int((px[k] - ox) / res)), int(1.0 / res))
            if j is None:
                # Not reachable as far as this robot's own map knows (it may be boxed in by
                # teammates that have since walked off, or just not have seen the way).
                unverified.append((np.array([px[k], py[k]]), len(cl) * cres))
                continue
            out.append((np.array([ox + (i + 0.5) * res, oy + (j + 0.5) * res]), len(cl) * cres))
        if not out and unverified:
            # Nothing verified: let Nav2 try the nearest one (a failure is blacklisted).
            out = [min(unverified, key=lambda f: math.hypot(f[0][0] - rx, f[0][1] - ry))]
        return out

    def blacklisted(self, x, y):
        now = self.now()
        return any(math.hypot(x - bx, y - by) < self.blacklist_radius and now - t < self.blacklist_ttl
                   for bx, by, t in self.blacklist)

    def add_blacklist(self, goal):
        self.blacklist.append((goal[0], goal[1], self.now()))

    def check_stuck(self):
        if self.goal_handle is None or self.pose is None:
            return
        now = self.now()
        last_pose, last_t = self.last_progress
        if last_pose is None or math.hypot(self.pose[0] - last_pose[0], self.pose[1] - last_pose[1]) > 0.3:
            self.last_progress = (self.pose, now)
        elif now - last_t > self.stuck_time:
            self.get_logger().warn(f'no progress towards {self.goal}, giving up on it')
            if self.state == EXPLORING:
                self.add_blacklist(self.goal)
            self.cancel_goal()
            self.start_escape()
            self.last_progress = (self.pose, now)

    # ---- navigation ------------------------------------------------------------
    def go_home(self, why):
        self.state = RETURNING
        self.message = why
        self.get_logger().info(f'{why}; returning to the entrance')
        self.send_goal(0.0, 0.0, 'home')

    def send_goal(self, x, y, kind):
        if not self.nav.server_is_ready():
            # Keep it (operator points and the way home must not be lost); step() retries.
            if kind != 'frontier':
                self.deferred = (x, y, kind)
            self.message = 'waiting for Nav2'
            # A Nav2 bringup that aborted (a lifecycle service timing out at startup) never
            # brings the action server up, so no goal is ever rejected either: restart it.
            if self.nav2_missing_since is None:
                self.nav2_missing_since = self.now()
            elif self.now() - self.nav2_missing_since > 30.0:
                self.restart_nav2()
            return
        self.nav2_missing_since = None
        self.deferred = None
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = x, y
        yaw = math.atan2(y - self.pose[1], x - self.pose[0]) if self.pose else 0.0
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        goal.pose = ps
        self.goal = (x, y)
        self.goal_sent_at = self.now()
        self.goal_sent_pose = self.pose
        self.last_progress = (self.pose, self.now())
        self.pending = True
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(lambda f, g=(x, y), k=kind: self.on_goal_response(f, g, k))

    def on_goal_response(self, future, goal, kind):
        handle = future.result()
        if self.goal == goal:
            self.pending = False
        if not handle.accepted:
            # Rejected means Nav2 isn't active (yet), not that the spot is unreachable:
            # don't blacklist it; restart Nav2 if it keeps happening.
            self.get_logger().warn(f'{kind} goal {goal} rejected (Nav2 not active yet?)')
            self.goal = None
            if kind != 'frontier':
                self.deferred = (goal[0], goal[1], kind)     # retried by step()
            self.rejections += 1
            if self.rejections >= 5:
                self.restart_nav2()
            return
        if self.goal != goal:          # superseded while we waited
            return
        self.goal_handle = handle
        self.rejections = 0
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, goal, kind))

    def on_result(self, future, goal, kind):
        status = future.result().status
        if self.goal != goal:
            return                     # an older goal that was replaced
        self.goal_handle = None
        # Failed fast, or failed without the robot getting anywhere (Nav2's recoveries can
        # take longer than 15 s, and each new goal restarts the no-progress timer, so a
        # boxed-in robot would otherwise cycle through goals forever).
        stayed = (self.pose is not None and self.goal_sent_pose is not None and
                  math.hypot(self.pose[0] - self.goal_sent_pose[0], self.pose[1] - self.goal_sent_pose[1]) < 0.3)
        quick_fail = status == GoalStatus.STATUS_ABORTED and (self.now() - self.goal_sent_at < 15.0 or stayed)
        self.fail_streak = self.fail_streak + 1 if quick_fail else 0
        if self.fail_streak >= 3:
            self.start_escape()
        if kind == 'operator':
            if self.state == OPERATOR_GOAL:
                self.state = MANUAL
                self.message = ('reached the operator point, holding' if status == GoalStatus.STATUS_SUCCEEDED
                                else 'could not reach the operator point, holding')
            return
        if kind == 'home':
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.state = DONE
                self._pause_clock()
                self.message = f'mission complete: {self.message}'
                self.get_logger().info('back at the entrance, mission complete')
            elif self.state == RETURNING and self.escape_cmd is None:
                self.go_home(self.message)          # keep trying
            return
        # Reached or failed: either way this spot can't yield more, unless the
        # frontier vanished (then it's simply gone from the next search).
        if status != GoalStatus.STATUS_CANCELED:
            self.add_blacklist(goal)
        self.goal = None

    def start_escape(self, seconds=4.0, speed=0.3):
        """Walk (sideways if need be) away from lethal cells within 0.6 m. Obstacles that
        close may be stale (the lidar can't see there), so the height map forgets them."""
        self.fail_streak = 0
        self.pub_forget.publish(Empty())
        if self.grid is None or self.pose is None:
            return
        res = self.info.resolution
        ox, oy = self.info.origin.position.x, self.info.origin.position.y
        rx, ry, yaw = self.pose
        k = int(0.6 / res)
        ci, cj = int((rx - ox) / res), int((ry - oy) / res)
        win = self.grid[max(0, cj - k):cj + k + 1, max(0, ci - k):ci + k + 1]
        jj, ii = np.nonzero(win >= gu.LETHAL)
        dx, dy = -math.cos(yaw), -math.sin(yaw)          # default: back up
        if len(jj):
            px = ox + (ii + max(0, ci - k) + 0.5) * res - rx
            py = oy + (jj + max(0, cj - k) + 0.5) * res - ry
            d2 = np.maximum(px * px + py * py, 1e-4)
            ax, ay = -(px / d2).sum(), -(py / d2).sum()    # push away, nearest strongest
            if math.hypot(ax, ay) > 1e-6:
                dx, dy = ax, ay
        # Stuck here before? Turn the escape direction 60 degrees further each time.
        repeats = sum(1 for ex, ey in self.escapes if math.hypot(ex - rx, ey - ry) < 0.6)
        self.escapes = (self.escapes + [(rx, ry)])[-20:]
        if repeats:
            a = math.radians(60 * repeats)
            dx, dy = dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)
        n = math.hypot(dx, dy)
        bx = (math.cos(yaw) * dx + math.sin(yaw) * dy) / n   # into the body frame
        by = (-math.sin(yaw) * dx + math.cos(yaw) * dy) / n
        self.cancel_goal()
        cmd = Twist()
        cmd.linear.x, cmd.linear.y = speed * bx, speed * by
        self.escape_cmd = cmd
        self.escape_until = self.now() + seconds
        self.get_logger().warn('goals keep failing here: stepping away from the obstacles')

    def escape_tick(self):
        if self.escape_cmd is None:
            return
        if self.now() >= self.escape_until:
            self.pub_cmd.publish(Twist())
            self.escape_cmd = None
            if self.state == RETURNING:
                self.go_home(self.message)
            return
        self.pub_cmd.publish(self.escape_cmd)

    def restart_nav2(self):
        """Reset then start Nav2's lifecycle nodes (at most once every 30 s)."""
        if self.now() - self.nav2_restart_at < 30.0 or not self.nav2_manager.service_is_ready():
            return
        self.nav2_restart_at = self.now()
        self.rejections = 0
        self.message = 'restarting Nav2'
        self.get_logger().warn('Nav2 is not taking goals: resetting and starting its lifecycle nodes')

        def startup(_future):
            self.nav2_manager.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP))
        self.nav2_manager.call_async(ManageLifecycleNodes.Request(
            command=ManageLifecycleNodes.Request.RESET)).add_done_callback(startup)

    def cancel_goal(self):
        self.deferred = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal = None

    # ---- outputs ----------------------------------------------------------------
    def elapsed(self):
        running = self.now() - self.started_at if self.started_at is not None else 0.0
        return self.elapsed_before + running

    def explored_m2(self):
        if self.grid is None or self.region is None:
            return 0.0
        known = (self.grid != gu.UNKNOWN) & self.region
        return float(known.sum() * self.info.resolution ** 2)

    def publish_status(self):
        status = {
            'robot': self.robot,
            'state': self.state,
            'message': self.message,
            'pose': [round(v, 2) for v in self.pose] if self.pose else None,
            'goal': [round(v, 2) for v in self.goal] if self.goal else None,
            'frontiers': self.n_frontiers,
            'blacklisted': sum(1 for b in self.blacklist if self.now() - b[2] < self.blacklist_ttl),
            'explored_m2': round(self.explored_m2(), 1),
            'distance_m': round(self.distance, 1),
            'elapsed_s': round(self.elapsed(), 1),
        }
        self.pub_status.publish(String(data=json.dumps(status)))

        markers = MarkerArray()
        m = Marker()
        m.header.frame_id = self.map_frame
        m.ns, m.id, m.type = 'frontiers', 0, Marker.SPHERE_LIST
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 1.0, 0.9
        for x, y in self.frontier_points:
            p = PoseStamped().pose.position
            p.x, p.y, p.z = float(x), float(y), 0.1
            m.points.append(p)
        markers.markers.append(m)
        g = Marker()
        g.header.frame_id = self.map_frame
        g.ns, g.id, g.type = 'goal', 1, Marker.CYLINDER
        if self.goal:
            g.pose.position.x, g.pose.position.y = self.goal
        else:
            g.action = Marker.DELETE
        g.scale.x = g.scale.y = g.scale.z = 0.4
        g.pose.position.z = 0.2
        g.pose.orientation.w = 1.0
        g.color.r, g.color.g, g.color.b, g.color.a = 1.0, 0.5, 0.0, 0.8
        markers.markers.append(g)
        self.pub_markers.publish(markers)


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
