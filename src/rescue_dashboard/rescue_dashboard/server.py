"""Web mission control: http://localhost:8080

A ROS 2 node (background thread) gathers what the robots publish; an aiohttp server
(main thread) serves the page, a WebSocket with the live state, MJPEG camera streams,
and takes the operator's commands:

  state (WebSocket, 4 Hz)   robots (state, world pose, speed, distance, explored, radio
                            contact, goal), team victims and hazards (merged across
                            robots), radio links, coverage, mission clock, events,
                            time series for the charts
  /api/map.png              all robots' maps combined in the building frame
  /api/frame/<r>/<kind>.jpg latest frame: rgb (with detection boxes), thermal (false colour)
  /api/stream/<r>/<kind>    the same as an MJPEG stream (for other viewers)
  /api/snapshot/<n>.jpg     camera image from the moment victim n was first reported
  commands (WebSocket)      start / stop / return_home / takeover / resume (one robot or
                            all), goto (world point), teleop (velocities, only while that
                            robot is under manual control), spawn_victim (live, into the
                            running Gazebo world and the ground truth), save_report,
                            save_scenario (victims file for victims_file:=...)

Everything shown comes from the robots' own topics; the ground-truth file is read only
for the building outline, entrance and start poses (the rescuers' plan) and is updated
when the operator places a victim.
"""
import asyncio
import datetime
import json
import math
import os
import random
import subprocess
import threading
import time
import webbrowser

import cv2
import numpy as np
import rclpy
from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
MERGE_RADIUS = 3.0          # same as the trackers: one marker per person
COMMANDS = ('start', 'stop', 'return_home', 'takeover', 'resume')


def to_numpy(msg: Image):
    if msg.encoding in ('mono16', '16UC1'):
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    return img[:, :, :3] if msg.encoding in ('rgb8', 'rgba8') else img[:, :, 2::-1]


def thermal_colour(centi_kelvin):
    t = centi_kelvin.astype(np.float32) / 100.0
    norm = np.clip((t - 290.0) / (320.0 - 290.0) * 255, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)                # BGR


class MissionNode(Node):
    """Collects the robots' topics; all attributes are written by ROS callbacks."""

    def __init__(self, robots, world):
        super().__init__('mission_dashboard')
        self.robots, self.world = robots, world
        self.lock = threading.Lock()
        self.status, self.speed, self.maps, self.team = {}, {}, {}, {}
        self.images = {}          # (robot, kind) -> Image msg; kind: rgb, det, thermal
        self.links = {}
        self.clients_ = {r: {k: self.create_client(Trigger, f'/{r}/mission/{k}') for k in COMMANDS}
                         for r in robots}
        self.pub_cmd = {r: self.create_publisher(Twist, f'/{r}/cmd_vel', 10) for r in robots}
        self.pub_goto = {r: self.create_publisher(PointStamped, f'/{r}/mission/goto', 10) for r in robots}
        self.pub_truth = self.create_publisher(String, '/scenario/ground_truth', LATCHED)
        for r in robots:
            sub = self.create_subscription
            sub(String, f'/{r}/mission/status', lambda m, r=r: self.status.__setitem__(r, json.loads(m.data)), 10)
            sub(Odometry, f'/{r}/odom', lambda m, r=r: self.speed.__setitem__(
                r, math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)), 10)
            sub(OccupancyGrid, f'/{r}/nav_map', lambda m, r=r: self.maps.__setitem__(r, m), LATCHED)
            sub(String, f'/{r}/team/state', lambda m, r=r: self.team.__setitem__(r, json.loads(m.data)), LATCHED)
            for kind, topic in (('rgb', 'camera/image_raw'), ('det', 'perception/image'),
                                ('thermal', 'thermal/image_raw')):
                sub(Image, f'/{r}/{topic}', lambda m, r=r, k=kind: self.images.__setitem__((r, k), m),
                    qos_profile_sensor_data)
        self.create_subscription(String, '/radio/links', lambda m: self.links.update(json.loads(m.data)), 10)

    def call(self, robot, what):
        c = self.clients_[robot][what]
        if not c.service_is_ready():
            return 'not ready'
        c.call_async(Trigger.Request())
        return 'sent'


class Dashboard:
    """State shaping, map/camera rendering and command handling (asyncio side)."""

    def __init__(self, node: MissionNode, world_json, robots, port):
        self.node, self.robots, self.port = node, robots, port
        self.world_json = world_json
        with open(world_json) as f:
            self.gt = json.load(f)
        self.rect = (0.0, 0.0, self.gt['building']['width'], self.gt['building']['height'])
        self.starts = {s['name']: (s['x'], s['y'], s['yaw']) for s in self.gt['spawns'] if s['name'] in robots}
        self.trails = {r: [] for r in robots}
        self.events = []
        self.victim_ids = []           # [(x, y, n)] stable numbering
        self.snapshots = {}            # n -> jpeg bytes
        self.last_states = {}
        self.history = {'t': [], 'coverage': [], 'victims': [], 'distance': {r: [] for r in robots}}
        self.map_png, self.map_meta, self.map_stamp = None, None, None
        self.map_rendered = -1e9
        self.jpeg_cache = {}           # (robot, kind) -> (msg id, bytes)
        self.teleop = {}               # robot -> (time, Twist)
        self.user_victims = 0
        self.mission_t = 0.0

    # ---- geometry --------------------------------------------------------------------
    def to_world(self, r, x, y):
        mx, my, myaw = self.starts[r]
        c, s = math.cos(myaw), math.sin(myaw)
        return mx + c * x - s * y, my + s * x + c * y

    def robot_world(self, r):
        st = self.node.status.get(r)
        if not st or not st.get('pose'):
            return None
        x, y, yaw = st['pose']
        wx, wy = self.to_world(r, x, y)
        return wx, wy, yaw + self.starts[r][2]

    # ---- shaping the state -------------------------------------------------------------
    def merged(self, key):
        out = []
        for r in self.robots:
            for v in self.node.team.get(r, {}).get(key, []):
                for o in out:
                    if math.hypot(o['x'] - v['x'], o['y'] - v['y']) < MERGE_RADIUS:
                        if v['confidence'] > o['confidence']:
                            o.update({k: v[k] for k in ('kind', 'confidence', 'sightings', 'last_seen') if k in v})
                        break
                else:
                    out.append(dict(v))
        taken = set()
        for v in out:
            n = next((n for x, y, n in self.victim_ids
                      if n not in taken and math.hypot(x - v['x'], y - v['y']) < MERGE_RADIUS), None)
            if n is None:
                n = len(self.victim_ids) + 1
                self.victim_ids.append((v['x'], v['y'], n))
            taken.add(n)
            v['n'] = n
        return sorted(out, key=lambda v: v['n'])

    def log(self, text, kind='info'):
        self.events.append({'t': round(self.mission_t, 1), 'text': text, 'kind': kind,
                            'wall': datetime.datetime.now().strftime('%H:%M:%S')})
        self.events = self.events[-300:]

    def state(self):
        n = self.node
        self.mission_t = max([s.get('elapsed_s', 0) for s in n.status.values()] or [0])
        links = n.links.get('links', [])
        robots = {}
        for r in self.robots:
            st = n.status.get(r, {})
            pose = self.robot_world(r)
            if pose:
                tr = self.trails[r]
                if not tr or math.hypot(tr[-1][0] - pose[0], tr[-1][1] - pose[1]) > 0.15:
                    tr.append([round(pose[0], 2), round(pose[1], 2)])
            goal = self.to_world(r, *st['goal']) if st.get('goal') else None
            peers = sorted({b if a == r else a for a, b in links if r in (a, b)})
            state = st.get('state', 'offline')
            if self.last_states.get(r) != state:
                if r in self.last_states:
                    self.log(f'{r}: {self.last_states[r]} → {state} ({st.get("message", "")})', 'robot')
                self.last_states[r] = state
            robots[r] = {
                'state': state, 'message': st.get('message', ''),
                'pose': [round(v, 2) for v in pose] if pose else None,
                'goal': [round(v, 2) for v in goal] if goal else None,
                'speed': round(n.speed.get(r, 0.0), 2), 'distance': st.get('distance_m', 0),
                'explored': st.get('explored_m2', 0), 'frontiers': st.get('frontiers', 0),
                'peers': peers, 'trail': self.trails[r][-600:],
            }
        victims, hazards = self.merged('victims'), self.merged('hazards')
        for v in victims:
            if v['n'] not in self.snapshots:
                self.snapshots[v['n']] = None
                self.log(f"Victim V{v['n']} ({v['kind']}) at ({v['x']:.1f}, {v['y']:.1f}) "
                         f"found by {v.get('found_by', '?')}", 'victim')
            if self.snapshots[v['n']] is None:          # retry until that camera has a frame
                self.snapshots[v['n']] = self.snapshot(v.get('found_by'))
        fractions = [t.get('explored_fraction', {}).get('team', 0) for t in n.team.values()]
        coverage = max(fractions) if fractions else 0.0
        h = self.history
        if not h['t'] or self.mission_t - h['t'][-1] >= 2.0:
            h['t'].append(round(self.mission_t, 1))
            h['coverage'].append(round(100 * coverage, 1))
            h['victims'].append(len(victims))
            for r in self.robots:
                h['distance'][r].append(robots[r]['distance'])
        return {
            'type': 'state', 'mission_time': self.mission_t, 'coverage': coverage,
            'robots': robots, 'victims': victims, 'hazards': hazards, 'links': links,
            'radio_range': n.links.get('los_range', 12.0),
            'events': self.events[-80:], 'history': h,
            'map': self.map_meta,
        }

    def static_info(self):
        g = self.gt
        return {'type': 'world', 'seed': g.get('seed'), 'difficulty': g.get('difficulty', 'normal'),
                'building': g['building'], 'entrance': g['entrance'], 'spawns': g['spawns'],
                'extras': g.get('extras', []), 'robots': self.robots}

    # ---- images ----------------------------------------------------------------------
    def snapshot(self, robot):
        for kind in ('det', 'rgb'):
            jpg = self.jpeg(robot, kind) if robot else None
            if jpg:
                return jpg
        return None

    def jpeg(self, robot, kind):
        msg = self.node.images.get((robot, 'det' if kind == 'rgb' and (robot, 'det') in self.node.images
                                    else kind))
        if msg is None:
            return None
        cached = self.jpeg_cache.get((robot, kind))
        if cached and cached[0] is msg:
            return cached[1]
        img = to_numpy(msg)
        bgr = thermal_colour(img) if kind == 'thermal' else np.ascontiguousarray(img[:, :, ::-1])
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            return None
        self.jpeg_cache[(robot, kind)] = (msg, buf.tobytes())
        return buf.tobytes()

    def render_map(self):
        """All robots' nav_maps in the building frame, as one RGBA PNG (rebuilt on change)."""
        maps = {r: m for r, m in self.node.maps.items() if r in self.starts}
        stamp = tuple(id(m) for m in maps.values())
        if not maps or stamp == self.map_stamp or time.monotonic() - self.map_rendered < 2.0:
            return
        self.map_stamp = stamp
        self.map_rendered = time.monotonic()
        res, margin = 0.1, 3.0
        x0, y0 = self.rect[0] - margin, self.rect[1] - margin - 1.0
        w = int((self.rect[2] - self.rect[0] + 2 * margin) / res)
        h = int((self.rect[3] - self.rect[1] + 2 * margin + 1.0) / res)
        best = np.full((h, w), -1, np.int16)          # max over robots (unknown < free < blocked)
        jj, ii = np.mgrid[0:h, 0:w]
        wx, wy = x0 + (ii + 0.5) * res, y0 + (jj + 0.5) * res
        for r, m in maps.items():
            mx, my, myaw = self.starts[r]
            c, s = math.cos(-myaw), math.sin(-myaw)
            px, py = c * (wx - mx) - s * (wy - my), s * (wx - mx) + c * (wy - my)
            g = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
            gi = ((px - m.info.origin.position.x) / m.info.resolution).astype(np.int64)
            gj = ((py - m.info.origin.position.y) / m.info.resolution).astype(np.int64)
            ok = (gi >= 0) & (gi < m.info.width) & (gj >= 0) & (gj < m.info.height)
            vals = np.full((h, w), -1, np.int16)
            vals[ok] = g[gj[ok], gi[ok]]
            best = np.maximum(best, vals)
        inside = (wx > self.rect[0] - 0.3) & (wx < self.rect[2] + 0.3) & (wy > self.rect[1] - 3.5) & \
                 (wy < self.rect[3] + 0.3)
        rgba = np.zeros((h, w, 4), np.uint8)
        rgba[(best >= 0) & (best < 20)] = (52, 61, 72, 255)          # floor
        rgba[(best >= 20) & (best < 100)] = (120, 98, 60, 255)       # climbable
        rgba[best >= 100] = (205, 210, 218, 255)                     # blocked
        rgba[~inside] = 0
        # Outside the building only real floor (around the entrance) is drawn: the rest is
        # just the robots' geofence.
        in_building = (wx > self.rect[0] - 0.15) & (wx < self.rect[2] + 0.15) & \
                      (wy > self.rect[1] - 0.15) & (wy < self.rect[3] + 0.15)
        rgba[~in_building & (best >= 100)] = 0
        rgba = rgba[::-1]
        ok, buf = cv2.imencode('.png', cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
        if ok:
            self.map_png = buf.tobytes()
            self.map_meta = {'x0': x0, 'y0': y0, 'res': res, 'w': w, 'h': h, 'version': time.time()}

    # ---- commands ----------------------------------------------------------------------
    def command(self, data):
        cmd = data.get('cmd')
        targets = self.robots if data.get('robot', 'all') == 'all' else [data['robot']]
        if cmd in COMMANDS:
            res = {r: self.node.call(r, cmd) for r in targets}
            self.log(f"Operator: {cmd.replace('_', ' ')} → " + ', '.join(f'{r} {v}' for r, v in res.items()),
                     'operator')
            return {'ok': True}
        if cmd == 'goto':
            r = data['robot']
            p = PointStamped()
            p.header.frame_id = 'world'
            p.point.x, p.point.y = float(data['x']), float(data['y'])
            self.node.pub_goto[r].publish(p)
            self.log(f"Operator: {r} → go to ({p.point.x:.1f}, {p.point.y:.1f})", 'operator')
            return {'ok': True}
        if cmd == 'teleop':
            r = data['robot']
            if self.node.status.get(r, {}).get('state') not in ('manual', 'operator goal'):
                return {'ok': False, 'error': f'{r} is autonomous: press "Take over" first'}
            t = Twist()
            t.linear.x = max(-0.3, min(0.45, float(data.get('vx', 0))))
            t.linear.y = max(-0.25, min(0.25, float(data.get('vy', 0))))
            t.angular.z = max(-1.0, min(1.0, float(data.get('wz', 0))))
            self.teleop[r] = (time.monotonic(), t)
            self.node.pub_cmd[r].publish(t)
            return {'ok': True}
        if cmd == 'spawn_victim':
            return self.spawn_victim(float(data['x']), float(data['y']), data.get('posture', 'lying'),
                                     bool(data.get('buried')), float(data.get('yaw', random.uniform(-180, 180))))
        if cmd == 'save_report':
            return {'ok': True, 'path': self.save_report()}
        if cmd == 'save_scenario':
            return {'ok': True, 'path': self.save_scenario()}
        return {'ok': False, 'error': f'unknown command {cmd}'}

    def teleop_watchdog(self):
        """Stop a teleoperated robot if the browser stops sending (released key, lost tab)."""
        now = time.monotonic()
        for r, (t, tw) in list(self.teleop.items()):
            if now - t > 0.4:
                self.node.pub_cmd[r].publish(Twist())
                del self.teleop[r]

    def spawn_victim(self, x, y, posture, buried, yaw_deg):
        from rescue_world import humans
        from rescue_world.layout import _Builder
        from rescue_world.sdf_writer import _model
        cols, rows = self.gt['building']['cols'], self.gt['building']['rows']
        b = _Builder(random.Random(), cols, rows, self.rect[2] / cols, self.rect[3] / rows,
                     'randy' if humans.randy_available() else 'primitive')
        self.user_victims += 1
        vid = f'victim_user{self.user_victims}'
        kind = 'buried' if buried else 'visible'
        v = b.place_victim_at(vid, kind, 'lying' if buried else posture, x, y, math.radians(yaw_deg))
        sdfs = [(vid, _model(vid, (x, y, 0, 0, 0, v.yaw), v.parts))]
        if b.parts:                                         # debris on a buried victim
            sdfs.append((f'{vid}_debris', _model(f'{vid}_debris', (0, 0, 0, 0, 0, 0), b.parts)))
        for name, model in sdfs:
            sdf = f'<?xml version="1.0"?><sdf version="1.8">{model}</sdf>'
            res = subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'collapsed_building',
                                  '-name', name, '-string', sdf], capture_output=True, text=True, timeout=30)
            if res.returncode != 0:
                return {'ok': False, 'error': res.stderr[-300:]}
        entry = {'id': vid, 'kind': kind, 'posture': v.posture, 'x': round(x, 3), 'y': round(y, 3),
                 'yaw': round(v.yaw, 3), 'room': list(v.room), 'placed_by': 'operator'}
        self.gt['victims'].append(entry)
        with open(self.world_json, 'w') as f:
            json.dump(self.gt, f, indent=2)
        self.node.pub_truth.publish(String(data=json.dumps(self.gt['victims'])))
        self.log(f'Operator placed a {kind} victim ({v.posture}) at ({x:.1f}, {y:.1f})', 'operator')
        return {'ok': True, 'id': vid}

    def save_report(self):
        out = os.path.expanduser('~/rescue_reports')
        os.makedirs(out, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        st = self.state()
        path = os.path.join(out, f'mission_{stamp}')
        with open(path + '.json', 'w') as f:
            json.dump({k: st[k] for k in ('mission_time', 'coverage', 'robots', 'victims', 'hazards', 'events')},
                      f, indent=2)
        with open(path + '.md', 'w') as f:
            f.write(f"# Rescue mission report ({stamp})\n\nSeed {self.gt.get('seed')}, "
                    f"mission time {st['mission_time'] / 60:.1f} min, building explored "
                    f"{100 * st['coverage']:.0f} %.\n\n## Victims\n\n"
                    '| # | type | confidence | position (m) | found by |\n|---|---|---|---|---|\n')
            for v in st['victims']:
                f.write(f"| V{v['n']} | {v['kind']} | {v['confidence']:.0%} | ({v['x']:.1f}, {v['y']:.1f}) "
                        f"| {v.get('found_by', '')} |\n")
            hazards = ''.join(f"- hot spot at ({h['x']:.1f}, {h['y']:.1f})\n" for h in st['hazards'])
            f.write('\n## Hazards\n\n' + hazards)
            f.write('\n## Timeline\n\n' + ''.join(f"- [{e['t']:.0f} s] {e['text']}\n" for e in st['events']))
        for n, jpg in self.snapshots.items():
            if jpg:
                with open(f'{path}_V{n}.jpg', 'wb') as f:
                    f.write(jpg)
        self.log(f'Report saved: {path}.md', 'operator')
        return path + '.md'

    def save_scenario(self):
        out = os.path.expanduser('~/rescue_reports')
        os.makedirs(out, exist_ok=True)
        path = os.path.join(out, f"scenario_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
        victims = [{'x': v['x'], 'y': v['y'], 'posture': v.get('posture', 'lying'),
                    'buried': v['kind'] == 'buried', 'yaw': round(math.degrees(v.get('yaw', 0.0)), 1)}
                   for v in self.gt['victims']]
        with open(path, 'w') as f:
            json.dump({'seed': self.gt.get('seed'), 'victims': victims}, f, indent=2)
        self.log(f'Scenario saved: {path} (use victims_file:={path})', 'operator')
        return path


# ---- HTTP / WebSocket ----------------------------------------------------------------
def make_app(dash: Dashboard, web_dir):
    app = web.Application()
    sockets = set()

    async def index(_req):
        return web.FileResponse(os.path.join(web_dir, 'index.html'))

    async def ws_handler(req):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(req)
        sockets.add(ws)
        await ws.send_json(dash.static_info())
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    loop = asyncio.get_running_loop()
                    # spawning runs a subprocess: keep it off the event loop
                    reply = await loop.run_in_executor(None, dash.command, data) if data.get('cmd') == \
                        'spawn_victim' else dash.command(data)
                    reply.update({'type': 'reply', 'cmd': data.get('cmd'), 'req': data.get('req')})
                    await ws.send_json(reply)
        finally:
            sockets.discard(ws)
        return ws

    async def map_png(_req):
        if dash.map_png is None:
            return web.Response(status=404)
        return web.Response(body=dash.map_png, content_type='image/png', headers={'Cache-Control': 'no-store'})

    async def snapshot(req):
        jpg = dash.snapshots.get(int(req.match_info['n']))
        if not jpg:
            return web.Response(status=404)
        return web.Response(body=jpg, content_type='image/jpeg')

    async def frame(req):
        """Latest camera frame. The page polls this instead of holding MJPEG streams open:
        browsers allow 6 connections per host, and 6 endless streams starve everything else."""
        jpg = dash.jpeg(req.match_info['robot'], req.match_info['kind'])
        if jpg is None:
            return web.Response(status=204)
        return web.Response(body=jpg, content_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    async def stream(req):
        robot, kind = req.match_info['robot'], req.match_info['kind']
        resp = web.StreamResponse(headers={'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                                           'Cache-Control': 'no-store'})
        await resp.prepare(req)
        last = None
        try:
            while True:
                jpg = dash.jpeg(robot, kind)
                if jpg is not None and jpg is not last:
                    last = jpg
                    await resp.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
                await asyncio.sleep(0.15)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp

    async def broadcaster(app_):
        async def loop():
            while True:
                try:
                    dash.render_map()
                    dash.teleop_watchdog()
                    st = dash.state()
                    for ws in list(sockets):
                        if not ws.closed:
                            await ws.send_json(st)
                except Exception as e:  # noqa: BLE001  (keep serving whatever happens)
                    dash.node.get_logger().warn(f'dashboard loop: {e}')
                await asyncio.sleep(0.25)
        task = asyncio.create_task(loop())
        yield
        task.cancel()

    app.router.add_get('/', index)
    app.router.add_get('/ws', ws_handler)
    app.router.add_get('/api/map.png', map_png)
    app.router.add_get('/api/snapshot/{n}.jpg', snapshot)
    app.router.add_get('/api/frame/{robot}/{kind}.jpg', frame)
    app.router.add_get('/api/stream/{robot}/{kind}', stream)
    # follow_symlinks: colcon --symlink-install installs the web files as symlinks
    app.router.add_static('/static/', web_dir, follow_symlinks=True)
    app.cleanup_ctx.append(broadcaster)
    return app


def main():
    rclpy.init()
    probe = rclpy.create_node('dashboard_params')
    robots = list(probe.declare_parameter('robots', ['r1', 'r2', 'r3']).value)
    seed = probe.declare_parameter('seed', 0).value
    world_json = probe.declare_parameter('world_json', '').value or \
        os.path.expanduser(f'~/.ros/rescue_worlds/rescue_{seed}.json')
    port = probe.declare_parameter('port', 8080).value
    host = probe.declare_parameter('host', '0.0.0.0').value
    open_browser = probe.declare_parameter('open_browser', True).value
    probe.destroy_node()

    with open(world_json) as f:
        world = json.load(f)
    node = MissionNode(robots, world)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    dash = Dashboard(node, world_json, robots, port)
    dash.log(f"Mission control up: seed {world.get('seed')}, {len(robots)} robots, "
             f"{world.get('difficulty', 'normal')} building", 'info')
    web_dir = os.path.join(get_package_share_directory('rescue_dashboard'), 'web')
    url = f'http://localhost:{port}'
    node.get_logger().info(f'Mission control at {url}')
    if open_browser:
        threading.Timer(2.0, lambda: webbrowser.open(url)).start()
    try:
        web.run_app(make_app(dash, web_dir), host=host, port=port, print=None, handle_signals=True)
    finally:
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
