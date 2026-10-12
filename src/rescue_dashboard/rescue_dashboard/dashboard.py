"""Mission dashboard: start/stop the autonomous search and watch it live.

Everything shown comes from the robots' own topics (what a base station would get),
nothing from the simulator's ground truth:

  /rN/mission/status   state, pose, goal, distance, explored area   (explorer)
  /rN/odom             speed                                         (legged odometry)
  /rN/nav_map          the robot's map, drawn in the building frame
  /rN/team/state       team victim list, explored fractions, radio contact (swarm agent)
  /rN/perception/image camera with detection boxes                   (detector)
  /radio/links         which robots can hear each other right now     (radio channel)

Buttons call /rN/mission/{start,stop,return_home} on every robot.
"""
import datetime
import json
import math
import os
import signal
import sys
import threading

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
ROBOT_COLORS = ['#ff8c1a', '#33aaff', '#d24dff', '#33cc66', '#e6c619']
KIND_COLORS = {'visible': '#ff3333', 'buried': '#ff9900', 'hazard': '#3377ff'}
STATE_COLORS = {'idle': '#888888', 'exploring': '#33aaff', 'returning': '#e6c619',
                'done': '#33cc66', 'stopped': '#ff5555'}
STYLE = """
QWidget { background: #1e1f22; color: #e6e6e6; font-size: 12px; }
QPushButton { border-radius: 6px; padding: 10px 18px; font-size: 15px; font-weight: bold; color: white; }
QPushButton:disabled { background: #444; color: #888; }
QFrame#card { background: #2a2c30; border-radius: 8px; }
QLabel#big { font-size: 22px; font-weight: bold; }
QLabel#dim { color: #9aa0a6; }
QTableWidget { background: #2a2c30; gridline-color: #3a3c40; border: none; }
QHeaderView::section { background: #33353a; color: #cfd2d6; border: none; padding: 4px; }
QPlainTextEdit { background: #2a2c30; border: none; font-family: monospace; font-size: 11px; }
QComboBox { background: #2a2c30; padding: 4px; }
"""


# ---------------------------------------------------------------------------- ROS side
class DashboardNode(Node):
    def __init__(self, robots):
        super().__init__('mission_dashboard')
        self.robots = robots
        self.status, self.speed, self.maps, self.team, self.images = {}, {}, {}, {}, {}
        self.links = {}
        self.mission_clients = {r: {k: self.create_client(Trigger, f'/{r}/mission/{k}')
                                    for k in ('start', 'stop', 'return_home')} for r in robots}
        for r in robots:
            self.create_subscription(String, f'/{r}/mission/status',
                                     lambda m, r=r: self.status.__setitem__(r, json.loads(m.data)), 10)
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.speed.__setitem__(
                r, math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)), 10)
            self.create_subscription(OccupancyGrid, f'/{r}/nav_map', lambda m, r=r: self.maps.__setitem__(r, m),
                                     LATCHED)
            self.create_subscription(String, f'/{r}/team/state',
                                     lambda m, r=r: self.team.__setitem__(r, json.loads(m.data)), LATCHED)
            self.create_subscription(Image, f'/{r}/perception/image',
                                     lambda m, r=r: self.images.__setitem__(r, m), qos_profile_sensor_data)
        self.create_subscription(String, '/radio/links', lambda m: self.links.update(json.loads(m.data)), 10)

    def call_all(self, what):
        results = {}
        for r in self.robots:
            c = self.mission_clients[r][what]
            if c.service_is_ready():
                c.call_async(Trigger.Request())
                results[r] = 'sent'
            else:
                results[r] = 'not ready'
        return results


# ---------------------------------------------------------------------------- map view
class MapView(QtWidgets.QWidget):
    """Building frame (world) view: robot maps, robots, goals, radio links, victims."""

    def __init__(self, dash):
        super().__init__()
        self.dash = dash
        self.setMinimumSize(520, 440)
        self.map_images = {}        # robot -> (QImage, info) cached per map message
        self.trails = {}

    def world_to_px(self):
        W, H = self.dash.rect[2], self.dash.rect[3]
        margin = 2.5
        sx = self.width() / (W + 2 * margin)
        sy = self.height() / (H + 2 * margin + 1.5)
        s = min(sx, sy)
        ox = (self.width() - (W + 2 * margin) * s) / 2 + margin * s
        oy = (self.height() + (H + 2 * margin + 1.5) * s) / 2 - (margin + 1.5) * s
        return s, ox, oy

    def to_px(self, x, y):
        s, ox, oy = self.world_to_px()
        return QtCore.QPointF(ox + x * s, oy - y * s)

    def map_image(self, r, msg):
        cached = self.map_images.get(r)
        if cached and cached[1] is msg:
            return cached[0]
        g = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        # Only the building (and around the entrance) is drawn: the robots' maps mark
        # everything else blocked (their geofence), which would swamp the view.
        res = msg.info.resolution
        xs = msg.info.origin.position.x + (np.arange(g.shape[1]) + 0.5) * res
        ys = msg.info.origin.position.y + (np.arange(g.shape[0]) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)
        mx, my, myaw = self.dash.starts[r]
        wx = mx + math.cos(myaw) * X - math.sin(myaw) * Y
        wy = my + math.sin(myaw) * X + math.cos(myaw) * Y
        W, H = self.dash.rect[2], self.dash.rect[3]
        shown = (wx > -0.3) & (wx < W + 0.3) & (wy > -3.5) & (wy < H + 0.3)
        rgba = np.zeros(g.shape + (4,), np.uint8)
        free = (g >= 0) & (g < 100)
        rgba[free] = (58, 66, 74, 255)
        climb = (g >= 20) & (g < 100)
        rgba[climb] = (95, 85, 60, 255)
        rgba[g >= 100] = (215, 215, 215, 255)
        rgba[~shown] = 0
        rgba = np.ascontiguousarray(rgba[::-1])          # image rows go down, map y goes up
        img = QtGui.QImage(rgba.data, rgba.shape[1], rgba.shape[0], QtGui.QImage.Format_RGBA8888).copy()
        self.map_images[r] = (img, msg)
        return img

    def paintEvent(self, _ev):
        d = self.dash
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('#15161a'))
        s, ox, oy = self.world_to_px()

        # Each robot's own map, placed in the building frame by its known start pose.
        p.save()
        p.setOpacity(0.9)
        for r in d.robots:
            msg = d.node.maps.get(r)
            if msg is None:
                continue
            img = self.map_image(r, msg)
            mx, my, myaw = d.starts[r]
            t = QtGui.QTransform()
            t.translate(ox, oy)
            t.scale(s, -s)                               # world (y up) -> screen
            t.translate(mx, my)
            t.rotateRadians(myaw)
            t.translate(msg.info.origin.position.x, msg.info.origin.position.y)
            t.scale(msg.info.resolution, msg.info.resolution)
            t.translate(0, msg.info.height)
            t.scale(1, -1)
            p.setTransform(t)
            p.setCompositionMode(QtGui.QPainter.CompositionMode_Lighten)
            p.drawImage(0, 0, img)
        p.restore()

        # Building outline and entrance.
        W, H = d.rect[2], d.rect[3]
        p.setPen(QtGui.QPen(QtGui.QColor('#5f6368'), 1, QtCore.Qt.DashLine))
        p.drawRect(QtCore.QRectF(self.to_px(0, H), self.to_px(W, 0)))
        ent = d.entrance
        if ent:
            p.setPen(QtGui.QPen(QtGui.QColor('#33cc66'), 3))
            p.drawLine(self.to_px(ent['x'] - ent['width'] / 2, 0), self.to_px(ent['x'] + ent['width'] / 2, 0))

        # Radio links.
        pos = {r: self.robot_world(r) for r in d.robots}
        for a, b in d.node.links.get('links', []):
            if pos.get(a) and pos.get(b):
                p.setPen(QtGui.QPen(QtGui.QColor(80, 220, 120, 160), 2, QtCore.Qt.DotLine))
                p.drawLine(self.to_px(*pos[a][:2]), self.to_px(*pos[b][:2]))

        # Victims and hazards (team knowledge, union over robots).
        for v in d.all_victims() + d.all_hazards():
            c = QtGui.QColor(KIND_COLORS.get(v['kind'], '#aaaaaa'))
            p.setPen(QtGui.QPen(QtGui.QColor('white'), 1.5))
            p.setBrush(c)
            pt = self.to_px(v['x'], v['y'])
            p.drawEllipse(pt, 7, 7)
            p.setPen(QtGui.QColor('white'))
            p.drawText(pt + QtCore.QPointF(9, -6), f"V{v['n']}" if v['kind'] != 'hazard' else 'HOT')

        # Robots, trails and goals.
        for k, r in enumerate(d.robots):
            col = QtGui.QColor(ROBOT_COLORS[k % len(ROBOT_COLORS)])
            w = pos.get(r)
            if not w:
                continue
            tr = self.trails.setdefault(r, [])
            if not tr or math.hypot(tr[-1][0] - w[0], tr[-1][1] - w[1]) > 0.15:
                tr.append(w[:2])
            if len(tr) > 1:
                p.setPen(QtGui.QPen(col, 1.5))
                p.drawPolyline(QtGui.QPolygonF([self.to_px(*q) for q in tr]))
            st = d.node.status.get(r, {})
            if st.get('goal'):
                gx, gy = d.map_to_world(r, *st['goal'])
                p.setPen(QtGui.QPen(col, 1.5, QtCore.Qt.DashLine))
                p.drawLine(self.to_px(*w[:2]), self.to_px(gx, gy))
                p.setBrush(QtCore.Qt.NoBrush)
                p.drawEllipse(self.to_px(gx, gy), 5, 5)
            x, y, yaw = w
            pts = [(x + 0.5 * math.cos(yaw), y + 0.5 * math.sin(yaw)),
                   (x + 0.3 * math.cos(yaw + 2.5), y + 0.3 * math.sin(yaw + 2.5)),
                   (x + 0.3 * math.cos(yaw - 2.5), y + 0.3 * math.sin(yaw - 2.5))]
            p.setPen(QtGui.QPen(QtGui.QColor('white'), 1))
            p.setBrush(col)
            p.drawPolygon(QtGui.QPolygonF([self.to_px(*q) for q in pts]))
            p.drawText(self.to_px(x, y) + QtCore.QPointF(10, 14), r)

        # Legend.
        p.setPen(QtGui.QColor('#9aa0a6'))
        p.drawText(10, self.height() - 10, 'map: grey floor, brown climbable, white blocked | '
                   'V red visible, orange buried, HOT hazard | dotted green = radio link')
        p.end()

    def robot_world(self, r):
        st = self.dash.node.status.get(r)
        if not st or not st.get('pose'):
            return None
        x, y, yaw = st['pose']
        wx, wy = self.dash.map_to_world(r, x, y)
        return wx, wy, yaw + self.dash.starts[r][2]


# ---------------------------------------------------------------------------- widgets
class RobotCard(QtWidgets.QFrame):
    def __init__(self, name, color):
        super().__init__()
        self.setObjectName('card')
        lay = QtWidgets.QGridLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        title = QtWidgets.QLabel(f'<span style="color:{color}; font-size:16px; font-weight:bold">● {name}</span>')
        self.state = QtWidgets.QLabel('—')
        lay.addWidget(title, 0, 0)
        lay.addWidget(self.state, 0, 1, alignment=QtCore.Qt.AlignRight)
        self.fields = {}
        for k, key in enumerate(('position', 'speed', 'distance', 'explored', 'goal', 'radio')):
            a = QtWidgets.QLabel(key)
            a.setObjectName('dim')
            b = QtWidgets.QLabel('—')
            lay.addWidget(a, 1 + k, 0)
            lay.addWidget(b, 1 + k, 1, alignment=QtCore.Qt.AlignRight)
            self.fields[key] = b
        self.message = QtWidgets.QLabel('')
        self.message.setObjectName('dim')
        self.message.setWordWrap(True)
        lay.addWidget(self.message, 8, 0, 1, 2)

    def update_from(self, st, speed, world_pose, contact):
        state = st.get('state', '—')
        self.state.setText(f'<b style="color:{STATE_COLORS.get(state, "#ccc")}">{state.upper()}</b>')
        if world_pose:
            self.fields['position'].setText(f'({world_pose[0]:.1f}, {world_pose[1]:.1f}) m')
        self.fields['speed'].setText(f'{speed:.2f} m/s' if speed is not None else '—')
        self.fields['distance'].setText(f"{st.get('distance_m', 0):.1f} m")
        self.fields['explored'].setText(f"{st.get('explored_m2', 0):.0f} m²")
        self.fields['goal'].setText('frontier' if st.get('goal') and state == 'exploring' else
                                    ('entrance' if state == 'returning' else '—'))
        self.fields['radio'].setText(contact)
        self.message.setText(st.get('message', ''))


class Dashboard(QtWidgets.QWidget):
    def __init__(self, node, robots, rect, starts, entrance, seed, explorable_m2):
        super().__init__()
        self.node, self.robots, self.rect, self.starts = node, robots, rect, starts
        self.entrance, self.seed, self.explorable = entrance, seed, explorable_m2
        self.victim_index = []            # [(x, y, n)]: stable V-numbers across updates
        self.events = []
        self.last_states = {}
        self.known_victims = set()
        self.setWindowTitle('Rescue swarm: mission control')
        self.setStyleSheet(STYLE)
        self.resize(1500, 900)
        self.build_ui()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)

    # ---- layout -----------------------------------------------------------------------
    def build_ui(self):
        root = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel(f'<b style="font-size:18px">Collapsed building search</b>'
                                 f'<span style="color:#9aa0a6">  seed {self.seed} · {len(self.robots)} hexapods</span>')
        top.addWidget(title)
        top.addStretch(1)
        self.clock = self.stat(top, 'mission time', '00:00')
        self.coverage = self.stat(top, 'building explored', '0 %')
        self.victims_found = self.stat(top, 'victims found', '0')
        top.addSpacing(20)
        self.btn_start = self.button(top, 'START', '#1e8e3e', lambda: self.command('start'))
        self.btn_stop = self.button(top, 'STOP', '#c5221f', lambda: self.command('stop'))
        self.btn_home = self.button(top, 'RETURN HOME', '#b06000', lambda: self.command('return_home'))
        self.button(top, 'Save report', '#3c4043', self.save_report)
        root.addLayout(top)

        body = QtWidgets.QHBoxLayout()
        self.map = MapView(self)
        body.addWidget(self.map, 3)
        side = QtWidgets.QVBoxLayout()
        cards = QtWidgets.QHBoxLayout()
        self.cards = {}
        for k, r in enumerate(self.robots):
            c = RobotCard(r, ROBOT_COLORS[k % len(ROBOT_COLORS)])
            cards.addWidget(c)
            self.cards[r] = c
        side.addLayout(cards)
        side.addWidget(self.section('Victims'))
        self.table = QtWidgets.QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(['#', 'type', 'confidence', 'position (m)', 'found by',
                                              'evidence', 'last seen'])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        side.addWidget(self.table, 2)
        cam_row = QtWidgets.QHBoxLayout()
        cam_row.addWidget(self.section('Camera'))
        self.cam_pick = QtWidgets.QComboBox()
        self.cam_pick.addItems(self.robots)
        cam_row.addWidget(self.cam_pick)
        cam_row.addStretch(1)
        side.addLayout(cam_row)
        bottom = QtWidgets.QHBoxLayout()
        self.camera = QtWidgets.QLabel('no image yet')
        self.camera.setMinimumSize(320, 240)
        self.camera.setAlignment(QtCore.Qt.AlignCenter)
        self.camera.setStyleSheet('background:#000; border-radius:6px')
        bottom.addWidget(self.camera, 1)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        bottom.addWidget(self.log, 1)
        side.addLayout(bottom, 2)
        body.addLayout(side, 3)
        root.addLayout(body, 1)

    def stat(self, layout, label, value):
        box = QtWidgets.QVBoxLayout()
        v = QtWidgets.QLabel(value)
        v.setObjectName('big')
        l_ = QtWidgets.QLabel(label)
        l_.setObjectName('dim')
        box.addWidget(v, alignment=QtCore.Qt.AlignCenter)
        box.addWidget(l_, alignment=QtCore.Qt.AlignCenter)
        layout.addLayout(box)
        layout.addSpacing(18)
        return v

    def button(self, layout, text, color, slot):
        b = QtWidgets.QPushButton(text)
        b.setStyleSheet(f'QPushButton {{ background: {color}; }}')
        b.clicked.connect(slot)
        layout.addWidget(b)
        return b

    def section(self, text):
        lab = QtWidgets.QLabel(f'<b style="font-size:14px">{text}</b>')
        return lab

    # ---- data helpers -------------------------------------------------------------------
    def map_to_world(self, r, x, y):
        mx, my, myaw = self.starts[r]
        c, s = math.cos(myaw), math.sin(myaw)
        return mx + c * x - s * y, my + s * x + c * y

    def _merged(self, key):
        """Union over every robot's team view, merging entries closer than 3 m."""
        out = []
        for r in self.robots:
            for v in self.node.team.get(r, {}).get(key, []):
                for o in out:
                    if math.hypot(o['x'] - v['x'], o['y'] - v['y']) < 3.0:   # same as the trackers
                        if v['confidence'] > o['confidence']:
                            o.update({k: v[k] for k in ('kind', 'confidence', 'sightings', 'last_seen')})
                        break
                else:
                    out.append(dict(v))
        taken = set()
        for v in out:
            match = next((n for kx, ky, n in self.victim_index
                          if n not in taken and math.hypot(kx - v['x'], ky - v['y']) < 3.0), None)
            if match is None:
                match = len(self.victim_index) + 1
                self.victim_index.append((v['x'], v['y'], match))
            taken.add(match)
            v['n'] = match
        return sorted(out, key=lambda v: v['n'])

    def all_victims(self):
        return self._merged('victims')

    def all_hazards(self):
        return self._merged('hazards')

    def log_event(self, text):
        stamp = self.clock.text()
        self.events.append(f'[{stamp}] {text}')
        self.log.appendPlainText(f'[{stamp}] {text}')

    # ---- actions -------------------------------------------------------------------------
    def command(self, what):
        res = self.node.call_all(what)
        self.log_event(f'{what.replace("_", " ").upper()} → ' + ', '.join(f'{r}: {v}' for r, v in res.items()))

    def save_report(self):
        out_dir = os.path.expanduser('~/rescue_reports')
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        victims = self.all_victims()
        report = {
            'seed': self.seed, 'saved': stamp, 'mission_time': self.clock.text(),
            'building_explored': self.coverage.text(),
            'victims': victims, 'hazards': self.all_hazards(),
            'robots': {r: self.node.status.get(r) for r in self.robots}, 'events': self.events,
        }
        path = os.path.join(out_dir, f'mission_{stamp}')
        with open(path + '.json', 'w') as f:
            json.dump(report, f, indent=2)
        with open(path + '.md', 'w') as f:
            f.write(f'# Rescue mission report ({stamp})\n\nSeed {self.seed}, mission time {self.clock.text()}, '
                    f'building explored {self.coverage.text()}.\n\n## Victims\n\n'
                    '| # | type | confidence | position (m) | found by |\n|---|---|---|---|---|\n')
            for v in victims:
                f.write(f"| V{v['n']} | {v['kind']} | {v['confidence']:.0%} | ({v['x']:.1f}, {v['y']:.1f}) "
                        f"| {v.get('found_by', '')} |\n")
            hazards = ''.join(f"- hot spot at ({h['x']:.1f}, {h['y']:.1f}), {h.get('max_temp_c')} °C\n"
                              for h in self.all_hazards())
            f.write('\n## Hazards\n\n' + hazards)
            f.write('\n## Events\n\n' + ''.join(f'- {e}\n' for e in self.events))
        self.log_event(f'report saved: {path}.md')

    # ---- refresh -------------------------------------------------------------------------
    def refresh(self):
        n = self.node
        elapsed = max([s.get('elapsed_s', 0) for s in n.status.values()] or [0])
        self.clock.setText(f'{int(elapsed // 60):02d}:{int(elapsed % 60):02d}')
        fractions = [t.get('explored_fraction', {}).get('team', 0) for t in n.team.values()]
        if fractions:
            self.coverage.setText(f'{100 * max(fractions):.0f} %')
        elif n.status:
            area = max(s.get('explored_m2', 0) for s in n.status.values())
            self.coverage.setText(f'{100 * area / (self.rect[2] * self.rect[3]):.0f} %')

        links = {tuple(sorted(l_)) for l_ in n.links.get('links', [])}
        for r in self.robots:
            st = n.status.get(r)
            if not st:
                continue
            peers = [o for o in self.robots if o != r and tuple(sorted((r, o))) in links]
            self.cards[r].update_from(st, n.speed.get(r), self.map.robot_world(r),
                                      ', '.join(peers) if peers else 'no contact')
            if self.last_states.get(r) != st['state']:
                if r in self.last_states:
                    self.log_event(f'{r}: {self.last_states[r]} → {st["state"]} ({st.get("message", "")})')
                self.last_states[r] = st['state']

        victims = self.all_victims()
        self.victims_found.setText(str(len(victims)))
        for v in victims:
            if v['n'] not in self.known_victims:
                self.known_victims.add(v['n'])
                self.log_event(f"VICTIM V{v['n']} ({v['kind']}) at ({v['x']:.1f}, {v['y']:.1f}) "
                               f"found by {v.get('found_by', '?')}")
        self.table.setRowCount(len(victims))
        for row, v in enumerate(victims):
            s = v.get('sightings', {})
            cells = [f"V{v['n']}", v['kind'], f"{v['confidence']:.0%}", f"({v['x']:.1f}, {v['y']:.1f})",
                     v.get('found_by', ''), f"camera {s.get('rgb', 0)} · thermal {s.get('thermal', 0)}",
                     f"{v.get('last_seen', 0):.0f} s"]
            for col, text in enumerate(cells):
                item = QtWidgets.QTableWidgetItem(text)
                if col == 1:
                    item.setForeground(QtGui.QColor(KIND_COLORS.get(v['kind'], '#ccc')))
                self.table.setItem(row, col, item)

        img = n.images.get(self.cam_pick.currentText())
        if img is not None:
            q = QtGui.QImage(bytes(img.data), img.width, img.height, img.step, QtGui.QImage.Format_RGB888)
            self.camera.setPixmap(QtGui.QPixmap.fromImage(q).scaled(
                self.camera.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        any_running = any(s.get('state') in ('exploring', 'returning') for s in n.status.values())
        self.btn_start.setEnabled(not any_running)
        self.map.update()


def main():
    rclpy.init()
    probe = rclpy.create_node('dashboard_params')
    seed = probe.declare_parameter('seed', 0).value
    robots = list(probe.declare_parameter('robots', ['r1', 'r2', 'r3']).value)
    snapshot = probe.declare_parameter('snapshot', '').value   # save a screenshot every 10 s
    probe.destroy_node()
    with open(os.path.expanduser(f'~/.ros/rescue_worlds/rescue_{seed}.json')) as f:
        gt = json.load(f)       # only the ground plan rescuers have: outline, entrance, start poses
    rect = (0.0, 0.0, gt['building']['width'], gt['building']['height'])
    starts = {s['name']: (s['x'], s['y'], s['yaw']) for s in gt['spawns'] if s['name'] in robots}
    node = DashboardNode(robots)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')                  # same dark look on every platform plugin
    pal = QtGui.QPalette()
    for role, color in ((QtGui.QPalette.Window, '#1e1f22'), (QtGui.QPalette.WindowText, '#e6e6e6'),
                        (QtGui.QPalette.Base, '#2a2c30'), (QtGui.QPalette.Text, '#e6e6e6'),
                        (QtGui.QPalette.Button, '#33353a'), (QtGui.QPalette.ButtonText, '#e6e6e6')):
        pal.setColor(role, QtGui.QColor(color))
    app.setPalette(pal)
    win = Dashboard(node, robots, rect, starts, gt.get('entrance'), seed, gt.get('explorable_area_m2'))
    win.show()
    # Qt's event loop never returns to Python on its own, so Ctrl+C / launch shutdown
    # would be ignored: handle the signals and wake Python up regularly.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    wake = QtCore.QTimer()
    wake.timeout.connect(lambda: None)
    wake.start(200)
    if snapshot:
        shot = QtCore.QTimer()
        shot.timeout.connect(lambda: win.grab().save(snapshot))
        shot.start(10000)
    code = app.exec_()
    executor.shutdown()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
