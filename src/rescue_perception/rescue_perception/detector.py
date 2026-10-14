"""Victim detection for a team of robots: YOLOv8 person detection on the RGB camera,
body-temperature blobs on the thermal camera, placed in 3D with the lidar.

One process serves every robot so the GPU holds a single YOLO model (three CUDA
contexts would not fit next to Gazebo on a 4 GB card); each robot still gets its own
tracker and its own topics, as if the detector ran on board:

  in   /rN/camera/{image_raw,camera_info}   /rN/thermal/{image_raw,camera_info}
       /rN/lidar/points   /rN/odom   /rN/odom/tilt   (+ map->odom and mounts from /tf)
  out  /rN/victims          std_msgs/String JSON, 1 Hz: confirmed victims, candidates,
                            hazards (map frame and world frame)
       /rN/victim_markers   visualization_msgs/MarkerArray
       /rN/perception/image camera image with detection boxes, ~2 Hz

A cue (person box / warm blob / too-hot blob) is placed in 3D by projecting the lidar
cloud into the image and taking the nearest surface inside the box; see vision.py.
Sightings are fused per robot by tracker.Tracker.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry  # noqa: F401  (BodyPose subscribes)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from rescue_navigation.body_pose import BodyPose, FrameWatch, compose, inverse, stamp_seconds
from rescue_navigation.traversability import cloud_xyz
from rescue_perception import tracker as tk
from rescue_perception import vision as vi

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
DEFAULT_WEIGHTS = os.path.expanduser('~/.ros/rescue_models/yolov8n.pt')


def load_yolo(weights, logger):
    """YOLOv8n on the GPU if there is one; None (thermal-only) if ultralytics is missing."""
    try:
        import torch
        from ultralytics import YOLO
    except ImportError:
        logger.warn('ultralytics not installed: running thermal-only (pip install --user ultralytics)')
        return None, None
    if not os.path.exists(weights):
        os.makedirs(os.path.dirname(weights), exist_ok=True)
        cwd = os.getcwd()
        os.chdir(os.path.dirname(weights))       # ultralytics downloads into the cwd
        try:
            YOLO(os.path.basename(weights))
        finally:
            os.chdir(cwd)
    # Inference runs on the GPU; without a cap torch's CPU-side pre/post-processing grabs
    # every core and starves Gazebo.
    torch.set_num_threads(2)
    device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
    model = YOLO(weights)
    model.predict(np.zeros((480, 640, 3), np.uint8), device=device, verbose=False)   # warm-up
    logger.info(f'YOLO {os.path.basename(weights)} on {device}')
    return model, device


def to_numpy(msg: Image):
    if msg.encoding in ('mono16', '16UC1'):
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    return img[:, :, :3] if msg.encoding in ('rgb8', 'rgba8') else img[:, :, 2::-1]


class RobotPipeline:
    """Everything one robot's detector keeps."""

    def __init__(self, node, ns, map_in_world, frames, body_height=0.15):
        self.ns = ns
        self.node = node
        self.map_in_world = map_in_world
        self.pose = BodyPose(node, frames, f'{ns}/', body_height, topic_ns=f'/{ns}/')
        self.tracker = tk.Tracker()
        self.rgb = self.thermal = self.cloud = None
        self.K_rgb = self.K_th = None
        self.processed_stamp = None
        sub = node.create_subscription
        sub(Image, f'/{ns}/camera/image_raw', lambda m: setattr(self, 'rgb', m), qos_profile_sensor_data)
        sub(Image, f'/{ns}/thermal/image_raw', lambda m: setattr(self, 'thermal', m), qos_profile_sensor_data)
        sub(PointCloud2, f'/{ns}/lidar/points', lambda m: setattr(self, 'cloud', m), qos_profile_sensor_data)
        sub(CameraInfo, f'/{ns}/camera/camera_info', lambda m: setattr(self, 'K_rgb', np.array(m.k).reshape(3, 3)), 10)
        sub(CameraInfo, f'/{ns}/thermal/camera_info', lambda m: setattr(self, 'K_th', np.array(m.k).reshape(3, 3)), 10)
        self.pub_victims = node.create_publisher(String, f'/{ns}/victims', LATCHED)
        self.pub_markers = node.create_publisher(MarkerArray, f'/{ns}/victim_markers', 10)
        self.pub_image = node.create_publisher(Image, f'/{ns}/perception/image', 5)

    def to_world(self, x, y):
        mx, my, myaw = self.map_in_world
        c, s = math.cos(myaw), math.sin(myaw)
        return mx + c * x - s * y, my + s * x + c * y

    def to_map(self, wx, wy):
        mx, my, myaw = self.map_in_world
        c, s = math.cos(myaw), math.sin(myaw)
        return c * (wx - mx) + s * (wy - my), -s * (wx - mx) + c * (wy - my)


class Detector(Node):
    def __init__(self):
        super().__init__('victim_detector')
        robots = list(self.declare_parameter('robots', ['r1']).value)
        poses = list(self.declare_parameter('maps_in_world', [0.0, 0.0, 0.0]).value)
        weights = self.declare_parameter('weights', DEFAULT_WEIGHTS).value
        self.conf = self.declare_parameter('person_confidence', 0.35).value
        self.max_range = self.declare_parameter('max_range', 8.0).value
        self.floor_only_range = self.declare_parameter('floor_only_range', 5.0).value
        # Victims lie on the building floor (map z = 0), not on whatever the robot is standing
        # on: with the camera ~0.3 m up, a robot on a 12 cm step would place a victim 4 m
        # away at 2.4 m if it used its own ground as the floor plane.
        self.floor_height = self.declare_parameter('floor_height', 0.0).value
        # Measured: tracks sat 0.4-0.7 m short of the body's middle, along the line of sight.
        self.body_depth = self.declare_parameter('body_depth', 0.3).value
        # Victims are inside the building; a teammate hexapod is not a person.
        self.building = list(self.declare_parameter('building_rect', [-1e9, -1e9, 1e9, 1e9]).value)
        self.robot_gap = self.declare_parameter('robot_exclusion', 0.7).value
        self.teammate_radius = self.declare_parameter('teammate_radius', 0.3).value
        self.teammate_height = self.declare_parameter('teammate_height', 0.2).value
        self.body_height = self.declare_parameter('body_height', 0.15).value
        self.robot_world = {}         # robot -> (x, y) world, updated as frames are processed
        rate = self.declare_parameter('rate', 6.0).value

        self.frames = FrameWatch(self, [(f'{ns}/map', f'{ns}/odom') for ns in robots])
        self.robots = [RobotPipeline(self, ns, poses[3 * k:3 * k + 3] if len(poses) >= 3 * k + 3
                                     else [0.0, 0.0, 0.0], self.frames, self.body_height)
                       for k, ns in enumerate(robots)]
        self.yolo, self.device = load_yolo(weights, self.get_logger())
        self.turn = 0
        self.timing = []
        self.create_timer(1.0 / rate, self.step)
        self.create_timer(1.0, self.publish)
        self.create_timer(30.0, self.report)

    # ---- detection ---------------------------------------------------------------
    def step(self):
        """One robot per tick, round robin, on its newest frame not yet processed."""
        for _ in range(len(self.robots)):
            r = self.robots[self.turn]
            self.turn = (self.turn + 1) % len(self.robots)
            if r.rgb is not None and r.rgb.header.stamp != r.processed_stamp and r.cloud is not None \
                    and r.K_rgb is not None:
                t0 = time.monotonic()
                self.process(r)
                self.timing.append(time.monotonic() - t0)
                return

    def process(self, r: RobotPipeline):
        rgb_msg, th_msg, cloud_msg = r.rgb, r.thermal, r.cloud
        r.processed_stamp = rgb_msg.header.stamp
        t_img = stamp_seconds(rgb_msg.header.stamp)
        cam_frame = rgb_msg.header.frame_id
        try:
            m_fp, m_cam = r.pose.at(t_img, cam_frame)
            _, m_lidar = r.pose.at(stamp_seconds(cloud_msg.header.stamp), cloud_msg.header.frame_id)
        except LookupError:
            return
        # Lidar points into the camera frame at image time (corrects for motion between them).
        cam_lidar = compose(inverse(m_cam), m_lidar)
        pts = cloud_xyz(cloud_msg)
        pts = pts[np.linalg.norm(pts, axis=1) < self.max_range]
        pts_cam = pts @ cam_lidar[0].T + cam_lidar[1]

        rgb = to_numpy(rgb_msg)
        cues = []
        if self.yolo is not None:
            res = self.yolo.predict(rgb, classes=[0], conf=self.conf, device=self.device,
                                    verbose=False, imgsz=640)[0]
            for box, score in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.conf.cpu().numpy()):
                cues.append(vi.Cue('rgb', tuple(float(v) for v in box), float(score)))
        th = None
        if th_msg is not None and r.K_th is not None and abs(stamp_seconds(th_msg.header.stamp) - t_img) < 0.4:
            th = to_numpy(th_msg)
            cues += vi.thermal_cues(th)

        robot_xy = m_fp[1][:2]
        floor_z = self.floor_height
        self.robot_world[r.ns] = r.to_world(float(robot_xy[0]), float(robot_xy[1]))
        for c in cues:
            K = r.K_rgb if c.source == 'rgb' else r.K_th
            shape = rgb.shape if c.source == 'rgb' else th.shape
            if vi.touches_border(c.box, shape):
                continue          # partly out of view: its bottom edge isn't where it stands
            if c.source == 'rgb' and any(
                    vi.box_covers(K, m_cam[0], m_cam[1], (*r.to_map(px, py), floor_z + self.teammate_height), c.box, self.teammate_radius)
                    for ns, (px, py) in self.robot_world.items() if ns != r.ns):
                continue          # a teammate: YOLO sometimes takes the black hexapod for a person
            # Distance from the box's bottom edge meeting the floor; lidar refines it when
            # it agrees, but a lying body is often below the lidar rings.
            x0, y0, x1, y1 = c.box
            # The box bottom is where feet or body touch the floor: intersect with the floor
            # itself (a plane even 5 cm up shortens a 5 m estimate by ~1 m at this camera height).
            on_floor = vi.floor_point(K, m_cam[0], m_cam[1], ((x0 + x1) / 2, y1), floor_z)
            expected = None
            if on_floor is not None:
                expected = float((m_cam[0].T @ (on_floor - m_cam[1]))[2])
                if expected > self.max_range:
                    continue
            p = vi.locate(pts_cam, K, c.box, expected_depth=expected)
            if p is not None:
                pm = m_cam[0] @ p + m_cam[1]
            elif on_floor is not None and expected < self.floor_only_range:
                pm = on_floor       # the camera is low: far away a pixel of error is metres
            else:
                continue
            if not (-0.5 < pm[2] - floor_z < 1.5):          # victims are on the floor
                continue
            # Camera and lidar see the near side of a body; its middle is further away.
            ray = pm[:2] - robot_xy
            pm = pm.copy()
            pm[:2] += self.body_depth * ray / max(float(np.linalg.norm(ray)), 1e-6)
            wx, wy = r.to_world(float(pm[0]), float(pm[1]))
            x0, y0, x1, y1 = self.building
            if not (x0 - 0.2 < wx < x1 + 0.2 and y0 - 0.2 < wy < y1 + 0.2):
                continue                                    # outside the building
            if any(math.hypot(wx - px, wy - py) < self.robot_gap for px, py in self.robot_world.values()):
                continue                                    # that's a robot, not a person
            dist = float(math.hypot(pm[0] - robot_xy[0], pm[1] - robot_xy[1]))
            r.tracker.add(c.source, float(pm[0]), float(pm[1]), t_img, tuple(robot_xy),
                          temp_k=c.temp_k, found_by=r.ns, distance=dist)

        if self.pub_image_due(r):
            out = vi.annotate(rgb, cues, th.shape if th is not None else None)
            msg = Image()
            msg.header = rgb_msg.header
            msg.height, msg.width = out.shape[:2]
            msg.encoding, msg.step = 'rgb8', out.shape[1] * 3
            msg.data = out.tobytes()
            r.pub_image.publish(msg)

    def pub_image_due(self, r):
        now = time.monotonic()
        if now - getattr(r, '_last_img', 0.0) > 0.45:
            r._last_img = now
            return True
        return False

    # ---- outputs -----------------------------------------------------------------
    def publish(self):
        for r in self.robots:
            def entry(t):
                d = t.to_dict()
                d['world_x'], d['world_y'] = (round(v, 2) for v in r.to_world(t.x, t.y))
                return d
            report = {
                'robot': r.ns, 'frame': f'{r.ns}/map',
                'victims': [entry(t) for t in r.tracker.confirmed()],
                'candidates': [entry(t) for t in r.tracker.tracks if t.kind == 'candidate'
                               or (t.kind in ('visible', 'buried') and t not in r.tracker.confirmed())],
                'hazards': [entry(t) for t in r.tracker.hazards()],
            }
            r.pub_victims.publish(String(data=json.dumps(report)))
            r.pub_markers.publish(self.markers(r))

    def markers(self, r):
        arr = MarkerArray()
        colors = {'visible': (1.0, 0.1, 0.1), 'buried': (1.0, 0.55, 0.0), 'hazard': (0.2, 0.4, 1.0),
                  'candidate': (0.7, 0.7, 0.7)}
        for t in r.tracker.tracks:
            for k, (mtype, z, scale) in enumerate(((Marker.CYLINDER, 0.3, (0.5, 0.5, 0.6)),
                                                   (Marker.TEXT_VIEW_FACING, 1.0, (0.3, 0.3, 0.3)))):
                m = Marker()
                m.header.frame_id = f'{r.ns}/map'
                m.ns, m.id, m.type = f'victims_{k}', t.id, mtype
                m.pose.position.x, m.pose.position.y, m.pose.position.z = t.x, t.y, z
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = scale
                m.color.r, m.color.g, m.color.b = colors[t.kind]
                m.color.a = 0.8 if t.kind != 'candidate' else 0.4
                if mtype == Marker.TEXT_VIEW_FACING:
                    m.text = f'{t.kind} {t.confidence:.0%}' if t.kind != 'candidate' else '?'
                arr.markers.append(m)
        return arr

    def report(self):
        if self.timing:
            ms = 1000 * np.mean(self.timing)
            self.timing = []
            counts = ', '.join(f'{r.ns}: {len(r.tracker.confirmed())} victims' for r in self.robots)
            self.get_logger().info(f'{ms:.0f} ms per frame | {counts}')


def main():
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():                 # a real error, not a publish racing Ctrl+C
            raise


if __name__ == '__main__':
    main()
