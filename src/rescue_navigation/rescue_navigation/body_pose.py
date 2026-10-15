"""Where was the robot (and each of its sensors) in the map frame at sensor time t?

Uses time-stamped odometry and body-tilt samples from wheelleg_odom rather than a TF
buffer: a Python TF listener that has to swallow every robot's leg and wheel transforms
falls behind (exact-time lookups fail) and costs a lot of CPU. Only two things come from TF,
read straight off the topics by FrameWatch:
  map -> odom      SLAM's slowly changing correction (latest value is good enough)
  body -> sensor   fixed mounts from the URDF (/tf_static)
Transforms are (R, p) pairs: x_parent = R @ x_child + p.
"""
import bisect
import math
from collections import deque

import numpy as np
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

STATIC = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE)


def tf_matrix(t):
    q, p = t.transform.rotation, t.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.array([p.x, p.y, p.z])


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def compose(a, b):
    """Transform a followed by b."""
    return a[0] @ b[0], a[0] @ b[1] + a[1]


def inverse(a):
    return a[0].T, -a[0].T @ a[1]


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class History:
    """Time-stamped samples, linearly interpolated; `angles` marks values that wrap at pi."""

    def __init__(self, angles, seconds=3.0):
        self.t, self.v = deque(), deque()
        self.angles = angles
        self.keep = seconds

    def add(self, t, values):
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t)
        self.v.append(values)
        while self.t and self.t[0] < t - self.keep:
            self.t.popleft()
            self.v.popleft()

    def at(self, t, tolerance=0.15):
        if not self.t or t < self.t[0] - tolerance or t > self.t[-1] + tolerance:
            return None
        ts = list(self.t)
        k = bisect.bisect_left(ts, t)
        if k <= 0:
            return self.v[0]
        if k >= len(ts):
            return self.v[-1]
        t0, t1 = ts[k - 1], ts[k]
        a = (t - t0) / (t1 - t0)
        out = []
        for x0, x1, ang in zip(self.v[k - 1], self.v[k], self.angles):
            d = x1 - x0
            if ang:
                d = math.atan2(math.sin(d), math.cos(d))
            out.append(x0 + a * d)
        return out


class FrameWatch:
    """Latest transform for a few (parent, child) pairs from /tf, and the static tree from
    /tf_static, without a TF buffer: everything else on /tf is ignored cheaply."""

    def __init__(self, node, dynamic_pairs):
        self.wanted = set(dynamic_pairs)
        self.latest = {}
        self.static = {}            # child -> (parent, (R, p))
        node.create_subscription(TFMessage, '/tf', self._on_tf, 100)
        node.create_subscription(TFMessage, '/tf_static', self._on_static, STATIC)

    def _on_tf(self, msg):
        for t in msg.transforms:
            if (t.header.frame_id, t.child_frame_id) in self.wanted:
                self.latest[(t.header.frame_id, t.child_frame_id)] = tf_matrix(t)

    def _on_static(self, msg):
        for t in msg.transforms:
            self.static[t.child_frame_id] = (t.header.frame_id, tf_matrix(t))

    def get(self, parent, child):
        """Latest dynamic transform parent<-child, or raise LookupError."""
        try:
            return self.latest[(parent, child)]
        except KeyError:
            raise LookupError(f'no {parent} -> {child} transform yet') from None

    def static_chain(self, ancestor, child):
        """Compose static transforms from `child` up to `ancestor`."""
        R, p = np.eye(3), np.zeros(3)
        frame = child
        for _ in range(20):
            if frame == ancestor:
                return R, p
            if frame not in self.static:
                break
            parent, tr = self.static[frame]
            R, p = compose(tr, (R, p))
            frame = parent
        raise LookupError(f'no static chain {ancestor} -> {child}')


class BodyPose:
    """Subscribes to <topic_ns>odom and <topic_ns>odom/tilt. From a node inside the robot's
    namespace leave topic_ns empty; a node serving several robots passes '/r1/' etc."""

    def __init__(self, node, frames: FrameWatch, prefix, body_height=0.15, topic_ns=''):
        self.frames = frames
        self.map_frame, self.odom_frame = f'{prefix}map', f'{prefix}odom'
        self.body_frame = f'{prefix}base_link'
        self.body_height = body_height
        self.odom = History((False, False, False, True))   # x, y, ground z, yaw
        self.tilt = History((True, True))                  # roll, pitch
        self.mounts = {}
        node.create_subscription(Odometry, f'{topic_ns}odom', self._on_odom, 50)
        node.create_subscription(QuaternionStamped, f'{topic_ns}odom/tilt', self._on_tilt, 50)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom.add(stamp_seconds(msg.header.stamp), (p.x, p.y, p.z, quat_rpy(msg.pose.pose.orientation)[2]))

    def _on_tilt(self, msg):
        roll, pitch, _ = quat_rpy(msg.quaternion)
        self.tilt.add(stamp_seconds(msg.header.stamp), (roll, pitch))

    def at(self, t, sensor_frame=None):
        """(map<-footprint, map<-sensor or None) at time t; raises LookupError with a reason."""
        odom, tilt = self.odom.at(t), self.tilt.at(t)
        if odom is None or tilt is None:
            raise LookupError(f'no odometry/tilt sample near t={t:.2f}')
        m_o = self.frames.get(self.map_frame, self.odom_frame)
        if sensor_frame is not None and sensor_frame not in self.mounts:
            self.mounts[sensor_frame] = self.frames.static_chain(self.body_frame, sensor_frame)
        x, y, gz, yaw = odom
        roll, pitch = tilt
        footprint = compose(m_o, (rpy_matrix(0.0, 0.0, yaw), np.array([x, y, gz])))
        if sensor_frame is None:
            return footprint, None
        body = compose(m_o, (rpy_matrix(roll, pitch, yaw), np.array([x, y, gz + self.body_height])))
        return footprint, compose(body, self.mounts[sensor_frame])
