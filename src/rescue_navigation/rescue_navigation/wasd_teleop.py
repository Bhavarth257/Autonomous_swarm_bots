"""Drive a robot from the keyboard: hold the arrow keys (or W/A/S/D).

  UP / DOWN      forward / backward      SPACE   stop now
  LEFT / RIGHT   turn left / right       + / -   faster / slower
  1 2 3      drive r1 / r2 / r3          R / F   wheel-leg robot: stand taller / lower
  X or Ctrl+C  quit

A terminal reports key presses, not releases: a held key arrives through auto-repeat,
so the robot keeps going while one keeps arriving and stops 0.6 s after the last.
Only the most recently pressed key repeats, so hold one key at a time.

  ros2 run rescue_navigation teleop                    # starts with r1
  ros2 run rescue_navigation teleop --ros-args -p robot:=r2

If the autonomous mission is running, take the robot over first (dashboard "Take over"),
or the explorer and this node will both be steering it.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

HOLD = 0.6               # s a key keeps acting after its last repeat
ARROWS = {'A': 'w', 'B': 's', 'C': 'd', 'D': 'a'}   # ESC [ A..D: up, down, right, left


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.speed = self.declare_parameter('speed', 0.4).value          # m/s
        self.turn = self.declare_parameter('turn_rate', 1.2).value       # rad/s
        self.splay = 0.8                                                 # wheel-leg stance, rad
        self.seen = {}                                                   # key -> last time
        self.pub = self.pub_stance = None
        self.select(self.declare_parameter('robot', 'r1').value)
        self.create_timer(0.05, self.tick)

    def select(self, robot):
        if self.pub is not None:
            self.pub.publish(Twist())                                    # leave the old one stopped
            self.destroy_publisher(self.pub)
            self.destroy_publisher(self.pub_stance)
        self.robot = robot
        self.pub = self.create_publisher(Twist, f'/{robot}/cmd_vel', 10)
        self.pub_stance = self.create_publisher(Float64, f'/{robot}/stance', 10)
        self.seen.clear()

    def key(self, k):
        k = k.lower()
        now = time.monotonic()
        if k in 'wasd':
            self.seen[k] = now
        elif k == ' ':
            self.seen.clear()
        elif k in '+=':
            self.speed = min(0.9, self.speed + 0.05)
        elif k in '-_':
            self.speed = max(0.05, self.speed - 0.05)
        elif k in '123':
            self.select(f'r{k}')
        elif k in 'rf':
            # R: close the V (stand taller), F: open it (lower, longer; much above 0.8 it can't turn)
            self.splay = min(1.1, max(0.3, self.splay + (-0.1 if k == 'r' else 0.1)))
            self.pub_stance.publish(Float64(data=self.splay))

    def held(self, k):
        return time.monotonic() - self.seen.get(k, -1e9) < HOLD

    def tick(self):
        t = Twist()
        t.linear.x = self.speed * (self.held('w') - self.held('s'))
        t.angular.z = self.turn * (self.held('a') - self.held('d'))
        self.pub.publish(t)
        sys.stdout.write(f'\r  {self.robot}  speed {self.speed:.2f} m/s  stance {self.splay:.2f} rad  '
                         f'->  v {t.linear.x:+.2f}  turn {t.angular.z:+.2f}     ')
        sys.stdout.flush()


def main():
    if not sys.stdin.isatty():
        print('wasd_teleop needs a terminal (run it in its own terminal window).')
        return
    print(__doc__.split('\n\nA terminal')[0])
    rclpy.init()
    node = WasdTeleop()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        pending = ''
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if not select.select([fd], [], [], 0.02)[0]:
                continue
            # Raw bytes, not sys.stdin: Python's buffer would swallow the rest of an arrow
            # key's escape sequence (ESC [ A) and its last letter would read as a key.
            pending += os.read(fd, 64).decode(errors='ignore')
            quit_ = False
            while pending:
                if pending[0] == '\x1b':
                    if len(pending) < 3:
                        if select.select([fd], [], [], 0.02)[0]:
                            pending += os.read(fd, 64).decode(errors='ignore')
                            continue
                        pending = ''                     # a lone ESC: ignore it
                        break
                    k = ARROWS.get(pending[2], '') if pending[1] in '[O' else ''
                    pending = pending[3:]
                else:
                    k, pending = pending[0], pending[1:]
                if k in ('x', 'X', '\x03'):
                    quit_ = True
                    break
                if k:
                    node.key(k)
            if quit_:
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
