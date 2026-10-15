import numpy as np

from rescue_navigation.wall_guard import Clearance, StallWatch, clearances, guard, obstacle_points

HL, HW = 0.40, 0.30


def wall(y, x0=-1.0, x1=1.0, n=60):
    """A wall parallel to the robot at lateral offset y, as the lidar sees it: nothing
    closer than 0.4 m."""
    xs = np.linspace(x0, x1, n)
    pts = np.stack([xs, np.full(n, y)], axis=1)
    return pts[np.hypot(pts[:, 0], pts[:, 1]) >= 0.4]


def test_obstacle_points_drops_floor_and_far_points():
    xyz = np.array([[0.6, 0.0, -0.53],     # floor (lidar 0.54 up)
                    [0.6, 0.0, -0.10],     # wall at 0.44 m above the floor
                    [3.0, 0.0, -0.10]])    # too far to matter
    assert obstacle_points(xyz, 0.54, 0.15, 1.2).tolist() == [[0.6, 0.0]]


def test_wall_beside_seen_only_ahead_and_behind():
    c = clearances(wall(0.33), HL, HW)
    assert abs(c.left - 0.03) < 1e-6 and c.right == float('inf')
    assert c.front == float('inf')


def test_own_body_ignored():
    inside = np.array([[0.2, 0.1], [-0.35, -0.25]])
    assert clearances(inside, HL, HW) == Clearance()


def test_steers_away_from_close_wall_and_slows():
    c = clearances(wall(0.33), HL, HW)      # left wall 3 cm away
    vx, wz = guard(0.45, 0.0, c)
    assert wz < -0.3 and vx < 0.45


def test_doorway_centres():
    c = clearances(np.vstack([wall(0.36), wall(-0.40)]), HL, HW)   # left 6 cm, right 10 cm
    _, wz = guard(0.3, 0.0, c)
    assert wz < 0
    c = clearances(np.vstack([wall(0.38), wall(-0.38)]), HL, HW)
    _, wz = guard(0.3, 0.0, c)
    assert abs(wz) < 1e-9


def test_open_space_passes_through():
    assert guard(0.45, 0.3, Clearance()) == (0.45, 0.3)


def test_blocked_ahead_stops_forward_but_keeps_turn():
    c = clearances(np.array([[0.44, 0.0], [0.44, 0.1]]), HL, HW)
    assert guard(0.3, 0.5, c) == (0.0, 0.5)


def test_turn_in_place_creeps_away_from_corner():
    c = clearances(np.array([[0.42, 0.25]]), HL, HW)     # front-left corner zone
    vx, wz = guard(0.0, 1.0, c)
    assert vx < 0 and wz == 1.0


def test_stall_backs_off_away_from_wall():
    s = StallWatch()
    c = clearances(wall(0.33), HL, HW)
    t = 0.0
    while (out := s.update(t, 0.4, (1.0, 2.0), c)) is None:   # pushing, not moving
        t += 0.1
        assert t < 4.0
    assert out[0] < 0 and out[1] < 0          # back up, front swings right (away from the left wall)
    assert s.update(t + 0.5, 0.4, (1.0, 2.0), c)[0] < 0      # still backing off
    assert s.update(t + 1.5, 0.4, (1.0, 2.0), c) is None     # then Nav2 again


def test_moving_robot_never_stalls():
    s = StallWatch()
    c = clearances(wall(0.33), HL, HW)
    assert all(s.update(k * 0.1, 0.4, (k * 0.04, 0.0), c) is None for k in range(60))
