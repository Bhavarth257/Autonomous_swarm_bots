import numpy as np

from rescue_perception import tracker as tk
from rescue_perception import vision as vi


def test_thermal_cues_classes():
    t = np.full((240, 320), 29300, dtype=np.uint16)       # 20 C background
    t[100:140, 50:120] = 30900                              # a body
    t[30:50, 200:230] = 34000                               # fire
    t[200:202, 10:12] = 30900                               # 4 px of noise
    cues = vi.thermal_cues(t)
    kinds = sorted(c.source for c in cues)
    assert kinds == ['hot', 'thermal']
    body = next(c for c in cues if c.source == 'thermal')
    assert body.box == (50, 100, 120, 140) and abs(body.temp_k - 309.0) < 0.01


def test_locate_picks_nearest_surface():
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1.0]])
    person = np.column_stack([np.linspace(-0.2, 0.2, 20), np.linspace(-0.3, 0.3, 20), np.full(20, 3.0)])
    wall = np.column_stack([np.linspace(-0.5, 0.5, 40), np.linspace(-0.5, 0.5, 40), np.full(40, 6.0)])
    p = vi.locate(np.vstack([person, wall]), K, (250, 140, 390, 340))
    assert abs(p[2] - 3.0) < 0.05
    assert vi.locate(wall, K, (0, 0, 10, 10)) is None


def test_tracker_kinds_and_confirmation():
    tr = tk.Tracker()
    # Person seen by YOLO from several places, thermal agrees.
    for k in range(3):
        tr.add('rgb', 5.0 + 0.1 * k, 2.0, k, robot_xy=(k * 1.0, 0.0))
        tr.add('thermal', 5.1, 2.1, k, robot_xy=(k * 1.0, 0.0), temp_k=309)
    # Buried: thermal only.
    for k in range(6):
        tr.add('thermal', 10.0, 8.0, k, robot_xy=(8.0 + k, 6.0), temp_k=303)
    # Fire: hot only.
    for k in range(3):
        tr.add('hot', 1.0, 9.0, k, robot_xy=(0.0, 6.0), temp_k=345)
    # One stray YOLO hit.
    tr.add('rgb', 15.0, 1.0, 0, robot_xy=(14.0, 0.0))
    kinds = {round(t.x): t.kind for t in tr.tracks}
    assert kinds == {5: 'visible', 10: 'buried', 1: 'hazard', 15: 'candidate'}
    assert {round(t.x) for t in tr.confirmed()} == {5, 10}
    assert [round(t.x) for t in tr.hazards()] == [1]


def test_merge_external_keeps_evidence():
    a, b = tk.Tracker(), tk.Tracker()
    for k in range(3):
        a.add('rgb', 4.0, 4.0, k, robot_xy=(k, 0.0))
    v = a.tracks[0].to_dict()
    b.merge_external(v, 10.0)
    assert b.tracks[0].kind == 'visible' and b.tracks[0].n_rgb == 3


def test_floor_point_and_depth_gate():
    # Camera 0.2 m above the floor looking along +x (optical z -> map x, x -> -y, y -> -z).
    R = np.array([[0, 0, 1.0], [-1.0, 0, 0], [0, -1.0, 0]])
    p_cam = np.array([0.0, 0.0, 0.2])
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1.0]])
    # A pixel 25 px below centre looks down atan(25/500): hits the floor at 0.2 / 0.05 = 4 m.
    pt = vi.floor_point(K, R, p_cam, (320, 265), 0.0)
    assert abs(pt[0] - 4.0) < 1e-6 and abs(pt[2]) < 1e-9
    assert vi.floor_point(K, R, p_cam, (320, 200), 0.0) is None          # above the horizon
    # Lidar only hit the wall at 7 m behind a body at 4 m: the depth gate rejects it.
    wall = np.column_stack([np.linspace(-0.3, 0.3, 30), np.linspace(-0.2, 0.2, 30), np.full(30, 7.0)])
    assert vi.locate(wall, K, (250, 200, 390, 280), expected_depth=4.0) is None
    assert vi.locate(wall, K, (250, 200, 390, 280)) is not None


def test_merge_external_buried_keeps_viewpoints():
    a, b = tk.Tracker(), tk.Tracker()
    for k in range(5):
        a.add('thermal', 10.0, 8.0, k, robot_xy=(8.0 + k, 6.0), temp_k=303)
    assert a.tracks[0].kind == 'buried'
    b.merge_external(a.tracks[0].to_dict(), 10.0)
    assert b.tracks[0].kind == 'buried' and b.tracks[0] in b.confirmed()


def test_ghosts_folded_into_strong_track():
    tr = tk.Tracker()
    for k in range(40):
        tr.add('thermal', 5.0, 5.0, k, robot_xy=(k * 1.0, 0.0), temp_k=303)
    for k in range(8):
        tr.add('thermal', 7.0, 5.0, k, robot_xy=(k * 1.0, 2.0), temp_k=303)    # ghost 2 m off
    for k in range(20):
        tr.add('thermal', 12.0, 5.0, k, robot_xy=(k * 1.0, 4.0), temp_k=303)   # a real second victim
    assert sorted(round(t.x) for t in tr.confirmed()) == [5, 12]
    assert vi.touches_border((0, 10, 50, 60), (240, 320)) and not vi.touches_border((10, 10, 50, 60), (240, 320))


def test_far_sightings_weigh_less():
    tr = tk.Tracker()
    for k in range(10):                                   # far views, 1.5 m off
        tr.add('rgb', 11.5, 5.0, k, robot_xy=(k, 0.0), distance=7.0)
    for k in range(3):                                    # a few close views, accurate
        tr.add('rgb', 10.0, 5.0, 20 + k, robot_xy=(9.0 + k * 0.9, 4.0), distance=1.5)
    assert len(tr.tracks) == 1 and abs(tr.tracks[0].x - 10.0) < 0.3


def test_box_covers_teammate():
    R = np.array([[0, 0, 1.0], [-1.0, 0, 0], [0, -1.0, 0]])
    p_cam = np.array([0.0, 0.0, 0.2])
    K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1.0]])
    # A teammate 6 m ahead, 0.5 m to the left, body at 0.2 m: u = 320 + 500 * 0.5 / 6 = 278.
    mate = (6.0, 0.5, 0.2)
    assert vi.box_covers(K, R, p_cam, mate, (270, 230, 285, 245), 0.25)
    assert not vi.box_covers(K, R, p_cam, mate, (400, 230, 420, 245), 0.25)
    assert not vi.box_covers(K, R, p_cam, (-3.0, 0.0, 0.2), (0, 0, 640, 480), 0.25)   # behind
