import math

import numpy as np

from rescue_navigation import grid_utils as gu


def test_traversability_classes():
    h = np.zeros((20, 20), dtype=np.float32)
    h[5:8, 5:8] = 0.10          # climbable block
    h[12:15, 12:15] = 0.20      # victim-height obstacle: too tall to step on
    h[0, :] = 2.0               # wall
    h[18:, 18:] = np.nan        # unseen
    t = gu.traversability(h, step_max=0.14, obstacle_height=0.4)
    assert t[10, 2] == 0                          # flat floor
    assert 20 <= t[5, 5] < gu.LETHAL              # block edge: climbable, costly
    assert t[6, 6] == 0                           # top of the block is flat
    assert t[12, 12] == gu.LETHAL                 # edge of the 20 cm obstacle
    assert t[0, 3] == gu.LETHAL
    assert t[1, 3] == 0                           # floor next to the wall stays floor
    assert t[11, 12] == 0                         # ... and next to the victim
    assert t[19, 19] == gu.UNKNOWN


def test_frontiers_and_clusters():
    g = np.full((30, 30), gu.UNKNOWN, dtype=np.int8)
    g[:, :15] = 0                 # known free left half
    g[10:12, :15] = gu.LETHAL     # a wall crossing it
    mask = gu.frontier_mask(g)
    assert mask[:, 14].sum() == 28              # the boundary column, minus the wall cells
    cl = gu.clusters(mask, min_size=3)
    assert len(cl) == 2                          # the wall splits the frontier in two
    region = np.zeros_like(mask)
    region[:10] = True
    assert len(gu.clusters(gu.frontier_mask(g, region), 3)) == 1


def test_rect_region_rotated_map():
    # Map frame at world (5, -2) rotated 90 deg: map +x points along world +y.
    region = gu.rect_region((40, 40), 0.5, (-10.0, -10.0), (0, 0, 10, 10), (5, -2, math.pi / 2))
    X, Y = gu.cell_centers((40, 40), 0.5, (-10.0, -10.0))
    j, i = np.argwhere(region)[0]
    wx, wy = 5 - Y[j, i], -2 + X[j, i]
    assert 0 < wx < 10 and 0 < wy < 10


def test_reachable_through_door_only():
    g = np.zeros((40, 40), dtype=np.int8)
    g[20, :] = gu.LETHAL            # wall across the room
    g[20, 30:36] = 0                # 6-cell door
    g[30:, 5:10] = gu.UNKNOWN       # unseen floor is passable
    r = gu.reachable(g, (5, 5), radius_cells=2)
    assert r[35, 7]                 # through the door, across unknown floor
    g[20, 30:36] = gu.LETHAL        # close the door
    r = gu.reachable(g, (5, 5), radius_cells=2)
    assert r[10, 10] and not r[35, 7]
    # Start brushing the wall (inside the grown obstacle): snaps to the nearest open cell.
    assert gu.reachable(g, (19, 10), radius_cells=2)[10, 10]


def test_path_distance_goes_around_walls():
    free = np.ones((200, 200), dtype=bool)
    free[95:105, :170] = False                  # a (robot-grown) wall with a gap on the right
    d = gu.path_distance(free, (80, 20), 0.05)
    assert abs(d[80, 40] - 1.0) < 0.21           # open floor: about the straight line
    assert d[120, 20] > 12.0                     # 1 m away through the wall, ~15 m around it
    assert np.isinf(gu.path_distance(np.zeros((10, 10), dtype=bool), (5, 5), 0.05)[0, 0])


def test_sector_cost_splits_building_in_strips():
    import types
    from rescue_navigation.explorer import Explorer
    e = types.SimpleNamespace(team_size=3, robot_index=0, rect=[0.0, 0.0, 24.0, 20.0], sector_penalty=5.0)
    cost = lambda x: Explorer.sector_cost(e, x)       # noqa: E731
    assert cost(4.0) == 0.0                           # r1 owns x 0-8
    assert cost(10.0) == 5.0 + 0.5 * 2.0              # 2 m into r2's strip
    e.robot_index = 2
    assert cost(20.0) == 0.0 and cost(4.0) > 5.0
