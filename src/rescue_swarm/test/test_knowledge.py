import numpy as np

from rescue_swarm.knowledge import ExploredGrid, TeamView
from rescue_swarm.radio import link_ok


def test_explored_grid_roundtrip_and_merge():
    a, b = ExploredGrid((0, 0, 10, 5)), ExploredGrid((0, 0, 10, 5))
    xs, ys = np.meshgrid(np.arange(0.025, 2.0, 0.05), np.arange(0.025, 1.0, 0.05))
    a.update_from_points(xs.ravel(), ys.ravel(), np.ones(xs.size, bool), per_cell=100)
    assert a.explored(0.3, 0.3) and not a.explored(5, 3)
    b.merge_encoded(a.encode())
    assert (b.mask == a.mask).all() and abs(b.fraction() - 8 / 200) < 1e-9


def test_gossip_relays_news():
    rect = (0, 0, 18, 15)
    r1, r2, r3 = TeamView('r1', rect), TeamView('r2', rect), TeamView('r3', rect)
    r1.set_self((2.0, 2.0, 0.0), (5.0, 5.0), 'exploring', 10.0)
    r1.add_own_victims({'victims': [{'id': 1, 'kind': 'visible', 'confidence': 0.9, 'x': 0, 'y': 0,
                                     'world_x': 4.0, 'world_y': 3.0,
                                     'sightings': {'rgb': 3, 'thermal': 2, 'hot': 0}}]}, 10.0)
    r2.receive(r1.message(10.0), 10.5)        # r1 -> r2 in range
    r2.set_self((9.0, 2.0, 0.0), None, 'exploring', 11.0)
    r3.receive(r2.message(11.0), 11.5)        # r2 -> r3; r1 never in range of r3
    assert r3.poses['r1'][:2] == [2.0, 2.0]
    assert (5.0, 5.0) in r3.others() and (9.0, 2.0) in r3.others()
    assert len(r3.victims.confirmed()) == 1 and r3.victims.confirmed()[0].found_by == 'r1'
    # Explored areas stay attributed: r1's map reaches r3 as "others", not as r3's own.
    xs, ys = np.meshgrid(np.arange(0.025, 2.0, 0.05), np.arange(0.025, 2.0, 0.05))
    r1.mine().update_from_points(xs.ravel(), ys.ravel(), np.ones(xs.size, bool), per_cell=100)
    r2.receive(r1.message(12.0), 12.1)
    r3.receive(r2.message(12.2), 12.3)
    assert r3.explored_by_others().explored(1.0, 1.0) and not r3.mine().explored(1.0, 1.0)
    # Older news never overwrites newer.
    r1.set_self((3.0, 3.0, 0.0), None, 'exploring', 20.0)
    r3.receive(r1.message(20.0), 20.1)
    r3.receive(r2.message(11.0), 20.2)        # stale relay of r1's old pose
    assert r3.poses['r1'][:2] == [3.0, 3.0]


def test_radio_link_model():
    walls = np.zeros((100, 100), bool)
    walls[:, 50] = True                        # wall at x = 5 m (res 0.1, origin 0)
    grid = (walls, 0.1, (0.0, 0.0))
    assert link_ok((1, 1), (4, 9), grid, los_range=12.0, wall_range=4.0)       # same side
    assert not link_ok((1, 1), (8, 1), grid, los_range=12.0, wall_range=4.0)   # wall, 7 m
    assert link_ok((4, 1), (6, 1), grid, los_range=12.0, wall_range=4.0)       # wall, 2 m
    assert not link_ok((0.5, 0.5), (0.5, 9.9), grid, los_range=8.0, wall_range=4.0)  # too far
