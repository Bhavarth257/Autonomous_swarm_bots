from rescue_swarm import rooms as rm

RECT = (0.0, 0.0, 24.0, 20.0)          # 4 x 4 rooms of 6 x 5 m, entrance bottom middle
ROBOTS = ['r1', 'r2', 'r3']
ENTRANCE = (15.0, 0.0)


def test_initial_split_is_even_contiguous_and_left_to_right():
    rooms = rm.room_rects(RECT, 4, 4)
    split = rm.initial_split(rooms, ROBOTS, ENTRANCE)
    counts = {r: sum(1 for v in split.values() if v == r) for r in ROBOTS}
    assert sorted(counts.values()) == [5, 5, 6]
    mean_x = {r: sum(rm.centre(rooms[k])[0] for k, v in split.items() if v == r) / counts[r] for r in ROBOTS}
    assert mean_x['r1'] < mean_x['r2'] < mean_x['r3']           # r1 left, r3 right
    # everyone computes the same split without talking
    assert split == rm.initial_split(rm.room_rects(RECT, 4, 4), ROBOTS, ENTRANCE)


def test_finished_robot_takes_half_of_busiest_teammates_nearest_rooms():
    rooms = rm.room_rects(RECT, 4, 4)
    book = rm.RoomBook(rooms, ROBOTS, ENTRANCE)
    done = {k: book.owner(k) == 'r1' for k in rooms}             # r1 finished, r2/r3 nothing yet
    r3_rooms = book.open_rooms('r3', done)
    # r3 is busiest only if it has more rooms: make r2's mostly done
    for k in book.open_rooms('r2', done)[:3]:
        done[k] = True
    r3_pos = rm.centre(rooms[r3_rooms[0]])
    taken = book.rebalance('r1', (2.0, 2.0), [r3_pos], done, t=50.0)
    assert taken and all(k in r3_rooms for k in taken)
    assert len(taken) == len(r3_rooms) // 2
    assert rm.room_at(rooms, *r3_pos) not in taken                # never the room r3 is in
    assert all(book.owner(k) == 'r1' for k in taken)
    # nothing more while r1 still has open rooms
    assert book.rebalance('r1', (2.0, 2.0), [], done, t=60.0) == []


def test_books_merge_newest_wins():
    rooms = rm.room_rects(RECT, 4, 4)
    a, b = rm.RoomBook(rooms, ROBOTS, ENTRANCE), rm.RoomBook(rooms, ROBOTS, ENTRANCE)
    k = next(iter(rooms))
    b.book[k] = ['r2', 10.0]
    a.merge(b.message())
    assert a.owner(k) == 'r2'
    a.book[k] = ['r3', 20.0]
    b.merge(a.message())
    a.merge({k: ['r1', 5.0]})                                    # older news is ignored
    assert a.owner(k) == 'r3' and b.owner(k) == 'r3'


def test_no_ping_pong_over_the_last_room():
    rooms = rm.room_rects(RECT, 4, 4)
    book = rm.RoomBook(rooms, ROBOTS, ENTRANCE)
    last = next(k for k in rooms if book.owner(k) == 'r1')
    done = {k: k != last for k in rooms}                         # only r1's `last` room is open
    assert book.rebalance('r2', (20.0, 2.0), [], done, t=100.0) == []    # r1 has one room: leave it
    # a room that just changed hands isn't taken again within a minute
    two = [k for k in rooms if book.owner(k) == 'r3'][:2]
    done = {k: k not in two for k in rooms}
    book.book[two[0]] = ['r3', 95.0]
    taken = book.rebalance('r1', (2.0, 2.0), [], done, t=100.0)
    assert two[0] not in taken


def test_down_robots_rooms_go_to_the_nearest_healthy_robot():
    rooms = rm.room_rects(RECT, 4, 4)
    pos = {'r1': (3.0, 10.0), 'r2': (12.0, 10.0), 'r3': (21.0, 10.0)}
    books = {r: rm.RoomBook(rooms, ROBOTS, ENTRANCE) for r in ROBOTS}
    done = {k: False for k in rooms}
    r2_rooms = books['r1'].open_rooms('r2', done)
    t1 = books['r1'].adopt('r1', pos, {'r2'}, done, t=30.0)
    t3 = books['r3'].adopt('r3', pos, {'r2'}, done, t=30.0)
    assert sorted(t1 + t3) == sorted(r2_rooms) and not set(t1) & set(t3)   # split, no overlap
    for k in t1:
        assert rm.centre(rooms[k])[0] <= 12.0                              # r1 takes the left ones
    # the down robot itself never adopts, and healthy robots' rooms are left alone
    assert books['r2'].adopt('r2', pos, {'r2'}, done, t=30.0) == []
    assert all(books['r1'].owner(k) != 'r3' or k not in t1 for k in rooms)
    # after gossip everyone agrees r2 owns nothing open
    for b in books.values():
        for o in books.values():
            b.merge(o.message())
    assert all(not b.open_rooms('r2', done) for b in books.values())


def test_adopt_ignores_done_rooms_and_unknown_positions():
    rooms = rm.room_rects(RECT, 4, 4)
    book = rm.RoomBook(rooms, ROBOTS, ENTRANCE)
    done = {k: book.owner(k) == 'r2' for k in rooms}                       # r2 finished before it fell
    assert book.adopt('r1', {'r1': (3.0, 10.0)}, {'r2'}, done, t=30.0) == []
    done = {k: False for k in rooms}
    taken = book.adopt('r1', {'r1': (3.0, 10.0)}, {'r2'}, done, t=30.0)   # heard nobody else: takes all
    assert taken and all(book.owner(k) == 'r1' for k in taken)


def test_rebalance_does_not_help_a_down_teammate():
    rooms = rm.room_rects(RECT, 4, 4)
    book = rm.RoomBook(rooms, ROBOTS, ENTRANCE)
    done = {k: book.owner(k) == 'r1' for k in rooms}
    for k in book.open_rooms('r2', done)[:3]:
        done[k] = True                                                     # r3 now the busiest...
    taken = book.rebalance('r1', (2.0, 2.0), [], done, t=50.0, unavailable={'r2', 'r3'})
    assert taken == []                                                     # ...but nobody healthy to help


def test_down_robot_never_takes_rooms():
    rooms = rm.room_rects(RECT, 4, 4)
    book = rm.RoomBook(rooms, ROBOTS, ENTRANCE)
    done = {k: book.owner(k) == 'r2' for k in rooms}                       # r2's own rooms done
    # r2 tipped over: its explorer may still say "exploring", but it must not grab work
    assert book.rebalance('r2', (12.0, 10.0), [], done, t=200.0, unavailable={'r2'}) == []
