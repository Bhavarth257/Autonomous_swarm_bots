import math

from rescue_swarm.health import DOWN, OK, STUCK, HealthWatch, unavailable

TILT = math.radians(80)


def test_tipped_over_is_down_after_a_few_seconds_and_ok_when_righted():
    h = HealthWatch()
    assert h.update(0.0, (0, 0), 'exploring', (TILT, 0.0)) == OK
    assert h.update(3.0, (0, 0), 'exploring', (TILT, 0.0)) == OK        # could be a bump
    assert h.update(5.5, (0, 0), 'exploring', (0.0, -TILT)) == DOWN
    assert 'tipped' in h.reason
    assert h.update(6.0, (0, 0), 'exploring', (0.05, 0.0)) == OK


def test_brief_tilts_never_add_up():
    h = HealthWatch()
    for k in range(20):
        tilt = (TILT, 0.0) if k % 2 else (0.0, 0.0)
        assert h.update(k * 2.0, (k * 1.0, 0), 'exploring', tilt) == OK


def test_stuck_after_two_minutes_in_one_spot_then_recovers():
    h = HealthWatch()
    for k in range(13):                                         # 120 s jiggling within 0.3 m
        out = h.update(k * 10.0, (0.3 * (k % 2), 0.0), 'exploring')
    assert out == STUCK
    assert h.update(130.0, (0.8, 0.0), 'exploring') == STUCK   # not far enough yet
    assert h.update(140.0, (1.2, 0.0), 'exploring') == OK
    assert h.update(150.0, (1.3, 0.0), 'exploring') == OK      # fresh window


def test_moving_or_not_trying_is_never_stuck():
    h = HealthWatch()
    assert all(h.update(k * 10.0, (k * 0.6, 0.0), 'exploring') == OK for k in range(40))
    h = HealthWatch()
    assert all(h.update(k * 10.0, (0.0, 0.0), s) == OK
               for k in range(40) for s in ('idle', 'done', 'manual', 'stopped'))


def test_unavailable():
    health = {'r1': [OK, 1.0], 'r2': [DOWN, 2.0], 'r3': [OK, 1.0]}
    states = {'r1': ['exploring', 1.0], 'r2': ['exploring', 1.0], 'r3': ['stopped', 1.0]}
    assert unavailable(health, states) == {'r2', 'r3'}
