from rescue_eval.evaluate import markdown, score_victims


def test_score_victims_hits_misses_false_positives():
    truth = [{'id': 'victim_0', 'kind': 'visible', 'x': 2.0, 'y': 2.0},
             {'id': 'victim_1', 'kind': 'buried', 'x': 8.0, 'y': 5.0}]
    reported = [{'x': 2.4, 'y': 2.3, 'kind': 'visible', 'confidence': 0.9, 'first_seen': 40},
                {'x': 2.1, 'y': 1.9, 'kind': 'visible', 'confidence': 0.8},     # duplicate -> FP
                {'x': 15.0, 'y': 1.0, 'kind': 'buried', 'confidence': 0.7}]     # nothing there
    hits, fps, missed = score_victims(reported, truth)
    assert [h['truth'] for h in hits] == ['victim_0'] and abs(hits[0]['error_m'] - 0.5) < 1e-6
    assert len(fps) == 2 and [m['id'] for m in missed] == ['victim_1']


def test_markdown_renders():
    r = {'seed': 4, 'end': 'all robots finished', 'mission_time_s': 600, 'coverage_of_floor': 0.93,
         'victims': {'found': 1, 'total': 2, 'false_positives': 0, 'mean_error_m': 0.4,
                     'by_kind': {'visible': {'found': 1, 'total': 1}, 'buried': {'found': 0, 'total': 1}},
                     'hits': [{'truth': 'victim_0', 'kind_true': 'visible', 'kind_reported': 'visible',
                               'error_m': 0.4, 'confidence': 0.9, 'found_by': 'r1', 'first_seen_s': 40}],
                     'missed': [{'id': 'victim_1', 'kind': 'buried'}]},
         'hazards': {'decoys': 1, 'reported_as_hazard': 1, 'reported_as_victim': 0},
         'robots': {'r1': {'state': 'done', 'distance_m': 80.0, 'back_at_start': True}}}
    md = markdown(r)
    assert '| **all victims** | **1** | **2** |' in md and '**missed**' in md
