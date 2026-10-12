import json
import xml.etree.ElementTree as ET

import pytest

from rescue_world.generate import generate
from rescue_world.layout import generate_layout


@pytest.mark.parametrize('seed', range(10))
def test_layout_is_valid(seed):
    layout = generate_layout(seed=seed)
    assert sum(v.kind == 'visible' for v in layout.victims) == 4
    assert sum(v.kind == 'buried' for v in layout.victims) == 2
    assert len(layout.spawns) == 3
    # Spawn cells are reachable by construction; the building must be mostly explorable.
    assert layout.reachable.sum() > 0


def test_same_seed_same_world():
    a, b = generate_layout(seed=3), generate_layout(seed=3)
    assert [(v.x, v.y) for v in a.victims] == [(v.x, v.y) for v in b.victims]
    assert len(a.parts) == len(b.parts)


def test_outputs(tmp_path):
    paths = generate(seed=1, out_dir=str(tmp_path))
    root = ET.parse(paths['sdf']).getroot()
    models = {m.get('name') for m in root.iter('model')}
    gt = json.load(open(paths['json']))
    assert {v['id'] for v in gt['victims']} <= models
    assert 'structure' in models
    assert gt['explorable_area_m2'] > 100
