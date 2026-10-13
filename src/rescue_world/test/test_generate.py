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


def test_hard_world_has_smoke_dark_rooms_and_narrow_doors():
    layout = generate_layout(seed=2, difficulty='hard')
    kinds = [e['type'] for e in layout.extras]
    assert 'smoke' in kinds and 'dark' in kinds
    assert layout.cols == 4 and layout.width == 24.0
    # The entrance is in the middle of a room, not where an interior wall meets the shell.
    assert layout.entrance[0] % 6.0 == 3.0


def test_user_victims_replace_random_ones(tmp_path):
    mine = [{'x': 3.0, 'y': 2.5, 'posture': 'lying', 'yaw': 30},
            {'x': 15.0, 'y': 12.0, 'posture': 'sitting', 'yaw': -90},
            {'x': 20.0, 'y': 7.0, 'buried': True}]
    layout = generate_layout(seed=1, custom_victims=mine)
    assert [(v.x, v.y, v.kind) for v in layout.victims] == [(3.0, 2.5, 'visible'), (15.0, 12.0, 'visible'),
                                                         (20.0, 7.0, 'buried')]
    path = tmp_path / 'v.json'
    path.write_text(json.dumps({'victims': mine}))
    from rescue_world.generate import load_victims
    assert load_victims(str(path)) == mine


def test_smoke_is_invisible_to_lidar():
    # Fortress: only the ParticleEmitter plugin hands the scatter ratio to the lidar, and it
    # reads 0 as "use the 0.65 default"; either mistake makes the robots map smoke as walls.
    from rescue_world.sdf_writer import to_sdf
    root = ET.fromstring(to_sdf(generate_layout(seed=2, difficulty='hard')))
    assert not list(root.iter('particle_emitter'))
    ratios = [float(p.find('particle_scatter_ratio').text) for p in root.iter('plugin')
              if p.get('name') == 'ignition::gazebo::systems::ParticleEmitter']
    assert ratios and all(0 < r < 1e-4 for r in ratios)
