"""Build an RViz2 config for N namespaced robots (fixed frame: world)."""
from __future__ import annotations

import yaml

COLORS = ['255; 120; 0', '0; 170; 255', '200; 0; 200', '0; 200; 80', '230; 200; 0']


def _qos(topic, reliable=True, transient=False):
    return {'Value': topic, 'Depth': 5, 'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable' if reliable else 'Best Effort',
            'Durability Policy': 'Transient Local' if transient else 'Volatile'}


def robot_displays(ns, color):
    return {
        'Class': 'rviz_common/Group', 'Name': ns, 'Enabled': True,
        'Displays': [
            {'Class': 'rviz_default_plugins/RobotModel', 'Name': 'model', 'Enabled': True,
             'Description Source': 'Topic', 'TF Prefix': ns, 'Visual Enabled': True,
             'Description Topic': _qos(f'/{ns}/robot_description', transient=True)},
            {'Class': 'rviz_default_plugins/PointCloud2', 'Name': 'lidar 3D', 'Enabled': ns == 'r1',
             'Topic': _qos(f'/{ns}/lidar/points', reliable=False), 'Color Transformer': 'AxisColor',
             'Axis': 'Z', 'Size (m)': 0.03, 'Style': 'Flat Squares', 'Decay Time': 0},
            {'Class': 'rviz_default_plugins/LaserScan', 'Name': 'scan', 'Enabled': True,
             'Topic': _qos(f'/{ns}/scan', reliable=False), 'Color Transformer': 'FlatColor',
             'Color': color, 'Size (m)': 0.05, 'Style': 'Flat Squares'},
            {'Class': 'rviz_default_plugins/Odometry', 'Name': 'odom', 'Enabled': False,
             'Topic': _qos(f'/{ns}/odom')},
            {'Class': 'rviz_default_plugins/Image', 'Name': 'camera', 'Enabled': ns == 'r1',
             'Topic': _qos(f'/{ns}/camera/image_raw', reliable=False)},
        ],
    }


def build(namespaces):
    cfg = {
        'Panels': [
            {'Class': 'rviz_common/Displays', 'Name': 'Displays'},
            {'Class': 'rviz_common/Views', 'Name': 'Views'},
        ],
        'Visualization Manager': {
            'Class': '',
            'Global Options': {'Fixed Frame': 'world', 'Background Color': '48; 48; 48',
                               'Frame Rate': 30},
            'Displays': [
                {'Class': 'rviz_default_plugins/Grid', 'Name': 'Grid', 'Enabled': True,
                 'Reference Frame': '<Fixed Frame>', 'Cell Size': 1, 'Plane Cell Count': 30,
                 'Offset': {'X': 9, 'Y': 7, 'Z': 0}},
                {'Class': 'rviz_default_plugins/TF', 'Name': 'TF', 'Enabled': False},
            ] + [robot_displays(ns, COLORS[k % len(COLORS)]) for k, ns in enumerate(namespaces)],
            'Tools': [{'Class': 'rviz_default_plugins/Interact'},
                      {'Class': 'rviz_default_plugins/MoveCamera'},
                      {'Class': 'rviz_default_plugins/SetGoal', 'Topic': _qos('/r1/goal_pose')},
                      {'Class': 'rviz_default_plugins/PublishPoint',
                       'Topic': _qos('/clicked_point')}],
            'Views': {'Current': {'Class': 'rviz_default_plugins/TopDownOrtho', 'Name': 'Current View',
                                  'X': 9, 'Y': 6, 'Scale': 28, 'Angle': 0,
                                  'Target Frame': '<Fixed Frame>'}},
        },
        'Window Geometry': {'Width': 1400, 'Height': 900},
    }
    return yaml.safe_dump(cfg, sort_keys=False)
