"""Primitive human figures for victims.

Parts are expressed in the victim model frame: origin on the floor at the
pelvis, body axis along +x (head at +x when lying, legs at +x when sitting).
Skin is ~36 C and clothing a few degrees cooler, which is what a thermal
camera sees on a real casualty. Torso and legs are at least 0.17 m tall, above
the robot's climbable step height, so traversability never treats a body as
something to drive over.
"""
from __future__ import annotations

import math
import os

from rescue_world.primitives import Part

# Rescue Randy (Gazebo Fuel, OpenRobotics): the rescue mannequin of the DARPA SubT
# challenge, realistic enough for a COCO-trained person detector. Used when the models
# are in the local Fuel cache (`ign fuel download`, see README); otherwise victims are
# built from primitives.
_FUEL = 'https://fuel.gazebosim.org/1.0/openrobotics/models'
RANDY_MESH = f'{_FUEL}/rescue%20randy/3/files/meshes/rescue_randy.dae'
RANDY_ALBEDO = f'{_FUEL}/rescue%20randy/3/files/materials/textures/rescue_randy_albedo.png'
RANDY_SITTING_MESH = f'{_FUEL}/rescue%20randy%20sitting/8/files/meshes/rescue_randy.dae'
# The sitting mesh refers to its texture by a relative path that doesn't resolve when the
# mesh is loaded by URL, so give it explicitly.
RANDY_SITTING_ALBEDO = f'{_FUEL}/rescue%20randy%20sitting/8/files/materials/textures/RescueRandy_Diffuse.jpg'
_CACHE = os.path.expanduser('~/.ignition/fuel/fuel.gazebosim.org/openrobotics/models')


def randy_available():
    return (os.path.isdir(os.path.join(_CACHE, 'rescue%20randy'))
            and os.path.isdir(os.path.join(_CACHE, 'rescue%20randy%20sitting')))


SKIN_TONES = [(0.96, 0.80, 0.69), (0.87, 0.67, 0.53), (0.68, 0.49, 0.36), (0.45, 0.31, 0.22)]
SHIRTS = [(0.75, 0.12, 0.10), (0.15, 0.30, 0.65), (0.90, 0.75, 0.15), (0.20, 0.55, 0.25),
          (0.85, 0.85, 0.82), (0.35, 0.35, 0.38)]
PANTS = [(0.12, 0.17, 0.32), (0.20, 0.20, 0.22), (0.40, 0.33, 0.24)]
SKIN_K, CLOTH_K = 309.0, 304.0


def build(posture, rng, warm=True, style='primitive'):
    if style == 'randy':
        return _randy(posture, warm)
    skin, shirt, pants = rng.choice(SKIN_TONES), rng.choice(SHIRTS), rng.choice(PANTS)
    ts = SKIN_K if warm else None
    tc = CLOTH_K if warm else None
    if posture == 'lying':
        arm_x = rng.choice([0.05, 0.55])   # arms by the side or stretched above the head
        parts = [
            Part('torso', 'box', (0.55, 0.36, 0.2), (0.15, 0, 0.1, 0, 0, 0), shirt, tc),
            Part('head', 'sphere', (0.11,), (0.55, 0, 0.11, 0, 0, 0), skin, ts),
            Part('leg_l', 'box', (0.85, 0.15, 0.17), (-0.55, 0.09, 0.085, 0, 0, 0), pants, tc),
            Part('leg_r', 'box', (0.85, 0.15, 0.17), (-0.55, -0.09, 0.085, 0, 0, 0), pants, tc),
            Part('arm_l', 'box', (0.55, 0.09, 0.09), (arm_x, 0.24, 0.05, 0, 0, 0), shirt, tc, False),
            Part('arm_r', 'box', (0.55, 0.09, 0.09), (0.05, -0.24, 0.05, 0, 0, 0), shirt, tc, False),
            Part('hand_l', 'sphere', (0.05,), (arm_x + 0.32, 0.24, 0.05, 0, 0, 0), skin, ts, False),
            Part('hand_r', 'sphere', (0.05,), (0.37, -0.24, 0.05, 0, 0, 0), skin, ts, False),
            Part('foot_l', 'box', (0.1, 0.1, 0.2), (-1.0, 0.09, 0.1, 0, 0, 0), pants, tc, False),
            Part('foot_r', 'box', (0.1, 0.1, 0.2), (-1.0, -0.09, 0.1, 0, 0, 0), pants, tc, False),
        ]
    else:
        lean = 0.25   # torso leans back against the wall
        parts = [
            Part('torso', 'box', (0.24, 0.38, 0.58), (-0.02, 0, 0.42, 0, -lean, 0), shirt, tc),
            Part('head', 'sphere', (0.11,), (-0.1, 0, 0.84, 0, 0, 0), skin, ts),
            Part('leg_l', 'box', (0.85, 0.15, 0.17), (0.47, 0.09, 0.085, 0, 0, 0), pants, tc),
            Part('leg_r', 'box', (0.85, 0.15, 0.17), (0.47, -0.09, 0.085, 0, 0, 0), pants, tc),
            Part('arm_l', 'box', (0.09, 0.09, 0.5), (0.0, 0.25, 0.4, 0, -0.1, 0), shirt, tc, False),
            Part('arm_r', 'box', (0.09, 0.09, 0.5), (0.0, -0.25, 0.4, 0, -0.1, 0), shirt, tc, False),
            Part('hand_l', 'sphere', (0.05,), (0.05, 0.25, 0.14, 0, 0, 0), skin, ts, False),
            Part('hand_r', 'sphere', (0.05,), (0.05, -0.25, 0.14, 0, 0, 0), skin, ts, False),
            Part('foot_l', 'box', (0.1, 0.1, 0.2), (0.92, 0.09, 0.1, 0, 0, 0), pants, tc, False),
            Part('foot_r', 'box', (0.1, 0.1, 0.2), (0.92, -0.09, 0.1, 0, 0, 0), pants, tc, False),
        ]
    return parts


def _randy(posture, warm):
    """Mesh victim plus invisible collision boxes, in the same frame as the primitives."""
    t = SKIN_K if warm else None
    white = (1.0, 1.0, 1.0)
    if posture == 'lying':
        # Mesh: standing T-pose, feet at the origin, 1.82 m tall along z. Rotate height
        # onto +x (head) and chest depth onto z, so he lies on his back.
        return [
            Part('body', 'mesh', (RANDY_MESH, RANDY_ALBEDO), (-1.12, 0, 0.18, math.pi / 2, 0, math.pi / 2),
                 white, t, collide=False),
            Part('col_body', 'box', (1.8, 0.5, 0.3), (-0.22, 0, 0.15, 0, 0, 0), white, None, True, False),
        ]
    # Sitting mesh: legs along -y, back at +y, seat 0.62 m above the mesh origin.
    return [
        Part('body', 'mesh', (RANDY_SITTING_MESH, RANDY_SITTING_ALBEDO), (0.15, 0.10, -0.618, 0, 0, math.pi / 2),
             white, t, collide=False),
        Part('col_body', 'box', (0.95, 0.5, 0.7), (0.38, 0, 0.35, 0, 0, 0), white, None, True, False),
    ]
