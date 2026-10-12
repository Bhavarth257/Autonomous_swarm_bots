"""Geometry types shared by the layout generator, the human figures and the SDF writer."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Part:
    """A single primitive: rendered, optionally collidable, optionally warm."""
    name: str
    shape: str                 # box | sphere | cylinder
    size: tuple                # box (sx, sy, sz) | sphere (r,) | cylinder (r, length)
    pose: tuple                # x y z roll pitch yaw
    color: tuple
    temperature: float | None = None
    collide: bool = True


@dataclass
class Footprint:
    """Oriented floor rectangle covered by an obstacle."""
    x: float
    y: float
    yaw: float
    lx: float
    ly: float
    climbable: bool = False    # a hexabot can walk over it
