"""2D OBB helpers for ego / NPC / obstacle collision (geometry only)."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class Box2D:
    x_m: float
    y_m: float
    yaw_rad: float
    length_m: float
    width_m: float

    def corners(self) -> list[tuple[float, float]]:
        hx = self.length_m * 0.5
        hy = self.width_m * 0.5
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        local = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
        return [
            (self.x_m + c * lx - s * ly, self.y_m + s * lx + c * ly) for lx, ly in local
        ]


def _axes(corners: list[tuple[float, float]]) -> list[tuple[float, float]]:
    axes: list[tuple[float, float]] = []
    for i in range(2):
        x0, y0 = corners[i]
        x1, y1 = corners[(i + 1) % 4]
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy) or 1.0
        # edge normal
        axes.append((-dy / L, dx / L))
    return axes


def _project(corners: list[tuple[float, float]], axis: tuple[float, float]) -> tuple[float, float]:
    ax, ay = axis
    dots = [p[0] * ax + p[1] * ay for p in corners]
    return min(dots), max(dots)


def boxes_overlap(a: Box2D, b: Box2D) -> bool:
    """Separating-axis test for two oriented boxes."""
    ca, cb = a.corners(), b.corners()
    for axis in _axes(ca) + _axes(cb):
        amin, amax = _project(ca, axis)
        bmin, bmax = _project(cb, axis)
        if amax < bmin or bmax < amin:
            return False
    return True
