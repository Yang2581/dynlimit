"""Friction lookup: default track μ overridden by polygon zones."""

from __future__ import annotations

import math
from typing import Sequence

from backend.app.schemas.track import FrictionZone, TrackDefinition


def point_in_polygon(x: float, y: float, points_xy: Sequence[Sequence[float]]) -> bool:
    """Ray-casting even-odd rule (boundary counts as inside)."""
    n = len(points_xy)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = float(points_xy[i][0]), float(points_xy[i][1])
        xj, yj = float(points_xy[j][0]), float(points_xy[j][1])
        # On vertex
        if math.hypot(x - xi, y - yi) < 1e-9:
            return True
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-30) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def friction_at(
    track: TrackDefinition, x: float, y: float
) -> tuple[float, str | None]:
    """Return (μ, zone_id). Later zones in the list override earlier ones."""
    zones: list[FrictionZone] = list(track.friction_zones or [])
    hit_id: str | None = None
    mu = float(track.friction_mu)
    for z in zones:
        if point_in_polygon(x, y, z.points_xy):
            mu = float(z.mu)
            hit_id = z.id
    return mu, hit_id
