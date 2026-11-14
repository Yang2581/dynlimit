"""Polyline projection helpers for geometric path trackers."""

from __future__ import annotations

import math
from dataclasses import dataclass


def dist(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)


@dataclass(frozen=True)
class PathProjection:
    seg_index: int
    t: float
    proj_x: float
    proj_y: float
    crosstrack_m: float  # + left of path tangent
    path_yaw_rad: float


def project_on_polyline(
    x: float,
    y: float,
    waypoints: list[tuple[float, float]],
    *,
    hint_idx: int = 0,
) -> PathProjection:
    """Closest point on open polyline; search biased near ``hint_idx``."""
    n = len(waypoints)
    if n < 2:
        raise ValueError("need >= 2 waypoints")

    best_i = 0
    best_t = 0.0
    best_d2 = float("inf")
    best_px, best_py = waypoints[0]
    # Windowed search around hint, then full scan fallback if needed
    window = list(range(max(0, hint_idx - 5), min(n - 1, hint_idx + 40)))
    if not window:
        window = list(range(0, n - 1))
    candidates = window + [i for i in range(0, n - 1) if i not in window]

    for i in candidates:
        ax, ay = waypoints[i]
        bx, by = waypoints[i + 1]
        abx, aby = bx - ax, by - ay
        ab2 = abx * abx + aby * aby
        if ab2 < 1e-12:
            continue
        t = ((x - ax) * abx + (y - ay) * aby) / ab2
        t_c = max(0.0, min(1.0, t))
        px = ax + t_c * abx
        py = ay + t_c * aby
        d2 = (x - px) * (x - px) + (y - py) * (y - py)
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
            best_t = t_c
            best_px, best_py = px, py

    ax, ay = waypoints[best_i]
    bx, by = waypoints[best_i + 1]
    tx, ty = bx - ax, by - ay
    path_yaw = math.atan2(ty, tx)
    tang_n = math.hypot(tx, ty) + 1e-12
    ex, ey = x - best_px, y - best_py
    signed_e = (tx * ey - ty * ex) / tang_n
    return PathProjection(
        seg_index=best_i,
        t=best_t,
        proj_x=best_px,
        proj_y=best_py,
        crosstrack_m=signed_e,
        path_yaw_rad=path_yaw,
    )
