"""Cheap one-shot AES swerve from static obstacles (S3 autonomy arm)."""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence


def _smoothstep(u: float) -> float:
    u = max(0.0, min(1.0, float(u)))
    return u * u * (3.0 - 2.0 * u)


def swerve_path_from_obstacle(
    centerline_xy: Sequence[Sequence[float]],
    obstacle: Mapping[str, Any],
    *,
    road_half_width_m: float = 6.0,
    ego_width_m: float = 2.0,
    min_clearance_m: float = 0.3,
    entry_lead_m: float = 25.0,
    exit_trail_m: float = 25.0,
    pad_m: float = 1.0,
) -> tuple[list[list[float]], float]:
    """Return ``(path_xy, peak_offset_m)`` swerving away from obstacle."""
    if len(centerline_xy) < 3:
        return [[float(p[0]), float(p[1])] for p in centerline_xy], 0.0

    ox = float(obstacle.get("x_m", 0.0) or 0.0)
    oy = float(obstacle.get("y_m", 0.0) or 0.0)
    o_w = float(obstacle.get("width_m", 2.5) or 2.5)
    half_sum = 0.5 * float(ego_width_m) + 0.5 * o_w
    need = half_sum + float(min_clearance_m) + float(pad_m)
    side_sign = -1.0 if oy >= 0.0 else 1.0
    peak = float(oy) + side_sign * need
    if side_sign < 0:
        peak = min(peak, oy - need)
    else:
        peak = max(peak, oy + need)
    max_abs = float(road_half_width_m) - 0.25
    if abs(peak) > max_abs:
        peak = math.copysign(max_abs, peak)

    x_entry = ox - float(entry_lead_m)
    x_exit = ox + float(exit_trail_m)
    x_hold0 = ox - 8.0
    x_hold1 = ox + 8.0
    path: list[list[float]] = []
    for p in centerline_xy:
        x = float(p[0])
        y0 = float(p[1]) if len(p) > 1 else 0.0
        if x <= x_entry:
            lat = 0.0
        elif x < x_hold0:
            lat = peak * _smoothstep((x - x_entry) / max(1e-6, x_hold0 - x_entry))
        elif x <= x_hold1:
            lat = peak
        elif x < x_exit:
            lat = peak * (1.0 - _smoothstep((x - x_hold1) / max(1e-6, x_exit - x_hold1)))
        else:
            lat = 0.0
        path.append([x, y0 + lat])
    return path, peak
