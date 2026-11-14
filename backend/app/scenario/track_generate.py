"""Parametric track centerline generators (single reference spine)."""

from __future__ import annotations

import math
from typing import Any, Literal

from backend.app.schemas.track import TrackDefinition


def _sample_arc(
    cx: float,
    cy: float,
    radius: float,
    a0: float,
    a1: float,
    step_m: float,
) -> list[list[float]]:
    arc = abs(a1 - a0) * radius
    n = max(4, int(math.ceil(arc / max(step_m, 0.05))))
    pts: list[list[float]] = []
    for i in range(n + 1):
        t = i / n
        a = a0 + t * (a1 - a0)
        pts.append([cx + radius * math.cos(a), cy + radius * math.sin(a), 0.0])
    return pts


def generate_stadium(
    *,
    straight_m: float = 80.0,
    radius_m: float = 30.0,
    resolution_m: float = 2.0,
) -> list[list[float]]:
    """CCW stadium (capsule) oval centered near origin, start at bottom straight going +X."""
    if straight_m < 1.0 or radius_m < 1.0:
        raise ValueError("straight_m and radius_m must be >= 1")
    L = float(straight_m)
    R = float(radius_m)
    step = max(0.5, float(resolution_m))
    # Bottom straight: (-L/2, -R) → (+L/2, -R)
    n_s = max(2, int(math.ceil(L / step)))
    bottom = [
        [-L / 2 + i * L / n_s, -R, 0.0] for i in range(n_s + 1)
    ]
    # Right semicircle around (+L/2, 0): -pi/2 → +pi/2
    right = _sample_arc(L / 2, 0.0, R, -math.pi / 2, math.pi / 2, step)[1:]
    # Top straight: (+L/2, +R) → (-L/2, +R)
    top = [[L / 2 - i * L / n_s, R, 0.0] for i in range(1, n_s + 1)]
    # Left semicircle around (-L/2, 0): +pi/2 → +3pi/2
    left = _sample_arc(-L / 2, 0.0, R, math.pi / 2, 3 * math.pi / 2, step)[1:]
    pts = bottom + right + top + left
    # close
    if math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) > 1e-6:
        pts.append([pts[0][0], pts[0][1], 0.0])
    return pts


def generate_rectangle(
    *,
    length_m: float = 100.0,
    width_m: float = 50.0,
    resolution_m: float = 2.0,
) -> list[list[float]]:
    """Axis-aligned rectangular loop, CCW, start bottom-left going +X."""
    if length_m < 5.0 or width_m < 5.0:
        raise ValueError("length_m and width_m must be >= 5")
    L, W = float(length_m), float(width_m)
    step = max(0.5, float(resolution_m))

    def edge(x0, y0, x1, y1) -> list[list[float]]:
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(seg / step)))
        return [[x0 + i * (x1 - x0) / n, y0 + i * (y1 - y0) / n, 0.0] for i in range(n)]

    pts: list[list[float]] = []
    pts += edge(0.0, 0.0, L, 0.0)
    pts += edge(L, 0.0, L, W)
    pts += edge(L, W, 0.0, W)
    pts += edge(0.0, W, 0.0, 0.0)
    pts.append([0.0, 0.0, 0.0])
    return pts


def generate_straight(
    *,
    length_m: float = 200.0,
    resolution_m: float = 2.0,
) -> list[list[float]]:
    """Open polyline along +X from origin (start → end)."""
    if length_m < 5.0:
        raise ValueError("length_m must be >= 5")
    L = float(length_m)
    step = max(0.5, float(resolution_m))
    n = max(2, int(math.ceil(L / step)))
    return [[i * L / n, 0.0, 0.0] for i in range(n + 1)]


ShapeName = Literal["stadium", "rectangle", "straight"]


def apply_generator(
    track: TrackDefinition,
    *,
    shape: ShapeName,
    params: dict[str, Any],
) -> TrackDefinition:
    res = float(params.get("resolution_m", track.sampling_resolution_m or 2.0))
    closed = True
    if shape == "stadium":
        cl = generate_stadium(
            straight_m=float(params.get("straight_m", 80.0)),
            radius_m=float(params.get("radius_m", 30.0)),
            resolution_m=res,
        )
    elif shape == "rectangle":
        cl = generate_rectangle(
            length_m=float(params.get("length_m", 100.0)),
            width_m=float(params.get("width_m", 50.0)),
            resolution_m=res,
        )
    elif shape == "straight":
        cl = generate_straight(
            length_m=float(params.get("length_m", 200.0)),
            resolution_m=res,
        )
        closed = False
    else:
        raise ValueError(f"unknown shape '{shape}'")
    data = track.model_dump()
    data["centerline_m"] = cl
    # New road shape → reset driving reference to road mid-line (spine copy).
    data["reference_line_m"] = [list(p) for p in cl]
    data["reference_lateral_m"] = None
    data["sampling_resolution_m"] = res
    data["closed_loop"] = closed
    # Lanes are unidirectional unless the author explicitly sets bidirectional.
    data["bidirectional"] = False
    return TrackDefinition.model_validate(data)
