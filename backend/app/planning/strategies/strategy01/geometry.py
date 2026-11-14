"""Frenet project, offset polyline, circle collision."""

from __future__ import annotations

import math
from typing import Sequence

Point = tuple[float, float]
Pts = Sequence[Sequence[float]]

_EPS = 1e-12
_CLOSED_EPS = 1e-3


def _as_points(pts: Pts) -> list[Point]:
    return [(float(p[0]), float(p[1])) for p in pts]


def _prepare(
    pts: Pts,
) -> tuple[list[Point], bool, list[tuple[float, float, float, float, float, float, float]]]:
    """Return (points, closed, segments).

    Each segment is (x0, y0, x1, y1, length, tx, ty) with unit tangent.
    If last≈first the closing vertex is dropped and the loop wraps.
    """
    points = _as_points(pts)
    closed = False
    if len(points) >= 3:
        dx = points[-1][0] - points[0][0]
        dy = points[-1][1] - points[0][1]
        if math.hypot(dx, dy) <= _CLOSED_EPS:
            closed = True
            points = points[:-1]
    n = len(points)
    segs: list[tuple[float, float, float, float, float, float, float]] = []
    if n < 2:
        return points, closed, segs
    nseg = n if closed else n - 1
    for i in range(nseg):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < _EPS:
            tx, ty = 1.0, 0.0
        else:
            tx, ty = dx / length, dy / length
        segs.append((x0, y0, x1, y1, length, tx, ty))
    return points, closed, segs


def _length(segs: Sequence[tuple[float, float, float, float, float, float, float]]) -> float:
    return sum(seg[4] for seg in segs)


def _pose_at_s(
    segs: Sequence[tuple[float, float, float, float, float, float, float]],
    s_m: float,
    closed: bool,
) -> tuple[float, float, float, float, float, float]:
    """Interpolate (px, py, tx, ty, nx, ny) at arc length s_m. Normal is +left."""
    if not segs:
        return 0.0, 0.0, 1.0, 0.0, 0.0, 1.0
    total = _length(segs)
    if total < _EPS:
        x0, y0, _, _, _, tx, ty = segs[0]
        return x0, y0, tx, ty, -ty, tx
    if closed:
        s = s_m % total
        if s < 0:
            s += total
    else:
        s = max(0.0, min(s_m, total))
    remaining = s
    for x0, y0, x1, y1, length, tx, ty in segs:
        if remaining <= length or length < _EPS:
            t = 0.0 if length < _EPS else remaining / length
            t = max(0.0, min(1.0, t))
            px = x0 + t * (x1 - x0)
            py = y0 + t * (y1 - y0)
            return px, py, tx, ty, -ty, tx
        remaining -= length
    x0, y0, x1, y1, _, tx, ty = segs[-1]
    return x1, y1, tx, ty, -ty, tx


def project_to_polyline(
    x: float, y: float, pts: Pts
) -> tuple[float, int, float, float, float, float, float, float]:
    """Project (x, y) onto an open or closed polyline.

    Returns ``(s_m, idx, px, py, tx, ty, nx, ny)`` with unit tangent and
    left-normal. ``s_m`` is cumulative chord length along the polyline.
    """
    _points, _closed, segs = _prepare(pts)
    if not segs:
        if _points:
            px, py = _points[0]
        else:
            px, py = float(x), float(y)
        return 0.0, 0, px, py, 1.0, 0.0, 0.0, 1.0

    best_d2 = float("inf")
    best: tuple[float, int, float, float, float, float, float, float] | None = None
    cum = 0.0
    for i, (x0, y0, x1, y1, length, tx, ty) in enumerate(segs):
        if length < _EPS:
            t = 0.0
            px, py = x0, y0
        else:
            t = ((x - x0) * (x1 - x0) + (y - y0) * (y1 - y0)) / (length * length)
            t = max(0.0, min(1.0, t))
            px = x0 + t * (x1 - x0)
            py = y0 + t * (y1 - y0)
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < best_d2:
            best_d2 = d2
            nx, ny = -ty, tx
            best = (cum + t * length, i, px, py, tx, ty, nx, ny)
        cum += length
    assert best is not None
    return best


def offset_path(
    pts: Pts, s0: float, horizon_m: float, ds_m: float, offset_m: float
) -> list[tuple[float, float]]:
    """Sample the polyline from ``s0`` for ``horizon_m`` and shift by Frenet offset."""
    _points, closed, segs = _prepare(pts)
    if not segs:
        if _points:
            x, y = _points[0]
            return [(x, y), (x, y)]
        return [(0.0, 0.0), (0.0, 0.0)]
    step = ds_m if ds_m > _EPS else max(horizon_m, 1.0)
    horizon = max(0.0, float(horizon_m))
    n = int(horizon / step) + 1
    if n < 2:
        n = 2
        step = horizon if horizon > _EPS else step
    samples: list[tuple[float, float]] = []
    for i in range(n):
        s = s0 + i * step
        px, py, _tx, _ty, nx, ny = _pose_at_s(segs, s, closed)
        samples.append((px + offset_m * nx, py + offset_m * ny))
    return samples


def circles_collide(ax: float, ay: float, ar: float, bx: float, by: float, br: float) -> bool:
    """True if two circles overlap (strict: distance < r_a + r_b)."""
    return math.hypot(ax - bx, ay - by) < (ar + br)
