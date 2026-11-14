"""Centerline queries: projection, lateral offset, on-track test."""

from __future__ import annotations

import math
from dataclasses import dataclass

from backend.app.schemas.track import TrackDefinition


@dataclass
class Projection:
    s_m: float
    x_m: float
    y_m: float
    yaw_rad: float
    lat_m: float  # signed: +left of tangent
    segment_index: int


class TrackQuery:
    def __init__(
        self,
        track: TrackDefinition,
        *,
        polyline: list[list[float]] | None = None,
    ) -> None:
        self.track = track
        src = polyline if polyline is not None else track.centerline_m
        self._pts = [(float(p[0]), float(p[1])) for p in src if len(p) >= 2]
        self._seg_len: list[float] = []
        self._cum_s: list[float] = [0.0]
        for i in range(len(self._pts) - 1):
            dx = self._pts[i + 1][0] - self._pts[i][0]
            dy = self._pts[i + 1][1] - self._pts[i][1]
            L = math.hypot(dx, dy)
            self._seg_len.append(L)
            self._cum_s.append(self._cum_s[-1] + L)
        self.length_m = self._cum_s[-1] if self._cum_s else 0.0

    def project(self, x: float, y: float) -> Projection:
        if len(self._pts) < 2:
            return Projection(0.0, x, y, 0.0, 0.0, 0)
        best_d2 = float("inf")
        best: Projection | None = None
        for i, L in enumerate(self._seg_len):
            if L < 1e-12:
                continue
            x0, y0 = self._pts[i]
            x1, y1 = self._pts[i + 1]
            dx, dy = x1 - x0, y1 - y0
            t = ((x - x0) * dx + (y - y0) * dy) / (L * L)
            t = max(0.0, min(1.0, t))
            px, py = x0 + t * dx, y0 + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                yaw = math.atan2(dy, dx)
                # signed lateral: +left of travel direction
                lat = (x - px) * (-dy / L) + (y - py) * (dx / L)
                best = Projection(
                    s_m=self._cum_s[i] + t * L,
                    x_m=px,
                    y_m=py,
                    yaw_rad=yaw,
                    lat_m=lat,
                    segment_index=i,
                )
        assert best is not None
        return best

    def lateral_distance(self, x: float, y: float) -> float:
        return self.project(x, y).lat_m

    def is_inside(self, x: float, y: float, margin_m: float = 0.0) -> bool:
        half = self.track.width_m * 0.5 + margin_m
        return abs(self.lateral_distance(x, y)) <= half

    @property
    def start_xy(self) -> tuple[float, float]:
        if not self._pts:
            return (0.0, 0.0)
        return self._pts[0]

    @property
    def end_xy(self) -> tuple[float, float]:
        if not self._pts:
            return (0.0, 0.0)
        return self._pts[-1]
