"""Track definition (authoritative geometry source for MVP)."""

from __future__ import annotations

import math

from pydantic import BaseModel, Field, field_validator, model_validator


def _clean_polyline(pts: list[list[float]], *, name: str) -> list[list[float]]:
    if len(pts) < 3:
        raise ValueError(f"{name} needs at least 3 points")
    cleaned: list[list[float]] = []
    for i, p in enumerate(pts):
        if len(p) < 2:
            raise ValueError(f"{name} point {i} must have [x, y, ...]")
        x, y = float(p[0]), float(p[1])
        z = float(p[2]) if len(p) > 2 else 0.0
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            raise ValueError(f"{name} point {i} has non-finite values")
        cleaned.append([x, y, z])
    return cleaned


def _ensure_segments_and_close(
    pts: list[list[float]], *, closed: bool, name: str
) -> list[list[float]]:
    min_seg = 1e-4
    # Open roads must not carry a loop-closure vertex (common editor artifact).
    # That would make end≈start and falsely trigger open-loop goal finish.
    if (not closed) and len(pts) >= 3:
        while len(pts) >= 3:
            dx = pts[-1][0] - pts[0][0]
            dy = pts[-1][1] - pts[0][1]
            if math.hypot(dx, dy) > 0.05:
                break
            pts = list(pts[:-1])
    for i in range(len(pts) - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        if math.hypot(dx, dy) < min_seg:
            raise ValueError(f"{name} segment {i} is too short")
    if closed:
        dx = pts[-1][0] - pts[0][0]
        dy = pts[-1][1] - pts[0][1]
        if math.hypot(dx, dy) > 0.05:
            pts = list(pts) + [[pts[0][0], pts[0][1], pts[0][2]]]
    return pts


def _polyline_yaws(pts: list[list[float]], closed: bool) -> list[float]:
    n = len(pts)
    if n < 2:
        return [0.0] * n
    yaws: list[float] = []
    for i in range(n):
        if i < n - 1:
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
        elif closed and n > 2:
            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]
        else:
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
        if abs(dx) < 1e-12 and abs(dy) < 1e-12 and i > 0:
            yaws.append(yaws[-1])
        else:
            yaws.append(math.atan2(dy, dx))
    return yaws


class FrictionZone(BaseModel):
    """Planar friction patch in track XY (meters). Last matching zone wins."""

    id: str
    mu: float = Field(gt=0.0, le=2.0, description="Friction coefficient μ")
    kind: str = "polygon"
    points_xy: list[list[float]] = Field(
        description="Polygon vertices [x, y] (closed optional)"
    )

    @field_validator("points_xy")
    @classmethod
    def _poly_ok(cls, pts: list[list[float]]) -> list[list[float]]:
        if len(pts) < 3:
            raise ValueError("friction zone needs ≥3 points")
        cleaned: list[list[float]] = []
        for i, p in enumerate(pts):
            if len(p) < 2:
                raise ValueError(f"friction point {i} must have [x, y]")
            x, y = float(p[0]), float(p[1])
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError(f"friction point {i} non-finite")
            cleaned.append([x, y])
        return cleaned


class TrackDefinition(BaseModel):
    """Road spine + equal-width lanes + independently editable driving reference.

    ``centerline_m`` = geometric road center (spine). Multi-lane is derived:
    ``width_m = num_lanes * lane_width_m`` (lane index 0 = rightmost).

    ``reference_line_m`` = Pure-Pursuit / driving path. Defaults to a copy of the
    spine (mid-road). Points are edited independently of the road surface.
    """

    schema_version: str = "1.0"
    track_id: str
    closed_loop: bool = True
    width_m: float = 10.0
    friction_mu: float = 1.0
    centerline_m: list[list[float]] = Field(
        description="Road-center (spine) polyline [x, y, z] meters"
    )
    sampling_resolution_m: float = 0.5
    name: str | None = None
    notes: str | None = None
    num_lanes: int = 1
    lane_width_m: float | None = None
    # Independent driving reference polyline (defaults to spine copy).
    reference_line_m: list[list[float]] | None = None
    # Legacy: if set and reference_line_m omitted, build reference by constant offset.
    reference_lateral_m: float | None = None
    # Explicit oncoming left-of-spine; None/False → unidirectional (platform default).
    bidirectional: bool | None = None
    friction_zones: list[FrictionZone] = Field(default_factory=list)

    def is_bidirectional(self) -> bool:
        """Oncoming left-of-spine lanes only when explicitly enabled.

        Platform default is unidirectional: every lane shares the spine travel sense.
        """
        return bool(self.bidirectional) if self.bidirectional is not None else False

    @field_validator("width_m")
    @classmethod
    def _width_positive(cls, v: float) -> float:
        if v <= 0:
            raise ValueError("width_m must be > 0")
        return v

    @field_validator("num_lanes")
    @classmethod
    def _lanes_ok(cls, v: int) -> int:
        if v < 1 or v > 8:
            raise ValueError("num_lanes must be in 1..8")
        return int(v)

    @field_validator("centerline_m")
    @classmethod
    def _centerline_ok(cls, pts: list[list[float]]) -> list[list[float]]:
        return _clean_polyline(pts, name="centerline_m")

    @field_validator("reference_line_m")
    @classmethod
    def _reference_ok(
        cls, pts: list[list[float]] | None
    ) -> list[list[float]] | None:
        if pts is None:
            return None
        return _clean_polyline(pts, name="reference_line_m")

    @model_validator(mode="after")
    def _segments_lanes_and_close(self) -> TrackDefinition:
        self.centerline_m = _ensure_segments_and_close(
            self.centerline_m, closed=bool(self.closed_loop), name="centerline_m"
        )

        if self.lane_width_m is None:
            self.lane_width_m = float(self.width_m) / float(self.num_lanes)
        if self.lane_width_m <= 0:
            raise ValueError("lane_width_m must be > 0")
        self.width_m = float(self.num_lanes) * float(self.lane_width_m)

        if self.reference_line_m is None:
            lat = float(self.reference_lateral_m or 0.0)
            if abs(lat) > 1e-12:
                self.reference_line_m = self.offset_polyline_m(lat)
            else:
                self.reference_line_m = [list(p) for p in self.centerline_m]
        else:
            self.reference_line_m = _ensure_segments_and_close(
                self.reference_line_m,
                closed=bool(self.closed_loop),
                name="reference_line_m",
            )
        return self

    def lane_center_offsets_m(self) -> list[float]:
        """Lateral offsets of each lane center from spine (+ = left of travel).

        Index 0 = rightmost lane.
        """
        n = self.num_lanes
        lw = float(self.lane_width_m or (self.width_m / n))
        half = self.width_m * 0.5
        return [-half + (i + 0.5) * lw for i in range(n)]

    def dense_spine_m(self) -> list[list[float]]:
        """Road spine smoothed + densified for lane geometry / export."""
        from backend.app.scenario.polyline_smooth import smooth_resample_polyline

        return smooth_resample_polyline(
            self.centerline_m,
            closed=bool(self.closed_loop),
            resolution_m=float(self.sampling_resolution_m or 0.5),
        )

    def offset_polyline_m(self, lat_m: float, *, dense: bool = True) -> list[list[float]]:
        """Offset spine by constant lateral meters (+ = left of travel).

        ``dense=True`` (default) offsets the smoothed/densified spine so lane
        edges follow the curve instead of sparse control chords.
        """
        pts = self.dense_spine_m() if dense else self.centerline_m
        if abs(lat_m) < 1e-12:
            return [list(p) for p in pts]
        closed = bool(self.closed_loop)
        work = pts
        if (
            closed
            and len(pts) > 2
            and math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) < 1e-6
        ):
            work = pts[:-1]
        yaws = _polyline_yaws(work, closed=closed)
        out: list[list[float]] = []
        for p, yaw in zip(work, yaws):
            s = math.sin(yaw)
            c = math.cos(yaw)
            lx, ly = -s, c
            z = p[2] if len(p) > 2 else 0.0
            out.append([p[0] + lat_m * lx, p[1] + lat_m * ly, z])
        if closed and out:
            out.append([out[0][0], out[0][1], out[0][2]])
        return out

    def reference_control_m(self) -> list[list[float]]:
        """Sparse authoring vertices of the driving reference (editor handles)."""
        if self.reference_line_m is None:
            return [list(p) for p in self.centerline_m]
        return [list(p) for p in self.reference_line_m]

    def driving_reference_m(self) -> list[list[float]]:
        """Pure-Pursuit / MPPI waypoints: smoothed + densified reference."""
        from backend.app.scenario.polyline_smooth import smooth_resample_polyline

        return smooth_resample_polyline(
            self.reference_control_m(),
            closed=bool(self.closed_loop),
            resolution_m=float(self.sampling_resolution_m or 0.5),
        )

    def lanes(self) -> list:
        """Per-lane geometry (centerline + edges). See ``scenario.lanes``."""
        from backend.app.scenario.lanes import build_lane_geometries

        return build_lane_geometries(self)
