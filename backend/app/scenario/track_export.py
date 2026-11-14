"""Export track reference line + left/right road boundaries (+ per-lane)."""

from __future__ import annotations

import math
from typing import Any

from backend.app.schemas.track import TrackDefinition


def _polyline_headings(pts: list[tuple[float, float]], closed: bool) -> list[float]:
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


def _cum_s(pts: list[tuple[float, float]]) -> list[float]:
    s = [0.0]
    for i in range(len(pts) - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        s.append(s[-1] + math.hypot(dx, dy))
    return s


def densify_centerline(
    centerline_m: list[list[float]],
    *,
    closed: bool,
    resolution_m: float,
) -> list[list[float]]:
    """Smooth (Catmull-Rom) then resample at roughly resolution_m spacing."""
    from backend.app.scenario.polyline_smooth import smooth_resample_polyline

    if resolution_m <= 0 or len(centerline_m) < 2:
        return [list(p) for p in centerline_m]
    return smooth_resample_polyline(
        centerline_m, closed=closed, resolution_m=float(resolution_m)
    )


def _offset_polyline(
    cl_work: list[list[float]],
    yaws: list[float],
    s_vals: list[float],
    lat_m: float,
) -> list[dict[str, float]]:
    out: list[dict[str, float]] = []
    for i, (p, yaw) in enumerate(zip(cl_work, yaws)):
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = -s, c  # left normal
        x, y, z = p[0], p[1], p[2] if len(p) > 2 else 0.0
        out.append(
            {
                "x_m": x + lat_m * lx,
                "y_m": y + lat_m * ly,
                "z_m": z,
                "s_m": s_vals[i],
                "yaw_rad": yaw,
            }
        )
    return out


def export_track_geometry(
    track: TrackDefinition,
    *,
    resolution_m: float | None = None,
) -> dict[str, Any]:
    """Build reference line + outer boundaries + per-lane geometry.

    Multi-lane: single reference spine; each lane is a lateral offset
    (index 0 = rightmost). Avoids independently editable lane paths.
    """
    res = resolution_m if resolution_m is not None else float(track.sampling_resolution_m or 0.5)
    cl = densify_centerline(
        track.centerline_m, closed=bool(track.closed_loop), resolution_m=max(0.05, res)
    )
    xy = [(p[0], p[1]) for p in cl]
    closed = bool(track.closed_loop)
    if closed and len(xy) > 2 and math.hypot(xy[-1][0] - xy[0][0], xy[-1][1] - xy[0][1]) < 1e-6:
        xy_work = xy[:-1]
        cl_work = cl[:-1]
    else:
        xy_work = xy
        cl_work = cl

    yaws = _polyline_headings(xy_work, closed=closed)
    s_vals = _cum_s(xy_work + ([xy_work[0]] if closed else []))
    if closed:
        s_vals = s_vals[:-1]

    half = float(track.width_m) * 0.5
    road_centerline = _offset_polyline(cl_work, yaws, s_vals, 0.0)
    left_boundary = [
        {k: v for k, v in p.items() if k != "yaw_rad"}
        for p in _offset_polyline(cl_work, yaws, s_vals, half)
    ]
    right_boundary = [
        {k: v for k, v in p.items() if k != "yaw_rad"}
        for p in _offset_polyline(cl_work, yaws, s_vals, -half)
    ]

    # Independent driving reference (may differ from road spine).
    ref_src = densify_centerline(
        track.driving_reference_m(),
        closed=bool(track.closed_loop),
        resolution_m=max(0.05, res),
    )
    ref_xy = [(p[0], p[1]) for p in ref_src]
    if closed and len(ref_xy) > 2 and math.hypot(ref_xy[-1][0] - ref_xy[0][0], ref_xy[-1][1] - ref_xy[0][1]) < 1e-6:
        ref_xy_work = ref_xy[:-1]
        ref_cl_work = ref_src[:-1]
    else:
        ref_xy_work = ref_xy
        ref_cl_work = ref_src
    ref_yaws = _polyline_headings(ref_xy_work, closed=closed)
    ref_s = _cum_s(ref_xy_work + ([ref_xy_work[0]] if closed else []))
    if closed:
        ref_s = ref_s[:-1]
    reference_line = _offset_polyline(ref_cl_work, ref_yaws, ref_s, 0.0)

    lw = float(track.lane_width_m or (track.width_m / track.num_lanes))
    lanes: list[dict[str, Any]] = []
    for i, lat in enumerate(track.lane_center_offsets_m()):
        center = _offset_polyline(cl_work, yaws, s_vals, lat)
        left = [
            {k: v for k, v in p.items() if k != "yaw_rad"}
            for p in _offset_polyline(cl_work, yaws, s_vals, lat + lw * 0.5)
        ]
        right = [
            {k: v for k, v in p.items() if k != "yaw_rad"}
            for p in _offset_polyline(cl_work, yaws, s_vals, lat - lw * 0.5)
        ]
        if closed and center:
            center = center + [dict(center[0])]
            left = left + [dict(left[0])]
            right = right + [dict(right[0])]
        lanes.append(
            {
                "index": i,
                "id": f"lane_{i}",
                "side": "rightmost" if i == 0 else ("leftmost" if i == track.num_lanes - 1 else "inner"),
                "lateral_offset_m": lat,
                "width_m": lw,
                "center_line": center,
                "left_boundary": left,
                "right_boundary": right,
            }
        )

    if closed and reference_line:
        reference_line.append(dict(reference_line[0]))
        road_centerline.append(dict(road_centerline[0]))
        left_boundary.append(dict(left_boundary[0]))
        right_boundary.append(dict(right_boundary[0]))

    length_m = s_vals[-1] if s_vals else 0.0
    if closed and len(xy_work) >= 2:
        length_m += math.hypot(
            xy_work[0][0] - xy_work[-1][0], xy_work[0][1] - xy_work[-1][1]
        )

    return {
        "schema_version": "1.2",
        "track_id": track.track_id,
        "name": track.name,
        "closed_loop": track.closed_loop,
        "width_m": track.width_m,
        "num_lanes": track.num_lanes,
        "lane_width_m": lw,
        "friction_mu": track.friction_mu,
        "resolution_m": res,
        "length_m": length_m,
        "n_points": len(reference_line),
        "frame": "platform_xy_m",
        "notes": {
            "model": "road spine + equal-width lanes; independent editable driving reference_line_m",
            "road_centerline": "geometric road center (spine) / 道路脊线",
            "reference_line": "Pure-Pursuit / driving reference (independent polyline)",
            "left_boundary": "outer road shoulder at +width/2 (left of travel)",
            "right_boundary": "outer road shoulder at -width/2 (right of travel)",
            "lanes": "index 0 = rightmost; each has center_line + left/right edges",
        },
        "road_centerline": road_centerline,
        "reference_line": reference_line,
        "left_boundary": left_boundary,
        "right_boundary": right_boundary,
        "lanes": lanes,
        "centerline_m": track.centerline_m,
        "reference_line_m": track.driving_reference_m(),
    }
