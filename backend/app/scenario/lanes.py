"""Per-lane geometry derived from the road spine (equal-width lanes)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from backend.app.schemas.track import TrackDefinition


@dataclass(frozen=True)
class LaneGeometry:
    """One lane's centerline and edges in platform XY meters.

    Index 0 = rightmost lane (same convention as track export).
    ``travel_forward``: True → follow spine sense; False → oncoming (polyline reversed for NPCs).
    """

    index: int
    id: str
    side: str  # rightmost | leftmost | inner
    lateral_offset_m: float
    width_m: float
    centerline_m: list[list[float]]
    left_boundary_m: list[list[float]]
    right_boundary_m: list[list[float]]
    length_m: float
    travel_forward: bool = True

    def as_dict(self) -> dict:
        return {
            "index": self.index,
            "id": self.id,
            "side": self.side,
            "lateral_offset_m": self.lateral_offset_m,
            "width_m": self.width_m,
            "centerline_m": self.centerline_m,
            "left_boundary_m": self.left_boundary_m,
            "right_boundary_m": self.right_boundary_m,
            "length_m": self.length_m,
            "travel_forward": self.travel_forward,
        }


def _polyline_length_m(pts: list[list[float]]) -> float:
    total = 0.0
    for i in range(len(pts) - 1):
        total += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
    return total


def build_lane_geometries(track: TrackDefinition) -> list[LaneGeometry]:
    """Build all lane polylines from ``track`` (spine + num_lanes × lane_width)."""
    lw = float(track.lane_width_m or (track.width_m / track.num_lanes))
    lanes: list[LaneGeometry] = []
    n = track.num_lanes
    bidirectional = track.is_bidirectional()
    for i, lat in enumerate(track.lane_center_offsets_m()):
        center = track.offset_polyline_m(lat)
        left = track.offset_polyline_m(lat + lw * 0.5)
        right = track.offset_polyline_m(lat - lw * 0.5)
        side = "rightmost" if i == 0 else ("leftmost" if i == n - 1 else "inner")
        # Left of spine (+lat) = oncoming when bidirectional.
        travel_forward = True if not bidirectional else (lat <= 1e-9)
        lanes.append(
            LaneGeometry(
                index=i,
                id=f"lane_{i}",
                side=side,
                lateral_offset_m=lat,
                width_m=lw,
                centerline_m=center,
                left_boundary_m=left,
                right_boundary_m=right,
                length_m=_polyline_length_m(center),
                travel_forward=travel_forward,
            )
        )
    return lanes
