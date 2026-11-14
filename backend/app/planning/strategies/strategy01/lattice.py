"""Minimal lateral-offset lattice: sample, predict, cost, select."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Sequence

from backend.app.planning.strategies.strategy01.geometry import (
    circles_collide,
    offset_path,
    project_to_polyline,
)

Npc = Mapping[str, object]


@dataclass
class LatticeParams:
    lateral_offsets_m: list[float] = field(default_factory=list)
    d_m: float = 1.75
    horizon_m: float = 40.0
    ds_m: float = 2.0
    ego_radius_m: float = 1.5
    npc_radius_m: float = 1.5
    predict_dt_s: float = 0.2
    w_lat: float = 1.0
    w_change: float = 0.5
    v_min_mps: float = 1.0
    road_half_width_m: float | None = None  # if set, prune paths outside road

    def __post_init__(self) -> None:
        if not self.lateral_offsets_m:
            d = self.d_m
            self.lateral_offsets_m = [-2.0 * d, -d, 0.0, d, 2.0 * d]


@dataclass
class LatticeResult:
    path_xy: list[list[float]]
    offset_m: float
    collision_free: bool
    n_candidates: int
    n_legal: int


def _npc_xy(npc: Npc, t: float) -> tuple[float, float]:
    x = float(npc.get("x_m", 0.0) or 0.0)
    y = float(npc.get("y_m", 0.0) or 0.0)
    yaw = float(npc.get("yaw_rad", 0.0) or 0.0)
    vx = float(npc.get("vx_mps", 0.0) or 0.0)
    return x + vx * math.cos(yaw) * t, y + vx * math.sin(yaw) * t


def obstacle_radius_m(obs: Mapping[str, object]) -> float:
    """Circle radius enclosing an axis-aligned obstacle box."""
    length = float(obs.get("length_m", 2.0) or 2.0)
    width = float(obs.get("width_m", 2.0) or 2.0)
    return 0.5 * math.hypot(length, width)


def _path_cum(path: Sequence[tuple[float, float]]) -> list[float]:
    cum = [0.0]
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        cum.append(cum[-1] + math.hypot(dx, dy))
    return cum


def _point_nearest_travel(path: Sequence[tuple[float, float]], travel_m: float) -> tuple[float, float]:
    if not path:
        return 0.0, 0.0
    cum = _path_cum(path)
    total = cum[-1]
    target = max(0.0, min(float(travel_m), total))
    best_i = 0
    best_d = abs(cum[0] - target)
    for i, s in enumerate(cum):
        d = abs(s - target)
        if d < best_d:
            best_d = d
            best_i = i
    return path[best_i]


def plan_lattice(
    *,
    ego_x: float,
    ego_y: float,
    ego_yaw: float,
    ego_v: float,
    centerline_xy: Sequence[Sequence[float]],
    npcs: Sequence[Npc],
    params: LatticeParams,
    last_offset_m: float = 0.0,
    obstacles: Sequence[Npc] | None = None,
) -> LatticeResult:
    """Sample lateral offsets, prune colliding predictions, pick min cost legal path."""
    _ = ego_yaw
    offsets = list(params.lateral_offsets_m)
    s0, *_rest = project_to_polyline(ego_x, ego_y, centerline_xy)
    v = max(float(ego_v), params.v_min_mps)
    horizon = params.horizon_m
    T = horizon / v if v > 0.0 else 0.0
    dt = params.predict_dt_s
    times: list[float] = []
    t = 0.0
    if T <= 0.0:
        times = [0.0]
    else:
        if dt <= 0.0:
            dt = T
        while t <= T + 1e-12:
            times.append(t)
            t += dt

    npc_list = list(npcs or [])
    obs_list = list(obstacles or [])
    scored: list[tuple[bool, int, float, float, list[tuple[float, float]]]] = []
    n_legal = 0
    for off in offsets:
        path = offset_path(centerline_xy, s0, horizon, params.ds_m, off)
        illegal = 0
        for ti in times:
            ex, ey = _point_nearest_travel(path, v * ti)
            if params.road_half_width_m is not None:
                # Approximate road bound as |y| for straight paper tracks.
                if abs(ey) > float(params.road_half_width_m):
                    illegal += 1
            for npc in npc_list:
                nx, ny = _npc_xy(npc, ti)
                if circles_collide(ex, ey, params.ego_radius_m, nx, ny, params.npc_radius_m):
                    illegal += 1
            for obs in obs_list:
                ox = float(obs.get("x_m", 0.0) or 0.0)
                oy = float(obs.get("y_m", 0.0) or 0.0)
                orad = obstacle_radius_m(obs)
                if circles_collide(ex, ey, params.ego_radius_m, ox, oy, orad):
                    illegal += 1
        legal = illegal == 0
        if legal:
            n_legal += 1
        cost = params.w_lat * abs(off) + params.w_change * abs(off - last_offset_m)
        scored.append((legal, illegal, cost, off, path))

    n_candidates = len(scored)
    if not scored:
        return LatticeResult(
            path_xy=[],
            offset_m=0.0,
            collision_free=False,
            n_candidates=0,
            n_legal=0,
        )

    legal_ones = [c for c in scored if c[0]]
    if legal_ones:
        _legal, _ill, _cost, off, path = min(legal_ones, key=lambda c: (c[2], abs(c[3])))
        collision_free = True
    else:
        _legal, _ill, _cost, off, path = min(
            scored, key=lambda c: (c[1], abs(c[3]), c[2])
        )
        collision_free = False

    return LatticeResult(
        path_xy=[[p[0], p[1]] for p in path],
        offset_m=off,
        collision_free=collision_free,
        n_candidates=n_candidates,
        n_legal=n_legal,
    )
