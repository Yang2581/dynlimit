"""Pluggable NPC motion strategies (trajectory now; IDM / others later)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from backend.app.scenario.lanes import LaneGeometry
from backend.app.schemas.scenario import NpcConfig
from backend.app.schemas.track import TrackDefinition
from backend.app.schemas.vehicle import VehicleState


@dataclass
class NpcPose:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    vx_mps: float = 0.0
    s_m: float = 0.0
    lane_index: int | None = None


@dataclass
class NpcStepContext:
    """World snapshot passed to strategies each physics step."""

    dt_s: float
    track: TrackDefinition
    lanes: list[LaneGeometry]
    ego: VehicleState | None = None
    others: list[NpcPose] = field(default_factory=list)
    time_s: float = 0.0


@runtime_checkable
class NpcStrategy(Protocol):
    """Strategy interface — implement ``reset`` + ``step`` for new behaviors (e.g. IDM)."""

    name: str

    def reset(
        self,
        config: NpcConfig,
        track: TrackDefinition,
        lanes: list[LaneGeometry],
    ) -> NpcPose:
        """Initialize internal state; return initial pose."""
        ...

    def step(self, pose: NpcPose, ctx: NpcStepContext) -> NpcPose:
        """Advance one physics step; return updated pose."""
        ...


def _path_cum(pts: list[tuple[float, float]]) -> tuple[list[float], float]:
    cum = [0.0]
    for i in range(len(pts) - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        cum.append(cum[-1] + math.hypot(dx, dy))
    return cum, (cum[-1] if cum else 0.0)


def _place_on_path(
    pts: list[tuple[float, float]],
    cum: list[float],
    total: float,
    s: float,
    *,
    loop: bool,
) -> tuple[float, float, float, float]:
    """Return x, y, yaw, clamped_s."""
    if total <= 1e-9 or len(pts) < 2:
        x, y = pts[0] if pts else (0.0, 0.0)
        return x, y, 0.0, 0.0
    if loop:
        s = s % total
    else:
        s = max(0.0, min(total, s))
    i = 0
    for j in range(len(cum) - 1):
        if cum[j + 1] >= s:
            i = j
            break
        i = j
    s0, s1 = cum[i], cum[i + 1]
    L = max(s1 - s0, 1e-12)
    t = (s - s0) / L
    x0, y0 = pts[i]
    x1, y1 = pts[i + 1]
    return x0 + t * (x1 - x0), y0 + t * (y1 - y0), math.atan2(y1 - y0, x1 - x0), s


def _project_xy_to_s(
    x: float,
    y: float,
    pts: list[tuple[float, float]],
    cum: list[float],
) -> float:
    """Nearest arc-length s on polyline to (x, y)."""
    if len(pts) < 2 or len(cum) < 2:
        return 0.0
    best_s = 0.0
    best_d2 = float("inf")
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        dx = x1 - x0
        dy = y1 - y0
        L2 = dx * dx + dy * dy
        if L2 < 1e-18:
            continue
        t = ((x - x0) * dx + (y - y0) * dy) / L2
        t = max(0.0, min(1.0, t))
        px = x0 + t * dx
        py = y0 + t * dy
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_s = cum[i] + t * math.sqrt(L2)
    return best_s


def _initial_s_from_config(
    config: NpcConfig,
    pts: list[tuple[float, float]],
    cum: list[float],
) -> float:
    """Edit placement marker: a single trajectory_xy point projects onto the path.

    Multi-point trajectories / empty markers still start at s=0.
    """
    if config.trajectory_xy and len(config.trajectory_xy) == 1:
        p = config.trajectory_xy[0]
        return _project_xy_to_s(float(p[0]), float(p[1]), pts, cum)
    return 0.0


def resolve_npc_path_xy(
    config: NpcConfig,
    track: TrackDefinition,
    lanes: list[LaneGeometry],
) -> list[tuple[float, float]]:
    """Path for trajectory-style strategies: explicit polyline or assigned lane centerline."""
    if config.trajectory_xy and len(config.trajectory_xy) >= 2:
        return [(float(p[0]), float(p[1])) for p in config.trajectory_xy]
    if config.lane_index is not None:
        if config.lane_index < 0 or config.lane_index >= len(lanes):
            raise ValueError(
                f"NPC {config.id}: lane_index={config.lane_index} out of range 0..{len(lanes) - 1}"
            )
        lane = lanes[config.lane_index]
        pts = [(float(p[0]), float(p[1])) for p in lane.centerline_m]
        # Oncoming lanes: travel opposite the spine sense.
        if not lane.travel_forward:
            pts = list(reversed(pts))
        return pts
    # fallback: road spine
    return [(p[0], p[1]) for p in track.centerline_m]


class TrajectoryFollowStrategy:
    """Constant-speed kinematic follow along a polyline (current default)."""

    name = "trajectory"

    def __init__(self) -> None:
        self._config: NpcConfig | None = None
        self._pts: list[tuple[float, float]] = []
        self._cum: list[float] = []
        self._total: float = 0.0
        self._lane_index: int | None = None

    def reset(
        self,
        config: NpcConfig,
        track: TrackDefinition,
        lanes: list[LaneGeometry],
    ) -> NpcPose:
        self._config = config
        self._lane_index = config.lane_index
        self._pts = resolve_npc_path_xy(config, track, lanes)
        self._cum, self._total = _path_cum(self._pts)
        s0 = _initial_s_from_config(config, self._pts, self._cum)
        x, y, yaw, s = _place_on_path(
            self._pts, self._cum, self._total, s0, loop=bool(config.loop)
        )
        return NpcPose(
            x_m=x,
            y_m=y,
            yaw_rad=yaw,
            vx_mps=float(config.speed_mps),
            s_m=s,
            lane_index=self._lane_index,
        )

    def step(self, pose: NpcPose, ctx: NpcStepContext) -> NpcPose:
        assert self._config is not None
        speed = float(self._config.speed_mps)
        x, y, yaw, s = _place_on_path(
            self._pts,
            self._cum,
            self._total,
            pose.s_m + speed * ctx.dt_s,
            loop=bool(self._config.loop),
        )
        return NpcPose(
            x_m=x,
            y_m=y,
            yaw_rad=yaw,
            vx_mps=speed,
            s_m=s,
            lane_index=self._lane_index,
        )


class IdmStrategy:
    """Placeholder for Intelligent Driver Model (not implemented yet).

    Reserved so configs can declare ``strategy: "idm"`` and lane assignment early.
    """

    name = "idm"

    def reset(
        self,
        config: NpcConfig,
        track: TrackDefinition,
        lanes: list[LaneGeometry],
    ) -> NpcPose:
        raise NotImplementedError(
            "IDM strategy is not implemented yet; use strategy='trajectory' "
            f"(NPC id={config.id}). Lane geometries are available via ctx.lanes."
        )

    def step(self, pose: NpcPose, ctx: NpcStepContext) -> NpcPose:
        raise NotImplementedError("IDM strategy is not implemented yet")


_STRATEGY_FACTORIES: dict[str, type] = {
    "trajectory": TrajectoryFollowStrategy,
    "idm": IdmStrategy,
}


def create_npc_strategy(name: str, params: dict[str, Any] | None = None) -> NpcStrategy:
    """Factory for NPC strategies. ``params`` reserved for IDM gains etc."""
    key = (name or "trajectory").lower().strip()
    cls = _STRATEGY_FACTORIES.get(key)
    if cls is None:
        raise ValueError(f"unknown NPC strategy '{name}'; known={list(_STRATEGY_FACTORIES)}")
    # params unused for trajectory; IDM will consume later
    _ = params
    return cls()  # type: ignore[return-value]


def list_npc_strategies() -> list[str]:
    return sorted(_STRATEGY_FACTORIES.keys())
