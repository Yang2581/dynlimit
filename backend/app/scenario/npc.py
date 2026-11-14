"""NPC agent wrapping a pluggable motion strategy."""

from __future__ import annotations

from dataclasses import dataclass, field

from backend.app.scenario.lanes import LaneGeometry
from backend.app.scenario.npc_strategy import (
    NpcPose,
    NpcStepContext,
    NpcStrategy,
    create_npc_strategy,
)
from backend.app.schemas.scenario import NpcConfig
from backend.app.schemas.track import TrackDefinition
from backend.app.schemas.vehicle import VehicleState


def _stub_track_for_npc(config: NpcConfig) -> TrackDefinition:
    """Minimal track so strategies can reset without a full scenario (unit tests)."""
    pts = config.trajectory_xy
    if len(pts) >= 3:
        cl = [[float(p[0]), float(p[1]), 0.0] for p in pts]
    elif len(pts) == 2:
        x0, y0 = float(pts[0][0]), float(pts[0][1])
        x1, y1 = float(pts[1][0]), float(pts[1][1])
        cl = [[x0, y0, 0.0], [(x0 + x1) * 0.5, (y0 + y1) * 0.5, 0.0], [x1, y1, 0.0]]
    else:
        cl = [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 0.0, 0.0]]
    n_lanes = 1 if config.lane_index is None else max(1, int(config.lane_index) + 1)
    return TrackDefinition(
        track_id="_npc_stub",
        closed_loop=False,
        width_m=max(4.0, 3.5 * n_lanes),
        num_lanes=n_lanes,
        lane_width_m=3.5,
        centerline_m=cl,
    )


@dataclass
class TrajectoryNpc:
    """Runtime NPC. Name kept for compatibility; motion comes from ``strategy``."""

    config: NpcConfig
    track: TrackDefinition | None = None
    lanes: list[LaneGeometry] = field(default_factory=list)
    strategy: NpcStrategy | None = None
    pose: NpcPose = field(default_factory=NpcPose)
    s_m: float = 0.0
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.track is None:
            self.track = _stub_track_for_npc(self.config)
        if not self.lanes:
            self.lanes = self.track.lanes()
        if self.strategy is None:
            self.strategy = create_npc_strategy(
                self.config.strategy, self.config.strategy_params
            )
        self._apply_pose(self.strategy.reset(self.config, self.track, self.lanes))

    def _apply_pose(self, pose: NpcPose) -> None:
        self.pose = pose
        self.x_m = pose.x_m
        self.y_m = pose.y_m
        self.yaw_rad = pose.yaw_rad
        self.s_m = pose.s_m

    def bind_world(self, track: TrackDefinition, lanes: list[LaneGeometry] | None = None) -> None:
        """Attach scenario track/lanes and reset strategy."""
        self.track = track
        self.lanes = lanes if lanes is not None else track.lanes()
        self.strategy = create_npc_strategy(self.config.strategy, self.config.strategy_params)
        self._apply_pose(self.strategy.reset(self.config, self.track, self.lanes))

    def advance(
        self,
        dt_s: float,
        *,
        ego: VehicleState | None = None,
        others: list[NpcPose] | None = None,
        time_s: float = 0.0,
    ) -> None:
        assert self.strategy is not None and self.track is not None
        ctx = NpcStepContext(
            dt_s=dt_s,
            track=self.track,
            lanes=self.lanes,
            ego=ego,
            others=others or [],
            time_s=time_s,
        )
        self._apply_pose(self.strategy.step(self.pose, ctx))

    def as_dict(self) -> dict:
        return {
            "id": self.config.id,
            "x_m": self.x_m,
            "y_m": self.y_m,
            "yaw_rad": self.yaw_rad,
            "vx_mps": self.pose.vx_mps,
            "length_m": self.config.length_m,
            "width_m": self.config.width_m,
            "strategy": self.config.strategy,
            "lane_index": self.pose.lane_index
            if self.pose.lane_index is not None
            else self.config.lane_index,
            "s_m": self.s_m,
        }
