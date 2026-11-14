"""Scenario configuration."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field, field_validator


class Pose3D(BaseModel):
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    yaw_rad: float = 0.0


class EgoConfig(BaseModel):
    vehicle_config_id: str = "mock_bicycle"
    initial_pose: Pose3D = Field(default_factory=Pose3D)
    initial_speed_mps: float = 10.0


class NpcConfig(BaseModel):
    """NPC with pluggable strategy (default: kinematic trajectory follow).

    Path source for ``trajectory`` strategy:
    - ``trajectory_xy`` if provided (>= 2 points), else
    - assigned ``lane_index`` centerline, else road spine.
    """

    id: str
    model: Literal["trajectory"] = "trajectory"  # legacy alias; prefer ``strategy``
    strategy: Literal["trajectory", "idm"] = "trajectory"
    trajectory_xy: list[list[float]] = Field(default_factory=list)
    speed_mps: float = 5.0
    loop: bool = True
    length_m: float = 4.0
    width_m: float = 2.0
    # Optional lane assignment (0 = rightmost); used when trajectory_xy empty / for IDM later
    lane_index: int | None = None
    strategy_params: dict[str, Any] = Field(default_factory=dict)

    @field_validator("trajectory_xy")
    @classmethod
    def _traj_ok(cls, pts: list[list[float]]) -> list[list[float]]:
        out: list[list[float]] = []
        for i, p in enumerate(pts):
            if len(p) < 2:
                raise ValueError(f"trajectory point {i} must be [x, y]")
            out.append([float(p[0]), float(p[1])])
        return out

    @field_validator("speed_mps")
    @classmethod
    def _speed_ok(cls, v: float) -> float:
        if v < 0:
            raise ValueError("speed_mps must be >= 0")
        return v

    @field_validator("lane_index")
    @classmethod
    def _lane_ok(cls, v: int | None) -> int | None:
        if v is None:
            return None
        if v < 0 or v > 7:
            raise ValueError("lane_index must be in 0..7 or null")
        return int(v)


class ObstacleConfig(BaseModel):
    id: str
    x_m: float
    y_m: float
    yaw_rad: float = 0.0
    length_m: float = 1.0
    width_m: float = 1.0

    @field_validator("length_m", "width_m")
    @classmethod
    def _size_ok(cls, v: float) -> float:
        if v <= 0:
            raise ValueError("obstacle size must be > 0")
        return v


class ControllerConfig(BaseModel):
    type: Literal[
        "pure_pursuit",
        "lane_pp",
        "lattice_pid",
        "lattice_pid_ml",
        "stanley",
        "constant",
        "mppi",
        "strategy01",  # deprecated alias of lattice_pid
    ] = "pure_pursuit"
    params: dict[str, Any] = Field(default_factory=dict)


class TerminationConfig(BaseModel):
    max_sim_time_s: float = 10.0
    stop_on_collision: bool = False
    stop_on_off_track: bool = False
    # null → enable automatically when track.closed_loop is False
    stop_on_goal: bool | None = None
    goal_s_frac: float = Field(default=0.98, gt=0.0, le=1.0)
    goal_radius_m: float = Field(default=8.0, gt=0.0)


class ScenarioConfig(BaseModel):
    schema_version: str = "1.0"
    scenario_id: str
    track_id: str
    ego: EgoConfig = Field(default_factory=EgoConfig)
    npcs: list[NpcConfig] = Field(default_factory=list)
    obstacles: list[ObstacleConfig] = Field(default_factory=list)
    controller: ControllerConfig = Field(default_factory=ControllerConfig)
    termination: TerminationConfig = Field(default_factory=TerminationConfig)
