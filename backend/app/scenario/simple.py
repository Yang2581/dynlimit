"""Track + scenario loader with trajectory NPCs and geometric events."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from backend.app.scenario.geometry import Box2D, boxes_overlap
from backend.app.scenario.lanes import LaneGeometry, build_lane_geometries
from backend.app.scenario.npc import TrajectoryNpc
from backend.app.scenario.track_query import TrackQuery
from backend.app.schemas.observation import Observation
from backend.app.schemas.scenario import ScenarioConfig
from backend.app.schemas.track import TrackDefinition
from backend.app.schemas.vehicle import VehicleState

EGO_LENGTH_M = 4.4
EGO_WIDTH_M = 2.0
# Downsample ego states for paper goal evaluators (~20 Hz).
_GOAL_SAMPLE_DT_S = 0.05
_WHEEL_IDS = ("L1", "R1", "L2", "R2")


def _tire_fields_from_wheel_states(wheel_states: dict[str, Any] | None) -> dict[str, float]:
    """Flatten CarSim keys or MOCK nested axle forces into Fx_*/Fy_*/Fz_*."""
    if not wheel_states:
        return {}
    out: dict[str, float] = {}
    for wid in _WHEEL_IDS:
        for prefix in ("Fx", "Fy", "Fz"):
            key = f"{prefix}_{wid}"
            if key in wheel_states and wheel_states[key] is not None:
                out[key] = float(wheel_states[key])
    if out:
        return out
    front = wheel_states.get("front") or wheel_states.get("F")
    rear = wheel_states.get("rear") or wheel_states.get("R")
    if isinstance(front, dict) and isinstance(rear, dict):
        for side, src in (("L1", front), ("R1", front), ("L2", rear), ("R2", rear)):
            if "Fx_N" in src:
                out[f"Fx_{side}"] = float(src["Fx_N"]) / 2.0
            if "Fy_N" in src:
                out[f"Fy_{side}"] = float(src["Fy_N"]) / 2.0
            if "Fz_N" in src:
                out[f"Fz_{side}"] = float(src["Fz_N"]) / 2.0
    return out


class SimpleScenario:
    def __init__(self) -> None:
        self._track: TrackDefinition | None = None
        self._config: ScenarioConfig | None = None
        self._query: TrackQuery | None = None
        self._ref_query: TrackQuery | None = None
        self._lanes: list[LaneGeometry] = []
        self._npcs: list[TrajectoryNpc] = []
        self._lat_samples: list[float] = []
        self._path_length_m: float = 0.0
        self._prev_xy: tuple[float, float] | None = None
        self._max_vx: float = 0.0
        self._collision: bool = False
        self._off_track: bool = False
        self._goal_reached: bool = False
        self._finished_reason: str | None = None
        self._time_s: float = 0.0
        self._goal_samples: list[dict[str, float]] = []
        self._last_goal_sample_t: float = -1e9

    def load_files(self, track_path: str | Path, scenario_path: str | Path) -> None:
        track = TrackDefinition.model_validate_json(Path(track_path).read_text(encoding="utf-8"))
        config = ScenarioConfig.model_validate_json(Path(scenario_path).read_text(encoding="utf-8"))
        self.initialize({"track": track.model_dump(), "scenario": config.model_dump()})

    def initialize(self, config: dict[str, Any]) -> None:
        if "track" in config and "scenario" in config:
            self._track = TrackDefinition.model_validate(config["track"])
            self._config = ScenarioConfig.model_validate(config["scenario"])
            if self._config.track_id != self._track.track_id:
                raise ValueError(
                    f"scenario.track_id={self._config.track_id} != track.track_id={self._track.track_id}"
                )
            self._query = TrackQuery(self._track)
            self._ref_query = TrackQuery(
                self._track, polyline=self._track.driving_reference_m()
            )
            self._lanes = build_lane_geometries(self._track)
            self._npcs = []
            for n in self._config.npcs:
                if n.lane_index is not None and n.lane_index >= len(self._lanes):
                    raise ValueError(
                        f"NPC {n.id}: lane_index={n.lane_index} but track has {len(self._lanes)} lanes"
                    )
                npc = TrajectoryNpc(n, track=self._track, lanes=self._lanes)
                self._npcs.append(npc)
            self._lat_samples = []
            self._path_length_m = 0.0
            self._prev_xy = None
            self._max_vx = 0.0
            self._collision = False
            self._off_track = False
            self._goal_reached = False
            self._finished_reason = None
            self._time_s = 0.0
            self._goal_samples = []
            self._last_goal_sample_t = -1e9
            return
        raise ValueError("SimpleScenario.initialize expects keys 'track' and 'scenario'")

    def observation(self, t_ns: int) -> Observation:
        self._require_ready()
        assert self._config is not None
        assert self._track is not None
        return Observation(
            sim_time_ns=t_ns,
            track_id=self._config.track_id,
            ego_ref=None,
            npcs=[n.as_dict() for n in self._npcs],
            extras={
                "centerline_m": self._track.driving_reference_m(),
                "road_centerline_m": self._track.centerline_m,
                "reference_line_m": self._track.driving_reference_m(),
                "closed_loop": bool(self._track.closed_loop),
                "lanes": [L.as_dict() for L in self._lanes],
                "obstacles": [o.model_dump() for o in self._config.obstacles],
                "friction_mu_default": float(self._track.friction_mu),
                "friction_zones": [z.model_dump() for z in (self._track.friction_zones or [])],
            },
        )

    def centerline_m(self) -> list[list[float]]:
        """Waypoints for Pure Pursuit = independent driving reference."""
        self._require_ready()
        assert self._track is not None
        return self._track.driving_reference_m()

    def lanes(self) -> list[LaneGeometry]:
        return list(self._lanes)

    def advance_npcs(self, dt_s: float, ego: VehicleState | None = None) -> None:
        self._time_s += dt_s
        poses = [n.pose for n in self._npcs]
        for i, n in enumerate(self._npcs):
            others = poses[:i] + poses[i + 1 :]
            n.advance(dt_s, ego=ego, others=others, time_s=self._time_s)

    def check_events(self, ego: VehicleState) -> list[dict[str, Any]]:
        """Geometric checks; also updates running metrics samples."""
        self._require_ready()
        assert self._config is not None
        assert self._query is not None
        events: list[dict[str, Any]] = []

        # metrics
        if self._prev_xy is not None:
            dx = ego.x_m - self._prev_xy[0]
            dy = ego.y_m - self._prev_xy[1]
            self._path_length_m += (dx * dx + dy * dy) ** 0.5
        self._prev_xy = (ego.x_m, ego.y_m)
        self._max_vx = max(self._max_vx, abs(ego.vx_mps))
        proj = self._query.project(ego.x_m, ego.y_m)
        lat = proj.lat_m
        self._lat_samples.append(abs(lat))
        t_s = float(ego.sim_time_ns) * 1e-9
        if (t_s - self._last_goal_sample_t) >= _GOAL_SAMPLE_DT_S - 1e-12:
            sample: dict[str, Any] = {
                "t_s": t_s,
                "x_m": float(ego.x_m),
                "y_m": float(ego.y_m),
                "vx_mps": float(ego.vx_mps),
                "vy_mps": float(ego.vy_mps),
                "yaw_rate_radps": float(ego.yaw_rate_radps),
                "s_m": float(proj.s_m),
                "track_length_m": float(self._query.length_m),
            }
            # Optional tire forces for U_tire (CarSim flat keys or MOCK nested).
            ws = getattr(ego, "wheel_states", None)
            if isinstance(ws, dict) and ws:
                sample.update(_tire_fields_from_wheel_states(ws))
            self._goal_samples.append(sample)
            self._last_goal_sample_t = t_s

        half = self._track.width_m * 0.5 if self._track else 5.0
        if abs(lat) > half and not self._off_track:
            self._off_track = True
            events.append(
                {
                    "type": "off_track",
                    "message": f"|lat|={abs(lat):.2f}m > half_width={half:.2f}m",
                    "payload": {"lat_m": lat},
                }
            )

        goal_ev = self._maybe_goal_event(ego)
        if goal_ev is not None:
            events.append(goal_ev)

        ego_box = Box2D(ego.x_m, ego.y_m, ego.yaw_rad, EGO_LENGTH_M, EGO_WIDTH_M)
        if not self._collision:
            for n in self._npcs:
                nb = Box2D(n.x_m, n.y_m, n.yaw_rad, n.config.length_m, n.config.width_m)
                if boxes_overlap(ego_box, nb):
                    self._collision = True
                    events.append(
                        {
                            "type": "collision",
                            "message": f"collision with NPC {n.config.id}",
                            "payload": {"other_id": n.config.id, "kind": "npc"},
                        }
                    )
                    break
        if not self._collision:
            for o in self._config.obstacles:
                ob = Box2D(o.x_m, o.y_m, o.yaw_rad, o.length_m, o.width_m)
                if boxes_overlap(ego_box, ob):
                    self._collision = True
                    events.append(
                        {
                            "type": "collision",
                            "message": f"collision with obstacle {o.id}",
                            "payload": {"other_id": o.id, "kind": "obstacle"},
                        }
                    )
                    break
        return events

    def npc_states(self) -> list[dict[str, Any]]:
        return [n.as_dict() for n in self._npcs]

    def obstacles(self) -> list[dict[str, Any]]:
        self._require_ready()
        assert self._config is not None
        return [o.model_dump() for o in self._config.obstacles]

    def metrics_snapshot(self, sim_time_ns: int, reason: str | None = None) -> dict[str, Any]:
        mean_lat = (
            sum(self._lat_samples) / len(self._lat_samples) if self._lat_samples else 0.0
        )
        if reason:
            self._finished_reason = reason
        max_s = max((s["s_m"] for s in self._goal_samples), default=0.0)
        track_len = (
            float(self._goal_samples[-1]["track_length_m"]) if self._goal_samples else 0.0
        )
        return {
            "duration_s": sim_time_ns * 1e-9,
            "path_length_m": self._path_length_m,
            "max_vx": self._max_vx,
            "mean_abs_lat_m": mean_lat,
            "off_track": self._off_track,
            "collision": self._collision,
            "finished_reason": self._finished_reason or reason,
            "n_lat_samples": len(self._lat_samples),
            "max_s_m": max_s,
            "track_length_m": track_len,
            "max_s_frac": (max_s / track_len) if track_len > 1e-9 else 0.0,
            "n_goal_samples": len(self._goal_samples),
        }

    def goal_trajectory(self) -> list[dict[str, float]]:
        """Downsampled ego samples for paper goal evaluators."""
        return list(self._goal_samples)

    def max_sim_time_s(self) -> float | None:
        self._require_ready()
        assert self._config is not None
        return self._config.termination.max_sim_time_s

    def stop_on_collision(self) -> bool:
        self._require_ready()
        assert self._config is not None
        return self._config.termination.stop_on_collision

    def stop_on_off_track(self) -> bool:
        self._require_ready()
        assert self._config is not None
        return self._config.termination.stop_on_off_track

    def track_id(self) -> str:
        self._require_ready()
        assert self._config is not None
        return self._config.track_id

    def scenario_id(self) -> str:
        self._require_ready()
        assert self._config is not None
        return self._config.scenario_id

    def config(self) -> ScenarioConfig:
        self._require_ready()
        assert self._config is not None
        return self._config

    def track(self) -> TrackDefinition:
        self._require_ready()
        assert self._track is not None
        return self._track

    def ego_initial_condition(self) -> dict[str, Any]:
        self._require_ready()
        assert self._config is not None
        pose = self._config.ego.initial_pose
        return {
            "pose": pose.model_dump(),
            "speed_mps": self._config.ego.initial_speed_mps,
            "sim_time_ns": 0,
        }

    def apply_ego_ic_override(self, ego_ic: dict[str, Any] | None) -> None:
        """Merge optional run-time ego IC over loaded scenario (in-memory only)."""
        if not ego_ic:
            return
        self._require_ready()
        assert self._config is not None
        pose = self._config.ego.initial_pose
        if "x_m" in ego_ic and ego_ic["x_m"] is not None:
            pose.x_m = float(ego_ic["x_m"])
        if "y_m" in ego_ic and ego_ic["y_m"] is not None:
            pose.y_m = float(ego_ic["y_m"])
        if "z_m" in ego_ic and ego_ic["z_m"] is not None:
            pose.z_m = float(ego_ic["z_m"])
        if "yaw_rad" in ego_ic and ego_ic["yaw_rad"] is not None:
            pose.yaw_rad = float(ego_ic["yaw_rad"])
        if "speed_mps" in ego_ic and ego_ic["speed_mps"] is not None:
            self._config.ego.initial_speed_mps = float(ego_ic["speed_mps"])

    def apply_friction_mu(self, mu: float | None) -> None:
        """Override track default friction (in-memory; for pressure grids)."""
        if mu is None:
            return
        self._require_ready()
        assert self._track is not None
        self._track.friction_mu = float(mu)

    def _goal_enabled(self) -> bool:
        assert self._config is not None
        assert self._track is not None
        flag = self._config.termination.stop_on_goal
        if flag is not None:
            return bool(flag)
        return not bool(self._track.closed_loop)

    def _maybe_goal_event(self, ego: VehicleState) -> dict[str, Any] | None:
        if self._goal_reached or not self._goal_enabled():
            return None
        assert self._config is not None
        q = self._ref_query or self._query
        if q is None:
            return None
        L = float(q.length_m)
        if L <= 1e-9:
            return None
        proj = q.project(ego.x_m, ego.y_m)
        s_frac = float(proj.s_m) / L
        ex, ey = q.end_xy
        sx, sy = q.start_xy
        dist_end = math.hypot(ego.x_m - ex, ego.y_m - ey)
        dist_start = math.hypot(ego.x_m - sx, ego.y_m - sy)
        term = self._config.termination
        if s_frac < float(term.goal_s_frac):
            return None
        if dist_end > float(term.goal_radius_m):
            return None
        # Guard: open polylines that accidentally close make end≈start; ignore
        # "goal" while still near the start.
        if dist_start <= float(term.goal_radius_m) and dist_start <= dist_end + 1e-6:
            return None
        self._goal_reached = True
        return {
            "type": "finished",
            "message": (
                f"goal reached s_frac={s_frac:.3f} dist_end={dist_end:.2f}m"
            ),
            "payload": {
                "reason": "goal",
                "s_m": float(proj.s_m),
                "track_length_m": L,
                "s_frac": s_frac,
                "dist_end_m": dist_end,
            },
        }

    def _require_ready(self) -> None:
        if self._track is None or self._config is None:
            raise RuntimeError("SimpleScenario not loaded")
