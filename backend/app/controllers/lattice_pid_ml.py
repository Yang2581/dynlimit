"""Multi-lane Lattice PID: lane-center lattice + Pure Pursuit."""

from __future__ import annotations

from typing import Any

from backend.app.controllers.lane_pp import _lanes_from_observation
from backend.app.controllers.pure_pursuit import PurePursuitController
from backend.app.planning.multi_lane_lattice import (
    MultiLaneLatticeConfig,
    MultiLaneLatticePlanner,
)
from backend.app.scenario.lanes import build_lane_geometries
from backend.app.schemas.control import ControlCommand
from backend.app.schemas.observation import Observation
from backend.app.schemas.track import TrackDefinition
from backend.app.schemas.vehicle import VehicleState

PP_KEYS = frozenset(
    {
        "lookahead_m",
        "wheelbase_m",
        "throttle_cmd",
        "max_steer_rad",
        "target_speed_mps",
    }
)


class LatticePidMlController:
    def __init__(self, **kwargs: Any) -> None:
        self._pp = PurePursuitController(**{k: v for k, v in kwargs.items() if k in PP_KEYS})
        self._cfg = MultiLaneLatticeConfig()
        self._planner: MultiLaneLatticePlanner | None = None
        self._target_lane_index: int | None = None
        self.last_diagnostics: dict[str, Any] = {}
        for key in (
            "horizon_m",
            "lc_length_m",
            "arrive_lat_m",
            "arrive_yaw_rad",
            "ds_m",
            "arrive_s_frac",
            "ego_radius_m",
            "npc_radius_m",
            "predict_dt_s",
            "w_change",
            "v_min_mps",
        ):
            if key in kwargs:
                setattr(self._cfg, key, type(getattr(self._cfg, key))(kwargs[key]))
        if "replan_every_n" in kwargs:
            self._cfg.replan_every_n = int(kwargs["replan_every_n"])
        if kwargs.get("target_lane_index") is not None:
            self._target_lane_index = int(kwargs["target_lane_index"])

    def set_waypoints(
        self, centerline_m: list[list[float]], *, closed_loop: bool | None = None
    ) -> None:
        self._pp.set_waypoints(centerline_m, closed_loop=closed_loop)

    def configure(self, config: dict[str, Any]) -> None:
        lp = config.get("lane_planner") if isinstance(config.get("lane_planner"), dict) else {}
        merged = {**config, **(lp or {})}
        for key in (
            "horizon_m",
            "lc_length_m",
            "arrive_lat_m",
            "arrive_yaw_rad",
            "ds_m",
            "arrive_s_frac",
            "ego_radius_m",
            "npc_radius_m",
            "predict_dt_s",
            "w_change",
            "v_min_mps",
        ):
            if key in merged:
                setattr(self._cfg, key, type(getattr(self._cfg, key))(merged[key]))
        if "replan_every_n" in merged:
            self._cfg.replan_every_n = int(merged["replan_every_n"])
        if merged.get("target_lane_index") is not None:
            self._target_lane_index = int(merged["target_lane_index"])
        self._planner = None

    def reset(self, config: dict[str, Any], initial_state: VehicleState) -> None:
        self.configure(config)
        self._pp.reset(config, initial_state)

    def _ensure_planner(self, observation: Observation) -> MultiLaneLatticePlanner:
        if self._planner is not None:
            return self._planner
        lanes = _lanes_from_observation(observation)
        closed = bool(observation.extras.get("closed_loop", True))
        global_ref = observation.extras.get("centerline_m") or observation.extras.get(
            "reference_line_m"
        )
        if not lanes and observation.extras.get("track"):
            try:
                track = TrackDefinition.model_validate(observation.extras["track"])
                lanes = build_lane_geometries(track)
                closed = bool(track.closed_loop)
            except Exception:
                lanes = []
        self._planner = MultiLaneLatticePlanner(
            lanes,
            closed_loop=closed,
            config=self._cfg,
            global_ref_m=global_ref,
        )
        if self._target_lane_index is not None:
            self._planner.set_target_lane(self._target_lane_index)
        return self._planner

    def compute_control(
        self, state: VehicleState, observation: Observation
    ) -> tuple[ControlCommand, dict[str, Any]]:
        planner = self._ensure_planner(observation)
        ext = observation.extras.get("target_lane_index", self._target_lane_index)
        try:
            ext_i = int(ext) if ext is not None else None
        except (TypeError, ValueError):
            ext_i = None
        obstacles = observation.extras.get("obstacles") or []
        if not isinstance(obstacles, list):
            obstacles = []

        plan = planner.step(
            float(state.x_m),
            float(state.y_m),
            float(state.yaw_rad),
            target_lane_index=ext_i,
            ego_v=float(state.vx_mps or 0.0),
            npcs=observation.npcs or [],
            obstacles=obstacles,
        )
        extras = dict(observation.extras or {})
        extras["centerline_m"] = plan.local_path_xy
        extras["closed_loop"] = False
        extras["lane_planner"] = {
            "active_lane": plan.active_lane,
            "target_queue": plan.target_queue,
            "lc_phase": plan.lc_phase,
            "planner_mode": plan.planner_mode,
            "chosen_lane": plan.chosen_lane,
        }
        obs2 = observation.model_copy(update={"extras": extras})
        self._pp.set_waypoints(plan.local_path_xy, closed_loop=False)
        cmd, diag = self._pp.compute_control(state, obs2)
        diag = {
            **diag,
            "controller": "lattice_pid_ml",
            "active_lane": plan.active_lane,
            "target_queue": list(plan.target_queue),
            "lc_phase": plan.lc_phase,
            "planner_mode": plan.planner_mode,
            "chosen_lane": plan.chosen_lane,
            "collision_free": plan.collision_free,
            "n_candidates": plan.n_candidates,
            **(plan.diagnostics or {}),
        }
        self.last_diagnostics = diag
        return cmd, diag
