"""Pure Pursuit driven by LaneLocalPlanner local reference."""

from __future__ import annotations

from typing import Any

from backend.app.controllers.pure_pursuit import PurePursuitController
from backend.app.planning.lane_local_planner import LaneLocalConfig, LaneLocalPlanner
from backend.app.scenario.lanes import LaneGeometry, build_lane_geometries
from backend.app.schemas.control import ControlCommand
from backend.app.schemas.observation import Observation
from backend.app.schemas.track import TrackDefinition
from backend.app.schemas.vehicle import VehicleState


def _lanes_from_observation(observation: Observation) -> list[LaneGeometry]:
    raw = observation.extras.get("lanes") or []
    out: list[LaneGeometry] = []
    for i, L in enumerate(raw):
        if isinstance(L, LaneGeometry):
            out.append(L)
            continue
        if not isinstance(L, dict):
            continue
        out.append(
            LaneGeometry(
                index=int(L.get("index", i)),
                id=str(L.get("id", f"lane_{i}")),
                side=str(L.get("side", "inner")),
                lateral_offset_m=float(L.get("lateral_offset_m", 0.0)),
                width_m=float(L.get("width_m", 3.5)),
                centerline_m=list(L.get("centerline_m") or []),
                left_boundary_m=list(L.get("left_boundary_m") or []),
                right_boundary_m=list(L.get("right_boundary_m") or []),
                length_m=float(L.get("length_m", 0.0)),
                travel_forward=bool(L.get("travel_forward", True)),
            )
        )
    return out


class LanePurePursuitController:
    """Wraps PurePursuit with a lane-local FOLLOW/LC planner."""

    def __init__(self, **pp_kwargs: Any) -> None:
        self._pp = PurePursuitController(**pp_kwargs)
        self._planner: LaneLocalPlanner | None = None
        self._lane_cfg = LaneLocalConfig()
        self._target_lane_index: int | None = None
        self.last_diagnostics: dict[str, Any] = {}

    def set_waypoints(
        self, centerline_m: list[list[float]], *, closed_loop: bool | None = None
    ) -> None:
        self._pp.set_waypoints(centerline_m, closed_loop=closed_loop)

    def configure_lane_planner(self, config: dict[str, Any]) -> None:
        """Apply scenario controller.params (and nested lane_planner) before first step."""
        lp = config.get("lane_planner") if isinstance(config.get("lane_planner"), dict) else {}
        merged = {**config, **(lp or {})}
        for key, attr, cast in (
            ("horizon_m", "horizon_m", float),
            ("lc_length_m", "lc_length_m", float),
            ("arrive_lat_m", "arrive_lat_m", float),
            ("arrive_yaw_rad", "arrive_yaw_rad", float),
            ("ds_m", "ds_m", float),
            ("arrive_s_frac", "arrive_s_frac", float),
        ):
            if key in merged:
                setattr(self._lane_cfg, attr, cast(merged[key]))
        if "target_lane_index" in merged and merged["target_lane_index"] is not None:
            self._target_lane_index = int(merged["target_lane_index"])
        self._planner = None

    def reset(self, config: dict[str, Any], initial_state: VehicleState) -> None:
        self.configure_lane_planner(config)
        self._pp.reset(config, initial_state)

    def _ensure_planner(self, observation: Observation) -> LaneLocalPlanner:
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
        self._planner = LaneLocalPlanner(
            lanes,
            closed_loop=closed,
            config=self._lane_cfg,
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
        if ext is not None:
            try:
                ext_i = int(ext)
            except (TypeError, ValueError):
                ext_i = None
        else:
            ext_i = None
        suggest = observation.extras.get("suggest_adjacent_lane")
        suggest_i = int(suggest) if suggest is not None else None

        plan = planner.step(
            float(state.x_m),
            float(state.y_m),
            float(state.yaw_rad),
            target_lane_index=ext_i,
            suggest_adjacent=suggest_i,
        )

        # Feed local path into PP as open polyline
        extras = dict(observation.extras or {})
        extras["centerline_m"] = plan.local_path_xy
        extras["closed_loop"] = False
        extras["lane_planner"] = {
            "active_lane": plan.active_lane,
            "target_queue": plan.target_queue,
            "lc_phase": plan.lc_phase,
            "planner_mode": plan.planner_mode,
            "local_path_xy": plan.local_path_xy[:: max(1, len(plan.local_path_xy) // 40)],
        }
        obs2 = observation.model_copy(update={"extras": extras})
        self._pp.set_waypoints(plan.local_path_xy, closed_loop=False)
        cmd, diag = self._pp.compute_control(state, obs2)
        diag = {
            **diag,
            "controller": "lane_pp",
            "active_lane": plan.active_lane,
            "target_queue": list(plan.target_queue),
            "lc_phase": plan.lc_phase,
            "planner_mode": plan.planner_mode,
            **(plan.diagnostics or {}),
        }
        self.last_diagnostics = diag
        return cmd, diag
