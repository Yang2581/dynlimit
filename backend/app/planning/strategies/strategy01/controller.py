"""Lattice PID controller: lattice / AES-swerve plan then Pure Pursuit tracking.

Public name: ``lattice_pid``. Deprecated alias: ``strategy01``.
"""

from __future__ import annotations

from typing import Any

from backend.app.controllers.pure_pursuit import PurePursuitController
from backend.app.planning.strategies.strategy01.aes_avoid import swerve_path_from_obstacle
from backend.app.planning.strategies.strategy01.lattice import LatticeParams, plan_lattice
from backend.app.schemas.observation import Observation
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

LATTICE_KEYS = frozenset(
    {
        "lateral_offsets_m",
        "d_m",
        "horizon_m",
        "ds_m",
        "ego_radius_m",
        "npc_radius_m",
        "predict_dt_s",
        "w_lat",
        "w_change",
        "v_min_mps",
        "road_half_width_m",
    }
)


def _as_xy_list(centerline_m: Any) -> list[list[float]]:
    pts: list[list[float]] = []
    for p in centerline_m or []:
        if len(p) >= 2:
            pts.append([float(p[0]), float(p[1])])
    return pts


def _lattice_from_kwargs(kwargs: dict[str, Any]) -> LatticeParams:
    return LatticeParams(**{k: kwargs[k] for k in LATTICE_KEYS if k in kwargs})


class LatticePidController:
    """Lateral lattice on a single reference + Pure Pursuit (lon PID)."""

    def __init__(
        self,
        replan_every_n: int = 1,
        avoidance_mode: str = "lattice",
        **kwargs: Any,
    ) -> None:
        self._pp = PurePursuitController(**{k: v for k, v in kwargs.items() if k in PP_KEYS})
        self._params = _lattice_from_kwargs(kwargs)
        self._replan_every_n = max(1, int(replan_every_n))
        self._avoidance_mode = str(avoidance_mode or "lattice")
        self._tick = 0
        self._last_offset = 0.0
        self._last_collision_free = True
        self._last_n_candidates = 0
        self._centerline: list[list[float]] = []
        self._local: list[list[float]] = []
        self._aes_planned = False

    def set_waypoints(self, centerline_m: list[list[float]]) -> None:
        self._centerline = _as_xy_list(centerline_m)
        self._pp.set_waypoints(centerline_m)

    def reset(self, config: dict[str, Any], initial_state: VehicleState) -> None:
        cfg = dict(config or {})
        if "replan_every_n" in cfg:
            self._replan_every_n = max(1, int(cfg["replan_every_n"]))
        if "avoidance_mode" in cfg:
            self._avoidance_mode = str(cfg["avoidance_mode"])
        lattice_kw = {k: cfg[k] for k in LATTICE_KEYS if k in cfg}
        if lattice_kw:
            merged = {k: getattr(self._params, k) for k in LATTICE_KEYS}
            if "d_m" in lattice_kw and "lateral_offsets_m" not in lattice_kw:
                merged["lateral_offsets_m"] = []
            merged.update(lattice_kw)
            self._params = LatticeParams(**merged)
        pp_cfg = {k: cfg[k] for k in PP_KEYS if k in cfg}
        if "centerline_m" in cfg:
            self._centerline = _as_xy_list(cfg["centerline_m"])
            pp_cfg["centerline_m"] = cfg["centerline_m"]
        self._pp.reset(pp_cfg, initial_state)
        self._tick = 0
        self._last_offset = 0.0
        self._last_collision_free = True
        self._last_n_candidates = 0
        self._local = []
        self._aes_planned = False

    def compute_control(
        self, state: VehicleState, observation: Observation
    ) -> tuple[Any, dict[str, Any]]:
        try:
            extras = observation.extras or {}
            cl = self._centerline or _as_xy_list(extras.get("centerline_m") or [])
            if len(cl) < 2:
                cmd, d = self._pp.compute_control(state, observation)
                d.update({"planner_error": "no_centerline", "controller": "lattice_pid"})
                return cmd, d
            # Prefer road spine for online avoidance (not AES tracking reference).
            road_cl = _as_xy_list(extras.get("road_centerline_m") or []) or cl
            obstacles = extras.get("obstacles") or []
            if not isinstance(obstacles, list):
                obstacles = []

            if self._avoidance_mode == "aes_swerve":
                plan_diag = self._plan_aes_swerve(road_cl, obstacles)
            elif self._tick % self._replan_every_n == 0:
                result = plan_lattice(
                    ego_x=state.x_m,
                    ego_y=state.y_m,
                    ego_yaw=state.yaw_rad,
                    ego_v=state.vx_mps,
                    centerline_xy=road_cl,
                    npcs=observation.npcs or [],
                    obstacles=list(obstacles),
                    params=self._params,
                    last_offset_m=self._last_offset,
                )
                self._last_offset = result.offset_m
                self._last_collision_free = result.collision_free
                self._last_n_candidates = result.n_candidates
                self._local = result.path_xy
                if len(result.path_xy) >= 2:
                    self._pp.set_waypoints(result.path_xy)
                plan_diag = self._plan_diag(reused=False)
            else:
                plan_diag = self._plan_diag(reused=True)
            self._tick += 1
            cmd, d = self._pp.compute_control(state, observation)
            d.update(plan_diag)
            d["controller"] = "lattice_pid"
            d["avoidance_mode"] = self._avoidance_mode
            return cmd, d
        except Exception as exc:
            cmd, d = self._pp.compute_control(state, observation)
            d["planner_error"] = str(exc)
            d["controller"] = "lattice_pid"
            return cmd, d

    def _plan_aes_swerve(
        self, road_cl: list[list[float]], obstacles: list[Any]
    ) -> dict[str, Any]:
        """One-shot swerve from first static obstacle (cheap CarSim-safe path)."""
        if self._aes_planned and len(self._local) >= 2:
            return self._plan_diag(reused=True)
        if not obstacles:
            self._local = road_cl
            self._pp.set_waypoints(road_cl)
            self._last_offset = 0.0
            self._last_collision_free = True
            self._last_n_candidates = 1
            self._aes_planned = True
            return self._plan_diag(reused=False)
        obs0 = obstacles[0] if isinstance(obstacles[0], dict) else {}
        half = (
            float(self._params.road_half_width_m)
            if self._params.road_half_width_m is not None
            else 6.0
        )
        path, peak = swerve_path_from_obstacle(
            road_cl,
            obs0,
            road_half_width_m=half,
        )
        self._local = path
        self._last_offset = peak
        self._last_collision_free = True
        self._last_n_candidates = 1
        self._aes_planned = True
        if len(path) >= 2:
            self._pp.set_waypoints(path)
        return self._plan_diag(reused=False)

    def _plan_diag(self, *, reused: bool) -> dict[str, Any]:
        # Omit local_path_xy from hot-path diagnostics (large; MCAP/log bloat).
        diag: dict[str, Any] = {
            "chosen_offset_m": self._last_offset,
            "collision_free": self._last_collision_free,
            "n_candidates": self._last_n_candidates,
            "n_path_pts": len(self._local),
        }
        if reused:
            diag["reused_path"] = True
        return diag


# Deprecated alias — keep old imports / registry keys working.
Strategy01Controller = LatticePidController
