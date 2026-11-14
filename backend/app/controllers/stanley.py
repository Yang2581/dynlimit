"""Stanley lateral tracker + shared LongitudinalPid (paper A1)."""

from __future__ import annotations

import math
import time
from typing import Any

from backend.app.controllers.longitudinal_pid import LongitudinalPid
from backend.app.controllers.path_geometry import dist, project_on_polyline
from backend.app.schemas.control import ControlCommand
from backend.app.schemas.observation import Observation
from backend.app.schemas.vehicle import VehicleState


class StanleyController:
    """Classical Stanley steering with the same speed PID as A0 Pure Pursuit."""

    def __init__(
        self,
        *,
        k_stanley: float = 2.5,
        wheelbase_m: float = 2.8,
        max_steer_rad: float = 0.45,
        target_speed_mps: float = 16.0,
        soft_v_mps: float = 1.0,
        lon_kp: float = 0.25,
        lon_ki: float = 0.05,
        lon_kd: float = 0.0,
        throttle_ff: float = 0.15,
    ) -> None:
        self.k_stanley = float(k_stanley)
        self.wheelbase_m = float(wheelbase_m)
        self.max_steer_rad = float(max_steer_rad)
        self.target_speed_mps = float(target_speed_mps)
        self.soft_v_mps = float(soft_v_mps)
        self._waypoints: list[tuple[float, float]] = []
        self._seg_hint = 0
        self._lon = LongitudinalPid(
            kp=lon_kp, ki=lon_ki, kd=lon_kd, throttle_ff=throttle_ff
        )
        self.last_diagnostics: dict[str, Any] = {}

    def set_waypoints(self, centerline_m: list[list[float]]) -> None:
        pts = [(float(p[0]), float(p[1])) for p in centerline_m if len(p) >= 2]
        if len(pts) >= 2 and dist(pts[0][0], pts[0][1], pts[-1][0], pts[-1][1]) < 1e-6:
            pts = pts[:-1]
        self._waypoints = pts
        self._seg_hint = 0

    def reset(self, config: dict[str, Any], initial_state: VehicleState) -> None:
        del initial_state
        if "k_stanley" in config:
            self.k_stanley = float(config["k_stanley"])
        if "target_speed_mps" in config:
            self.target_speed_mps = float(config["target_speed_mps"])
        if "centerline_m" in config:
            self.set_waypoints(config["centerline_m"])
        self._lon.reset()

    def compute_control(
        self, state: VehicleState, observation: Observation
    ) -> tuple[ControlCommand, dict[str, Any]]:
        t0 = time.perf_counter()
        if not self._waypoints and observation.extras.get("centerline_m"):
            self.set_waypoints(observation.extras["centerline_m"])

        steer = 0.0
        crosstrack = 0.0
        heading_err = 0.0
        if len(self._waypoints) >= 2:
            proj = project_on_polyline(
                state.x_m, state.y_m, self._waypoints, hint_idx=self._seg_hint
            )
            self._seg_hint = proj.seg_index
            crosstrack = proj.crosstrack_m
            heading_err = proj.path_yaw_rad - state.yaw_rad
            heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))
            v = abs(float(state.vx_mps or 0.0))
            steer = heading_err + math.atan2(
                self.k_stanley * crosstrack, v + self.soft_v_mps
            )
            steer = max(-self.max_steer_rad, min(self.max_steer_rad, steer))

        lon_mode = str(observation.extras.get("longitudinal_mode") or "control")
        throttle, brake = self._lon.step(
            speed_mps=float(state.vx_mps or 0.0),
            target_speed_mps=self.target_speed_mps,
            lon_mode=lon_mode,
        )

        cmd = ControlCommand(
            steering_cmd=steer,
            throttle_cmd=throttle,
            brake_cmd=brake,
            source="StanleyController",
        )
        solve_ms = (time.perf_counter() - t0) * 1000.0
        diagnostics = {
            "controller": "stanley",
            "steer_rad": steer,
            "crosstrack_m": crosstrack,
            "heading_err_rad": heading_err,
            "longitudinal_mode": lon_mode,
            "solve_ms": solve_ms,
            "n_waypoints": len(self._waypoints),
        }
        self.last_diagnostics = diagnostics
        return cmd, diagnostics
