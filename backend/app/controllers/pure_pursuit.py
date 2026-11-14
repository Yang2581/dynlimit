"""Geometric Pure Pursuit + shared LongitudinalPid (paper A0)."""

from __future__ import annotations

import math
import time
from typing import Any

from backend.app.controllers.longitudinal_pid import LongitudinalPid
from backend.app.controllers.path_geometry import dist
from backend.app.schemas.control import ControlCommand
from backend.app.schemas.observation import Observation
from backend.app.schemas.vehicle import VehicleState


class PurePursuitController:
    def __init__(
        self,
        lookahead_m: float = 12.0,
        wheelbase_m: float = 2.8,
        throttle_cmd: float = 0.35,
        max_steer_rad: float = 0.45,
        target_speed_mps: float = 12.0,
        lon_kp: float = 0.25,
        lon_ki: float = 0.05,
        lon_kd: float = 0.0,
        use_speed_pid: bool = True,
    ) -> None:
        self.lookahead_m = lookahead_m
        self.wheelbase_m = wheelbase_m
        self.throttle_cmd = throttle_cmd
        self.max_steer_rad = max_steer_rad
        self.target_speed_mps = target_speed_mps
        self.use_speed_pid = use_speed_pid
        self._waypoints: list[tuple[float, float]] = []
        self._s_cursor = 0
        self._closed_loop = True
        self._lon = LongitudinalPid(
            kp=lon_kp, ki=lon_ki, kd=lon_kd, throttle_ff=float(throttle_cmd) * 0.4
        )
        self.last_diagnostics: dict[str, Any] = {}

    def set_waypoints(
        self, centerline_m: list[list[float]], *, closed_loop: bool | None = None
    ) -> None:
        pts = [(float(p[0]), float(p[1])) for p in centerline_m if len(p) >= 2]
        if len(pts) >= 2 and dist(pts[0][0], pts[0][1], pts[-1][0], pts[-1][1]) < 1e-6:
            pts = pts[:-1]
            if closed_loop is None:
                closed_loop = True
        self._waypoints = pts
        self._s_cursor = 0
        if closed_loop is not None:
            self._closed_loop = bool(closed_loop)

    def reset(self, config: dict[str, Any], initial_state: VehicleState) -> None:
        del initial_state
        if "lookahead_m" in config:
            self.lookahead_m = float(config["lookahead_m"])
        if "throttle_cmd" in config:
            self.throttle_cmd = float(config["throttle_cmd"])
            self._lon.throttle_ff = float(self.throttle_cmd) * 0.4
        if "target_speed_mps" in config:
            self.target_speed_mps = float(config["target_speed_mps"])
        if "centerline_m" in config:
            self.set_waypoints(config["centerline_m"])
        self._lon.reset()

    def compute_control(
        self, state: VehicleState, observation: Observation
    ) -> tuple[ControlCommand, dict[str, Any]]:
        t0 = time.perf_counter()
        if observation.extras.get("closed_loop") is not None:
            self._closed_loop = bool(observation.extras["closed_loop"])
        if not self._waypoints and observation.extras.get("centerline_m"):
            self.set_waypoints(
                observation.extras["centerline_m"],
                closed_loop=self._closed_loop,
            )

        steer = 0.0
        target = None
        if len(self._waypoints) >= 2:
            target = self._lookahead_point(state.x_m, state.y_m)
            dx = target[0] - state.x_m
            dy = target[1] - state.y_m
            alpha = math.atan2(dy, dx) - state.yaw_rad
            alpha = math.atan2(math.sin(alpha), math.cos(alpha))
            ld = max(self.lookahead_m, 1.0)
            steer = math.atan2(2.0 * self.wheelbase_m * math.sin(alpha), ld)
            steer = max(-self.max_steer_rad, min(self.max_steer_rad, steer))

        lon_mode = str(observation.extras.get("longitudinal_mode") or "control")
        plant_speed = observation.extras.get("plant_speed_mps")

        if lon_mode == "plant":
            throttle = 0.0
            brake = 0.0
        elif self.use_speed_pid:
            throttle, brake = self._lon.step(
                speed_mps=float(state.vx_mps or 0.0),
                target_speed_mps=self.target_speed_mps,
                lon_mode=lon_mode,
            )
        else:
            # Legacy open-loop throttle (tests / demos)
            throttle = self.throttle_cmd
            brake = 0.0
            if state.vx_mps is not None:
                if state.vx_mps > self.target_speed_mps + 1.0:
                    throttle = 0.0
                    brake = 0.2
                elif state.vx_mps > self.target_speed_mps:
                    throttle = max(0.05, self.throttle_cmd * 0.3)

        cmd = ControlCommand(
            steering_cmd=steer,
            throttle_cmd=throttle,
            brake_cmd=brake,
            source="PurePursuitController",
        )
        solve_ms = (time.perf_counter() - t0) * 1000.0
        diagnostics = {
            "controller": "pure_pursuit",
            "steer_rad": steer,
            "target": target,
            "n_waypoints": len(self._waypoints),
            "longitudinal_mode": lon_mode,
            "plant_speed_mps": plant_speed,
            "solve_ms": solve_ms,
            "use_speed_pid": self.use_speed_pid,
        }
        self.last_diagnostics = diagnostics
        return cmd, diagnostics

    def _lookahead_point(self, x: float, y: float) -> tuple[float, float]:
        wps = self._waypoints
        n = len(wps)
        closed = self._closed_loop
        best_i = self._s_cursor
        best_d = float("inf")
        scan = n if closed else max(1, n - best_i)
        for k in range(scan):
            i = (self._s_cursor + k) % n if closed else min(self._s_cursor + k, n - 1)
            d = dist(x, y, wps[i][0], wps[i][1])
            if d < best_d:
                best_d = d
                best_i = i
            if k > 5 and d > best_d + self.lookahead_m:
                break
        self._s_cursor = best_i
        acc = 0.0
        i = best_i
        while acc < self.lookahead_m:
            if not closed and i >= n - 1:
                return wps[-1]
            j = (i + 1) % n if closed else i + 1
            acc += dist(wps[i][0], wps[i][1], wps[j][0], wps[j][1])
            i = j
            if closed and i == best_i and acc > 0:
                break
        return wps[i]
