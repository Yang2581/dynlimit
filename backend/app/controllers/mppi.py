"""MPPI controller — sampling MPPI with pluggable CPU/CUDA rollout backend."""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from backend.app.compute.rollout import RolloutParams, create_rollout_backend
from backend.app.schemas.control import ControlCommand
from backend.app.schemas.observation import Observation
from backend.app.schemas.vehicle import VehicleState


class MppiController:
    """Sampling-based MPPI on a kinematic bicycle; follows observation centerline."""

    def __init__(
        self,
        *,
        horizon: int = 25,
        num_samples: int = 64,
        dt_s: float = 0.05,
        wheelbase_m: float = 2.8,
        seed: int = 0,
        temperature: float = 1.0,
        steer_std: float = 0.25,
        throttle_std: float = 0.15,
        max_steer_rad: float = 0.45,
        throttle_nominal: float = 0.35,
        w_lat: float = 4.0,
        w_yaw: float = 1.5,
        w_steer: float = 0.05,
        w_throttle: float = 0.02,
        top_k_viz: int = 4,
        target_speed_mps: float = 16.0,
        rollout_backend: str = "auto",
    ) -> None:
        self.horizon = int(horizon)
        self.num_samples = int(num_samples)
        self.dt_s = float(dt_s)
        self.wheelbase_m = float(wheelbase_m)
        self.seed = int(seed)
        self.temperature = max(float(temperature), 1e-6)
        self.steer_std = float(steer_std)
        self.throttle_std = float(throttle_std)
        self.max_steer_rad = float(max_steer_rad)
        self.throttle_nominal = float(throttle_nominal)
        self.w_lat = float(w_lat)
        self.w_yaw = float(w_yaw)
        self.w_steer = float(w_steer)
        self.w_throttle = float(w_throttle)
        self.top_k_viz = max(1, int(top_k_viz))
        self.target_speed_mps = float(target_speed_mps)
        self.rollout_backend_name = rollout_backend
        self._backend = create_rollout_backend(rollout_backend)
        self._rng = np.random.default_rng(self.seed)
        self._u_mean = np.zeros((self.horizon, 2), dtype=np.float64)
        self._u_mean[:, 1] = self.throttle_nominal
        self._path = np.zeros((0, 2), dtype=np.float64)
        self.last_diagnostics: dict[str, Any] = {}

    def _params(self) -> RolloutParams:
        return RolloutParams(
            dt_s=self.dt_s,
            wheelbase_m=self.wheelbase_m,
            max_steer_rad=self.max_steer_rad,
            target_speed_mps=self.target_speed_mps,
            w_lat=self.w_lat,
            w_yaw=self.w_yaw,
            w_steer=self.w_steer,
            w_throttle=self.w_throttle,
            throttle_nominal=self.throttle_nominal,
        )

    def set_waypoints(self, centerline_m: list[list[float]]) -> None:
        pts = [(float(p[0]), float(p[1])) for p in centerline_m if len(p) >= 2]
        if len(pts) >= 2 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-6:
            pts = pts[:-1]
        self._path = np.asarray(pts, dtype=np.float64) if pts else np.zeros((0, 2))

    def reset(self, config: dict[str, Any], initial_state: VehicleState) -> None:
        del initial_state
        mapping = {
            "horizon": ("horizon", int),
            "num_samples": ("num_samples", int),
            "dt_s": ("dt_s", float),
            "seed": ("seed", int),
            "temperature": ("temperature", float),
            "steer_std": ("steer_std", float),
            "throttle_std": ("throttle_std", float),
            "max_steer_rad": ("max_steer_rad", float),
            "throttle_cmd": ("throttle_nominal", float),
            "throttle_nominal": ("throttle_nominal", float),
            "w_lat": ("w_lat", float),
            "w_yaw": ("w_yaw", float),
            "top_k_viz": ("top_k_viz", int),
            "target_speed_mps": ("target_speed_mps", float),
            "wheelbase_m": ("wheelbase_m", float),
            "rollout_backend": ("rollout_backend_name", str),
        }
        for key, (attr, cast) in mapping.items():
            if key in config:
                setattr(self, attr, cast(config[key]))
        self._backend = create_rollout_backend(self.rollout_backend_name)
        self._rng = np.random.default_rng(self.seed)
        self._u_mean = np.zeros((self.horizon, 2), dtype=np.float64)
        self._u_mean[:, 1] = self.throttle_nominal
        if "centerline_m" in config:
            self.set_waypoints(config["centerline_m"])

    def compute_control(
        self, state: VehicleState, observation: Observation
    ) -> tuple[ControlCommand, dict[str, Any]]:
        t0 = time.perf_counter()
        if self._path.shape[0] < 2 and observation.extras.get("centerline_m"):
            self.set_waypoints(observation.extras["centerline_m"])

        if self._path.shape[0] < 2:
            cmd = ControlCommand(
                steering_cmd=0.0,
                throttle_cmd=self.throttle_nominal,
                brake_cmd=0.0,
                source="MppiController",
            )
            self.last_diagnostics = {"controller": "mppi", "ok": False, "reason": "no_path"}
            return cmd, self.last_diagnostics

        noise = np.zeros((self.num_samples, self.horizon, 2), dtype=np.float64)
        noise[:, :, 0] = self._rng.normal(0.0, self.steer_std, size=(self.num_samples, self.horizon))
        noise[:, :, 1] = self._rng.normal(
            0.0, self.throttle_std, size=(self.num_samples, self.horizon)
        )
        U = self._u_mean[None, :, :] + noise
        U[:, :, 0] = np.clip(U[:, :, 0], -self.max_steer_rad, self.max_steer_rad)
        U[:, :, 1] = np.clip(U[:, :, 1], 0.0, 1.0)

        trajs, costs = self._backend.rollout_and_cost(
            x0=state.x_m,
            y0=state.y_m,
            yaw0=state.yaw_rad,
            v0=float(state.vx_mps),
            U=U,
            path_xy=self._path,
            params=self._params(),
        )
        c_min = float(np.min(costs))
        weights = np.exp(-(costs - c_min) / self.temperature)
        weights /= float(np.sum(weights) + 1e-12)
        u_star = np.sum(weights[:, None, None] * U, axis=0)
        self._u_mean = np.vstack([u_star[1:], u_star[-1:]])

        steer = float(np.clip(u_star[0, 0], -self.max_steer_rad, self.max_steer_rad))
        thr = float(np.clip(u_star[0, 1], 0.0, 1.0))
        brake = 0.0
        lon_mode = str(observation.extras.get("longitudinal_mode") or "control")
        if lon_mode == "plant":
            thr = 0.0
            brake = 0.0
        elif state.vx_mps > self.target_speed_mps + 2.0:
            thr = 0.0
            brake = 0.25

        order = np.argsort(costs)
        best_i = int(order[0])
        best_xy = trajs[best_i].tolist()
        samples = [trajs[int(i)].tolist() for i in order[: self.top_k_viz]]

        solve_ms = (time.perf_counter() - t0) * 1000.0
        cmd = ControlCommand(
            steering_cmd=steer,
            throttle_cmd=thr,
            brake_cmd=brake,
            source="MppiController",
        )
        self.last_diagnostics = {
            "controller": "mppi",
            "ok": True,
            "seed": self.seed,
            "num_samples": self.num_samples,
            "horizon": self.horizon,
            "solve_ms": round(solve_ms, 3),
            "best_cost": float(costs[best_i]),
            "best_traj_xy": best_xy,
            "sample_trajs_xy": samples,
            "steering_cmd": steer,
            "throttle_cmd": thr,
            "rollout_backend": self._backend.name,
            "longitudinal_mode": lon_mode,
            "plant_speed_mps": observation.extras.get("plant_speed_mps"),
        }
        return cmd, self.last_diagnostics
