"""Planar kinematic bicycle core (compatibility / light CI)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from backend.app.schemas.control import ControlCommand


@dataclass
class KinematicBicycleState:
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    yaw_rad: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    yaw_rate_radps: float = 0.0
    ax_mps2: float = 0.0
    ay_mps2: float = 0.0
    sim_time_ns: int = 0


class KinematicBicycleCore:
    """Integrate pose from longitudinal accel map and kinematic yaw rate."""

    def __init__(
        self,
        *,
        wheelbase_m: float = 2.8,
        max_accel_mps2: float = 3.0,
        max_decel_mps2: float = 6.0,
        max_speed_mps: float = 25.0,
    ) -> None:
        self.wheelbase_m = float(wheelbase_m)
        self.max_accel_mps2 = float(max_accel_mps2)
        self.max_decel_mps2 = float(max_decel_mps2)
        self.max_speed_mps = float(max_speed_mps)
        self.state = KinematicBicycleState()
        self._cmd = ControlCommand(
            steering_cmd=0.0,
            throttle_cmd=0.0,
            brake_cmd=0.0,
            source="init",
        )

    def set_control(self, cmd: ControlCommand) -> None:
        self._cmd = cmd

    def reset(self, initial_condition: dict) -> None:
        pose = initial_condition.get("pose", {})
        self.state = KinematicBicycleState(
            x_m=float(pose.get("x_m", 0.0)),
            y_m=float(pose.get("y_m", 0.0)),
            z_m=float(pose.get("z_m", 0.0)),
            yaw_rad=float(pose.get("yaw_rad", 0.0)),
            vx_mps=float(initial_condition.get("speed_mps", 0.0)),
            vy_mps=0.0,
            yaw_rate_radps=0.0,
            ax_mps2=0.0,
            ay_mps2=0.0,
            sim_time_ns=int(initial_condition.get("sim_time_ns", 0)),
        )
        self._cmd = ControlCommand(
            steering_cmd=0.0,
            throttle_cmd=0.0,
            brake_cmd=0.0,
            source="reset",
        )

    def step(self, dt_s: float) -> None:
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        s = self.state
        accel = (
            self.max_accel_mps2 * self._cmd.throttle_cmd
            - self.max_decel_mps2 * self._cmd.brake_cmd
        )
        s.ax_mps2 = accel
        s.vx_mps = max(0.0, min(self.max_speed_mps, s.vx_mps + accel * dt_s))
        steer = self._cmd.steering_cmd
        if abs(s.vx_mps) < 1e-6:
            s.yaw_rate_radps = 0.0
        else:
            s.yaw_rate_radps = s.vx_mps * math.tan(steer) / self.wheelbase_m
        s.ay_mps2 = s.vx_mps * s.yaw_rate_radps
        s.yaw_rad += s.yaw_rate_radps * dt_s
        s.x_m += s.vx_mps * math.cos(s.yaw_rad) * dt_s
        s.y_m += s.vx_mps * math.sin(s.yaw_rad) * dt_s
        s.sim_time_ns += int(round(dt_s * 1e9))
