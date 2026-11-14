"""Dynamic single-track bicycle with pure-lateral Pacejka Fy and independent Fx."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from backend.app.schemas.control import ControlCommand
from backend.app.vehicle.tire_longitudinal import (
    DEFAULT_LONGITUDINAL,
    LongitudinalSlipParams,
    kappa_cmd_from_pedals,
    longitudinal_force_n,
)
from backend.app.vehicle.tire_pacejka import (
    DEFAULT_PACEJKA_LATERAL,
    PacejkaLateralParams,
    lateral_force_n,
)

_G = 9.81


@dataclass
class PacejkaBicycleParams:
    mass_kg: float = 1500.0
    iz_kgm2: float = 2500.0
    lf_m: float = 1.2
    lr_m: float = 1.6
    max_steer_rad: float = 0.6
    vx_min_mps: float = 0.5
    drive_front_frac: float = 0.6
    low_speed_damp: float = 8.0
    lateral: PacejkaLateralParams = field(default_factory=lambda: DEFAULT_PACEJKA_LATERAL)
    longitudinal: LongitudinalSlipParams = field(
        default_factory=lambda: DEFAULT_LONGITUDINAL
    )


@dataclass
class PacejkaBicycleState:
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
    wheel_states: dict[str, Any] | None = None


class PacejkaBicycleCore:
    """Planar 6-state integration; axle forces only (no combined slip)."""

    def __init__(self, params: PacejkaBicycleParams | None = None) -> None:
        self.params = params or PacejkaBicycleParams()
        self.state = PacejkaBicycleState()
        self.mu: float = 1.0
        self._cmd = ControlCommand(
            steering_cmd=0.0,
            throttle_cmd=0.0,
            brake_cmd=0.0,
            source="init",
        )
        self.last_error: str | None = None

    @property
    def wheelbase_m(self) -> float:
        return self.params.lf_m + self.params.lr_m

    def set_control(self, cmd: ControlCommand) -> None:
        self._cmd = cmd

    def set_mu(self, mu: float) -> None:
        self.mu = float(mu)

    def reset(self, initial_condition: dict) -> None:
        pose = initial_condition.get("pose", {})
        self.state = PacejkaBicycleState(
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
            wheel_states=None,
        )
        self._cmd = ControlCommand(
            steering_cmd=0.0,
            throttle_cmd=0.0,
            brake_cmd=0.0,
            source="reset",
        )
        self.last_error = None

    def step(self, dt_s: float) -> bool:
        """Advance one Euler step. Returns False if update rejected (non-finite)."""
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        p = self.params
        s = self.state
        m = p.mass_kg
        L = p.lf_m + p.lr_m
        fz_f = m * _G * p.lr_m / L
        fz_r = m * _G * p.lf_m / L
        delta = max(-p.max_steer_rad, min(p.max_steer_rad, float(self._cmd.steering_cmd)))
        kappa = kappa_cmd_from_pedals(
            throttle=self._cmd.throttle_cmd,
            brake=self._cmd.brake_cmd,
            params=p.longitudinal,
        )
        mu = self.mu

        fx_f, fx_r = self._axle_longitudinal(kappa, fz_f, fz_r, mu)

        if abs(s.vx_mps) < p.vx_min_mps:
            # Low-speed damping: avoid singular slip angles.
            damp = p.low_speed_damp
            ax = (fx_f + fx_r) / m
            s.vx_mps = max(0.0, s.vx_mps + ax * dt_s)
            s.vy_mps *= math.exp(-damp * dt_s)
            s.yaw_rate_radps *= math.exp(-damp * dt_s)
            s.ax_mps2 = ax
            s.ay_mps2 = 0.0
            s.yaw_rad += s.yaw_rate_radps * dt_s
            c, sn = math.cos(s.yaw_rad), math.sin(s.yaw_rad)
            s.x_m += (s.vx_mps * c - s.vy_mps * sn) * dt_s
            s.y_m += (s.vx_mps * sn + s.vy_mps * c) * dt_s
            s.sim_time_ns += int(round(dt_s * 1e9))
            s.wheel_states = {
                "front": {
                    "Fx_N": fx_f,
                    "Fy_N": 0.0,
                    "Fz_N": fz_f,
                    "alpha_rad": 0.0,
                    "kappa": kappa,
                },
                "rear": {
                    "Fx_N": fx_r,
                    "Fy_N": 0.0,
                    "Fz_N": fz_r,
                    "alpha_rad": 0.0,
                    "kappa": kappa,
                },
            }
            return self._finite_ok()

        # ISO slip angles (match tire_pacejka: +α → restoring −Fy).
        # Prior δ − atan form had the opposite α sign and drove unstable reverse turns.
        alpha_f = math.atan2(s.vy_mps + p.lf_m * s.yaw_rate_radps, s.vx_mps) - delta
        alpha_r = math.atan2(s.vy_mps - p.lr_m * s.yaw_rate_radps, s.vx_mps)
        fy_f = lateral_force_n(
            alpha_rad=alpha_f, fz_n=fz_f, mu=mu, params=p.lateral
        )
        fy_r = lateral_force_n(
            alpha_rad=alpha_r, fz_n=fz_r, mu=mu, params=p.lateral
        )

        cos_d, sin_d = math.cos(delta), math.sin(delta)
        fx_f_b = fx_f * cos_d - fy_f * sin_d
        fy_f_b = fx_f * sin_d + fy_f * cos_d
        fx_sum = fx_f_b + fx_r
        fy_sum = fy_f_b + fy_r
        mz = p.lf_m * fy_f_b - p.lr_m * fy_r

        ax = fx_sum / m + s.vy_mps * s.yaw_rate_radps
        ay = fy_sum / m - s.vx_mps * s.yaw_rate_radps
        r_dot = mz / p.iz_kgm2

        new_vx = s.vx_mps + ax * dt_s
        new_vy = s.vy_mps + ay * dt_s
        new_r = s.yaw_rate_radps + r_dot * dt_s
        new_yaw = s.yaw_rad + new_r * dt_s
        c, sn = math.cos(new_yaw), math.sin(new_yaw)
        new_x = s.x_m + (new_vx * c - new_vy * sn) * dt_s
        new_y = s.y_m + (new_vx * sn + new_vy * c) * dt_s

        vals = (new_vx, new_vy, new_r, new_yaw, new_x, new_y, ax, ay)
        if not all(math.isfinite(v) for v in vals):
            self.last_error = "non-finite state after Pacejka step"
            return False

        s.vx_mps = new_vx
        s.vy_mps = new_vy
        s.yaw_rate_radps = new_r
        s.yaw_rad = new_yaw
        s.x_m = new_x
        s.y_m = new_y
        s.ax_mps2 = ax
        s.ay_mps2 = ay
        s.sim_time_ns += int(round(dt_s * 1e9))
        s.wheel_states = {
            "front": {
                "Fx_N": fx_f,
                "Fy_N": fy_f,
                "Fz_N": fz_f,
                "alpha_rad": alpha_f,
                "kappa": kappa,
            },
            "rear": {
                "Fx_N": fx_r,
                "Fy_N": fy_r,
                "Fz_N": fz_r,
                "alpha_rad": alpha_r,
                "kappa": kappa,
            },
        }
        self.last_error = None
        return True

    def _axle_longitudinal(
        self, kappa: float, fz_f: float, fz_r: float, mu: float
    ) -> tuple[float, float]:
        p = self.params
        if kappa >= 0.0:
            fx_f = p.drive_front_frac * longitudinal_force_n(
                kappa=kappa, fz_n=fz_f, mu=mu, params=p.longitudinal
            )
            fx_r = (1.0 - p.drive_front_frac) * longitudinal_force_n(
                kappa=kappa, fz_n=fz_r, mu=mu, params=p.longitudinal
            )
        else:
            fx_f = longitudinal_force_n(
                kappa=kappa, fz_n=fz_f, mu=mu, params=p.longitudinal
            )
            fx_r = longitudinal_force_n(
                kappa=kappa, fz_n=fz_r, mu=mu, params=p.longitudinal
            )
        return fx_f, fx_r

    def _finite_ok(self) -> bool:
        s = self.state
        vals = (
            s.x_m,
            s.y_m,
            s.yaw_rad,
            s.vx_mps,
            s.vy_mps,
            s.yaw_rate_radps,
            s.ax_mps2,
            s.ay_mps2,
        )
        if not all(math.isfinite(v) for v in vals):
            self.last_error = "non-finite state after Pacejka step"
            return False
        self.last_error = None
        return True
