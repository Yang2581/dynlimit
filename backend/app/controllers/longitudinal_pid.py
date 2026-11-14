"""Shared longitudinal speed PID for A0/A1 fairness (paper protocol)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LongitudinalPid:
    """Maps speed error → throttle/brake in [0, 1]."""

    kp: float = 0.25
    ki: float = 0.05
    kd: float = 0.0
    throttle_ff: float = 0.15
    max_throttle: float = 1.0
    max_brake: float = 1.0
    integral_limit: float = 2.0
    brake_deadband_mps: float = 0.3

    def __post_init__(self) -> None:
        self._integral = 0.0
        self._prev_err: float | None = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_err = None

    def step(
        self,
        *,
        speed_mps: float,
        target_speed_mps: float,
        dt_s: float = 0.02,
        lon_mode: str = "control",
    ) -> tuple[float, float]:
        if lon_mode == "plant":
            return 0.0, 0.0

        err = float(target_speed_mps) - float(speed_mps)
        dt = max(1e-3, float(dt_s))
        self._integral = max(
            -self.integral_limit,
            min(self.integral_limit, self._integral + err * dt),
        )
        deriv = 0.0
        if self._prev_err is not None:
            deriv = (err - self._prev_err) / dt
        self._prev_err = err

        u = self.throttle_ff + self.kp * err + self.ki * self._integral + self.kd * deriv
        throttle = 0.0
        brake = 0.0
        if u >= 0.0:
            throttle = max(0.0, min(self.max_throttle, u))
        else:
            # Negative effort → brake; ignore tiny undershoot
            if err < -self.brake_deadband_mps:
                brake = max(0.0, min(self.max_brake, -u))
            throttle = 0.0
        return throttle, brake
