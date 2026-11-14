"""Actuation limiting / guard."""

from __future__ import annotations

from backend.app.schemas.control import ActuationLimits, ControlCommand


class ActuationGuard:
    def __init__(self, limits: ActuationLimits | None = None) -> None:
        self.limits = limits or ActuationLimits()
        self._last_steering: float | None = None
        self._last_time_ns: int | None = None

    def reset(self) -> None:
        self._last_steering = None
        self._last_time_ns = None

    def validate_and_limit(
        self, cmd: ControlCommand, sim_time_ns: int
    ) -> tuple[ControlCommand, dict]:
        limits = self.limits
        steering = max(
            -limits.max_steering_rad,
            min(limits.max_steering_rad, cmd.steering_cmd),
        )
        clamped_steer_mag = steering != cmd.steering_cmd

        if self._last_steering is not None and self._last_time_ns is not None:
            dt_s = max((sim_time_ns - self._last_time_ns) * 1e-9, 1e-9)
            max_delta = limits.max_steering_rate_radps * dt_s
            delta = steering - self._last_steering
            if abs(delta) > max_delta:
                steering = self._last_steering + max_delta * (1.0 if delta > 0 else -1.0)
                clamped_steer_mag = True

        throttle = max(0.0, min(limits.max_throttle, cmd.throttle_cmd))
        brake = max(0.0, min(limits.max_brake, cmd.brake_cmd))

        self._last_steering = steering
        self._last_time_ns = sim_time_ns

        out = cmd.model_copy(
            update={
                "steering_cmd": steering,
                "throttle_cmd": throttle,
                "brake_cmd": brake,
            }
        )
        diagnostics = {
            "clamped_steering": clamped_steer_mag,
            "clamped_throttle": throttle != cmd.throttle_cmd,
            "clamped_brake": brake != cmd.brake_cmd,
        }
        return out, diagnostics
