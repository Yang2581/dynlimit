"""Constant open-loop controller."""

from __future__ import annotations

from typing import Any

from backend.app.schemas.control import ControlCommand
from backend.app.schemas.observation import Observation
from backend.app.schemas.vehicle import VehicleState


class ConstantController:
    def __init__(
        self,
        steering_cmd: float = 0.0,
        throttle_cmd: float = 0.2,
        brake_cmd: float = 0.0,
    ) -> None:
        self.steering_cmd = steering_cmd
        self.throttle_cmd = throttle_cmd
        self.brake_cmd = brake_cmd

    def reset(self, config: dict[str, Any], initial_state: VehicleState) -> None:
        del initial_state
        if "steering_cmd" in config:
            self.steering_cmd = float(config["steering_cmd"])
        if "throttle_cmd" in config:
            self.throttle_cmd = float(config["throttle_cmd"])
        if "brake_cmd" in config:
            self.brake_cmd = float(config["brake_cmd"])

    def compute_control(
        self, state: VehicleState, observation: Observation
    ) -> tuple[ControlCommand, dict[str, Any]]:
        del observation
        cmd = ControlCommand(
            steering_cmd=self.steering_cmd,
            throttle_cmd=self.throttle_cmd,
            brake_cmd=self.brake_cmd,
            source="ConstantController",
            valid_until_ns=None,
        )
        diagnostics = {
            "controller": "ConstantController",
            "sim_time_ns": state.sim_time_ns,
        }
        return cmd, diagnostics
