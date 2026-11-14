"""Dual-mode MOCK plant: Pacejka bicycle (default) or kinematic fallback."""

from __future__ import annotations

from typing import Any, Literal

from backend.app.schemas.capabilities import VehicleCapabilities
from backend.app.schemas.control import ControlCommand
from backend.app.schemas.vehicle import VehicleState
from backend.app.vehicle.carsim_road_overlay import clamp_friction_mu
from backend.app.vehicle.kinematic_bicycle import KinematicBicycleCore
from backend.app.vehicle.pacejka_bicycle import PacejkaBicycleCore, PacejkaBicycleParams
from backend.app.vehicle.tire_longitudinal import LongitudinalSlipParams
from backend.app.vehicle.tire_pacejka import PacejkaLateralParams

MockModel = Literal["pacejka", "kinematic"]


class MockDynamicsAdapter:
    """Unified mock dynamics facade (SI units)."""

    def __init__(
        self,
        vehicle_id: str = "ego",
        wheelbase_m: float = 2.8,
        max_accel_mps2: float = 3.0,
        max_decel_mps2: float = 6.0,
        max_speed_mps: float = 25.0,
        model: MockModel = "pacejka",
    ) -> None:
        self.vehicle_id = vehicle_id
        self.model: MockModel = model
        self.wheelbase_m = wheelbase_m
        self.max_accel_mps2 = max_accel_mps2
        self.max_decel_mps2 = max_decel_mps2
        self.max_speed_mps = max_speed_mps
        self._initialized = False
        self.plant_ic_applied: bool = False
        self.plant_error: str | None = None
        self.has_longitudinal_control: bool = True
        self.longitudinal_mode: str = "control"
        self.plant_speed_mps: float | None = None
        self.plant_road_mu: float | None = None
        self._kinematic = KinematicBicycleCore(
            wheelbase_m=wheelbase_m,
            max_accel_mps2=max_accel_mps2,
            max_decel_mps2=max_decel_mps2,
            max_speed_mps=max_speed_mps,
        )
        self._pacejka = PacejkaBicycleCore(
            PacejkaBicycleParams(
                lf_m=0.45 * wheelbase_m,
                lr_m=0.55 * wheelbase_m,
            )
        )
        self._cmd = ControlCommand(
            steering_cmd=0.0,
            throttle_cmd=0.0,
            brake_cmd=0.0,
            source="init",
        )

    def initialize(self, vehicle_config: dict[str, Any], road_config: dict[str, Any]) -> None:
        cfg = vehicle_config or {}
        model = str(cfg.get("model", self.model)).lower()
        if model not in ("pacejka", "kinematic"):
            raise ValueError(f"Unknown mock model: {model}")
        self.model = model  # type: ignore[assignment]

        if "wheelbase_m" in cfg:
            self.wheelbase_m = float(cfg["wheelbase_m"])
            self._kinematic.wheelbase_m = self.wheelbase_m
            self._pacejka.params.lf_m = 0.45 * self.wheelbase_m
            self._pacejka.params.lr_m = 0.55 * self.wheelbase_m

        if self.model == "pacejka":
            p = self._pacejka.params
            if "mass_kg" in cfg:
                p.mass_kg = float(cfg["mass_kg"])
            if "iz_kgm2" in cfg:
                p.iz_kgm2 = float(cfg["iz_kgm2"])
            if "lf_m" in cfg:
                p.lf_m = float(cfg["lf_m"])
            if "lr_m" in cfg:
                p.lr_m = float(cfg["lr_m"])
            if "drive_front_frac" in cfg:
                p.drive_front_frac = float(cfg["drive_front_frac"])
            if "max_steer_rad" in cfg:
                p.max_steer_rad = float(cfg["max_steer_rad"])
            lat = cfg.get("pacejka_lateral") or {}
            if lat:
                p.lateral = PacejkaLateralParams(
                    B=float(lat.get("B", p.lateral.B)),
                    C=float(lat.get("C", p.lateral.C)),
                    D0=float(lat.get("D0", p.lateral.D0)),
                    E=float(lat.get("E", p.lateral.E)),
                )
            lon = cfg.get("longitudinal") or {}
            if lon:
                p.longitudinal = LongitudinalSlipParams(
                    B=float(lon.get("B", p.longitudinal.B)),
                    C=float(lon.get("C", p.longitudinal.C)),
                    E=float(lon.get("E", p.longitudinal.E)),
                    kappa_max_drive=float(
                        lon.get("kappa_max_drive", p.longitudinal.kappa_max_drive)
                    ),
                    kappa_max_brake=float(
                        lon.get("kappa_max_brake", p.longitudinal.kappa_max_brake)
                    ),
                )

        mu = None
        if road_config and road_config.get("friction_mu") is not None:
            mu = float(road_config["friction_mu"])
        elif self.plant_road_mu is not None:
            mu = float(self.plant_road_mu)
        if mu is not None:
            self.plant_road_mu = clamp_friction_mu(mu)
            self._pacejka.set_mu(self.plant_road_mu)
        else:
            self._pacejka.set_mu(1.0)

        self._initialized = True
        self.plant_error = None

    def set_control(self, cmd: ControlCommand) -> None:
        self._cmd = cmd
        self._kinematic.set_control(cmd)
        self._pacejka.set_control(cmd)

    def step(self, dt_s: float) -> None:
        if not self._initialized:
            raise RuntimeError("MockDynamicsAdapter not initialized")
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")

        if self.plant_road_mu is not None:
            self._pacejka.set_mu(clamp_friction_mu(self.plant_road_mu))

        if self.model == "kinematic":
            self._kinematic.step(dt_s)
            self.plant_speed_mps = abs(self._kinematic.state.vx_mps)
            return

        ok = self._pacejka.step(dt_s)
        if not ok:
            self.plant_error = self._pacejka.last_error or "pacejka step failed"
            return
        self.plant_error = None
        self.plant_speed_mps = abs(self._pacejka.state.vx_mps)

    def get_state(self) -> VehicleState:
        if self.model == "kinematic":
            s = self._kinematic.state
            return VehicleState(
                sim_time_ns=s.sim_time_ns,
                vehicle_id=self.vehicle_id,
                x_m=s.x_m,
                y_m=s.y_m,
                z_m=s.z_m,
                yaw_rad=s.yaw_rad,
                roll_rad=0.0,
                pitch_rad=0.0,
                vx_mps=s.vx_mps,
                vy_mps=s.vy_mps,
                yaw_rate_radps=s.yaw_rate_radps,
                ax_mps2=s.ax_mps2,
                ay_mps2=s.ay_mps2,
                steering_wheel_rad=None,
                wheel_states=None,
            )
        s = self._pacejka.state
        return VehicleState(
            sim_time_ns=s.sim_time_ns,
            vehicle_id=self.vehicle_id,
            x_m=s.x_m,
            y_m=s.y_m,
            z_m=s.z_m,
            yaw_rad=s.yaw_rad,
            roll_rad=0.0,
            pitch_rad=0.0,
            vx_mps=s.vx_mps,
            vy_mps=s.vy_mps,
            yaw_rate_radps=s.yaw_rate_radps,
            ax_mps2=s.ax_mps2,
            ay_mps2=s.ay_mps2,
            steering_wheel_rad=None,
            wheel_states=s.wheel_states,
        )

    def read_cached_state(self) -> VehicleState:
        return self.get_state()

    def reset(self, initial_condition: dict[str, Any]) -> None:
        self._kinematic.reset(initial_condition)
        self._pacejka.reset(initial_condition)
        self._cmd = ControlCommand(
            steering_cmd=0.0,
            throttle_cmd=0.0,
            brake_cmd=0.0,
            source="reset",
        )
        self.plant_ic_applied = True
        self.plant_error = None
        speed = float(initial_condition.get("speed_mps", 0.0))
        self.plant_speed_mps = abs(speed)

    def close(self) -> None:
        self._initialized = False
        self.plant_ic_applied = False

    def capabilities(self) -> VehicleCapabilities:
        if self.model == "pacejka":
            return VehicleCapabilities(
                has_accel=True,
                has_steering_wheel=False,
                has_wheel_states=True,
                has_tire_forces=True,
                notes=(
                    "Dynamic single-track MOCK: pure-lateral Pacejka Fy + independent "
                    "Fx(kappa_cmd); axle forces only; no combined slip / load transfer"
                ),
            )
        return VehicleCapabilities(
            has_accel=True,
            has_steering_wheel=False,
            has_wheel_states=False,
            has_tire_forces=False,
            notes="Planar kinematic bicycle mock; no tire or steering-wheel channels",
        )
