"""CarSim dynamics adapter wrapping pycarsimlib.CarsimManager."""

from __future__ import annotations

import sys
from datetime import timedelta
from pathlib import Path
from typing import Any

from backend.app.schemas.capabilities import VehicleCapabilities
from backend.app.schemas.control import ControlCommand
from backend.app.schemas.vehicle import VehicleState
from backend.app.vehicle.coordinate_transform import (
    control_to_imports,
    conversion_notes,
    exports_to_state,
    has_longitudinal_imports,
)

# Default = lateral only (matches many VS datasets). Add throttle/motor/brake
# labels via input_list / vehicle_config to enable platform longitudinal control.
DEFAULT_INPUTS = ["IMP_STEER_L1", "IMP_STEER_R1"]
# Must match CarSim GUI "I/O Channels: Export" order (index = vs_integrate_io slot).
# Current dataset ``mpc_output_data_collection`` (PORTS_EXP 1,27):
#   Vy,Vx,Yaw,AVz,Yo,Xo, Kappa_L1/R1, Fy_L1/L2/R1/R2, AAz,Ay,Ax,
#   Fx_L1/L2/R1/R2, Fz_*, Alpha_L1/L2/R1/R2
# Label names are dict keys only; do not insert/reorder without updating the VS export table.
CORE_OUTPUTS = ["Vy", "Vx", "Yaw", "AVz", "Yo", "Xo"]
DEFAULT_OUTPUTS = [
    "Vy",
    "Vx",
    "Yaw",
    "AVz",
    "Yo",
    "Xo",
    "Kappa_L1",
    "Kappa_R1",
    "Fy_L1",
    "Fy_L2",
    "Fy_R1",
    "Fy_R2",
    "AAz",
    "Ay",
    "Ax",
    "Fx_L1",
    "Fx_L2",
    "Fx_R1",
    "Fx_R2",
    "Fz_L1",
    "Fz_L2",
    "Fz_R1",
    "Fz_R2",
    "Alpha_L1",
    "Alpha_L2",
    "Alpha_R1",
    "Alpha_R2",
]


class CarSimUnavailableError(RuntimeError):
    """Raised when pycarsimlib / CarSim DLL cannot be used."""


def _ensure_pycarsimlib_on_path(extra_roots: list[Path] | None = None) -> None:
    """Prefer repo-local pycarsimlib without requiring pip install."""
    candidates: list[Path] = []
    if extra_roots:
        candidates.extend(extra_roots)
    # racing_sim/python_carsim/pycarsimlib  (package parent)
    here = Path(__file__).resolve()
    repo_root = here.parents[3]  # backend/app/vehicle -> repo
    candidates.append(repo_root / "python_carsim" / "pycarsimlib")
    for root in candidates:
        if (root / "pycarsimlib").is_dir():
            s = str(root)
            if s not in sys.path:
                sys.path.insert(0, s)
            return


class CarSimAdapter:
    """VehicleDynamicsAdapter backed by CarsimManager.

    Notes
    -----
    - Reset strategy: terminate + construct a new CarsimManager (upstream reset N/I).
    - Scenario IC: recorded on reset; ``plant_ic_applied`` is True only if
      ``_try_apply_plant_ic`` succeeds. pycarsimlib has no VS set-state API today,
      so CarSim typically reports ``plant_ic_applied=False`` (procedure IC wins).
    - Longitudinal: if input_list includes throttle/motor/brake imports →
      ``longitudinal_mode="control"``; else ``"plant"`` (lateral-only; speed from VS).
    - ``close()`` is idempotent and always nulls the manager even if terminate fails.
    """

    def __init__(
        self,
        carsim_db_dir: str | Path,
        *,
        vehicle_type: str = "normal_vehicle",
        input_list: list[str] | None = None,
        output_list: list[str] | None = None,
        vehicle_id: str = "ego",
        speed_in_kmh: bool = True,
        yaw_in_degrees: bool = True,
        yaw_rate_in_degrees: bool = True,
        steer_in_degrees: bool = True,
        motor_torque_nm: float = 200.0,
        brake_p_max_mpa: float = 10.0,
        pycarsimlib_root: str | Path | None = None,
    ) -> None:
        self.carsim_db_dir = Path(carsim_db_dir)
        self.vehicle_type = vehicle_type
        self.input_list = list(input_list or DEFAULT_INPUTS)
        self.output_list = list(output_list or DEFAULT_OUTPUTS)
        self.vehicle_id = vehicle_id
        self.speed_in_kmh = speed_in_kmh
        self.yaw_in_degrees = yaw_in_degrees
        self.yaw_rate_in_degrees = yaw_rate_in_degrees
        self.steer_in_degrees = steer_in_degrees
        self.motor_torque_nm = float(motor_torque_nm)
        self.brake_p_max_mpa = float(brake_p_max_mpa)
        self._pycarsimlib_root = Path(pycarsimlib_root) if pycarsimlib_root else None

        self._manager: Any = None
        self._cmd = ControlCommand(
            steering_cmd=0.0,
            throttle_cmd=0.0,
            brake_cmd=0.0,
            source="init",
        )
        self._cached: VehicleState | None = None
        self._sim_time_ns = 0
        self._t_step_s: float | None = None
        self._initialized = False
        self.plant_ic_applied: bool = False
        self.plant_error: str | None = None
        self._last_ic: dict[str, Any] | None = None
        self.plant_speed_mps: float | None = None
        self.plant_road_mu: float | None = None
        self.plant_road_mu_applied: bool = False
        self._road_config: dict[str, Any] = {}
        self._speed_samples = 0
        self._speed_sum = 0.0
        self._freeze_watch_x: float | None = None
        self._freeze_watch_steps = 0
        self._freeze_recovered = False

    @property
    def has_longitudinal_control(self) -> bool:
        return has_longitudinal_imports(self.input_list)

    @property
    def longitudinal_mode(self) -> str:
        """``control`` = platform throttle/torque; ``plant`` = CarSim owns speed."""
        return "control" if self.has_longitudinal_control else "plant"

    def initialize(self, vehicle_config: dict[str, Any], road_config: dict[str, Any]) -> None:
        rc = dict(self._road_config)
        if road_config:
            rc.update(dict(road_config))
        if rc:
            self._road_config = rc
            if rc.get("friction_mu") is not None:
                self.plant_road_mu = float(rc["friction_mu"])
            if "overlay_applied" in rc:
                self.plant_road_mu_applied = bool(rc["overlay_applied"])
        if "input_list" in vehicle_config:
            self.input_list = list(vehicle_config["input_list"])
        if "output_list" in vehicle_config:
            self.output_list = list(vehicle_config["output_list"])
        if "motor_torque_nm" in vehicle_config:
            self.motor_torque_nm = float(vehicle_config["motor_torque_nm"])
        if "brake_p_max_mpa" in vehicle_config:
            self.brake_p_max_mpa = float(vehicle_config["brake_p_max_mpa"])
        self.plant_error = None
        self._freeze_recovered = False
        self._reset_speed_tracker()
        self._open_manager()
        self._initialized = True
        # Prime exports with zero steer
        self.set_control(
            ControlCommand(
                steering_cmd=0.0,
                throttle_cmd=0.0,
                brake_cmd=0.0,
                source="initialize",
            )
        )
        self.step(self._t_step_s or 0.001)
        # initialize alone does not apply scenario IC (Engine.load calls reset next)
        if self._last_ic is None:
            self.plant_ic_applied = False

    def set_control(self, cmd: ControlCommand) -> None:
        if not self.has_longitudinal_control:
            # Lateral-only: never push fake throttle into unused channels
            cmd = cmd.model_copy(update={"throttle_cmd": 0.0, "brake_cmd": 0.0})
        self._cmd = cmd

    def step(self, dt_s: float) -> None:
        self._require_manager()
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        assert self._t_step_s is not None
        # CarsimManager loops integer t_step slices; warn on non-multiple via floor behavior
        action = control_to_imports(
            self._cmd,
            self.input_list,
            steer_in_degrees=self.steer_in_degrees,
            motor_torque_nm=self.motor_torque_nm,
            brake_p_max_mpa=self.brake_p_max_mpa,
        )
        observed, _terminated, updated_time_sec = self._manager.step(
            action=action,
            delta_time=timedelta(seconds=dt_s),
        )
        self._sim_time_ns = int(round(float(updated_time_sec) * 1e9))
        self._cached = exports_to_state(
            dict(observed),
            sim_time_ns=self._sim_time_ns,
            vehicle_id=self.vehicle_id,
            speed_in_kmh=self.speed_in_kmh,
            yaw_in_degrees=self.yaw_in_degrees,
            yaw_rate_in_degrees=self.yaw_rate_in_degrees,
        )
        self._update_plant_speed(self._cached)
        self._maybe_recover_frozen_plant()

    def get_state(self) -> VehicleState:
        if self._cached is None:
            raise RuntimeError("CarSimAdapter has no state; call initialize/step first")
        return self._cached

    def read_cached_state(self) -> VehicleState:
        return self.get_state()

    def reset(self, initial_condition: dict[str, Any]) -> None:
        """Record scenario IC; reopen solver only when no live manager.

        ``Engine.load`` calls ``initialize`` then ``reset``. Re-opening here used
        to double ``LoadLibrary``/``vs_initialize`` on every Create/Reset and made
        the next Start feel much slower. Session Rebuild / ``Engine.reset`` must
        ``close()`` first so this path opens once.
        """
        self._last_ic = dict(initial_condition) if initial_condition else {}
        if not (self._initialized and self._manager is not None):
            was_init = self._initialized
            self.close()
            if was_init:
                self.initialize({}, {})
        self._freeze_recovered = False
        self._reset_speed_tracker()
        applied = self._try_apply_plant_ic(self._last_ic)
        self.plant_ic_applied = bool(applied)
        if applied:
            self.plant_error = None
        else:
            self.plant_error = (
                "Scenario IC not written to CarSim plant (procedure IC active). "
                "Edit scenario ego or ensure overlay rewrote SV_VXS / path start."
            )

    def close(self) -> None:
        """Idempotent release of the VS solver handle."""
        if self._manager is not None:
            try:
                self._manager.close()
            except Exception as exc:
                self.plant_error = f"terminate_run failed: {exc}"
            self._manager = None
        self._initialized = False
        self._cached = None
        self._t_step_s = None
        self.plant_ic_applied = False
        self._reset_speed_tracker()

    def _try_apply_plant_ic(self, initial_condition: dict[str, Any]) -> bool:
        """Return True when scenario IC was baked into the session Run_all.par.

        Live VS set-state is still unavailable; success means the road-overlay
        path rewrote ``SV_VXS`` / path-start fields before ``vs_initialize``.
        """
        del initial_condition
        ic_par = (self._road_config or {}).get("ic_par") or {}
        fields = ic_par.get("fields") or []
        return bool(ic_par.get("applied")) or bool(fields)

    def _reset_speed_tracker(self) -> None:
        self.plant_speed_mps = None
        self._speed_samples = 0
        self._speed_sum = 0.0
        self._freeze_watch_x = None
        self._freeze_watch_steps = 0

    def _maybe_recover_frozen_plant(self) -> None:
        """Detect stuck plant (Vx>0 but Xo frozen). Do **not** reopen mid-run.

        Re-opening the VS solver snaps the vehicle to the path IC (looks like a
        teleport). Surface ``plant_error`` instead; operator can Stop/Create.
        """
        if self._freeze_recovered or self._cached is None:
            return
        st = self._cached
        if abs(float(st.vx_mps)) < 0.5:
            self._freeze_watch_x = st.x_m
            self._freeze_watch_steps = 0
            return
        if self._freeze_watch_x is None:
            self._freeze_watch_x = st.x_m
            self._freeze_watch_steps = 0
            return
        if abs(st.x_m - self._freeze_watch_x) > 0.05:
            self._freeze_watch_x = st.x_m
            self._freeze_watch_steps = 0
            return
        self._freeze_watch_steps += 1
        # ~0.25 s at 1 ms physics with no translation while speed > 0.5
        if self._freeze_watch_steps < 250:
            return
        self._freeze_recovered = True
        self.plant_error = (
            "CarSim plant frozen (Vx>0 but Xo stuck). Stop and Create a new session."
        )

    def _update_plant_speed(self, state: VehicleState) -> None:
        """Track measured Vx; in plant mode this is the CarSim-owned cruise estimate."""
        vx = state.vx_mps
        if vx is None:
            return
        speed = abs(float(vx))
        # Short moving average after start so UI/controllers see stable "设定速度"
        if self._speed_samples < 100:
            self._speed_sum += speed
            self._speed_samples += 1
            self.plant_speed_mps = self._speed_sum / self._speed_samples
        else:
            # EMA thereafter
            assert self.plant_speed_mps is not None
            self.plant_speed_mps = 0.95 * self.plant_speed_mps + 0.05 * speed

    def capabilities(self) -> VehicleCapabilities:
        has_accel = all(k in self.output_list for k in ("Ax", "Ay"))
        has_tires = any(
            k.startswith(("Fy_", "Fx_", "Fz_", "Alpha_", "Kappa_")) for k in self.output_list
        )
        return VehicleCapabilities(
            has_accel=has_accel,
            has_steering_wheel=any(
                k in self.output_list for k in ("Steer_SW", "STEER_SW")
            ),
            has_wheel_states=has_tires,
            has_tire_forces=has_tires,
            notes=str(conversion_notes()),
        )

    def _open_manager(self) -> None:
        roots = [self._pycarsimlib_root] if self._pycarsimlib_root else None
        _ensure_pycarsimlib_on_path(roots)
        try:
            from pycarsimlib.core import CarsimManager  # type: ignore
        except ImportError as exc:
            raise CarSimUnavailableError(
                "pycarsimlib not importable. Expected under "
                "python_carsim/pycarsimlib (needs deps: rich, numpy) "
                "or pass pycarsimlib_root=... "
                f"Underlying error: {exc}"
            ) from exc

        if not self.carsim_db_dir.is_dir():
            raise CarSimUnavailableError(f"carsim_db_dir not found: {self.carsim_db_dir}")

        try:
            self._manager = CarsimManager(
                carsim_db_dir=str(self.carsim_db_dir),
                vehicle_type=self.vehicle_type,
                input_list=self.input_list,
                output_list=self.output_list,
            )
        except Exception as exc:
            # Retry with kinematics-only labels if a custom output_list fails.
            if list(self.output_list) != list(CORE_OUTPUTS):
                try:
                    self.output_list = list(CORE_OUTPUTS)
                    self._manager = CarsimManager(
                        carsim_db_dir=str(self.carsim_db_dir),
                        vehicle_type=self.vehicle_type,
                        input_list=self.input_list,
                        output_list=self.output_list,
                    )
                    self.plant_error = (
                        f"CarSim rejected output_list; fell back to CORE_OUTPUTS ({exc})"
                    )
                except Exception as exc2:
                    raise CarSimUnavailableError(
                        f"Failed to initialize CarsimManager: {exc2}"
                    ) from exc2
            else:
                raise CarSimUnavailableError(
                    f"Failed to initialize CarsimManager: {exc}"
                ) from exc

        # Guard: label count must not exceed VS n_export (zip would silently drop tails;
        # a *longer* VS table with a *shorter* mis-ordered label list is worse — wrong slots).
        n_exp = int(getattr(self._manager, "num_of_export_values", 0) or 0)
        if n_exp > 0 and len(self.output_list) > n_exp:
            self.output_list = self.output_list[:n_exp]
            self._manager.export_label_array = list(self.output_list)

        self._t_step_s = float(self._manager.t_step)
        self._sim_time_ns = int(round(float(self._manager.t_current) * 1e9))

    def _require_manager(self) -> None:
        if self._manager is None or not self._initialized:
            raise RuntimeError("CarSimAdapter not initialized")
