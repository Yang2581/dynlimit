"""One-shot closed-loop experiment (no Live realtime / WebSocket)."""

from __future__ import annotations

import json
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal

from backend.app.controllers.registry import create_controller
from backend.app.core.engine import Engine
from backend.app.experiments.naming import normalize_display_name
from backend.app.platform.paths import get_paths
from backend.app.runtime.actuation import ActuationGuard
from backend.app.runtime.monitor import CompositeMonitor, GeometryMonitor, TimeoutMonitor
from backend.app.runtime.publisher import NullPublisher
from backend.app.scenario import config_store
from backend.app.scenario.simple import SimpleScenario
from backend.app.telemetry.mcap_writer import McapRecorder, NullRecorder
from backend.app.vehicle.carsim_adapter import CarSimAdapter, CarSimUnavailableError
from backend.app.vehicle.carsim_road_overlay import (
    build_constant_mu_overlay,
    cleanup_overlay,
    mirror_session_out_to_gui_results,
)
from backend.app.vehicle.mock_adapter import MockDynamicsAdapter

PlantName = Literal["mock", "carsim"]
ControllerName = Literal[
    "pure_pursuit",
    "lane_pp",
    "lattice_pid",
    "lattice_pid_ml",
    "stanley",
    "mppi",
    "constant",
    "strategy01",
]


def _load_carsim_io(carsim_db: Path) -> dict[str, Any]:
    paths = get_paths()
    candidates = [
        paths.config_root / "carsim_platform_io.json",
        paths.repo_root / "configs" / "carsim_platform_io.json",
        Path(carsim_db) / "platform_io.json",
    ]
    for path in candidates:
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            return data if isinstance(data, dict) else {}
        except (OSError, json.JSONDecodeError):
            continue
    return {}


def _new_experiment_id(display_name: str | None) -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    suffix = uuid.uuid4().hex[:8]
    if display_name:
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in display_name)[:24]
        return f"run_{safe}_{stamp}_{suffix}"
    return f"run_{stamp}_{suffix}"


def _build_controller(
    name: ControllerName,
    params: dict[str, Any],
    centerline: list[list[float]],
) -> Any:
    p = dict(params)
    if name == "mppi":
        if "throttle_cmd" in p and "throttle_nominal" not in p:
            p["throttle_nominal"] = p.pop("throttle_cmd")
        allowed = {
            "horizon",
            "num_samples",
            "dt_s",
            "wheelbase_m",
            "seed",
            "temperature",
            "steer_std",
            "throttle_std",
            "max_steer_rad",
            "throttle_nominal",
            "w_lat",
            "w_yaw",
            "w_steer",
            "w_throttle",
            "top_k_viz",
            "target_speed_mps",
            "rollout_backend",
        }
        filtered = {k: v for k, v in p.items() if k in allowed}
        ctl = create_controller("mppi", **filtered)
        ctl.set_waypoints(centerline)
        return ctl
    if name == "pure_pursuit":
        kw_pp: dict[str, Any] = {
            "throttle_cmd": float(p.get("throttle_cmd", 0.35)),
            "lookahead_m": float(p.get("lookahead_m", 12.0)),
        }
        if "target_speed_mps" in p:
            kw_pp["target_speed_mps"] = float(p["target_speed_mps"])
        if "use_speed_pid" in p:
            kw_pp["use_speed_pid"] = bool(p["use_speed_pid"])
        ctl = create_controller("pure_pursuit", **kw_pp)
        ctl.set_waypoints(centerline)
        return ctl
    if name == "stanley":
        kw_st: dict[str, Any] = {}
        for key in (
            "k_stanley",
            "wheelbase_m",
            "max_steer_rad",
            "target_speed_mps",
            "soft_v_mps",
            "lon_kp",
            "lon_ki",
            "lon_kd",
            "throttle_ff",
        ):
            if key in p:
                kw_st[key] = p[key]
        ctl = create_controller("stanley", **kw_st)
        ctl.set_waypoints(centerline)
        return ctl
    if name in ("strategy01", "lattice_pid"):
        kw: dict[str, Any] = {
            "throttle_cmd": float(p.get("throttle_cmd", 0.35)),
            "lookahead_m": float(p.get("lookahead_m", 12.0)),
            "replan_every_n": int(p.get("replan_every_n", 1)),
        }
        for key in (
            "lateral_offsets_m",
            "d_m",
            "horizon_m",
            "ds_m",
            "ego_radius_m",
            "npc_radius_m",
            "predict_dt_s",
            "w_lat",
            "w_change",
            "v_min_mps",
            "wheelbase_m",
            "max_steer_rad",
            "target_speed_mps",
            "avoidance_mode",
            "road_half_width_m",
        ):
            if key in p:
                kw[key] = p[key]
        ctl = create_controller("lattice_pid", **kw)
        ctl.set_waypoints(centerline)
        return ctl
    if name == "lattice_pid_ml":
        kw_ml: dict[str, Any] = {
            "throttle_cmd": float(p.get("throttle_cmd", 0.35)),
            "lookahead_m": float(p.get("lookahead_m", 12.0)),
            "replan_every_n": int(p.get("replan_every_n", 1)),
        }
        for key in (
            "horizon_m",
            "lc_length_m",
            "ds_m",
            "ego_radius_m",
            "npc_radius_m",
            "predict_dt_s",
            "w_change",
            "v_min_mps",
            "target_speed_mps",
            "target_lane_index",
        ):
            if key in p:
                kw_ml[key] = p[key]
        ctl = create_controller("lattice_pid_ml", **kw_ml)
        ctl.set_waypoints(centerline)
        ctl.configure(p)
        return ctl
    return create_controller(
        "constant",
        steering_cmd=float(p.get("steering_cmd", 0.0)),
        throttle_cmd=float(p.get("throttle_cmd", 0.35)),
        brake_cmd=float(p.get("brake_cmd", 0.0)),
    )


def _build_vehicle(
    plant: PlantName,
    *,
    experiment_id: str,
    scenario: SimpleScenario,
    carsim_db: Path,
) -> tuple[Any, Path | None]:
    """Return (vehicle, overlay_dir_or_none)."""
    if plant == "mock":
        return MockDynamicsAdapter(), None
    if plant != "carsim":
        raise ValueError(f"unknown plant '{plant}'")

    io_cfg = _load_carsim_io(carsim_db)
    mu = float(scenario.track().friction_mu)
    ego_ic = scenario.ego_initial_condition()
    track = scenario.track()
    try:
        ref_xy = track.driving_reference_m()
    except Exception:
        ref_xy = scenario.centerline_m()
    overlay = build_constant_mu_overlay(
        session_id=experiment_id,
        carsim_db=carsim_db,
        friction_mu=mu,
        ego_ic=ego_ic,
        reference_xy=ref_xy,
        path_closed=bool(getattr(track, "closed_loop", False)),
    )
    vehicle = CarSimAdapter(
        overlay.session_dir,
        input_list=io_cfg.get("input_list"),
        output_list=io_cfg.get("output_list"),
        motor_torque_nm=float(io_cfg.get("motor_torque_nm", 200.0)),
        brake_p_max_mpa=float(io_cfg.get("brake_p_max_mpa", 10.0)),
    )
    vehicle.plant_road_mu = overlay.friction_mu
    vehicle.plant_road_mu_applied = True
    vehicle._road_config = {
        "friction_mu": overlay.friction_mu,
        "overlay_applied": True,
        "rdmu_path": str(overlay.rdmu_path),
        "session_dir": str(overlay.session_dir),
        "ic_par": dict(overlay.ic_par or {}),
    }
    if (
        overlay.procedure_tstop_s is not None
        and overlay.procedure_tstop_s > 0
        and scenario._config is not None
        and scenario._config.termination.max_sim_time_s is None
    ):
        scenario._config.termination.max_sim_time_s = float(overlay.procedure_tstop_s)
    return vehicle, overlay.session_dir


def run_experiment(
    *,
    plant: PlantName,
    controller: ControllerName = "pure_pursuit",
    scenario_id: str = "web_demo_001",
    track_id: str | None = None,
    display_name: str | None = None,
    duration_s: float | None = None,
    record: bool = True,
    controller_params: dict[str, Any] | None = None,
    carsim_db: Path | str | None = None,
    experiments_dir: Path | None = None,
    ego_ic: dict[str, Any] | None = None,
    friction_mu: float | None = None,
) -> dict[str, Any]:
    """Run one closed-loop experiment to termination; return summary dict.

    ``plant`` is required (caller must pass mock|carsim). Never realtime-paced.
    Optional ``ego_ic`` overrides scenario ego pose/speed in memory only.
    Optional ``friction_mu`` overrides track default friction (pressure grids).
    """
    if plant not in ("mock", "carsim"):
        raise ValueError(f"plant must be 'mock' or 'carsim', got {plant!r}")

    display = normalize_display_name(display_name)
    scenario_cfg = config_store.load_scenario(scenario_id)
    resolved_track_id = scenario_cfg.track_id
    if track_id and track_id != resolved_track_id:
        raise ValueError(
            f"scenario '{scenario_id}' belongs to track '{resolved_track_id}', "
            f"not '{track_id}'"
        )
    track_path = config_store.resolve_track_file(resolved_track_id)
    scenario_path = config_store.resolve_scenario_file(scenario_id)
    db = Path(carsim_db) if carsim_db is not None else get_paths().carsim_db
    exp_root = Path(experiments_dir) if experiments_dir is not None else get_paths().experiments_dir
    experiment_id = _new_experiment_id(display)
    experiment_dir = exp_root / experiment_id
    experiment_dir.mkdir(parents=True, exist_ok=True)

    scenario = SimpleScenario()
    scenario.load_files(track_path, scenario_path)
    scenario.apply_ego_ic_override(ego_ic)
    scenario.apply_friction_mu(friction_mu)
    if duration_s is not None and duration_s > 0 and scenario._config is not None:
        scenario._config.termination.max_sim_time_s = float(duration_s)

    overlay_dir: Path | None = None
    vehicle: Any = None
    try:
        vehicle, overlay_dir = _build_vehicle(
            plant,
            experiment_id=experiment_id,
            scenario=scenario,
            carsim_db=db,
        )
        params = dict(controller_params or {})
        ctl = _build_controller(controller, params, scenario.centerline_m())
        recorder: Any = McapRecorder(experiment_dir) if record else NullRecorder()
        engine = Engine(
            vehicle=vehicle,
            controller=ctl,
            scenario=scenario,
            recorder=recorder,
            actuation_guard=ActuationGuard(),
            snapshot_publisher=NullPublisher(),
            event_monitor=CompositeMonitor(TimeoutMonitor(), GeometryMonitor()),
            dt_physics_s=0.001,
            dt_control_s=0.01 if plant == "carsim" else 0.02,
            dt_publish_s=0.05,
        )
        meta: dict[str, Any] = {
            "schema_version": "1.0",
            "kind": "one_shot_run",
            "experiment_id": experiment_id,
            "plant": plant,
            "controller": controller,
            "scenario_id": scenario_id,
            "track_id": resolved_track_id,
            "track_path": str(track_path),
            "scenario_path": str(scenario_path),
            "controller_params": params,
            "duration_s_cap": duration_s,
            "ego_ic_override": ego_ic or None,
            "friction_mu_override": friction_mu,
        }
        if display:
            meta["display_name"] = display
        t0 = time.perf_counter()
        engine.load(meta)
        engine.start()
        status = engine.run_while_running()
        wall_s = time.perf_counter() - t0
        metrics = dict(engine.experiment_metrics or {})
        metrics["wall_s"] = wall_s
        metrics["engine_status"] = status
        if hasattr(ctl, "last_diagnostics"):
            diag = getattr(ctl, "last_diagnostics") or {}
            metrics["last_solve_ms"] = diag.get("solve_ms")
            metrics["rollout_backend"] = diag.get("rollout_backend")
        goal_samples: list[dict[str, float]] = []
        if hasattr(scenario, "goal_trajectory"):
            goal_samples = list(scenario.goal_trajectory())  # type: ignore[attr-defined]
        return {
            "experiment_id": experiment_id,
            "status": status,
            "metrics": metrics,
            "goal_samples": goal_samples,
            "experiment_dir": str(experiment_dir.resolve()),
            "metadata": meta,
        }
    finally:
        if vehicle is not None:
            try:
                vehicle.close()
            except Exception:
                pass
        if overlay_dir is not None:
            try:
                mirror_session_out_to_gui_results(overlay_dir, carsim_db=db)
            except Exception:
                pass
            cleanup_overlay(overlay_dir)


__all__ = ["run_experiment", "PlantName", "ControllerName", "CarSimUnavailableError"]
