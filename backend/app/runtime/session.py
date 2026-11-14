"""Simulation session: sole worker thread owning the Engine / plant."""

from __future__ import annotations

import json
import queue
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

from backend.app.controllers.registry import create_controller
from backend.app.core.engine import Engine
from backend.app.core.state_machine import IllegalTransitionError, SessionState
from backend.app.platform.paths import get_paths
from backend.app.runtime.actuation import ActuationGuard
from backend.app.runtime.monitor import CompositeMonitor, GeometryMonitor, TimeoutMonitor
from backend.app.runtime.publisher import LatestOnlyPublisher
from backend.app.scenario.simple import SimpleScenario
from backend.app.telemetry.mcap_writer import McapRecorder, NullRecorder
from backend.app.vehicle.carsim_adapter import CarSimAdapter, CarSimUnavailableError
from backend.app.vehicle.carsim_road_overlay import (
    build_constant_mu_overlay,
    cleanup_overlay,
    mirror_session_out_to_gui_results,
)
from backend.app.vehicle.mock_adapter import MockDynamicsAdapter

CommandName = Literal["start", "pause", "resume", "stop", "reset", "step"]


def _load_carsim_io(carsim_db: Path) -> dict[str, Any]:
    """Optional IO override enabling throttle/motor when present in CarSim GUI."""
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


@dataclass
class SessionCommand:
    name: CommandName
    result_q: queue.Queue = field(default_factory=queue.Queue)


class SimSession:
    def __init__(
        self,
        *,
        plant: str = "mock",
        track_path: Path | None = None,
        scenario_path: Path | None = None,
        carsim_db: Path | None = None,
        controller: str = "pure_pursuit",
        steering_cmd: float = 0.0,
        throttle_cmd: float = 0.35,
        record: bool = True,
        realtime: bool = False,
        display_name: str | None = None,
    ) -> None:
        self.session_id = "session_" + uuid.uuid4().hex[:10]
        self.plant = plant
        paths = get_paths()
        self.track_path = track_path or (paths.tracks_dir / "oval_01.json")
        self.scenario_path = scenario_path or (
            paths.scenarios_dir / "web_demo_001.json"
        )
        self.carsim_db = Path(carsim_db) if carsim_db is not None else paths.carsim_db
        self.controller_name = controller
        self.steering_cmd = steering_cmd
        self.throttle_cmd = throttle_cmd
        self.record = record
        self.realtime = realtime
        self.display_name = display_name

        self.publisher = LatestOnlyPublisher()
        self.publisher.session_id = self.session_id
        self.engine: Engine | None = None
        self.experiment_dir: Path | None = None
        self.error: str | None = None
        self._carsim_overlay_dir: Path | None = None
        self._max_sim_time_s: float | None = None
        self._duration_source: str | None = None  # "carsim_tstop" | "scenario"
        # Max mtime of track/scenario at last engine build (reload if files change).
        self._assets_mtime_at_build: float = 0.0
        self._last_gui_mirror: dict[str, Any] | None = None

        self._cmds: queue.Queue[SessionCommand | None] = queue.Queue()
        self._alive = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        # Wall-clock pace anchors (realtime Live); reset on pause/stop.
        self._pace_wall0: float | None = None
        self._pace_sim0_ns: int = 0

    def start_worker(self) -> None:
        if self._thread is not None:
            return
        self._alive = True
        self._thread = threading.Thread(
            target=self._run,
            name=f"sim-{self.session_id}",
            daemon=True,
        )
        self._thread.start()

    def stop_worker(self) -> None:
        self._alive = False
        self._cmds.put(None)
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        # Always release plant (CarSim DLL is single-instance).
        if self.engine is not None:
            try:
                rec = getattr(self.engine, "recorder", None)
                if rec is not None and not getattr(rec, "_closed", True):
                    rec.finalize("closed")
            except Exception:
                pass
            try:
                self.engine.vehicle.close()
            except Exception:
                pass
        self._mirror_carsim_out_to_gui()
        if self._carsim_overlay_dir is not None:
            cleanup_overlay(self._carsim_overlay_dir)
            self._carsim_overlay_dir = None

    def submit(self, name: CommandName, timeout_s: float = 30.0) -> dict[str, Any]:
        cmd = SessionCommand(name=name)
        self._cmds.put(cmd)
        try:
            ok, detail = cmd.result_q.get(timeout=timeout_s)
        except queue.Empty:
            return {"accepted": False, "error": "command timed out"}
        return {"accepted": ok, "detail": detail, "state": self.status()["state"]}

    def status(self) -> dict[str, Any]:
        with self._lock:
            eng = self.engine
            state = eng.state.value if eng is not None else "CREATED"
            t_ns = eng.clock.simulation_time_ns if eng is not None else 0
            step = eng.clock.step_index if eng is not None else 0
            plant_ic_applied = False
            plant_error: str | None = None
            longitudinal_mode = "control"
            plant_speed_mps: float | None = None
            has_longitudinal_control = True
            deadline_miss_count = 0
            last_solve_ms: float | None = None
            deadline_budget_ms: float | None = None
            plant_road_mu: float | None = None
            plant_road_mu_applied = False
            plant_ic_par: dict[str, Any] | None = None
            if eng is not None:
                veh = eng.vehicle
                plant_ic_applied = bool(getattr(veh, "plant_ic_applied", False))
                pe = getattr(veh, "plant_error", None)
                plant_error = str(pe) if pe else None
                longitudinal_mode = str(getattr(veh, "longitudinal_mode", "control"))
                ps = getattr(veh, "plant_speed_mps", None)
                plant_speed_mps = float(ps) if ps is not None else None
                has_longitudinal_control = bool(
                    getattr(veh, "has_longitudinal_control", True)
                )
                deadline_miss_count = int(getattr(eng, "deadline_miss_count", 0))
                ls = getattr(eng, "last_solve_ms", None)
                last_solve_ms = float(ls) if ls is not None else None
                deadline_budget_ms = float(getattr(eng, "deadline_budget_ms", 20.0))
                prm = getattr(veh, "plant_road_mu", None)
                plant_road_mu = float(prm) if prm is not None else None
                plant_road_mu_applied = bool(
                    getattr(veh, "plant_road_mu_applied", False)
                )
                rc = getattr(veh, "_road_config", None) or {}
                if isinstance(rc, dict) and rc.get("ic_par"):
                    plant_ic_par = dict(rc["ic_par"])
        return {
            "session_id": self.session_id,
            "state": state,
            "plant": self.plant,
            "controller": self.controller_name,
            "sim_time_ns": t_ns,
            "step_index": step,
            "experiment_dir": str(self.experiment_dir) if self.experiment_dir else None,
            "error": self.error,
            "plant_ic_applied": plant_ic_applied,
            "plant_error": plant_error,
            "plant_ic_par": plant_ic_par,
            "longitudinal_mode": longitudinal_mode,
            "plant_speed_mps": plant_speed_mps,
            "has_longitudinal_control": has_longitudinal_control,
            "deadline_miss_count": deadline_miss_count,
            "last_solve_ms": last_solve_ms,
            "deadline_budget_ms": deadline_budget_ms,
            "plant_road_mu": plant_road_mu,
            "plant_road_mu_applied": plant_road_mu_applied,
            "max_sim_time_s": self._max_sim_time_s,
            "duration_source": self._duration_source,
            "plant_gui_mirror": self._last_gui_mirror,
        }

    def _run_metadata(self, extra: dict[str, Any] | None = None) -> dict[str, Any]:
        meta: dict[str, Any] = {
            "schema_version": "1.0",
            "session_id": self.session_id,
            "plant": self.plant,
            "controller": self.controller_name,
            "track_path": str(self.track_path),
            "scenario_path": str(self.scenario_path),
        }
        if self.display_name:
            meta["display_name"] = self.display_name
        if self.engine is not None:
            vehicle = getattr(self.engine, "vehicle", None)
            out_list = getattr(vehicle, "output_list", None)
            if isinstance(out_list, list) and out_list:
                meta["recorded_outputs"] = list(out_list)
            if hasattr(self.engine.scenario, "track_id"):
                meta["track_id"] = self.engine.scenario.track_id()
                if hasattr(self.engine.scenario, "scenario_id"):
                    meta["scenario_id"] = self.engine.scenario.scenario_id()  # type: ignore[attr-defined]
                if hasattr(self.engine.scenario, "config"):
                    cfg = self.engine.scenario.config()  # type: ignore[attr-defined]
                    meta["scenario"] = cfg.model_dump()
        if extra:
            meta.update(extra)
        return meta

    def _run(self) -> None:
        try:
            self._build_engine()
            assert self.engine is not None
            self.engine.load(self._run_metadata())
        except Exception as exc:
            self.error = str(exc)
            self._alive = False
            return

        while self._alive:
            self._drain_commands()
            eng = self.engine
            if eng is None:
                break
            if eng.state == SessionState.RUNNING:
                eng.advance_if_running()
                if self.realtime:
                    # Sync wall clock to sim time (avoids 1 ms sleep jitter on Windows).
                    if self._pace_wall0 is None:
                        self._pace_wall0 = time.perf_counter()
                        self._pace_sim0_ns = eng.clock.simulation_time_ns
                    target = self._pace_wall0 + (
                        eng.clock.simulation_time_ns - self._pace_sim0_ns
                    ) * 1e-9
                    delay = target - time.perf_counter()
                    if delay > 0.0005:
                        time.sleep(delay)
            else:
                self._pace_wall0 = None
                time.sleep(0.005)

    def _drain_commands(self) -> None:
        while True:
            try:
                cmd = self._cmds.get_nowait()
            except queue.Empty:
                return
            if cmd is None:
                self._alive = False
                return
            try:
                self._handle(cmd.name)
                cmd.result_q.put((True, cmd.name))
            except Exception as exc:
                cmd.result_q.put((False, str(exc)))

    def _handle(self, name: CommandName) -> None:
        assert self.engine is not None
        if name == "start":
            self._pace_wall0 = None
            if self.engine.state == SessionState.PAUSED:
                self.engine.resume()
            elif self.engine.state == SessionState.READY:
                # Do not rebuild here: mid-Start CarSim reopen can hang the worker
                # (command timeout → UI “Start 没反应”). After Edit→Save use Reset.
                self.engine.start()
            elif self.engine.state in (
                SessionState.STOPPED,
                SessionState.FINISHED,
                SessionState.ERROR,
            ):
                # Rebuild then start (recorder already finalized)
                self._teardown_engine_for_rebuild()
                self._build_engine()
                assert self.engine is not None
                self.engine.load(self._run_metadata({"restart": True}))
                self.engine.start()
            else:
                raise IllegalTransitionError(f"start not allowed from {self.engine.state}")
        elif name == "pause":
            self.engine.pause()
            self._pace_wall0 = None
        elif name == "resume":
            self._pace_wall0 = None
            self.engine.resume()
        elif name == "stop":
            # Keep worker alive so Reset/new Start can rebuild; only close() kills thread.
            if self.engine.state in (SessionState.RUNNING, SessionState.PAUSED):
                self.engine.stop()
            self._mirror_carsim_out_to_gui()
            self._pace_wall0 = None
        elif name == "reset":
            if self.engine.state not in (
                SessionState.FINISHED,
                SessionState.STOPPED,
                SessionState.ERROR,
                SessionState.PAUSED,
                SessionState.READY,
            ):
                raise IllegalTransitionError(f"reset not allowed from {self.engine.state}")
            # Always rebuild so recorder/plant restart cleanly (CarSim soft-reset N/I).
            self._teardown_engine_for_rebuild()
            self._build_engine()
            assert self.engine is not None
            self.engine.load(self._run_metadata({"reset": True}))
            self._pace_wall0 = None
        elif name == "step":
            self.engine.step_one()
        else:
            raise ValueError(f"unknown command {name}")

    def _mirror_carsim_out_to_gui(self) -> None:
        """Publish session ERD/bin into carsim_db/Results for CarSim GUI Animate."""
        if self.plant != "carsim" or self._carsim_overlay_dir is None:
            return
        try:
            report = mirror_session_out_to_gui_results(
                self._carsim_overlay_dir, carsim_db=self.carsim_db
            )
            self._last_gui_mirror = report
        except Exception as exc:
            self._last_gui_mirror = {"mirrored": False, "reason": str(exc)}

    def _teardown_engine_for_rebuild(self) -> None:
        """Finalize recorder + release plant before replacing the Engine.

        Reset used to drop the old ``McapRecorder`` without ``finalize``, leaving
        an open handle on ``simulation.mcap`` while the next load truncates it.
        Also wipe CarSim ``out/`` so ERD/log from the previous run do not grow
        and slow the next ``vs_integrate`` cycle.
        """
        eng = self.engine
        if eng is not None:
            try:
                rec = getattr(eng, "recorder", None)
                if rec is not None and not getattr(rec, "_closed", True):
                    rec.finalize("reset")
            except Exception:
                pass
            try:
                eng.vehicle.close()
            except Exception:
                pass
        # Mirror before wiping session out/ so GUI can Animate the last Live run.
        self._mirror_carsim_out_to_gui()
        if self._carsim_overlay_dir is not None:
            out_dir = self._carsim_overlay_dir / "out"
            if out_dir.is_dir():
                for p in out_dir.iterdir():
                    try:
                        if p.is_file():
                            p.unlink()
                    except OSError:
                        pass

    def _asset_mtimes(self) -> float:
        mt = 0.0
        for p in (self.track_path, self.scenario_path):
            try:
                mt = max(mt, float(p.stat().st_mtime))
            except OSError:
                pass
        return mt

    def _assets_stale(self) -> bool:
        return self._asset_mtimes() > self._assets_mtime_at_build + 1e-6

    def _build_engine(self) -> None:
        scenario = SimpleScenario()
        scenario.load_files(self.track_path, self.scenario_path)
        self._assets_mtime_at_build = self._asset_mtimes()
        if self.plant == "mock":
            vehicle: Any = MockDynamicsAdapter()
        elif self.plant == "carsim":
            try:
                # Optional override: configs/carsim_platform_io.json or carsim_db/platform_io.json
                io_cfg = _load_carsim_io(self.carsim_db)
                mu = float(scenario.track().friction_mu)
                ego_ic = scenario.ego_initial_condition()
                track = scenario.track()
                try:
                    ref_xy = track.driving_reference_m()
                except Exception:
                    ref_xy = scenario.centerline_m()
                overlay = build_constant_mu_overlay(
                    session_id=self.session_id,
                    carsim_db=self.carsim_db,
                    friction_mu=mu,
                    ego_ic=ego_ic,
                    reference_xy=ref_xy,
                    path_closed=bool(getattr(track, "closed_loop", False)),
                )
                if self._carsim_overlay_dir is not None and (
                    self._carsim_overlay_dir != overlay.session_dir
                ):
                    cleanup_overlay(self._carsim_overlay_dir)
                self._carsim_overlay_dir = overlay.session_dir
                # CarsimManager reads <dir>/simfile.sim — point at overlay, not GUI db.
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
                # Surface map-path outcome on the adapter so Live status can show it.
                if overlay.ic_par.get("path_error"):
                    vehicle.plant_error = (
                        f"map path fallback: {overlay.ic_par.get('path_error')}"
                    )
                elif overlay.ic_par.get("path_source") == "map_reference":
                    # Clear stale plant_error from prior IC attempts when path inject OK.
                    if vehicle.plant_error and "map path" in str(vehicle.plant_error):
                        vehicle.plant_error = None
                # Align platform timeout with CarSim procedure TSTOP when present.
                if (
                    overlay.procedure_tstop_s is not None
                    and overlay.procedure_tstop_s > 0
                    and scenario._config is not None
                ):
                    scenario._config.termination.max_sim_time_s = float(
                        overlay.procedure_tstop_s
                    )
                    self._duration_source = "carsim_tstop"
            except CarSimUnavailableError:
                raise
            except Exception as exc:
                raise CarSimUnavailableError(
                    f"CarSim road μ overlay failed: {exc}"
                ) from exc
        else:
            raise ValueError(f"unknown plant {self.plant}")

        if self._duration_source is None:
            self._duration_source = "scenario"
        self._max_sim_time_s = scenario.max_sim_time_s()

        cfg = scenario.config()
        params = dict(cfg.controller.params) if cfg.controller else {}
        # Session create body wins; else scenario controller.type
        name = self.controller_name or (cfg.controller.type if cfg.controller else "pure_pursuit")
        # Multi-lane maps: Pure Pursuit tracks nearest-lane local ref via lane_pp.
        try:
            n_lanes = int(scenario.track().num_lanes)
        except Exception:
            n_lanes = 1
        if name == "pure_pursuit" and n_lanes >= 2:
            name = "lane_pp"
        if name == "mppi":
            controller = create_controller(
                "mppi",
                throttle_nominal=float(params.get("throttle_cmd", self.throttle_cmd)),
                num_samples=int(params.get("num_samples", 64)),
                horizon=int(params.get("horizon", 25)),
                seed=int(params.get("seed", 0)),
                rollout_backend=str(params.get("rollout_backend", "auto")),
            )
            controller.set_waypoints(scenario.centerline_m())
        elif name == "pure_pursuit":
            kw_pp: dict[str, Any] = {
                "throttle_cmd": float(params.get("throttle_cmd", self.throttle_cmd)),
                "lookahead_m": float(params.get("lookahead_m", 12.0)),
            }
            if "target_speed_mps" in params:
                kw_pp["target_speed_mps"] = float(params["target_speed_mps"])
            controller = create_controller("pure_pursuit", **kw_pp)
            controller.set_waypoints(scenario.centerline_m())
        elif name == "lane_pp":
            kw_lpp: dict[str, Any] = {
                "throttle_cmd": float(params.get("throttle_cmd", self.throttle_cmd)),
                "lookahead_m": float(params.get("lookahead_m", 12.0)),
            }
            if "target_speed_mps" in params:
                kw_lpp["target_speed_mps"] = float(params["target_speed_mps"])
            controller = create_controller("lane_pp", **kw_lpp)
            controller.set_waypoints(scenario.centerline_m())
            controller.configure_lane_planner(params)
        elif name == "stanley":
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
                if key in params:
                    kw_st[key] = params[key]
            if "target_speed_mps" not in kw_st and "throttle_cmd" in params:
                # keep defaults; throttle_cmd alone does not set speed target
                pass
            controller = create_controller("stanley", **kw_st)
            controller.set_waypoints(scenario.centerline_m())
        elif name in ("strategy01", "lattice_pid"):
            kw: dict[str, Any] = {
                "throttle_cmd": float(params.get("throttle_cmd", self.throttle_cmd)),
                "lookahead_m": float(params.get("lookahead_m", 12.0)),
                "replan_every_n": int(params.get("replan_every_n", 1)),
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
            ):
                if key in params:
                    kw[key] = params[key]
            controller = create_controller("lattice_pid", **kw)
            controller.set_waypoints(scenario.centerline_m())
        elif name == "lattice_pid_ml":
            kw_ml: dict[str, Any] = {
                "throttle_cmd": float(params.get("throttle_cmd", self.throttle_cmd)),
                "lookahead_m": float(params.get("lookahead_m", 12.0)),
                "replan_every_n": int(params.get("replan_every_n", 1)),
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
                if key in params:
                    kw_ml[key] = params[key]
            controller = create_controller("lattice_pid_ml", **kw_ml)
            controller.set_waypoints(scenario.centerline_m())
            controller.configure(params)
        else:
            controller = create_controller(
                "constant",
                steering_cmd=float(params.get("steering_cmd", self.steering_cmd)),
                throttle_cmd=float(params.get("throttle_cmd", self.throttle_cmd)),
                brake_cmd=float(params.get("brake_cmd", 0.0)),
            )
        if self.record:
            exp = get_paths().experiments_dir / self.session_id
            self.experiment_dir = exp
            recorder: Any = McapRecorder(exp)
        else:
            recorder = NullRecorder()

        with self._lock:
            self.engine = Engine(
                vehicle=vehicle,
                controller=controller,
                scenario=scenario,
                recorder=recorder,
                actuation_guard=ActuationGuard(),
                snapshot_publisher=self.publisher,
                event_monitor=CompositeMonitor(TimeoutMonitor(), GeometryMonitor()),
            )


class SessionManager:
    def __init__(self) -> None:
        self._sessions: dict[str, SimSession] = {}
        self._lock = threading.Lock()

    def create(self, **kwargs: Any) -> SimSession:
        # CarSim vs_integrate is single-instance; opening a second manager
        # can hard-crash the API process (no Python traceback).
        self.close_all()
        session = SimSession(**kwargs)
        session.start_worker()
        # Wait until load finishes (CREATED→READY). CarSim initialize can take
        # many seconds; returning early left Start queued behind a stuck load.
        ready_wait_s = 90.0 if session.plant == "carsim" else 5.0
        deadline = time.perf_counter() + ready_wait_s
        while time.perf_counter() < deadline:
            st = session.status()["state"]
            if st != "CREATED" or session.error:
                break
            time.sleep(0.05)
        with self._lock:
            self._sessions[session.session_id] = session
        return session

    def get(self, session_id: str) -> SimSession | None:
        with self._lock:
            return self._sessions.get(session_id)

    def list_ids(self) -> list[str]:
        with self._lock:
            return list(self._sessions.keys())

    def close_all(self) -> None:
        with self._lock:
            ids = list(self._sessions.keys())
        for sid in ids:
            self.close(sid)

    def close(self, session_id: str) -> None:
        with self._lock:
            session = self._sessions.pop(session_id, None)
        if session is not None:
            session.stop_worker()

    def release_experiment(self, experiment_id: str) -> list[str]:
        """Stop any live session whose MCAP dir matches ``experiment_id``.

        Needed on Windows so Logs→Delete is not blocked by an open recorder
        handle (WinError 32).
        """
        with self._lock:
            victims = [
                sid
                for sid, sess in self._sessions.items()
                if sid == experiment_id
                or (
                    sess.experiment_dir is not None
                    and sess.experiment_dir.name == experiment_id
                )
            ]
        for sid in victims:
            self.close(sid)
        return victims
