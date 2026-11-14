"""Simulation engine — sole owner of plant stepping."""

from __future__ import annotations

import time
from typing import Any

from backend.app.controllers.base import Controller
from backend.app.core.clock import SimulationClock
from backend.app.core.scheduler import MultiRateScheduler
from backend.app.core.state_machine import IllegalTransitionError, SessionState, SessionStateMachine
from backend.app.runtime.actuation import ActuationGuard
from backend.app.runtime.monitor import EventMonitor
from backend.app.runtime.publisher import SnapshotPublisher
from backend.app.scenario.base import Scenario
from backend.app.schemas.snapshot import SimulationSnapshot
from backend.app.telemetry.base import Recorder
from backend.app.vehicle.base import VehicleDynamicsAdapter


class Engine:
    def __init__(
        self,
        *,
        vehicle: VehicleDynamicsAdapter,
        controller: Controller,
        scenario: Scenario,
        recorder: Recorder,
        actuation_guard: ActuationGuard,
        snapshot_publisher: SnapshotPublisher,
        event_monitor: EventMonitor,
        dt_physics_s: float = 0.001,
        dt_control_s: float = 0.01,
        dt_publish_s: float = 0.016,  # ~62.5 Hz UI / MCAP (integer × dt_physics)
        deadline_budget_ms: float = 20.0,
    ) -> None:
        self.vehicle = vehicle
        self.controller = controller
        self.scenario = scenario
        self.recorder = recorder
        self.actuation_guard = actuation_guard
        self.snapshot_publisher = snapshot_publisher
        self.event_monitor = event_monitor
        self.deadline_budget_ms = float(deadline_budget_ms)
        self.clock = SimulationClock(dt_physics_s=dt_physics_s)
        self.scheduler = MultiRateScheduler(
            dt_physics_s=dt_physics_s,
            dt_control_s=dt_control_s,
            dt_publish_s=dt_publish_s,
        )
        self.fsm = SessionStateMachine()
        self._finish_requested = False
        self._finish_reason: str | None = None
        self._last_ctrl_diag: dict[str, Any] = {}
        self._trail: list[list[float]] = []
        self._trail_max = 400
        self.experiment_metrics: dict[str, Any] | None = None
        self.deadline_miss_count: int = 0
        self.last_solve_ms: float | None = None
        self._last_friction_mu: float | None = None
        self._last_friction_zone_id: str | None = None

    @property
    def state(self) -> SessionState:
        return self.fsm.state

    def load(self, metadata: dict[str, Any] | None = None) -> None:
        # Stay CREATED until plant IC is fully primed. Transitioning to READY at the
        # start raced SessionManager.create (it returns as soon as state != CREATED)
        # so the UI could Start while CarSim was still inside initialize/step and
        # the worker never drained the Start command (30s timeout → “没反应”).
        self.vehicle.initialize({}, {})
        ic = self.scenario.ego_initial_condition()
        self.vehicle.reset(ic)
        self.controller.reset({}, self.vehicle.get_state())
        self.actuation_guard.reset()
        if hasattr(self.event_monitor, "reset"):
            self.event_monitor.reset()  # type: ignore[attr-defined]
        self.clock.reset()
        self.recorder.start(metadata or {"schema_version": "1.0"})
        self._finish_requested = False
        self._finish_reason = None
        self.experiment_metrics = None
        self._trail = []
        self._last_ctrl_diag = {}
        self.deadline_miss_count = 0
        self.last_solve_ms = None
        # Push initial pose so Live UI refreshes on Create / Reset (not only while RUNNING).
        self._publish_current_snapshot()
        self.fsm.transition("load")

    def start(self) -> None:
        self.fsm.transition("start")

    def pause(self) -> None:
        self.fsm.transition("pause")

    def resume(self) -> None:
        self.fsm.transition("resume")

    def stop(self) -> None:
        self.fsm.transition("stop")
        self._write_metrics("stopped")
        self.recorder.finalize("stopped")
        # Release plant (esp. CarSim DLL) so Create/Reset can reopen cleanly.
        try:
            self.vehicle.close()
        except Exception:
            pass

    def reset(self) -> None:
        self.fsm.transition("reset")
        # CarSim has no soft reset: close then initialize once, then IC (no double-open).
        try:
            self.vehicle.close()
        except Exception:
            pass
        self.vehicle.initialize({}, {})
        self.vehicle.reset(self.scenario.ego_initial_condition())
        self.controller.reset({}, self.vehicle.get_state())
        self.actuation_guard.reset()
        if hasattr(self.event_monitor, "reset"):
            self.event_monitor.reset()  # type: ignore[attr-defined]
        self.clock.reset()
        self._finish_requested = False
        self._finish_reason = None
        self.experiment_metrics = None
        self._trail = []
        self._last_ctrl_diag = {}
        self.deadline_miss_count = 0
        self.last_solve_ms = None
        self._publish_current_snapshot()

    def step_one(self) -> None:
        if self.fsm.state != SessionState.PAUSED:
            raise IllegalTransitionError("step_one only allowed while PAUSED")
        self.fsm.transition("step_one")
        self._advance_one_physics_step()

    def advance_if_running(self) -> bool:
        """Advance one physics step when RUNNING. Returns False if not running."""
        if self.fsm.state != SessionState.RUNNING:
            return False
        self._advance_one_physics_step()
        if self._finish_requested:
            self.fsm.transition("finish")
            self._write_metrics(self._finish_reason or "finished")
            self.recorder.finalize(self._finish_reason or "finished")
        return True

    def run_while_running(self, max_steps: int | None = None) -> str:
        """Advance while RUNNING until finished/stopped/error or max_steps."""
        steps = 0
        status = "running"
        while self.fsm.state == SessionState.RUNNING:
            self.advance_if_running()
            steps += 1
            if self.fsm.state == SessionState.FINISHED:
                status = "finished"
                break
            if max_steps is not None and steps >= max_steps:
                status = "max_steps"
                break
        return status

    def _advance_one_physics_step(self) -> None:
        t = self.clock.simulation_time_ns
        step_index = self.clock.step_index

        if self.scheduler.controller_due(step_index):
            state = self.vehicle.read_cached_state()
            observation = self.scenario.observation(t)
            # Enrich with plant longitudinal capability (CarSim may be lateral-only)
            extras = dict(observation.extras or {})
            extras["longitudinal_mode"] = getattr(
                self.vehicle, "longitudinal_mode", "control"
            )
            extras["plant_speed_mps"] = getattr(self.vehicle, "plant_speed_mps", None)
            extras["has_longitudinal_control"] = bool(
                getattr(self.vehicle, "has_longitudinal_control", True)
            )
            self._update_friction_at(state.x_m, state.y_m)
            if self._last_friction_mu is not None:
                extras["friction_mu"] = self._last_friction_mu
                extras["friction_zone_id"] = self._last_friction_zone_id
            observation = observation.model_copy(update={"extras": extras})
            t0 = time.perf_counter()
            cmd, diagnostics = self.controller.compute_control(state, observation)
            wall_ms = (time.perf_counter() - t0) * 1000.0
            cmd, clamp_diag = self.actuation_guard.validate_and_limit(cmd, t)
            diagnostics = {**diagnostics, **clamp_diag}
            diagnostics = self._annotate_deadline(diagnostics, wall_ms=wall_ms)
            self._last_ctrl_diag = diagnostics
            self.vehicle.set_control(cmd)
            self.recorder.enqueue(
                "/ego/control",
                t,
                cmd.model_dump(),
            )
            # Keep MCAP payload light: drop heavy traj arrays from log (still in snapshot)
            log_diag = {
                k: v
                for k, v in diagnostics.items()
                if k
                not in (
                    "best_traj_xy",
                    "sample_trajs_xy",
                    "local_path_xy",
                )
            }
            self.recorder.enqueue(
                "/controller/diagnostics",
                t,
                log_diag,
            )

        self.vehicle.step(self.clock.dt_physics_s)
        self.clock.tick()

        state_next = self.vehicle.get_state()
        # Align reported plant time with clock authority
        state_next = state_next.model_copy(update={"sim_time_ns": self.clock.simulation_time_ns})
        # NPCs see updated ego (needed for future IDM / interactive strategies)
        adv = getattr(self.scenario, "advance_npcs", None)
        if callable(adv):
            try:
                adv(self.clock.dt_physics_s, ego=state_next)
            except TypeError:
                adv(self.clock.dt_physics_s)

        events = self.event_monitor.evaluate(state_next, self.scenario)
        for ev in events:
            self.recorder.enqueue("/events", ev.sim_time_ns, ev.model_dump())
            if ev.type in ("timeout", "finished"):
                self._finish_requested = True
                self._finish_reason = ev.type
            elif ev.type == "collision" and getattr(
                self.scenario, "stop_on_collision", lambda: False
            )():
                self._finish_requested = True
                self._finish_reason = "collision"
            elif ev.type == "off_track" and getattr(
                self.scenario, "stop_on_off_track", lambda: False
            )():
                self._finish_requested = True
                self._finish_reason = "off_track"

        self._trail.append([state_next.x_m, state_next.y_m])
        if len(self._trail) > self._trail_max:
            self._trail = self._trail[-self._trail_max :]

        # Record + publish at dt_publish (not every 1 ms physics step) so
        # offline replay indexes stay small and smooth at UI rates.
        if self.scheduler.snapshot_due(self.clock.step_index):
            t_ns = self.clock.simulation_time_ns
            self.recorder.enqueue("/ego/state", t_ns, state_next.model_dump())
            npcs_rec: list[Any] = []
            obstacles_rec: list[Any] = []
            if hasattr(self.scenario, "npc_states"):
                npcs_rec = self.scenario.npc_states()  # type: ignore[attr-defined]
            if hasattr(self.scenario, "obstacles"):
                obstacles_rec = self.scenario.obstacles()  # type: ignore[attr-defined]
            self.recorder.enqueue(
                "/world/npcs",
                t_ns,
                {"sim_time_ns": t_ns, "npcs": npcs_rec},
            )
            self.recorder.enqueue(
                "/world/obstacles",
                t_ns,
                {"sim_time_ns": t_ns, "obstacles": obstacles_rec},
            )
            self._publish_current_snapshot(state=state_next)

    def _resolve_track(self) -> Any | None:
        track_obj = getattr(self.scenario, "track", None)
        if callable(track_obj):
            try:
                track_obj = track_obj()
            except Exception:
                track_obj = None
        if track_obj is None:
            track_obj = getattr(self.scenario, "_track", None)
        return track_obj

    def _update_friction_at(self, x_m: float, y_m: float) -> None:
        track_obj = self._resolve_track()
        if track_obj is None:
            return
        try:
            from backend.app.scenario.friction import friction_at

            mu, zid = friction_at(track_obj, x_m, y_m)
            self._last_friction_mu = mu
            self._last_friction_zone_id = zid
        except Exception:
            pass

    def _publish_current_snapshot(self, state: Any | None = None) -> None:
        """Publish ego/NPC pose for Live UI (also used after load/reset)."""
        state_next = state if state is not None else self.vehicle.get_state()
        state_next = state_next.model_copy(update={"sim_time_ns": self.clock.simulation_time_ns})
        self._update_friction_at(float(state_next.x_m), float(state_next.y_m))
        npcs: list[Any] = []
        obstacles: list[Any] = []
        if hasattr(self.scenario, "npc_states"):
            npcs = self.scenario.npc_states()  # type: ignore[attr-defined]
        if hasattr(self.scenario, "obstacles"):
            obstacles = self.scenario.obstacles()  # type: ignore[attr-defined]
        from backend.app.telemetry.dyn_diagnostics import beta_rad, u_tire_frame

        mu_live = (
            float(self._last_friction_mu)
            if self._last_friction_mu is not None
            else 1.0
        )
        beta_live = beta_rad(float(state_next.vx_mps), float(state_next.vy_mps))
        ws_live = getattr(state_next, "wheel_states", None)
        u_live = u_tire_frame(
            ws_live if isinstance(ws_live, dict) else None, mu=mu_live
        )
        live_metrics: dict[str, Any] = {
            "trail_xy": self._trail[::2],  # downsample for WS payload
            "controller": getattr(
                getattr(self.controller, "__class__", None), "__name__", ""
            ),
            "npcs": npcs,
            "obstacles": obstacles,
            "friction_mu": self._last_friction_mu,
            "friction_zone_id": self._last_friction_zone_id,
            "beta_rad": beta_live,
            "ctrl_diag": {
                k: self._last_ctrl_diag[k]
                for k in (
                    "controller",
                    "ok",
                    "solve_ms",
                    "best_cost",
                    "seed",
                    "best_traj_xy",
                    "sample_trajs_xy",
                    "rollout_backend",
                    "longitudinal_mode",
                    "plant_speed_mps",
                    "deadline_miss",
                    "deadline_budget_ms",
                    "deadline_miss_count",
                    "solve_ms",
                )
                if k in self._last_ctrl_diag
            },
        }
        if u_live is not None:
            live_metrics["u_tire"] = u_live
        snap = SimulationSnapshot(
            sim_time_ns=self.clock.simulation_time_ns,
            step_index=self.clock.step_index,
            vehicles=[state_next],
            metrics=live_metrics,
        )
        self.snapshot_publisher.offer_latest(snap)

    def _annotate_deadline(
        self, diagnostics: dict[str, Any], *, wall_ms: float
    ) -> dict[str, Any]:
        """Attach deadline-miss fields; prefer controller solve_ms when present."""
        reported = diagnostics.get("solve_ms")
        try:
            solve_ms = float(reported) if reported is not None else float(wall_ms)
        except (TypeError, ValueError):
            solve_ms = float(wall_ms)
        miss = solve_ms > self.deadline_budget_ms
        if miss:
            self.deadline_miss_count += 1
        self.last_solve_ms = solve_ms
        out = dict(diagnostics)
        out["solve_ms"] = round(solve_ms, 3)
        out["deadline_budget_ms"] = self.deadline_budget_ms
        out["deadline_miss"] = miss
        out["deadline_miss_count"] = self.deadline_miss_count
        return out

    def _write_metrics(self, reason: str) -> None:
        if hasattr(self.scenario, "metrics_snapshot"):
            self.experiment_metrics = self.scenario.metrics_snapshot(  # type: ignore[attr-defined]
                self.clock.simulation_time_ns, reason=reason
            )
        else:
            self.experiment_metrics = {
                "duration_s": self.clock.simulation_time_ns * 1e-9,
                "finished_reason": reason,
            }
        if isinstance(self.experiment_metrics, dict):
            self.experiment_metrics.setdefault(
                "duration_s", self.clock.simulation_time_ns * 1e-9
            )
            self.experiment_metrics["n_states"] = int(self.clock.step_index)
        # Persist beside MCAP when recorder has an experiment_dir
        exp_dir = getattr(self.recorder, "experiment_dir", None)
        if exp_dir is not None and self.experiment_metrics is not None:
            import json
            from pathlib import Path

            path = Path(exp_dir) / "metrics.json"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(self.experiment_metrics, indent=2), encoding="utf-8")
