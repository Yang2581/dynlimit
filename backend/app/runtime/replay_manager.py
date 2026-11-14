"""Manage offline replay players (no dynamics)."""

from __future__ import annotations

import gc
import json
import shutil
import threading
import time
from pathlib import Path
from typing import Any

from backend.app.platform.paths import get_paths
from backend.app.telemetry.dyn_diagnostics import resolve_friction_mu, summarize_series
from backend.app.telemetry.replay import ReplayIndex, ReplayPlayer, list_experiments


def _rmtree_windows_retry(path: Path, *, attempts: int = 10) -> None:
    """``shutil.rmtree`` with backoff for WinError 32 (file still locked)."""
    last: OSError | None = None
    for i in range(attempts):
        gc.collect()
        try:
            shutil.rmtree(path)
            return
        except OSError as exc:
            last = exc
            winerr = getattr(exc, "winerror", None)
            if winerr not in (32, 5) and getattr(exc, "errno", None) not in (13, 11):
                raise
            time.sleep(0.05 * (i + 1))
    assert last is not None
    raise last


class ReplayManager:
    def __init__(self, experiments_dir: Path | None = None) -> None:
        self.experiments_dir = experiments_dir or get_paths().experiments_dir
        self._players: dict[str, ReplayPlayer] = {}
        self._lock = threading.Lock()

    def list(self) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        for e in list_experiments(self.experiments_dir):
            metrics = None
            metrics_path = Path(e.path) / "metrics.json"
            if metrics_path.exists():
                try:
                    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
                except Exception:
                    metrics = None
            meta = e.metadata or {}
            out.append(
                {
                    "id": e.id,
                    "path": e.path,
                    "metadata": meta,
                    "display_name": meta.get("display_name"),
                    "metrics": metrics,
                    "status": e.status,
                    "t_start_ns": e.t_start_ns,
                    "t_end_ns": e.t_end_ns,
                    "n_states": e.n_states,
                    "has_mcap": e.has_mcap,
                    "duration_s": (e.t_end_ns - e.t_start_ns) * 1e-9,
                }
            )
        return out

    def set_display_name(
        self, experiment_id: str, display_name: str | None
    ) -> dict[str, Any]:
        d = self.experiments_dir / experiment_id
        meta_path = d / "metadata.json"
        if not d.is_dir():
            raise FileNotFoundError(experiment_id)
        meta: dict[str, Any] = {}
        if meta_path.exists():
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            if not isinstance(meta, dict):
                meta = {}
        if display_name is None:
            meta.pop("display_name", None)
        else:
            meta["display_name"] = display_name
        meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
        return {"id": experiment_id, "display_name": display_name, "metadata": meta}

    def delete(self, experiment_id: str) -> None:
        self.close(experiment_id)
        d = self.experiments_dir / experiment_id
        if not d.is_dir():
            raise FileNotFoundError(experiment_id)
        _rmtree_windows_retry(d)

    def get_or_create_player(self, experiment_id: str) -> ReplayPlayer:
        with self._lock:
            if experiment_id in self._players:
                return self._players[experiment_id]
            path = self.experiments_dir / experiment_id / "simulation.mcap"
            if not path.exists():
                raise FileNotFoundError(f"experiment mcap not found: {experiment_id}")
            index = ReplayIndex.load(path)
            index.friction_mu = resolve_friction_mu(path.parent)
            # Older runs logged only /ego/*; reconstruct NPC motion from metadata.scenario.
            index.backfill_actors_from_metadata(path.parent)
            player = ReplayPlayer(experiment_id, index)
            player.start_thread()
            self._players[experiment_id] = player
            return player

    def series(self, experiment_id: str) -> dict[str, Any]:
        path = self.experiments_dir / experiment_id / "simulation.mcap"
        if not path.exists():
            raise FileNotFoundError(experiment_id)
        try:
            index = ReplayIndex.load(path)
        except ValueError as exc:
            # Truncated / empty MCAP (has file but no /ego/state) — do not 500 Logs UI.
            return {
                "experiment_id": experiment_id,
                "t_start_ns": 0,
                "t_end_ns": 0,
                "series": {
                    "t_s": [],
                    "vx_mps": [],
                    "vy_mps": [],
                    "yaw_rate_radps": [],
                    "yaw_rad": [],
                    "ax_mps2": [],
                    "ay_mps2": [],
                    "steering_cmd": [],
                    "x_m": [],
                    "y_m": [],
                    "beta_rad": [],
                },
                "empty": True,
                "detail": str(exc),
            }
        index.friction_mu = resolve_friction_mu(path.parent)
        series = index.series()
        run_metrics: dict[str, Any] | None = None
        metrics_path = path.parent / "metrics.json"
        if metrics_path.is_file():
            try:
                raw = json.loads(metrics_path.read_text(encoding="utf-8"))
                if isinstance(raw, dict):
                    run_metrics = raw
            except Exception:
                run_metrics = None
        summary = summarize_series(series, run_metrics=run_metrics)
        return {
            "experiment_id": experiment_id,
            "t_start_ns": index.t_start_ns,
            "t_end_ns": index.t_end_ns,
            "friction_mu": index.friction_mu,
            "diagnostics_available": {
                "beta_rad": True,
                "abs_beta_deg": True,
                "u_tire": "u_tire" in series,
                "solve_ms": "solve_ms" in series,
                "deadline_miss": "deadline_miss" in series,
                "summary": True,
                "flags": True,
            },
            "summary": summary,
            "series": series,
        }

    def close(self, experiment_id: str) -> None:
        with self._lock:
            player = self._players.pop(experiment_id, None)
        if player is not None:
            player.stop_thread()

    def close_all(self) -> None:
        with self._lock:
            ids = list(self._players.keys())
        for i in ids:
            self.close(i)
