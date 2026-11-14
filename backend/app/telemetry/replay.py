"""Offline MCAP replay — no CarSim / Engine involvement."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from mcap.reader import make_reader

from backend.app.platform.paths import get_paths


@dataclass
class IndexedFrame:
    t_ns: int
    state: dict[str, Any]
    control: dict[str, Any] | None = None
    npcs: list[dict[str, Any]] | None = None
    obstacles: list[dict[str, Any]] | None = None
    diagnostics: dict[str, Any] | None = None


@dataclass
class ExperimentSummary:
    id: str
    path: str
    metadata: dict[str, Any] = field(default_factory=dict)
    status: str | None = None
    t_start_ns: int = 0
    t_end_ns: int = 0
    n_states: int = 0
    has_mcap: bool = False


def list_experiments(root: Path | None = None) -> list[ExperimentSummary]:
    """List experiment dirs without parsing MCAP bodies.

    Full ``ReplayIndex.load`` over dozens of runs can take tens of seconds and
    blocks the API (Logs UI stayed empty until the request finished). Duration /
    state counts come from ``metrics.json`` when present; open Replay / series
    still builds the index on demand.
    """
    base = root or get_paths().experiments_dir
    if not base.is_dir():
        return []
    out: list[ExperimentSummary] = []
    dirs = [d for d in base.iterdir() if d.is_dir()]
    # Newest first (mtime) — name sort buried runs like test01 among 50+ ids.
    dirs.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    for d in dirs:
        mcap = d / "simulation.mcap"
        meta: dict[str, Any] = {}
        meta_path = d / "metadata.json"
        if meta_path.exists():
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                meta = {}
        status = None
        status_path = d / "status.txt"
        if status_path.exists():
            status = status_path.read_text(encoding="utf-8").strip()
        metrics: dict[str, Any] = {}
        metrics_path = d / "metrics.json"
        if metrics_path.exists():
            try:
                raw = json.loads(metrics_path.read_text(encoding="utf-8"))
                if isinstance(raw, dict):
                    metrics = raw
            except json.JSONDecodeError:
                metrics = {}
        # Ignore empty/partial MCAPs still open for write (0 bytes until finalize).
        mcap_ok = mcap.is_file() and mcap.stat().st_size > 64
        duration_s = float(metrics.get("duration_s") or 0.0)
        n_states = int(metrics.get("n_states") or 0)
        summary = ExperimentSummary(
            id=d.name,
            path=str(d),
            metadata=meta,
            status=status,
            has_mcap=mcap_ok,
            t_start_ns=0,
            t_end_ns=int(max(0.0, duration_s) * 1e9),
            n_states=n_states,
        )
        out.append(summary)
    return out


class ReplayIndex:
    """In-memory index of /ego/state (+ nearest control / diagnostics / world)."""

    def __init__(self, frames: list[IndexedFrame], *, friction_mu: float = 1.0) -> None:
        self.frames = frames
        self.friction_mu = float(friction_mu)
        self.t_start_ns = frames[0].t_ns if frames else 0
        self.t_end_ns = frames[-1].t_ns if frames else 0
        # Precompute for O(1) trail slices (MCAP may store 1 kHz states).
        self._xy: list[list[float]] = [
            [float(f.state.get("x_m", 0.0)), float(f.state.get("y_m", 0.0))] for f in frames
        ]

    @classmethod
    def load(cls, mcap_path: str | Path) -> "ReplayIndex":
        path = Path(mcap_path)
        states: list[tuple[int, dict[str, Any]]] = []
        controls: list[tuple[int, dict[str, Any]]] = []
        diags: list[tuple[int, dict[str, Any]]] = []
        npc_msgs: list[tuple[int, list[dict[str, Any]]]] = []
        obs_msgs: list[tuple[int, list[dict[str, Any]]]] = []
        with path.open("rb") as f:
            reader = make_reader(f)
            for _schema, channel, message in reader.iter_messages():
                payload = json.loads(message.data.decode("utf-8"))
                t = int(payload.get("sim_time_ns", message.log_time))
                if channel.topic == "/ego/state":
                    states.append((t, payload))
                elif channel.topic == "/ego/control":
                    controls.append((t, payload))
                elif channel.topic == "/controller/diagnostics":
                    if isinstance(payload, dict):
                        diags.append((t, payload))
                elif channel.topic == "/world/npcs":
                    raw = payload.get("npcs", [])
                    if isinstance(raw, list):
                        npc_msgs.append((t, [x for x in raw if isinstance(x, dict)]))
                elif channel.topic == "/world/obstacles":
                    raw = payload.get("obstacles", [])
                    if isinstance(raw, list):
                        obs_msgs.append((t, [x for x in raw if isinstance(x, dict)]))
        states.sort(key=lambda x: x[0])
        controls.sort(key=lambda x: x[0])
        diags.sort(key=lambda x: x[0])
        npc_msgs.sort(key=lambda x: x[0])
        obs_msgs.sort(key=lambda x: x[0])
        frames: list[IndexedFrame] = []
        ci = 0
        di = 0
        ni = 0
        oi = 0
        last_ctrl: dict[str, Any] | None = None
        last_diag: dict[str, Any] | None = None
        last_npcs: list[dict[str, Any]] | None = None
        last_obs: list[dict[str, Any]] | None = None
        for t, st in states:
            while ci < len(controls) and controls[ci][0] <= t:
                last_ctrl = controls[ci][1]
                ci += 1
            while di < len(diags) and diags[di][0] <= t:
                last_diag = diags[di][1]
                di += 1
            while ni < len(npc_msgs) and npc_msgs[ni][0] <= t:
                last_npcs = npc_msgs[ni][1]
                ni += 1
            while oi < len(obs_msgs) and obs_msgs[oi][0] <= t:
                last_obs = obs_msgs[oi][1]
                oi += 1
            frames.append(
                IndexedFrame(
                    t_ns=t,
                    state=st,
                    control=last_ctrl,
                    diagnostics=dict(last_diag) if last_diag is not None else None,
                    npcs=list(last_npcs) if last_npcs is not None else None,
                    obstacles=list(last_obs) if last_obs is not None else None,
                )
            )
        if not frames:
            raise ValueError(f"No /ego/state messages in {path}")
        return cls(frames)

    def backfill_actors_from_metadata(self, experiment_dir: str | Path) -> None:
        """Synthesize NPC/obstacle poses for older MCAPs that lack /world/* topics."""
        if not self.frames:
            return
        has_npcs = any(f.npcs for f in self.frames)
        has_obs = any(f.obstacles for f in self.frames)
        if has_npcs and has_obs:
            return
        meta_path = Path(experiment_dir) / "metadata.json"
        if not meta_path.is_file():
            return
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            return
        if not isinstance(meta, dict):
            return
        scenario_raw = meta.get("scenario")
        if not isinstance(scenario_raw, dict):
            return

        if not has_obs:
            obs_out: list[dict[str, Any]] = []
            for o in scenario_raw.get("obstacles") or []:
                if not isinstance(o, dict):
                    continue
                obs_out.append(
                    {
                        "id": str(o.get("id", "obs")),
                        "x_m": float(o.get("x_m", 0.0)),
                        "y_m": float(o.get("y_m", 0.0)),
                        "yaw_rad": float(o.get("yaw_rad") or 0.0),
                        "length_m": float(o.get("length_m", 2.0)),
                        "width_m": float(o.get("width_m", 1.0)),
                    }
                )
            if obs_out:
                for fr in self.frames:
                    if fr.obstacles is None:
                        fr.obstacles = list(obs_out)

        if has_npcs:
            return
        npc_cfgs = scenario_raw.get("npcs") or []
        if not isinstance(npc_cfgs, list) or not npc_cfgs:
            return

        try:
            from backend.app.scenario.npc import TrajectoryNpc
            from backend.app.schemas.scenario import NpcConfig
            from backend.app.schemas.track import TrackDefinition
            from backend.app.schemas.vehicle import VehicleState
        except Exception:
            return

        track: TrackDefinition | None = None
        track_path = meta.get("track_path")
        if isinstance(track_path, str) and Path(track_path).is_file():
            try:
                track = TrackDefinition.model_validate_json(
                    Path(track_path).read_text(encoding="utf-8")
                )
            except Exception:
                track = None
        if track is None:
            track_id = meta.get("track_id") or scenario_raw.get("track_id")
            if isinstance(track_id, str) and track_id:
                try:
                    from backend.app.scenario import config_store

                    track = config_store.load_track(track_id)
                except Exception:
                    track = None
        if track is None:
            return

        agents: list[Any] = []
        for raw in npc_cfgs:
            if not isinstance(raw, dict):
                continue
            try:
                cfg = NpcConfig.model_validate(raw)
                agents.append(TrajectoryNpc(cfg, track=track))
            except Exception:
                continue
        if not agents:
            return

        t_prev = self.frames[0].t_ns
        for i, fr in enumerate(self.frames):
            if i > 0:
                dt = max(0.0, (fr.t_ns - t_prev) * 1e-9)
                # Match publish-rate stepping; sub-step if a seek gap is large.
                remain = dt
                st = fr.state
                ego = VehicleState(
                    sim_time_ns=fr.t_ns,
                    vehicle_id=str(st.get("vehicle_id", "ego")),
                    x_m=float(st.get("x_m", 0.0)),
                    y_m=float(st.get("y_m", 0.0)),
                    z_m=float(st.get("z_m", 0.0)),
                    yaw_rad=float(st.get("yaw_rad", 0.0)),
                    roll_rad=float(st.get("roll_rad") or 0.0),
                    pitch_rad=float(st.get("pitch_rad") or 0.0),
                    vx_mps=float(st.get("vx_mps", 0.0)),
                    vy_mps=float(st.get("vy_mps", 0.0)),
                    yaw_rate_radps=float(st.get("yaw_rate_radps", 0.0)),
                )
                while remain > 1e-9:
                    step = min(remain, 0.02)
                    poses = [a.pose for a in agents]
                    for a in agents:
                        a.advance(
                            step,
                            ego=ego,
                            others=poses,
                            time_s=fr.t_ns * 1e-9,
                        )
                    remain -= step
            fr.npcs = [a.as_dict() for a in agents]
            t_prev = fr.t_ns

    def index_at(self, t_ns: int) -> int:
        if not self.frames:
            raise RuntimeError("empty replay")
        if t_ns <= self.frames[0].t_ns:
            return 0
        if t_ns >= self.frames[-1].t_ns:
            return len(self.frames) - 1
        lo, hi = 0, len(self.frames) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self.frames[mid].t_ns <= t_ns:
                lo = mid
            else:
                hi = mid - 1
        return lo

    def frame_at(self, t_ns: int) -> IndexedFrame:
        return self.frames[self.index_at(t_ns)]

    def trail_upto(self, t_ns: int, max_points: int = 200) -> list[list[float]]:
        """Downsampled trail without scanning all prior frames each tick."""
        if not self._xy:
            return []
        end = self.index_at(t_ns) + 1
        if end <= max_points:
            return self._xy[:end]
        step = max(1, end // max_points)
        out = self._xy[:end:step]
        # always include the newest point
        if out[-1] is not self._xy[end - 1]:
            out = out + [self._xy[end - 1]]
        return out

    def series(self, max_points: int = 800) -> dict[str, list[float]]:
        from backend.app.telemetry.dyn_diagnostics import beta_rad, u_tire_frame

        frames = self.frames
        if len(frames) > max_points:
            step = max(1, len(frames) // max_points)
            frames = frames[::step]
        t_s = [(f.t_ns - self.t_start_ns) * 1e-9 for f in frames]
        out: dict[str, list[float]] = {
            "t_s": t_s,
            "vx_mps": [float(f.state.get("vx_mps", 0.0)) for f in frames],
            "vy_mps": [float(f.state.get("vy_mps", 0.0)) for f in frames],
            "yaw_rate_radps": [float(f.state.get("yaw_rate_radps", 0.0)) for f in frames],
            "yaw_rad": [float(f.state.get("yaw_rad", 0.0)) for f in frames],
            "ax_mps2": [float(f.state.get("ax_mps2") or 0.0) for f in frames],
            "ay_mps2": [float(f.state.get("ay_mps2") or 0.0) for f in frames],
            "steering_cmd": [
                float((f.control or {}).get("steering_cmd", 0.0)) for f in frames
            ],
            "x_m": [float(f.state.get("x_m", 0.0)) for f in frames],
            "y_m": [float(f.state.get("y_m", 0.0)) for f in frames],
            "beta_rad": [
                beta_rad(
                    float(f.state.get("vx_mps", 0.0)),
                    float(f.state.get("vy_mps", 0.0)),
                )
                for f in frames
            ],
        }
        import math as _math

        out["abs_beta_deg"] = [
            abs(float(b)) * (180.0 / _math.pi) for b in out["beta_rad"]
        ]
        # U_tire only when at least one frame has tire forces.
        # Use None (JSON null) for gaps — NaN is not JSON-safe for the browser.
        u_vals: list[float | None] = []
        any_u = False
        for f in frames:
            ws = f.state.get("wheel_states")
            u = u_tire_frame(ws if isinstance(ws, dict) else None, mu=self.friction_mu)
            if u is None:
                u_vals.append(None)
            else:
                any_u = True
                u_vals.append(float(u))
        if any_u:
            out["u_tire"] = u_vals  # type: ignore[assignment]

        # Controller diagnostics (numeric scalars only).
        diag_keys = ("solve_ms", "deadline_miss", "deadline_miss_count", "best_cost")
        for key in diag_keys:
            vals: list[float | None] = []
            present = False
            for f in frames:
                d = f.diagnostics or {}
                if key in d and d[key] is not None:
                    present = True
                    try:
                        vals.append(float(d[key]))
                    except (TypeError, ValueError):
                        vals.append(1.0 if bool(d[key]) else 0.0)
                else:
                    vals.append(None)
            if present:
                out[key] = vals  # type: ignore[assignment]

        wheel_keys: set[str] = set()
        for f in frames:
            ws = f.state.get("wheel_states") or {}
            if not isinstance(ws, dict):
                continue
            for k, v in ws.items():
                if isinstance(v, (int, float)) and not isinstance(v, bool):
                    wheel_keys.add(str(k))
        for k in sorted(wheel_keys):
            out[k] = [
                float((f.state.get("wheel_states") or {}).get(k) or 0.0) for f in frames
            ]
        return out


class ReplayPlayer:
    """Background ticker that publishes snapshots for a replay session."""

    def __init__(self, experiment_id: str, index: ReplayIndex) -> None:
        self.experiment_id = experiment_id
        self.index = index
        self.t_ns = index.t_start_ns
        self.playing = False
        self.rate = 1.0
        self._lock = threading.Lock()
        self._listeners: list[Any] = []
        self._thread: threading.Thread | None = None
        self._alive = False
        self._seq = 0

    def start_thread(self) -> None:
        if self._thread is not None:
            return
        self._alive = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name=f"replay-{self.experiment_id}")
        self._thread.start()

    def stop_thread(self) -> None:
        self._alive = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def play(self) -> None:
        with self._lock:
            self.playing = True

    def pause(self) -> None:
        with self._lock:
            self.playing = False

    def seek(self, t_ns: int) -> dict[str, Any]:
        with self._lock:
            self.t_ns = max(self.index.t_start_ns, min(self.index.t_end_ns, int(t_ns)))
            return self._snapshot_unlocked()

    def set_rate(self, rate: float) -> None:
        with self._lock:
            self.rate = max(0.1, min(8.0, float(rate)))

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "experiment_id": self.experiment_id,
                "playing": self.playing,
                "rate": self.rate,
                "t_ns": self.t_ns,
                "t_start_ns": self.index.t_start_ns,
                "t_end_ns": self.index.t_end_ns,
                "progress": 0.0
                if self.index.t_end_ns == self.index.t_start_ns
                else (self.t_ns - self.index.t_start_ns)
                / (self.index.t_end_ns - self.index.t_start_ns),
            }

    def current_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_unlocked()

    def _snapshot_unlocked(self) -> dict[str, Any]:
        from backend.app.telemetry.dyn_diagnostics import (
            beta_rad,
            slim_ctrl_diag,
            u_tire_frame,
        )

        fr = self.index.frame_at(self.t_ns)
        st = fr.state
        self._seq += 1
        vx = float(st.get("vx_mps", 0.0))
        vy = float(st.get("vy_mps", 0.0))
        ws = st.get("wheel_states")
        u = u_tire_frame(
            ws if isinstance(ws, dict) else None, mu=self.index.friction_mu
        )
        metrics: dict[str, Any] = {
            "beta_rad": beta_rad(vx, vy),
            "friction_mu": self.index.friction_mu,
            "ctrl_diag": slim_ctrl_diag(fr.diagnostics),
        }
        if u is not None:
            metrics["u_tire"] = u
        return {
            "schema_version": "1.0",
            "type": "replay.snapshot",
            "experiment_id": self.experiment_id,
            "seq": self._seq,
            "sim_time_ns": fr.t_ns,
            "vehicles": [
                {
                    "id": st.get("vehicle_id", "ego"),
                    "x_m": float(st.get("x_m", 0.0)),
                    "y_m": float(st.get("y_m", 0.0)),
                    "z_m": float(st.get("z_m", 0.0)),
                    "yaw_rad": float(st.get("yaw_rad", 0.0)),
                    "vx_mps": vx,
                    "vy_mps": vy,
                    "yaw_rate_radps": float(st.get("yaw_rate_radps", 0.0)),
                }
            ],
            "control": fr.control,
            "npcs": list(fr.npcs or []),
            "obstacles": list(fr.obstacles or []),
            "trail_xy": self.index.trail_upto(self.t_ns),
            "metrics": metrics,
            "replay": {
                "playing": self.playing,
                "rate": self.rate,
                "t_start_ns": self.index.t_start_ns,
                "t_end_ns": self.index.t_end_ns,
                "t_ns": self.t_ns,
            },
        }

    def _loop(self) -> None:
        last = time.perf_counter()
        tick = 1.0 / 60.0
        while self._alive:
            now = time.perf_counter()
            dt = now - last
            last = now
            with self._lock:
                if self.playing:
                    self.t_ns += int(dt * self.rate * 1e9)
                    if self.t_ns >= self.index.t_end_ns:
                        self.t_ns = self.index.t_end_ns
                        self.playing = False
            # Cap sleep so heavy load doesn't accumulate huge dt jumps.
            elapsed = time.perf_counter() - now
            time.sleep(max(0.0, tick - elapsed))

