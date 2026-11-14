import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  createSession,
  fetchExperiment,
  fetchScenario,
  fetchSeries,
  fetchTrack,
  listExperiments,
  listScenarios,
  listTracks,
  openReplaySocket,
  openSessionSocket,
  replayCommand,
  sessionCommand,
  type ExperimentInfo,
  type ScenarioDef,
  type SeriesPayload,
  type SessionStatus,
  type SnapshotMsg,
  type TrackDef,
} from "./api";
import { AssetsPanel } from "./AssetsPanel";
import { EditorPanel } from "./EditorPanel";
import { LogsPanel } from "./LogsPanel";
import { createPlotWindow, PlotWindow, type PlotWindowState } from "./PlotWindow";
import {
  EDIT_POINT_HIT_M,
  Viewport,
  type CameraMode,
  type ContextMenuRequest,
  type EditPhase,
  type EditTool,
} from "./scene/Viewport";

type EditUndoSnap = { track: TrackDef | null; scenario: ScenarioDef | null };

function cloneEditSnap(track: TrackDef | null, scenario: ScenarioDef | null): EditUndoSnap {
  return {
    track: track ? (JSON.parse(JSON.stringify(track)) as TrackDef) : null,
    scenario: scenario ? (JSON.parse(JSON.stringify(scenario)) as ScenarioDef) : null,
  };
}

type AppMode = "live" | "replay" | "edit" | "assets" | "logs";

/** Blank closed rectangle — mirrors EditorPanel for Assets → New Track. */
function defaultRectangleTrack(id: string): TrackDef {
  const pts: number[][] = [
    [0, 0, 0],
    [40, 0, 0],
    [80, 0, 0],
    [80, 20, 0],
    [80, 40, 0],
    [40, 40, 0],
    [0, 40, 0],
    [0, 20, 0],
    [0, 0, 0],
  ];
  return {
    schema_version: "1.0",
    track_id: id,
    name: id,
    closed_loop: true,
    width_m: 12,
    num_lanes: 1,
    lane_width_m: 12,
    reference_line_m: pts.map((p) => [...p]),
    friction_mu: 1,
    centerline_m: pts,
    sampling_resolution_m: 0.5,
  };
}

function egoPoseFromTrack(track: TrackDef | null | undefined): {
  x_m: number;
  y_m: number;
  z_m: number;
  yaw_rad: number;
} {
  const pts =
    track?.reference_line_m && track.reference_line_m.length >= 2
      ? track.reference_line_m
      : track?.centerline_m;
  if (!pts || pts.length < 2) {
    return { x_m: 0, y_m: 0, z_m: 0, yaw_rad: 0 };
  }
  const a = pts[0];
  const b = pts[1];
  const yaw = Math.atan2(b[1] - a[1], b[0] - a[0]);
  return { x_m: a[0], y_m: a[1], z_m: a[2] ?? 0, yaw_rad: yaw };
}

/** Blank scenario bound to a track — mirrors EditorPanel for Assets → New Scenario. */
function defaultScenario(id: string, trackId: string, track?: TrackDef | null): ScenarioDef {
  const pose = egoPoseFromTrack(track ?? null);
  return {
    schema_version: "1.0",
    scenario_id: id,
    track_id: trackId,
    ego: {
      vehicle_config_id: "mock_bicycle",
      initial_pose: pose,
      initial_speed_mps: 12,
    },
    npcs: [],
    obstacles: [],
    controller: { type: "pure_pursuit", params: { throttle_cmd: 0.35, lookahead_m: 12 } },
    termination: {
      max_sim_time_s: 60,
      stop_on_collision: false,
      stop_on_off_track: false,
    },
  };
}

/** Nearest equal-width lane (0 = rightmost). Falls back to 0 if only a spine. */
function nearestLaneIndex(
  x: number,
  y: number,
  track: Pick<TrackDef, "centerline_m" | "num_lanes" | "lane_width_m" | "width_m">
): number {
  const n = Math.max(1, Math.min(8, Math.round(track.num_lanes ?? 1)));
  if (n <= 1) return 0;
  const cl = track.centerline_m;
  if (!cl || cl.length < 2) return 0;
  const lw = track.lane_width_m ?? track.width_m / n;
  const half = (track.width_m ?? n * lw) * 0.5;
  const offsets = Array.from({ length: n }, (_, i) => -half + (i + 0.5) * lw);

  let bestD2 = Infinity;
  let lat = 0;
  for (let i = 0; i < cl.length - 1; i++) {
    const x0 = cl[i][0];
    const y0 = cl[i][1];
    const x1 = cl[i + 1][0];
    const y1 = cl[i + 1][1];
    const dx = x1 - x0;
    const dy = y1 - y0;
    const L2 = dx * dx + dy * dy;
    if (L2 < 1e-18) continue;
    const L = Math.sqrt(L2);
    let t = ((x - x0) * dx + (y - y0) * dy) / L2;
    t = Math.max(0, Math.min(1, t));
    const px = x0 + t * dx;
    const py = y0 + t * dy;
    const d2 = (x - px) ** 2 + (y - py) ** 2;
    if (d2 < bestD2) {
      bestD2 = d2;
      lat = (x - px) * (-dy / L) + (y - py) * (dx / L);
    }
  }

  let bestI = 0;
  let bestAbs = Infinity;
  for (let i = 0; i < n; i++) {
    const a = Math.abs(lat - offsets[i]);
    if (a < bestAbs) {
      bestAbs = a;
      bestI = i;
    }
  }
  return bestI;
}

function trackIsBidirectional(track: TrackDef | null | undefined): boolean {
  if (!track) return false;
  return !!track.bidirectional;
}

/** Spine tangent at nearest projection; flipped π on oncoming (left) lanes when bidirectional. */
function npcPreviewYaw(
  x: number,
  y: number,
  track: TrackDef | null | undefined,
  laneIndex: number | null | undefined
): number {
  const cl = track?.centerline_m;
  if (!cl || cl.length < 2) return 0;
  let bestD2 = Infinity;
  let yaw = Math.atan2(cl[1][1] - cl[0][1], cl[1][0] - cl[0][0]);
  let lat = 0;
  for (let i = 0; i < cl.length - 1; i++) {
    const x0 = cl[i][0];
    const y0 = cl[i][1];
    const x1 = cl[i + 1][0];
    const y1 = cl[i + 1][1];
    const dx = x1 - x0;
    const dy = y1 - y0;
    const L2 = dx * dx + dy * dy;
    if (L2 < 1e-18) continue;
    const L = Math.sqrt(L2);
    let t = ((x - x0) * dx + (y - y0) * dy) / L2;
    t = Math.max(0, Math.min(1, t));
    const px = x0 + t * dx;
    const py = y0 + t * dy;
    const d2 = (x - px) ** 2 + (y - py) ** 2;
    if (d2 < bestD2) {
      bestD2 = d2;
      yaw = Math.atan2(dy, dx);
      lat = (x - px) * (-dy / L) + (y - py) * (dx / L);
    }
  }
  const n = Math.max(1, Math.round(track?.num_lanes ?? 1));
  const lw = track?.lane_width_m ?? (track?.width_m ?? 12) / n;
  const half = (track?.width_m ?? n * lw) * 0.5;
  const li =
    laneIndex != null
      ? Math.max(0, Math.min(n - 1, laneIndex))
      : nearestLaneIndex(x, y, track!);
  const laneLat = -half + (li + 0.5) * lw;
  const oncoming = trackIsBidirectional(track) && laneLat > 1e-9;
  // Prefer projection lat if lane_index missing consistency
  void lat;
  return oncoming ? yaw + Math.PI : yaw;
}

export function App() {
  const [mode, setMode] = useState<AppMode>("live");
  const [plant, setPlant] = useState<"mock" | "carsim">("carsim");
  const [controller, setController] = useState<
    | "pure_pursuit"
    | "lane_pp"
    | "lattice_pid"
    | "lattice_pid_ml"
    | "constant"
    | "mppi"
    | "strategy01"
  >("pure_pursuit");
  const [runName, setRunName] = useState("");
  const [session, setSession] = useState<SessionStatus | null>(null);
  const [snapshot, setSnapshot] = useState<SnapshotMsg | null>(null);
  const [trackPoints, setTrackPoints] = useState<number[][]>([]);
  const [trackWidth, setTrackWidth] = useState(12);
  const [numLanes, setNumLanes] = useState(1);
  const [laneWidth, setLaneWidth] = useState(12);
  const [referencePoints, setReferencePoints] = useState<number[][]>([]);
  const [editPhase, setEditPhase] = useState<EditPhase>("lanes");
  const [ctxMenu, setCtxMenu] = useState<ContextMenuRequest | null>(null);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const [cameraMode, setCameraMode] = useState<CameraMode>("orbit");
  const [draftTrack, setDraftTrack] = useState<TrackDef | null>(null);
  const [draftScenario, setDraftScenario] = useState<ScenarioDef | null>(null);
  const [selectedTrackId, setSelectedTrackId] = useState("oval_01");
  const [selectedScenarioId, setSelectedScenarioId] = useState("web_demo_001");
  const [editTool, setEditTool] = useState<EditTool>("move");
  const [trackCatalog, setTrackCatalog] = useState<Array<{ id: string }>>([]);
  const [scenarioCatalog, setScenarioCatalog] = useState<
    Array<{ id: string; track_id?: string }>
  >([]);
  const editUndoRef = useRef<EditUndoSnap[]>([]);
  const draftTrackRef = useRef(draftTrack);
  const draftScenarioRef = useRef(draftScenario);
  draftTrackRef.current = draftTrack;
  draftScenarioRef.current = draftScenario;

  const [experiments, setExperiments] = useState<ExperimentInfo[]>([]);
  const [experimentId, setExperimentId] = useState<string>("");
  const [replayStatus, setReplayStatus] = useState<{
    playing: boolean;
    rate: number;
    t_ns: number;
    t_start_ns: number;
    t_end_ns: number;
  } | null>(null);
  const [series, setSeries] = useState<SeriesPayload | null>(null);
  const [plotWindows, setPlotWindows] = useState<PlotWindowState[]>([]);
  const plotZRef = useRef(1);

  const refreshCatalogs = useCallback(async () => {
    const [t, s] = await Promise.all([listTracks(), listScenarios()]);
    setTrackCatalog(t.tracks);
    setScenarioCatalog(s.scenarios);
    return { tracks: t.tracks, scenarios: s.scenarios };
  }, []);

  const scenariosForTrack = useMemo(
    () => scenarioCatalog.filter((s) => s.track_id === selectedTrackId),
    [scenarioCatalog, selectedTrackId]
  );

  const applyTrackSelection = useCallback(
    async (trackId: string, preferScenarioId?: string): Promise<TrackDef> => {
      editUndoRef.current = [];
      setSelectedTrackId(trackId);
      setErr(null);
      const t = await fetchTrack(trackId);
      const lanes = Math.max(1, t.num_lanes ?? 1);
      const lw = t.lane_width_m ?? (t.width_m ?? 12) / lanes;
      const refPts =
        t.reference_line_m && t.reference_line_m.length >= 3
          ? t.reference_line_m
          : t.centerline_m.map((p) => [...p]);
      const draft: TrackDef = {
        ...t,
        track_id: t.track_id,
        width_m: t.width_m ?? lanes * lw,
        num_lanes: lanes,
        lane_width_m: lw,
        reference_line_m: refPts,
        centerline_m: t.centerline_m,
        closed_loop: t.closed_loop ?? true,
        schema_version: t.schema_version || "1.0",
      };
      setTrackPoints(t.centerline_m);
      setReferencePoints(refPts);
      setTrackWidth(t.width_m ?? lanes * lw);
      setNumLanes(lanes);
      setLaneWidth(lw);
      setDraftTrack(draft);
      const listed = await listScenarios();
      setScenarioCatalog(listed.scenarios);
      const onTrack = listed.scenarios.filter((s) => s.track_id === trackId);
      const nextScenario =
        (preferScenarioId && onTrack.find((s) => s.id === preferScenarioId)?.id) ||
        onTrack[0]?.id ||
        "";
      setSelectedScenarioId(nextScenario);
      if (nextScenario) {
        const sc = await fetchScenario(nextScenario);
        setDraftScenario({ ...sc, npcs: sc.npcs || [], obstacles: sc.obstacles || [] });
      } else {
        setDraftScenario(null);
      }
      return draft;
    },
    []
  );

  const applyScenarioSelection = useCallback(async (scenarioId: string) => {
    setErr(null);
    const sc = await fetchScenario(scenarioId);
    if (sc.track_id !== selectedTrackId) {
      // Scenario belongs to another map — switch map first.
      await applyTrackSelection(sc.track_id, scenarioId);
      return;
    }
    editUndoRef.current = [];
    setSelectedScenarioId(scenarioId);
    setDraftScenario({ ...sc, npcs: sc.npcs || [], obstacles: sc.obstacles || [] });
  }, [applyTrackSelection, selectedTrackId]);

  useEffect(() => {
    refreshCatalogs()
      .then(() => applyTrackSelection("oval_01", "web_demo_001"))
      .catch((e) => setErr(String(e)));
  }, [refreshCatalogs, applyTrackSelection]);

  const refreshExperiments = useCallback(() => {
    listExperiments()
      .then((r) => {
        setExperiments(r.experiments.filter((e) => e.has_mcap));
      })
      .catch((e) => setErr(String(e)));
  }, []);

  useEffect(() => {
    if (mode === "replay" || mode === "logs") refreshExperiments();
  }, [mode, refreshExperiments]);

  // Live WS
  useEffect(() => {
    if (mode !== "live" || !session) return;
    const ws = openSessionSocket(session.session_id);
    ws.onmessage = (ev) => {
      const msg = JSON.parse(ev.data);
      if (msg.type === "simulation.snapshot") setSnapshot(msg as SnapshotMsg);
      if (msg.type === "session.status" || msg.type === "session.ping") {
        setSession((prev) =>
          prev
            ? {
                ...prev,
                state: msg.state ?? prev.state,
                sim_time_ns: msg.sim_time_ns ?? prev.sim_time_ns,
                step_index: msg.step_index ?? prev.step_index,
                controller: msg.controller ?? prev.controller,
                error: msg.error !== undefined ? msg.error : prev.error,
                plant_ic_applied:
                  msg.plant_ic_applied !== undefined
                    ? msg.plant_ic_applied
                    : prev.plant_ic_applied,
                plant_error:
                  msg.plant_error !== undefined ? msg.plant_error : prev.plant_error,
                plant_ic_par:
                  msg.plant_ic_par !== undefined ? msg.plant_ic_par : prev.plant_ic_par,
                longitudinal_mode:
                  msg.longitudinal_mode !== undefined
                    ? msg.longitudinal_mode
                    : prev.longitudinal_mode,
                plant_speed_mps:
                  msg.plant_speed_mps !== undefined
                    ? msg.plant_speed_mps
                    : prev.plant_speed_mps,
                has_longitudinal_control:
                  msg.has_longitudinal_control !== undefined
                    ? msg.has_longitudinal_control
                    : prev.has_longitudinal_control,
                deadline_miss_count:
                  msg.deadline_miss_count !== undefined
                    ? msg.deadline_miss_count
                    : prev.deadline_miss_count,
                last_solve_ms:
                  msg.last_solve_ms !== undefined ? msg.last_solve_ms : prev.last_solve_ms,
                deadline_budget_ms:
                  msg.deadline_budget_ms !== undefined
                    ? msg.deadline_budget_ms
                    : prev.deadline_budget_ms,
                max_sim_time_s:
                  msg.max_sim_time_s !== undefined
                    ? msg.max_sim_time_s
                    : prev.max_sim_time_s,
                duration_source:
                  msg.duration_source !== undefined
                    ? msg.duration_source
                    : prev.duration_source,
              }
            : prev
        );
      }
    };
    ws.onerror = () => setErr("Live WebSocket failed");
    return () => ws.close();
  }, [mode, session?.session_id]);

  // Replay WS + restore the map that was used when the experiment was recorded
  useEffect(() => {
    if (mode !== "replay" || !experimentId) return;
    let cancelled = false;
    const ws = openReplaySocket(experimentId);
    ws.onmessage = (ev) => {
      const msg = JSON.parse(ev.data);
      if (msg.type === "replay.snapshot") {
        setSnapshot(msg as SnapshotMsg);
        if (msg.replay) {
          setReplayStatus({
            playing: !!msg.replay.playing,
            rate: Number(msg.replay.rate ?? 1),
            t_ns: Number(msg.replay.t_ns ?? msg.sim_time_ns),
            t_start_ns: Number(msg.replay.t_start_ns),
            t_end_ns: Number(msg.replay.t_end_ns),
          });
        }
      }
      if (msg.type === "replay.status") {
        setReplayStatus({
          playing: !!msg.playing,
          rate: Number(msg.rate ?? 1),
          t_ns: Number(msg.t_ns),
          t_start_ns: Number(msg.t_start_ns),
          t_end_ns: Number(msg.t_end_ns),
        });
      }
    };
    ws.onerror = () => setErr("Replay WebSocket failed");

    void (async () => {
      try {
        const exp = await fetchExperiment(experimentId);
        if (cancelled) return;
        const meta = (exp.metadata || {}) as Record<string, unknown>;
        const scenarioMeta =
          meta.scenario && typeof meta.scenario === "object"
            ? (meta.scenario as Record<string, unknown>)
            : {};
        const trackId = String(meta.track_id || scenarioMeta.track_id || "");
        const scenarioId = String(meta.scenario_id || scenarioMeta.scenario_id || "");
        if (trackId) {
          await applyTrackSelection(trackId, scenarioId || undefined);
        }
      } catch (e) {
        if (!cancelled) setErr(String(e));
      }
    })();

    fetchSeries(experimentId)
      .then((s) => {
        if (cancelled) return;
        setSeries(s);
        const keys = Object.keys(s.series || {}).filter(
          (k) => k !== "t_s" && Array.isArray(s.series[k]) && (s.series[k]?.length ?? 0) > 0
        );
        const egoPreferred = ["vx_mps", "steering_cmd", "yaw_rate_radps", "ax_mps2"].filter((k) =>
          keys.includes(k)
        );
        const dynPreferred = [
          "u_tire",
          "abs_beta_deg",
          "beta_rad",
          "solve_ms",
          "deadline_miss",
        ].filter((k) => keys.includes(k));
        const egoCh = egoPreferred.length ? egoPreferred.slice(0, 2) : keys.slice(0, 2);
        const dynCh = dynPreferred.slice(0, 2);
        setPlotWindows((prev) => {
          if (prev.length === 0) {
            plotZRef.current = dynCh.length ? 2 : 1;
            const wins = [createPlotWindow({ channels: egoCh, z: 1 }, 0)];
            if (dynCh.length) {
              wins.push(
                createPlotWindow(
                  {
                    channels: dynCh,
                    z: 2,
                    x: 460,
                    y: 88,
                  },
                  0
                )
              );
            }
            return wins;
          }
          return prev.map((w) => ({
            ...w,
            channels: w.channels.filter((c) => keys.includes(c)).length
              ? w.channels.filter((c) => keys.includes(c))
              : egoCh,
          }));
        });
      })
      .catch((e) => {
        if (!cancelled) setErr(String(e));
      });
    return () => {
      cancelled = true;
      ws.close();
    };
  }, [mode, experimentId, applyTrackSelection]);

  const sessionLocked = session?.state === "RUNNING" || session?.state === "PAUSED";

  const clearSession = useCallback(() => {
    setSession(null);
    setSnapshot(null);
    setErr(null);
  }, []);

  const onCreate = useCallback(async () => {
    if (!selectedScenarioId) {
      setErr("No scenario on this map — create one in Edit first");
      return;
    }
    setBusy(true);
    setErr(null);
    try {
      // Drop previous finished/stopped session so plant can be changed.
      if (session && (session.state === "RUNNING" || session.state === "PAUSED")) {
        try {
          await sessionCommand(session.session_id, "stop");
        } catch {
          /* ignore */
        }
      }
      const s = await createSession({
        plant,
        controller,
        throttle_cmd: 0.35,
        track_id: selectedTrackId,
        scenario_id: selectedScenarioId,
        display_name: runName.trim() || undefined,
      });
      setSession(s);
      setSnapshot(null);
    } catch (e) {
      setErr(String(e));
    } finally {
      setBusy(false);
    }
  }, [plant, controller, session, selectedTrackId, selectedScenarioId, runName]);

  const onDraftTrack = useCallback((t: TrackDef) => {
    const lanes = Math.max(1, t.num_lanes ?? 1);
    const lw = t.lane_width_m ?? t.width_m / lanes;
    const refPts =
      t.reference_line_m && t.reference_line_m.length >= 3
        ? t.reference_line_m
        : t.centerline_m.map((p) => [...p]);
    setDraftTrack({
      ...t,
      num_lanes: lanes,
      lane_width_m: lw,
      width_m: lanes * lw,
      reference_line_m: refPts,
    });
    setTrackPoints(t.centerline_m);
    setReferencePoints(refPts);
    setTrackWidth(lanes * lw);
    setNumLanes(lanes);
    setLaneWidth(lw);
    setSelectedTrackId(t.track_id);
  }, []);

  const pushEditUndo = useCallback(() => {
    const snap = cloneEditSnap(draftTrackRef.current, draftScenarioRef.current);
    editUndoRef.current = [...editUndoRef.current, snap].slice(-40);
  }, []);

  const undoEdit = useCallback(() => {
    const stack = editUndoRef.current;
    if (!stack.length) return;
    const prev = stack[stack.length - 1];
    editUndoRef.current = stack.slice(0, -1);
    if (prev.track) onDraftTrack(prev.track);
    else {
      setDraftTrack(null);
      setTrackPoints([]);
      setReferencePoints([]);
    }
    setDraftScenario(prev.scenario);
  }, [onDraftTrack]);

  useEffect(() => {
    if (mode !== "edit") return;
    const onKey = (ev: KeyboardEvent) => {
      if (!(ev.ctrlKey || ev.metaKey) || ev.key.toLowerCase() !== "z" || ev.shiftKey) return;
      const el = ev.target as HTMLElement | null;
      const tag = el?.tagName?.toLowerCase();
      if (tag === "input" || tag === "textarea" || tag === "select" || el?.isContentEditable) {
        return;
      }
      ev.preventDefault();
      undoEdit();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [mode, undoEdit]);

  const openTrackInEdit = useCallback(
    async (id: string) => {
      setCtxMenu(null);
      setErr(null);
      try {
        await applyTrackSelection(id);
        setMode("edit");
      } catch (e) {
        setErr(String(e));
      }
    },
    [applyTrackSelection]
  );

  const openScenarioInEdit = useCallback(
    async (id: string) => {
      setCtxMenu(null);
      setErr(null);
      try {
        await applyScenarioSelection(id);
        setMode("edit");
      } catch (e) {
        setErr(String(e));
      }
    },
    [applyScenarioSelection]
  );

  const newTrackFromAssets = useCallback(() => {
    setCtxMenu(null);
    setErr(null);
    const base = `track_${Date.now().toString(36).slice(-6)}`;
    const id = window.prompt("New track id (alphanumeric / _ / -):", base);
    if (!id) return;
    if (!/^[a-zA-Z0-9_\-]+$/.test(id)) {
      setErr("Invalid track id");
      return;
    }
    const t = defaultRectangleTrack(id);
    onDraftTrack(t);
    setDraftScenario(defaultScenario(`${id}_demo`, id, t));
    setEditPhase("lanes");
    setMode("edit");
  }, [onDraftTrack]);

  const newScenarioFromAssets = useCallback(
    async (trackId: string) => {
      setCtxMenu(null);
      setErr(null);
      const base = `scenario_${Date.now().toString(36).slice(-6)}`;
      const id = window.prompt("New scenario id:", base);
      if (!id) return;
      if (!/^[a-zA-Z0-9_\-]+$/.test(id)) {
        setErr("Invalid scenario id");
        return;
      }
      try {
        // Prompt/validate first so Cancel does not clobber Live/Edit drafts.
        const t = await applyTrackSelection(trackId);
        setDraftScenario(defaultScenario(id, trackId, t));
        setMode("edit");
      } catch (e) {
        setErr(String(e));
      }
    },
    [applyTrackSelection]
  );

  const onCenterlineChange = useCallback(
    (pts: number[][]) => {
      if (!draftTrack) return;
      onDraftTrack({ ...draftTrack, centerline_m: pts });
    },
    [draftTrack, onDraftTrack]
  );

  const onReferenceChange = useCallback(
    (pts: number[][]) => {
      if (!draftTrack) return;
      onDraftTrack({ ...draftTrack, reference_line_m: pts });
    },
    [draftTrack, onDraftTrack]
  );

  const editPolyline = useCallback(
    (pts: number[][], x: number, y: number, tool: EditTool): number[][] | null => {
      if (tool === "delete") {
        if (pts.length <= 3) return null;
        let best = -1;
        let bestD = EDIT_POINT_HIT_M;
        for (let i = 0; i < pts.length; i++) {
          const d = Math.hypot(pts[i][0] - x, pts[i][1] - y);
          if (d < bestD) {
            bestD = d;
            best = i;
          }
        }
        if (best < 0) return null;
        const next = pts.filter((_, i) => i !== best);
        return next.length < 3 ? null : next;
      }
      if (tool !== "add") return null;
      let bestI = 0;
      let bestD = Infinity;
      const nSeg = Math.max(1, pts.length - 1);
      for (let i = 0; i < nSeg; i++) {
        const a = pts[i];
        const b = pts[(i + 1) % pts.length];
        const mx = (a[0] + b[0]) * 0.5;
        const my = (a[1] + b[1]) * 0.5;
        const d = Math.hypot(mx - x, my - y);
        if (d < bestD) {
          bestD = d;
          bestI = i;
        }
      }
      return [...pts.slice(0, bestI + 1), [x, y, 0], ...pts.slice(bestI + 1)];
    },
    []
  );

  const onWorldClick = useCallback(
    (x: number, y: number, tool: EditTool) => {
      if (!draftTrack) return;
      if (tool === "place_ego") return;
      if (editPhase === "lanes") {
        const next = editPolyline(draftTrack.centerline_m, x, y, tool);
        if (next) {
          pushEditUndo();
          onDraftTrack({ ...draftTrack, centerline_m: next });
        }
        return;
      }
      if (editPhase === "reference") {
        const cur =
          draftTrack.reference_line_m && draftTrack.reference_line_m.length >= 3
            ? draftTrack.reference_line_m
            : draftTrack.centerline_m;
        const next = editPolyline(cur, x, y, tool);
        if (next) {
          pushEditUndo();
          onDraftTrack({ ...draftTrack, reference_line_m: next });
        }
      }
    },
    [draftTrack, onDraftTrack, editPhase, editPolyline, pushEditUndo]
  );

  const onEgoIcPlace = useCallback(
    (x: number, y: number, yaw_rad: number) => {
      if (!draftScenario) return;
      setDraftScenario({
        ...draftScenario,
        ego: {
          ...draftScenario.ego,
          initial_pose: {
            ...draftScenario.ego.initial_pose,
            x_m: x,
            y_m: y,
            yaw_rad,
          },
        },
      });
    },
    [draftScenario]
  );

  const closeCtxMenu = useCallback(() => setCtxMenu(null), []);

  const onContextMenuRequest = useCallback((req: ContextMenuRequest) => {
    setCtxMenu(req);
  }, []);

  const ctxAddPoint = useCallback(() => {
    if (!ctxMenu || !draftTrack) return;
    onWorldClick(ctxMenu.worldX, ctxMenu.worldY, "add");
    setCtxMenu(null);
  }, [ctxMenu, draftTrack, onWorldClick]);

  const ctxAddNpc = useCallback(() => {
    if (!ctxMenu || !draftTrack || !draftScenario) return;
    const { worldX: x, worldY: y } = ctxMenu;
    const laneIndex = nearestLaneIndex(x, y, draftTrack);
    const id = `npc_${Date.now().toString(36).slice(-5)}`;
    pushEditUndo();
    setDraftScenario({
      ...draftScenario,
      npcs: [
        ...draftScenario.npcs,
        {
          id,
          model: "trajectory",
          strategy: "trajectory",
          // Single marker for Edit placement; runtime still uses lane when < 2 pts.
          trajectory_xy: [[x, y]],
          speed_mps: 6,
          loop: true,
          length_m: 4,
          width_m: 2,
          lane_index: laneIndex,
        },
      ],
    });
    setCtxMenu(null);
  }, [ctxMenu, draftTrack, draftScenario, pushEditUndo]);

  const ctxAddObstacle = useCallback(() => {
    if (!ctxMenu || !draftScenario) return;
    const id = `obs_${Date.now().toString(36).slice(-5)}`;
    pushEditUndo();
    setDraftScenario({
      ...draftScenario,
      obstacles: [
        ...draftScenario.obstacles,
        {
          id,
          x_m: ctxMenu.worldX,
          y_m: ctxMenu.worldY,
          yaw_rad: 0,
          length_m: 1.5,
          width_m: 1.5,
        },
      ],
    });
    setCtxMenu(null);
  }, [ctxMenu, draftScenario, pushEditUndo]);

  const ctxDeletePoint = useCallback(() => {
    if (!ctxMenu || !draftTrack || ctxMenu.hit.kind !== "point") return;
    const { index, phase } = ctxMenu.hit;
    if (phase === "lanes") {
      const pts = draftTrack.centerline_m;
      if (pts.length <= 3) {
        setCtxMenu(null);
        return;
      }
      const next = pts.filter((_, i) => i !== index);
      if (next.length >= 3) {
        pushEditUndo();
        onDraftTrack({ ...draftTrack, centerline_m: next });
      }
    } else {
      const pts =
        draftTrack.reference_line_m && draftTrack.reference_line_m.length >= 3
          ? draftTrack.reference_line_m
          : draftTrack.centerline_m;
      if (pts.length <= 3) {
        setCtxMenu(null);
        return;
      }
      const next = pts.filter((_, i) => i !== index);
      if (next.length >= 3) {
        pushEditUndo();
        onDraftTrack({ ...draftTrack, reference_line_m: next });
      }
    }
    setCtxMenu(null);
  }, [ctxMenu, draftTrack, onDraftTrack, pushEditUndo]);

  const ctxDeleteNpc = useCallback(() => {
    if (!ctxMenu || !draftScenario || ctxMenu.hit.kind !== "npc") return;
    const id = ctxMenu.hit.id;
    pushEditUndo();
    setDraftScenario({
      ...draftScenario,
      npcs: draftScenario.npcs.filter((n) => n.id !== id),
    });
    setCtxMenu(null);
  }, [ctxMenu, draftScenario, pushEditUndo]);

  const ctxDeleteObstacle = useCallback(() => {
    if (!ctxMenu || !draftScenario || ctxMenu.hit.kind !== "obstacle") return;
    const id = ctxMenu.hit.id;
    pushEditUndo();
    setDraftScenario({
      ...draftScenario,
      obstacles: draftScenario.obstacles.filter((o) => o.id !== id),
    });
    setCtxMenu(null);
  }, [ctxMenu, draftScenario, pushEditUndo]);

  const runFromEditor = useCallback(
    async (trackId: string, scenarioId: string) => {
      setMode("live");
      setBusy(true);
      setErr(null);
      try {
        // Always trust scenario → track binding from server.
        const sc = await fetchScenario(scenarioId);
        const tid = sc.track_id || trackId;
        await applyTrackSelection(tid, scenarioId);
        const ctrl =
          (sc.controller?.type as
            | "pure_pursuit"
            | "lane_pp"
            | "lattice_pid"
            | "lattice_pid_ml"
            | "constant"
            | "mppi"
            | "strategy01"
            | undefined) || controller;
        setController(ctrl);
        const s = await createSession({
          plant,
          controller: ctrl,
          throttle_cmd: 0.35,
          track_id: tid,
          scenario_id: scenarioId,
          display_name: runName.trim() || undefined,
        });
        setSession(s);
        setSnapshot(null);
      } catch (e) {
        setErr(String(e));
      } finally {
        setBusy(false);
      }
    },
    [plant, controller, applyTrackSelection, runName]
  );

  const draftNpcs = useMemo(
    () =>
      (draftScenario?.npcs || []).map((n) => {
        // Prefer placement marker (≥1 pt); empty → fall back near ego / origin for legacy demos.
        const p =
          n.trajectory_xy[0] ||
          (draftScenario?.ego?.initial_pose
            ? [draftScenario.ego.initial_pose.x_m, draftScenario.ego.initial_pose.y_m]
            : [0, 0]);
        return {
          id: n.id,
          x_m: p[0],
          y_m: p[1],
          yaw_rad: npcPreviewYaw(p[0], p[1], draftTrack, n.lane_index),
          length_m: n.length_m ?? 4,
          width_m: n.width_m ?? 2,
        };
      }),
    [draftScenario, draftTrack]
  );
  const draftObstacles = useMemo(
    () =>
      (draftScenario?.obstacles || []).map((o) => ({
        id: o.id,
        x_m: o.x_m,
        y_m: o.y_m,
        yaw_rad: o.yaw_rad ?? 0,
        length_m: o.length_m,
        width_m: o.width_m,
      })),
    [draftScenario]
  );

  const onNpcMove = useCallback(
    (id: string, x: number, y: number) => {
      if (!draftScenario) return;
      const laneIndex = draftTrack ? nearestLaneIndex(x, y, draftTrack) : undefined;
      const npcs = draftScenario.npcs.map((n) => {
        if (n.id !== id) return n;
        // Empty or single placement marker: move marker + re-bind lane for travel direction.
        if (n.trajectory_xy.length <= 1) {
          return {
            ...n,
            trajectory_xy: [[x, y]],
            lane_index: laneIndex ?? n.lane_index ?? 0,
          };
        }
        const origin = n.trajectory_xy[0];
        const dx = x - origin[0];
        const dy = y - origin[1];
        return {
          ...n,
          trajectory_xy: n.trajectory_xy.map((p) => [p[0] + dx, p[1] + dy]),
          lane_index: laneIndex ?? n.lane_index ?? 0,
        };
      });
      setDraftScenario({ ...draftScenario, npcs });
    },
    [draftScenario, draftTrack]
  );

  const onObstacleMove = useCallback(
    (id: string, x: number, y: number) => {
      if (!draftScenario) return;
      setDraftScenario({
        ...draftScenario,
        obstacles: draftScenario.obstacles.map((o) =>
          o.id === id ? { ...o, x_m: x, y_m: y } : o
        ),
      });
    },
    [draftScenario]
  );

  const runCmd = useCallback(
    async (cmd: "start" | "pause" | "step" | "reset" | "stop") => {
      if (!session) return;
      setBusy(true);
      setErr(null);
      try {
        const res = await sessionCommand(session.session_id, cmd);
        if (!res.accepted) setErr(res.error || "Command rejected");
        if (res.state) {
          setSession((s) =>
            s
              ? {
                  ...s,
                  state: res.state!,
                  // Reset rebuilds the run — clear stale HUD time until ping/snapshot arrive
                  ...(cmd === "reset" ? { sim_time_ns: 0, step_index: 0 } : {}),
                }
              : s
          );
        }
        if (cmd === "reset") {
          // Optimistic clear of trail overlay; WS snapshot will follow with IC pose
          setSnapshot((prev) =>
            prev
              ? {
                  ...prev,
                  sim_time_ns: 0,
                  step_index: 0,
                  trail_xy: [],
                  metrics: { ...prev.metrics, trail_xy: [], ctrl_diag: {} },
                }
              : prev
          );
        }
        if (cmd === "stop") refreshExperiments();
      } catch (e) {
        setErr(String(e));
      } finally {
        setBusy(false);
      }
    },
    [session, refreshExperiments]
  );

  const onSeek = async (t_ns: number) => {
    if (!experimentId) return;
    const res = await replayCommand(experimentId, { action: "seek", t_ns });
    if (res.snapshot) setSnapshot(res.snapshot);
  };

  const ego = snapshot?.vehicles[0];
  const timeS = useMemo(() => {
    // Live: prefer snapshot clock (publish rate); session.ping is ~1 Hz and looks frozen.
    const ns =
      mode === "replay"
        ? replayStatus?.t_ns ?? snapshot?.sim_time_ns ?? 0
        : snapshot?.sim_time_ns ?? session?.sim_time_ns ?? 0;
    return (ns * 1e-9).toFixed(1);
  }, [mode, replayStatus, session, snapshot]);

  const progress = useMemo(() => {
    if (!replayStatus || replayStatus.t_end_ns === replayStatus.t_start_ns) return 0;
    return (
      (replayStatus.t_ns - replayStatus.t_start_ns) /
      (replayStatus.t_end_ns - replayStatus.t_start_ns)
    );
  }, [replayStatus]);

  const selectedExp = experiments.find((e) => e.id === experimentId);

  const curveChannelOptions = useMemo(() => {
    if (!series?.series) return [] as string[];
    return Object.keys(series.series)
      .filter(
        (k) =>
          k !== "t_s" &&
          Array.isArray(series.series[k]) &&
          (series.series[k]?.length ?? 0) > 0
      )
      .sort((a, b) => a.localeCompare(b));
  }, [series]);

  const addPlotWindow = useCallback(() => {
    plotZRef.current += 1;
    const n = plotWindows.length;
    const preferred = ["vx_mps", "steering_cmd"].filter((k) =>
      curveChannelOptions.includes(k)
    );
    setPlotWindows((prev) => [
      ...prev,
      createPlotWindow(
        {
          channels: preferred.length ? preferred : curveChannelOptions.slice(0, 2),
          z: plotZRef.current,
        },
        n
      ),
    ]);
  }, [plotWindows.length, curveChannelOptions]);

  const focusPlotWindow = useCallback((id: string) => {
    setPlotWindows((prev) => {
      const maxZ = prev.reduce((m, w) => Math.max(m, w.z), 0);
      const cur = prev.find((w) => w.id === id);
      if (cur && cur.z >= maxZ) return prev;
      plotZRef.current = maxZ + 1;
      const z = plotZRef.current;
      return prev.map((w) => (w.id === id ? { ...w, z } : w));
    });
  }, []);

  const replayCursorT = useMemo(
    () =>
      replayStatus
        ? (replayStatus.t_ns - replayStatus.t_start_ns) * 1e-9
        : undefined,
    [replayStatus]
  );

  return (
    <div className="app-shell">
      <Viewport
        trackPoints={trackPoints}
        closedLoop={draftTrack?.closed_loop !== false}
        trackWidth={trackWidth}
        numLanes={numLanes}
        laneWidth={laneWidth}
        referencePoints={referencePoints.length ? referencePoints : trackPoints}
        showReference={mode !== "edit" || editPhase === "reference"}
        showSpine={true}
        frictionZones={
          draftTrack?.friction_zones?.length
            ? draftTrack.friction_zones
            : undefined
        }
        snapshot={mode === "edit" ? null : snapshot}
        cameraMode={cameraMode}
        editMode={mode === "edit"}
        editPhase={editPhase}
        editTool={editTool}
        draftNpcs={mode === "edit" ? draftNpcs : undefined}
        draftObstacles={mode === "edit" ? draftObstacles : undefined}
        onCenterlineChange={mode === "edit" && editPhase === "lanes" ? onCenterlineChange : undefined}
        onReferenceChange={
          mode === "edit" && editPhase === "reference" ? onReferenceChange : undefined
        }
        onNpcMove={mode === "edit" ? onNpcMove : undefined}
        onObstacleMove={mode === "edit" ? onObstacleMove : undefined}
        onWorldClick={mode === "edit" ? onWorldClick : undefined}
        onContextMenuRequest={mode === "edit" ? onContextMenuRequest : undefined}
        onEditDragBegin={mode === "edit" ? pushEditUndo : undefined}
        egoIcPose={
          mode === "edit" && draftScenario
            ? {
                x_m: draftScenario.ego.initial_pose.x_m,
                y_m: draftScenario.ego.initial_pose.y_m,
                yaw_rad: draftScenario.ego.initial_pose.yaw_rad,
              }
            : null
        }
        onEgoIcPlace={mode === "edit" ? onEgoIcPlace : undefined}
        onEditToolChange={mode === "edit" ? setEditTool : undefined}
      />

      {mode === "edit" && ctxMenu && (
        <>
          <div className="ctx-menu-backdrop" onClick={closeCtxMenu} onContextMenu={(e) => e.preventDefault()} />
          <div
            className="ctx-menu"
            style={{ left: ctxMenu.clientX, top: ctxMenu.clientY }}
            onContextMenu={(e) => e.preventDefault()}
          >
            {ctxMenu.hit.kind === "empty" && (
              <>
                <button type="button" onClick={ctxAddPoint}>
                  Add point
                </button>
                <button type="button" onClick={ctxAddNpc} disabled={!draftScenario}>
                  Add NPC
                </button>
                <button type="button" onClick={ctxAddObstacle} disabled={!draftScenario}>
                  Add obstacle
                </button>
              </>
            )}
            {ctxMenu.hit.kind === "point" && (
              <button type="button" onClick={ctxDeletePoint}>
                Delete point
              </button>
            )}
            {ctxMenu.hit.kind === "npc" && (
              <button type="button" onClick={ctxDeleteNpc}>
                Delete NPC
              </button>
            )}
            {ctxMenu.hit.kind === "obstacle" && (
              <button type="button" onClick={ctxDeleteObstacle}>
                Delete obstacle
              </button>
            )}
            <button type="button" className="ctx-menu-cancel" onClick={closeCtxMenu}>
              Cancel
            </button>
          </div>
        </>
      )}

      <div className="hud-top">
        <div className="brand">
          Dyn<span>Limit</span>
        </div>
        <div className="mode-switch">
          <button
            type="button"
            className={mode === "live" ? "active" : ""}
            onClick={() => {
              setCtxMenu(null);
              setMode("live");
            }}
          >
            Live
          </button>
          <button
            type="button"
            className={mode === "replay" ? "active" : ""}
            onClick={() => {
              setCtxMenu(null);
              setMode("replay");
            }}
          >
            Replay
          </button>
          <button
            type="button"
            className={mode === "edit" ? "active" : ""}
            onClick={() => {
              setCtxMenu(null);
              setMode("edit");
            }}
          >
            Edit
          </button>
          <button
            type="button"
            className={mode === "assets" ? "active" : ""}
            onClick={() => {
              setCtxMenu(null);
              setMode("assets");
            }}
          >
            Assets
          </button>
          <button
            type="button"
            className={mode === "logs" ? "active" : ""}
            onClick={() => {
              setCtxMenu(null);
              setMode("logs");
            }}
          >
            Logs
          </button>
        </div>
        <div className="status-chip">
          {mode === "live" ? (
            session ? (
              <>
                <b>{session.state}</b>
                <span>{session.plant}</span>
                <span>{session.controller || "—"}</span>
                <span
                  title={
                    session.max_sim_time_s != null
                      ? `Sim time / limit (${session.duration_source === "carsim_tstop" ? "CarSim TSTOP" : "scenario"})`
                      : "Sim time"
                  }
                >
                  t={timeS}
                  {session.max_sim_time_s != null
                    ? ` / ${Number(session.max_sim_time_s).toFixed(0)}s`
                    : "s"}
                </span>
                {session.plant === "carsim" && session.plant_ic_applied === false && (
                  <span className="chip-warn" title={session.plant_error || undefined}>
                    IC=procedure
                  </span>
                )}
                {session.plant === "carsim" &&
                  session.plant_ic_applied &&
                  session.plant_ic_par?.station_m != null && (
                    <span
                      className="chip-ok"
                      title={`par: ${(session.plant_ic_par.fields || []).join(", ")}`}
                    >
                      s={Number(session.plant_ic_par.station_m).toFixed(1)}m
                      {session.plant_ic_par.speed_mps != null
                        ? ` v=${Number(session.plant_ic_par.speed_mps).toFixed(1)}`
                        : ""}
                    </span>
                  )}
                {session.plant === "carsim" && session.longitudinal_mode === "plant" && (
                  <span
                    className="chip-warn"
                    title="No throttle/motor import — CarSim owns speed; platform steers only"
                  >
                    lat-only
                    {session.plant_speed_mps != null
                      ? ` ${session.plant_speed_mps.toFixed(1)}m/s`
                      : ""}
                  </span>
                )}
                {session.plant === "carsim" && session.longitudinal_mode === "control" && (
                  <span className="chip-ok" title="Throttle/motor imports enabled">
                    lon+lat
                  </span>
                )}
                {(session.deadline_miss_count ?? 0) > 0 && (
                  <span
                    className="chip-warn"
                    title={`Control solve exceeded ${session.deadline_budget_ms ?? 20} ms budget (last ${session.last_solve_ms ?? "?"} ms)`}
                  >
                    miss {session.deadline_miss_count}
                  </span>
                )}
                {session.error && (
                  <span className="chip-err" title={session.error}>
                    error
                  </span>
                )}
              </>
            ) : (
              <span>no live session</span>
            )
          ) : mode === "edit" ? (
            <span>
              {draftTrack?.track_id || "—"} / {draftScenario?.scenario_id || "—"}
            </span>
          ) : mode === "assets" ? (
            <span>tracks & scenarios</span>
          ) : mode === "logs" ? (
            <span>experiment logs</span>
          ) : experimentId ? (
            <>
              <b>{replayStatus?.playing ? "PLAYING" : "PAUSED"}</b>
              <span>
                {(selectedExp?.display_name?.trim() || experimentId).slice(0, 24)}
              </span>
              <span>t={timeS}s</span>
            </>
          ) : (
            <span>select experiment</span>
          )}
        </div>
      </div>

      {mode === "logs" && (
        <LogsPanel
          experiments={experiments}
          onExperimentsChange={setExperiments}
          onOpenReplay={(id) => {
            setExperimentId(id);
            setMode("replay");
          }}
        />
      )}

      {mode === "edit" && (
        <EditorPanel
          track={draftTrack}
          scenario={draftScenario}
          editPhase={editPhase}
          onEditPhaseChange={setEditPhase}
          editTool={editTool}
          onEditToolChange={setEditTool}
          onTrackChange={onDraftTrack}
          onScenarioChange={setDraftScenario}
          onRunLive={runFromEditor}
          onError={setErr}
        />
      )}

      {mode === "assets" && (
        <AssetsPanel
          busy={busy}
          onOpenTrack={(id) => void openTrackInEdit(id)}
          onOpenScenario={(id) => void openScenarioInEdit(id)}
          onNewTrack={newTrackFromAssets}
          onNewScenario={(trackId) => void newScenarioFromAssets(trackId)}
          onAssetsChanged={() => {
            void refreshCatalogs().catch((e) => setErr(String(e)));
          }}
          onError={setErr}
        />
      )}

      {mode !== "edit" && mode !== "assets" && mode !== "logs" && (
      <div className="cam-bar">
        {(
          [
            ["orbit", "Orbit"],
            ["top", "Top"],
            ["chase", "Chase"],
            ["overview", "Overview"],
          ] as const
        ).map(([id, label]) => (
          <button
            key={id}
            type="button"
            className={cameraMode === id ? "active" : ""}
            onClick={() => setCameraMode(id)}
          >
            {label}
          </button>
        ))}
      </div>
      )}

      {mode !== "edit" && mode !== "assets" && mode !== "logs" && (
      <aside className="hud-right">
        <h2>Telemetry</h2>
        <div className="metric">
          <span>time</span>
          <span>
            {timeS}s
            {session?.max_sim_time_s != null
              ? ` / ${Number(session.max_sim_time_s).toFixed(0)}s`
              : ""}
          </span>
        </div>
        <div className="metric">
          <span>vx</span>
          <span>{ego ? `${ego.vx_mps.toFixed(1)} m/s` : "—"}</span>
        </div>
        <div className="metric">
          <span>yaw rate</span>
          <span>{ego ? ego.yaw_rate_radps.toFixed(3) : "—"}</span>
        </div>
        <div className="metric">
          <span>steer</span>
          <span>
            {snapshot?.control?.steering_cmd != null
              ? Number(snapshot.control.steering_cmd).toFixed(3)
              : "—"}
          </span>
        </div>
        <div className="metric">
          <span>x / y</span>
          <span>{ego ? `${ego.x_m.toFixed(1)}, ${ego.y_m.toFixed(1)}` : "—"}</span>
        </div>
        <div className="metric">
          <span>μ</span>
          <span>
            {snapshot?.metrics?.friction_mu != null
              ? Number(snapshot.metrics.friction_mu).toFixed(2)
              : "—"}
            {snapshot?.metrics?.friction_zone_id
              ? ` (${String(snapshot.metrics.friction_zone_id)})`
              : ""}
          </span>
        </div>
        <div className="metric" title="Sideslip β = atan2(vy, vx)">
          <span>β</span>
          <span>
            {snapshot?.metrics?.beta_rad != null
              ? `${(Number(snapshot.metrics.beta_rad) * (180 / Math.PI)).toFixed(2)}°`
              : "—"}
          </span>
        </div>
        <div className="metric" title="Tire utilization U_tire (max over wheels)">
          <span>U_tire</span>
          <span>
            {snapshot?.metrics?.u_tire != null
              ? Number(snapshot.metrics.u_tire).toFixed(3)
              : "—"}
          </span>
        </div>
        {(() => {
          const diag = snapshot?.metrics?.ctrl_diag as
            | {
                solve_ms?: number;
                rollout_backend?: string;
                best_cost?: number;
                deadline_miss?: boolean;
                deadline_miss_count?: number;
              }
            | undefined;
          if (!diag || Object.keys(diag).length === 0) return null;
          return (
            <>
              {diag.rollout_backend != null && (
                <div className="metric">
                  <span>rollout</span>
                  <span>{diag.rollout_backend}</span>
                </div>
              )}
              {diag.solve_ms != null && (
                <div className="metric">
                  <span>solve ms</span>
                  <span>{Number(diag.solve_ms).toFixed(1)}</span>
                </div>
              )}
              {diag.deadline_miss != null && (
                <div className="metric">
                  <span>deadline</span>
                  <span>
                    {diag.deadline_miss ? "MISS" : "ok"}
                    {diag.deadline_miss_count != null
                      ? ` · ${diag.deadline_miss_count}`
                      : ""}
                  </span>
                </div>
              )}
            </>
          );
        })()}

        {mode === "replay" && series?.summary && (
          <>
            <h2 title="Run-level dynamics-aware diagnostics">Diag summary</h2>
            {series.summary.U_tire_max != null && (
              <div className="metric" title="max_t U_tire">
                <span>U max</span>
                <span>{Number(series.summary.U_tire_max).toFixed(3)}</span>
              </div>
            )}
            {series.summary.U_tire_p95 != null && (
              <div className="metric" title="P95 of U_tire">
                <span>U p95</span>
                <span>{Number(series.summary.U_tire_p95).toFixed(3)}</span>
              </div>
            )}
            {series.summary.U_tire_frac_above_thresh != null && (
              <div
                className="metric"
                title={`Fraction of time U_tire > ${series.summary.u_tire_thresh ?? 0.9}`}
              >
                <span>U&gt;thr</span>
                <span>
                  {(Number(series.summary.U_tire_frac_above_thresh) * 100).toFixed(1)}%
                </span>
              </div>
            )}
            {series.summary.max_abs_beta_deg != null && (
              <div className="metric" title="max |β|">
                <span>|β| max</span>
                <span>{Number(series.summary.max_abs_beta_deg).toFixed(2)}°</span>
              </div>
            )}
            {series.summary.p95_abs_beta_deg != null && (
              <div className="metric" title="P95 |β|">
                <span>|β| p95</span>
                <span>{Number(series.summary.p95_abs_beta_deg).toFixed(2)}°</span>
              </div>
            )}
            {series.summary.solve_ms_p95 != null && (
              <div className="metric">
                <span>solve p95</span>
                <span>{Number(series.summary.solve_ms_p95).toFixed(2)} ms</span>
              </div>
            )}
            {series.summary.deadline_miss_rate != null && (
              <div className="metric">
                <span>P_miss</span>
                <span>
                  {(Number(series.summary.deadline_miss_rate) * 100).toFixed(1)}%
                </span>
              </div>
            )}
            {series.summary.flags && (
              <div className="diag-flags" title="Layered success annotations (not a scoreboard)">
                <span
                  className={
                    series.summary.flags.task_only_success ? "flag-ok" : "flag-bad"
                  }
                >
                  task
                </span>
                <span
                  className={
                    series.summary.flags.physics_safe_success ? "flag-ok" : "flag-bad"
                  }
                >
                  physics
                </span>
                <span
                  className={
                    series.summary.flags.deadline_feasible_success
                      ? "flag-ok"
                      : "flag-bad"
                  }
                >
                  deadline
                </span>
              </div>
            )}
          </>
        )}

      </aside>
      )}

      {mode === "replay" && series &&
        plotWindows.map((win) => (
          <PlotWindow
            key={win.id}
            win={win}
            series={series}
            channelOptions={curveChannelOptions}
            cursorT={replayCursorT}
            onChange={(next) =>
              setPlotWindows((prev) => prev.map((w) => (w.id === next.id ? next : w)))
            }
            onClose={() => setPlotWindows((prev) => prev.filter((w) => w.id !== win.id))}
            onFocus={() => focusPlotWindow(win.id)}
          />
        ))}

      {mode === "live" ? (
        <div className="hud-bottom">
          <select
            value={plant}
            onChange={(e) => setPlant(e.target.value as "mock" | "carsim")}
            disabled={sessionLocked || busy}
            title={
              sessionLocked
                ? "Stop the current session before switching Mock / CarSim"
                : "Dynamics backend"
            }
          >
            <option value="mock">Mock plant</option>
            <option value="carsim">CarSim plant</option>
          </select>
          <select
            value={controller}
            disabled={sessionLocked || busy}
            onChange={(e) =>
              setController(
                e.target.value as
                  | "pure_pursuit"
                  | "lane_pp"
                  | "lattice_pid"
                  | "lattice_pid_ml"
                  | "constant"
                  | "mppi"
                  | "strategy01"
              )
            }
            title="Control / planning strategy"
          >
            <option value="pure_pursuit">Pure Pursuit</option>
            <option value="lane_pp">Lane PP (multi-lane)</option>
            <option value="lattice_pid">Lattice PID</option>
            <option value="lattice_pid_ml">Lattice PID Multi-lane</option>
            <option value="mppi">MPPI</option>
            <option value="strategy01">strategy01 (legacy)</option>
            <option value="constant">Constant</option>
          </select>
          <select
            value={selectedTrackId}
            disabled={sessionLocked || busy}
            onChange={(e) => {
              void applyTrackSelection(e.target.value).catch((err) => setErr(String(err)));
            }}
            title="Select map (Track)"
          >
            {trackCatalog.map((t) => (
              <option key={t.id} value={t.id}>
                map: {t.id}
              </option>
            ))}
          </select>
          <select
            value={selectedScenarioId}
            disabled={sessionLocked || busy || scenariosForTrack.length === 0}
            onChange={(e) => {
              void applyScenarioSelection(e.target.value).catch((err) => setErr(String(err)));
            }}
            title="Select scenario on this map"
          >
            {scenariosForTrack.length === 0 ? (
              <option value="">No scenarios on this map</option>
            ) : (
              scenariosForTrack.map((s) => (
                <option key={s.id} value={s.id}>
                  scene: {s.id}
                </option>
              ))
            )}
          </select>
          <input
            className="run-name"
            type="text"
            value={runName}
            maxLength={80}
            placeholder="Run name (optional)"
            disabled={sessionLocked || busy}
            onChange={(e) => setRunName(e.target.value)}
            title="Stored as metadata.display_name; optional"
          />
          <button
            className="primary"
            disabled={busy || sessionLocked || !selectedScenarioId}
            onClick={onCreate}
            title="Create session from map + scenario"
          >
            Create
          </button>
          <button
            disabled={
              busy ||
              !session ||
              !!session.error ||
              session.state === "RUNNING" ||
              session.state === "FINISHED"
            }
            onClick={() => runCmd("start")}
            title={session?.error ? session.error : "Start"}
          >
            Start
          </button>
          <button disabled={busy || !session || session.state !== "RUNNING"} onClick={() => runCmd("pause")}>
            Pause
          </button>
          <button disabled={busy || !session || session.state !== "PAUSED"} onClick={() => runCmd("step")}>
            Step
          </button>
          <button disabled={busy || !session} onClick={() => runCmd("reset")}>
            Reset
          </button>
          <button disabled={busy || !session || session.state === "STOPPED"} onClick={() => runCmd("stop")}>
            Stop
          </button>
          <button disabled={busy || !session} onClick={clearSession} title="Clear local session so you can re-pick plant">
            New
          </button>
        </div>
      ) : mode === "replay" ? (
        <div className="hud-bottom replay-bar">
          <select
            value={experimentId}
            onChange={(e) => {
              setExperimentId(e.target.value);
              setSnapshot(null);
              setSeries(null);
            }}
          >
            <option value="">Select experiment…</option>
            {experiments.map((e) => {
              const reason = e.metrics?.finished_reason;
              const base = e.display_name?.trim() || e.id.slice(0, 18);
              const label = reason
                ? `${base} (${e.duration_s.toFixed(1)}s · ${String(reason)})`
                : `${base} (${e.duration_s.toFixed(1)}s)`;
              return (
                <option key={e.id} value={e.id}>
                  {label}
                </option>
              );
            })}
          </select>
          <button type="button" onClick={refreshExperiments}>
            Refresh
          </button>
          <button
            className="primary"
            disabled={!experimentId}
            title={replayStatus?.playing ? "Pause" : "Play"}
            onClick={() =>
              replayCommand(experimentId, {
                action: replayStatus?.playing ? "pause" : "play",
              })
            }
          >
            {replayStatus?.playing ? "Pause" : "Play"}
          </button>
          <button
            disabled={!experimentId}
            onClick={() => replayCommand(experimentId, { action: "rate", rate: 1 })}
          >
            1×
          </button>
          <button
            disabled={!experimentId}
            onClick={() => replayCommand(experimentId, { action: "rate", rate: 2 })}
          >
            2×
          </button>
          <button
            type="button"
            disabled={!experimentId || !series}
            onClick={addPlotWindow}
            title="Add floating plot window"
          >
            + Plot
          </button>
          <input
            className="timeline"
            type="range"
            min={0}
            max={1000}
            value={Math.round(progress * 1000)}
            disabled={!replayStatus}
            onChange={(e) => {
              if (!replayStatus) return;
              const u = Number(e.target.value) / 1000;
              const t =
                replayStatus.t_start_ns +
                u * (replayStatus.t_end_ns - replayStatus.t_start_ns);
              void onSeek(Math.round(t));
            }}
          />
          {selectedExp?.metrics && (
            <span className="metrics-chip" title={JSON.stringify(selectedExp.metrics)}>
              max_vx={Number(selectedExp.metrics.max_vx ?? 0).toFixed(1)} · lat=
              {Number(selectedExp.metrics.mean_abs_lat_m ?? 0).toFixed(2)}m
              {selectedExp.metrics.collision ? " · COLLISION" : ""}
              {selectedExp.metrics.off_track ? " · OFF_TRACK" : ""}
            </span>
          )}
        </div>
      ) : null}

      {err && <div className="err-banner">{err}</div>}
    </div>
  );
}
