import { useCallback, useEffect, useState } from "react";
import {
  createScenario,
  createTrack,
  exportTrack,
  fetchScenario,
  fetchTrack,
  generateTrack,
  listScenarios,
  listTracks,
  saveScenario,
  saveTrack,
  type ScenarioDef,
  type TrackDef,
} from "./api";
import { referencePolyline, snapToPolyline } from "./geo";
import type { EditPhase, EditTool } from "./scene/Viewport";

function defaultRectangleTrack(id: string): TrackDef {
  // Simple closed rectangle — edit handles in viewport afterwards.
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

function syncLaneWidth(t: TrackDef, numLanes: number, laneWidth: number): TrackDef {
  const n = Math.max(1, Math.min(8, Math.round(numLanes)));
  const lw = Math.max(0.5, laneWidth);
  return { ...t, num_lanes: n, lane_width_m: lw, width_m: n * lw };
}

/** Ego IC from driving reference (or road spine) — not the oval_01 template (5,0). */
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

type Props = {
  track: TrackDef | null;
  scenario: ScenarioDef | null;
  editPhase: EditPhase;
  onEditPhaseChange: (p: EditPhase) => void;
  editTool: EditTool;
  onEditToolChange: (t: EditTool) => void;
  onTrackChange: (t: TrackDef) => void;
  onScenarioChange: (s: ScenarioDef) => void;
  onRunLive: (trackId: string, scenarioId: string) => void;
  onError: (msg: string | null) => void;
};

export function EditorPanel({
  track,
  scenario,
  editPhase,
  onEditPhaseChange,
  editTool,
  onEditToolChange,
  onTrackChange,
  onScenarioChange,
  onRunLive,
  onError,
}: Props) {
  const [tracks, setTracks] = useState<Array<{ id: string }>>([]);
  const [scenarios, setScenarios] = useState<Array<{ id: string; track_id?: string }>>([]);
  const [busy, setBusy] = useState(false);
  const [warnings, setWarnings] = useState<string[]>([]);
  const [okMsg, setOkMsg] = useState<string | null>(null);
  const [genShape, setGenShape] = useState<"stadium" | "rectangle" | "straight">("stadium");
  const [genStraight, setGenStraight] = useState(80);
  const [genRadius, setGenRadius] = useState(30);
  const [genLength, setGenLength] = useState(100);
  const [genWidth, setGenWidth] = useState(50);
  const [genOpenLength, setGenOpenLength] = useState(200);

  const refreshLists = useCallback(async () => {
    const [t, s] = await Promise.all([listTracks(), listScenarios()]);
    setTracks(t.tracks);
    setScenarios(s.scenarios);
    return s.scenarios;
  }, []);

  useEffect(() => {
    refreshLists().catch((e) => onError(String(e)));
  }, [refreshLists, onError]);

  const scenariosOnTrack = scenarios.filter((s) => s.track_id === track?.track_id);

  const loadTrack = async (id: string) => {
    setBusy(true);
    onError(null);
    try {
      const t = await fetchTrack(id);
      const refPts =
        t.reference_line_m && t.reference_line_m.length >= 3
          ? t.reference_line_m
          : t.centerline_m.map((p) => [...p]);
      onTrackChange({
        ...t,
        closed_loop: t.closed_loop ?? true,
        schema_version: t.schema_version || "1.0",
        num_lanes: t.num_lanes ?? 1,
        lane_width_m: t.lane_width_m ?? t.width_m,
        reference_line_m: refPts,
      });
      onEditPhaseChange("lanes");
      const listed = await refreshLists();
      const onThis = listed.filter((s) => s.track_id === id);
      if (onThis.length > 0) {
        const s = await fetchScenario(onThis[0].id);
        onScenarioChange({ ...s, npcs: s.npcs || [], obstacles: s.obstacles || [] });
      } else {
        const draftTrack = {
          ...t,
          closed_loop: t.closed_loop ?? true,
          schema_version: t.schema_version || "1.0",
          num_lanes: t.num_lanes ?? 1,
          lane_width_m: t.lane_width_m ?? t.width_m,
          reference_line_m: refPts,
        };
        onScenarioChange(defaultScenario(`${id}_demo`, id, draftTrack));
      }
    } catch (e) {
      onError(String(e));
    } finally {
      setBusy(false);
    }
  };

  const loadScenario = async (id: string) => {
    setBusy(true);
    onError(null);
    try {
      const s = await fetchScenario(id);
      // Map first: if scenario's track differs, load that track geometry.
      if (!track || s.track_id !== track.track_id) {
        const t = await fetchTrack(s.track_id);
        const refPts =
          t.reference_line_m && t.reference_line_m.length >= 3
            ? t.reference_line_m
            : t.centerline_m.map((p) => [...p]);
        onTrackChange({
          ...t,
          closed_loop: t.closed_loop ?? true,
          schema_version: t.schema_version || "1.0",
          num_lanes: t.num_lanes ?? 1,
          lane_width_m: t.lane_width_m ?? t.width_m,
          reference_line_m: refPts,
        });
        onEditPhaseChange("lanes");
      }
      onScenarioChange({
        ...s,
        npcs: s.npcs || [],
        obstacles: s.obstacles || [],
      });
    } catch (e) {
      onError(String(e));
    } finally {
      setBusy(false);
    }
  };

  const persistTrack = async (): Promise<void> => {
    if (!track) throw new Error("no track");
    const exists = tracks.some((t) => t.id === track.track_id);
    if (exists) await saveTrack(track);
    else await createTrack(track);
  };

  const doSaveTrack = async () => {
    if (!track) return;
    setBusy(true);
    onError(null);
    setOkMsg(null);
    try {
      await persistTrack();
      await refreshLists();
      setWarnings([]);
      setOkMsg(`Track saved: ${track.track_id}`);
    } catch (e) {
      onError(String(e));
    } finally {
      setBusy(false);
    }
  };

  const doExportTrack = async () => {
    if (!track) return;
    setBusy(true);
    onError(null);
    setOkMsg(null);
    try {
      const data = await exportTrack(track, track.sampling_resolution_m ?? 0.5);
      const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = `${track.track_id}_export.json`;
      a.click();
      URL.revokeObjectURL(url);
      setOkMsg(
        `Exported ${data.reference_line.length} pts (ref + L/R edges) → ${track.track_id}_export.json`
      );
    } catch (e) {
      onError(String(e));
    } finally {
      setBusy(false);
    }
  };

  const doNewTrack = () => {
    onError(null);
    const base = `track_${Date.now().toString(36).slice(-6)}`;
    const id = window.prompt("New track id (alphanumeric / _ / -):", base);
    if (!id) return;
    if (!/^[a-zA-Z0-9_\-]+$/.test(id)) {
      onError("Invalid track id");
      return;
    }
    const t = defaultRectangleTrack(id);
    onTrackChange(t);
    onEditPhaseChange("lanes");
    // New map starts with a draft scenario bound to this track (save both before Run).
    onScenarioChange(defaultScenario(`${id}_demo`, id, t));
  };

  const doSaveScenario = async () => {
    if (!scenario || !track) return;
    setBusy(true);
    onError(null);
    setOkMsg(null);
    try {
      // Scenario save requires the track file to exist — persist map first.
      await persistTrack();
      const bound: ScenarioDef = { ...scenario, track_id: track.track_id };
      onScenarioChange(bound);
      await refreshLists();
      const listed = await listScenarios();
      setScenarios(listed.scenarios);
      const exists = listed.scenarios.some((s) => s.id === bound.scenario_id);
      const res = exists ? await saveScenario(bound) : await createScenario(bound);
      const saveWarnings = res.warnings || [];
      setWarnings(saveWarnings);
      await refreshLists();
      setOkMsg(
        `Scenario saved: ${bound.scenario_id} (map ${track.track_id})` +
          (saveWarnings.length ? ` · ${saveWarnings.length} warning(s): ${saveWarnings.join("; ")}` : "") +
          " · Live: Create/Reset to apply (READY sessions auto-reload on Start)"
      );
    } catch (e) {
      onError(String(e));
    } finally {
      setBusy(false);
    }
  };

  const doNewScenario = () => {
    onError(null);
    if (!track) {
      onError("Select or create a track first");
      return;
    }
    const base = `scenario_${Date.now().toString(36).slice(-6)}`;
    const id = window.prompt("New scenario id:", base);
    if (!id) return;
    if (!/^[a-zA-Z0-9_\-]+$/.test(id)) {
      onError("Invalid scenario id");
      return;
    }
    onScenarioChange(defaultScenario(id, track.track_id, track));
  };

  const patchScenario = (patch: Partial<ScenarioDef>) => {
    if (!scenario) return;
    onScenarioChange({ ...scenario, ...patch });
  };

  const patchEgo = (field: string, value: number) => {
    if (!scenario) return;
    if (field === "speed") {
      onScenarioChange({
        ...scenario,
        ego: { ...scenario.ego, initial_speed_mps: value },
      });
      return;
    }
    const pose = { ...scenario.ego.initial_pose };
    if (field === "yaw_rad") {
      pose.yaw_rad = value;
      onScenarioChange({
        ...scenario,
        ego: { ...scenario.ego, initial_pose: pose },
      });
      return;
    }
    (pose as Record<string, number>)[field] = value;
    onScenarioChange({
      ...scenario,
      ego: { ...scenario.ego, initial_pose: pose },
    });
  };

  const addNpc = () => {
    if (!scenario || !track) return;
    const nLanes = Math.max(1, Math.round(track.num_lanes ?? 1));
    const laneIndex = Math.max(0, Math.min(nLanes - 1, 0));
    const id = `npc_${scenario.npcs.length + 1}`;
    const pose = scenario.ego?.initial_pose;
    const marker: number[] = pose
      ? [pose.x_m + 4, pose.y_m]
      : track.centerline_m[0]
        ? [track.centerline_m[0][0], track.centerline_m[0][1]]
        : [0, 0];
    patchScenario({
      npcs: [
        ...scenario.npcs,
        {
          id,
          model: "trajectory",
          strategy: "trajectory",
          // Single marker for Edit; runtime uses lane when < 2 pts.
          trajectory_xy: [marker],
          speed_mps: 6,
          loop: true,
          length_m: 4,
          width_m: 2,
          lane_index: laneIndex,
        },
      ],
    });
  };

  const addObstacle = () => {
    if (!scenario || !track) return;
    const p = track.centerline_m[Math.floor(track.centerline_m.length / 4)] || [10, 0];
    const id = `obs_${scenario.obstacles.length + 1}`;
    patchScenario({
      obstacles: [
        ...scenario.obstacles,
        { id, x_m: p[0], y_m: p[1], yaw_rad: 0, length_m: 1.5, width_m: 1.5 },
      ],
    });
  };

  return (
    <aside className="hud-left editor-panel">
      <h2>Scene Editor</h2>

      <div className="field-row">
        <label className="field">
          <span>Track</span>
          <select
            disabled={busy}
            value={tracks.some((t) => t.id === track?.track_id) ? track?.track_id || "" : ""}
            onChange={(e) => loadTrack(e.target.value)}
          >
            <option value="" disabled>
              {track && !tracks.some((t) => t.id === track.track_id)
                ? `(new) ${track.track_id}`
                : "select…"}
            </option>
            {tracks.map((t) => (
              <option key={t.id} value={t.id}>
                {t.id}
              </option>
            ))}
          </select>
        </label>
        <button type="button" disabled={busy} onClick={doNewTrack} title="New blank rectangular track">
          + Track
        </button>
      </div>

      {track && (
        <>
          <label className="field">
            <span>Track id</span>
            <input
              type="text"
              value={track.track_id}
              onChange={(e) => onTrackChange({ ...track, track_id: e.target.value.trim() })}
            />
          </label>
          <div className="tool-row">
            <button
              type="button"
              className={editPhase === "lanes" ? "tool-active" : ""}
              onClick={() => {
                onEditPhaseChange("lanes");
                onEditToolChange("move");
              }}
            >
              1. Lane shape
            </button>
            <button
              type="button"
              className={editPhase === "reference" ? "tool-active" : ""}
              onClick={() => {
                onEditPhaseChange("reference");
                onEditToolChange("move");
              }}
            >
              2. Reference
            </button>
          </div>

          {editPhase === "lanes" && (
            <>
              <h3>Lanes</h3>
              <div className="field-row">
                <label className="field">
                  <span>Num lanes</span>
                  <input
                    type="number"
                    min={1}
                    max={8}
                    step={1}
                    value={track.num_lanes ?? 1}
                    onChange={(e) =>
                      onTrackChange(
                        syncLaneWidth(
                          track,
                          Number(e.target.value),
                          track.lane_width_m ??
                            track.width_m / Math.max(1, track.num_lanes ?? 1)
                        )
                      )
                    }
                  />
                </label>
                <label className="field">
                  <span>Lane width m</span>
                  <input
                    type="number"
                    min={0.5}
                    step={0.25}
                    value={track.lane_width_m ?? track.width_m}
                    onChange={(e) =>
                      onTrackChange(
                        syncLaneWidth(track, track.num_lanes ?? 1, Number(e.target.value))
                      )
                    }
                  />
                </label>
              </div>
              <label className="field">
                <span>Total width m (auto)</span>
                <input type="number" value={Number(track.width_m).toFixed(2)} disabled readOnly />
              </label>
              <label className="field">
                <span>Sample spacing m</span>
                <input
                  type="number"
                  min={0.2}
                  max={10}
                  step={0.1}
                  value={track.sampling_resolution_m ?? 0.5}
                  onChange={(e) =>
                    onTrackChange({
                      ...track,
                      sampling_resolution_m: Math.max(0.2, Number(e.target.value) || 0.5),
                    })
                  }
                  title="Path upsample spacing for control / lanes"
                />
              </label>
              <label className="field checkbox-row">
                <input
                  type="checkbox"
                  checked={track.closed_loop !== false}
                  onChange={(e) => {
                    onTrackChange({
                      ...track,
                      closed_loop: e.target.checked,
                      // Platform default: all lanes unidirectional.
                      bidirectional: false,
                    });
                  }}
                />
                <span>Closed loop (uncheck for open start→end road)</span>
              </label>
              <label className="field checkbox-row">
                <input
                  type="checkbox"
                  checked={!!track.bidirectional}
                  onChange={(e) =>
                    onTrackChange({ ...track, bidirectional: e.target.checked })
                  }
                  title="Optional: left-of-spine lanes travel opposite the road spine"
                />
                <span>Bidirectional (optional oncoming)</span>
              </label>

              <h3>Parametric shape</h3>
              <label className="field">
                <span>Shape</span>
                <select
                  value={genShape}
                  onChange={(e) =>
                    setGenShape(e.target.value as "stadium" | "rectangle" | "straight")
                  }
                >
                  <option value="stadium">Stadium oval</option>
                  <option value="rectangle">Rectangle loop</option>
                  <option value="straight">Straight (open)</option>
                </select>
              </label>
              {genShape === "stadium" ? (
                <div className="field-row">
                  <label className="field">
                    <span>Straight m</span>
                    <input
                      type="number"
                      value={genStraight}
                      onChange={(e) => setGenStraight(Number(e.target.value))}
                    />
                  </label>
                  <label className="field">
                    <span>Radius m</span>
                    <input
                      type="number"
                      value={genRadius}
                      onChange={(e) => setGenRadius(Number(e.target.value))}
                    />
                  </label>
                </div>
              ) : genShape === "straight" ? (
                <div className="field-row">
                  <label className="field">
                    <span>Length m</span>
                    <input
                      type="number"
                      min={5}
                      value={genOpenLength}
                      onChange={(e) => setGenOpenLength(Number(e.target.value))}
                    />
                  </label>
                </div>
              ) : (
                <div className="field-row">
                  <label className="field">
                    <span>Length m</span>
                    <input
                      type="number"
                      value={genLength}
                      onChange={(e) => setGenLength(Number(e.target.value))}
                    />
                  </label>
                  <label className="field">
                    <span>Width m</span>
                    <input
                      type="number"
                      value={genWidth}
                      onChange={(e) => setGenWidth(Number(e.target.value))}
                    />
                  </label>
                </div>
              )}
              <button
                type="button"
                disabled={busy}
                onClick={() => {
                  void (async () => {
                    setBusy(true);
                    onError(null);
                    setOkMsg(null);
                    try {
                      const params =
                        genShape === "stadium"
                          ? { straight_m: genStraight, radius_m: genRadius, resolution_m: 2 }
                          : genShape === "straight"
                            ? { length_m: genOpenLength, resolution_m: 2 }
                            : { length_m: genLength, width_m: genWidth, resolution_m: 2 };
                      const res = await generateTrack(track, genShape, params);
                      const n = track.num_lanes ?? 1;
                      const lw =
                        track.lane_width_m ?? track.width_m / Math.max(1, track.num_lanes ?? 1);
                      const cl = res.track.centerline_m;
                      onTrackChange({
                        ...res.track,
                        num_lanes: n,
                        lane_width_m: lw,
                        width_m: n * lw,
                        reference_line_m: cl.map((p) => [...p]),
                        friction_mu: track.friction_mu,
                        friction_zones: track.friction_zones,
                      });
                      setOkMsg(`Applied ${genShape} spine (${cl.length} pts); reference reset`);
                    } catch (e) {
                      onError(String(e));
                    } finally {
                      setBusy(false);
                    }
                  })();
                }}
              >
                Apply shape
              </button>

              <span className="field-label">Road spine points ({track.centerline_m.length})</span>
              <div className="tool-row">
                {(
                  [
                    ["move", "Move"],
                    ["add", "Add"],
                    ["delete", "Delete"],
                  ] as const
                ).map(([id, label]) => (
                  <button
                    key={id}
                    type="button"
                    className={editTool === id ? "tool-active" : ""}
                    onClick={() => onEditToolChange(id)}
                  >
                    {label}
                  </button>
                ))}
              </div>
            </>
          )}

          {editPhase === "reference" && (
            <>
              <h3>Driving reference</h3>
              <span className="field-label">
                Reference points ({(track.reference_line_m || track.centerline_m).length})
              </span>
              <div className="tool-row">
                {(
                  [
                    ["move", "Move"],
                    ["add", "Add"],
                    ["delete", "Delete"],
                  ] as const
                ).map(([id, label]) => (
                  <button
                    key={id}
                    type="button"
                    className={editTool === id ? "tool-active" : ""}
                    onClick={() => onEditToolChange(id)}
                  >
                    {label}
                  </button>
                ))}
              </div>
              <button
                type="button"
                disabled={busy}
                onClick={() => {
                  onTrackChange({
                    ...track,
                    reference_line_m: track.centerline_m.map((p) => [...p]),
                  });
                  setOkMsg("Reference reset to road spine");
                }}
              >
                Reset to road spine
              </button>
            </>
          )}

          <h3>Friction zones</h3>
          <label className="field">
            <span>Default μ</span>
            <input
              type="number"
              min={0.05}
              max={2}
              step={0.05}
              value={track.friction_mu ?? 1}
              onChange={(e) =>
                onTrackChange({
                  ...track,
                  friction_mu: Math.max(0.05, Number(e.target.value) || 1),
                })
              }
            />
          </label>
          <div className="tool-row">
            <button
              type="button"
              disabled={busy}
              onClick={() => {
                const zones = [...(track.friction_zones || [])];
                const n = zones.length + 1;
                const cx = track.centerline_m[0]?.[0] ?? 0;
                const cy = track.centerline_m[0]?.[1] ?? 0;
                zones.push({
                  id: `zone_${n}`,
                  mu: 0.4,
                  kind: "polygon",
                  points_xy: [
                    [cx + 5, cy - 4],
                    [cx + 20, cy - 4],
                    [cx + 20, cy + 4],
                    [cx + 5, cy + 4],
                  ],
                });
                onTrackChange({ ...track, friction_zones: zones });
                setOkMsg(`Added zone_${n}`);
              }}
            >
              + Rectangle zone
            </button>
          </div>
          {(track.friction_zones || []).map((z, idx) => (
            <div key={z.id || idx} className="field-row" style={{ alignItems: "end" }}>
              <label className="field">
                <span>{z.id || `zone_${idx}`}</span>
                <input
                  type="number"
                  min={0.05}
                  max={2}
                  step={0.05}
                  value={z.mu}
                  onChange={(e) => {
                    const zones = (track.friction_zones || []).map((zz, i) =>
                      i === idx
                        ? { ...zz, mu: Math.max(0.05, Number(e.target.value) || 0.4) }
                        : zz
                    );
                    onTrackChange({ ...track, friction_zones: zones });
                  }}
                />
              </label>
              <button
                type="button"
                onClick={() => {
                  const zones = (track.friction_zones || []).filter((_, i) => i !== idx);
                  onTrackChange({ ...track, friction_zones: zones });
                }}
              >
                Delete
              </button>
            </div>
          ))}
        </>
      )}

      <div className="field-row">
        <label className="field">
          <span>Scenario (on this map)</span>
          <select
            disabled={busy || !track}
            value={
              scenariosOnTrack.some((s) => s.id === scenario?.scenario_id)
                ? scenario?.scenario_id || ""
                : ""
            }
            onChange={(e) => loadScenario(e.target.value)}
          >
            <option value="" disabled>
              {scenario &&
              track &&
              scenario.track_id === track.track_id &&
              !scenariosOnTrack.some((s) => s.id === scenario.scenario_id)
                ? `(new) ${scenario.scenario_id}`
                : scenariosOnTrack.length === 0
                  ? "No saved scenarios on this map"
                  : "select…"}
            </option>
            {scenariosOnTrack.map((s) => (
              <option key={s.id} value={s.id}>
                {s.id}
              </option>
            ))}
          </select>
        </label>
        <button type="button" disabled={busy || !track} onClick={doNewScenario} title="New scenario on this map">
          + Scenario
        </button>
      </div>

      {scenario && track && (
        <label className="field">
          <span>Scenario id · bound to {track.track_id}</span>
          <input
            type="text"
            value={scenario.scenario_id}
            onChange={(e) =>
              onScenarioChange({
                ...scenario,
                scenario_id: e.target.value.trim(),
                track_id: track.track_id,
              })
            }
          />
        </label>
      )}

      {scenario && (
        <>
          <h3>Ego</h3>
          <div className="tool-row">
            <button
              type="button"
              className={editTool === "place_ego" ? "tool-active" : ""}
              disabled={!track}
              onClick={() => onEditToolChange("place_ego")}
              title="Click to place, drag for heading, release to confirm"
            >
              Place ego
            </button>
            <button
              type="button"
              disabled={!track}
              onClick={() => {
                if (!track || !scenario) return;
                const pose = egoPoseFromTrack(track);
                onScenarioChange({
                  ...scenario,
                  ego: { ...scenario.ego, initial_pose: pose },
                });
              }}
              title="Snap ego IC to track reference start"
            >
              Snap IC to track start
            </button>
          </div>
          <div className="field-row">
            <label>
              x
              <input
                type="number"
                step="0.5"
                value={scenario.ego.initial_pose.x_m}
                onChange={(e) => patchEgo("x_m", Number(e.target.value))}
              />
            </label>
            <label>
              y
              <input
                type="number"
                step="0.5"
                value={scenario.ego.initial_pose.y_m}
                onChange={(e) => patchEgo("y_m", Number(e.target.value))}
              />
            </label>
          </div>
          <div className="field-row">
            <label>
              yaw °
              <input
                type="number"
                step="1"
                value={Number(
                  ((scenario.ego.initial_pose.yaw_rad * 180) / Math.PI).toFixed(1)
                )}
                onChange={(e) =>
                  patchEgo("yaw_rad", (Number(e.target.value) * Math.PI) / 180)
                }
              />
            </label>
            <label>
              vx
              <input
                type="number"
                step="0.5"
                value={scenario.ego.initial_speed_mps}
                onChange={(e) => patchEgo("speed", Number(e.target.value))}
              />
            </label>
          </div>
          {(() => {
            const snap = snapToPolyline(
              scenario.ego.initial_pose.x_m,
              scenario.ego.initial_pose.y_m,
              referencePolyline(track)
            );
            if (!snap) return null;
            return (
              <p className="editor-hint">s ≈ {snap.station_m.toFixed(1)} m</p>
            );
          })()}

          <h3>Controller</h3>
          <label className="field">
            <span>Type</span>
            <select
              value={scenario.controller?.type || "pure_pursuit"}
              onChange={(e) =>
                patchScenario({
                  controller: {
                    type: e.target.value as
                      | "pure_pursuit"
                      | "lane_pp"
                      | "lattice_pid"
                      | "lattice_pid_ml"
                      | "constant"
                      | "mppi"
                      | "strategy01",
                    params: scenario.controller?.params || { throttle_cmd: 0.35 },
                  },
                })
              }
            >
              <option value="pure_pursuit">Pure Pursuit</option>
              <option value="lane_pp">Lane PP (multi-lane)</option>
              <option value="lattice_pid">Lattice PID</option>
              <option value="lattice_pid_ml">Lattice PID Multi-lane</option>
              <option value="mppi">MPPI (CPU)</option>
              <option value="strategy01">strategy01 (legacy)</option>
              <option value="constant">Constant</option>
            </select>
          </label>

          <h3>NPCs ({scenario.npcs.length})</h3>
          <button type="button" disabled={busy} onClick={addNpc}>
            + NPC
          </button>
          {scenario.npcs.map((n, i) => (
            <div key={n.id} className="mini-card">
              <span>{n.id}</span>
              <input
                type="number"
                step="0.5"
                value={n.speed_mps}
                title="speed"
                onChange={(e) => {
                  const npcs = scenario.npcs.slice();
                  npcs[i] = { ...n, speed_mps: Number(e.target.value) };
                  patchScenario({ npcs });
                }}
              />
              <button
                type="button"
                onClick={() => patchScenario({ npcs: scenario.npcs.filter((x) => x.id !== n.id) })}
              >
                ×
              </button>
            </div>
          ))}

          <h3>Obstacles ({scenario.obstacles.length})</h3>
          <button type="button" disabled={busy} onClick={addObstacle}>
            + Obstacle
          </button>
          {scenario.obstacles.map((o, i) => (
            <div key={o.id} className="mini-card">
              <span>{o.id}</span>
              <button
                type="button"
                onClick={() =>
                  patchScenario({ obstacles: scenario.obstacles.filter((x) => x.id !== o.id) })
                }
              >
                ×
              </button>
              <div className="field-row">
                <input
                  type="number"
                  value={o.x_m}
                  onChange={(e) => {
                    const obstacles = scenario.obstacles.slice();
                    obstacles[i] = { ...o, x_m: Number(e.target.value) };
                    patchScenario({ obstacles });
                  }}
                />
                <input
                  type="number"
                  value={o.y_m}
                  onChange={(e) => {
                    const obstacles = scenario.obstacles.slice();
                    obstacles[i] = { ...o, y_m: Number(e.target.value) };
                    patchScenario({ obstacles });
                  }}
                />
              </div>
            </div>
          ))}

          <h3>Termination</h3>
          <label className="field">
            <span>max t (s)</span>
            <input
              type="number"
              value={scenario.termination.max_sim_time_s}
              onChange={(e) =>
                patchScenario({
                  termination: {
                    ...scenario.termination,
                    max_sim_time_s: Number(e.target.value),
                  },
                })
              }
            />
          </label>
          <label className="check">
            <input
              type="checkbox"
              checked={!!scenario.termination.stop_on_collision}
              onChange={(e) =>
                patchScenario({
                  termination: { ...scenario.termination, stop_on_collision: e.target.checked },
                })
              }
            />
            stop on collision
          </label>
          <label className="check">
            <input
              type="checkbox"
              checked={!!scenario.termination.stop_on_off_track}
              onChange={(e) =>
                patchScenario({
                  termination: { ...scenario.termination, stop_on_off_track: e.target.checked },
                })
              }
            />
            stop on off-track
          </label>
        </>
      )}

      <div className="editor-actions">
        <button type="button" className="primary" disabled={busy || !track} onClick={() => void doSaveTrack()}>
          {busy ? "Saving…" : "Save Track"}
        </button>
        <button
          type="button"
          disabled={busy || !track}
          onClick={() => void doExportTrack()}
          title="Export reference + left/right edges JSON"
        >
          Export Map
        </button>
        <button
          type="button"
          className="primary"
          disabled={busy || !scenario || !track}
          onClick={() => void doSaveScenario()}
        >
          {busy ? "Saving…" : "Save Scenario"}
        </button>
        <button
          type="button"
          disabled={busy || !track || !scenario}
          onClick={() => {
            if (!track || !scenario) return;
            if (scenario.track_id !== track.track_id) {
              onError(
                `Scenario bound to ${scenario.track_id}, not current map ${track.track_id} — Save Scenario first`
              );
              return;
            }
            onRunLive(track.track_id, scenario.scenario_id);
          }}
        >
          Run in Live
        </button>
      </div>

      {okMsg && <p className="ok-msg">{okMsg}</p>}
      {warnings.length > 0 && (
        <ul className="warn-list">
          {warnings.map((w) => (
            <li key={w}>{w}</li>
          ))}
        </ul>
      )}
    </aside>
  );
}
