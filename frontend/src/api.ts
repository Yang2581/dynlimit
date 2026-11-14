export type SessionStatus = {
  session_id: string;
  state: string;
  plant: string;
  controller?: string;
  sim_time_ns: number;
  step_index: number;
  experiment_dir?: string | null;
  error?: string | null;
  plant_ic_applied?: boolean;
  plant_error?: string | null;
  plant_ic_par?: {
    applied?: boolean;
    station_m?: number;
    pose_x_m?: number;
    pose_y_m?: number;
    pose_yaw_deg?: number;
    speed_mps?: number;
    speed_kmh?: number;
    fields?: string[];
  } | null;
  longitudinal_mode?: "control" | "plant" | string;
  plant_speed_mps?: number | null;
  has_longitudinal_control?: boolean;
  deadline_miss_count?: number;
  last_solve_ms?: number | null;
  deadline_budget_ms?: number | null;
  /** Platform run limit (s); CarSim plant uses procedure TSTOP when available */
  max_sim_time_s?: number | null;
  duration_source?: "carsim_tstop" | "scenario" | string | null;
};

export type ActorPose = {
  id: string;
  x_m: number;
  y_m: number;
  yaw_rad?: number;
  length_m?: number;
  width_m?: number;
  z_m?: number;
};

export type SnapshotMsg = {
  schema_version: string;
  type: string;
  session_id?: string;
  experiment_id?: string;
  seq: number;
  sim_time_ns: number;
  step_index?: number;
  vehicles: Array<{
    id: string;
    x_m: number;
    y_m: number;
    z_m: number;
    yaw_rad: number;
    vx_mps: number;
    vy_mps: number;
    yaw_rate_radps: number;
  }>;
  metrics: Record<string, unknown>;
  trail_xy?: number[][];
  controller?: string;
  control?: Record<string, number> | null;
  npcs?: ActorPose[];
  obstacles?: ActorPose[];
  replay?: {
    playing: boolean;
    rate: number;
    t_start_ns: number;
    t_end_ns: number;
    t_ns?: number;
  };
};

export type ExperimentInfo = {
  id: string;
  status: string | null;
  duration_s: number;
  n_states: number;
  has_mcap: boolean;
  display_name?: string | null;
  metadata: Record<string, unknown>;
  metrics?: Record<string, unknown> | null;
};

export type TrackDef = {
  schema_version?: string;
  track_id: string;
  closed_loop?: boolean;
  /** Explicit oncoming left-of-spine lanes; default/omit = unidirectional. */
  bidirectional?: boolean | null;
  width_m: number;
  friction_mu?: number;
  centerline_m: number[][];
  sampling_resolution_m?: number;
  name?: string | null;
  notes?: string | null;
  /** Equal-width lanes; total width_m = num_lanes * lane_width_m */
  num_lanes?: number;
  lane_width_m?: number;
  /** Independent driving reference polyline; defaults to centerline copy */
  reference_line_m?: number[][];
  /** @deprecated legacy constant offset; prefer reference_line_m */
  reference_lateral_m?: number;
  friction_zones?: Array<{
    id: string;
    mu: number;
    kind?: string;
    points_xy: number[][];
  }>;
};

export type ScenarioDef = {
  schema_version?: string;
  scenario_id: string;
  track_id: string;
  ego: {
    vehicle_config_id?: string;
    initial_pose: { x_m: number; y_m: number; z_m?: number; yaw_rad: number };
    initial_speed_mps: number;
  };
  npcs: Array<{
    id: string;
    model?: string;
    /** Pluggable motion strategy; default trajectory. idm reserved. */
    strategy?: "trajectory" | "idm";
    trajectory_xy: number[][];
    speed_mps: number;
    loop?: boolean;
    length_m?: number;
    width_m?: number;
    /** 0 = rightmost; used when trajectory empty / for future IDM */
    lane_index?: number | null;
    strategy_params?: Record<string, number | string | boolean>;
  }>;
  obstacles: Array<{
    id: string;
    x_m: number;
    y_m: number;
    yaw_rad?: number;
    length_m: number;
    width_m: number;
  }>;
  controller?: {
    type: "pure_pursuit" | "constant" | "mppi" | string;
    params?: Record<string, number>;
  };
  termination: {
    max_sim_time_s: number;
    stop_on_collision?: boolean;
    stop_on_off_track?: boolean;
  };
};

export type DiagnosticsSummary = {
  u_tire_thresh?: number;
  max_abs_beta?: number;
  p95_abs_beta?: number;
  max_abs_beta_deg?: number;
  p95_abs_beta_deg?: number;
  U_tire_max?: number;
  U_tire_p95?: number;
  U_tire_frac_above_thresh?: number;
  solve_ms_max?: number;
  solve_ms_p95?: number;
  solve_ms_median?: number;
  deadline_miss_rate?: number;
  flags?: {
    task_only_success?: boolean;
    physics_safe_success?: boolean;
    deadline_feasible_success?: boolean;
    goal_reached?: boolean;
    no_collision?: boolean;
    no_off_track?: boolean;
    deadline_ok?: boolean;
  };
};

export type SeriesPayload = {
  experiment_id: string;
  t_start_ns: number;
  t_end_ns: number;
  friction_mu?: number;
  diagnostics_available?: {
    beta_rad?: boolean;
    abs_beta_deg?: boolean;
    u_tire?: boolean;
    solve_ms?: boolean;
    deadline_miss?: boolean;
    summary?: boolean;
    flags?: boolean;
  };
  summary?: DiagnosticsSummary;
  series: {
    t_s: number[];
    vx_mps?: number[];
    vy_mps?: number[];
    yaw_rate_radps?: number[];
    yaw_rad?: number[];
    ax_mps2?: number[];
    ay_mps2?: number[];
    steering_cmd?: number[];
    x_m?: number[];
    y_m?: number[];
    /** Sideslip β = atan2(vy, vx). */
    beta_rad?: Array<number | null>;
    abs_beta_deg?: Array<number | null>;
    /** Per-frame tire utilization (when tire forces logged). */
    u_tire?: Array<number | null>;
    solve_ms?: Array<number | null>;
    deadline_miss?: Array<number | null>;
    /** Extra recorded channels (e.g. CarSim wheel_states). */
    [key: string]: Array<number | null> | undefined;
  };
  /** True when MCAP exists but has no /ego/state (truncated run). */
  empty?: boolean;
  detail?: string;
};

async function json<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(path, {
    headers: { "Content-Type": "application/json", ...(init?.headers || {}) },
    ...init,
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(text || res.statusText);
  }
  return res.json() as Promise<T>;
}

export function createSession(body: {
  plant: "mock" | "carsim";
  controller?: "pure_pursuit" | "constant" | "mppi" | "strategy01" | string;
  steering_cmd?: number;
  throttle_cmd?: number;
  realtime?: boolean;
  track_id?: string;
  scenario_id?: string;
  display_name?: string;
}): Promise<SessionStatus> {
  return json("/api/sessions", {
    method: "POST",
    body: JSON.stringify({ realtime: true, ...body }),
  });
}

export function patchExperiment(
  id: string,
  body: { display_name: string | null }
): Promise<{ id: string; display_name: string | null }> {
  return json(`/api/experiments/${id}`, {
    method: "PATCH",
    body: JSON.stringify(body),
  });
}

export function deleteExperiment(id: string): Promise<{ ok: boolean; id: string }> {
  return json(`/api/experiments/${id}`, { method: "DELETE" });
}

export function listTracks(): Promise<{ tracks: Array<{ id: string; path: string; name?: string; width_m?: number }> }> {
  return json("/api/tracks");
}

export function listScenarios(): Promise<{
  scenarios: Array<{ id: string; track_id?: string; path?: string }>;
}> {
  return json("/api/scenarios");
}

export function deleteTrack(id: string): Promise<{ ok: boolean }> {
  return json(`/api/tracks/${id}`, { method: "DELETE" });
}

export function deleteScenario(id: string): Promise<{ ok: boolean }> {
  return json(`/api/scenarios/${id}`, { method: "DELETE" });
}

export function fetchScenario(id: string): Promise<ScenarioDef> {
  return json(`/api/scenarios/${id}`);
}

export function saveTrack(track: TrackDef): Promise<{ ok: boolean; warnings?: string[] }> {
  return json(`/api/tracks/${track.track_id}`, {
    method: "PUT",
    body: JSON.stringify(track),
  });
}

/** Create a new track file (409 if id already exists). */
export function createTrack(track: TrackDef): Promise<{ ok: boolean; path?: string }> {
  return json("/api/tracks", {
    method: "POST",
    body: JSON.stringify(track),
  });
}

export type TrackExport = {
  schema_version: string;
  track_id: string;
  width_m: number;
  length_m: number;
  closed_loop: boolean;
  resolution_m: number;
  reference_line: Array<{ x_m: number; y_m: number; z_m: number; s_m: number; yaw_rad: number }>;
  left_boundary: Array<{ x_m: number; y_m: number; z_m: number; s_m: number }>;
  right_boundary: Array<{ x_m: number; y_m: number; z_m: number; s_m: number }>;
  centerline_m: number[][];
  notes?: Record<string, string>;
};

/** Export map reference line + left/right boundaries (works for unsaved drafts). */
export function exportTrack(
  track: TrackDef,
  resolution_m?: number
): Promise<TrackExport> {
  return json("/api/tracks/export", {
    method: "POST",
    body: JSON.stringify({ ...track, resolution_m }),
  });
}

export function exportSavedTrack(
  trackId: string,
  resolution_m?: number
): Promise<TrackExport> {
  const q =
    resolution_m != null ? `?resolution_m=${encodeURIComponent(String(resolution_m))}` : "";
  return json(`/api/tracks/${trackId}/export${q}`);
}

export function generateTrack(
  track: TrackDef,
  shape: "stadium" | "rectangle" | "straight",
  params: Record<string, number | undefined>
): Promise<{ ok: boolean; track: TrackDef }> {
  return json("/api/tracks/generate", {
    method: "POST",
    body: JSON.stringify({ track, shape, params }),
  });
}

export function saveScenario(scenario: ScenarioDef): Promise<{ ok: boolean; warnings?: string[] }> {
  return json(`/api/scenarios/${scenario.scenario_id}`, {
    method: "PUT",
    body: JSON.stringify(scenario),
  });
}

export function createScenario(scenario: ScenarioDef): Promise<{ ok: boolean; warnings?: string[] }> {
  return json("/api/scenarios", {
    method: "POST",
    body: JSON.stringify(scenario),
  });
}

export function sessionCommand(
  id: string,
  cmd: "start" | "pause" | "step" | "reset" | "stop"
): Promise<{ accepted: boolean; state?: string; error?: string }> {
  return json(`/api/sessions/${id}/${cmd}`, { method: "POST" });
}

export function fetchTrack(trackId: string): Promise<TrackDef> {
  return json(`/api/tracks/${trackId}/geometry`);
}

export type SweepRun = {
  run_id: string;
  rank: number;
  params: Record<string, unknown>;
  metrics: {
    mean_abs_lat_m?: number;
    path_length_m?: number;
    duration_s?: number;
    wall_s?: number;
    off_track?: boolean;
    collision?: boolean;
    finished_reason?: string;
    last_solve_ms?: number;
    rollout_backend?: string;
  };
};

export type SweepSummary = {
  sweep_id: string;
  controller: string;
  scenario_id: string;
  n_runs: number;
  best_run_id: string | null;
  grid: Record<string, unknown[]>;
  runs: SweepRun[];
};

export function listExperiments(): Promise<{ experiments: ExperimentInfo[] }> {
  return json("/api/experiments");
}

export function runSweep(body: {
  controller?: string;
  scenario_id?: string;
  duration_s?: number;
  base_params?: Record<string, unknown>;
  grid?: Record<string, unknown[]>;
}): Promise<SweepSummary> {
  return json("/api/sweeps", {
    method: "POST",
    body: JSON.stringify(body),
  });
}

export function listSweeps(): Promise<{ sweeps: Array<{ sweep_id: string; n_runs?: number }> }> {
  return json("/api/sweeps");
}

export function fetchSweep(id: string): Promise<SweepSummary> {
  return json(`/api/sweeps/${id}`);
}

export function fetchExperiment(id: string): Promise<ExperimentInfo> {
  return json(`/api/experiments/${id}`);
}

export function fetchSeries(id: string): Promise<SeriesPayload> {
  return json(`/api/experiments/${id}/series`);
}

export function replayCommand(
  id: string,
  body: { action: "play" | "pause" | "seek" | "rate"; t_ns?: number; rate?: number }
): Promise<{ accepted: boolean; status: Record<string, unknown>; snapshot?: SnapshotMsg }> {
  return json(`/api/experiments/${id}/replay`, {
    method: "POST",
    body: JSON.stringify(body),
  });
}

export function openSessionSocket(sessionId: string): WebSocket {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return new WebSocket(`${proto}://${location.host}/ws/sessions/${sessionId}`);
}

export function openReplaySocket(experimentId: string): WebSocket {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return new WebSocket(`${proto}://${location.host}/ws/replay/${experimentId}`);
}
