import { useCallback, useEffect, useState } from "react";
import {
  deleteScenario,
  deleteTrack,
  listScenarios,
  listTracks,
} from "./api";

export type AssetsPanelProps = {
  busy?: boolean;
  onOpenTrack: (id: string) => void;
  onOpenScenario: (id: string) => void;
  onNewTrack: () => void;
  /** Parent creates a blank scenario for the chosen track and opens Edit. */
  onNewScenario: (trackId: string) => void;
  onError?: (msg: string | null) => void;
  /** Fired after a successful delete so Live dropdowns can refresh. */
  onAssetsChanged?: () => void;
};

type Tab = "tracks" | "scenarios";

type TrackRow = { id: string; path?: string; name?: string; width_m?: number };
type ScenarioRow = { id: string; track_id?: string; path?: string };

export function AssetsPanel({
  busy = false,
  onOpenTrack,
  onOpenScenario,
  onNewTrack,
  onNewScenario,
  onError,
  onAssetsChanged,
}: AssetsPanelProps) {
  const [tab, setTab] = useState<Tab>("tracks");
  const [tracks, setTracks] = useState<TrackRow[]>([]);
  const [scenarios, setScenarios] = useState<ScenarioRow[]>([]);
  const [newScenarioTrackId, setNewScenarioTrackId] = useState("");
  const [loading, setLoading] = useState(false);
  const [localBusy, setLocalBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  const reportError = useCallback(
    (msg: string | null) => {
      setErr(msg);
      onError?.(msg);
    },
    [onError]
  );

  const refresh = useCallback(async () => {
    setLoading(true);
    reportError(null);
    try {
      const [t, s] = await Promise.all([listTracks(), listScenarios()]);
      setTracks(t.tracks);
      setScenarios(s.scenarios);
      setNewScenarioTrackId((prev) => {
        if (prev && t.tracks.some((row) => row.id === prev)) return prev;
        return t.tracks[0]?.id ?? "";
      });
    } catch (e) {
      reportError(String(e));
    } finally {
      setLoading(false);
    }
  }, [reportError]);

  useEffect(() => {
    void refresh();
  }, [refresh]);

  const disabled = busy || localBusy || loading;

  async function handleDeleteTrack(id: string) {
    if (!window.confirm(`Delete track "${id}"? This cannot be undone.`)) return;
    setLocalBusy(true);
    reportError(null);
    try {
      await deleteTrack(id);
      await refresh();
      onAssetsChanged?.();
    } catch (e) {
      reportError(String(e));
    } finally {
      setLocalBusy(false);
    }
  }

  async function handleDeleteScenario(id: string) {
    if (!window.confirm(`Delete scenario "${id}"? This cannot be undone.`)) return;
    setLocalBusy(true);
    reportError(null);
    try {
      await deleteScenario(id);
      await refresh();
      onAssetsChanged?.();
    } catch (e) {
      reportError(String(e));
    } finally {
      setLocalBusy(false);
    }
  }

  return (
    <div className="assets-panel">
      <div className="assets-panel-inner">
        <div className="assets-head">
          <h2>Assets</h2>
          <div className="assets-subtabs">
            <button
              type="button"
              className={tab === "tracks" ? "active" : ""}
              disabled={disabled}
              onClick={() => setTab("tracks")}
            >
              Tracks
            </button>
            <button
              type="button"
              className={tab === "scenarios" ? "active" : ""}
              disabled={disabled}
              onClick={() => setTab("scenarios")}
            >
              Scenarios
            </button>
          </div>
        </div>

        {err && <div className="assets-error">{err}</div>}
        {loading && <p className="assets-muted">Loading…</p>}

        {tab === "tracks" && (
          <>
            <div className="assets-toolbar">
              <button type="button" className="primary" disabled={disabled} onClick={onNewTrack}>
                + New Track
              </button>
              <button type="button" disabled={disabled} onClick={() => void refresh()}>
                Refresh
              </button>
            </div>
            <div className="assets-table-wrap">
              <table className="assets-table">
                <thead>
                  <tr>
                    <th>ID</th>
                    <th>Name</th>
                    <th>Width</th>
                    <th>Actions</th>
                  </tr>
                </thead>
                <tbody>
                  {tracks.length === 0 && !loading ? (
                    <tr>
                      <td colSpan={4} className="assets-empty">
                        No tracks
                      </td>
                    </tr>
                  ) : (
                    tracks.map((t) => (
                      <tr key={t.id}>
                        <td className="assets-id">{t.id}</td>
                        <td>{t.name ?? "—"}</td>
                        <td>{t.width_m != null ? `${t.width_m} m` : "—"}</td>
                        <td className="assets-actions">
                          <button
                            type="button"
                            disabled={disabled}
                            onClick={() => onOpenTrack(t.id)}
                          >
                            Open
                          </button>
                          <button
                            type="button"
                            disabled={disabled}
                            onClick={() => void handleDeleteTrack(t.id)}
                          >
                            Delete
                          </button>
                        </td>
                      </tr>
                    ))
                  )}
                </tbody>
              </table>
            </div>
          </>
        )}

        {tab === "scenarios" && (
          <>
            <div className="assets-toolbar">
              <label className="assets-track-pick">
                Track for new
                <select
                  value={newScenarioTrackId}
                  disabled={disabled || tracks.length === 0}
                  onChange={(e) => setNewScenarioTrackId(e.target.value)}
                >
                  {tracks.length === 0 ? (
                    <option value="">No tracks</option>
                  ) : (
                    tracks.map((t) => (
                      <option key={t.id} value={t.id}>
                        {t.id}
                      </option>
                    ))
                  )}
                </select>
              </label>
              <button
                type="button"
                className="primary"
                disabled={disabled || !newScenarioTrackId}
                onClick={() => onNewScenario(newScenarioTrackId)}
              >
                + New Scenario
              </button>
              <button type="button" disabled={disabled} onClick={() => void refresh()}>
                Refresh
              </button>
            </div>
            <div className="assets-table-wrap">
              <table className="assets-table">
                <thead>
                  <tr>
                    <th>ID</th>
                    <th>Track</th>
                    <th>Actions</th>
                  </tr>
                </thead>
                <tbody>
                  {scenarios.length === 0 && !loading ? (
                    <tr>
                      <td colSpan={3} className="assets-empty">
                        No scenarios
                      </td>
                    </tr>
                  ) : (
                    scenarios.map((s) => (
                      <tr key={s.id}>
                        <td className="assets-id">{s.id}</td>
                        <td>{s.track_id ?? "—"}</td>
                        <td className="assets-actions">
                          <button
                            type="button"
                            disabled={disabled}
                            onClick={() => onOpenScenario(s.id)}
                          >
                            Open
                          </button>
                          <button
                            type="button"
                            disabled={disabled}
                            onClick={() => void handleDeleteScenario(s.id)}
                          >
                            Delete
                          </button>
                        </td>
                      </tr>
                    ))
                  )}
                </tbody>
              </table>
            </div>
          </>
        )}
      </div>
    </div>
  );
}
