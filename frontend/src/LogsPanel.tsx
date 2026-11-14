import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  deleteExperiment,
  fetchSeries,
  listExperiments,
  patchExperiment,
  type ExperimentInfo,
  type SeriesPayload,
} from "./api";

type Props = {
  onOpenReplay: (id: string) => void;
  /** Shared list from App (Replay + Logs); avoids empty UI while re-fetching. */
  experiments?: ExperimentInfo[];
  onExperimentsChange?: (items: ExperimentInfo[]) => void;
};

function experimentLabel(e: ExperimentInfo): string {
  const name = e.display_name?.trim();
  return name || e.id.slice(0, 18);
}

export function LogsPanel({
  onOpenReplay,
  experiments: experimentsProp,
  onExperimentsChange,
}: Props) {
  const [localExperiments, setLocalExperiments] = useState<ExperimentInfo[]>([]);
  const [selectedId, setSelectedId] = useState("");
  const [series, setSeries] = useState<SeriesPayload | null>(null);
  const [rename, setRename] = useState("");
  const [busy, setBusy] = useState(false);
  const [loading, setLoading] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const [checkedIds, setCheckedIds] = useState<Set<string>>(() => new Set());
  const [ctxMenu, setCtxMenu] = useState<{
    id: string;
    clientX: number;
    clientY: number;
  } | null>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const trajWrapRef = useRef<HTMLDivElement>(null);
  const [trajSize, setTrajSize] = useState({ w: 720, h: 420 });
  const lastClickedIdRef = useRef<string | null>(null);

  const experiments = experimentsProp ?? localExperiments;
  const setExperiments = useCallback(
    (items: ExperimentInfo[]) => {
      if (onExperimentsChange) onExperimentsChange(items);
      else setLocalExperiments(items);
    },
    [onExperimentsChange]
  );

  const refresh = useCallback(async () => {
    setErr(null);
    setLoading(true);
    try {
      const r = await listExperiments();
      const withMcap = r.experiments.filter((e) => e.has_mcap);
      const alive = new Set(withMcap.map((e) => e.id));
      setExperiments(withMcap);
      setCheckedIds((prev) => {
        const next = new Set<string>();
        for (const id of prev) if (alive.has(id)) next.add(id);
        return next;
      });
      setSelectedId((prev) => {
        if (prev && withMcap.some((e) => e.id === prev)) return prev;
        return withMcap[0]?.id ?? "";
      });
    } catch (e) {
      setErr(String(e));
    } finally {
      setLoading(false);
    }
  }, [setExperiments]);

  useEffect(() => {
    if (experimentsProp && experimentsProp.length > 0) {
      setSelectedId((prev) => {
        if (prev && experimentsProp.some((e) => e.id === prev)) return prev;
        return experimentsProp[0]?.id ?? "";
      });
      return;
    }
    void refresh();
  }, [experimentsProp, refresh]);

  const selected = useMemo(
    () => experiments.find((e) => e.id === selectedId) ?? null,
    [experiments, selectedId]
  );

  useEffect(() => {
    if (!selectedId) {
      setSeries(null);
      setRename("");
      return;
    }
    setRename(selected?.display_name?.trim() || "");
    let cancelled = false;
    fetchSeries(selectedId)
      .then((s) => {
        if (!cancelled) {
          setSeries(s);
          // Empty/truncated MCAP is a soft empty preview, not a hard error.
          if (s.empty) setErr(null);
        }
      })
      .catch((e) => {
        if (!cancelled) {
          setSeries(null);
          setErr(String(e));
        }
      });
    return () => {
      cancelled = true;
    };
  }, [selectedId, selected?.display_name]);

  useEffect(() => {
    const wrap = trajWrapRef.current;
    if (!wrap || typeof ResizeObserver === "undefined") return;
    const sync = () => {
      const rect = wrap.getBoundingClientRect();
      const w = Math.max(160, Math.floor(rect.width));
      const h = Math.max(160, Math.floor(rect.height));
      setTrajSize((prev) => (prev.w === w && prev.h === h ? prev : { w, h }));
    };
    sync();
    const ro = new ResizeObserver(sync);
    ro.observe(wrap);
    return () => ro.disconnect();
  }, [selectedId]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const cssW = trajSize.w;
    const cssH = trajSize.h;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.max(1, Math.floor(cssW * dpr));
    canvas.height = Math.max(1, Math.floor(cssH * dpr));
    canvas.style.width = `${cssW}px`;
    canvas.style.height = `${cssH}px`;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    ctx.clearRect(0, 0, cssW, cssH);
    ctx.fillStyle = "#0d1015";
    ctx.fillRect(0, 0, cssW, cssH);

    const xs = series?.series?.x_m ?? [];
    const ys = series?.series?.y_m ?? [];
    if (xs.length < 2 || ys.length < 2) {
      ctx.fillStyle = "#9aa3b2";
      ctx.font = "14px sans-serif";
      ctx.fillText("No trajectory data", 16, 28);
      return;
    }

    let minX = Infinity;
    let maxX = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    for (let i = 0; i < xs.length; i++) {
      const x = xs[i];
      const y = ys[i];
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      minX = Math.min(minX, x);
      maxX = Math.max(maxX, x);
      minY = Math.min(minY, y);
      maxY = Math.max(maxY, y);
    }
    if (!Number.isFinite(minX) || !Number.isFinite(minY)) {
      ctx.fillStyle = "#9aa3b2";
      ctx.font = "14px sans-serif";
      ctx.fillText("No valid trajectory points", 16, 28);
      return;
    }

    const pad = Math.max(28, Math.min(cssW, cssH) * 0.06);
    const spanX = Math.max(maxX - minX, 1e-3);
    const spanY = Math.max(maxY - minY, 1e-3);
    // Uniform scale + center so short arcs are not stuck in a corner.
    const scale = Math.min((cssW - pad * 2) / spanX, (cssH - pad * 2) / spanY);
    const drawnW = spanX * scale;
    const drawnH = spanY * scale;
    const ox = (cssW - drawnW) / 2;
    const oy = (cssH - drawnH) / 2;

    const toCanvas = (x: number, y: number): [number, number] => {
      const cx = ox + (x - minX) * scale;
      const cy = oy + drawnH - (y - minY) * scale;
      return [cx, cy];
    };

    // Subtle frame so empty padding still reads as a plot area.
    ctx.strokeStyle = "rgba(255,255,255,0.08)";
    ctx.lineWidth = 1;
    ctx.strokeRect(ox - 4, oy - 4, drawnW + 8, drawnH + 8);

    ctx.strokeStyle = "#5eead4";
    ctx.lineWidth = Math.max(2, Math.min(cssW, cssH) / 220);
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < xs.length; i++) {
      if (!Number.isFinite(xs[i]) || !Number.isFinite(ys[i])) continue;
      const [cx, cy] = toCanvas(xs[i], ys[i]);
      if (!started) {
        ctx.moveTo(cx, cy);
        started = true;
      } else ctx.lineTo(cx, cy);
    }
    ctx.stroke();

    const [sx, sy] = toCanvas(xs[0], ys[0]);
    const [ex, ey] = toCanvas(xs[xs.length - 1], ys[ys.length - 1]);
    const r = Math.max(4, Math.min(cssW, cssH) / 120);
    ctx.fillStyle = "#4ade80";
    ctx.beginPath();
    ctx.arc(sx, sy, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#ffb020";
    ctx.beginPath();
    ctx.arc(ex, ey, r, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = "#9aa3b2";
    ctx.font = "12px sans-serif";
    ctx.fillText(
      `${spanX.toFixed(0)}×${spanY.toFixed(0)} m · ${xs.length} pts`,
      12,
      cssH - 12
    );
  }, [series, trajSize]);

  const onSaveName = async () => {
    if (!selectedId) return;
    setBusy(true);
    setErr(null);
    try {
      await patchExperiment(selectedId, {
        display_name: rename.trim() ? rename.trim() : null,
      });
      await refresh();
    } catch (e) {
      setErr(String(e));
    } finally {
      setBusy(false);
    }
  };

  const checkedCount = checkedIds.size;
  const allChecked =
    experiments.length > 0 && experiments.every((e) => checkedIds.has(e.id));

  const toggleChecked = (id: string, next?: boolean) => {
    setCheckedIds((prev) => {
      const on = next ?? !prev.has(id);
      const copy = new Set(prev);
      if (on) copy.add(id);
      else copy.delete(id);
      return copy;
    });
  };

  const toggleRange = (toId: string) => {
    const ids = experiments.map((e) => e.id);
    const fromId = lastClickedIdRef.current;
    const toIdx = ids.indexOf(toId);
    if (fromId == null || toIdx < 0) {
      toggleChecked(toId, true);
      lastClickedIdRef.current = toId;
      return;
    }
    const fromIdx = ids.indexOf(fromId);
    if (fromIdx < 0) {
      toggleChecked(toId, true);
      lastClickedIdRef.current = toId;
      return;
    }
    const lo = Math.min(fromIdx, toIdx);
    const hi = Math.max(fromIdx, toIdx);
    setCheckedIds((prev) => {
      const copy = new Set(prev);
      for (let i = lo; i <= hi; i++) copy.add(ids[i]);
      return copy;
    });
    lastClickedIdRef.current = toId;
  };

  const selectAll = () => {
    setCheckedIds(new Set(experiments.map((e) => e.id)));
  };

  const clearChecked = () => setCheckedIds(new Set());

  const deleteIds = async (ids: string[], confirmLabel?: string) => {
    const unique = [...new Set(ids.filter(Boolean))];
    if (unique.length === 0) return;
    const msg =
      confirmLabel ??
      (unique.length === 1
        ? `Delete log "${
            (() => {
              const t = experiments.find((e) => e.id === unique[0]);
              return t ? experimentLabel(t) : unique[0].slice(0, 18);
            })()
          }"? This cannot be undone.`
        : `Delete ${unique.length} selected logs? This cannot be undone.`);
    if (!window.confirm(msg)) return;
    setBusy(true);
    setErr(null);
    setCtxMenu(null);
    if (unique.includes(selectedId)) {
      setSelectedId("");
      setSeries(null);
    }
    const failedIds = new Set<string>();
    const failedMsgs: string[] = [];
    for (const id of unique) {
      try {
        await deleteExperiment(id);
      } catch (e) {
        failedIds.add(id);
        failedMsgs.push(`${id.slice(0, 14)}…: ${String(e)}`);
      }
    }
    setCheckedIds(new Set(failedIds));
    await refresh();
    if (failedMsgs.length) {
      setErr(
        `Delete failed ${failedMsgs.length}/${unique.length}\n${failedMsgs.slice(0, 3).join("\n")}`
      );
    }
    setBusy(false);
  };

  const deleteById = async (doomed: string) => {
    await deleteIds([doomed]);
  };

  const onDelete = async () => {
    if (!selectedId) return;
    await deleteById(selectedId);
  };

  const onBatchDelete = async () => {
    await deleteIds([...checkedIds]);
  };

  const emptyLabel = loading ? "Loading…" : "No recorded experiments";

  return (
    <div className="logs-panel">
      {ctxMenu && (
        <>
          <div
            className="ctx-menu-backdrop"
            onClick={() => setCtxMenu(null)}
            onContextMenu={(e) => {
              e.preventDefault();
              setCtxMenu(null);
            }}
          />
          <div
            className="ctx-menu"
            style={{ left: ctxMenu.clientX, top: ctxMenu.clientY }}
            onContextMenu={(e) => e.preventDefault()}
          >
            <button
              type="button"
              disabled={busy}
              onClick={() => {
                setSelectedId(ctxMenu.id);
                setCtxMenu(null);
              }}
            >
              Open
            </button>
            <button
              type="button"
              disabled={busy}
              onClick={() => {
                toggleChecked(ctxMenu.id);
                setCtxMenu(null);
              }}
            >
              {checkedIds.has(ctxMenu.id) ? "Uncheck" : "Check"}
            </button>
            <button
              type="button"
              disabled={busy}
              onClick={() => void deleteById(ctxMenu.id)}
            >
              Delete this
            </button>
            {checkedCount > 0 && (
              <button
                type="button"
                disabled={busy}
                onClick={() => void onBatchDelete()}
              >
                Delete selected ({checkedCount})
              </button>
            )}
            <button
              type="button"
              className="ctx-menu-cancel"
              onClick={() => setCtxMenu(null)}
            >
              Cancel
            </button>
          </div>
        </>
      )}
      <div className="logs-list">
        <div className="logs-list-head">
          <h2>Logs</h2>
          <button type="button" disabled={busy || loading} onClick={() => void refresh()}>
            Refresh
          </button>
        </div>
        <div className="logs-batch-bar">
          <label className="logs-check-all" title="Select all / clear">
            <input
              type="checkbox"
              checked={allChecked}
              disabled={busy || experiments.length === 0}
              onChange={() => (allChecked ? clearChecked() : selectAll())}
            />
            <span>Select all</span>
          </label>
          <button
            type="button"
            disabled={busy || checkedCount === 0}
            onClick={() => void onBatchDelete()}
          >
            Delete selected{checkedCount > 0 ? ` (${checkedCount})` : ""}
          </button>
          {checkedCount > 0 && (
            <button type="button" disabled={busy} onClick={clearChecked}>
              Clear checks
            </button>
          )}
        </div>
        <p className="logs-batch-hint">Check items to batch-delete; Shift+click for range</p>
        {err && <div className="logs-error">{err}</div>}
        <ul>
          {experiments.length === 0 && <li className="logs-empty">{emptyLabel}</li>}
          {experiments.map((e) => {
            const reason = e.metrics?.finished_reason ?? e.status;
            const dur = Number(e.duration_s ?? 0);
            const checked = checkedIds.has(e.id);
            return (
              <li key={e.id} className={checked ? "checked" : undefined}>
                <label className="logs-row-check">
                  <input
                    type="checkbox"
                    checked={checked}
                    disabled={busy}
                    onClick={(ev) => {
                      ev.stopPropagation();
                      if (ev.shiftKey) {
                        ev.preventDefault();
                        toggleRange(e.id);
                      } else {
                        lastClickedIdRef.current = e.id;
                      }
                    }}
                    onChange={(ev) => {
                      if ((ev.nativeEvent as MouseEvent).shiftKey) return;
                      toggleChecked(e.id, ev.target.checked);
                      lastClickedIdRef.current = e.id;
                    }}
                  />
                </label>
                <button
                  type="button"
                  className={e.id === selectedId ? "active" : ""}
                  onClick={(ev) => {
                    if (ev.shiftKey) {
                      toggleRange(e.id);
                      return;
                    }
                    if (ev.ctrlKey || ev.metaKey) {
                      toggleChecked(e.id);
                      lastClickedIdRef.current = e.id;
                      return;
                    }
                    setSelectedId(e.id);
                    lastClickedIdRef.current = e.id;
                  }}
                  onContextMenu={(ev) => {
                    ev.preventDefault();
                    ev.stopPropagation();
                    setSelectedId(e.id);
                    setCtxMenu({
                      id: e.id,
                      clientX: ev.clientX,
                      clientY: ev.clientY,
                    });
                  }}
                >
                  <span className="logs-name">{experimentLabel(e)}</span>
                  <span className="logs-meta">
                    {dur.toFixed(1)}s
                    {reason ? ` · ${String(reason)}` : ""}
                  </span>
                </button>
              </li>
            );
          })}
        </ul>
      </div>
      <div className="logs-detail">
        {selected ? (
          <>
            <div className="logs-detail-head">
              <div>
                <h2>{experimentLabel(selected)}</h2>
                <p className="logs-id">{selected.id}</p>
              </div>
              <div className="logs-actions">
                <button
                  type="button"
                  className="primary"
                  disabled={busy}
                  onClick={() => onOpenReplay(selected.id)}
                >
                  Open in Replay
                </button>
                <button type="button" disabled={busy} onClick={() => void onDelete()}>
                  Delete
                </button>
              </div>
            </div>
            <div className="logs-rename">
              <input
                type="text"
                value={rename}
                maxLength={80}
                placeholder="Display name"
                disabled={busy}
                onChange={(e) => setRename(e.target.value)}
              />
              <button type="button" disabled={busy} onClick={() => void onSaveName()}>
                Save name
              </button>
            </div>
            <div className="logs-summary">
              <span>plant: {String(selected.metadata?.plant ?? "—")}</span>
              <span>controller: {String(selected.metadata?.controller ?? "—")}</span>
              <span>states: {selected.n_states}</span>
            </div>
            <div className="logs-traj-wrap" ref={trajWrapRef}>
              <canvas ref={canvasRef} className="logs-traj" aria-label="trajectory preview" />
            </div>
          </>
        ) : (
          <div className="logs-empty-detail">Select a log to preview trajectory</div>
        )}
      </div>
    </div>
  );
}
