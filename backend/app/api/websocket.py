"""WebSocket snapshot streams for live sessions and offline replay."""

from __future__ import annotations

import asyncio
import json

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from backend.app.runtime.replay_manager import ReplayManager
from backend.app.runtime.session import SessionManager

router = APIRouter()


@router.websocket("/ws/sessions/{session_id}")
async def session_ws(websocket: WebSocket, session_id: str) -> None:
    await websocket.accept()
    mgr: SessionManager = websocket.app.state.sessions
    session = mgr.get(session_id)
    if session is None:
        await websocket.send_json({"type": "error", "message": "session not found"})
        await websocket.close()
        return

    loop = asyncio.get_running_loop()
    session.publisher.bind_loop(loop)
    q = session.publisher.subscribe()
    try:
        status = session.status()
        await websocket.send_json({"type": "session.status", **status})
        latest = session.publisher.latest()
        if latest is not None:
            await websocket.send_json(latest)
        while True:
            try:
                msg = await asyncio.wait_for(q.get(), timeout=1.0)
                await websocket.send_json(msg)
            except asyncio.TimeoutError:
                await websocket.send_json({"type": "session.ping", **session.status()})
    except WebSocketDisconnect:
        pass
    finally:
        session.publisher.unsubscribe(q)


@router.websocket("/ws/replay/{experiment_id}")
async def replay_ws(websocket: WebSocket, experiment_id: str) -> None:
    """Push replay snapshots; client may send play/pause/seek/rate JSON commands."""
    await websocket.accept()
    replays: ReplayManager = websocket.app.state.replays
    try:
        player = replays.get_or_create_player(experiment_id)
    except FileNotFoundError:
        await websocket.send_json({"type": "error", "message": "experiment not found"})
        await websocket.close()
        return

    await websocket.send_json({"type": "replay.status", **player.status()})
    await websocket.send_json(player.current_snapshot())

    async def _reader() -> None:
        while True:
            raw = await websocket.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            action = msg.get("action")
            if action == "play":
                player.play()
            elif action == "pause":
                player.pause()
            elif action == "seek" and "t_ns" in msg:
                player.seek(int(msg["t_ns"]))
            elif action == "rate" and "rate" in msg:
                player.set_rate(float(msg["rate"]))

    reader_task = asyncio.create_task(_reader())
    try:
        while True:
            # ~60 Hz; status is already embedded in snapshot.replay
            await asyncio.sleep(1.0 / 60.0)
            snap = player.current_snapshot()
            snap["type"] = "replay.snapshot"
            await websocket.send_json(snap)
    except (WebSocketDisconnect, RuntimeError):
        # Client gone / send after close — avoid ASGI traceback noise
        pass
    finally:
        reader_task.cancel()
        try:
            await reader_task
        except (asyncio.CancelledError, WebSocketDisconnect):
            pass
