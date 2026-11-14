"""FastAPI application entrypoint."""

from __future__ import annotations

from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from backend.app.api.routes import router as api_router
from backend.app.api.websocket import router as ws_router
from backend.app.platform.paths import configure_paths
from backend.app.runtime.replay_manager import ReplayManager
from backend.app.runtime.session import SessionManager

ROOT = Path(__file__).resolve().parents[2]


@asynccontextmanager
async def lifespan(app: FastAPI):
    configure_paths()
    app.state.sessions = SessionManager()
    app.state.replays = ReplayManager()
    yield
    for sid in list(app.state.sessions.list_ids()):
        app.state.sessions.close(sid)
    app.state.replays.close_all()


app = FastAPI(title="DynLimit API", version="0.2.0", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:5173", "http://localhost:5173", "http://127.0.0.1:8000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
app.include_router(api_router, prefix="/api")
app.include_router(ws_router)

# Only serve built UI when explicitly enabled — avoids stale frontend/dist in dev.
import os

frontend_dist = ROOT / "frontend" / "dist"
if os.environ.get("SERVE_FRONTEND") == "1" and frontend_dist.is_dir():
    app.mount("/", StaticFiles(directory=str(frontend_dist), html=True), name="frontend")
else:

    @app.get("/")
    def root() -> dict[str, str]:
        """Dev mode: open the Vite UI, not this API port."""
        return {
            "service": "dynlimit-api",
            "status": "ok",
            "ui": "http://127.0.0.1:5173",
            "health": "/api/health",
            "docs": "/docs",
            "hint": "Use Vite (npm run dev). Stale frontend/dist is not auto-served.",
        }


def run() -> None:
    import uvicorn

    configure_paths()
    port = int(os.environ.get("DYNLIMIT_API_PORT", "8000"))
    uvicorn.run("backend.app.main:app", host="127.0.0.1", port=port, reload=False)


if __name__ == "__main__":
    run()
