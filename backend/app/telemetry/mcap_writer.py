"""Minimal synchronous MCAP recorder (JSON payloads)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from mcap.writer import Writer


class McapRecorder:
    def __init__(self, experiment_dir: str | Path) -> None:
        self.experiment_dir = Path(experiment_dir)
        self._writer: Writer | None = None
        self._fh = None
        self._schemas: dict[str, int] = {}
        self._channels: dict[str, int] = {}
        self._seq = 0
        self._closed = True

    def start(self, metadata: dict[str, Any]) -> None:
        self.experiment_dir.mkdir(parents=True, exist_ok=True)
        meta_path = self.experiment_dir / "metadata.json"
        meta_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        mcap_path = self.experiment_dir / "simulation.mcap"
        self._fh = open(mcap_path, "wb")
        self._writer = Writer(self._fh)
        self._writer.start(profile="", library="dynlimit/0.1.0")
        self._closed = False

    def enqueue(self, topic: str, t_ns: int, payload: dict[str, Any]) -> None:
        if self._writer is None or self._closed:
            raise RuntimeError("McapRecorder not started")
        channel_id = self._ensure_channel(topic)
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self._writer.add_message(
            channel_id=channel_id,
            log_time=t_ns,
            data=data,
            publish_time=t_ns,
            sequence=self._seq,
        )
        self._seq += 1

    def finalize(self, status: str) -> None:
        if self._writer is None and self._fh is None:
            return
        # Attach status into a side file for easy inspection
        try:
            status_path = self.experiment_dir / "status.txt"
            status_path.write_text(status, encoding="utf-8")
        except OSError:
            pass
        try:
            if self._writer is not None:
                self._writer.finish()
        finally:
            # Always release the Windows file lock even if finish() fails.
            if self._fh is not None:
                try:
                    self._fh.close()
                except OSError:
                    pass
            self._closed = True
            self._writer = None
            self._fh = None

    def _ensure_channel(self, topic: str) -> int:
        assert self._writer is not None
        if topic in self._channels:
            return self._channels[topic]
        schema_id = self._writer.register_schema(
            name=f"dynlimit.json.{topic.strip('/')}",
            encoding="jsonschema",
            data=b'{"type":"object"}',
        )
        channel_id = self._writer.register_channel(
            topic=topic,
            message_encoding="json",
            schema_id=schema_id,
        )
        self._schemas[topic] = schema_id
        self._channels[topic] = channel_id
        return channel_id


class NullRecorder:
    def start(self, metadata: dict[str, Any]) -> None:
        del metadata

    def enqueue(self, topic: str, t_ns: int, payload: dict[str, Any]) -> None:
        del topic, t_ns, payload

    def finalize(self, status: str) -> None:
        del status
