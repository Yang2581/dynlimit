"""Experiment display_name validation."""

from __future__ import annotations

_MAX = 80


def normalize_display_name(raw: str | None) -> str | None:
    """Trim and validate; empty → None. Raises ValueError on invalid input."""
    if raw is None:
        return None
    name = raw.strip()
    if not name:
        return None
    if len(name) > _MAX:
        raise ValueError(f"display_name max length is {_MAX}")
    if any(c in name for c in "/\\") or any(ord(c) < 32 for c in name):
        raise ValueError("display_name contains invalid characters")
    return name
