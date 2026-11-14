"""Session state machine for simulation control."""

from __future__ import annotations

from enum import Enum


class SessionState(str, Enum):
    CREATED = "CREATED"
    READY = "READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    STOPPED = "STOPPED"
    FINISHED = "FINISHED"
    ERROR = "ERROR"


class IllegalTransitionError(RuntimeError):
    pass


_TRANSITIONS: dict[tuple[SessionState, str], SessionState] = {
    (SessionState.CREATED, "load"): SessionState.READY,
    (SessionState.READY, "start"): SessionState.RUNNING,
    (SessionState.RUNNING, "pause"): SessionState.PAUSED,
    (SessionState.PAUSED, "resume"): SessionState.RUNNING,
    (SessionState.PAUSED, "step_one"): SessionState.PAUSED,
    (SessionState.RUNNING, "stop"): SessionState.STOPPED,
    (SessionState.PAUSED, "stop"): SessionState.STOPPED,
    (SessionState.RUNNING, "finish"): SessionState.FINISHED,
    (SessionState.RUNNING, "error"): SessionState.ERROR,
    (SessionState.PAUSED, "error"): SessionState.ERROR,
    (SessionState.FINISHED, "reset"): SessionState.READY,
    (SessionState.STOPPED, "reset"): SessionState.READY,
    (SessionState.ERROR, "reset"): SessionState.READY,
}


class SessionStateMachine:
    def __init__(self) -> None:
        self.state = SessionState.CREATED

    def transition(self, event: str) -> SessionState:
        key = (self.state, event)
        if key not in _TRANSITIONS:
            raise IllegalTransitionError(
                f"Illegal transition: state={self.state.value} event={event}"
            )
        self.state = _TRANSITIONS[key]
        return self.state

    def can(self, event: str) -> bool:
        return (self.state, event) in _TRANSITIONS
