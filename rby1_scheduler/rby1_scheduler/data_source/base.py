"""Replaceable boundary between the scheduler and an external laboratory server."""
from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Any

class ExternalDataSource(ABC):
    @abstractmethod
    def read_snapshot(self) -> dict[str, Any]:
        raise NotImplementedError

    @abstractmethod
    def write_scheduler_state(self, state: dict[str, Any]) -> None:
        raise NotImplementedError

    @abstractmethod
    def append_events(self, events: list[dict[str, Any]]) -> None:
        raise NotImplementedError
