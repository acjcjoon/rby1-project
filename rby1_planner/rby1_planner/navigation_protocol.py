"""Pure value objects for the planner-to-navigation topic interface."""

from dataclasses import dataclass
from enum import Enum


class NavigationCommandStatus(str, Enum):
    """Lifecycle reported by ``/rby1/navigation/state``."""

    PENDING = 'pending'
    ACCEPTED = 'accepted'
    RUNNING = 'running'
    SUCCEEDED = 'succeeded'
    REJECTED = 'rejected'
    FAILED = 'failed'
    CANCELED = 'canceled'

    @property
    def terminal(self) -> bool:
        return self in {
            self.SUCCEEDED,
            self.REJECTED,
            self.FAILED,
            self.CANCELED,
        }


@dataclass(frozen=True)
class NavigationCommandState:
    """Latest state received for one navigation command ID."""

    status: NavigationCommandStatus
    progress: float = 0.0
    message: str = ''


__all__ = ['NavigationCommandState', 'NavigationCommandStatus']
