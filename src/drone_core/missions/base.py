"""
Base Mission Classes

Defines the base classes for mission planning and execution.
"""

import asyncio
from abc import ABC, abstractmethod
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, Dict, List
import time


class MissionStatus(Enum):
    """Mission execution status."""
    IDLE = "idle"
    PREPARING = "preparing"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    ABORTED = "aborted"


@dataclass
class MissionResult:
    """Results from mission execution."""
    status: MissionStatus
    start_time: float
    end_time: float
    duration: float
    distance_traveled: float
    threats_detected: int
    data: Dict = field(default_factory=dict)
    errors: List[str] = field(default_factory=list)


class Mission(ABC):
    """
    Abstract base class for all missions.

    Subclasses must implement the execute() method.
    """

    def __init__(self, name: str, config: dict):
        """
        Initialize mission.

        Args:
            name: Mission name
            config: Mission configuration
        """
        self.name = name
        self.config = config
        self.status = MissionStatus.IDLE

        self._start_time = 0.0
        self._end_time = 0.0
        self._should_abort = False
        self._pause_event = asyncio.Event()
        self._pause_event.set()  # Not paused initially

    @abstractmethod
    async def execute(self, drone, vision_system, navigation_system) -> MissionResult:
        """
        Execute the mission.

        Args:
            drone: DroneInterface instance
            vision_system: VisionSystem instance
            navigation_system: NavigationSystem instance

        Returns:
            MissionResult with execution details
        """
        pass

    async def prepare(self, drone, vision_system, navigation_system) -> bool:
        """
        Prepare for mission execution (optional override).

        Args:
            drone: DroneInterface instance
            vision_system: VisionSystem instance
            navigation_system: NavigationSystem instance

        Returns:
            True if preparation successful
        """
        self.status = MissionStatus.PREPARING
        return True

    async def cleanup(self, drone, vision_system, navigation_system):
        """
        Cleanup after mission (optional override).

        Args:
            drone: DroneInterface instance
            vision_system: VisionSystem instance
            navigation_system: NavigationSystem instance
        """
        pass

    def abort(self):
        """Request mission abort."""
        self._should_abort = True
        self.status = MissionStatus.ABORTED

    def pause(self):
        """Pause mission execution."""
        self._pause_event.clear()
        self.status = MissionStatus.PAUSED

    def resume(self):
        """Resume mission execution."""
        self._pause_event.set()
        self.status = MissionStatus.RUNNING

    async def check_pause(self):
        """Check if mission is paused and wait if necessary."""
        await self._pause_event.wait()

    def should_abort(self) -> bool:
        """Check if mission should abort."""
        return self._should_abort

    def get_elapsed_time(self) -> float:
        """Get elapsed time since mission start."""
        if self._start_time == 0:
            return 0.0
        if self._end_time > 0:
            return self._end_time - self._start_time
        return time.time() - self._start_time

    def _create_result(self, status: MissionStatus,
                      distance: float = 0.0,
                      threats: int = 0,
                      data: Optional[Dict] = None,
                      errors: Optional[List[str]] = None) -> MissionResult:
        """Create a MissionResult object."""
        return MissionResult(
            status=status,
            start_time=self._start_time,
            end_time=self._end_time,
            duration=self.get_elapsed_time(),
            distance_traveled=distance,
            threats_detected=threats,
            data=data or {},
            errors=errors or []
        )
