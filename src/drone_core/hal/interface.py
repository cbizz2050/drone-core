"""
Drone Interface - Hardware Abstraction Layer

Defines the standard interface that all drone backends (simulation and hardware)
must implement. This ensures code portability between simulation and real hardware.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Tuple, Optional
import numpy as np


class DroneMode(Enum):
    """Drone flight modes."""
    MANUAL = "manual"
    STABILIZE = "stabilize"
    GUIDED = "guided"
    AUTO = "auto"
    LOITER = "loiter"
    RTL = "rtl"  # Return to Launch
    LAND = "land"


class DroneState(Enum):
    """Drone operational states."""
    DISARMED = "disarmed"
    ARMED = "armed"
    FLYING = "flying"
    LANDING = "landing"
    EMERGENCY = "emergency"


@dataclass
class DroneStatus:
    """Current drone status information."""
    position: Tuple[float, float, float]  # (x, y, z) or (lat, lon, alt)
    velocity: Tuple[float, float, float]  # (vx, vy, vz)
    attitude: Tuple[float, float, float]  # (roll, pitch, yaw) in radians
    battery_voltage: float
    battery_percent: float
    gps_fix: bool
    gps_satellites: int
    mode: DroneMode
    state: DroneState
    armed: bool


@dataclass
class CameraFrame:
    """Camera frame data."""
    image: np.ndarray  # OpenCV format (BGR)
    timestamp: float
    camera_id: int
    resolution: Tuple[int, int]  # (width, height)


class DroneInterface(ABC):
    """
    Abstract base class for drone control.

    All drone backends (simulation and hardware) must implement this interface.
    This ensures that mission code can run identically in simulation and on real hardware.
    """

    def __init__(self, config: dict):
        """
        Initialize the drone interface.

        Args:
            config: Configuration dictionary with backend-specific settings
        """
        self.config = config
        self._connected = False

    @abstractmethod
    async def connect(self) -> bool:
        """
        Connect to the drone (simulation or hardware).

        Returns:
            True if connection successful, False otherwise
        """
        pass

    @abstractmethod
    async def disconnect(self) -> None:
        """Disconnect from the drone."""
        pass

    @abstractmethod
    async def arm(self) -> bool:
        """
        Arm the drone motors.

        Returns:
            True if arming successful, False otherwise
        """
        pass

    @abstractmethod
    async def disarm(self) -> bool:
        """
        Disarm the drone motors.

        Returns:
            True if disarming successful, False otherwise
        """
        pass

    @abstractmethod
    async def takeoff(self, altitude: float) -> bool:
        """
        Takeoff to specified altitude.

        Args:
            altitude: Target altitude in meters

        Returns:
            True if takeoff command accepted, False otherwise
        """
        pass

    @abstractmethod
    async def land(self) -> bool:
        """
        Land the drone at current position.

        Returns:
            True if land command accepted, False otherwise
        """
        pass

    @abstractmethod
    async def return_to_launch(self) -> bool:
        """
        Return to launch position and land.

        Returns:
            True if RTL command accepted, False otherwise
        """
        pass

    @abstractmethod
    async def goto_position(self, x: float, y: float, z: float,
                           yaw: Optional[float] = None) -> bool:
        """
        Fly to a specific position.

        Args:
            x: X position (meters or latitude)
            y: Y position (meters or longitude)
            z: Z position (altitude in meters)
            yaw: Optional yaw angle in radians

        Returns:
            True if command accepted, False otherwise
        """
        pass

    @abstractmethod
    async def set_velocity(self, vx: float, vy: float, vz: float,
                          yaw_rate: Optional[float] = None) -> bool:
        """
        Set drone velocity.

        Args:
            vx: Velocity in X direction (m/s)
            vy: Velocity in Y direction (m/s)
            vz: Velocity in Z direction (m/s)
            yaw_rate: Optional yaw rate (rad/s)

        Returns:
            True if command accepted, False otherwise
        """
        pass

    @abstractmethod
    async def set_mode(self, mode: DroneMode) -> bool:
        """
        Set the drone flight mode.

        Args:
            mode: Desired flight mode

        Returns:
            True if mode change successful, False otherwise
        """
        pass

    @abstractmethod
    async def get_status(self) -> DroneStatus:
        """
        Get current drone status.

        Returns:
            DroneStatus object with current state information
        """
        pass

    @abstractmethod
    async def get_camera_frame(self, camera_id: int = 0) -> Optional[CameraFrame]:
        """
        Get the latest camera frame.

        Args:
            camera_id: Camera identifier (0 for primary)

        Returns:
            CameraFrame object or None if unavailable
        """
        pass

    @abstractmethod
    async def emergency_stop(self) -> None:
        """
        Emergency stop - immediately halt all motors.

        WARNING: This will cause the drone to fall. Use only in emergencies.
        """
        pass

    def is_connected(self) -> bool:
        """Check if drone is connected."""
        return self._connected

    async def wait_for_position(self, target: Tuple[float, float, float],
                                tolerance: float = 0.5,
                                timeout: float = 30.0) -> bool:
        """
        Wait for drone to reach target position.

        Args:
            target: Target (x, y, z) position
            tolerance: Position tolerance in meters
            timeout: Maximum wait time in seconds

        Returns:
            True if position reached, False if timeout
        """
        import asyncio
        start_time = asyncio.get_event_loop().time()

        while True:
            status = await self.get_status()
            distance = np.linalg.norm(
                np.array(status.position) - np.array(target)
            )

            if distance < tolerance:
                return True

            if asyncio.get_event_loop().time() - start_time > timeout:
                return False

            await asyncio.sleep(0.1)

    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.disconnect()
