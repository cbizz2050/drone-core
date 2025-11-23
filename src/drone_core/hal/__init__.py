"""
Hardware Abstraction Layer (HAL)

Provides unified interface for drone control that works identically
in simulation and on real hardware.
"""

from drone_core.hal.interface import DroneInterface
from drone_core.hal.simulation import SimulationBackend
from drone_core.hal.hardware import HardwareBackend

__all__ = ["DroneInterface", "SimulationBackend", "HardwareBackend"]
