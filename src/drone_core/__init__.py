"""
Drone Core - Autonomous Security Drone Platform

A modular, simulation-first autonomous drone system for security applications.
"""

__version__ = "0.1.0"
__author__ = "Cbizz"

from drone_core.hal import DroneInterface
from drone_core.vision import VisionSystem
from drone_core.navigation import NavigationSystem
from drone_core.missions import Mission, PerimeterPatrol, AreaSurveillance

__all__ = [
    "DroneInterface",
    "VisionSystem",
    "NavigationSystem",
    "Mission",
    "PerimeterPatrol",
    "AreaSurveillance",
]
