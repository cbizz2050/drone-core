"""
Navigation Controller

Executes waypoint navigation and handles low-level flight control.
"""

import asyncio
import numpy as np
from typing import List, Optional
from drone_core.navigation.planner import Waypoint
from drone_core.hal.interface import DroneInterface


class NavigationController:
    """
    Navigation controller for waypoint following.

    Handles low-level control to fly the drone along a planned path.
    """

    def __init__(self, drone: DroneInterface, config: dict):
        """
        Initialize navigation controller.

        Args:
            drone: DroneInterface instance
            config: Configuration dict
        """
        self.drone = drone
        self.config = config

        self.waypoint_reached_callback = config.get("waypoint_reached_callback", None)
        self.path_complete_callback = config.get("path_complete_callback", None)

        self.current_waypoint_index = 0
        self.waypoints: List[Waypoint] = []
        self.is_navigating = False

    async def execute_path(self, waypoints: List[Waypoint]) -> bool:
        """
        Execute a path defined by waypoints.

        Args:
            waypoints: List of waypoints to follow

        Returns:
            True if path completed successfully, False otherwise
        """
        if len(waypoints) == 0:
            return False

        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.is_navigating = True

        try:
            for i, waypoint in enumerate(waypoints):
                self.current_waypoint_index = i

                # Navigate to waypoint
                success = await self._goto_waypoint(waypoint)

                if not success:
                    print(f"Failed to reach waypoint {i}")
                    return False

                # Trigger callback
                if self.waypoint_reached_callback:
                    self.waypoint_reached_callback(i, waypoint)

            # Path complete
            if self.path_complete_callback:
                self.path_complete_callback()

            return True

        finally:
            self.is_navigating = False

    async def _goto_waypoint(self, waypoint: Waypoint) -> bool:
        """Navigate to a single waypoint."""
        # Send position command
        x, y, z = waypoint.position
        await self.drone.goto_position(x, y, z, waypoint.yaw)

        # Wait for waypoint to be reached
        success = await self.drone.wait_for_position(
            waypoint.position,
            tolerance=waypoint.acceptance_radius,
            timeout=60.0
        )

        if not success:
            return False

        # Loiter if required
        if waypoint.loiter_time > 0:
            await asyncio.sleep(waypoint.loiter_time)

        return True

    def stop(self):
        """Stop navigation."""
        self.is_navigating = False

    def get_progress(self) -> dict:
        """Get navigation progress information."""
        total_waypoints = len(self.waypoints)
        current = self.current_waypoint_index

        return {
            "current_waypoint": current,
            "total_waypoints": total_waypoints,
            "progress_percent": (current / total_waypoints * 100) if total_waypoints > 0 else 0,
            "is_navigating": self.is_navigating
        }
