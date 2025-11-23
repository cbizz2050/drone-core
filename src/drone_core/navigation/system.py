"""
Navigation System

High-level navigation system integrating path planning and control.
"""

from typing import List, Tuple, Optional
from drone_core.navigation.planner import PathPlanner, Waypoint
from drone_core.navigation.controller import NavigationController
from drone_core.hal.interface import DroneInterface


class NavigationSystem:
    """
    Integrated navigation system.

    Provides high-level navigation commands that combine planning and execution.
    """

    def __init__(self, drone: DroneInterface, config: dict):
        """
        Initialize navigation system.

        Args:
            drone: DroneInterface instance
            config: Configuration dict
        """
        self.drone = drone
        self.config = config

        planner_config = config.get("planner", {})
        controller_config = config.get("controller", {})

        self.planner = PathPlanner(planner_config)
        self.controller = NavigationController(drone, controller_config)

    async def patrol_perimeter(self, bounds: List[Tuple[float, float]],
                              altitude: float = 10.0,
                              speed: float = 5.0,
                              num_loops: int = 1) -> bool:
        """
        Execute perimeter patrol mission.

        Args:
            bounds: List of (x, y) boundary points
            altitude: Flight altitude
            speed: Flight speed
            num_loops: Number of patrol loops

        Returns:
            True if mission completed successfully
        """
        waypoints = self.planner.plan_perimeter_patrol(bounds, altitude, speed)

        # Repeat for multiple loops
        if num_loops > 1:
            full_path = waypoints * num_loops
        else:
            full_path = waypoints

        return await self.controller.execute_path(full_path)

    async def search_area(self, bounds: List[Tuple[float, float]],
                         pattern: str = "grid",
                         altitude: float = 15.0,
                         speed: float = 5.0,
                         spacing: float = 10.0) -> bool:
        """
        Execute area search mission.

        Args:
            bounds: Search area boundaries
            pattern: Search pattern ('grid', 'spiral')
            altitude: Flight altitude
            speed: Flight speed
            spacing: Line spacing for grid pattern

        Returns:
            True if mission completed successfully
        """
        if pattern == "grid":
            waypoints = self.planner.plan_grid_search(bounds, spacing, altitude, speed)
        elif pattern == "spiral":
            # Calculate center and radius
            import numpy as np
            bounds_array = np.array(bounds)
            center = bounds_array.mean(axis=0)
            max_radius = np.max(np.linalg.norm(bounds_array - center, axis=1))
            waypoints = self.planner.plan_spiral_search(tuple(center), max_radius, altitude, speed)
        else:
            print(f"Unknown pattern: {pattern}")
            return False

        return await self.controller.execute_path(waypoints)

    async def orbit_point(self, center: Tuple[float, float],
                         radius: float,
                         altitude: float = 10.0,
                         speed: float = 5.0,
                         duration: Optional[float] = None) -> bool:
        """
        Orbit around a point.

        Args:
            center: Center point (x, y)
            radius: Orbit radius
            altitude: Flight altitude
            speed: Flight speed
            duration: Optional duration in seconds (continuous if None)

        Returns:
            True if completed successfully
        """
        waypoints = self.planner.plan_orbit(center, radius, altitude, speed)

        if duration:
            # Calculate how many loops needed
            orbit_time = self.planner.estimate_flight_time(waypoints)
            num_loops = int(duration / orbit_time) + 1
            full_path = waypoints * num_loops
        else:
            full_path = waypoints

        return await self.controller.execute_path(full_path)

    async def follow_path(self, waypoints: List[Waypoint]) -> bool:
        """
        Follow a custom waypoint path.

        Args:
            waypoints: List of waypoints to follow

        Returns:
            True if completed successfully
        """
        return await self.controller.execute_path(waypoints)

    def stop_navigation(self):
        """Stop current navigation."""
        self.controller.stop()

    def get_navigation_status(self) -> dict:
        """Get current navigation status."""
        return self.controller.get_progress()
