"""
Path Planning Module

Provides path planning algorithms for autonomous navigation.
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np
from enum import Enum


class WaypointType(Enum):
    """Type of waypoint."""
    NORMAL = "normal"
    LOITER = "loiter"
    LAND = "land"
    TAKEOFF = "takeoff"


@dataclass
class Waypoint:
    """Represents a navigation waypoint."""
    position: Tuple[float, float, float]  # (x, y, z) or (lat, lon, alt)
    yaw: Optional[float] = None  # Heading angle in radians
    speed: float = 5.0  # Desired speed to waypoint (m/s)
    loiter_time: float = 0.0  # Time to loiter at waypoint (seconds)
    waypoint_type: WaypointType = WaypointType.NORMAL
    acceptance_radius: float = 1.0  # Distance to consider waypoint reached


class PathPlanner:
    """
    Path planner for autonomous navigation.

    Provides algorithms for:
    - Waypoint-based navigation
    - Grid/perimeter patterns
    - Search patterns
    - Dynamic replanning
    """

    def __init__(self, config: dict):
        """
        Initialize path planner.

        Args:
            config: Configuration dict with keys:
                - default_altitude: Default flight altitude (meters)
                - default_speed: Default cruise speed (m/s)
                - waypoint_radius: Default waypoint acceptance radius (m)
        """
        self.config = config
        self.default_altitude = config.get("default_altitude", 10.0)
        self.default_speed = config.get("default_speed", 5.0)
        self.waypoint_radius = config.get("waypoint_radius", 1.0)

    def plan_perimeter_patrol(self, bounds: List[Tuple[float, float]],
                             altitude: Optional[float] = None,
                             speed: Optional[float] = None) -> List[Waypoint]:
        """
        Plan a perimeter patrol path.

        Args:
            bounds: List of (x, y) coordinates defining the perimeter
            altitude: Flight altitude (uses default if None)
            speed: Flight speed (uses default if None)

        Returns:
            List of waypoints for perimeter patrol
        """
        alt = altitude or self.default_altitude
        spd = speed or self.default_speed

        waypoints = []

        # Create waypoints for each corner
        for i, (x, y) in enumerate(bounds):
            wp = Waypoint(
                position=(x, y, alt),
                speed=spd,
                waypoint_type=WaypointType.NORMAL,
                acceptance_radius=self.waypoint_radius
            )
            waypoints.append(wp)

        # Close the loop by returning to first point
        if len(bounds) > 0:
            x, y = bounds[0]
            wp = Waypoint(
                position=(x, y, alt),
                speed=spd,
                waypoint_type=WaypointType.NORMAL,
                acceptance_radius=self.waypoint_radius
            )
            waypoints.append(wp)

        return waypoints

    def plan_grid_search(self, bounds: List[Tuple[float, float]],
                        spacing: float = 10.0,
                        altitude: Optional[float] = None,
                        speed: Optional[float] = None,
                        orientation: float = 0.0) -> List[Waypoint]:
        """
        Plan a grid search pattern (lawn mower pattern).

        Args:
            bounds: List of (x, y) coordinates defining the search area
            spacing: Distance between parallel search lines (meters)
            altitude: Flight altitude
            speed: Flight speed
            orientation: Grid orientation angle (radians)

        Returns:
            List of waypoints for grid search
        """
        alt = altitude or self.default_altitude
        spd = speed or self.default_speed

        # Convert bounds to numpy array
        bounds_array = np.array(bounds)

        # Get bounding box
        min_x, min_y = bounds_array.min(axis=0)
        max_x, max_y = bounds_array.max(axis=0)

        waypoints = []

        # Generate grid lines
        y = min_y
        direction = 1  # 1 for left-to-right, -1 for right-to-left

        while y <= max_y:
            if direction == 1:
                x_start, x_end = min_x, max_x
            else:
                x_start, x_end = max_x, min_x

            # Add waypoint at start of line
            wp = Waypoint(
                position=(x_start, y, alt),
                speed=spd,
                waypoint_type=WaypointType.NORMAL,
                acceptance_radius=self.waypoint_radius
            )
            waypoints.append(wp)

            # Add waypoint at end of line
            wp = Waypoint(
                position=(x_end, y, alt),
                speed=spd,
                waypoint_type=WaypointType.NORMAL,
                acceptance_radius=self.waypoint_radius
            )
            waypoints.append(wp)

            # Move to next line
            y += spacing
            direction *= -1

        return waypoints

    def plan_spiral_search(self, center: Tuple[float, float],
                          max_radius: float,
                          altitude: Optional[float] = None,
                          speed: Optional[float] = None,
                          num_loops: int = 5) -> List[Waypoint]:
        """
        Plan a spiral search pattern expanding from center.

        Args:
            center: (x, y) center point
            max_radius: Maximum search radius
            altitude: Flight altitude
            speed: Flight speed
            num_loops: Number of spiral loops

        Returns:
            List of waypoints for spiral search
        """
        alt = altitude or self.default_altitude
        spd = speed or self.default_speed

        cx, cy = center
        waypoints = []

        # Number of waypoints per loop
        points_per_loop = 8

        for loop in range(num_loops):
            radius = (loop + 1) * max_radius / num_loops

            for i in range(points_per_loop):
                angle = 2 * np.pi * i / points_per_loop
                x = cx + radius * np.cos(angle)
                y = cy + radius * np.sin(angle)

                wp = Waypoint(
                    position=(x, y, alt),
                    yaw=angle + np.pi / 2,  # Point tangent to circle
                    speed=spd,
                    waypoint_type=WaypointType.NORMAL,
                    acceptance_radius=self.waypoint_radius
                )
                waypoints.append(wp)

        return waypoints

    def plan_orbit(self, center: Tuple[float, float],
                   radius: float,
                   altitude: Optional[float] = None,
                   speed: Optional[float] = None,
                   num_waypoints: int = 12,
                   clockwise: bool = True) -> List[Waypoint]:
        """
        Plan a circular orbit pattern.

        Args:
            center: (x, y) center point
            radius: Orbit radius
            altitude: Flight altitude
            speed: Flight speed
            num_waypoints: Number of waypoints in circle
            clockwise: Direction of orbit

        Returns:
            List of waypoints for orbit
        """
        alt = altitude or self.default_altitude
        spd = speed or self.default_speed

        cx, cy = center
        waypoints = []

        for i in range(num_waypoints + 1):  # +1 to close the loop
            if clockwise:
                angle = -2 * np.pi * i / num_waypoints
            else:
                angle = 2 * np.pi * i / num_waypoints

            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)

            # Calculate yaw to point toward center
            yaw = angle + np.pi

            wp = Waypoint(
                position=(x, y, alt),
                yaw=yaw,
                speed=spd,
                waypoint_type=WaypointType.LOITER if i == 0 else WaypointType.NORMAL,
                acceptance_radius=self.waypoint_radius
            )
            waypoints.append(wp)

        return waypoints

    def optimize_path(self, waypoints: List[Waypoint]) -> List[Waypoint]:
        """
        Optimize a path by removing redundant waypoints.

        Args:
            waypoints: Original waypoint list

        Returns:
            Optimized waypoint list
        """
        if len(waypoints) <= 2:
            return waypoints

        optimized = [waypoints[0]]

        for i in range(1, len(waypoints) - 1):
            prev_pos = np.array(waypoints[i - 1].position)
            curr_pos = np.array(waypoints[i].position)
            next_pos = np.array(waypoints[i + 1].position)

            # Check if waypoint is collinear (on straight line)
            v1 = curr_pos - prev_pos
            v2 = next_pos - curr_pos

            # Normalize
            v1_norm = v1 / (np.linalg.norm(v1) + 1e-6)
            v2_norm = v2 / (np.linalg.norm(v2) + 1e-6)

            # If vectors are parallel, skip this waypoint
            dot_product = np.dot(v1_norm, v2_norm)
            if dot_product < 0.999:  # Not parallel (some angle)
                optimized.append(waypoints[i])

        optimized.append(waypoints[-1])
        return optimized

    def interpolate_waypoints(self, waypoints: List[Waypoint],
                             max_segment_length: float = 10.0) -> List[Waypoint]:
        """
        Interpolate additional waypoints for smoother path.

        Args:
            waypoints: Original waypoints
            max_segment_length: Maximum distance between waypoints

        Returns:
            Interpolated waypoint list
        """
        if len(waypoints) <= 1:
            return waypoints

        interpolated = [waypoints[0]]

        for i in range(len(waypoints) - 1):
            curr = waypoints[i]
            next_wp = waypoints[i + 1]

            curr_pos = np.array(curr.position)
            next_pos = np.array(next_wp.position)

            distance = np.linalg.norm(next_pos - curr_pos)

            if distance > max_segment_length:
                # Add intermediate waypoints
                num_segments = int(np.ceil(distance / max_segment_length))

                for j in range(1, num_segments):
                    t = j / num_segments
                    interp_pos = curr_pos + t * (next_pos - curr_pos)

                    wp = Waypoint(
                        position=tuple(interp_pos),
                        speed=curr.speed,
                        waypoint_type=WaypointType.NORMAL,
                        acceptance_radius=self.waypoint_radius
                    )
                    interpolated.append(wp)

            interpolated.append(next_wp)

        return interpolated

    def calculate_path_length(self, waypoints: List[Waypoint]) -> float:
        """
        Calculate total path length.

        Args:
            waypoints: List of waypoints

        Returns:
            Total path length in meters
        """
        if len(waypoints) <= 1:
            return 0.0

        total_length = 0.0

        for i in range(len(waypoints) - 1):
            curr_pos = np.array(waypoints[i].position)
            next_pos = np.array(waypoints[i + 1].position)
            total_length += np.linalg.norm(next_pos - curr_pos)

        return total_length

    def estimate_flight_time(self, waypoints: List[Waypoint]) -> float:
        """
        Estimate flight time for waypoint list.

        Args:
            waypoints: List of waypoints

        Returns:
            Estimated time in seconds
        """
        total_time = 0.0

        for i in range(len(waypoints) - 1):
            curr = waypoints[i]
            next_wp = waypoints[i + 1]

            curr_pos = np.array(curr.position)
            next_pos = np.array(next_wp.position)

            distance = np.linalg.norm(next_pos - curr_pos)
            speed = curr.speed if curr.speed > 0 else self.default_speed

            segment_time = distance / speed
            total_time += segment_time + curr.loiter_time

        return total_time
