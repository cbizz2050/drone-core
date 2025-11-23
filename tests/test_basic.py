"""
Basic tests to verify installation and core functionality.
"""

import pytest
import asyncio
from drone_core.hal.interface import DroneMode, DroneState
from drone_core.vision.detector import ObjectDetector, Detection
from drone_core.navigation.planner import PathPlanner, Waypoint


def test_imports():
    """Test that all core modules can be imported."""
    from drone_core import DroneInterface, VisionSystem, NavigationSystem
    from drone_core.missions import Mission, PerimeterPatrol, AreaSurveillance
    assert True


def test_path_planner_perimeter():
    """Test perimeter patrol path planning."""
    planner = PathPlanner({
        "default_altitude": 10.0,
        "default_speed": 5.0
    })

    bounds = [(0, 0), (10, 0), (10, 10), (0, 10)]
    waypoints = planner.plan_perimeter_patrol(bounds)

    # Should have len(bounds) + 1 waypoints (closing the loop)
    assert len(waypoints) == len(bounds) + 1

    # First and last waypoints should be at same position
    assert waypoints[0].position[:2] == waypoints[-1].position[:2]

    # All should be at default altitude
    for wp in waypoints:
        assert wp.position[2] == 10.0


def test_path_planner_grid():
    """Test grid search pattern."""
    planner = PathPlanner({"default_altitude": 15.0})

    bounds = [(0, 0), (40, 0), (40, 40), (0, 40)]
    waypoints = planner.plan_grid_search(bounds, spacing=10.0)

    # Should have waypoints
    assert len(waypoints) > 0

    # All should be at correct altitude
    for wp in waypoints:
        assert wp.position[2] == 15.0


def test_path_length_calculation():
    """Test path length calculation."""
    planner = PathPlanner({})

    waypoints = [
        Waypoint(position=(0, 0, 10)),
        Waypoint(position=(10, 0, 10)),
        Waypoint(position=(10, 10, 10)),
    ]

    length = planner.calculate_path_length(waypoints)

    # Should be 10 + sqrt(100) = 10 + 10 = 20
    assert abs(length - 20.0) < 0.1


def test_object_detector_creation():
    """Test object detector initialization."""
    detector = ObjectDetector({
        "model_type": "yolo",
        "confidence_threshold": 0.5
    })

    assert detector.confidence_threshold == 0.5
    assert len(detector.classes) == 80  # COCO dataset


def test_detection_filtering():
    """Test detection filtering."""
    detector = ObjectDetector({"model_type": "yolo"})

    # Create mock detections
    detections = [
        Detection(
            class_id=0,
            class_name="person",
            confidence=0.9,
            bbox=(10, 10, 50, 50),
            center=(30, 30),
            area=1600
        ),
        Detection(
            class_id=2,
            class_name="car",
            confidence=0.6,
            bbox=(100, 100, 200, 200),
            center=(150, 150),
            area=10000
        ),
        Detection(
            class_id=0,
            class_name="person",
            confidence=0.4,
            bbox=(300, 300, 320, 320),
            center=(310, 310),
            area=400
        ),
    ]

    # Filter by confidence
    filtered = detector.filter_detections(detections, min_confidence=0.5)
    assert len(filtered) == 2

    # Filter by class
    filtered = detector.filter_detections(detections, classes=["person"])
    assert len(filtered) == 2

    # Filter by area
    filtered = detector.filter_detections(detections, min_area=1000)
    assert len(filtered) == 2


@pytest.mark.asyncio
async def test_simulation_backend_creation():
    """Test simulation backend can be created."""
    try:
        from drone_core.hal.simulation import SimulationBackend

        drone = SimulationBackend({
            "use_gui": False,
            "start_position": [0, 0, 1]
        })

        assert drone is not None
        assert not drone.is_connected()

    except ImportError:
        pytest.skip("PyBullet not installed")


def test_mission_creation():
    """Test mission objects can be created."""
    from drone_core.missions import PerimeterPatrol, AreaSurveillance

    patrol = PerimeterPatrol(
        waypoints=[(0, 0), (10, 0), (10, 10), (0, 10)],
        altitude=10.0
    )

    assert patrol.altitude == 10.0
    assert len(patrol.waypoints) == 4

    surveillance = AreaSurveillance(
        zone=[(0, 0), (20, 0), (20, 20), (0, 20)],
        pattern="grid"
    )

    assert surveillance.pattern == "grid"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
