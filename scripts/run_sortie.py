#!/usr/bin/env python3
"""
Run a test sortie mission in simulation.

This script demonstrates a complete autonomous sortie including:
- Takeoff
- Perimeter patrol
- Threat detection
- Landing
"""

import asyncio
import argparse
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from drone_core.hal.simulation import SimulationBackend
from drone_core.vision.system import VisionSystem
from drone_core.navigation.system import NavigationSystem
from drone_core.missions import PerimeterPatrol, AreaSurveillance


async def run_perimeter_patrol(use_gui=False):
    """Run a perimeter patrol sortie."""
    print("=" * 70)
    print("DRONE CORE - PERIMETER PATROL SORTIE")
    print("=" * 70)

    # Configure simulation
    sim_config = {
        "use_gui": use_gui,
        "timestep": 1.0 / 240.0,
        "start_position": [0, 0, 0.5]
    }

    # Configure vision system
    vision_config = {
        "detector": {
            "model_type": "yolo",
            "confidence_threshold": 0.5
        },
        "tracker": {
            "max_distance": 50,
            "max_disappeared": 30
        },
        "threat_classes": ["person"]
    }

    # Configure navigation
    nav_config = {
        "planner": {
            "default_altitude": 10.0,
            "default_speed": 5.0
        },
        "controller": {}
    }

    # Create systems
    print("\n[1/5] Initializing simulation...")
    drone = SimulationBackend(sim_config)

    print("[2/5] Connecting to drone...")
    if not await drone.connect():
        print("ERROR: Failed to connect to simulation")
        return False

    print("[3/5] Initializing vision and navigation systems...")
    vision_system = VisionSystem(vision_config)
    navigation_system = NavigationSystem(drone, nav_config)

    # Define patrol perimeter (50m x 50m square)
    patrol_bounds = [
        (0, 0),
        (50, 0),
        (50, 50),
        (0, 50)
    ]

    print("[4/5] Creating mission...")
    mission = PerimeterPatrol(
        waypoints=patrol_bounds,
        altitude=10.0,
        speed=5.0,
        detect_intruders=True,
        num_loops=2,
        name="Test Perimeter Patrol"
    )

    # Execute mission
    print("[5/5] Executing sortie...\n")
    print("-" * 70)

    try:
        result = await mission.execute(drone, vision_system, navigation_system)

        print("-" * 70)
        print("\nMISSION COMPLETE")
        print("=" * 70)
        print(f"Status: {result.status.value.upper()}")
        print(f"Duration: {result.duration:.1f} seconds")
        print(f"Distance Traveled: {result.distance_traveled:.1f} meters")
        print(f"Threats Detected: {result.threats_detected}")

        if result.errors:
            print("\nErrors:")
            for error in result.errors:
                print(f"  - {error}")

        return result.status.value == "completed"

    except KeyboardInterrupt:
        print("\n\nMission aborted by user")
        mission.abort()
        await drone.land()
        return False

    except Exception as e:
        print(f"\n\nMission failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        print("\nDisconnecting...")
        await drone.disconnect()


async def run_area_search(use_gui=False):
    """Run an area search sortie."""
    print("=" * 70)
    print("DRONE CORE - AREA SEARCH SORTIE")
    print("=" * 70)

    # Similar setup as patrol
    sim_config = {
        "use_gui": use_gui,
        "timestep": 1.0 / 240.0,
        "start_position": [0, 0, 0.5]
    }

    vision_config = {
        "detector": {"model_type": "yolo"},
        "tracker": {},
        "threat_classes": ["person"]
    }

    nav_config = {
        "planner": {"default_altitude": 15.0},
        "controller": {}
    }

    drone = SimulationBackend(sim_config)

    if not await drone.connect():
        print("ERROR: Failed to connect")
        return False

    vision_system = VisionSystem(vision_config)
    navigation_system = NavigationSystem(drone, nav_config)

    # Define search area (40m x 40m)
    search_zone = [
        (0, 0),
        (40, 0),
        (40, 40),
        (0, 40)
    ]

    mission = AreaSurveillance(
        zone=search_zone,
        altitude=15.0,
        speed=5.0,
        pattern="grid",
        spacing=10.0,
        threat_detection=True,
        name="Test Area Search"
    )

    print("\nExecuting area search...\n")

    try:
        result = await mission.execute(drone, vision_system, navigation_system)

        print("\nMISSION COMPLETE")
        print(f"Status: {result.status.value.upper()}")
        print(f"Duration: {result.duration:.1f} seconds")
        print(f"Threats Detected: {result.threats_detected}")

        return result.status.value == "completed"

    except KeyboardInterrupt:
        print("\nMission aborted")
        return False
    except Exception as e:
        print(f"\nMission failed: {e}")
        return False
    finally:
        await drone.disconnect()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Run a test sortie mission",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run perimeter patrol with GUI
  python scripts/run_sortie.py --mission perimeter_patrol --gui

  # Run area search without GUI
  python scripts/run_sortie.py --mission area_search

  # Run quick test
  python scripts/run_sortie.py --mission perimeter_patrol
        """
    )

    parser.add_argument(
        "--mission",
        choices=["perimeter_patrol", "area_search"],
        default="perimeter_patrol",
        help="Mission type to run"
    )

    parser.add_argument(
        "--gui",
        action="store_true",
        help="Show PyBullet GUI"
    )

    args = parser.parse_args()

    # Run selected mission
    if args.mission == "perimeter_patrol":
        success = asyncio.run(run_perimeter_patrol(use_gui=args.gui))
    else:
        success = asyncio.run(run_area_search(use_gui=args.gui))

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
