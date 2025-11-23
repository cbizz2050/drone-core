"""
Command Line Interface

Provides CLI commands for drone operations.
"""

import asyncio
import argparse
import yaml
from pathlib import Path


def simulate():
    """Launch simulation environment."""
    print("Drone Core - Simulation Mode")
    print("=" * 50)

    # Import here to avoid circular dependencies
    from drone_core.hal.simulation import SimulationBackend
    from drone_core.vision.system import VisionSystem
    from drone_core.navigation.system import NavigationSystem

    async def run_simulation():
        # Load config
        config = {
            "use_gui": True,
            "timestep": 1.0 / 240.0,
            "start_position": [0, 0, 0.5]
        }

        # Create drone interface
        drone = SimulationBackend(config)

        print("Connecting to simulation...")
        if not await drone.connect():
            print("Failed to connect to simulation")
            return

        print("Simulation running. Press Ctrl+C to exit.")

        try:
            # Keep simulation running
            while drone.is_connected():
                await asyncio.sleep(1.0)
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            await drone.disconnect()

    asyncio.run(run_simulation())


def mission():
    """Execute a mission."""
    parser = argparse.ArgumentParser(description="Execute a drone mission")
    parser.add_argument("--mission", required=True, help="Mission file (YAML)")
    parser.add_argument("--simulate", action="store_true", help="Run in simulation")
    parser.add_argument("--gui", action="store_true", help="Show simulation GUI")

    args = parser.parse_args()

    # Import mission modules
    from drone_core.hal.simulation import SimulationBackend
    from drone_core.hal.hardware import HardwareBackend
    from drone_core.vision.system import VisionSystem
    from drone_core.navigation.system import NavigationSystem
    from drone_core.missions import PerimeterPatrol, AreaSurveillance

    async def run_mission():
        # Load mission config
        with open(args.mission, 'r') as f:
            mission_config = yaml.safe_load(f)

        # Create drone backend
        if args.simulate:
            drone = SimulationBackend({
                "use_gui": args.gui,
                "timestep": 1.0 / 240.0,
                "start_position": [0, 0, 0.5]
            })
        else:
            drone = HardwareBackend(mission_config.get("drone", {}))

        # Connect
        print("Connecting to drone...")
        if not await drone.connect():
            print("Failed to connect")
            return

        # Create systems
        vision_system = VisionSystem(mission_config.get("vision", {}))
        navigation_system = NavigationSystem(drone, mission_config.get("navigation", {}))

        # Create and execute mission
        mission_type = mission_config.get("type")

        if mission_type == "perimeter_patrol":
            mission_obj = PerimeterPatrol(**mission_config.get("params", {}))
        elif mission_type == "area_surveillance":
            mission_obj = AreaSurveillance(**mission_config.get("params", {}))
        else:
            print(f"Unknown mission type: {mission_type}")
            return

        print(f"Executing mission: {mission_obj.name}")
        result = await mission_obj.execute(drone, vision_system, navigation_system)

        print(f"\nMission completed with status: {result.status.value}")
        print(f"Duration: {result.duration:.1f}s")
        print(f"Threats detected: {result.threats_detected}")

        await drone.disconnect()

    asyncio.run(run_mission())


def control():
    """Manual control interface."""
    print("Drone Core - Manual Control")
    print("Not yet implemented")


if __name__ == "__main__":
    print("Use 'drone-sim', 'drone-mission', or 'drone-control' commands")
