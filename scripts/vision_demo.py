#!/usr/bin/env python3
"""
Enhanced vision demo with object detection in simulation.

This script demonstrates:
- PyBullet simulation with objects
- YOLO object detection
- Threat identification and tracking
- Real-time visualization
"""

import asyncio
import argparse
import sys
from pathlib import Path
import numpy as np
import cv2

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    print("ERROR: PyBullet not installed. Run: pip install pybullet")
    sys.exit(1)

from drone_core.hal.simulation import SimulationBackend
from drone_core.vision.system import VisionSystem
from drone_core.navigation.system import NavigationSystem


class EnhancedSimulationDemo:
    """Enhanced simulation with objects for vision testing."""

    def __init__(self, use_gui=False, enable_vision=True):
        self.use_gui = use_gui
        self.enable_vision = enable_vision
        self.objects = []

    async def setup_scene(self, drone):
        """Add objects to the simulation scene."""
        print("Setting up simulation scene with objects...")

        # Add some test objects (using primitive shapes as placeholders)
        # In a real scenario, these would be proper URDF models

        # Create "person" objects (tall cylinders)
        for i, pos in enumerate([
            [10, 10, 0],
            [15, 20, 0],
            [-10, 15, 0],
        ]):
            # Create a tall cylinder to represent a person
            col_shape = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=0.3,
                height=1.8
            )
            vis_shape = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=0.3,
                length=1.8,
                rgbaColor=[0.8, 0.6, 0.4, 1]  # Skin-like color
            )

            obj_id = p.createMultiBody(
                baseMass=70,  # kg
                baseCollisionShapeIndex=col_shape,
                baseVisualShapeIndex=vis_shape,
                basePosition=[pos[0], pos[1], 0.9],  # Half height above ground
            )

            self.objects.append({
                'id': obj_id,
                'type': 'person',
                'position': pos
            })

        # Add some "vehicle" objects (boxes)
        for i, pos in enumerate([
            [20, 5, 0],
            [-15, -10, 0],
        ]):
            col_shape = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[2, 1, 0.8]
            )
            vis_shape = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[2, 1, 0.8],
                rgbaColor=[0.2, 0.2, 0.8, 1]  # Blue
            )

            obj_id = p.createMultiBody(
                baseMass=1500,
                baseCollisionShapeIndex=col_shape,
                baseVisualShapeIndex=vis_shape,
                basePosition=[pos[0], pos[1], 0.8],
            )

            self.objects.append({
                'id': obj_id,
                'type': 'car',
                'position': pos
            })

        print(f"Added {len(self.objects)} objects to scene")

    async def run_demo(self):
        """Run the vision demo."""
        print("=" * 70)
        print("DRONE CORE - VISION SYSTEM DEMO")
        print("=" * 70)

        # Configure simulation
        sim_config = {
            "use_gui": self.use_gui,
            "timestep": 1.0 / 240.0,
            "start_position": [0, 0, 5]  # Start at 5m altitude
        }

        # Configure vision system
        vision_config = {
            "detector": {
                "model_type": "yolo",
                "confidence_threshold": 0.3,
                "model_size": "n"  # nano for speed
            },
            "tracker": {
                "max_distance": 50,
                "max_disappeared": 30
            },
            "threat_classes": ["person"]
        }

        # Create systems
        print("\n[1/6] Initializing simulation...")
        drone = SimulationBackend(sim_config)

        print("[2/6] Connecting to drone...")
        if not await drone.connect():
            print("ERROR: Failed to connect to simulation")
            return False

        # Setup scene with objects
        print("[3/6] Adding objects to scene...")
        await self.setup_scene(drone)

        print("[4/6] Initializing vision system...")
        if self.enable_vision:
            try:
                vision_system = VisionSystem(vision_config)
                print("Vision system ready (YOLO loaded)")
            except Exception as e:
                print(f"Warning: Could not initialize vision: {e}")
                print("Running without vision detection")
                vision_system = None
        else:
            vision_system = None

        # Arm and prepare drone
        print("[5/6] Arming drone...")
        await drone.arm()

        print("[6/6] Starting vision demo...\n")
        print("-" * 70)

        # Demo flight pattern - slowly scan the area
        waypoints = [
            [0, 0, 10],
            [10, 10, 10],
            [20, 10, 10],
            [20, 20, 10],
            [10, 20, 10],
            [0, 20, 10],
            [-10, 10, 10],
            [0, 0, 10],
        ]

        total_threats = 0
        frames_processed = 0

        try:
            for i, waypoint in enumerate(waypoints):
                print(f"\nFlying to waypoint {i+1}/{len(waypoints)}: {waypoint}")

                # Fly to waypoint
                await drone.goto(waypoint[0], waypoint[1], waypoint[2])

                # Process several frames at this location
                for frame_idx in range(10):
                    # Get camera frame
                    camera_frame = await drone.get_camera_frame()

                    if camera_frame and vision_system:
                        # Process frame through vision pipeline
                        result = vision_system.process_frame(camera_frame.image)

                        frames_processed += 1
                        num_detections = len(result['detections'])
                        num_threats = len(result['threats'])
                        total_threats = max(total_threats, num_threats)

                        if num_detections > 0:
                            print(f"  Frame {frame_idx}: {num_detections} detections, "
                                  f"{num_threats} threats, FPS: {result['stats']['fps']:.1f}")

                        # Show annotated frame if GUI is enabled
                        if self.use_gui and result['annotated_frame'] is not None:
                            cv2.imshow("Drone Vision", result['annotated_frame'])
                            cv2.waitKey(1)

                    # Small delay between frames
                    await asyncio.sleep(0.1)

            print("\n" + "-" * 70)
            print("\nVISION DEMO COMPLETE")
            print("=" * 70)

            if vision_system:
                stats = vision_system.get_stats()
                print(f"Frames Processed: {frames_processed}")
                print(f"Total Detections: {stats['total_detections']}")
                print(f"Total Threats: {stats['total_threats']}")
                print(f"Average FPS: {stats['fps']:.1f}")
            else:
                print("Vision system was not enabled")

            # Land
            print("\nLanding drone...")
            await drone.land()

            return True

        except KeyboardInterrupt:
            print("\n\nDemo aborted by user")
            await drone.land()
            return False

        except Exception as e:
            print(f"\n\nDemo failed with error: {e}")
            import traceback
            traceback.print_exc()
            return False

        finally:
            print("\nDisconnecting...")
            if self.use_gui:
                cv2.destroyAllWindows()
            await drone.disconnect()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Run vision system demo with object detection",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with GUI and vision
  python scripts/vision_demo.py --gui --vision

  # Run headless (faster)
  python scripts/vision_demo.py --vision

  # Run without vision (just simulation)
  python scripts/vision_demo.py --gui
        """
    )

    parser.add_argument(
        "--gui",
        action="store_true",
        help="Show PyBullet GUI and OpenCV windows"
    )

    parser.add_argument(
        "--vision",
        action="store_true",
        help="Enable YOLO vision detection (requires ultralytics)"
    )

    args = parser.parse_args()

    demo = EnhancedSimulationDemo(
        use_gui=args.gui,
        enable_vision=args.vision
    )

    success = asyncio.run(demo.run_demo())
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
